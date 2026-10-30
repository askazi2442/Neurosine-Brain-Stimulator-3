#include "i2c_slave.h"
#include "dds_tables.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/platform.h"
#include "pico/i2c_slave.h"

/* -------------------------------------------------------------------------
 * Pin and peripheral definitions
 * ------------------------------------------------------------------------- */
#define I2C_PORT        i2c1
#define I2C_SDA_PIN     6
#define I2C_SCL_PIN     7
/* FIX: Reduced from 400000 to 100000 to match the master and to give the
 * ISO1641B isolator sufficient margin.  The isolator's ~1 µs blanking time
 * consumed ~40 % of each 400 kHz bit window; at 100 kHz it is negligible. */
#define I2C_BAUDRATE    100000
#define I2C_BASE_ADDR   0x50

#define ADDR_PIN_0      26
#define ADDR_PIN_1      27
#define ADDR_PIN_2      28


// LEAD DETECTION
#define DDS_CMD_GET_LEAD_SENSE 0x07

/* -------------------------------------------------------------------------
 * Command definitions — must match ESP32 i2c_dds_master.h
 * ------------------------------------------------------------------------- */
#define DDS_CMD_SET_FREQ      0x01
#define DDS_CMD_SET_WAVEFORM  0x02
#define DDS_CMD_SET_AMPLITUDE 0x03
#define DDS_CMD_SET_ENABLE    0x04
#define DDS_CMD_SET_ALL       0x05
#define DDS_CMD_GET_STATUS    0x06
#define DDS_CMD_RESET         0xFF

/* -------------------------------------------------------------------------
 * DDS parameters — must match main.c
 * ------------------------------------------------------------------------- */
#define SAMPLE_RATE     313036UL  /* must match SAMPLE_RATE in main.c */
#define FREQ_MIN_HZ     500UL
#define FREQ_MAX_HZ     10000UL
#define DAC_IOUT_MAX_UA 2000U

/* Shared DDS state — defined in main.c */
static volatile uint16_t s_lead_voltage_mv = 0;
extern volatile uint32_t    g_phase_inc;
extern volatile wave_type_t g_wave_type;
extern volatile uint32_t    g_amplitude_target;  /* DDS loop ramps toward this */

static inline uint32_t calc_phase_inc(uint32_t freq_hz) {
    return (uint32_t)(freq_hz * 4294967296ULL / SAMPLE_RATE);
}

static inline uint32_t calc_amp_scale(uint16_t amp_uA) {
    if (amp_uA >= DAC_IOUT_MAX_UA) return 65536;
    return (uint32_t)(amp_uA * 65536ULL / DAC_IOUT_MAX_UA);
}

/* -------------------------------------------------------------------------
 * Internal I2C state
 * ------------------------------------------------------------------------- */
static uint8_t s_slave_addr = I2C_BASE_ADDR;

#define RX_BUF_SIZE 16
static uint8_t s_rx_buf[RX_BUF_SIZE];
static uint8_t s_rx_count = 0;
static uint8_t s_cmd      = 0xFF;

/* TX buffer for GET_STATUS response (9 bytes) */
static uint8_t s_tx_buf[9];
static uint8_t s_tx_idx = 0;

/* Shadow registers — preserve ESP32's mHz and µA values exactly */
static uint32_t s_shadow_freq_mHz = 1000000;  /* 1 kHz default */
static uint16_t s_shadow_amp_uA   = 2000;     /* 2 mA = full scale default */
static uint8_t  s_shadow_waveform = 0;        /* sine default */
static uint8_t  s_shadow_enabled  = 0;        /* off default */
static uint8_t  s_shadow_fault    = 0;

/* -------------------------------------------------------------------------
 * Apply completed write transaction to DDS state
 * ------------------------------------------------------------------------- */
static void apply_write(void) {
    if (s_rx_count < 1) return;

    switch (s_cmd) {

        case DDS_CMD_SET_FREQ: {
            if (s_rx_count < 5) break;
            s_shadow_freq_mHz = (uint32_t)s_rx_buf[1]
                              | ((uint32_t)s_rx_buf[2] << 8)
                              | ((uint32_t)s_rx_buf[3] << 16)
                              | ((uint32_t)s_rx_buf[4] << 24);
            uint32_t hz = s_shadow_freq_mHz / 1000;
            if (hz < FREQ_MIN_HZ) hz = FREQ_MIN_HZ;
            if (hz > FREQ_MAX_HZ) hz = FREQ_MAX_HZ;
            g_phase_inc = calc_phase_inc(hz);
            break;
        }

        case DDS_CMD_SET_WAVEFORM: {
            if (s_rx_count < 2) break;
            s_shadow_waveform = s_rx_buf[1];
            if (s_shadow_waveform < WAVE_TYPE_COUNT) {
                g_wave_type = (wave_type_t)s_shadow_waveform;
            }
            break;
        }

        case DDS_CMD_SET_AMPLITUDE: {
            if (s_rx_count < 3) break;
            s_shadow_amp_uA = (uint16_t)s_rx_buf[1]
                            | ((uint16_t)s_rx_buf[2] << 8);
            /* Only update target if currently enabled — ramp to new level */
            if (s_shadow_enabled) {
                g_amplitude_target = calc_amp_scale(s_shadow_amp_uA);
            }
            break;
        }

        case DDS_CMD_SET_ENABLE: {
            if (s_rx_count < 2) break;
            s_shadow_enabled   = s_rx_buf[1];
            g_amplitude_target = s_shadow_enabled
                               ? calc_amp_scale(s_shadow_amp_uA)
                               : 0U;
            break;
        }

        case DDS_CMD_SET_ALL: {
            if (s_rx_count < 9) break;
            s_shadow_freq_mHz = (uint32_t)s_rx_buf[1]
                              | ((uint32_t)s_rx_buf[2] << 8)
                              | ((uint32_t)s_rx_buf[3] << 16)
                              | ((uint32_t)s_rx_buf[4] << 24);
            s_shadow_amp_uA   = (uint16_t)s_rx_buf[5]
                              | ((uint16_t)s_rx_buf[6] << 8);
            s_shadow_waveform = s_rx_buf[7];
            s_shadow_enabled  = s_rx_buf[8];

            uint32_t hz = s_shadow_freq_mHz / 1000;
            if (hz < FREQ_MIN_HZ) hz = FREQ_MIN_HZ;
            if (hz > FREQ_MAX_HZ) hz = FREQ_MAX_HZ;
            g_phase_inc        = calc_phase_inc(hz);
            g_amplitude_target = s_shadow_enabled
                               ? calc_amp_scale(s_shadow_amp_uA)
                               : 0U;
            if (s_shadow_waveform < WAVE_TYPE_COUNT) {
                g_wave_type = (wave_type_t)s_shadow_waveform;
            }
            break;
        }

        case DDS_CMD_RESET: {
            s_shadow_freq_mHz  = 1000000;
            s_shadow_amp_uA    = 2000;
            s_shadow_waveform  = 0;
            s_shadow_enabled   = 0;
            s_shadow_fault     = 0;
            g_phase_inc        = calc_phase_inc(1000);
            g_amplitude_target = 0;   /* ramp down to silence */
            g_wave_type        = WAVE_SINE;
            break;
        }

        default:
            break;
    }
}

/* -------------------------------------------------------------------------
 * Pack 9-byte status response
 * ------------------------------------------------------------------------- */
static void prepare_status_packet(void) {
    s_tx_buf[0] = (uint8_t)(s_shadow_freq_mHz);
    s_tx_buf[1] = (uint8_t)(s_shadow_freq_mHz >> 8);
    s_tx_buf[2] = (uint8_t)(s_shadow_freq_mHz >> 16);
    s_tx_buf[3] = (uint8_t)(s_shadow_freq_mHz >> 24);
    s_tx_buf[4] = (uint8_t)(s_shadow_amp_uA);
    s_tx_buf[5] = (uint8_t)(s_shadow_amp_uA >> 8);
    s_tx_buf[6] = s_shadow_waveform;
    s_tx_buf[7] = s_shadow_enabled;
    s_tx_buf[8] = s_shadow_fault;
}

void i2c_slave_set_lead_voltage(uint16_t voltage_mv)
{
    s_lead_voltage_mv = voltage_mv;
}
// 3-byte response buffer
static uint8_t s_lead_tx_buf[3];
static uint8_t s_lead_tx_idx = 0;

static void prepare_lead_packet(void)
{
    uint16_t voltage = s_lead_voltage_mv;

    s_lead_tx_buf[0] = (uint8_t)(voltage & 0xFF);
    s_lead_tx_buf[1] = (uint8_t)((voltage >> 8) & 0xFF);

    // 0 = UNKNOWN until connection detection is validated
    s_lead_tx_buf[2] = 0;

    s_lead_tx_idx = 0;
}

/* -------------------------------------------------------------------------
 * I2C slave event handler (called from IRQ)
 * ------------------------------------------------------------------------- */
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {

        case I2C_SLAVE_RECEIVE: {
            uint8_t byte = i2c_read_byte_raw(i2c);
            if (s_rx_count == 0) {
                s_cmd    = byte;
                s_tx_idx = 0;
                if (s_cmd == DDS_CMD_GET_STATUS) {
                    prepare_status_packet();
                }
                else if (s_cmd == DDS_CMD_GET_LEAD_SENSE) {
                    prepare_lead_packet();
                }
            }
            if (s_rx_count < RX_BUF_SIZE) {
                s_rx_buf[s_rx_count] = byte;
            }
            s_rx_count++;
            break;
        }

        case I2C_SLAVE_REQUEST: {
            if (s_cmd == DDS_CMD_GET_STATUS && s_tx_idx < 9) {
                i2c_write_byte_raw(i2c, s_tx_buf[s_tx_idx++]);
            } 
            else if (s_cmd == DDS_CMD_GET_LEAD_SENSE && s_lead_tx_idx < 3) {
                i2c_write_byte_raw(i2c, s_lead_tx_buf[s_lead_tx_idx++]);
            }
            else {
                i2c_write_byte_raw(i2c, 0xFF);
            }
            break;
        }

        case I2C_SLAVE_FINISH: {
            if (s_cmd != DDS_CMD_GET_STATUS && s_cmd != DDS_CMD_GET_LEAD_SENSE) {
                apply_write();
            }
            s_rx_count = 0;
            s_tx_idx   = 0;
            s_lead_tx_idx = 0;
            break;
        }
    }
}

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */
uint8_t i2c_slave_get_address(void) {
    return s_slave_addr;
}

void app_i2c_slave_init(void) {
    /* Read hardware address pins */
    gpio_init(ADDR_PIN_0); gpio_set_dir(ADDR_PIN_0, GPIO_IN); gpio_pull_down(ADDR_PIN_0);
    gpio_init(ADDR_PIN_1); gpio_set_dir(ADDR_PIN_1, GPIO_IN); gpio_pull_down(ADDR_PIN_1);
    gpio_init(ADDR_PIN_2); gpio_set_dir(ADDR_PIN_2, GPIO_IN); gpio_pull_down(ADDR_PIN_2);
    sleep_us(10);

    uint8_t addr_bits = (uint8_t)((gpio_get(ADDR_PIN_2) << 2)
                                | (gpio_get(ADDR_PIN_1) << 1)
                                |  gpio_get(ADDR_PIN_0));
    s_slave_addr = I2C_BASE_ADDR | addr_bits;

    /* Configure I2C1 */
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    /* FIX: Internal pull-ups disabled — external 2.2 kΩ resistors are fitted
     * on both sides of the ISO1641B isolator.  The RP2040's ~50 kΩ internal
     * pull-ups in parallel with the externals lower the effective impedance
     * on Side 2 and prevent clean low-level transitions from the isolator's
     * open-drain output.  The ESP32-S3 side is likewise configured with
     * flags.enable_internal_pullup = false in i2c_dds_master.c.            */
    gpio_disable_pulls(I2C_SDA_PIN);
    gpio_disable_pulls(I2C_SCL_PIN);
    i2c_init(I2C_PORT, I2C_BAUDRATE);

    /* Install slave handler */
    i2c_slave_init(I2C_PORT, s_slave_addr, &i2c_slave_handler);
}

void i2c_slave_task(void) {
    while (true) {
        tight_loop_contents();
    }
}