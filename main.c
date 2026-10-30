#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/watchdog.h"
#include "spi_dac.h"
#include "dds_tables.h"
#include "i2c_slave.h"
#include "hardware/adc.h"

/* RMS-DC converter sends output voltage to RP2040 PIN 3
*/
#define LEAD_SENSE_GPIO 29
#define LEAD_SENSE_ADC_CHANNEL 3

/* Calibrated from 4-point measurement (1k/2k/5k/10kHz):
 * actual loop rate = avg(measured * old_constant / target) = 323819        */
#define SAMPLE_RATE 313036UL

/* Ramp interval — step amplitude by 1 count every N samples.
 * 1 count = 2000uA / 65536 = 0.03052 uA
 * At SAMPLE_RATE=323819: interval=99 → 99.82 uA/sec ≤ 100 uA/sec limit.
 * Full scale ramp (0→65536) takes ~20 seconds.                              */
#define AMP_RAMP_INTERVAL 99U

/* Watchdog timeout in milliseconds.
 * The DDS loop runs at ~323kHz so the watchdog is pet every sample.
 * 500ms gives enormous headroom — if Core 0 stalls for half a second
 * something is seriously wrong and the DAC should be shut down.             */
#define WATCHDOG_TIMEOUT_MS 500U

/* Shared state — written by Core 1 (I2C), read by Core 0 (DDS)
 * Never write g_amplitude_target directly from the DDS loop —
 * use g_amplitude_target as the requested level; the loop ramps toward it.  */
volatile uint32_t    g_phase_inc        = 0;
volatile wave_type_t g_wave_type        = WAVE_SINE;
volatile uint32_t    g_amplitude_target = 0;      /* 0 = off, 65536 = full  */

static uint32_t calc_phase_inc(uint32_t freq_hz) {
    return (uint32_t)(freq_hz * 4294967296ULL / SAMPLE_RATE);
}

/* Core 1 — handles I2C slave forever */
void core1_entry(void) {
    // NOTE: need to keep ADC samplings eparate from waveform generation
    while (true) {
        uint16_t voltage_mv = lead_sense_read_mv();

        i2c_slave_set_lead_voltage(voltage_mv);

        sleep_ms(20);
    }
}

/* Lead detection methods - initialization and reading voltage from pin
*/
static void lead_sense_init(void) {
    adc_init();
    adc_gpio_init(LEAD_SENSE_GPIO);
    adc_select_input(LEAD_SENSE_ADC_CHANNEL);
}

static uint16_t lead_sense_read_mv(void) {
    adc_select_input(LEAD_SENSE_ADC_CHANNEL);
    uint16_t raw = adc_read();
    return (uint16_t)(((uint32_t)raw * 3300U) / 4095U);
}

int main(void) {
    stdio_init_all();

    /* ── Watchdog-reset safety shutdown ─────────────────────────────────────
     * If we're restarting after a watchdog timeout, the DDS loop stalled
     * while the DAC may have been outputting. Power it down immediately
     * before doing anything else — before even initialising SPI fully.
     * spi_dac_init() is called right after to bring it back to a clean state.*/
    if (watchdog_caused_reboot()) {
        spi_dac_init();
        lead_sense_init();
        spi_dac_powerdown();   /* tristate output — safe, high-impedance */
    }

    sleep_ms(10);

    spi_dac_init();
    app_i2c_slave_init();

    /* Default: 1kHz sine, output off (target=0) until ESP32 enables */
    g_wave_type        = WAVE_SINE;
    g_phase_inc        = calc_phase_inc(1000);
    g_amplitude_target = 0;

    multicore_launch_core1(core1_entry);

    /* Enable watchdog — must be pet at least once every WATCHDOG_TIMEOUT_MS.
     * pause_on_debug=true so the watchdog doesn't fire during a debug halt.  */
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);

    /* Core 0: free-running DDS loop with amplitude ramp.
     * s_amp_scale tracks the currently applied Q16 scale and steps toward
     * g_amplitude_target by 1 count every AMP_RAMP_INTERVAL samples,
     * enforcing the ≤ 100 uA/sec ramp limit.
     * Phase accumulator always advances — no hard output cutoff ever.
     * watchdog_update() is called every sample — at 323kHz this pets the
     * watchdog ~323000 times per second, well within the 500ms timeout.      */
    uint32_t phase_acc  = 0;
    uint32_t amp_scale  = 0;   /* currently applied scale */
    uint32_t ramp_count = 0;   /* interval counter */

    while (true) {
        /* ── Amplitude ramp step ────────────────────────────────────────── */
        uint32_t target = g_amplitude_target;
        if (amp_scale != target) {
            if (++ramp_count >= AMP_RAMP_INTERVAL) {
                ramp_count = 0;
                if (amp_scale < target) {
                    amp_scale++;
                } else {
                    amp_scale--;
                }
            }
        } else {
            ramp_count = 0;
        }

        /* ── DDS output — always running ────────────────────────────────── */
        phase_acc += g_phase_inc;
        uint32_t idx = phase_acc >> LUT_SHIFT;
        const uint16_t *lut = lut_table[(uint8_t)g_wave_type];
        uint16_t sample = (uint16_t)(((uint32_t)lut[idx] * amp_scale) >> 16);
        spi_dac_write(sample);

        /* ── Pet the watchdog ───────────────────────────────────────────── */
        watchdog_update();
    }
}