#include "spi_dac.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#define SCK_PIN   2
#define MOSI_PIN  3
#define SYNC_PIN  4

#define AD5662_PD_NORMAL   0x00u
#define AD5662_PD_TRISTATE 0x03u

static void ad5662_transfer(uint16_t value, uint8_t pd_mode) {
    uint8_t buf[3] = {
        (uint8_t)(pd_mode & 0x03u),   /* bits[17:16] = PD, bits[23:18] = 0 */
        (uint8_t)(value >> 8),         /* D15-D8 */
        (uint8_t)(value & 0xFF)        /* D7-D0  */
    };
    gpio_put(SYNC_PIN, 0);
    spi_write_blocking(spi0, buf, 3);
    gpio_put(SYNC_PIN, 1);
}

void spi_dac_init(void) {
    /* SYNC as plain GPIO — init BEFORE spi_init */
    gpio_init(SYNC_PIN);
    gpio_set_dir(SYNC_PIN, GPIO_OUT);
    gpio_put(SYNC_PIN, 1);

    /* SPI Mode 3: CPOL=1, CPHA=1, 20MHz */
    spi_init(spi0, 20000000);
    spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(SCK_PIN,  GPIO_FUNC_SPI);
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);

    /* Start at 0V */
    spi_dac_write(0x0000);
}

void spi_dac_write(uint16_t value) {
    ad5662_transfer(value, AD5662_PD_NORMAL);
}

void spi_dac_powerdown(void) {
    ad5662_transfer(0x0000, AD5662_PD_TRISTATE);
}

void spi_dac_wakeup(void) {
    spi_dac_write(0x0000);
}