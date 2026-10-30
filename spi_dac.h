#ifndef SPI_DAC_H
#define SPI_DAC_H

#include <stdint.h>

/*
 * AD5662 16-bit DAC — SPI interface
 *
 * SPI Mode 3 (CPOL=1, CPHA=1): clock idles HIGH, data sampled on FALLING edge.
 * 24-bit word format (MSB first):
 *   bits [23:18] — don't care
 *   bits [17:16] — PD1, PD0 (00 = normal, 01 = 1kΩ, 10 = 100kΩ, 11 = tristate)
 *   bits [15:0]  — DAC code D15–D0
 *
 * Pins (Seeed XIAO RP2040):
 *   SCK  — GPIO2 (D8)
 *   MOSI — GPIO3 (D10)
 *   SYNC — GPIO4 (D9, active-low, driven as plain GPIO)
 */

/* Initialise SPI0 and SYNC GPIO — call once before multicore_launch_core1() */
void spi_dac_init(void);

/* Write a 16-bit code to the DAC in normal operation mode */
void spi_dac_write(uint16_t value);

/* Power down the DAC output (tristate / Hi-Z) */
void spi_dac_powerdown(void);

/* Wake the DAC back to normal operation at 0V */
void spi_dac_wakeup(void);

#endif /* SPI_DAC_H */