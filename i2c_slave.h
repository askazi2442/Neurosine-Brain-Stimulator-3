#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

#include <stdint.h>

/* Initialise I2C1 peripheral in slave mode and install IRQ handler.
 * Reads ADDR_PIN_0/1/2 to determine the 7-bit slave address (0x50-0x57).
 * Call once from Core 0 before i2c_slave_task(). */
void app_i2c_slave_init(void);

/* Blocking Core 0 loop — waits for I2C transactions via __wfi().
 * Never returns. */
void i2c_slave_task(void);

/* Returns the resolved 7-bit I2C slave address.
 * Valid after app_i2c_slave_init(). */
uint8_t i2c_slave_get_address(void);

/* Stores lead voltage to determine if leads are connected
*/
void i2c_slave_set_lead_voltage(uint16_t voltage_mv);

#endif /* I2C_SLAVE_H */