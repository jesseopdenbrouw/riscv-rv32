/*
 * i2c.h -- definitions for the I2C
 */

#ifndef _I2C_H
#define _I2C_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize SPI1 */
void i2c1_init(uint32_t value);
/* Send address to I2C1 */
uint32_t i2c1_transmit_address(uint8_t address);
/* Send address only to I2C1 */
uint32_t i2c1_transmit_address_only(uint8_t address);
/* Send byte to I2C1 */
uint32_t i2c1_transmit_byte(uint8_t data);
/* Transmit a buffer of uint8_t to I2C1 */
uint32_t i2c1_transmit(uint8_t address, uint8_t *buf, uint32_t len);
/* Receive a byte */
uint8_t i2c1_receive_byte(void);
/* Receive a buffer of uint8_t from I2C1 */
uint32_t i2c1_receive(uint8_t address, uint8_t *buf, uint32_t len);

/* Due to rounding toward 0, some speeds may be a bit to high */
#define I2C_PRESCALER_FM(A) (((A/3UL/400000UL)-1) << 16)
#define I2C_PRESCALER_SM(A) (((A/2UL/100000UL)-1) << 16)
#define I2C_FAST_MODE (1 << 2)
#define I2C_STANDARD_MODE (0 << 2)

#ifdef __cplusplus
}
#endif

#endif
