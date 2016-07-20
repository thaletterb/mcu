#ifndef I2C_H__
#define I2C_H__

#include "stm32f10x.h"          // STM32VL headers
#include <stm32f10x_i2c.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>

/** Macros **/
#define I2C_TIMEOUT                 100000

/** Function Initializations **/
// Function to init I2C1 peripheral
void init_i2c1_peripheral(I2C_TypeDef *I2Cx);

// Sends one byte (slave_data) over *I2Cx to slave_addres. Possible SSD1306 specific
int i2c_send_command(I2C_TypeDef *I2Cx, uint8_t slave_address, uint8_t slave_data);

// Sends a byte
int i2c_send_command_new(I2C_TypeDef *I2Cx, uint8_t slave_address, uint8_t slave_data, uint8_t command_byte);

// Reads one byte from slave address's reg_addr
int i2c_read_reg_new(I2C_TypeDef *I2Cx, uint8_t slave_address, uint8_t reg_addr, uint8_t command_byte);

#endif

