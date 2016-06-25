#ifndef MY_SSD1306_H__
#define MY_SSD1306_H__

#include "my_i2c.h" 
#include "my_font.h"

/** Macros **/
#define SSD1306_I2C_SLAVE_ADDRESS8 0x78                      // 8 bit slave address (write)

extern uint8_t global_display_buffer[(128*64)/8];

// Inits the SSD1306 OLED 
void ssd1306_i2c_init(I2C_TypeDef *I2Cx, uint8_t ssd1306_slave_address);

// Sends the full buffer over I2C
int ssd1306_i2c_draw_buffer(I2C_TypeDef *I2Cx, uint8_t slave_address, uint8_t *buffer_pointer);

// Clears the display buffer
void ssd1306_clear_display_buffer(uint8_t *buffer_pointer);

// Draws a char to the display buffer
void ssd1306_draw_char_to_buffer(uint8_t x, uint8_t page_num, uint8_t which_char, uint8_t *buffer_pointer);

// Draws a string to display buffer
void ssd1306_draw_string_to_buffer(uint8_t x, uint8_t page_num, uint8_t *string, uint8_t *buffer_pointer);

#endif
