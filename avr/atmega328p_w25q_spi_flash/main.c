/** A small driver for interfacing atmega328p with
 *  w25q spi flash
 */

#include <avr/io.h>
#include <avr/delay.h>

#include "spi.h"

#define W25Q_SS_PIN 1

#define LOGIC_LOW   0
#define LOGIC_HIGH  1

int main(void)
{
    spi_init();

    // CS init
    w25q_ss_pin_init();
    w25q_ss_pin_drive(LOGIC_HIGH);
    while(1)
    {
        uint8_t man_id = w25q_read_manufacturer_id();
        w25q_ss_pin_drive(LOGIC_LOW);
        spi_transmit(man_id);
        w25q_ss_pin_drive(LOGIC_HIGH);
        _delay_ms(1);
        uint8_t dev_id = w25q_read_device_id();
        w25q_ss_pin_drive(LOGIC_LOW);
        spi_transmit(dev_id);
        w25q_ss_pin_drive(LOGIC_HIGH);
        _delay_ms(5);

    }
    return 1;
}
