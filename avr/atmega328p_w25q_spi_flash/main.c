/** A small driver for interfacing atmega328p with
 *  w25q spi flash
 */

#include <avr/io.h>
#include <avr/delay.h>

#include "spi.h"

#define W25Q_SS_PIN 1

int main(void)
{
    spi_init();

    // CS init
    w25q_ss_pin_init();
    w25q_ss_pin_drive(1);
    while(1)
    {
        w25q_ss_pin_drive(0);
        spi_transmit(0x90);
        spi_transmit(0x00);
        spi_transmit(0x00);
        spi_transmit(0x00);
        spi_transmit(0x00);
        spi_transmit(0x00);
        w25q_ss_pin_drive(1);
        _delay_ms(1);

    }
    return 1;
}
