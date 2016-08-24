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
    DDRB |= (1<<W25Q_SS_PIN);   // SS pin is connected to PB1. Make it an output
    PORTB |= (1<<W25Q_SS_PIN);  // Drive it high
    while(1)
    {
        PORTB &= ~(1<<W25Q_SS_PIN);
        spi_transmit(0x90);
        spi_transmit(0x00);
        spi_transmit(0x00);
        spi_transmit(0x00);
        spi_transmit(0x00);
        spi_transmit(0x00);
        PORTB |= (1<<W25Q_SS_PIN);
        _delay_ms(1);

    }
    return 1;
}
