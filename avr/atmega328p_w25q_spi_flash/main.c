/** A small driver for interfacing atmega328p with
 *  w25q spi flash
 */

#include <avr/io.h>
#include <avr/delay.h>

#include "spi.h"

int main(void)
{
    spi_init();
    while(1)
    {
        spi_transmit(0xAA);
        _delay_ms(25);

    }
    return 1;
}
