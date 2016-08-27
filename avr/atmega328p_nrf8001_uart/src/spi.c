#include "spi.h"

/**@brief: initializes the SPI peripheral
 *         sets the MOSI, SCLK and SS Pins as outputs
 *         enables the SPI in master mode, f/16 clk
 */

void spi_init(void)
{
    DDRB = ((1<<SPI_MOSI_PIN)|(1<<SPI_SCLK_PIN)|(1<<SPI_SS_PIN)); //spi pins on port b MOSI SCK,SS outputs
    //SPCR = ((1<<SPE)|(1<<MSTR)|(1<<SPR0));  // SPI enable, Master, f/16, 
    SPCR = ((1<<SPE)|(1<<MSTR));  // SPI enable, Master, f/16, 
                                            // CPOL0 SCK is low when idle, CPHA0 sample leading edge
}

/** @brief: spi transmit function
 *          sends and receives a byte at the same time
 */

char spi_transmit(char tx_byte)
{
   SPDR = tx_byte;
   while(!(SPSR & (1<<SPIF)))
      ;
   return SPDR;

}
