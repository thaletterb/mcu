/** A small SPI driver targeting atmega328p
 *  Should be easily extendable to other avrs
 */

#ifndef SPI_H__
#define SPI_H__

#include "avr/io.h"

#define SPI_CONTROL_PORT    PORTB
#define SPI_CONTROL_PIN     PINB
#define SPI_CONTROL_DDR     DDRB

#define SPI_SS_PIN          2   // PB2
#define SPI_MOSI_PIN        3   // PB3
#define SPI_MISO_PIN        4   // PB4
#define SPI_SCLK_PIN        5   // PB5

/** Begin Function Prototypes **/

/** @brief: initializes the SPI peripheral
 */
void spi_init(void);

/** @brief: uninitializes the SPI peripheral
 */
void spi_uninit(void);

/** @brief: spi transmit function
 *          sends and receives one byte simultaneously
 */

char spi_transmit(char tx_byte);


/** End Function Prototypes **/

#endif

