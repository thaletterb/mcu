#ifndef HARDWARE_H__
#define HARDWARE_H__

#include <avr/io.h>


#define NRF8001_RDYN_PORT           PORTD
#define NRF8001_RDYN_DDR            DDRD
#define NRF8001_RDYN_PIN_INPUT_REG  PIND

#define NRF8001_RDYN_PIN            2   // PD2 - needs to be on INT0/INT1

#define NRF8001_REQN_PORT           PORTB
#define NRF8001_REQN_DDR            DDRB
#define NRF8001_REQN_PIN_INPUT_REG  PINB

#define NRF8001_REQN_PIN            1   // PB1
#define NRF8001_REQN_CONFIG_OUT     NRF8001_REQN_DDR |= (1<NRF8001_REQN_PIN)    // Set as output
#define SET_REQN_LOW()              NRF8001_REQN_PORT &= ~(1<<NRF8001_REQN_PIN)
#define SET_REQN_HIGH()             NRF8001_REQN_PORT |= (1<<NRF8001_REQN_PIN)

#endif

