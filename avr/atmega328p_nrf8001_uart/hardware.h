#ifndef HARDWARE_H__
#define HARDWARE_H__

#include <avr/io.h>


#define NRF8001_RDYN_PORT           PORTD
#define NRF8001_RDYN_DDR            DDRD
#define NRF8001_RDYN_PIN_INPUT_REG  PIND

#define NRF8001_RDYN_PIN            2   // PD2 - needs to be on INT0/INT1
#define NRF8001_RDYN_PIN_PULLUP()   NRF8001_RDYN_PORT |= (1<<NRF8001_RDYN_PIN)

#define DISABLE_RDYN_INT()          EIMSK &= ~(1<<INT0);
#define ENABLE_RDYN_INT()           EIMSK |= (1<<INT0);

#define NRF8001_REQN_PORT           PORTB
#define NRF8001_REQN_DDR            DDRB
#define NRF8001_REQN_PIN_INPUT_REG  PINB

#define NRF8001_REQN_PIN            2   // PB1
#define NRF8001_REQN_CONFIG_OUT     NRF8001_REQN_DDR |= (1<NRF8001_REQN_PIN)    // Set as output
#define SET_REQN_LOW()              NRF8001_REQN_PORT &= ~(1<<NRF8001_REQN_PIN)
#define SET_REQN_HIGH()             NRF8001_REQN_PORT |= (1<<NRF8001_REQN_PIN)

#define NRF8001_RSTN_PORT           PORTB
#define NRF8001_RSTN_DDR            DDRB
#define NRF8001_RSTN_PIN_INPUT_REG  PINB

#define NRF8001_RSTN_PIN            0   // PB0
#define NRF8001_RSTN_CONFIG_OUT     NRF8001_RSTN_DDR |= (1<NRF8001_RSTN_PIN)    // Set as output
#define SET_RSTN_LOW()              NRF8001_RSTN_PORT &= ~(1<<NRF8001_RSTN_PIN)
#define SET_RSTN_HIGH()             NRF8001_RSTN_PORT |= (1<<NRF8001_RSTN_PIN)

#define LED1_PORT                   PORTB
#define LED1_DDR                    DDRB
#define LED1_PIN                    PINB

#define LED1_PHYSICAL_PIN           6   // PB6
#define LED1_CONFIG_OUT             LED1_DDR |= (1<LED1_PHYSICAL_PIN)    // Set as output
#define SET_LED1_LOW()              LED1_PORT &= ~(1<<LED1_PHYSICAL_PIN)
#define SET_LED1_HIGH()             LED1_PORT |= (1<<LED1_PHYSICAL_PIN)
#define TOGGLE_LED1                 LED1_PORT ^= (1<<LED1_PHYSICAL_PIN)

#define LED2_PHYSICAL_PIN           7   // PB7
#define LED2_CONFIG_OUT             LED1_DDR |= (1<LED2_PHYSICAL_PIN)    // Set as output
#define SET_LED2_LOW()              LED1_PORT &= ~(1<<LED2_PHYSICAL_PIN)
#define SET_LED2_HIGH()             LED1_PORT |= (1<<LED2_PHYSICAL_PIN)

#endif

