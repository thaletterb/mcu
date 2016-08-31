/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT. 
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $LastChangedRevision: 4808 $
 */ 

/** @file
@brief Implementation of the ACI transport layer module
*/

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include "hardware.h"
#include "hal_aci_tl.h"
#include "ble_system.h"
#include "spi.h"
#include <string.h>


typedef struct {
 hal_aci_data_t           aci_data[ACI_QUEUE_SIZE];
 uint8_t                  head;
 uint8_t                  tail;
}  __attribute__ ((__packed__)) aci_queue_t;

static hal_aci_data_t received_data;
static uint8_t        spi_readwrite(uint8_t aci_byte); 
//static bool           m_print_aci_data(hal_aci_data_t *p_data);
static aci_queue_t    aci_tx_q;
static aci_queue_t    aci_rx_q;

uint8_t rdynFlag = 0;
uint8_t spiByteRxFlag = 0;

uint8_t rxByte = 0;
static void m_aci_q_init(aci_queue_t *aci_q)
{
  aci_q->head = 0;
  aci_q->tail = 0;
}

static bool m_aci_q_enqueue(aci_queue_t *aci_q, hal_aci_data_t *p_data)
{
  const uint8_t next = (aci_q->tail + 1) % ACI_QUEUE_SIZE;
  const uint8_t length = p_data->buffer[0];
  
  if (next == aci_q->head)
  {
    /* full queue */
    return false;
  }
  aci_q->aci_data[aci_q->tail].status_byte = 0;
  // @comment : It would be better to copy the entire hal_aci_data_t into the buffer.
  memcpy((uint8_t *)&(aci_q->aci_data[aci_q->tail].buffer[0]), (uint8_t *)&p_data->buffer[0], length + 1);
  aci_q->tail = next;
  
  return true;
}

//@comment have a unit test for the queue, esp. the full and the empty states
static bool m_aci_q_dequeue(aci_queue_t *aci_q, hal_aci_data_t *p_data)
{
  if (aci_q->head == aci_q->tail)
  {
    /* empty queue */
    return false;
  }
  
  memcpy((uint8_t *)p_data, (uint8_t *)&(aci_q->aci_data[aci_q->head]), sizeof(hal_aci_data_t));
  aci_q->head = (aci_q->head + 1) % ACI_QUEUE_SIZE;
  
  return true;
}

static bool m_aci_q_is_empty(aci_queue_t *aci_q)
{
  return (aci_q->head == aci_q->tail);
}

static bool m_aci_q_is_full(aci_queue_t *aci_q)
{
  uint8_t next;
  bool state;
  
  //This should be done in a critical section
  //noInterrupts();
  //_disable_interrupts();
  cli();
  next = (aci_q->tail + 1) % ACI_QUEUE_SIZE;  
  
  if (next == aci_q->head)
  {
    state = true;
  }
  else
  {
    state = false;
  }
  
  //interrupts();
  //_enable_interrupts();
  sei();
  //end
  
  return state;
}

#if 0
static bool m_print_aci_data(hal_aci_data_t *p_data)
{
	// Disabled due to no print ability

  const uint8_t length = p_data->buffer[0];
  uint8_t i;
  Serial.print(length, DEC);
  Serial.print(" B:");
  for (i=0; i<=length; i++)
  {
    Serial.print(p_data->buffer[i], HEX);
    Serial.print(F(", "));
  }
  Serial.println(F(""));

}
#endif

void m_rdy_line_handle(void)
{
  hal_aci_data_t *p_aci_data;
  
  //sleep_disable();
  //detachInterrupt(1);
  
  // Receive or transmit data
  p_aci_data = hal_aci_tl_poll_get();
  
  // Check if we received data
  if (p_aci_data->buffer[0] > 0)
  {
    if (!m_aci_q_enqueue(&aci_rx_q, p_aci_data))
    {
      /* Receive Buffer full.
         Should never happen.
         Will spin in an infinite loop when this happens
         */
       while (1);
    }
    if (m_aci_q_is_full(&aci_rx_q))
    {
      /* Disable RDY line interrupt.
         Will latch any pending RDY lines, so when enabled it again this
         routine should be taken again */
      //EIMSK &= ~(0x1);
      DISABLE_RDYN_INT();
    }    
  }
}

bool hal_aci_tl_event_get(hal_aci_data_t *p_aci_data)
{
  bool was_full = m_aci_q_is_full(&aci_rx_q);
  
  if (m_aci_q_dequeue(&aci_rx_q, p_aci_data))
  {
    if (was_full)
    {
      /* Enable RDY line interrupt again */
    	ENABLE_RDYN_INT();
    	//EIMSK |= (0x1);
    }
    return true;
  }
  else
  {
    return false;
  }
}

static void configureNRF8001Interface(void)
{
    spi_init();             // Init the SPI interface

    NRF8001_RDYN_DDR &= ~(1<<NRF8001_RDYN_PIN);     // PD2 = input 
    PORTD |= (1<<NRF8001_RDYN_PIN);
    NRF8001_RDYN_PIN_PULLUP();
    
    NRF8001_REQN_CONFIG_OUT;
    SET_REQN_HIGH();

    NRF8001_RSTN_CONFIG_OUT;
    SET_RSTN_HIGH();

    DDRB |= (1<<LED1_PHYSICAL_PIN); // LED As output
    EIMSK |= (1<<INT0);     // Enable INT0 Interrupt
    EICRA |= (1<<ISC01);    // On falling edge of PD2/INT0
}


void hal_aci_tl_init()
{
	received_data.buffer[0] = 0;

	configureNRF8001Interface();

	/* initialize aci cmd queue */
	m_aci_q_init(&aci_tx_q);
	m_aci_q_init(&aci_rx_q);
}

bool hal_aci_tl_send(hal_aci_data_t *p_aci_cmd)
{
  const uint8_t length = p_aci_cmd->buffer[0];
  bool ret_val = false;

  if (p_aci_cmd->buffer[0] > HAL_ACI_MAX_LENGTH)
  {
  	//When the length of the ACI Message is greater than the maximum
  	//length of the ACI buffers, we will spin in a while loop
  	while(1);
  }
  
  if (length > HAL_ACI_MAX_LENGTH)
  {
    return false;
  }
  else
  {
    if (m_aci_q_enqueue(&aci_tx_q, p_aci_cmd))
    {
      ret_val = true;
    }
  }
  
  SET_REQN_LOW();

  return ret_val;
}

#if 0 //Get the ACI event by polling the ready line - Not used in the BLE-minimal
void hal_aci_tl_poll_rdy_line(void)
{
  uint8_t byte_cnt;
  uint8_t byte_sent_cnt;
  uint8_t max_bytes;
  hal_aci_data_t data_to_send;
  bool is_transmit_finished = false;
  
  if (1 == digitalRead(HAL_IO_RADIO_RDY))
  {
    return;
  }
    
  hal_pltf_enable_spi();  
  

  digitalWrite(HAL_IO_RADIO_REQN, 0);
  
  // Receive from queue
  if (m_aci_q_dequeue(&aci_tx_q, &data_to_send) == false)
  {
    /* queue was empty, nothing to send */
    data_to_send.status_byte = 0;
    data_to_send.buffer[0] = 0;
  }
  
  // Send length, receive header
  byte_sent_cnt = 0;
  received_data.status_byte = spi_readwrite(data_to_send.buffer[byte_sent_cnt++]);
  // Send first byte, receive length from slave
  received_data.buffer[0] = spi_readwrite(data_to_send.buffer[byte_sent_cnt++]);
  if (0 == data_to_send.buffer[0])
  {
    max_bytes = received_data.buffer[0];
  }
  else
  {
    // Set the maximum to the biggest size. One command byte is already sent
    max_bytes = (received_data.buffer[0] > (data_to_send.buffer[0] - 1)) 
      ? received_data.buffer[0] : (data_to_send.buffer[0] - 1);
  }

  if (max_bytes > HAL_ACI_MAX_LENGTH)
  {
    max_bytes = HAL_ACI_MAX_LENGTH;
  }

  // Transmit/receive the rest of the packet 
  for (byte_cnt = 0; byte_cnt < max_bytes; byte_cnt++)
  {
    received_data.buffer[byte_cnt+1] =  spi_readwrite(data_to_send.buffer[byte_sent_cnt++]);
  }
  
  digitalWrite(HAL_IO_RADIO_REQN, 1);

  hal_pltf_disable_spi();  

//   if (0 == received_data.status_byte)
//   {
//     //The debug/status byte should be 0 for a received event otherwise
//     //we will spin in this while loop
//     Serial.print(F("Error: Expected 0 as ACI Debug/Status byte in ACI Event got"));
//     Serial.println(DEC, received_data.status_byte);
//     Serial.println(F("Spinning in a while loop"));
//   	while(1);
//   }
  
  /* valid Rx available or transmit finished*/
  hal_aci_tl_msg_rcv_hook(&received_data);
}
#endif


hal_aci_data_t * hal_aci_tl_poll_get(void)
{
    uint8_t byte_cnt;
    uint8_t byte_sent_cnt;
    uint8_t max_bytes;
    hal_aci_data_t data_to_send;

    memset(data_to_send.buffer,0,sizeof(data_to_send.buffer));
    memset(received_data.buffer,0,sizeof(received_data.buffer));
    //bool is_transmit_finished = false; // Unreferenced variable

    // REQN already set
    //hal_pltf_enable_spi();
    //pinMode(5, OUTPUT);
    //digitalWrite(HAL_IO_RADIO_REQN, 0);
  
    // Set request pin low to indicate to nRF8001 we want to send data
    SET_REQN_LOW();

    // Wait for RDYN to go low, if it's not already low - TODO!
    //if(nRF8001_RDYN_PINREG & nRF8001_RDYN_PIN)
    if(NRF8001_RDYN_PIN_INPUT_REG & NRF8001_RDYN_PIN)
    {
    	do
    	{
    		// Wait for nRF8001 to indicate it is ready by waiting for RDYN
    		//_BIS_SR(LPM0_bits + GIE); // Enter LPM0 w/interrupt
    		//_nop();
            asm volatile ("nop");
    	}while(rdynFlag == 0);
    }

    // Receive from queue
    if (m_aci_q_dequeue(&aci_tx_q, &data_to_send) == false)
    {
        /* queue was empty, nothing to send */
        data_to_send.status_byte = 0;
        data_to_send.buffer[0] = 0;
    }

  
    //Change this if your mcu has DMA for the master SPI
    //When using DMA always clock out the max length of the message being sent and
    // event being received or HAL_ACI_MAX_LENGTH
  
    // Send length, receive header
    byte_sent_cnt = 0;
    received_data.status_byte = spi_readwrite(data_to_send.buffer[byte_sent_cnt++]);

    // Send first byte, receive length from slave
    received_data.buffer[0] = spi_readwrite(data_to_send.buffer[byte_sent_cnt++]);
    //received_data.status_byte = spi_readwrite(0x01);

    //// Send first byte, receive length from slave
    //received_data.buffer[0] = spi_readwrite(0x09);

    if (0 == data_to_send.buffer[0])
    {
      max_bytes = received_data.buffer[0];
    }
    else
    {
      // Set the maximum to the biggest size. One command byte is already sent
      max_bytes = (received_data.buffer[0] > (data_to_send.buffer[0] - 1)) 
        ? received_data.buffer[0] : (data_to_send.buffer[0] - 1);
    }


    if (max_bytes > HAL_ACI_MAX_LENGTH)
    {
        max_bytes = HAL_ACI_MAX_LENGTH;
    }

    // Transmit/receive the rest of the packet 
    for (byte_cnt = 0; byte_cnt < max_bytes; byte_cnt++)
    {
        received_data.buffer[byte_cnt+1] =  spi_readwrite(data_to_send.buffer[byte_sent_cnt++]);
    }
    // Deassert REQN to indicate end of transmission
    SET_REQN_HIGH();
    //digitalWrite(HAL_IO_RADIO_REQN, 1);
    //hal_pltf_disable_spi();
    //RDYN should follow the REQN line in approx 100ns

    //_delay_loop_1(10); // Delay to allow REQN to stay high for the required amount of time - TODO!
    _delay_ms(5); // Delay to allow REQN to stay high for the required amount of time - TODO!

  
  //If there are more ACI commands in the queue, lower the REQN line immediately
  if (false == m_aci_q_is_empty(&aci_tx_q))
  {
      //digitalWrite(HAL_IO_RADIO_REQN, 0);
      // Set request pin low to indicate to nRF8001 we want to send data
    	SET_REQN_LOW();

    	do
    	{
    		// Wait for nRF8001 to indicate it is ready by waiting for RDYN - TODO!
    		//_BIS_SR(LPM0_bits + GIE); // Enter LPM0 w/interrupt
    		//_nop();
            asm volatile ("nop");
    	}while(rdynFlag == 0);
  
  }
  
  //sleep_enable();
  //attachInterrupt(1, m_rdy_line_handle, LOW);

  /*
  // Enable interrupts and enter LPM0 so we wait for the RDYN to be
  // set low by the nRF8001, indicating its ready to receive data
  	if(nRF8001_RDYN_PINREG & nRF8001_RDYN_PIN)
  	{
  		do
  		{
  			// Wait for nRF8001 to indicate it is ready by waiting for RDYN
  			_BIS_SR(LPM0_bits + GIE); // Enter LPM0 w/interrupt
  			_nop();
  		}while(rdynFlag == 0);
  	}
    */

  
  /* valid Rx available or transmit finished*/
  return (&received_data);
}

// Receive a byte from the nRF8001 by sending
// an ACI_BYTE.
static uint8_t spi_readwrite(const uint8_t aci_byte)
{
	
    #define DUMMY_BYTE 0x00
    SPDR = aci_byte;
    while (!(SPSR & _BV(SPIF)))
      ;
    SPDR = DUMMY_BYTE;
    while (!(SPSR & _BV(SPIF)))
      ;
    return SPDR;

	//// Send byte and wait for byte back

	//spiByteRxFlag = 0;

	//// Clear RX IFG
	//SPI_IFG &= ~(SPI_RX_IFG);

	//// Wait for SPI TX buffer to be ready
	//while(!(SPI_IFG & SPI_TX_IFG));

	//// Send byte over SPI
	//SPI_TX_BUF = aci_byte;

	//// Wait until a byte has been received
	//while(spiByteRxFlag == 0)
	//{
	//	// Wait for nRF8001 to indicate it is ready by waiting for RDYN
	//	_BIS_SR(LPM0_bits + GIE); // Enter LPM0 w/interrupt
	//	_nop();
	//}

	//return rxByte;
}

/* *************************************************************
 * Port Interrupt for RDYN line
 * *********************************************************** */
ISR(INT0_vect) 
{
	// Set flag so it can be processed in the main loop
	rdynFlag = 1;

    //PORTB ^= (1<<LED1_PHYSICAL_PIN);
	//// Exit from sleep mode upon ISR exit so data can be sent
	//_BIC_SR_IRQ( LPM0_bits );
	//__no_operation();
    asm volatile ("nop");
}


//// USCI A0/B0 RX interrupt vector TODO!
//#pragma vector=USCIAB0RX_VECTOR
//__interrupt void USCIA0RX_ISR(void)
//{
//	if(SPI_IFG & SPI_RX_IFG)
//	{
//		// Byte was received, so set flag and clear interrupt
//		// flag
//		SPI_IFG &= ~(SPI_RX_IFG);
//		spiByteRxFlag = 1;
//
//		rxByte = UCB0RXBUF;
//
//		// Exit from sleep mode upon ISR exit so data can be
//		// processed
//		_BIC_SR_IRQ( LPM0_bits );
//	}
//}
