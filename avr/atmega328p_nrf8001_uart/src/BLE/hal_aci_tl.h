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
 * $LastChangedRevision$
 */ 

/** @file
 * @brief Interface for hal_aci_tl.
 */
 
/** @defgroup hal_aci_tl hal_aci_tl
@{
@ingroup hal
 
@brief Module for the ACI Transport Layer interface
@details This module is responsible for sending and receiving messages over the ACI interface of the nRF8001 chip.
 The hal_aci_tl_send_cmd() can be called directly to send ACI commands.


The RDYN line is hooked to an interrupt on the MCU when the level is low.
The SPI master clocks in the interrupt context.
The ACI Command is taken from the head of the command queue is sent over the SPI
and the received ACI event is placed in the tail of the event queue.

*/
 
#ifndef HAL_ACI_TL_H__
#define HAL_ACI_TL_H__

#include "spi.h"
#include <stdint.h>
#include <stdbool.h>

#ifndef HAL_ACI_MAX_LENGTH
#define HAL_ACI_MAX_LENGTH 31
#endif //HAL_ACI_MAX_LENGTH

#define ACI_QUEUE_SIZE  3

/** Data type for ACI commands and events */
typedef struct hal_aci_data_t{
  uint8_t status_byte;
  uint8_t buffer[HAL_ACI_MAX_LENGTH+1];
} hal_aci_data_t;


/** ACI Transport Layer initialization.
 */
void hal_aci_tl_init(void);

/**@brief Sends an ACI command to the radio.
 *  @details
 *  This function sends an ACI command to the radio by placing it in the ACI command queue 
 *  @param aci_buffer Pointer to the message to send.
 *  @return True if the ACI command was successfully placed in the ACI command queue, 
 *          False if the ACI command queue is full.
 */
bool hal_aci_tl_send(hal_aci_data_t *aci_buffer);


/** @brief Check for pending transaction.
 *  @details 
 *  Call this function from the main context at regular intervals to check if the nRF8001 RDYN line indicates a pending transaction.
 *  If a transaction is pending, this function will treat it and call the receive hook.
 */
void hal_aci_tl_poll_rdy_line(void);

/** @brief Run the SPI in the interrupt context
 *  @details 
 *  Call this function from the interrupt context triggered by the nRF8001 RDYN line
 */
hal_aci_data_t * hal_aci_tl_poll_get(void);

/** @brief Get an ACI event from the ACI Event queue
 *  @details 
 *  Call this function from the main context.
 */
bool hal_aci_tl_event_get(hal_aci_data_t *p_aci_data);

/** @brief Flush the ACI command Queue and the ACI Event Queue
 *  @details
 *  Call this function in the main thread
*/
void m_aci_q_flush(void);

void m_rdy_line_handle(void);



#endif // HAL_ACI_TL_H__
/** @} */




