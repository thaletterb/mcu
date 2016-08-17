/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT. 
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRENTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $LastChangedRevision: 4808 $
 */ 
/* Attention! 
*  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile 
*  qualification listings, this section of source code must not be modified.
*/

/** @file
  @brief Implementation of the ACI library.
 */

#include "hardware.h"
#include "aci.h"
#include "aci_cmds.h"
#include "aci_evts.h"

#include "lib_aci.h"
#include "hal_aci_tl.h"


#define LIB_ACI_DEFAULT_CREDIT_NUMBER   1


static services_pipe_type_mapping_t * p_services_pipe_type_map;
static uint8_t                        pipe_count;
static hal_aci_data_t *               p_setup_msgs;
static uint8_t                        setup_msgs_count;
                  


static hal_aci_data_t  msg_to_send;

static hal_aci_data_t  *p_rcvd_evt;




static uint16_t cx_rf_interval;  // rf_interval = cx_rf_interval * 1.25 ms Range:0x0006 to 0x0C80
static uint16_t current_slave_latency;
static uint8_t total_nb_available_credits;
static uint8_t nb_available_credits;





bool lib_aci_event_get(aci_state_t *aci_stat, hal_aci_evt_t *p_aci_evt_data)
{
  bool status;
  status = hal_aci_tl_event_get((hal_aci_data_t *)p_aci_evt_data);
  
  /**
  Update the state of the ACI witn the 
  ACI Events -> Pipe Status, Disconnected, Connected, Bond Status, Pipe Error
  */
  {
    aci_evt_t * aci_evt;
    
    aci_evt = &p_aci_evt_data->evt;  
    
    switch(aci_evt->evt_opcode)
    {
        case ACI_EVT_PIPE_STATUS:
            {
                uint8_t i=0;
                
                for (i=0; i < PIPES_ARRAY_SIZE; i++)
                {
                  aci_stat->pipes_open_bitmap[i]   = aci_evt->params.pipe_status.pipes_open_bitmap[i];
                  aci_stat->pipes_closed_bitmap[i] = aci_evt->params.pipe_status.pipes_closed_bitmap[i];
                }
            }
            break;
        
        case ACI_EVT_DISCONNECTED:
            {
                uint8_t i=0;
                
                for (i=0; i < PIPES_ARRAY_SIZE; i++)
                {
                  aci_stat->pipes_open_bitmap[i] = 0;
                  aci_stat->pipes_closed_bitmap[i] = 0;
                }
                aci_stat->confirmation_pending = false;
                aci_stat->data_credit_available = aci_stat->data_credit_total;
                
            }
            break;
        
        
    }
    
  }
  
  return status;
}



