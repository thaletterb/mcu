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

//#include "hardware.h"
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

// Ported from MSP430 UART project

void lib_aci_init(aci_state_t *aci_stat)
{
  uint8_t i;

  for (i = 0; i < PIPES_ARRAY_SIZE; i++)
  {
    aci_stat->pipes_open_bitmap[i]          = 0;
    aci_stat->pipes_closed_bitmap[i]        = 0;
    //aci_cmd_params_open_adv_pipe.pipes[i]   = 0;
  }

  //removed above line and below lines because those variables dont matter

  /*
  is_request_operation_pending     = false;
  is_indicate_operation_pending    = false;
  is_open_remote_pipe_pending      = false;
  is_close_remote_pipe_pending     = false;
  cur_transaction_cmd              = ACI_CMD_INVALID;
  memorized_rcvd_cmd_opcode        = ACI_CMD_INVALID;
  memorized_transaction_cmd_opcode = ACI_CMD_INVALID;
  cx_rf_interval                   = 0;
  current_slave_latency            = 0;
  request_operation_pipe           = 0;
  indicate_operation_pipe          = 0;
  cur_error_code                   = 0;
  p_rcvd_evt                       = NULL;
  */

  p_services_pipe_type_map = aci_stat->aci_setup_info.services_pipe_type_mapping;
  pipe_count               = aci_stat->aci_setup_info.number_of_pipes;
  p_setup_msgs             = aci_stat->aci_setup_info.setup_msgs;
  setup_msgs_count         = aci_stat->aci_setup_info.num_setup_msgs;

  hal_aci_tl_init();
}

bool lib_aci_connect(uint16_t run_timeout, uint16_t adv_interval) {
      aci_cmd_params_connect_t aci_cmd_params_connect;
      aci_cmd_params_connect.timeout      = run_timeout;
      aci_cmd_params_connect.adv_interval = adv_interval;
      acil_encode_cmd_connect(&(msg_to_send.buffer[0]), &aci_cmd_params_connect);
      //maybe look into what's going on over here and optimize for power (timeouts etc)
      return hal_aci_tl_send(&msg_to_send);
}

bool lib_aci_is_pipe_available(aci_state_t *aci_stat, uint8_t pipe)
{
  uint8_t byte_idx;

  byte_idx = pipe / 8;
  if (aci_stat->pipes_open_bitmap[byte_idx] & (0x01 << (pipe % 8)))
  {
    return(true);
  }
  return(false);
}

bool lib_aci_send_data(uint8_t pipe, uint8_t *p_value, uint8_t size)
{
  bool ret_val = false;
  aci_cmd_params_send_data_t aci_cmd_params_send_data;


  if(!((p_services_pipe_type_map[pipe-1].pipe_type == ACI_TX) ||
      (p_services_pipe_type_map[pipe-1].pipe_type == ACI_TX_ACK)))
  {
    return false;
  }

  if (size > ACI_PIPE_TX_DATA_MAX_LEN)
  {
    return false;
  }
  {
      aci_cmd_params_send_data.tx_data.pipe_number = pipe;
      memcpy(&(aci_cmd_params_send_data.tx_data.aci_data[0]), p_value, size);
      acil_encode_cmd_send_data(&(msg_to_send.buffer[0]), &aci_cmd_params_send_data, size);
      //is_cmd_response_expected = false;
      //Define this??^
      ret_val = hal_aci_tl_send(&msg_to_send);
  }
  return ret_val;
}

