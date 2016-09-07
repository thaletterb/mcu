#include <stdint.h>
#include "ble_interface.h"
#include "services.h"
#include "lib_aci.h"
#include "aci.h"
#include "hal_aci_tl.h"

#include "util/delay.h"

/* Get the service pipe data created in nRFGo Studio */
#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
    static services_pipe_type_mapping_t
        services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
#else
    #define NUMBER_OF_PIPES 0
    static services_pipe_type_mapping_t * services_pipe_type_mapping = NULL;
#endif

static const hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] = SETUP_MESSAGES_CONTENT;
//This might fill up RAM. Might want to write it to Flash

#define BLE_UART_RXBUFFER_SIZE 64
static uint8_t ble_rx_buffer[BLE_UART_RXBUFFER_SIZE];
static uint8_t buffer[BLE_UART_RXBUFFER_SIZE];
static uint16_t ble_rx_head;
static uint16_t ble_rx_tail;
static uint8_t setup_ctr=0;
uint8_t sendbuffer[20];
uint8_t* received_array;
uint8_t num_elem_received;
bool new_data = false;

void begin_BLE(aci_state_t *aci_state) {
	  if (0 != services_pipe_type_mapping)
	  {
	    aci_state->aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
	  }
	  else
	  {
	    aci_state->aci_setup_info.services_pipe_type_mapping = 0;
	  }
	  aci_state->aci_setup_info.number_of_pipes    = NUMBER_OF_PIPES;
	  aci_state->aci_setup_info.setup_msgs         = (hal_aci_data_t*)setup_msgs;
	  aci_state->aci_setup_info.num_setup_msgs     = NB_SETUP_MESSAGES;

	  /* Pass the service data into the appropriate struct in the ACI */
	  lib_aci_init(aci_state);
}

int available(void)
{
  return (uint16_t)(BLE_UART_RXBUFFER_SIZE + ble_rx_head - ble_rx_tail)
    % BLE_UART_RXBUFFER_SIZE;
}

void defaultRX(uint8_t *buffer, uint8_t len)
{
	int i;
  for(i=0; i<len; i++)
  {
    uint16_t new_head = (uint16_t)(ble_rx_head + 1) % BLE_UART_RXBUFFER_SIZE;

    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (new_head != ble_rx_tail) {
      ble_rx_buffer[ble_rx_head] = buffer[i];

      // debug echo print
      // Serial.print((char)buffer[i]);

      ble_rx_head = new_head;
    }
  }
}

uint8_t write(uint8_t * buffer, uint8_t len, aci_state_t *aci_stat, hal_aci_evt_t* aci_data, hal_aci_data_t* aci_cmd)
{
  uint8_t bytesThisPass, sent = 0;

  while(len) { // Parcelize into chunks
    bytesThisPass = len;
    if(bytesThisPass > ACI_PIPE_TX_DATA_MAX_LEN)
       bytesThisPass = ACI_PIPE_TX_DATA_MAX_LEN;

    if(!lib_aci_is_pipe_available(aci_stat, PIPE_UART_OVER_BTLE_UART_TX_TX))
    {
      pollACI(aci_stat, aci_data, aci_cmd);
      continue;
    }

    lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, &buffer[sent],
      bytesThisPass);
    aci_stat->data_credit_available--;

    //__delay_cycles(100); // required delay between sends
    _delay_us(100); // required delay between sends

    if(!(len -= bytesThisPass)) break;
    sent += bytesThisPass;
  }

  return sent;
}

int read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (ble_rx_head == ble_rx_tail) {
    return -1;
  } else {
    unsigned char c = ble_rx_buffer[ble_rx_tail];
    ble_rx_tail ++;
    ble_rx_tail %= BLE_UART_RXBUFFER_SIZE;
    return c;
  }
}

uint8_t num_elems(void) {
	return ble_rx_head-ble_rx_tail;
}

 uint8_t* flush(void) {
	 memcpy(buffer, ble_rx_buffer, ble_rx_head-ble_rx_tail);
	 ble_rx_head = 0;
	 ble_rx_tail = 0;
	 return buffer;
 }

int peek(void)
{
  if (ble_rx_head == ble_rx_tail) {
    return -1;
  } else {
    return ble_rx_buffer[ble_rx_tail];
  }
}

void pollACI(aci_state_t* aci_state, hal_aci_evt_t* aci_data, hal_aci_data_t* aci_cmd) {
		// We enter the if statement only when there is a ACI event available to be processed
		if (lib_aci_event_get(aci_state, aci_data))
		{
			aci_evt_t * aci_evt;

			aci_evt = &(aci_data->evt);
			switch(aci_evt->evt_opcode)
			{
				/**
				As soon as you reset the nRF8001 you will get an ACI Device Started Event
				*/
				case ACI_EVT_ECHO:
					//_nop();
                    asm volatile ("nop");
					aci_cmd->buffer[0] = 2;    //Length of ACI command
					aci_cmd->buffer[1] = ACI_CMD_TEST; //Command - Test
					aci_cmd->buffer[2] = ACI_TEST_MODE_EXIT; //Command parameter
					hal_aci_tl_send(aci_cmd);
					break;
				case ACI_EVT_DEVICE_STARTED:
				  aci_state->data_credit_available = aci_evt->params.device_started.credit_available;
				  switch(aci_evt->params.device_started.device_mode)
				  {
				    case ACI_DEVICE_SLEEP:
					  aci_cmd->buffer[0] = 1;    //Length of ACI command
					  aci_cmd->buffer[1] = ACI_CMD_WAKEUP;
				    	break;
					case ACI_DEVICE_SETUP:
						/*
					  aci_cmd.buffer[0] = 2;    //Length of ACI command
					  aci_cmd.buffer[1] = ACI_CMD_TEST; //Command - Test
					  aci_cmd.buffer[2] = ACI_TEST_MODE_DTM_UART; //Command parameter
					  hal_aci_tl_send(&aci_cmd);
					  */
					  memcpy(aci_cmd, &setup_msgs[setup_ctr], 2+setup_msgs[setup_ctr].buffer[0]);
					  hal_aci_tl_send(aci_cmd);
					  setup_ctr++;
						/*
					  if (ACI_STATUS_TRANSACTION_COMPLETE != do_aci_setup(&aci_state)) {
						  //while(1);
						  _nop();
					  }
					  */
					  break;
					case ACI_DEVICE_STANDBY:
					 /* Start advertising ... first value is advertising time in seconds, the */
					 /* second value is the advertising interval in 0.625ms units */
						/*
					 	 if (device_name[0] != 0x00) { //device name
                           lib_aci_set_local_data(&aci_state, PIPE_GAP_DEVICE_NAME_SET , (uint8_t *)&device_name, strlen(device_name));
                         }
                         */
                         lib_aci_connect(180, 0x50);//check that this means seconds and miliseconds
                         //lib_aci_connect(adv_timeout, adv_interval);
                         /* I think this is for keeping track of the state on the MCU
                         defaultACICallback(ACI_EVT_DEVICE_STARTED);
                         if (aci_event) {
                        	 aci_event(ACI_EVT_DEVICE_STARTED);
                         */
                         break;

					case ACI_DEVICE_TEST:
						aci_cmd->buffer[0] = 3; //length
						aci_cmd->buffer[1] = ACI_CMD_ECHO;
						aci_cmd->buffer[2] = 0xDE;
						aci_cmd->buffer[3] = 0xAD;
					    hal_aci_tl_send(aci_cmd);
						//_nop();
                        asm volatile ("nop");
					  break;
					}
				  break; //ACI Device Started Event
			  case ACI_EVT_DATA_CREDIT:
				  aci_state->data_credit_available = aci_state->data_credit_available + aci_evt->params.data_credit.credit;
				  break;
			  case ACI_EVT_DATA_RECEIVED:
				defaultRX(aci_evt->params.data_received.rx_data.aci_data, aci_evt->len - 2);
				num_elem_received = num_elems();//this must run before flush!
				received_array = flush();
                write(received_array, num_elem_received, aci_state, aci_data, aci_cmd);
				new_data = true;
				/*
				if (rx_event)
					rx_event(aci_evt->params.data_received.rx_data.aci_data, aci_evt->len - 2);
				*/
				break;
			  case ACI_EVT_DISCONNECTED:
				  //__delay_cycles(1000);
				  _delay_ms(1);
                  lib_aci_connect(180, 0x50);//check that this means seconds and miliseconds
				  break;
		      case ACI_EVT_CONNECTED:
		    	  aci_state->data_credit_available = aci_state->data_credit_total;
		    	  break;
			  case ACI_EVT_CMD_RSP:
				//If an ACI command response event comes with an error -> stop
				if (ACI_STATUS_TRANSACTION_CONTINUE == aci_evt->params.cmd_rsp.cmd_status) {
					  memcpy(aci_cmd, &setup_msgs[setup_ctr], 2+setup_msgs[setup_ctr].buffer[0]);
					  hal_aci_tl_send(aci_cmd);
					  setup_ctr++;
				}
				else if (ACI_STATUS_TRANSACTION_COMPLETE == aci_evt->params.cmd_rsp.cmd_status) {
					//_nop();
                    asm volatile ("nop");
				}
				else if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status)
				{
					//_nop();
                    asm volatile ("nop");
				}
				break;
			}
		}
		else
		{
		// No event in the ACI Event queue.
		// Arduino can go to sleep
			if (new_data) {
				//write here!
				new_data = false;
			}
			//_nop();
            asm volatile ("nop");
		// Wakeup from sleep from the RDYN line
		}
}
