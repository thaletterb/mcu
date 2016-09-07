/*
 * ble_queue.h
 *
 *  Created on: Apr 9, 2015
 *      Author: Dany
 */

#ifndef BLE_QUEUE_H_
#define BLE_QUEUE_H_

#include "lib_aci.h"

void defaultRX(uint8_t *buffer, uint8_t len);
int available(void);
int read(void);
int peek(void);
uint8_t num_elems(void);
uint8_t* flush(void);
uint8_t write(uint8_t*, uint8_t, aci_state_t*, hal_aci_evt_t* aci_data, hal_aci_data_t* aci_cmd);
void pollACI(aci_state_t* aci_state, hal_aci_evt_t* aci_data, hal_aci_data_t* aci_cmd);
void begin_BLE(aci_state_t *aci_state);

#endif /* BLE_QUEUE_H_ */
