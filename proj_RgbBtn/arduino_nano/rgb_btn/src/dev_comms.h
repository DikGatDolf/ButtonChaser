/*****************************************************************************

dev_comms.h

Include file for dev_comms.c

******************************************************************************/
#ifndef __dev_comms_H__
#define __dev_comms_H__


/******************************************************************************
includes
******************************************************************************/
#include "Arduino.h"
#include "sys_utils.h"
#include "../../../../common/common_comms.h"
/******************************************************************************
Macros
******************************************************************************/

/******************************************************************************
Struct & Unions
******************************************************************************/

/******************************************************************************
Public variables
******************************************************************************/

/******************************************************************************
Public function definitions
******************************************************************************/
void dev_comms_init(void);

bool dev_comms_tx_ready(void);

int8_t dev_comms_rx_msg_available(uint8_t * _src, uint8_t * _dst, uint8_t * _data);

uint8_t dev_comms_addr_get(void);
bool dev_comms_addr_set(uint8_t addr);
uint8_t dev_comms_addr_new(void);
void dev_comms_blacklist_add(uint8_t new_addr);

// uint8_t dev_comms_rx_msg_src_addr();
// uint8_t dev_comms_rx_msg_dst_addr();
// uint8_t dev_comms_rx_msg_data_len();


void dev_comms_tx_service(void);
//bool dev_comms_response_start(void);
//unsigned int dev_comms_response_append(master_command_t cmd, response_code_t resp_code, uint8_t * data, uint8_t data_len);
unsigned int dev_comms_response_append(master_command_t cmd, response_code_t resp_code, uint8_t * data, uint8_t data_len, bool restart = false);
size_t dev_comms_response_add_byte(uint8_t data);
void dev_comms_response_send(void);
void dev_comms_transmit_now(void);

void dev_comms_check_error(void);


#endif /* __dev_comms_H__ */

/****************************** END OF FILE **********************************/
