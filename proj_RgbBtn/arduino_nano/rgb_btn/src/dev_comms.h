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
void dev_comms_init(uint8_t addr);

uint8_t dev_comms_cmd_available(uint8_t *cmd);

uint8_t dev_comms_cmd_read(uint8_t * dst);

void dev_comms_service(void);

/*! transmits a message on the bus
 * @param[in] cmd The command to send
 * @param[in] data The data to send
 * @param[in] len The length of the data
 * @param[out] msg_handle A (unique ID) handle of the message
 * @param[out] tx_cnt The number of bytes to be placed on the bus (with 
 *                      framing and escaping)
 * @return the number of bytes of the message sent
 */
//int dev_comms_msg_send(rgb_btn_command_t cmd, uint8_t * data, int len, uint8_t * msg_handle, uint8_t * tx_cnt);

/*! Prepares the response message header to be sent back to the Master
 */
void dev_comms_response_start(void);

unsigned int dev_comms_response_add_data(rgb_btn_command_t cmd_flag, uint8_t * data, uint8_t len);

void dev_comms_response_send();


#endif /* __dev_comms_H__ */

/****************************** END OF FILE **********************************/
