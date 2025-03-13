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

uint8_t dev_comms_rx_available(void);

size_t dev_comms_rx_read(uint8_t * data, size_t len);

void dev_comms_rx_reset(void);

bool dev_comms_rx_error(uint8_t * err_data);
char * dev_comms_rx_error_msg(void);

void dev_comms_service(void);

#endif /* __dev_comms_H__ */

/****************************** END OF FILE **********************************/
