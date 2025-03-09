/******************************************************************************
Project:   RGB Button Chaser
Module:     dev_comms.c
Purpose:    This file contains the communication implementation
Author:     Rudolph van Niekerk
Processor:  Arduino Nano (ATmega328)
Compiler:	Arduino AVR Compiler

This will implement the communications with the master device over I2C. 
Since device will be placed in an array of maximum 16 devices, we will use the 
4 inputs to determine the value of the bottom 4 bits of the I2C address. 

 ******************************************************************************/

#include <Wire.h>
 
#define __NOT_EXTERN__
#include "dev_comms.h"
#undef __NOT_EXTERN__

#include "std_utils.h"

/******************************************************************************
Macros
******************************************************************************/
#define I2C_SLAVE_ADDR      (0xC0)  /* b0011 xxxx */

/******************************************************************************
Local function definitions
******************************************************************************/

/******************************************************************************
Struct & Unions
******************************************************************************/

/******************************************************************************
Local variables
******************************************************************************/

/******************************************************************************
Local functions
******************************************************************************/

void i2c_slave_init(void)
{
    // Wire.begin(I2C_SLAVE_ADDR);
    // Wire.onReceive(receiveEvent);
    // Wire.onRequest(requestEvent);
}
/******************************************************************************
Public functions
******************************************************************************/

/*************************** END OF FILE *************************************/
