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

We start the system up by "listening" on the I2C bus for the BROADCAST address. 
This way we can have the master perform synchronisation.
The master can then also send a command to set a specific device (address 0x50 to 0x5F)
in "response" mode.



 ******************************************************************************/

#include <Wire.h>
 
#define __NOT_EXTERN__
#include "dev_comms.h"
#undef __NOT_EXTERN__

#include "sys_utils.h"
#include "str_helper.h"
#include "dev_console.h"


#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("RGB") /* This must be undefined at the end of the file*/

/******************************************************************************
Macros
******************************************************************************/
//#define I2C_BROADCAST_ADDR      (0x60)  /* b0110 xxxx */
#define I2C_SLAVE_ADDR      (0x50)  /* b0101 xxxx */
#define I2C_DYN_ADDR_PINS   (4)     /* Number of pins to determine the address */

/* Max number of bytes we can RX in the i2c in a single transaction */
#define COMMS_RX_BUFF     16 

/******************************************************************************
Struct & Unions
******************************************************************************/

typedef struct dev_comms_st
{
    bool initialised = false;
	struct
	{
		char buff[COMMS_RX_BUFF]; /* The rx buff for data read from the console. */
		int cnt;						/* The index into the rx buff. */
	} rx;
    
    uint8_t addr;   /* I2C address */
}dev_comms_t;

/******************************************************************************
Local function definitions
******************************************************************************/
void _i2c_rx_handler(int nr_of_bytes_rx_from_master);
void _i2c_tx_handler(void);

/******************************************************************************
Local variables
******************************************************************************/
static const PROGMEM uint8_t comms_pins[I2C_DYN_ADDR_PINS] = {btnChasePin_Addr0, btnChasePin_Addr1, btnChasePin_Addr2, btnChasePin_Addr3};

dev_comms_t _comms;
/******************************************************************************
Local functions
******************************************************************************/

void _i2c_rx_handler(int nr_of_bytes_rx_from_master)
{
    //This is executed in interrupt space.... which means:
    //NO printfs
    //NO delays
    //NO blocking functions

    while (Wire.available())
    {
        //Returns a single byte rx'd on the bus (from the master)
        int c = Wire.read();
        if (_comms.rx.cnt < COMMS_RX_BUFF)
        {
            _comms.rx.buff[_comms.rx.cnt++] = c;
        }
        //else Silently discard
    }

    return;
}

void _i2c_tx_handler(void)
{
    //This is executed in interrupt space.... which means:
    //NO printfs
    //NO delays
    //NO blocking functions
    
    //Call "Wire.write()" here to send data back to the master"

    return;
}

/******************************************************************************
Public functions
******************************************************************************/
void dev_comms_init(void)
{
    //First we need to set the I2C address
    // The top 4 bits will be fixed at 0101 0000 ( 0x50 )
    _comms.addr = I2C_SLAVE_ADDR;
    // The bottom 4 bits of the I2C address will be determined by the 4 inputs
    //  on the device
    for (uint8_t i = 0; i < I2C_DYN_ADDR_PINS; i++)
    {
        uint8_t pin = pgm_read_byte(&comms_pins[i]);
        pinMode(pin, INPUT);
        if (quickPinRead(pin) == HIGH)
            _comms.addr |= (1 << i);
    }

    //Clear the rx buffer
    _comms.rx.cnt = 0;

    Wire.begin(/*I2C_BROADCAST_ADDR*/ _comms.addr);
    Wire.onReceive(_i2c_rx_handler);
    Wire.onRequest(_i2c_tx_handler);
    Wire.setClock(400000); // let's try fast mode: 400kHz

    // Wire.begin(I2C_SLAVE_ADDR);
    // Wire.onReceive(receiveEvent);
    // Wire.onRequest(requestEvent);
    iprintln(trCOMMS, "#I2C address set to 0x%02X", _comms.addr);

    _comms.initialised = true;
}


void dev_comms_service(void)
{

}

#undef PRINTF_TAG
/*************************** END OF FILE *************************************/
