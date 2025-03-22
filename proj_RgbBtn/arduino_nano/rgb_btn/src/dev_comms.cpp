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

I2C
	RAM:   uses 1435 - 1247 = 188 bytes from 2048 bytes (9.18%)
	Flash: used 27334 - 25558 = 1776 bytes from 30720 bytes (5.78%)

SoftwareSerial
	RAM:   uses 1384 - 1261 = 123 bytes from 2048 bytes (6.01%)
	Flash: used 27334 - 25566 = 1768 bytes from 30720 bytes (5.76%)

HardwareSerial
Look, we are already using the HardwareSerial for the console, so we can't use 
it for the comms as well. Let's start with a simple STX-DLE-ETX protocol.



 ******************************************************************************/
#include "sys_utils.h"
#include "str_helper.h"
#include "dev_console.h"

#if (COMMS_USE_I2C == 1)
  #include <Wire.h>
#endif

#define __NOT_EXTERN__
#include "dev_comms.h"
#undef __NOT_EXTERN__

#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif

#if (COMMS_USE_I2C == 1)
  #define PRINTF_TAG ("I2C") /* This must be undefined at the end of the file*/
#else
  #define PRINTF_TAG ("COMMS") /* This must be undefined at the end of the file*/
#endif

/******************************************************************************
Macros
******************************************************************************/
//#define I2C_BROADCAST_ADDR      (0x60)  /* b0110 xxxx */

/* Max number of bytes we can RX in the i2c in a single transaction */
#define COMMS_MSG_LEN_MAX     (sizeof(rgb_btn_tx_data_t))

#define STX     (0x02)
#define DLE     (0x10)
#define ETX     (0x03)

/******************************************************************************
Struct & Unions
******************************************************************************/
// typedef struct {
//     uint8_t rx[RGB_BTN_MSG_MAX_LEN];
//     uint8_t tx[RGB_BTN_MSG_MAX_LEN];
//     struct {
//         uint8_t rx_done     : 1;
//         uint8_t rx_overflow : 1;
//         uint8_t tx_busy     : 1;
//         uint8_t tx_done     : 1;
//     } flags;
// }rgb_btn_commst_t;

typedef struct {
    struct {
        uint8_t buff[RGB_BTN_MSG_MAX_LEN];
        uint8_t in;       //RX'd bytes go in here
        uint8_t out;       //Read bytes come out here
        struct {
            uint8_t done     : 1;
            uint8_t overflow : 1;
            uint8_t crc_err  : 1;
        } flags;
    }rx;
    struct {
        uint8_t buff[RGB_BTN_MSG_MAX_LEN];
        uint8_t len;
        struct {
            uint8_t tx_busy     : 1;
            uint8_t tx_done     : 1;
        } flags;
    }tx;
}rgb_btn_msg_t;

typedef struct dev_comms_st
{
    bool initialised = false;
    rgb_btn_msg_t msg;
    int rx_cnt;
    int tx_cnt;
    uint8_t addr;   /* I2C address */
}dev_comms_t;

/******************************************************************************
Local function definitions
******************************************************************************/
#if (COMMS_USE_I2C == 1)
  void _i2c_rx_handler(int nr_of_bytes_rx_from_master);
  void _i2c_tx_handler(void);
#else
#endif
/******************************************************************************
Local variables
******************************************************************************/

dev_comms_t _comms;
/******************************************************************************
Local functions
******************************************************************************/

#if (COMMS_USE_I2C == 1)
void _i2c_rx_handler(int nr_of_bytes_rx_from_master)
{
    //This is executed in interrupt space.... which means:
    //NO printfs
    //NO delays
    //NO blocking functions

    //I think we will be getting this callback once the master has sent the entire message (up to 32 bytes)
    //We will need to read the data from the bus and store it in our buffer
    uint8_t calc_crc = 0;
    uint8_t cnt = 0;

    while (Wire.available())
    {
        //Returns a single byte rx'd on the bus (from the master)
        int c = Wire.read();
        
        if (((_comms.msg.rx.in + cnt) < RGB_BTN_MSG_MAX_LEN) && (_comms.msg.rx.flags.overflow == 0))
        {
            calc_crc = crc8(calc_crc, c);
            _comms.msg.rx.buff[_comms.msg.rx.in + cnt] = c;
        }
        else //We have received too many bytes
        {
            _comms.msg.rx.flags.overflow = 1;
        }
        cnt++;
    }
    
    //Increment, whether it is overflowing or not
    _comms.msg.rx.in += cnt;

    if (calc_crc != 0)
        _comms.msg.rx.flags.crc_err = 1;
    else if (cnt > 0) // reverse 1, don't count the crc byte
        _comms.msg.rx.in--;   //We don't want to include the CRC byte in the message

    //Only set this flag if we have received something
    _comms.msg.rx.flags.done = (_comms.msg.rx.in > 0 )? 1 : 0;
    //Using the done flag prevent the application from reading incomplete messages which are still busy with transmission
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
#endif

/******************************************************************************
Public functions
******************************************************************************/
void dev_comms_init(uint8_t addr)
{
    char buff[10];
    //First we need to set the I2C address
    // The top 4 bits will be fixed at 0000 0000 ( 0x00 )
    _comms.addr = addr;//RGB_BTN_I2C_TYPE_ID;

    dev_comms_rx_reset();

    //Just set a new address if we are already initialised
    if (!_comms.initialised)
    {
#if COMMS_USE_I2C == 1
        Wire.begin(/*I2C_BROADCAST_ADDR*/ _comms.addr);
        Wire.onReceive(_i2c_rx_handler);
        Wire.onRequest(_i2c_tx_handler);
        Wire.setClock(RGB_BTN_I2C_CLK_FREQ); // let's try fast mode: 400kHz
        iprintln(trCOMMS, "#Initialised at %s kHz (address: 0x%02X)", float2str(buff, RGB_BTN_I2C_CLK_FREQ/1000.0f, 1, 10), _comms.addr);
#else
        iprintln(trCOMMS, "#Initialised (address: 0x%02X)", _comms.addr);
#endif
    }
    else
    {
        iprintln(trCOMMS, "#Address updated: 0x%02X", _comms.addr);
    }

    _comms.initialised = true;
}

uint8_t dev_comms_rx_available(void)
{
    if (_comms.msg.rx.flags.done)
    {
        return (_comms.msg.rx.in - _comms.msg.rx.out);
    }
    return 0;
}

bool dev_comms_rx_error(uint8_t * err_data)
{
    if ((_comms.msg.rx.flags.overflow == 1) && (err_data != NULL))
        *err_data = _comms.msg.rx.in - RGB_BTN_MSG_MAX_LEN;

    return ((_comms.msg.rx.flags.overflow) || (_comms.msg.rx.flags.crc_err));
}

char * dev_comms_rx_error_msg(void)
{
    static char buff[32];
    buff[0] = 0;
    if ((_comms.msg.rx.flags.overflow) && (_comms.msg.rx.flags.crc_err))
        //                            12345678901234567890123456789012
        snprintf(buff, sizeof(buff), "Overflow (%d) & CRC", _comms.msg.rx.in - RGB_BTN_MSG_MAX_LEN);
    else if (_comms.msg.rx.flags.overflow)
        snprintf(buff, sizeof(buff), "Overflow (%d)", _comms.msg.rx.in - RGB_BTN_MSG_MAX_LEN);
    else if (_comms.msg.rx.flags.crc_err)
        snprintf(buff, sizeof(buff), "CRC Error");

    return buff;
}

size_t dev_comms_rx_read(uint8_t * data, size_t len)
{
    size_t read_len = 0;
    if (_comms.msg.rx.flags.done)
    {
        if (data != NULL)
        {
            //The head could have been moved beyond the buffer limit if there is an overflow
            read_len = min(RGB_BTN_MSG_MAX_LEN, _comms.msg.rx.in);
            //We need to subtract the current position of the tail and see if we can read that many bytes
            read_len = min(len, read_len);
            //RVN - TODO - we probably need to check that the tail is <= than the head at this point.... don't want to read more than we have
            for (uint8_t i = 0; i < read_len; i++)
                data[i] = _comms.msg.rx.buff[_comms.msg.rx.out + i];
        }
        _comms.msg.rx.out += read_len;
    }
    return read_len;
}

void dev_comms_rx_reset(void)
{
    _comms.msg.rx.in = 0;
    _comms.msg.rx.out = 0;
    _comms.msg.rx.flags = {0};
}

void dev_comms_service(void)
{
#if COMMS_USE_I2C == 1
    //Have we received anything?
    if (_comms.msg.rx.flags.done)
    {
        iprintln(trCOMMS, "#Received %d bytes", _comms.msg.rx.in);
        if (_comms.msg.rx.flags.overflow)
        {
            iprintln(trCOMMS, "#%d bytes Overflow ", _comms.msg.rx.in - RGB_BTN_MSG_MAX_LEN);
        }
    }
#endif
}

#undef PRINTF_TAG
/*************************** END OF FILE *************************************/
