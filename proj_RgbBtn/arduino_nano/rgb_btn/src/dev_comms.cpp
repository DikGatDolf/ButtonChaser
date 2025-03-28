/******************************************************************************
Project:   RGB Button Chaser
Module:     dev_comms.c
Purpose:    This file contains the communication implementation
Author:     Rudolph van Niekerk
Processor:  Arduino Nano (ATmega328)
Compiler:	Arduino AVR Compiler
86.805555

I2C
	RAM:   uses 1435 - 1247 = 188 bytes from 2048 bytes (9.18%)
	Flash: used 27334 - 25558 = 1776 bytes from 30720 bytes (5.78%)

SoftwareSerial
	RAM:   uses 1384 - 1261 = 123 bytes from 2048 bytes (6.01%)
	Flash: used 27334 - 25566 = 1768 bytes from 30720 bytes (5.76%)

HardwareSerial
Look, we are already using the HardwareSerial for the console, so we could maybe use 
it for the comms as well. Let's start with a simple STX-DLE-ETX protocol.

At 115200 baud, 1 byte of data transmission takes 86.80555us. 
This means that the time to transmit a 32 byte message is 2.78ms. 

RS485 runs in half duplex mode, meaning that everything we send, we also receive. 
This can be used to our advantage... after we sent something, we can check our RX 
buffer to see if it matches what we sent. If not, then we probably had some sort
of bus conflict.





 ******************************************************************************/
#include "hal_timers.h"
#include "str_helper.h"

//#include <HardwareSerial.h>
#include "hal_serial.h"

#include "dev_console.h"

#define __NOT_EXTERN__
#include "dev_comms.h"
#undef __NOT_EXTERN__

#ifdef CONSOLE_ENABLED
#include "dev_console.h"
#endif

#include "sys_utils.h"

#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif

#define PRINTF_TAG ("COMMS") /* This must be undefined at the end of the file*/

/******************************************************************************
Macros
******************************************************************************/
//#define I2C_BROADCAST_ADDR      (0x60)  /* b0110 xxxx */

/* Max number of bytes we can RX in the i2c in a single transaction */
#define COMMS_MSG_LEN_MAX     (sizeof(rgb_btn_tx_data_t))

/*
    A - 0x01
    B - 0x02
    C - 0x03
    D - 0x04
    E - 0x05
    F - 0x06
    G - 0x07
    H - 0x08
    I - 0x09
    J - 0x0A
    K - 0x0B
    L - 0x0C
    M - 0x0D
    N - 0x0E

*/
/******************************************************************************
Struct & Unions
******************************************************************************/
typedef enum e_comms_state
{
    comms_no_init,  //We've not been initialised
    comms_png,      // We've been initialised, but we don't know who we are yet
    comms_idle,     //WE are all ready to go
    comms_rxtx      //We are busy with a transaction
}comms_state_t;

typedef enum e_comms_msg_rx_state
{
    listening,     // Waiting for a message start (ETX)
    rx_busy,        // We are busy receiving a message (between ETX and STX)
    rx_escaping,    // The last byte was a DLE, so we need to check the next byte
    rx_done,
    rx_error,
}comms_msg_rx_state_t;

typedef enum e_comms_msg_tx_state
{
    idle,           // Waiting for a message to send
    tx_busy,        // We are busy transmitting a message (between ETX and STX)
    tx_check,       // Checking the RX'd msg to see if we had a collision
    tx_done,        // Checking the RX'd msg to see if we had a collision
    tx_error,
}comms_msg_tx_state_t;

typedef struct dev_comms_st
{
    comms_state_t state = comms_no_init;
    struct {
        rgb_btn_msg_t msg;
        struct {
            uint8_t sync     : 1;
            uint8_t overflow : 1;
            uint8_t crc      : 1;
            uint8_t version  : 1;
//            uint8_t reserved : 4;
        }err_flags;
        comms_msg_rx_state_t state = listening;
        int8_t length;
        uint8_t *cmd;
    }rx;
    struct {
        rgb_btn_msg_t msg;
        uint8_t buff[(RGB_BTN_MSG_MAX_LEN*2)+2];    //Absolute worst case scenario
        comms_msg_tx_state_t state = idle;
        uint8_t seq;
        uint8_t length;
    }tx;
    uint8_t addr;           /* My assigned address */
}dev_comms_t;

/******************************************************************************
Local function definitions
******************************************************************************/
void _dev_comms_tx_handler(void);
void _dev_comms_rx_handler(void);

char * _comms_rx_error_msg(int err_data);

comms_msg_rx_state_t _comms_check_rx_msg(int *_data);

/******************************************************************************
Local variables
******************************************************************************/

dev_comms_t _comms;

/******************************************************************************
Local functions
******************************************************************************/
char * _comms_rx_error_msg(int err_data)
{
    static char buff[32];
    buff[0] = 0;
    if (_comms.rx.err_flags.sync)
    {    
        snprintf(buff, sizeof(buff), "RX Sync Error (0x%02X)", err_data);
    }
    else if (_comms.rx.err_flags.overflow)
    {
        snprintf(buff, sizeof(buff), "RX Overflow (%d)", err_data);
    }
    else if (_comms.rx.err_flags.crc)
    {
        snprintf(buff, sizeof(buff), "CRC Failed (0x%02X)", err_data);
    }
    else if (_comms.rx.err_flags.version)
    {
        snprintf(buff, sizeof(buff), "Unkown Version (%d >= %d)", _comms.rx.msg.hdr.version, RGB_BTN_MSG_VERSION);
    }
    // else if (_comms.rx.err_flags.count)
    // {
    //     snprintf(buff, sizeof(buff), "Count Mismatch (%d >= %d)", _comms.rx.msg.hdr.n, _comms.rx.msg.hdr.total);
    // }
    else
    {    
        snprintf(buff, sizeof(buff), "None");
    }

    return buff;
}

// char * _comms_tx_error_msg(void)
// {
//     static char buff[32];
//     buff[0] = 0;
//     if (_comms.tx.flags.collision)
//     {    
//         snprintf(buff, sizeof(buff), "Collision");
//     }
//     else if (_comms.tx.flags.no_echo)
//     {
//         snprintf(buff, sizeof(buff), "No Echo (%dms)", (2*BUS_SILENCE_MIN_MS/5));
//     }
//     else
//     {    
//         snprintf(buff, sizeof(buff), "None");
//     }

//     return buff;
// }

comms_msg_rx_state_t _comms_check_rx_msg(int *_data)
{
    uint8_t crc = crc8_n(0, (uint8_t *)&_comms.rx.msg, _comms.rx.length);
    if (crc != 0)
    {
        //iprintln(trCOMMS, "#CRC Failed (%d) 0x%02X", _comms.rx.length, crc);
        _comms.rx.err_flags.crc = 1;
        *_data = crc;
        return rx_error;
    }

    if (_comms.rx.msg.hdr.version >= RGB_BTN_MSG_VERSION)
    {
        _comms.rx.err_flags.version = 1;
        *_data = _comms.rx.msg.hdr.version;
        return rx_error;
    }

    //CRC is good, Version is Good, Sync # is good - I guess we are done?

    //Was this the RX of the same message we were sending (and hoping to match perfectly)?
    if (_comms.tx.state == tx_check)
    {
        // if we were sending just now, we want to check if our rx and tx buffers match
        if (memcmp((uint8_t *)&_comms.rx.msg, (uint8_t *)&_comms.tx.msg, min(_comms.rx.length, RGB_BTN_MSG_MAX_LEN)) == 0)
        {
            //_comms.tx.flags.collision = 1;
            iprintln(trCOMMS, "#TX Error: %s (%d bytes)", "Collision", _comms.rx.length);
            iprintln(trCOMMS, "#TX %d bytes:", _comms.tx.length);
            console_print_ram(trCOMMS, &_comms.tx.msg, (unsigned long)&_comms.tx.msg, sizeof(rgb_btn_msg_t));
            iprintln(trCOMMS, "#RX %d bytes:", _comms.rx.length);
            console_print_ram(trCOMMS, &_comms.rx.msg, (unsigned long)&_comms.rx.msg, sizeof(rgb_btn_msg_t));
            _comms.tx.state = tx_error;
            //Could we have received a valid message while we were sending?
        }
        else
        {
            //RVN - This message was sent without any issues
            iprintln(trCOMMS, "#TX'd %d bytes successfully (RX'd %d bytes)", _comms.tx.length, _comms.rx.length);
            //_comms.tx.flags.done = 1;
            _comms.tx.state = tx_done;
        }
    }
    //This message is for us (or everyone)
    else if ((_comms.rx.msg.hdr.dst == _comms.addr) || (_comms.rx.msg.hdr.dst == RGB_BTN_ADDR_BROADCAST))
    {
        //WE might want to check if this message is:
        // 1. For us
        // 2. A broadcast
        // 3. A message for someone else (ignore)

        iprintln(trCOMMS, "#Received: %d bytes", _comms.rx.length);
        iprintln(trCOMMS, "#Data: ");
        console_print_ram(trCOMMS, &_comms.rx.msg, (unsigned long)&_comms.rx.msg, sizeof(rgb_btn_msg_t));

        //Set the cmd pointer to the start of the payload data (which should be the very first cmd)
        _comms.rx.cmd = &_comms.rx.msg.data[0];

        //Set the length to the size of the payload
        _comms.rx.length -= sizeof(rgb_btn_msg_hdr_t); //header
        _comms.rx.length -= sizeof(uint8_t); //CRC

        return rx_done;//Raise the flag for everyone to see
    }    

    return listening;
}

bool _dev_comms_bus_tx_ready(void)
{
    if ((hal_serial_rx_silence_ms() >= BUS_SILENCE_MIN_MS) && (_comms.tx.state == idle))
        return true;
    else
        return false;
}

void _dev_comms_rx_handler_add_data_check_overflow(uint8_t rx_data, int *err_data)
{
    //iprintln(trCOMMS, "#Got Byte (%d) 0x%02X", _comms.rx.length, rxData);
    if (_comms.rx.length < RGB_BTN_MSG_MAX_LEN)
    {
        ((uint8_t *)&_comms.rx.msg)[_comms.rx.length] = rx_data;
        _comms.rx.length++;
    }
    else
    {
        _comms.rx.err_flags.overflow = 1;
        *err_data = _comms.rx.length;
        _comms.rx.state = rx_error;
    }
}

void _dev_comms_rx_handler(void)
{
    int err_data;

    // read the incoming char:
    while (hal_serial_available() /*Serial.available()*/ > 0)
    {
        // read the incoming byte:
        uint8_t rx_data = (uint8_t)hal_serial_read();// Serial.read();

        //Regardless of the current state, an STX should restart a msg reception (we could have missed an ETX, or this message was interrupted by another device on the bus, but we will never know)
        if (rx_data == STX)
        {
            //iprintln(trCOMMS, "#Got STX");
            //Start again... no matter what!
            memset(&_comms.rx.msg, 0, sizeof(rgb_btn_msg_t));
            _comms.rx.length = 0;
            _comms.rx.state = rx_busy;
        }    
        else if (_comms.rx.state == listening)
        {
            //Not within STX-ETX, so pass this byte onto the console handler
            console_read_byte(rx_data);
        }
        else if (_comms.rx.state == rx_busy) 
        {
            if (rx_data == ETX)
            {
                _comms.rx.state = _comms_check_rx_msg(&err_data);
            }
            else if (rx_data == DLE)
            {
                //iprintln(trCOMMS, "#Got DLE (%d)", _comms.rx.length);
                _comms.rx.state = rx_escaping;
            }
            else
            {
                _dev_comms_rx_handler_add_data_check_overflow(rx_data, &err_data);
            }
        }
        else if (_comms.rx.state == rx_escaping)
        {
            //iprintln(trCOMMS, "#Got Escaped Byte (%d) 0x%02X", _comms.rx.length, rxData ^ DLE);
            if ((rx_data == ETX) || (rx_data == DLE))
            {
                _comms.rx.err_flags.sync = 1;
                err_data = rx_data;
                _comms.rx.state = rx_error;
            }
            else
            {
                _dev_comms_rx_handler_add_data_check_overflow(rx_data ^ DLE, &err_data);
            }
        }
        //else //(_comms.rx.state == rx_done) || (_comms.rx.state == rx_error)
        if (_comms.rx.state == rx_error)
        {
            iprintln(trCOMMS, "#RX Error: %s (%d bytes)", _comms_rx_error_msg(err_data), _comms.rx.length);
            iprintln(trCOMMS, "#Data: ");
            console_print_ram(trCOMMS, &_comms.rx.msg, (unsigned long)&_comms.rx.msg, sizeof(rgb_btn_msg_t));
            memset(&_comms.rx.msg, 0, sizeof(rgb_btn_msg_t));
            _comms.rx.state = listening;
        }
    }

}

void _dev_comms_tx_handler(void)
{
    switch (_comms.tx.state)
    {
        case idle:
            /* Do nothing */
            break;
        case tx_busy:
            if (hal_serial_rx_silence_ms() >= BUS_SILENCE_MIN_MS)
            {
                uint8_t * msg = (uint8_t *)&_comms.tx.msg;
                hal_serial_write(STX);
                for (uint8_t i = 0; i < sizeof(rgb_btn_msg_t); i++)
                {
                    uint8_t tx_data = msg[i];
                    if ((tx_data == STX) || (tx_data == DLE) || (tx_data == ETX))
                    {
                        hal_serial_write(DLE);
                        tx_data ^= DLE;
                    }
                    hal_serial_write(tx_data);
                }
                hal_serial_write(ETX);        
                //Right, we've sent the message, now we need to check if we received it correctly
                _comms.tx.state = tx_check;
            }
            break;
        case tx_check:
            //If we reach this point with certain amount of bus silence, say 10ms, in the TX_CHECK state, we can assume that the message was sent but not received for whatever reason (ETX missed?)
            if (hal_serial_rx_silence_ms() >= (2*BUS_SILENCE_MIN_MS/5) /* 10ms */)
            {
                //iprintln(trCOMMS, "#Bus Silence detected in TX_CHECK state");
                //_comms.tx.flags.no_echo = 1;
                iprintln(trCOMMS, "#TX Error: %s", "No Echo", 0);
                iprintln(trCOMMS, "#TX data:");
                console_print_ram(trCOMMS, &_comms.tx.msg, (unsigned long)&_comms.tx.msg, sizeof(rgb_btn_msg_t));
                // iprintln(trCOMMS, "#RX %d bytes:", _comms.rx.length);
                // console_print_ram(trCOMMS, &_comms.rx.msg, (unsigned long)&_comms.rx.msg, sizeof(rgb_btn_msg_t));
                _comms.tx.state = tx_error;
            }
            break;
        case tx_done:
        case tx_error:
            //We stay in this state until we get cleared by the caller
            //_comms.tx.state = idle;
            break;
        default:
            //We should never get here
            _comms.tx.state = tx_error;
            break;
    }
}

/******************************************************************************
Public functions
******************************************************************************/
void dev_comms_init(uint8_t addr)
{
    //First we need to set the I2C address
    // The top 4 bits will be fixed at 0000 0000 ( 0x00 )
    _comms.addr = addr;//RGB_BTN_I2C_TYPE_ID;

    memset(&_comms.rx.msg, 0, sizeof(rgb_btn_msg_t));
    _comms.rx.state = listening;
    _comms.tx.state = idle;

    hal_serial_init();

    console_init(hal_serial_write, hal_serial_flush);

    // void (*flush)(void);
    // size_t (*write)(uint8_t);
    // int (*available)(void);
    // int (*read)(void);

    //hal_serial_read, hal_serial_send, hal_serial_available, hal_serial_flush);

    //Just set a new address if we are already initialised
    if (_comms.state == comms_no_init) 
    {
        iprintln(trCOMMS, "#Initialised (address: 0x%02X)", _comms.addr);
    }
    else if (addr != _comms.addr)
    {
        if (addr != RGB_BTN_ADDR_MASTER) //Cannot set the address to that of the master
            iprintln(trCOMMS, "#Address updated: 0x%02X", _comms.addr);
        else
            iprintln(trCOMMS, "#Invalid Address: 0x%02X (Keeping 0x%02X)", addr, _comms.addr);

    }
    _comms.state = (addr == RGB_BTN_ADDR_MASTER)? comms_png : comms_idle;

}

void dev_comms_response_start(void)
{
    _comms.tx.length = sizeof(rgb_btn_msg_hdr_t);

    _comms.tx.msg.hdr.version = RGB_BTN_MSG_VERSION;

    _comms.tx.msg.hdr.src = _comms.addr;            //Our Address
    _comms.tx.msg.hdr.dst = RGB_BTN_ADDR_MASTER;    //We only ever talk to the master!!!!!
}

unsigned int dev_comms_response_add_data(rgb_btn_command_t cmd, uint8_t * data, uint8_t data_len)
{
    //Make sure we don't overflow the buffer and leave space for 1 byte of CRC
    if (data_len > (RGB_BTN_MSG_MAX_LEN - sizeof(uint8_t) - _comms.tx.length))
        return 0;

    uint8_t * msg_data = ((uint8_t *)&_comms.tx.msg) + _comms.tx.length;
    
    *msg_data = cmd;
    msg_data++;
    memcpy(msg_data, data, data_len);

    _comms.tx.length += data_len + 1; // +1 for the command byte

    return data_len; // The number of bytes added
}

void dev_comms_response_send()
{
    //We need 1 byte for a CRC
    if (_comms.tx.length > (RGB_BTN_MSG_MAX_LEN - sizeof(uint8_t)))
    {
        // This is a complete mess.... how did this happen?
        //RVN - TODO - Error!!!!
        _comms.tx.state = tx_error;
        return;

    }

    uint8_t * _msg = ((uint8_t *)&_comms.tx.msg);
    uint8_t * _crc = _msg + _comms.tx.length;

    // Update the sequence number only if we are sending
    if ((++_comms.tx.seq) == 0)  //0 is not a valid ID #, so we skip it
        _comms.tx.seq++;
    _comms.tx.msg.hdr.id = _comms.tx.seq;

    //We add a crc at the end of the message data
    *_crc = crc8_n(0, _msg, _comms.tx.length++);

    //Setting the state to busy will queue the message (even if the bus is not ready yet) then once it is ready, the message is sent from the service() method
    _comms.tx.state = tx_busy;
}

int8_t dev_comms_msg_send_status(void)
{
    int8_t ret = 0;
    if ((_comms.tx.state == tx_busy) || (_comms.tx.state == tx_check))  //Busy!
        ret = -1;

    else if (_comms.tx.state == tx_error)   // Bus Fault
        ret = 1;

    else //(_comms.tx.state == tx_done) //We are done
        ret = 0;

    _comms.tx.state = idle;
    return ret;
}

uint8_t dev_comms_cmd_available(uint8_t *cmd)
{
    //A message is only available while the rx_done state is active
    if (_comms.rx.state != rx_done)
        return 0;

    //Make sure our cmd pointer is valid is still with the bounds of the message payload
    if (_comms.rx.length <= 0)
        return 0;

    *cmd = *_comms.rx.cmd;
    //_comms.rx.cmd++;

    return 1;
}

uint8_t dev_comms_cmd_read(uint8_t * dst)
{
    if (_comms.rx.state != rx_done)
        return 0;

    uint8_t cmd = *_comms.rx.cmd;
    int8_t len = 0;

    if (cmd ==  cmd_wr_console)
        len = _comms.rx.length;
    else if (cmd == cmd_set_blink)
        len = sizeof(uint32_t);
    else if ((cmd == cmd_set_rgb_0) || (cmd == cmd_set_rgb_1))
        len = 3*sizeof(uint8_t);
    else
        len = 0;

    //Move to the start of the command data
    _comms.rx.cmd++;

    if (dst) // Don't copy to NULL
        memcpy(dst, _comms.rx.cmd, len);
    
    _comms.rx.cmd += len;

    return len;
}

bool dev_comms_verify_addr(uint8_t addr)
{
    //Cannot set the address to that of the master
    return (addr != RGB_BTN_ADDR_MASTER);
}

void dev_comms_reset_addr(uint8_t addr)
{
    if (dev_comms_verify_addr(addr))
    {
        _comms.addr = addr;        
    }
    else
    {
        iprintln(trCOMMS, "#Invalid Address: 0x%02X (Keeping 0x%02X)", addr, _comms.addr);
    }
}

void dev_comms_service(void)
{
    _dev_comms_tx_handler();

    _dev_comms_rx_handler();
}

#undef PRINTF_TAG
/*************************** END OF FILE *************************************/
