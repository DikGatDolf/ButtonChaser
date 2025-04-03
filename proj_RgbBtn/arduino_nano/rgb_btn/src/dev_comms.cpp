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



This is the format of the transmission protocol:

    [STX][PAYLOAD][ETX]

Occurrences of [STX], [ETX], and [DLE] within [Payload] data is escaped with DLE, etc
    [STX] => [DLE][DLE ^ STX]
    [ETX] => [DLE][DLE ^ ETX]
    [DLE] => [DLE][DLE ^ DLE]


The once all escaping [STX, [DLE], and [ETX] has been processed, the payload will contain a single message:
    [Header]
    [Data]
    [CRC]

The [Header] section of the message contains the following fields:
    [Version]  - The current version of the message format
    [ID]       - Unique ID/Sequence number of this message (to help with sync and collision detection)
    [SRC]      - The source address of the message
    [DST]      - The destination address of the message

The [Data] section of the message contains 1 or more sequences of commands, each followed by a variable length payload, e.g.:
    [Cmd 1][Data 1]
    [Cmd 2][Data 2]
    ...
    [Cmd N][Data N]
    
    The command is always 1 byte
    The length of each data block is determined by the command itself, and can vary from 0 to the remainder of the message

The [CRC] is calculated of the entire message, excluding the [STX] and [ETX] bytes. This way, to verify if the message is valid, we can
just calculate the CRC of the entire message (including the received CRC).... if it is valid, the result should be 0

Reception State Machine:
+---------------+-----------------------------------------------------------------------+
| State         | Description                                                           |
+---------------+-----------------------------------------------------------------------+
|  rx_listen    | Waiting for a message start (STX)                                     |
|               |   - All data received outside of [STX] and [ETX] is ignored           |
|  rx_busy      | Rx'd a STX, busy receiving a message, saving data                     |
|  rx_escaping  | Rx'd a DLE, will xor the next byte with DLE and save the data         |
|  rx_done      | Rx'd a ETX, msg is OK, CRC is valid, ver # is known                   |
|               |   - The state machine will remain in the rx_done state until we rx    |
|               |     the next STX byte (at which point the message data will be        |
|               |     overwritten by the incoming data) or all the payload data                                  |
|  rx_error     | An error occurred during reception:                                   |
|               |   - Buffer Overflowed                                                 |
|               |   - Received an unescaped ETX or DLE after a DLE                      |
|               |   - CRC Error                                                         |
|               |   - Version Error                                                     |        
+---------------+-----------------------------------------------------------------------+

Transmission State Machine:
+---------------+-----------------------------------------------------------------------+
| State         | Description                                                           |
+---------------+-----------------------------------------------------------------------+
| tx_idle       | Waiting for a transmission to start                                   |
| tx_busy       | A transmission has been queued, so we waiting for the bus to be free  |
|               |   - The transmission will be started when the bus is free             |
| tx_busycheck  | A transmission has been started, we are now waiting for the message   |
|               |   to be reflected back to us.                                         |
| tx_done       | The transmission is done, we have received the reflected message      |
|               |   perfectly.                                                          |       
| tx_error      | An error occurred during transmission:                                |
|               |   - The reflected message did not match our transmission              |
|               |   - We did not get a reflected message back (timout)                  |
+---------------+-----------------------------------------------------------------------+





    The reception of a message is done in the following way:
    1) Wait for the [STX] byte
    2) Start receiving the message until we receive the [ETX] byte
    3) Check if the message is valid (CRC, etc)
    4) If valid, process the message
    5) If not valid, discard the message and wait for the next [STX] byte

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

/******************************************************************************
Struct & Unions
******************************************************************************/
typedef enum e_comms_msg_rx_state
{
    rx_listen,      // Waiting for a message start (ETX)
    rx_busy,        // We are busy receiving a message (between ETX and STX)
    rx_escaping,    // The last byte was a DLE, so we need to xor the next byte with DLE
    // rx_done,        // RVN - TODO... this can be done away with
    // rx_error,       // RVN - TODO... this can be done away with if the available returns negative numbers
}comms_msg_rx_state_t;

typedef enum e_comms_msg_rx_errors
{
    rx_err_sync     = (-1),
    rx_err_ovflw    = (-2),
    rx_err_crc      = (-3),
    rx_err_version  = (-4),
}comms_msg_rx_error_t;

typedef enum e_comms_msg_tx_state
{
    tx_idle,        // Waiting for a message to send
    tx_busy,        // We are busy transmitting a message (between ETX and STX)
    tx_busycheck,   // Checking the RX'd msg to see if we had a collision
    tx_done,        // Transmission is done, our reflected message is all good.
    tx_error,
}comms_msg_tx_state_t;

typedef struct dev_comms_blacklist_st
{
    uint8_t addr[RGB_BTN_MAX_NODES];
    uint8_t cnt;
}dev_comms_blacklist_t;

typedef struct dev_comms_st
{
    bool init_done = false;
    struct {
        comms_msg_t msg;
        struct {
            uint8_t msg_ready       : 1;
            // uint8_t err_sync        : 1;
            // uint8_t err_ovflw       : 1;
            // uint8_t err_crc         : 1;
            // uint8_t err_version     : 1;
//            uint8_t reserved : 4;
        }flag;
        //comms_msg_rx_state_t state = rx_listen;
        int8_t length;
        int8_t data_length;
        int8_t data_rd_index;
    }rx;
    struct {
        comms_msg_t msg;
        uint8_t buff[(RGB_BTN_MSG_MAX_LEN*2)+2];    //Absolute worst case scenario
        comms_msg_tx_state_t state = tx_idle;
        uint8_t seq;
        uint8_t length;
        struct {
            uint8_t err_timeout     : 1;
            uint8_t err_mismatch    : 1;
            uint8_t msg_started     : 1;
            uint8_t msg_queued      : 1;
//            uint8_t reserved : 4;
        }flag;
    }tx;
    uint8_t addr;           /* My assigned address */
    dev_comms_blacklist_t blacklist; /* List of addresses that are not allowed to be used */
}dev_comms_t;

/******************************************************************************
Local function definitions
******************************************************************************/
char * _comms_rx_error_msg(int err_data);
int8_t _comms_check_rx_msg(int *_data);
bool _dev_comms_verify_addr(uint8_t addr);
void _dev_comms_tx_start(void);
int8_t _dev_comms_rx_handler_add_data_check_overflow(uint8_t rx_data, int *err_data);

/******************************************************************************
Local variables
******************************************************************************/

dev_comms_t _comms;

/******************************************************************************
Local functions
******************************************************************************/
char * _comms_rx_error_msg(int8_t code, int err_data)
{
    static char buff[32];
    buff[0] = 0;
    if (code == rx_err_sync)
    {    
        snprintf(buff, sizeof(buff), "RX Sync Error (0x%02X)", err_data);
    }
    else if (code == rx_err_ovflw)
    {
        snprintf(buff, sizeof(buff), "RX Overflow (%d)", err_data);
    }
    else if (code == rx_err_crc)
    {
        snprintf(buff, sizeof(buff), "CRC Failed (0x%02X)", err_data);
    }
    else if (code == rx_err_version)
    {
        snprintf(buff, sizeof(buff), "Unkown Version (%d >= %d)", _comms.rx.msg.hdr.version, RGB_BTN_MSG_VERSION);
    }
    else
    {    
        snprintf(buff, sizeof(buff), "Unkown Error %d (0x%02X)", code, err_data);
    }

    return buff;
}

int8_t _comms_check_rx_msg(int *_data)
{
    uint8_t crc = crc8_n(0, (uint8_t *)&_comms.rx.msg, _comms.rx.length);
    if (crc != 0)
    {
        // _comms.rx.flag.err_crc = 1;
        *_data = crc;
        return rx_err_crc;
    }

    if (_comms.rx.msg.hdr.version > RGB_BTN_MSG_VERSION)
    {
        // _comms.rx.flag.err_version = 1;
        *_data = _comms.rx.msg.hdr.version;
        return rx_err_version;
    }

    //CRC is good, Version is Good, Sync # is good - I guess we are done?

    //Was this the RX of the same message we were sending (and hoping to match perfectly)?
    if (_comms.tx.state == tx_busycheck)
    {
        // if we were sending just now, we want to check if our rx and tx buffers match
        if (memcmp((uint8_t *)&_comms.rx.msg, (uint8_t *)&_comms.tx.msg, min(_comms.rx.length, RGB_BTN_MSG_MAX_LEN)) == 0)
        {
            //_comms.tx.flags.collision = 1;
            iprintln(trCOMMS, "#TX Error: %s (%d bytes)", "Collision", _comms.rx.length);
            iprintln(trCOMMS, "#TX %d bytes:", _comms.tx.length);
            console_print_ram(trCOMMS, &_comms.tx.msg, (unsigned long)&_comms.tx.msg, _comms.tx.length);//sizeof(comms_msg_t));
            iprintln(trCOMMS, "#RX %d bytes:", _comms.rx.length);
            console_print_ram(trCOMMS, &_comms.rx.msg, (unsigned long)&_comms.rx.msg, _comms.rx.length);//sizeof(comms_msg_t));
            _comms.tx.state = tx_error;
            //Could we have received a valid message while we were sending?
        }
        else
        {
            //RVN - This message was sent without any issues
            //_comms.tx.flags.done = 1;
            iprintln(trCOMMS, "#TX'd %d bytes successfully (RX'd %d bytes)", _comms.tx.length, _comms.rx.length);

            _comms.tx.state = tx_done;
        }
    }
    //This message is for us (or everyone)
    //RVN - TODO, we are going to experiment with the slaves "snooping" the bus to check for potential conflicting addresses
    // else if ((_comms.rx.msg.hdr.dst == _comms.addr) || (_comms.rx.msg.hdr.dst == ADDR_BROADCAST))
    // {
    //     //WE might want to check if this message is:
    //     // 1. For us
    //     // 2. A broadcast
    //     // 3. A message for someone else (ignore)

    //     iprintln(trCOMMS, "#Received: %d bytes", _comms.rx.length);
    //     iprintln(trCOMMS, "#Data: ");
    //     console_print_ram(trCOMMS, &_comms.rx.msg, (unsigned long)&_comms.rx.msg, sizeof(comms_msg_t));

    //     //Set the rd pointer to the start of the payload data (which should be the very first cmd)
    //     _comms.rx.data_rd_index = 0;

    //     //Set the length to the size of the payload
    //     _comms.rx.data_length = _comms.rx.length - sizeof(comms_msg_hdr_t) - sizeof(uint8_t); //CRC

    //     return (int8_t)_comms.rx.data_length;//rx_done;//Raise the flag for everyone to see
    // }    
    //return 0;

    //Set the rd pointer to the start of the payload data (which should be the very first cmd)
    //Set the length to the size of the payload
    _comms.rx.data_length = _comms.rx.length - sizeof(comms_msg_hdr_t) - sizeof(uint8_t); //CRC
    _comms.rx.data_rd_index = 0;

    //Set the msg_ready flag to indicate that we have a message ready to be processed
    if (_comms.rx.data_length > 0)
        _comms.rx.flag.msg_ready = 1;

    // Our Payload data could be 0.... not much to do then?
    return _comms.rx.data_length;
}

int8_t _dev_comms_rx_handler_add_data_check_overflow(uint8_t rx_data, int *err_data)
{
    //iprintln(trCOMMS, "#Got 0x%02X (%d)",rx_data, _comms.rx.length);

    if (_comms.rx.length >= RGB_BTN_MSG_MAX_LEN)
    {
        // _comms.rx.flag.err_ovflw = 1;
        *err_data = _comms.rx.length;
        return rx_err_ovflw;
    }

    //If we had a message ready, it is goneskies now!
    _comms.rx.flag.msg_ready = 0;
    ((uint8_t *)&_comms.rx.msg)[_comms.rx.length] = rx_data;
    _comms.rx.length++;

    return 0;
}

/*! \brief Writes the data from the tx msg buffer to the serial port (including escaping)
 * \return None
 */
void _dev_comms_tx_start(void)
{
    if ((_comms.tx.flag.msg_started == 0) || (_comms.tx.length == 0))
    {
        //How did we get to this point?
        _comms.tx.state = tx_idle;
        iprintln(trCOMMS, "#No Msg data to Tx (%d bytes)", _comms.tx.length);
    }
    else
    {
        iprintln(trCOMMS, "#Msg Tx %d bytes (Seq # %d)", _comms.tx.length, _comms.tx.msg.hdr.id);

        //Make sure any console prints are finished before we start sending... 
        // this ensures that the RS-485 is enabled again.
        hal_serial_flush(); 

        //NO PRINT SECTION START - Make sure there are NO console prints in this section

        uint8_t * msg = (uint8_t *)&_comms.tx.msg;
        hal_serial_write(STX);
        for (uint8_t i = 0; i < min(_comms.tx.length, sizeof(comms_msg_t)); i++)
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
        _comms.tx.state = tx_busycheck;
        
        //Make sure the entire message is sent over RS285 before we continue (with potential printf's)
        hal_serial_flush(); 

        //NO PRINT SECTION END

        iprintln(trCOMMS, "#Msg Tx Complete");
    }

    _comms.tx.flag.msg_started = 0; //We are done with this message, so we can reset the flag which protects the msg buffer
}

bool _dev_comms_verify_addr(uint8_t addr)
{
    if (addr == ADDR_MASTER)
        return false; //Cannot set the address to that of the master

    if (addr == ADDR_BROADCAST)
        return false; //Cannot set the address to that of the broadcast
    
    // if (addr == COMMS_ADDR_SLAVE_DEFAULT)
    //     return false; //Cannot set the address to that of the default slave address

    //Check if the address is already in use
    for (uint8_t i = 0; i < _comms.blacklist.cnt; i++)
    {
        if (_comms.blacklist.addr[i] == addr) //iprintln(trCOMMS, "#Address %d is blacklisted", addr);
            return false;
    }
    
    return true;
}

/******************************************************************************
Public functions
******************************************************************************/
void dev_comms_init(void)
{
    if (_comms.init_done)
        return; //Already initialised

    memset(&_comms.rx.msg, 0, sizeof(comms_msg_t));
    //_comms.rx.state = rx_listen;
    _comms.tx.state = tx_idle;

    memset(&_comms.blacklist, 0, sizeof(dev_comms_blacklist_t));
    
    //Start with a random sequence number (to help detect bus collisions in the case of 2 nodes with the same address)
    _comms.tx.seq = (uint8_t)sys_random(0, ADDR_BROADCAST);

    hal_serial_init();

    console_init(hal_serial_write, hal_serial_flush, hal_serial_rs485_disable);

    //Generate a new random (and potentially, only temporary) address
    _comms.addr = dev_comms_addr_new();
    _comms.init_done = true;
    _comms.tx.flag.msg_started = 0;

    iprintln(trCOMMS, "#Initialised - Payload size: %d/%d (Seq # %d)", sizeof(_comms.rx.msg.data), sizeof(comms_msg_t), _comms.tx.seq);
}

unsigned int dev_comms_response_add_data(master_command_t cmd, uint8_t * data, uint8_t data_len, bool clear_previous_data)
{
    // We cannot send a message if we are not ready to send now, otherwise we will overwrite data being sent
    if ((_comms.tx.state == tx_busy) || (_comms.tx.state == tx_busycheck) ||        //Make sure we do not overwrite the data in the msg buffer
        (data_len > (RGB_BTN_MSG_MAX_LEN - sizeof(uint8_t) - _comms.tx.length)))        //Make sure we don't overflow the buffer and leave space for 1 byte of CRC
    {
        return 0;
    }

    //is this the first addition to the message?
    if ((!_comms.tx.flag.msg_started == 0) || (clear_previous_data))
    {    
        _comms.tx.length = sizeof(comms_msg_hdr_t);         //Reset to the beginning of the data
        _comms.tx.msg.hdr.version = RGB_BTN_MSG_VERSION;    //Superfluous, but just in case
        _comms.tx.msg.hdr.id = _comms.tx.seq;               //Should have incremented after the last transmission
        _comms.tx.msg.hdr.src = _comms.addr;                //Our Address (might have changed since our last message)
        _comms.tx.msg.hdr.dst = ADDR_MASTER;          //We only ever talk to the master!!!!!
    }
   
    ((uint8_t *)&_comms.tx.msg)[_comms.tx.length++] = cmd; //Add the command to the message
    if ((data != NULL) && (data_len > 0))
    {
        
        memcpy(&((uint8_t *)&_comms.tx.msg)[_comms.tx.length], data, data_len);
        _comms.tx.length += data_len; // for the payload
    }

    //(Re)calculate the crc of the message in its current format
    ((uint8_t *)&_comms.tx.msg)[_comms.tx.length] = crc8_n(0, ((uint8_t *)&_comms.tx.msg), _comms.tx.length);

    //Do NOT increment the length here,... in case more data gets added, the CRC should be overwritten 
    // and then re-calculated at the end of the newly added data

    //Make sure this flag is set, so we don't reset it again
    _comms.tx.flag.msg_started = 1;

    return data_len; // The number of bytes added
}

int8_t dev_comms_response_send(bool queue_if_busy)
{
    if (_comms.tx.length <= (sizeof(comms_msg_hdr_t )))
        return 1; //Nothing to send, let the user think everything is all good

    // We cannot send a message if we are not ready to send now, otherwise we will overwrite data being sent
    if ((_comms.tx.state == tx_busy) || (_comms.tx.state == tx_busycheck))
        return -1;

    //The CRC is already added to the message length, so we need to make sure we don't overflow the buffer
    if (_comms.tx.length > (RGB_BTN_MSG_MAX_LEN - sizeof(uint8_t)))
    {
        // This is a complete mess.... how did this happen?
        //RVN - TODO - Error!!!!
        return -2;
    }

    _comms.tx.length++; //For the CRC

    //If the bus is available, we can send the message now
    if (hal_serial_rx_silence_ms() >= BUS_SILENCE_MIN_MS)
    {
        _dev_comms_tx_start(); //    sets _comms.tx.state = tx_busycheck;
        return 1;
    }
    
    if (queue_if_busy)
    {
        //We are not ready to send yet, but we are OK to queue, so we will queue the message
        _comms.tx.state = tx_busy;
        _comms.tx.flag.msg_queued = 1;
        iprintln(trCOMMS, "#Msg Q'd %d bytes (Seq # %d)", _comms.tx.length, _comms.tx.msg.hdr.id);
        return 2;
    }

    // We could not send it and we are not allowed to queue, so let the user know he needs to retry
    return 0;

}

void dev_comms_tx_service(void)
{
    switch (_comms.tx.state)
    {
        case tx_idle:
            /* Do nothing */
            break;
        case tx_busy:
            if (hal_serial_rx_silence_ms() >= BUS_SILENCE_MIN_MS)
                _dev_comms_tx_start();
            break;
        case tx_busycheck:
            //RVN - TODO- Confirm these times!
            //If we reach this point with certain amount of bus silence, say 10ms, in the TX_CHECK state, we can assume that the message was sent but not received for whatever reason (ETX missed?)
            if (hal_serial_rx_silence_ms() >= (2*BUS_SILENCE_MIN_MS/5) /* 10ms */)
            {
                //iprintln(trCOMMS, "#Bus Silence detected in TX_CHECK state");
                //_comms.tx.flags.no_echo = 1;
                iprintln(trCOMMS, "#TX Error: %s (%d bytes)", "No Echo", _comms.rx.length);
                iprintln(trCOMMS, "#TX data:");
                console_print_ram(trCOMMS, &_comms.tx.msg, (unsigned long)&_comms.tx.msg, _comms.tx.length);//min(_comms.tx.length, sizeof(comms_msg_t)));
                // iprintln(trCOMMS, "#RX %d bytes:", _comms.rx.length);
                // console_print_ram(trCOMMS, &_comms.rx.msg, (unsigned long)&_comms.rx.msg, sizeof(comms_msg_t));
                _comms.tx.state = tx_error;
            }
            break;
        case tx_done:
        case tx_error:
            //We stay in this state until we get cleared by the caller
            //_comms.tx.state = tx_idle;
            break;
        default:
            //We should never get here
            _comms.tx.state = tx_error;
            break;
    }
}

int8_t dev_comms_tx_ready(void)
{
    int8_t ret = 0;
    if ((_comms.tx.state == tx_busy) || (_comms.tx.state == tx_busycheck) || (hal_serial_rx_silence_ms() < BUS_SILENCE_MIN_MS))
        return 0;

    //Otherwise we are either in a idle, error or done state.... either way, we need to clear the state and prepare ourselves for the next transmission

    //Update the Seq # only AFTER a successful transmission
    if (_comms.tx.state == tx_done)// || (_comms.tx.state == tx_error)) //We are done
        _comms.tx.seq++;

    //We return -1 for a fault, because technically the bus is available again (i.e. not busy), but we might want to resend the last message?
    ret = (_comms.tx.state == tx_error)? -1 : 1;

    _comms.tx.state = tx_idle;
    return ret;
}

uint8_t dev_comms_read_payload(uint8_t * dst, uint8_t len)
{
    if (!_comms.rx.flag.msg_ready)
        return 0;

    if (_comms.rx.data_rd_index >= _comms.rx.data_length)
        return 0; //No data available

    if (len < (_comms.rx.data_length - _comms.rx.data_rd_index))
        len = min(_comms.rx.data_length - _comms.rx.data_rd_index, 0);

    if ((dst) && (len > 0)) // Don't copy to NULL
        memcpy(dst, &_comms.rx.msg.data[_comms.rx.data_rd_index], len);
    
    _comms.rx.data_rd_index += len;

    //Clear the msg_ready flag if we are done with the message
    if (_comms.rx.data_rd_index >= _comms.rx.data_length)
        _comms.rx.flag.msg_ready = 0; //We are done with this message, so we can reset the flag which protects the msg buffer

    return len;
}

uint8_t dev_comms_addr_get(void)
{
    return _comms.addr;
}

bool dev_comms_addr_set(uint8_t addr)
{
    if (!_comms.init_done)
        return false;

    //Is the provided address valid?
    if (_dev_comms_verify_addr(addr))
    {
        //All good.... let's rather use this one.
        _comms.addr = addr;
        iprintln(trCOMMS, "#Address Set: 0x%02X", _comms.addr);
        //This also means we are now initialised
        _comms.init_done = true;
        return true;
    }
    iprintln(trCOMMS, "#Invalid Address: 0x%02X (Keeping 0x%02X)", addr, _comms.addr);
    return false;
}

void dev_comms_addr_blacklist(uint8_t new_addr)
{
    for (int i = 0; i < _comms.blacklist.cnt; i++)
    {
        if (_comms.blacklist.addr[i] == new_addr)
            return; //Already in the list
    }

    if (_comms.blacklist.cnt >= RGB_BTN_MAX_NODES)
    {   
        // Drop the "oldest address"
        iprintln(trCOMMS, "#Blacklist - Drop 0x%02X", _comms.blacklist.addr[0]);
        memmove(&_comms.blacklist.addr[0], &_comms.blacklist.addr[1], (RGB_BTN_MAX_NODES-1));
        _comms.blacklist.cnt = (RGB_BTN_MAX_NODES-1);
    }

    //Now add the new address to the end of the list
    _comms.blacklist.addr[_comms.blacklist.cnt] = new_addr;
    _comms.blacklist.cnt++;    

    iprintln(trCOMMS, "#Blacklist - Add 0x%02X (%d)", new_addr, _comms.blacklist.cnt);
}

uint8_t dev_comms_addr_new(void)
{
    uint8_t new_addr;
    int retries = 0;

    do
    {
        //We will be stuck here until we get a valid address
        new_addr = (uint8_t)sys_random(ADDR_SLAVE_MIN, ADDR_SLAVE_MAX);
        retries++;
    }while (!_dev_comms_verify_addr(new_addr));

    iprintln(trCOMMS, "#New Address: 0x%02X (%d runs)", new_addr, retries);

    _comms.addr = new_addr;
    return _comms.addr;
}

uint8_t dev_comms_rx_msg_src_addr()
{
    return _comms.rx.msg.hdr.src;
}

uint8_t dev_comms_rx_msg_dst_addr()
{
    return _comms.rx.msg.hdr.dst;
}

uint8_t dev_comms_rx_msg_data_len()
{
    if (!_comms.rx.flag.msg_ready)
        return 0;
    return (_comms.rx.data_length - _comms.rx.data_rd_index);
}

int8_t dev_comms_rx_msg_available(void)
{
    static comms_msg_rx_state_t rx_state = rx_listen;
    int8_t ret_val = 0;
    int err_data;


    if (!_comms.init_done)
        return 0;

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
            memset(&_comms.rx.msg, 0, sizeof(comms_msg_t));
            _comms.rx.length = 0;
            rx_state = rx_busy;
        }    
        else if (rx_state == rx_listen)
        {
            //Not within STX-ETX, so pass this byte onto the console handler
            console_read_byte(rx_data);
        }
        else if (rx_state == rx_busy) 
        {
            if (rx_data == ETX)
            {
                //iprintln(trCOMMS, "#Got ETX (%d)", _comms.rx.length);
                ret_val = _comms_check_rx_msg(&err_data);
                //iprintln(trCOMMS, "#rx_msg_check() = %d", ret_val);
                rx_state = rx_listen;
                break; //from the while loop to process the received message
            }
            else if (rx_data == DLE)
            {
                //iprintln(trCOMMS, "#Got DLE (%d)", _comms.rx.length);
                rx_state = rx_escaping;
            }
            else
            {
                ret_val = _dev_comms_rx_handler_add_data_check_overflow(rx_data, &err_data);
            }
        }
        else if (rx_state == rx_escaping)
        {
            //iprintln(trCOMMS, "#Got Escaped Byte (%d) 0x%02X", _comms.rx.length, rxData ^ DLE);
            if ((rx_data == ETX) || (rx_data == DLE))
            {
                // _comms.rx.flag.err_sync = 1;
                err_data = rx_data;
                ret_val = rx_err_sync;
            }
            else
            {
                ret_val = _dev_comms_rx_handler_add_data_check_overflow(rx_data ^ DLE, &err_data);
            }
        }
        //If we got a completed message or encountered an error along the way, go back to a listen state
        if (ret_val != 0)
        {
            rx_state = rx_listen;
            break; //from the while loop to process the received message (or handle the error)
        }
    }

    if (ret_val < 0)
    {
        iprintln(trCOMMS, "#RX Error: %s (%d bytes)", _comms_rx_error_msg(ret_val, err_data), _comms.rx.length);
        iprintln(trCOMMS, "#Data: ");
        console_print_ram(trCOMMS, &_comms.rx.msg, (unsigned long)&_comms.rx.msg, sizeof(comms_msg_t));
        memset(&_comms.rx.msg, 0, sizeof(comms_msg_t));
    }

    return ret_val;
}

#undef PRINTF_TAG
/*************************** END OF FILE *************************************/
