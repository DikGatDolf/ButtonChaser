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
+-------------------+-----------------------------------------------------------------------+
| State             | Description                                                           |
+-------------------+-----------------------------------------------------------------------+
|  rx_listen        | Waiting for a message start (STX)                                     |
|                   |   - All data received outside of [STX] and [ETX] is ignored           |
|  rx_busy          | Rx'd a STX, busy receiving a message, saving data                     |
|  rx_escaping      | Rx'd a DLE, will xor the next byte with DLE and save the data         |
|                   |     Msg's received while busy transmitting (echo) will be checked to  |
|                   |      confirm that the transmission was done correctly, but will       |
|                   |      subsequently be discarded.                                       |
+-------------------+-----------------------------------------------------------------------+

Transmission State Machine:
+---------------+-----------------------------------------------------------------------+
| State         | Description                                                           |
+---------------+-----------------------------------------------------------------------+
| tx_idle       | Waiting for a transmission to start                                   |
| tx_queued     | A transmission has been queued, so we waiting for the bus to be free  |
|               |   - The transmission will be started when the bus is free             |
| tx_echo_rx  | A transmission has been started, we are now waiting for the message   |
|               |   to be reflected back to us.                                         |
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

#include "dev_rgb.h"

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
    rx_listen,          // Waiting for a message start (ETX)
    rx_busy,            // We are busy receiving a message (between ETX and STX)
    rx_escaping,        // The last byte was a DLE, so we need to xor the next byte with DLE
}comms_msg_rx_state_t;

typedef enum e_comms_msg_rx_errors
{
    rx_err_crc      = (-1),
    rx_err_version  = (-2),
}comms_msg_rx_error_t;

typedef enum e_comms_msg_tx_state
{
    tx_idle,        // Waiting for a message to send
    tx_msg_busy,    // We have started to build a message
    tx_queued,      // We are busy transmitting a message (between ETX and STX)
    tx_echo_rx,     // Checking the RX'd msg to see if we had a collision
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
        int8_t length;
        int8_t data_length;
        // int8_t data_rd_index;
    }rx;
    struct {
        comms_msg_t msg;
        uint8_t buff[(RGB_BTN_MSG_MAX_LEN*2)+2];    //Absolute worst case scenario
        // comms_msg_tx_state_t state = tx_idle;
        uint8_t retry_cnt = 0;
        uint8_t seq;
        uint8_t length;
    }tx;
    uint8_t addr;           /* My assigned address */
    dev_comms_blacklist_t blacklist; /* List of addresses that are not allowed to be used */
}dev_comms_t;

/******************************************************************************
Local function definitions
******************************************************************************/
void _rx_irq_callback(uint8_t rx_data);
//char * _comms_rx_error_msg(int err_data);
int8_t _comms_check_rx_msg(int *_data);
bool _dev_comms_verify_addr(uint8_t addr);
void _dev_comms_tx_start(void);
bool _dev_comms_rx_handler_add_data_check_overflow(uint8_t rx_data);
unsigned int  _dev_comms_response_add_console_resp(uint8_t * data, uint8_t data_len);
unsigned int _dev_comms_response_add_data(uint8_t crc, uint8_t start_index, uint8_t * data, uint8_t data_len);

/******************************************************************************
Local variables
******************************************************************************/

dev_comms_t _comms;
volatile comms_msg_rx_state_t _rx_state = rx_listen;
volatile comms_msg_tx_state_t _tx_state = tx_idle;

volatile bool _msg_available = false;

stopwatch_ms_s _tx_sw;
timer_ms_t _tx_tmr; //The time we have to wait for the message to be sent

// volatile bool tx_success = false;
// volatile bool tx_error = false;
/******************************************************************************
Local functions
******************************************************************************/
// char * _comms_rx_error_msg(int8_t code, int err_data)
// {
//     static char buff[32];
//     buff[0] = 0;
//     // if (code == rx_err_sync)
//     // {    
//     //     snprintf(buff, sizeof(buff), "RX Sync Error (0x%02X)", err_data);
//     // }
//     // else if (code == rx_err_ovflw)
//     // {
//     //     snprintf(buff, sizeof(buff), "RX Overflow (%d)", err_data);
//     // }
//     // else 
//     if (code == rx_err_crc)
//     {
//         snprintf(buff, sizeof(buff), "CRC Failed (0x%02X)", err_data);
//     }
//     else if (code == rx_err_version)
//     {
//         snprintf(buff, sizeof(buff), "Unkown Version (%d >= %d)", _comms.rx.msg.hdr.version, RGB_BTN_MSG_VERSION);
//     }
//     else
//     {    
//         snprintf(buff, sizeof(buff), "Unkown Error %d (0x%02X)", code, err_data);
//     }

//     return buff;
// }

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

    //Set the length to the size of the payload
    _comms.rx.data_length = _comms.rx.length - sizeof(comms_msg_hdr_t) - sizeof(uint8_t); //CRC

    // Our Payload data could be 0.... not much to do then?
    return _comms.rx.data_length;
}

bool _dev_comms_rx_handler_add_data_check_overflow(uint8_t rx_data)
{
    //iprintln(trCOMMS, "#Got 0x%02X (%d)",rx_data, _comms.rx.length);

    if (_comms.rx.length >= RGB_BTN_MSG_MAX_LEN)
        return false;

    //If we had a message ready, it is goneskies now!
    ((uint8_t *)&_comms.rx.msg)[_comms.rx.length] = rx_data;
    _comms.rx.length++;

    return true;
}

/*! @brief Writes the data from the tx msg buffer to the serial port (including escaping)
 * This function will block until the message is completely sent 
 * @return true if the message was sent successfully, false if the bus was not available
 */
void _dev_comms_tx_start(void)
{
    if (_comms.tx.length == 0)
    {
        //How did we get to this point?
        _tx_state = tx_idle;
        //iprintln(trCOMMS, "#No Msg data to Tx (%d bytes)", _comms.tx.length);
        dev_rgb_set_colour(colGreen);
        return;
    }
    //iprintln(trCOMMS, "#Msg Tx %d bytes (Seq # %d)", _comms.tx.length, _comms.tx.msg.hdr.id);

    //Make sure any console prints are finished before we start sending... 
    // this ensures that the RS-485 is enabled again once the last TX complete IRQ has fired.
    hal_serial_flush(); 

    //Do this only once the bus is silent
    if (_rx_state != rx_listen)
    {
        dev_rgb_set_colour(colBlue);
        return;
    }

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
    _tx_state = tx_echo_rx;
    
    //Make sure the entire message is sent over RS485 before we continue (with potential printf's)
    hal_serial_flush(); 

    _comms.tx.retry_cnt++;
    
    //NO PRINT SECTION END

    //iprintln(trCOMMS, "#Msg Tx Complete (%d)", _comms.tx.retry_cnt);
    //dev_rgb_set_colour(colYellow);

    return;
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

void _rx_irq_callback(uint8_t rx_data)
{
    //This flag is set when (1) an STX is receivedan AND _msg_available is currently set or 
    // (2) when an error occurs within a message (e.g. buffer overflow). 
    // While set, no received data will be saved, preserving the data in the buffer.
    static bool prevent_buffer_overwrite = false;

    //This is a callback which is called from the serial interrupt handler
    if (rx_data == STX)
    {
        //Discard any encapsulated messaged we receive while we have an unhandled message in the RX buffer
        prevent_buffer_overwrite = _msg_available;

        _comms.rx.length = 0;
        _rx_state = rx_busy;
    }    
    else if (_rx_state == rx_listen)
    {
        //Not within STX-ETX, so pass this byte onto the console handler
        console_read_byte(rx_data);
    }
    else if (_rx_state == rx_busy) 
    {
        if (rx_data == ETX)
        {
            // Is this a half-duplex echo of what we were busy transmitting just now?
            if (_tx_state == tx_echo_rx)
            {
                // if we were sending just now, we want to check if our rx and tx buffers match
                if (memcmp((uint8_t *)&_comms.rx.msg, (uint8_t *)&_comms.tx.msg, min(_comms.rx.length, RGB_BTN_MSG_MAX_LEN)) == 0)
                {
                    // tx_success = true;
                    _comms.tx.seq++;
                    _tx_state = tx_idle;
                }
                //else  //only reason this would happen is if we had a bus collision
                // If we remain in this state we will get a timeout once the bus is free again and our retry mechanism will kick in

                //No need to check this message in the application, we are done with it

            }
            else if (!prevent_buffer_overwrite)
                _msg_available = true; //We have a message available, but we need to check if it is valid first

            prevent_buffer_overwrite = false; //Don't need this anymore
            _rx_state = rx_listen; 
        }
        else if (rx_data == DLE)
            _rx_state = rx_escaping;
        else 
        {
            if (!prevent_buffer_overwrite)
            {
                if (!_dev_comms_rx_handler_add_data_check_overflow(rx_data))
                    prevent_buffer_overwrite = true; //Error.... but what can we do... just discard the remainder of the incoming stream
            }
            // _rx_state = rx_busy;
        }
    }
    else //if (_rx_state == rx_escaping)
    {
        if (!prevent_buffer_overwrite)
        {
            if (!_dev_comms_rx_handler_add_data_check_overflow(rx_data))
                prevent_buffer_overwrite = true; //Error.... but what can we do... just discard the remainder of the incoming stream
        }
        _rx_state = rx_busy;
    }
}


unsigned int _dev_comms_response_add_data(uint8_t crc, uint8_t start_index, uint8_t * data, uint8_t data_len)
{
    if ((data != NULL) && (data_len > 0))
    {
        memcpy(&((uint8_t *)&_comms.tx.msg)[_comms.tx.length], data, data_len);
        _comms.tx.length += data_len; // for the payload
    }

    //Calculate the crc of the newly added data in the message
    ((uint8_t *)&_comms.tx.msg)[_comms.tx.length] = crc8_n(crc, (((uint8_t *)&_comms.tx.msg)+start_index), _comms.tx.length - start_index);

    //Do NOT increment the length here,... in case more data gets added, the CRC should be overwritten 
    // and then re-calculated at the end of the newly added data

    return data_len; // The number of bytes added (+2 for the command and response code)
}

unsigned int  _dev_comms_response_add_console_resp(uint8_t * data, uint8_t data_len)
{
    // We cannot send a message if we are not ready to send now, otherwise we will overwrite data being sent
    if (_tx_state != tx_msg_busy) //Must be preceded with start()
        return 0;

    if (data_len > (sizeof(_comms.tx.msg.data) - sizeof(uint8_t))) //For the CRC
        return 0;

    if ((_comms.tx.length + data_len) > (RGB_BTN_MSG_MAX_LEN - sizeof(uint8_t))) //For the CRC
    {
        //What say you we send what we have so far, and then start a new message?
        dev_comms_transmit_now();
        dev_rgb_set_colour(colPurple);
        dev_comms_response_add(cmd_wr_console_cont, btn_cmd_err_ok, NULL, 0);
        //dev_rgb_set_colour(colWhite);
        //RVN - TODO - This needs to be tested!!!!
    }

    uint8_t _start_index = _comms.tx.length; //Where the current CRC is located
    uint8_t _crc = ((uint8_t *)&_comms.tx.msg)[_comms.tx.length];

    return _dev_comms_response_add_data(_crc, _start_index, data, data_len); //The number of bytes added
    // if ((data != NULL) && (data_len > 0))
    // {
    //     memcpy(&((uint8_t *)&_comms.tx.msg)[_comms.tx.length], data, data_len);
    //     _comms.tx.length += data_len; // for the payload
    // }

    // //Calculate the crc of the newly added data in the message
    // ((uint8_t *)&_comms.tx.msg)[_comms.tx.length] = crc8_n(_crc, (((uint8_t *)&_comms.tx.msg)+_start_index), _comms.tx.length - _start_index);

    // //Do NOT increment the length here,... in case more data gets added, the CRC should be overwritten 
    // // and then re-calculated at the end of the newly added data

    // return data_len; // The number of bytes added (+2 for the command and response code)
}

/******************************************************************************
Public functions
******************************************************************************/
void dev_comms_init(void)
{
    if (_comms.init_done)
        return; //Already initialised

    memset(&_comms.rx.msg, 0, sizeof(comms_msg_t));
    _rx_state = rx_listen;
    _tx_state = tx_idle;

    memset(&_comms.blacklist, 0, sizeof(dev_comms_blacklist_t));
    
    //Start with a random sequence number (to help detect bus collisions in the case of 2 nodes with the same address)
    _comms.tx.seq = (uint8_t)sys_random(0, ADDR_BROADCAST);

    hal_serial_init(_rx_irq_callback);

    console_init(hal_serial_write, hal_serial_flush, hal_serial_rs485_disable);

    //Generate a new random (and potentially, only temporary) address
    _comms.addr = dev_comms_addr_new();
    _comms.init_done = true;

    sys_set_io_mode(output_Debug, OUTPUT);

    iprintln(trCOMMS, "#Initialised - Payload size: %d/%d (Seq # %d)", sizeof(_comms.rx.msg.data), sizeof(comms_msg_t), _comms.tx.seq);
}

bool dev_comms_response_start(void)
{
    //We can restart a message only if we are idle or if the buffer contains unsent data
    if ((_tx_state != tx_idle) && (_tx_state != tx_msg_busy))
        return false; //We are not ready to send a message

    _comms.tx.length = sizeof(comms_msg_hdr_t);         //Reset to the beginning of the data
    _comms.tx.msg.hdr.version = RGB_BTN_MSG_VERSION;    //Superfluous, but just in case
    _comms.tx.msg.hdr.id = _comms.tx.seq;               //Should have incremented after the last transmission
    _comms.tx.msg.hdr.src = _comms.addr;                //Our Address (might have changed since our last message)
    _comms.tx.msg.hdr.dst = ADDR_MASTER;          //We only ever talk to the master!!!!!

    ((uint8_t *)&_comms.tx.msg)[_comms.tx.length] = crc8_n(0, ((uint8_t *)&_comms.tx.msg), sizeof(comms_msg_hdr_t));

    _tx_state = tx_msg_busy; //We are starting to build a message
    return true;
}

unsigned int dev_comms_response_add(master_command_t cmd, response_code_t resp_code, uint8_t * data, uint8_t data_len)
{
    // We cannot send a message if we are not ready to send now, otherwise we will overwrite data being sent
    if (_tx_state != tx_msg_busy) //Must be preceded with start()
        return 0;

    if ((uint8_t)(data_len + 2) > (sizeof(_comms.tx.msg.data)))
        return 0; //This is NEVER gonna fit!!!

    if ((uint8_t)(_comms.tx.length + data_len + 2) > (RGB_BTN_MSG_MAX_LEN - sizeof(uint8_t))) //For the CRC
    {
        //What say you we send what we have so far, and then start a new message?
        dev_comms_transmit_now();
        //RVN - TODO - This needs to be tested!!!!
    }

    uint8_t _start_index = _comms.tx.length; //Where the current CRC is located
    uint8_t _crc = ((uint8_t *)&_comms.tx.msg)[_comms.tx.length];

    ((uint8_t *)&_comms.tx.msg)[_comms.tx.length++] = cmd;
    ((uint8_t *)&_comms.tx.msg)[_comms.tx.length++] = resp_code;

    return (2 + _dev_comms_response_add_data(_crc, _start_index, data, data_len)); //The number of bytes added (+2 for the command and response code)
}

size_t dev_comms_response_add_byte(uint8_t data)
{
    return (size_t) _dev_comms_response_add_console_resp(&data, 1);
}

void dev_comms_response_send(void)
{
    if ((_comms.tx.length <= (sizeof(comms_msg_hdr_t ))) || (_tx_state != tx_msg_busy)) //Must be preceded with start()
        return; //Nothing to send, let the user think everything is all good

    //The CRC is already added to the message length, so we need to make sure we don't overflow the buffer
    // if (_comms.tx.length > (RGB_BTN_MSG_MAX_LEN - sizeof(uint8_t)))
    // {
    //     // This is a complete mess.... how did this happen?
    //     //RVN - TODO - Error!!!!
    //     return -2;
    // }

    _comms.tx.length++; //For the CRC
    _comms.tx.retry_cnt = 0; //We are starting a new transmission, so reset the retry count
    _tx_state = tx_queued; //Will be started once the bus is free

    // return 0;
}

void dev_comms_transmit_now(void)
{
    // comms_msg_tx_state_t last_tx_state;
    dev_rgb_set_colour(colRed);
    dev_comms_response_send();
    //tx_state == tx_queued;
    sys_output_write(output_Debug, HIGH);
    // last_tx_state = _tx_state;
    sys_poll_tmr_start(&_tx_tmr, 10L, true); //Check for successful transmission only once every 5ms (RVN, cehck this)
    do { 
        sys_output_write(output_Debug, LOW);
        //Discard any incoming data while we are busy transmitting
        // dev_comms_rx_msg_available(NULL, NULL, NULL); //Check if we have a message available
        if (sys_poll_tmr_expired(&_tx_tmr))
        {
            dev_comms_tx_service();
            sys_output_write(output_Debug, HIGH); //Disable the RS-485 driver
        }

        // if (last_tx_state != _tx_state)
        // {
        //     switch (_tx_state)
        //     {
        //         case tx_idle:
        //             dev_rgb_set_colour(colGreen);
        //             break;
        //         case tx_msg_busy:
        //             dev_rgb_set_colour(colBlue);
        //             break;
        //         case tx_echo_rx:
        //             dev_rgb_set_colour(colPurple);
        //             break;
        //         case tx_queued:
        //             dev_rgb_set_colour(colOrange);
        //             break;
        //         default:
        //             break;
        //     }
        //     last_tx_state = _tx_state;
        // }
    }while (_tx_state != tx_idle); //Wait for the bus to be free again
    dev_comms_response_start();
}

void dev_comms_tx_service(void)
{
    switch (_tx_state)
    {
        case tx_idle:
            break;

        case tx_msg_busy:
            /* Do nothing */
            break;

        case tx_echo_rx:
            if (_comms.tx.retry_cnt >= 5)
            {
                //We have retried this message too many times, so we need to give up and move on
                _tx_state = tx_idle;
                iprintln(trCOMMS, "#TX Error: %s (%d bytes)", "No Echo", _comms.rx.length);
                iprintln(trCOMMS, "#TX data:");
                console_print_ram(trCOMMS, &_comms.tx.msg, (unsigned long)&_comms.tx.msg, _comms.tx.length);//min(_comms.tx.length, sizeof(comms_msg_t)));
            }
            else
                _dev_comms_tx_start();
            break;

        case tx_queued:
            _dev_comms_tx_start();
            break;

        default:
            //We should never get here
            break;
    }
}

uint8_t dev_comms_addr_get(void)
{
    return _comms.addr;
}

bool dev_comms_addr_set(uint8_t addr)
{
    //Is the provided address valid?
    if (_dev_comms_verify_addr(addr))
    {
        //All good.... let's rather use this one.
        _comms.addr = addr;
        iprintln(trCOMMS, "#Address Set: 0x%02X", _comms.addr);
        return true;
    }
    iprintln(trCOMMS, "#Invalid Address: 0x%02X (Keeping 0x%02X)", addr, _comms.addr);
    return false;
}

void dev_comms_blacklist_add(uint8_t new_addr)
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

int8_t dev_comms_rx_msg_available(uint8_t * _src, uint8_t * _dst, uint8_t * _data)
{
    // static comms_msg_rx_state_t _last_rx_state = rx_listen;
    // static comms_msg_tx_state_t _last_tx_state = tx_idle;
    int8_t ret_val = 0;
    int err_data;

    if (!_msg_available)
        return 0;

    ret_val = _comms_check_rx_msg(&err_data);

    if (ret_val < 0)
    {
        iprintln(trCOMMS, "#RX Error: %ds (%d bytes)", ret_val, /*_comms_rx_error_msg(ret_val, err_data), */ _comms.rx.length);
        iprintln(trCOMMS, "#Data: ");
        console_print_ram(trCOMMS, &_comms.rx.msg, (unsigned long)&_comms.rx.msg, sizeof(comms_msg_t));
        memset(&_comms.rx.msg, 0, sizeof(comms_msg_t));
    }
    else
    {
        if (_src != NULL)
            *_src = _comms.rx.msg.hdr.src;
        if (_dst != NULL)
            *_dst = _comms.rx.msg.hdr.dst;
        if ((_data != NULL) && (ret_val > 0))
            memcpy(_data, &_comms.rx.msg.data, ret_val);
    }

    //Any data we had has been copied, our buffer is free for the next message to come in
    _msg_available = false;

    return ret_val;
}

#undef PRINTF_TAG
/*************************** END OF FILE *************************************/
