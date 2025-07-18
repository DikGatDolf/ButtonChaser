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

#include "../../../../common/common_comms.h"

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
//    tx_queued,      // We are busy transmitting a message (between ETX and STX)
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
//        uint8_t buff[(RGB_BTN_MSG_MAX_LEN*2)+2];    //Absolute worst case scenario
        uint8_t retry_cnt = 0;
        uint8_t seq;
        uint8_t data_length;
    }tx;
    uint8_t addr;           /* My assigned address */
    dev_comms_blacklist_t blacklist; /* List of addresses that are not allowed to be used */
}dev_comms_t;

/******************************************************************************
Local function definitions
******************************************************************************/
void _rx_irq_callback(uint8_t rx_data);
void _bus_silence_expiry(void);

//char * _comms_rx_error_msg(int err_data);
int8_t _comms_check_rx_msg(int *_data);

bool _dev_comms_rx_handler_add_data_check_overflow(uint8_t rx_data);
#if REMOTE_CONSOLE_SUPPORTED == 1    
unsigned int  _dev_comms_response_add_console_resp(uint8_t * data, uint8_t data_len);
#endif /* REMOTE_CONSOLE_SUPPORTED */
unsigned int _dev_comms_response_add_data(uint8_t * data, uint8_t data_len);
void _dev_comms_response_start(void);

/******************************************************************************
Local variables
******************************************************************************/

dev_comms_t _comms;
volatile comms_msg_rx_state_t _rx_state = rx_listen;
volatile comms_msg_tx_state_t _tx_state = tx_idle;

//RVN - TODO - Consider seperating the MSG transmission state from the MSG state.
//This would make it easier to buffer messages for transmission as well.

volatile bool _msg_available = false;

stopwatch_ms_s _tx_sw;

/******************************************************************************
Local functions
******************************************************************************/

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

// uint8_t _tx_err_msg_len;
// comms_msg_t _tx_err_msg = {0};
// uint8_t _rx_err_msg_len;
// comms_msg_t _rx_err_msg = {0};

void _rx_irq_callback(uint8_t rx_data)
{
    //This flag is set when (1) an STX is receivedan AND _msg_available is currently set or 
    // (2) when an error occurs within a message (e.g. buffer overflow). 
    // While set, no received data will be saved, preserving the data in the buffer.
    static bool protecting_rx_msg_buffer = false;

    sys_cb_tmr_start(&_bus_silence_expiry, BUS_SILENCE_MIN_MS);

    //This is a callback which is called from the serial interrupt handler
    if (rx_data == STX)
    {
        //Discard any encapsulated messaged we receive while we have an unhandled message in the RX buffer
        protecting_rx_msg_buffer = _msg_available;

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
                    _comms.tx.seq++;
                    //No need to check this message in the application.... we are done with it
                    _tx_state = tx_idle;
                }
                // else  //only reason this would happen is if we had a bus collision
                // If we remain in this state we will get a timeout once the bus is free again and our retry mechanism will kick in
            }
            else if (!protecting_rx_msg_buffer)
                _msg_available = true; //We have a message available, but we need to check if it is valid first

            protecting_rx_msg_buffer = false; //Don't need this anymore
            _rx_state = rx_listen;
            sys_cb_tmr_stop(&_bus_silence_expiry); //Stop the bus silence timer
        }
        else if (rx_data == DLE)
            _rx_state = rx_escaping;
        else 
        {
            if (!protecting_rx_msg_buffer)
            {
                if (!_dev_comms_rx_handler_add_data_check_overflow(rx_data))
                    protecting_rx_msg_buffer = true; //Error.... but what can we do... just discard the remainder of the incoming stream
            }
            // _rx_state = rx_busy;
        }
    }
    else //if (_rx_state == rx_escaping)
    {
        if (!protecting_rx_msg_buffer)
        {
            if (!_dev_comms_rx_handler_add_data_check_overflow(rx_data^DLE))
                protecting_rx_msg_buffer = true; //Error.... but what can we do... just discard the remainder of the incoming stream
        }
        _rx_state = rx_busy;
    }
}

void _bus_silence_expiry(void)
{
    //If the RX state is not in listen mode after 15ms of silence, we force it to listen mode. Obviously a transmission was interrupted
    // and we need to start over again.
    if (_rx_state != rx_listen)
        _rx_state = rx_listen;
}

unsigned int _dev_comms_response_add_data(uint8_t * data, uint8_t data_len)
{
    if ((data == NULL) || (data_len == 0))
        return 0; //Nothing to do

    uint8_t _crc = _comms.tx.msg.data[_comms.tx.data_length];

    memcpy(&_comms.tx.msg.data[_comms.tx.data_length], data, data_len);

    //Calculate the crc of the newly added data in the message
    _comms.tx.msg.data[_comms.tx.data_length + data_len] = crc8_n(_crc, &_comms.tx.msg.data[_comms.tx.data_length], data_len);

    //NOW we can increment the datalenth
    _comms.tx.data_length += data_len; // for the payload

    return data_len;
}

#if REMOTE_CONSOLE_SUPPORTED == 1    
unsigned int  _dev_comms_response_add_console_resp(uint8_t * data, uint8_t data_len)
{
    // We cannot send a message if we are not ready to send now, otherwise we will overwrite data being sent
    if (_tx_state != tx_msg_busy) //Must be preceded with start()
        return 0;

    if (data_len > (sizeof(_comms.tx.msg.data) - sizeof(uint8_t))) //For the CRC
        return 0;

    if ((_comms.tx.data_length + data_len) > sizeof(_comms.tx.msg.data)) //For the CRC
    {
        //Let's send what we have so far, and then start a new message?
        if (!dev_comms_transmit_now())
            iprintln(trALWAYS, "!Sending interim console response msg");
        dev_comms_response_append(cmd_wr_console_cont, resp_ok, NULL, 0);
    }

    return _dev_comms_response_add_data(data, data_len); //The number of bytes added
}
#endif /* REMOTE_CONSOLE_SUPPORTED */

void _dev_comms_response_start(void)
{

    _comms.tx.data_length = 0;//sizeof(comms_msg_hdr_t);         //Reset to the beginning of the data
    _comms.tx.msg.hdr.version = RGB_BTN_MSG_VERSION;    //Superfluous, but just in case
    _comms.tx.msg.hdr.id = _comms.tx.seq;               //Should have incremented after the last transmission
    _comms.tx.msg.hdr.src = _comms.addr;                //Our Address (might have changed since our last message)
    _comms.tx.msg.hdr.dst = ADDR_MASTER;          //We only ever talk to the master!!!!!

    _comms.tx.msg.data[0] = crc8_n(0, ((uint8_t *)&_comms.tx.msg), sizeof(comms_msg_hdr_t));

    _tx_state = tx_msg_busy; //We are starting to build a message
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

    dev_comms_blacklist_clear();
    
    //Start with a random sequence number (to help detect bus collisions in the case of 2 nodes with the same address)
    _comms.tx.seq = (uint8_t)sys_random(0, ADDR_BROADCAST);

    hal_serial_init(_rx_irq_callback);

    //We have to implement the RS485 Driver and Receiver Enable pin for the  transceiver
    sys_set_io_mode(output_RS485_DE, OUTPUT);

    console_init(hal_serial_write, hal_serial_flush);

    //Generate a new random (and potentially, only temporary) address
    dev_comms_addr_new();
    _comms.init_done = true;

    //sys_set_io_mode(output_Debug, OUTPUT);
    sys_cb_tmr_start(&_bus_silence_expiry, BUS_SILENCE_MIN_MS);

    //iprintln(trCOMMS, "#Initialised - Payload size: %d/%d (Seq # %d)", sizeof(_comms.rx.msg.data), sizeof(comms_msg_t), _comms.tx.seq);
    iprintln(trCOMMS, "#Init %d/%d (Seq # %d)", sizeof(_comms.rx.msg.data), sizeof(comms_msg_t), _comms.tx.seq);
}

bool dev_comms_tx_ready(void)
{
    //We can restart a message only if we are idle or if the buffer contains unsent data
    return ((_tx_state == tx_idle) || (_tx_state == tx_msg_busy));
}

unsigned int dev_comms_response_append(master_command_t cmd, response_code_t resp_code, uint8_t * data, uint8_t data_len, bool restart)
{
    //We *know* we will be adding at least 2 bytes (cmd and resp_code)
    if ((uint8_t)(data_len +2) > sizeof(_comms.tx.msg.data))
        return 0; //This is NEVER gonna fit!!!

    //Indicates we are not supposed to answer
    if (data_len == 0xff)
        return 0; //Nothing to do
    //iprintln(trALWAYS, "#Responding to 0x%02X : 0x%02X (%d bytes)", cmd, resp_code, data_len);

    // We cannot send a message if we are not ready to send now, otherwise we will overwrite data being sent
    if ((_tx_state == tx_idle) || (restart))
        _dev_comms_response_start();
    
    //If another message is already in progress (queued or tx_echo_rx), starting a new one
    // will overwrite the buffered data used ot confirm good transmission
    if (_tx_state != tx_msg_busy)
        return 0;

    if ((uint8_t)(_comms.tx.data_length + data_len + 2) > sizeof(_comms.tx.msg.data))
    {
        //iprintln(trALWAYS, "#TX interim msg");
        //Let's send what we have so far, and then start a new message?
        if (!dev_comms_transmit_now()) //Send the response message now
            iprintln(trALWAYS, "!TX interim msg");
        //We need to "prime" the message first....
        _dev_comms_response_start();
        //RVN - TODO - This needs to be tested!!!!
    }

    uint8_t _crc = _comms.tx.msg.data[_comms.tx.data_length];

    _comms.tx.msg.data[_comms.tx.data_length    ] = cmd;
    _comms.tx.msg.data[_comms.tx.data_length + 1] = resp_code;
    _comms.tx.msg.data[_comms.tx.data_length + 2] = crc8_n(_crc, &_comms.tx.msg.data[_comms.tx.data_length], 2);
    
    _comms.tx.data_length += 2; //For the command and response code

    return (2 + _dev_comms_response_add_data(data, data_len)); //The number of bytes added (+2 for the command and response code)
}

#if REMOTE_CONSOLE_SUPPORTED == 1    
size_t dev_comms_response_add_byte(uint8_t data)
{
    return (size_t) _dev_comms_response_add_console_resp(&data, 1);
}
#endif /* REMOTE_CONSOLE_SUPPORTED */

bool dev_comms_transmit_now(void)
{

    if ((_comms.tx.data_length == 0) || (_tx_state != tx_msg_busy)) //Must be preceded with start()
        return true; //Nothing to send, let the user think everything is all good

    _comms.tx.retry_cnt = 0; //We are starting a new transmission, so reset the retry count
//    _tx_state = tx_queued; //Will be started once the bus is free

    do { //while (_comms.tx.retry_cnt < 5)

        // if (_comms.tx.data_length == 0)
        // {
        //     //How did we get to this point?
        //     _tx_state = tx_idle;
        //     return;
        // }
        uint8_t * msg = (uint8_t *)&_comms.tx.msg;
        uint8_t msg_len = (sizeof(comms_msg_hdr_t) + _comms.tx.data_length + sizeof(uint8_t));
        iprintln(trCOMMS, "#TX: %d bytes (%d)", msg_len, _comms.tx.msg.hdr.id);
        //iprintln(trCOMMS, "#TX: %d bytes (%d) - %d, %d ms", msg_len, _comms.tx.msg.hdr.id, _rx_state, hal_serial_rx_silence_ms());
    
        //Make sure any console prints are finished before we start sending... 
        // this ensures that the RS-485 is enabled again once the last TX complete IRQ has fired.
        hal_serial_flush(); 
    
        //wait here for the bus to go silent!
        while (_rx_state != rx_listen)
        {
            //Wait here for the bus to be free again (should not be more than 15ms)
        }
        
        {   //NO PRINT SECTION START            
            //Make sure any console prints are finished before we start sending... 
            hal_serial_flush(); 

            //We need to enable RS485 RX while we are doing this transmission, 
            //Will be disabled again once the transmission is done
            sys_output_write(output_RS485_DE, HIGH);

            hal_serial_write(STX);
            //We need to make sure our IRQ callback is processiong the received data/echo correctly. 
            //This state (tx_echo_rx) is handled in the serial receive IRQ callback
            _tx_state = tx_echo_rx;
        
            for (uint8_t i = 0; i < msg_len; i++)
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
            //Right, we've sent the message (well, actually we've only loaded it into 
            // the hal_serial tx buffer, but the transmission should have started already), 
            // now we need to wait and check if we received it correctly (pleasures of half-duplex comms)

            //Make sure the entire message is sent over RS485 before we disable the RS485 again
            hal_serial_flush(); 

            //We need to disable RS485 RX once the transmission is done, otherwise we will risk a collision on the bus with other nodes
            sys_output_write(output_RS485_DE, LOW);

            hal_serial_write('\r'); //Just to make sure we have a clean line
            hal_serial_write('\n'); //Just to make sure we have a clean line for our next printf message
        
            _comms.tx.retry_cnt++;
        } //NO PRINT SECTION END
    
        // If our TX state goes into idle, we got it (all handled by the interrupts)
        // otherwise, if the rx state goes into listen, a completed message has been received, but not matched (bus collision?)
        // Lastly, if the bus is silent for a period, then everybody is waiting for everybody.
        do //
        {
    
            if (_tx_state == tx_idle) // ECHO RECEIVED - All good, baby!
                return true; //we are done here

            if (_rx_state == rx_listen)
            {
                iprintln(trCOMMS, "#Bus Collision");
                iprintln(trCOMMS, "#TX Err - %d bytes (seq %d)", _comms.tx.data_length + sizeof(comms_msg_hdr_t) + sizeof(uint8_t), _comms.tx.msg.hdr.id);
                console_print_ram(trCOMMS, (uint8_t *)&_comms.tx.msg, 0, _comms.tx.data_length + sizeof(comms_msg_hdr_t) + sizeof(uint8_t));
                iprintln(trCOMMS, "#Last RX - %d bytes (seq %d)", _comms.rx.length, _comms.rx.msg.hdr.id);
                console_print_ram(trCOMMS, (uint8_t *)&_comms.rx.msg, 0, _comms.rx.length);
                break; //from do-while loop
            }

            if (_tx_state == tx_echo_rx)
            {
                iprintln(trCOMMS, "#No ECHO Rx'd");
                break; //from do-while loop
            }
            

        }while (_tx_state == tx_echo_rx); //Wait for the echo to be received
    
        //Reaching this point means we have not received an echo of our message, so we need to retry it
        //We have retried this message too many times, so we need to give up and move on
    }while (_comms.tx.retry_cnt < 5);//(_tx_state != tx_idle); //Wait for the bus to be free again

    iprintln(trCOMMS, "#TX Abandonded after %d tries (0x%02X)", _comms.tx.retry_cnt, _comms.tx.msg.hdr.id);

    //We increment the sequence number, so that the master will know that something went wrong
    _comms.tx.seq++;

    //Return to an idle state, so that we can start a new message
    _tx_state = tx_idle;

    return false; //We have failed to send the message
}

uint8_t dev_comms_addr_get(void)
{
    return _comms.addr;
}

void dev_comms_addr_set(uint8_t addr)
{
    _comms.addr = addr;
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

void dev_comms_blacklist_clear(void)
{
    memset(&_comms.blacklist, 0, sizeof(dev_comms_blacklist_t));
}

bool dev_comms_verify_addr(uint8_t addr)
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

uint8_t dev_comms_addr_new(void)
{
    uint8_t new_addr;
    int retries = 0;

    do
    {
        //We will be stuck here until we get a valid address.... 
        //our blacklist only has 31 possible values, so we still have a 
        //  1-(33/256) = ~87.1% chance of getting a valid address
        new_addr = (uint8_t)sys_random(ADDR_SLAVE_MIN, ADDR_SLAVE_MAX);
        retries++;
    }while (!dev_comms_verify_addr(new_addr));

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
        iprintln(trCOMMS, "#RX Error: %d (%d bytes):", ret_val, /*_comms_rx_error_msg(ret_val, err_data), */ _comms.rx.length);
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

    //iprintln(trCOMMS, "#RX %d bytes data", ret_val);
    //console_print_ram(trCOMMS, &_comms.rx.msg, 0, _comms.rx.length);

    //Any data we had has been copied, our buffer is free for the next message to come in
    _msg_available = false;

    return ret_val;
}

// void dev_comms_check_error(void)
// {
//     if (!_err.active)
//         return; //No error to report

//     iprintln(trCOMMS, "#TX Err - %d bytes (seq %d)", _err.tx_len, _err.seq);
//     console_print_ram(trCOMMS, _err.tx_data, 0, _err.tx_len);
//     iprintln(trCOMMS, "#Last RX - %d bytes (seq %d)", _err.rx_len, ((comms_msg_t *)&_err.rx_data)->hdr.id);
//     console_print_ram(trCOMMS, _err.rx_data, 0, _err.rx_len);
//     _err.active = false;
// }

#undef PRINTF_TAG
/*************************** END OF FILE *************************************/
