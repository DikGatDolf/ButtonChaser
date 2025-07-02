/*****************************************************************************

common_comms.h

Common include file for both sets of fw (ESP32 and Arduino Nano) in the 
 ButtonChaser project 

Messages between the master and the slave devices are sent with an STX-DLE-ETX 
framing.

Within the framing, the message format is as follows:
    [Version][Flags][ID][SRC][DST][PAYLOAD][CRC]:
    - VERSION: The current version of the message format
    - ID: Unique ID/Sequence number of this message (to help sync)
    - SRC: The source address of the message
    - DST: The destination address of the message
    - PAYLOAD: The data of the message which will be 1 or more Cmd-&-Data 
        elements, where the Cmd is 1 byte and the data is 0 or more bytes.
        The number of data bytes associated with each command is determined by 
        the command itself
        [Cmd_1][4 bytes][Cmd_2][0 bytes]...[Cmd_N][X bytes]
    - CRC: The CRC of the message (all bytes preceding this byte)

******************************************************************************/
#ifndef __common_comms_H__
#define __common_comms_H__

#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************
includes
******************************************************************************/
#include <stdint.h>
// #include <assert.h>

/******************************************************************************
definitions
******************************************************************************/
#ifdef __NOT_EXTERN__
#define EXT
#else
#define EXT extern
#endif /* __NOT_EXTERN__ */

/******************************************************************************
Macros
******************************************************************************/
// #define RGB_BTN_I2C_ADDR_BITS   (7)     /* 0x00 to 0x7F */
// #define RGB_BTN_MAX             (BIT_POS(RGB_BTN_I2C_ADDR_BITS)) /* 1<<7 = 128 */
// #define RGB_BTN_I2C_ADDR_MASK   (RGB_BTN_MAX-1)  /* 128-1 = 127 =  0x7F */
// #define RGB_BTN_I2C_TYPE_ID     (0x00)  /* b0000 xxxx - 0x00 to 0x0F*/

// #define RGB_BTN_I2C_CLK_FREQ    (200000)    /* 200kHz */

#define RGB_BTN_MSG_MAX_LEN     (32)    /* Keep as a multiple of 4 */
#define RGB_BTN_MSG_VERSION     (0)

#define STX     (0x02)
#define DLE     (0x10)
#define ETX     (0x03)
// #define ACK     (0x06)
// #define NAK     (0x15)

#define ADDR_MASTER       (0x00)
#define ADDR_BROADCAST    (0xFF)

#define ADDR_SLAVE_MIN    (ADDR_MASTER + 1)
#define ADDR_SLAVE_MAX    (ADDR_BROADCAST -1)

// #define ACK     (0x06)
// #define NAK     (0x15)

#define RGB_BTN_MAX_NODES       (31)
#define BUS_SILENCE_MIN_MS      (5)  /* ms */
/******************************************************************************
Struct & Unions
******************************************************************************/

typedef enum command_e
{/* command id                  #      Description                        MOSI Payload  Response (MISO) Payload */
    cmd_roll_call_all       = 0x00, /* Requests a Slave Roll-call - ALL    none             none                
                                        recipients should respond to this */
    cmd_roll_call_unreg     = 0x01, /* Requests a Slave Roll-call - ONLY   none             none                
                                        unregistered recipients should 
                                        respond to this */

    cmd_bcast_address_mask  = 0x02, /* Indicates the indices of            uint32_t         none
                                         intended recipients of the 
                                         broadcast message
                                        IMPORTANT: This should be the 1st cmd in any broadcast message (dst == 0xFF)*/


    cmd_set_rgb_0           = 0x10, /* Set the primary LED colour          24 bits          none                */
    cmd_set_rgb_1           = 0x11, /* Set the secondary LED colour        24 bits          none                */
    cmd_set_rgb_2           = 0x12, /* Set the 3rd LED colour              24 bits          none                */
    cmd_set_blink           = 0x13, /* Set the blinking interval           uint32_t         none                */
    cmd_start_sw            = 0x14, /* Starts the button stopwatch         none             none                */
    //Skip 0x15, which corresponds to the "get Flags" command
    cmd_set_dbg_led         = 0x16, /* Set the debug LED state             1 byte           none                */

    cmd_set_bitmask_index   = 0x1E, /* Registers a slave by assigning it   1 byte           none                
                                        a slot (address bit 0 to 31) */

    cmd_new_add             = 0x1F, /* Sets a new device address - MUST    1 byte             none                
                                        IMPORTANT: This must be the LAST cmd in any message                     */

    cmd_get_rgb_0           = 0x20, /* Get the primary LED colour          none             24 bits             */
    cmd_get_rgb_1           = 0x21, /* Get the secondary LED colour        none             24 bits             */
    cmd_get_rgb_2           = 0x22, /* Get the 3rd LED colour              none             24 bits             */
    cmd_get_blink           = 0x23, /* Get the blinking interval           none             uint32_t            */
    cmd_get_sw_time         = 0x24, /* Requests the elapsed time of the    none             4 bytes             
                                        reaction time sw                    */
    cmd_get_flags           = 0x25, /* Requests the system flags           none             1 bytes             */

    cmd_get_dbg_led         = 0x26, /* Get the debug LED state             none             1 byte              */

    /* This command cannot be "packed" along with other commands as the entire payload will be used*/
    cmd_wr_console_cont     = 0x40, /* Write to the slave device console   buffer           none
                                        1st byte in data is the length 
                                        of the data to send to the slave 
                                        console */  
    cmd_wr_console_done     = 0x41, /* Last part of a write to the slave   buffer           expects a potentially 
                                        device console. 1st byte in data                    multi-packet response 
                                        is the length of the data to send 
                                        to the slave console */  

    cmd_debug_0             = 0x80, /* Dummy command. just fills the data buffer */

    cmd_none                = 0xFF, /* Placeholder */
}master_command_t;

typedef enum response_e
{
    resp_ok                 = 0x00, /* The command was successful*/
    resp_err_payload_len    = 0x01, /* The command payload did not contain sufficient data for the command - followed by length byte*/
    resp_err_range          = 0x02, /* The payload contained a value outside of the acceptable range - followed by a value byte*/
    resp_err_unknown_cmd    = 0x03, /* The command was not recognised - followed by the command byte*/
    resp_err_none           = 0xFF, /* Placeholder */
}response_code_t;

enum system_flags_e
{
    flag_s_press     = BIT_POS(0),
    flag_l_press     = BIT_POS(1),
    flag_d_press     = BIT_POS(2),
    flag_active      = BIT_POS(3),
    flag_deactivated = BIT_POS(4),
    flag_blinking    = BIT_POS(5),
    flag_unreg       = BIT_POS(6),
    flag_reserved    = BIT_POS(7),
};

typedef enum dbg_blink_state_e
{
    dbg_led_off         = 0,
    dbg_led_on          = 1,
    dbg_led_blink_50ms  = 2,
    dbg_led_blink_200ms = 3,
    dbg_led_blink_500ms = 4,
    dbg_led_state_limit = 5,
}dbg_blink_state_t;

/******************************************************************************
Global (public) variables
******************************************************************************/
/* To get consistent packing between the 8-bit Arduino and the 32-bit ESP32 we
 need to pack the structure to 1-byte alignment*/
#pragma pack(push, 1)
typedef struct {
    uint8_t version;// Version number of this message format
    uint8_t id;     // Unique ID/Sequence number of this message (to help sync)
    uint8_t src;    // The source address of the message
    uint8_t dst;    // The destination address of the message
}comms_msg_hdr_t;

typedef struct {
    comms_msg_hdr_t hdr;
    uint8_t data[RGB_BTN_MSG_MAX_LEN - sizeof(comms_msg_hdr_t) - sizeof(uint8_t)];
    uint8_t crc;    // Not necisarely the last byte of the message, byte we should allow for it
}comms_msg_t;
#pragma pack(pop)

//STATIC_ASSERT(((sizeof(comms_msg_hdr_t)+sizeof(uint8_t)) < RGB_BTN_MSG_MAX_LEN), "rgb_btn_msg_hdr_t is too big");

#ifdef __cplusplus
}
#endif

#undef EXT
#endif /* __common_comms_H__ */

/****************************** END OF FILE **********************************/
