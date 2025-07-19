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
#include "common_defines.h"

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

#define REMOTE_CONSOLE_SUPPORTED (0) /* Enables/disables the remote console (which is still untested). Currently this equates to 650 bytes Flash and 2 bytes RAM */

#define CMD_TYPE_BROADCAST  0x01
#define CMD_TYPE_DIRECT     0x02
#define CMD_TYPE_RESTRICTED 0x04

/******************************************************************************
Struct & Unions
******************************************************************************/

typedef enum command_e
{/* command id                  #      Description                        MOSI Payload  Response (MISO) Payload */
    cmd_none                = 0x00, /* Placeholder (RVN - ping?)           none             none */
    
    cmd_roll_call           = 0x01, /* Requests a Slave Roll-call          1 byte          none                */

    cmd_bcast_address_mask  = 0x02, /* Indicates the indices of            uint32_t         none
                                         intended recipients of the 
                                         broadcast message
                                        IMPORTANT: This should be the 1st cmd in any broadcast message (dst == 0xFF)*/


    cmd_set_rgb_0           = 0x10, /* Set the primary LED colour          24 bits          none                */
    cmd_set_rgb_1           = 0x11, /* Set the secondary LED colour        24 bits          none                */
    cmd_set_rgb_2           = 0x12, /* Set the 3rd LED colour              24 bits          none                */
    cmd_set_blink           = 0x13, /* Set the blinking interval           uint32_t         none                */
    cmd_set_switch          = 0x14, /* Starts the button stopwatch         1 byte           none                */
    //Skip 0x15, which corresponds to the "get Flags" command
    cmd_set_dbg_led         = 0x16, /* Set the debug LED state             1 byte           none                */
    cmd_set_time            = 0x17, /* Set the system time                 uint32_t         none                */

#if CLOCK_CORRECTION_ENABLED == 1
    cmd_set_sync            = 0x18, /* Start/end the time sync process     uint32_t         none                */
#endif /* CLOCK_CORRECTION_ENABLED */
   
    /* ############# END OF BROADCAST'able COMMANDS!! #############
        ALL commands values higher than "cmd_set_bitmask_index" can ONLY be sent directly to a node */                                        

    cmd_set_bitmask_index   = 0x30, /* Registers a slave by assigning it   1 byte           none                
                                        a slot (address bit 0 to 31) */


    cmd_new_add             = 0x31, /* Sets a new device address - MUST    1 byte             none                
                                        IMPORTANT: This must be the LAST cmd in any message                     */

    cmd_get_rgb_0           = 0x40, /* Get the primary LED colour          none             24 bits             */
    cmd_get_rgb_1           = 0x41, /* Get the secondary LED colour        none             24 bits             */
    cmd_get_rgb_2           = 0x42, /* Get the 3rd LED colour              none             24 bits             */
    cmd_get_blink           = 0x43, /* Get the blinking interval           none             uint32_t            */
    cmd_get_reaction        = 0x44, /* Requests the button reaction time   none             4 bytes             */
    cmd_get_flags           = 0x45, /* Requests the system flags           none             1 bytes             */

    cmd_get_dbg_led         = 0x46, /* Get the debug LED state             none             1 byte              */

    cmd_get_time            = 0x47, /* Requests the system runtime         none             4 bytes             */

#if CLOCK_CORRECTION_ENABLED == 1
    cmd_get_sync            = 0x48, /* Requests the time sync value        none             uint32_t            */
#endif /* CLOCK_CORRECTION_ENABLED */
    cmd_get_version         = 0x49, /* Requests the fw veresion            none             uint32_t            */

#if REMOTE_CONSOLE_SUPPORTED == 1    
    /* This command cannot be "packed" along with other commands as the entire payload will be used*/
    cmd_wr_console_cont     = 0x40, /* Write to the slave device console   buffer           none
                                        1st byte in data is the length 
                                        of the data to send to the slave 
                                        console */  
    cmd_wr_console_done     = 0x41, /* Last part of a write to the slave   buffer           expects a potentially 
                                        device console. 1st byte in data                    multi-packet response 
                                        is the length of the data to send 
                                        to the slave console */  
#endif /* REMOTE_CONSOLE_SUPPORTED */

    cmd_debug_0             = 0x80, /* Dummy command. just fills the data buffer */
}master_command_t;

typedef enum response_e
{
    resp_ok                 = 0x00, /* The command was successful*/
    resp_err_payload_len    = 0x01, /* The command did not contain sufficient data - payload[1] = {# of bytes attempted to read}*/
    resp_err_range          = 0x02, /* The command was out of range - payload[2] =  {<RX value>, <threshold value>} */
    resp_err_unknown_cmd    = 0x03, /* The command was not recognised - no payload */
    resp_err_reject_cmd     = 0x04, /* The command was illegal at this time, and promptly rejected - payload[1] = identifying*/
    resp_err_none           = 0xFF, /* Placeholder */
}response_code_t;

enum system_flags_e
{
    flag_s_press     = BIT_POS(0),
    flag_l_press     = BIT_POS(1),
    flag_d_press     = BIT_POS(2),
    flag_activated   = BIT_POS(3),
    flag_deactivated = BIT_POS(4),
    flag_sw_stopped  = BIT_POS(5),
    flag_blinking    = BIT_POS(6),
    flag_unreg       = BIT_POS(7),
};

typedef enum dbg_blink_state_e
{
    dbg_led_off         = 0,
    dbg_led_blink_fast  = 5,
    dbg_led_blink       = 20,
    dbg_led_blink_slow  = 50,
    dbg_led_on          = 0xff, /* This is the "on" state, not a blink state */
}dbg_blink_state_t;

typedef struct
{
    uint32_t            version; // The address of the node (0x00 to 0x7F)
    uint32_t            rgb_colour[3]; // The 3 RGB colours of the LED 
    uint32_t            blink_ms; // The blink period (0 if inactive)
    dbg_blink_state_t   dbg_led_state; // The debug LED state
    uint32_t            reaction_ms; // The reaction time (ms)
    uint32_t            time_ms; // The current time (ms)
    uint8_t             flags; // The system flags
    float               time_factor; // The time factor (used for the time correction)
    bool                sw_active; // Is the button stopwatch active?
}button_t;

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

#define RGB_BTN_MSG_MAX_DATA_LEN     (RGB_BTN_MSG_MAX_LEN - sizeof(comms_msg_hdr_t) - sizeof(uint8_t)) // The maximum data length of the message, excluding the header and CRC

typedef struct {
    comms_msg_hdr_t hdr;
    uint8_t data[RGB_BTN_MSG_MAX_DATA_LEN];
    uint8_t crc;    // Not necessarily the last byte of the message, but we should allow for it
}comms_msg_t;
#pragma pack(pop)

typedef struct command_s
{
    master_command_t cmd; /* The command ID */
    uint8_t mosi_sz;      /* The size of the payload of cmds FROM the Master TO the Slave */
    uint8_t miso_sz;      /* The size of the payload of cmd responses FROM the slave TO the master */
    uint8_t access_flags; /* A mask indicating how this command can be accessed */
}command_payload_size_t;

#ifdef __NOT_EXTERN__
const command_payload_size_t cmd_table[] = 
{
 /* Command ID                 MOSI Payload                                  MISO Payload                             */
    {cmd_roll_call,           sizeof(uint8_t)   /* All or only Unreg     */, 0                   /* Nothing           */, CMD_TYPE_BROADCAST                   | CMD_TYPE_RESTRICTED},
    {cmd_bcast_address_mask,  sizeof(uint32_t)  /* Destination Mask      */, 0                   /* Nothing           */, CMD_TYPE_BROADCAST                   | CMD_TYPE_RESTRICTED},
    {cmd_set_rgb_0,           3*sizeof(uint8_t) /* RGB colour Code       */, 0                   /* Nothing           */, CMD_TYPE_BROADCAST | CMD_TYPE_DIRECT},
    {cmd_set_rgb_1,           3*sizeof(uint8_t) /* RGB colour Code       */, 0                   /* Nothing           */, CMD_TYPE_BROADCAST | CMD_TYPE_DIRECT},
    {cmd_set_rgb_2,           3*sizeof(uint8_t) /* RGB colour Code       */, 0                   /* Nothing           */, CMD_TYPE_BROADCAST | CMD_TYPE_DIRECT},
    {cmd_set_blink,           sizeof(uint32_t)  /* Blink Rate (ms)       */, 0                   /* Nothing           */, CMD_TYPE_BROADCAST | CMD_TYPE_DIRECT},
    {cmd_set_switch,          sizeof(uint8_t)   /* Switch State (on/off) */, 0                   /* Nothing           */, CMD_TYPE_BROADCAST | CMD_TYPE_DIRECT},
    {cmd_set_dbg_led,         sizeof(uint8_t)   /* Debug LED State       */, 0                   /* Nothing           */, CMD_TYPE_BROADCAST | CMD_TYPE_DIRECT},
    {cmd_set_time,            sizeof(uint32_t)  /* New Time in ms        */, 0                   /* Nothing           */, CMD_TYPE_BROADCAST | CMD_TYPE_DIRECT},
#if CLOCK_CORRECTION_ENABLED == 1
    {cmd_set_sync,            sizeof(uint32_t)  /* ms Elapsed on Master  */, 0                   /* Nothing           */, CMD_TYPE_BROADCAST | CMD_TYPE_DIRECT},
#endif /* CLOCK_CORRECTION_ENABLED */
    {cmd_set_bitmask_index,   sizeof(uint8_t)   /* Registration Slot     */, 0                   /* Nothing           */, CMD_TYPE_BROADCAST                  },
    {cmd_new_add,             sizeof(uint8_t)   /* New Address           */, 0                   /* Nothing           */,                      CMD_TYPE_DIRECT | CMD_TYPE_RESTRICTED},
    {cmd_get_rgb_0,           0                 /* Nothing               */, 3*sizeof(uint8_t)   /* RGB Colour Code   */,                      CMD_TYPE_DIRECT},
    {cmd_get_rgb_1,           0                 /* Nothing               */, 3*sizeof(uint8_t)   /* RGB Colour Code   */,                      CMD_TYPE_DIRECT},
    {cmd_get_rgb_2,           0                 /* Nothing               */, 3*sizeof(uint8_t)   /* RGB Colour Code   */,                      CMD_TYPE_DIRECT},
    {cmd_get_blink,           0                 /* Nothing               */, sizeof(uint32_t)    /* Blink Rate (ms)   */,                      CMD_TYPE_DIRECT},
    {cmd_get_reaction,        0                 /* Nothing               */, sizeof(uint32_t)    /* React Time (ms)   */,                      CMD_TYPE_DIRECT},
    {cmd_get_flags,           0                 /* Nothing               */, sizeof(uint8_t)     /* State Flags       */,                      CMD_TYPE_DIRECT},
    {cmd_get_dbg_led,         0                 /* Nothing               */, sizeof(uint8_t)     /* Debug LED state   */,                      CMD_TYPE_DIRECT},
    {cmd_get_time,            0                 /* Nothing               */, sizeof(uint32_t)    /* Runtime (ms)      */,                      CMD_TYPE_DIRECT},
#if CLOCK_CORRECTION_ENABLED == 1
    {cmd_get_sync,            0                 /* Nothing               */, sizeof(float)       /* correction factor */,                      CMD_TYPE_DIRECT},
#endif /* CLOCK_CORRECTION_ENABLED */
    {cmd_get_version,         0                 /* Nothing               */, sizeof(uint32_t)    /* Version           */,                      CMD_TYPE_DIRECT},
};
#else
extern const command_payload_size_t cmd_table[];
#endif /* __NOT_EXTERN__ */

#ifdef __cplusplus
}
#endif

#undef EXT
#endif /* __common_comms_H__ */

/****************************** END OF FILE **********************************/
