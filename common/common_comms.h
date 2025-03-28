/*****************************************************************************

common_comms.h

Common include file for both sets of fw (ESP32 and Arduino Nano) in the 
 ButtonChaser project 

Messages between the master and the slave devices are sent with an STX-DLE-ETX 
framing.

Within the framing, the message format is as follows:
    [Version][Flags][ID][SRC][DST][PAYLOAD][CRC]:
    - VERSION: The current version of the message format
    - FLASG: Flags for this message (e.g. WAIT_FOR_RESPONSE, etc)
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

#define RGB_BTN_ADDR_MASTER     (0x00)
#define RGB_BTN_ADDR_BROADCAST  (0xFF)

#define ACK     (0x06)
#define NAK     (0x15)

#define BUS_SILENCE_MIN_MS      (25)  /* 25 ms */
/******************************************************************************
Struct & Unions
******************************************************************************/

typedef enum rgb_btn_command_e
{
    cmd_set_rgb_0           = 0, /* Set the primary LED colour (24 bits payload, no response) */
    cmd_set_rgb_1           = 1, /* Set the secondary LED colour (24 bits payload, no response) */
    cmd_set_rgb_2           = 2, /* Set the secondary LED colour (24 bits payload, no response) */
    cmd_set_blink           = 3, /* Set the blinking interval (uint32_t payload, no response) */
    cmd_get_btn             = 4, /* Requests the state of the button (no payload, expects response) */
//    cmd_get_sw_lap          = 4, /* Requests the "lap" value of the current running stopwatch, without stopping it (no payload, expects response) */
    cmd_sw_start            = 5, /* MSB flag Starts the stopwatch (no payload, no response) */

    /* This command cannot be "packed" along with other commands as the entire payload will be used*/
    cmd_wr_console          = 6, /* Write to the slave device console (partial/full buffer) - expects a response 
                                        1st byte in data is the length of the data to send to the slave console */                                    
}rgb_btn_command_t;

#define RGB_BTN_FLAG_CMD_COMPLETE   (0x80)

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
}rgb_btn_msg_hdr_t;

// #if (sizeof(rgb_btn_msg_hdr_t) > RGB_BTN_MSG_MAX_LEN - sizeof(uint8_t))
// #error "rgb_btn_msg_hdr_t is too big for the message buffer"
// #endif

typedef struct {
    rgb_btn_msg_hdr_t hdr;
    uint8_t data[RGB_BTN_MSG_MAX_LEN - sizeof(rgb_btn_msg_hdr_t) - sizeof(uint8_t)];
    uint8_t crc;    // Not necisarely the last byte of the message, byte we should allow for it
}rgb_btn_msg_t;
#pragma pack(pop)

STATIC_ASSERT(((sizeof(rgb_btn_msg_hdr_t)+sizeof(uint8_t)) < RGB_BTN_MSG_MAX_LEN), "rgb_btn_msg_hdr_t is too big");

#ifdef __cplusplus
}
#endif

#undef EXT
#endif /* __common_comms_H__ */

/****************************** END OF FILE **********************************/
