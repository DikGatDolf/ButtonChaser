/*****************************************************************************

common_comms.h

Common include file for both sets of fw (ESP32 and Arduino Nano) in the 
 ButtonChaser project 

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
#define RGB_BTN_I2C_ADDR_BITS   (4)     /* b0000 xxxx - 0x00 to 0x0F*/
#define RGB_BTN_MAX             (BIT_POS(RGB_BTN_I2C_ADDR_BITS)) /* 1<<4 = 16 */
#define RGB_BTN_I2C_ADDR_MASK   (RGB_BTN_MAX-1)  /* 16-1 = 15 =  0x0F */
#define RGB_BTN_I2C_TYPE_MASK   (0xFF ^ (RGB_BTN_I2C_ADDR_MASK))  /* 0xFF ^ 0x0F =  0xF0 */
#define RGB_BTN_I2C_TYPE_ID     (0x00)  /* b0000 xxxx - 0x00 to 0x0F*/


/******************************************************************************
Struct & Unions
******************************************************************************/

typedef enum comms_command_e
{
    cmd_nop       = 0,
    cmd_set_rgb   = BIT_POS(0),
    cmd_get_btn   = BIT_POS(1),
    cmd_reset_btn = BIT_POS(2),
    //Keep this at the end
}comms_command_t;

typedef struct comms_package_st
{
    uint8_t command;
    union {
        uint8_t  u8 [4];
        uint16_t u16[2];
        uint32_t u32;
    } data;
    uint8_t crc;
} comms_tx_package_t;
/******************************************************************************
Global (public) variables
******************************************************************************/

/******************************************************************************
Global (public) function definitions
******************************************************************************/

/*! \brief A public function that adds two integers
 * \param arg1 The first integer
 * \param arg2 The second integer
 * \return The sum of the two integers
 */
int foo_public(int arg1, int arg2);

#ifdef __cplusplus
}
#endif

#undef EXT
#endif /* __common_comms_H__ */

/****************************** END OF FILE **********************************/
