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

#define RGB_BTN_I2C_CLK_FREQ    (200000)    /* 200kHz */

#define RGB_BTN_MSG_MAX_LEN     (32)    /* Keep as a multiple of 4 */

/******************************************************************************
Struct & Unions
******************************************************************************/

typedef enum rgb_btn_command_e
{
    cmd_set_rgb,            /* Set the LED colour (1 - colour - primary colour (8+24bits) = 4 bytes payload) */
    cmd_set_blink,          /* Set the LED blink state (2 colours (8+24bits) + blink timer (32bits) = 12 bytes payload) */
    cmd_sw_start,           /* Set the button state (0 byte payload */
    cmd_wr_console,         /* Write to the console (full buffer) */

    //Keep at the end
    cmd_max,         /* Read from the console (full buffer) */
}rgb_btn_command_t;

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
