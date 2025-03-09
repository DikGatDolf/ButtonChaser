/*****************************************************************************

dev_rgb.h

Include file for dev_rgb.c

******************************************************************************/
#ifndef __dev_rgb_H__
#define __dev_rgb_H__


/******************************************************************************
includes
******************************************************************************/
#include "Arduino.h"

/******************************************************************************
Macros
******************************************************************************/

/******************************************************************************
definitions
******************************************************************************/
/* The order is selected to match a W-R-G-B order in a uint32_t dword so that
 * the colours can be easily indexed using (u32_col_word >> (e_led_colour*8))*/
typedef enum e_led_colour
{
    rgbBlue   = 0,
    rgbGreen  = 1,
    rgbRed    = 2,
    rgbWhite  = 3,
    rgbMAX    = 4
}led_colour_type;

/******************************************************************************
Struct & Unions
******************************************************************************/

/******************************************************************************
variables
******************************************************************************/

/******************************************************************************
functions
******************************************************************************/

bool        dev_rgb_start(int pin_red, int pin_green, int pin_blue, int pin_white);
void        dev_rgb_stop();
bool        dev_rgb_enabled();
void        dev_rgb_set_1col(led_colour_type col, uint8_t duty_cycle_target);
void        dev_rgb_set_wrgb(uint32_t wrgb);
uint32_t    dev_rgb_get_wrgb(void);
// double dev_rgb_get_frequency();
// double dev_rgb_get_period();
// uint8_t dev_rgb_get_resolution();

#endif /* __dev_rgb_H__ */

/****************************** END OF FILE **********************************/
