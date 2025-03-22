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

    rgbMAX    = 3
}led_colour_type;

/******************************************************************************
Struct & Unions
******************************************************************************/
typedef struct
{
    union {
        struct {
            uint8_t blue;
            uint8_t green;
            uint8_t red;
            uint8_t unused_white;
        }col;
        uint32_t rgb;
    };
}rgb_colour_t;

/******************************************************************************
variables
******************************************************************************/

/******************************************************************************
functions
******************************************************************************/

bool        dev_rgb_start(int pin_red, int pin_green, int pin_blue);
void        dev_rgb_stop();
bool        dev_rgb_enabled();
void        dev_rgb_set_colour(uint32_t rgb);
uint32_t    dev_rgb_get_colour(void);
uint32_t    dev_rgb_get_pwm(void);

#endif /* __dev_rgb_H__ */

/****************************** END OF FILE **********************************/
