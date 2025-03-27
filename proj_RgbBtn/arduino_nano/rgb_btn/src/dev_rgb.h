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
#define RGB_MAX             (0x00FFFFFF)
#define AS_RGB(r, g, b)     (RGB_MAX & ((((uint32_t)r) << 16) | (((uint32_t)g) << 8) | ((uint32_t)b)))

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


typedef enum
{
    colBlack    = AS_RGB(0,   0,   0),
    colNavy     = AS_RGB(0,   0,   128),
    colBlue     = AS_RGB(0,   0,   255),
    colGreen    = AS_RGB(0,   128, 0),
    colTeal     = AS_RGB(0,   128, 128),
    colLime     = AS_RGB(0,   255, 0),
    colCyan     = AS_RGB(0,   255, 255),
    colMaroon   = AS_RGB(128, 0,   0),
    colPurple   = AS_RGB(128, 0,   128),
    // colOlive    = AS_RGB(128, 128, 0),
    // colGray     = AS_RGB(128, 128, 128),
    // colSilver   = AS_RGB(192, 192, 192),
    colMagenta  = AS_RGB(255, 0,   255),
    colRed      = AS_RGB(255, 0,   0),
    colOrange   = AS_RGB(255, 165, 0),
    // colPink     = AS_RGB(255, 192, 203),    
    colYellow   = AS_RGB(255, 255, 0),
    colWhite    = AS_RGB(255, 255, 255),

    // colBlack    = 0,    /* Blk */
    // colNavy     = 1,    /* */
    // colBlue     = 2,    /* Blu */
    // colGreen    = 3,    /* Grn */
    // colTeal     = 4,    /* */
    // colLime     = 5,    /* */
    // colCyan     = 6,    /* */
    // colMaroon   = 7,    /* */
    // colPurple   = 8,    /* Pur */
    // colOlive    = 9,    /* */
    // colGray     = 10,   /* */
    // colSilver   = 11,   /* */
    // colMagenta  = 12,   /* */
    // colRed      = 13,   /* Red */
    // colOrange   = 14,   /* Ora */
    // colPink     = 15,   /* Pnk */
    // colYellow   = 16,   /* Yel */
    // colWhite    = 17,   /* Wht */
    /* Keep this at the end of the enum */
    // col_max    
}eColour;


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
