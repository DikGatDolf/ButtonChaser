/*******************************************************************************
Module:     colour.c
Purpose:    This file contains the _______ functions
Author:     Rudolph van Niekerk

 *******************************************************************************/


/*******************************************************************************
includes
 *******************************************************************************/
#include <stdarg.h>
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>

#define __NOT_EXTERN__
#include "colour.h"
#undef __NOT_EXTERN__

/*******************************************************************************
 Local defines
*******************************************************************************/
#define PRINTF_TAG ("Colour") /* This must be undefined at the end of the file*/

/*******************************************************************************
 Local structure
*******************************************************************************/
typedef struct 
{
    char * Name;
    uint8_t r;
    uint8_t g;
    uint8_t b;
} stColour_t;

/*******************************************************************************
 Local function prototypes
 *******************************************************************************/

 /*******************************************************************************
 Local variables
 *******************************************************************************/
// static stColour_t _colour[col_max] = 
// {
//     [colBlack]      = {.Name = "Black",   .r = 0,   .g = 0,   .b = 0},
//     [colNavy]       = {.Name = "Navy",    .r = 0,   .g = 0,   .b = 128},
//     [colBlue]       = {.Name = "Blue",    .r = 0,   .g = 0,   .b = 255},
//     [colGreen]      = {.Name = "Green",   .r = 0,   .g = 128, .b = 0},
//     [colTeal]       = {.Name = "Teal",    .r = 0,   .g = 128, .b = 128},
//     [colLime]       = {.Name = "Lime",    .r = 0,   .g = 255, .b = 0},
//     [colCyan]       = {.Name = "Cyan",    .r = 0,   .g = 255, .b = 255},
//     [colMaroon]     = {.Name = "Maroon",  .r = 128, .g = 0,   .b = 0},
//     [colPurple]     = {.Name = "Purple",  .r = 128, .g = 0,   .b = 128},
//     [colOlive]      = {.Name = "Olive",   .r = 128, .g = 128, .b = 0},
//     [colGray]       = {.Name = "Gray",    .r = 128, .g = 128, .b = 128},
//     [colSilver]     = {.Name = "Silver",  .r = 192, .g = 192, .b = 192},
//     [colMagenta]    = {.Name = "Magenta", .r = 255, .g = 0,   .b = 255},
//     [colRed]        = {.Name = "Red",     .r = 255, .g = 0,   .b = 0},
//     [colOrange]     = {.Name = "Orange",  .r = 255, .g = 165, .b = 0},
//     [colPink]       = {.Name = "Pink",    .r = 255, .g = 192, .b = 203},    
//     [colYellow]     = {.Name = "Yellow",  .r = 255, .g = 255, .b = 0},
//     [colWhite]      = {.Name = "White",   .r = 255, .g = 255, .b = 255},
// };

/*******************************************************************************
 Local (private) Functions
*******************************************************************************/

/*******************************************************************************
 Global (public) Functions
*******************************************************************************/

//void hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
void hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *rgb)
{
    uint32_t r = RED_from_WRGB(*rgb);
    uint32_t g = GREEN_from_WRGB(*rgb);
    uint32_t b = BLUE_from_WRGB(*rgb);

    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        r = rgb_max;
        g = rgb_min + rgb_adj;
        b = rgb_min;
        break;
    case 1:
        r = rgb_max - rgb_adj;
        g = rgb_max;
        b = rgb_min;
        break;
    case 2:
        r = rgb_min;
        g = rgb_max;
        b = rgb_min + rgb_adj;
        break;
    case 3:
        r = rgb_min;
        g = rgb_max - rgb_adj;
        b = rgb_max;
        break;
    case 4:
        r = rgb_min + rgb_adj;
        g = rgb_min;
        b = rgb_max;
        break;
    default:
        r = rgb_max;
        g = rgb_min;
        b = rgb_max - rgb_adj;
        break;
    }
    *rgb = AS_RGB(r, g, b);
}

#undef PRINTF_TAG

/*************************** END OF FILE *************************************/
