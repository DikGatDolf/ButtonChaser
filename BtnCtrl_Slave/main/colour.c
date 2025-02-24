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
#define AS_RGB(r, g, b) (0x00FFFFFF & (((r) << 16) | ((g) << 8) | (b)))

/*******************************************************************************
 Local structure
*******************************************************************************/
typedef struct 
{
    const char * Name;
    uint8_t r;
    uint8_t g;
    uint8_t b;
}Colour_t;

/*******************************************************************************
 Local function prototypes
 *******************************************************************************/

/*! \brief A private function that adds two integers
 * \param arg1 The first integer
 * \param arg2 The second integer
 * \return The sum of the two integers
 */
 int _foo_private(int arg1, int arg2);
 
 /*******************************************************************************
 Local variables
 *******************************************************************************/
static const Colour_t _colour[col_max] = 
{
    [colBlack]      = {.Name = "Black",   .r = 0,   .g = 0,   .b = 0},
    [colNavy]       = {.Name = "Navy",    .r = 0,   .g = 0,   .b = 128},
    [colBlue]       = {.Name = "Blue",    .r = 0,   .g = 0,   .b = 255},
    [colGreen]      = {.Name = "Green",   .r = 0,   .g = 128, .b = 0},
    [colTeal]       = {.Name = "Teal",    .r = 0,   .g = 128, .b = 128},
    [colLime]       = {.Name = "Lime",    .r = 0,   .g = 255, .b = 0},
    [colCyan]       = {.Name = "Cyan",    .r = 0,   .g = 255, .b = 255},
    [colMaroon]     = {.Name = "Maroon",  .r = 128, .g = 0,   .b = 0},
    [colPurple]     = {.Name = "Purple",  .r = 128, .g = 0,   .b = 128},
    [colOlive]      = {.Name = "Olive",   .r = 128, .g = 128, .b = 0},
    [colGray]       = {.Name = "Gray",    .r = 128, .g = 128, .b = 128},
    [colSilver]     = {.Name = "Silver",  .r = 192, .g = 192, .b = 192},
    [colMagenta]    = {.Name = "Magenta", .r = 255, .g = 0,   .b = 255},
    [colRed]        = {.Name = "Red",     .r = 255, .g = 0,   .b = 0},
    [colOrange]     = {.Name = "Orange",  .r = 255, .g = 165, .b = 0},
    [colPink]       = {.Name = "Pink",    .r = 255, .g = 192, .b = 203},    
    [colYellow]     = {.Name = "Yellow",  .r = 255, .g = 255, .b = 0},
    [colWhite]      = {.Name = "White",   .r = 255, .g = 255, .b = 255},

};

/*******************************************************************************
 Local (private) Functions
*******************************************************************************/

int _foo_private(int arg1, int arg2)
{
    return arg1 + arg2;
}

/*******************************************************************************
 Global (public) Functions
*******************************************************************************/

uint32_t colour_as_rgb(eColour col)
{
    if (col >= col_max)
        col = col%col_max;
    
    return AS_RGB(_colour[col].r, _colour[col].g, _colour[col].b);
}

#undef PRINTF_TAG
#undef EXT
/*************************** END OF FILE *************************************/
