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
#include "sys_utils.h"
#include "str_helper.h"

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
    char * shortName;
    uint32_t rgb;
} stColour_t;

/*******************************************************************************
 Local function prototypes
 *******************************************************************************/

 /*******************************************************************************
 Local variables
 *******************************************************************************/
static stColour_t _colour[] = 
{
    {"Black",   "Bk",   colBlack},
    {"Navy",    "Nv",   colNavy},
    {"Blue",    "Bl",   colBlue},
    {"Green",   "Gn",   colGreen},
    {"Teal",    "Tl",   colTeal},
    {"Lime",    "Lm",   colLime},
    {"Cyan",    "Cn",   colCyan},
    {"Maroon",  "Mr",   colMaroon},
    {"Purple",  "Pr",   colPurple},
    {"Olive",   "Ol",   colOlive},
    {"Gray",    "Gy",   colGray},
    {"Silver",  "Sv",   colSilver},
    {"Magenta", "Mg",   colMagenta},
    {"Red",     "Rd",   colRed},
    {"Orange",  "Or",   colOrange},
    {"Pink",    "Pn",   colPink},
    {"Yellow",  "Ye",   colYellow},
    {"White",   "Wt",   colWhite},
};

/*******************************************************************************
 Local (private) Functions
*******************************************************************************/

/*******************************************************************************
 Global (public) Functions
*******************************************************************************/

esp_err_t str2rgb(uint32_t *rgb, const char * str)
{
    str = str_trim_l(str);
    uint32_t i = 0;

    for (i = 0; i < ARRAY_SIZE(_colour); i++)
    {
        if (strlen(str) == 2)
        {
            if (strcasecmp(str, _colour[i].shortName) == 0)
            {
                *rgb = _colour[i].rgb;
                return ESP_OK;
            }
        }
        else if (strcasecmp(str, _colour[i].Name) == 0)
        {
            *rgb = _colour[i].rgb;
            return ESP_OK;
        }
    }

    return ESP_ERR_NOT_FOUND;

}

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
