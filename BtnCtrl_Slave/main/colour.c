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
#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("Colour") /* This must be undefined at the end of the file*/

//#define SWOP_U64(x, y) do { uint64_t(x) _z = x; x = y; y = _z; } while(0)
#define MAX3(a, b, c)     (((a) > (b) ? (((a) > (c) ? (a) : (c))) : (((b) > (c) ? (b) : (c)))))
#define MIN3(a, b, c)     (((a) < (b) ? (((a) < (c) ? (a) : (c))) : (((b) < (c) ? (b) : (c)))))

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
//    {"Olive",   "Ol",   colOlive},
//    {"Gray",    "Gy",   colGray},
//    {"Silver",  "Sv",   colSilver},
    {"Magenta", "Mg",   colMagenta},
    {"Red",     "Rd",   colRed},
    {"Orange",  "Or",   colOrange},
//    {"Pink",    "Pn",   colPink},
    {"Yellow",  "Ye",   colYellow},
    {"White",   "Wt",   colWhite},
};

/*******************************************************************************
 Local (private) Functions
*******************************************************************************/

/*******************************************************************************
 Global (public) Functions
*******************************************************************************/

const char * rgb2name(uint32_t rgb_value)
{
    
    for (uint32_t i = 0; i < ARRAY_SIZE(_colour); i++)
    {
        if (_colour[i].rgb == rgb_value)
        {
            return _colour[i].Name;
        }
    }
    return NULL;
}

const char *  colour_list_item(int index)
{

    if (index < 0 || index >= ARRAY_SIZE(_colour))
        return NULL;

    return _colour[index].Name;
}

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

void rgb2hsv(uint32_t rgb, uint32_t *h, uint32_t *s, uint32_t *v)
{
    //int32_t out_h, out_s, out_v; 
    int r = (int)RED_from_WRGB(rgb);    //Range 0 to 255
    int g = (int)GREEN_from_WRGB(rgb);  //Range 0 to 255
    int b = (int)BLUE_from_WRGB(rgb);   //Range 0 to 255
    //hsv         out;
    uint32_t min, max, delta;
    double h_temp = 0.0f;

    min = MIN3(r, g, b);                //Min. value of RGB (range 0-255)
    max = MAX3(r, g, b);                //Max. value of RGB (range 1-255)
    delta = max - min;                  // Delta RGB value (range 0 to 255)

    //Value %
    if (v)
        *v = (uint32_t)(100.0f * max / 0xff);        // Range 0 to 100

    //If the numbers are all the same, then delta will be 0 and the hue and saturation are 0
    if (delta == 0)
    {
        //This solves a lot of division problems in the "else" below
        if (s)
            *s = 0;
        if (h)
            *h = 0;
    }
    else // delta > 0
    {
        //Saturation %
        if (s)
            *s = (uint32_t)(100.0f * delta / max);     // Range 0 to 100

        if (h)
        {
            //Hue (0 to 360 degrees)
            if( r == max )      // R is max - hue sits between yellow (60) & magenta (300)
                h_temp = HUE_RED + ((HUE_MAX/6.0f) * ((int)(g - b)) / delta);   
                //     =     360 + (     (60)      * (-255 to +255) / (0-255))    -> Range 300 to 420 (60)
            else if( g == max ) // G is max - hue sits between cyan (180) & yellow (60)
                h_temp = HUE_GRN + ((HUE_MAX/6.0f) * ((int)(b - r)) / delta);
                //     =     120 + (     (60)      * (int)(-255 to +255) / (0-255))    -> Range 60 to 180
            else                // B is max - hue sits between magenta & cyan
                h_temp = HUE_BLU + ((HUE_MAX/6.0f) * ((int)(r - g)) / delta);
                //     =     240 + (     (60)      * (int)(-255 to +255) / (0-255))    -> Range 180 to 300

            // Wrap >360 values back into range
            *h = (((uint32_t)h_temp) % HUE_MAX);
        }
    }
}



#undef PRINTF_TAG

/*************************** END OF FILE *************************************/
