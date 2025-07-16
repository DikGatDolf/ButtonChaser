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
#include "task_console.h"

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

 /*! @brief Parses the arguments in the string into H, S or V values
 * @param[in] str The string containing the arguments to parse (in the 
 *  format  "HSV:<H>[,<S>[,<V>]]")
 * @param[out] value The 24-bit RGB value if successful or an empty string 
 *  was received, otherwise unchanged
 * @param[in] limit The maximum value allowed for the element (H=360, S=V=100)
 * @param[in] type_str The type of element being parsed
 * @return ESP_OK if successful (or an empty string was received), otherwise an error code
 */
esp_err_t _parse_colour_hsv(char * str, uint32_t *value, uint32_t limit, const char * type_str);

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
esp_err_t _parse_colour_hsv(char * str, uint32_t *value, uint32_t limit, const char * type_str)
{
    if (value == NULL)
    {
        iprintln(trALWAYS, "Invalid dst ptr passed (NULL) - %s", type_str);
        return ESP_ERR_INVALID_ARG;
    }

    //Special case: if no string pointer is passed, then we return OK, but change nothing.
    if (str == NULL)
        return ESP_OK;

    if (str2uint32(value, str, 0))
    {
        if (*value < limit)
            return ESP_OK;

        //Value is over the limit
        iprintln(trALWAYS, "Invalid %s value (%d > %d)", type_str, *value, limit);
        return ESP_ERR_INVALID_ARG;
    }
    //Could not parse the string as a number
    iprintln(trALWAYS, "Invalid %s value (\"%s\")", type_str, str);
    return ESP_ERR_INVALID_ARG;
}

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

esp_err_t parse_str_to_colour(uint32_t * colour_value, const char * str)
{
    // uint32_t rgb_col_24bit;
    uint32_t hue_value = HUE_MAX, sat_value = SAT_MAX, val_value = VAL_MAX;
    const char * hue_str = NULL;
    char * sat_str = NULL;
    char * val_str = NULL;
    esp_err_t err = ESP_OK;

    //Righto, a colour can be in one of two valid formats:
    // 1. NOT USED: A 6-character hexadecimal string (0xHHHHHH), e.g. "0x000001", "#0F0F0F", etc
    // 2. A string containing any one of the assigned colour names, e.g. "Black", "wh", etc
    // 3. A HSV string degrees provided as HSV:<h>[,<s>[,<v>]]"

    if (str2rgb(colour_value, str) == ESP_OK)
        return ESP_OK;

    if (strncasecmp("HSV:", str, 4) == 0)
    {
        hue_str = str+4;
        sat_str = strchr(hue_str, ',');
        if (sat_str != NULL)
        {
            *sat_str = 0;
            sat_str++;
            val_str = strchr(sat_str, ',');
            if (val_str != NULL)
            {
                *val_str = 0;
                val_str++;
            }
        }
        
        err = _parse_colour_hsv((char *)hue_str, &hue_value, HUE_MAX, "Hue");
        if (err != ESP_OK)
            return err;
        err = _parse_colour_hsv(sat_str, &sat_value, SAT_MAX, "Saturation %%");
        if (err != ESP_OK)
            return err;
        err = _parse_colour_hsv(val_str, &val_value, VAL_MAX, "Value %%");
        if (err != ESP_OK)
            return err;

        hsv2rgb(hue_value, sat_value, val_value, colour_value);
        return ESP_OK;
    }

    //Reaching this point means this is NOT a hex RGB colour or an HSV value
    return ESP_ERR_NOT_FOUND;
}

#undef PRINTF_TAG

/*************************** END OF FILE *************************************/
