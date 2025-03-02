/*****************************************************************************

colour.h

Include file for colour.c

******************************************************************************/
#ifndef __colour_H__
#define __colour_H__


/******************************************************************************
includes
******************************************************************************/


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
#define AS_RGB(r, g, b) (0x00FFFFFF & (((r) << 16) | ((g) << 8) | (b)))
#define AS_WRGB(w, r, g, b) ((((w) << 24) | AS_RGB(r, g, b)))

#define WHITE_from_WRGB(x)   ((uint8_t)((x >> 24)& 0xFF))
#define RED_from_WRGB(x)     ((uint8_t)((x >> 16) & 0xFF))
#define GREEN_from_WRGB(x)   ((uint8_t)((x >> 8) & 0xFF))
#define BLUE_from_WRGB(x)    ((uint8_t)((x) & 0xFF))

#define HUE_MAX 360
#define SAT_MAX 100
#define VAL_MAX 100

/******************************************************************************
Struct & Unions
******************************************************************************/
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
    colOlive    = AS_RGB(128, 128, 0),
    colGray     = AS_RGB(128, 128, 128),
    colSilver   = AS_RGB(192, 192, 192),
    colMagenta  = AS_RGB(255, 0,   255),
    colRed      = AS_RGB(255, 0,   0),
    colOrange   = AS_RGB(255, 165, 0),
    colPink     = AS_RGB(255, 192, 203),    
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
Global (public) variables
******************************************************************************/

/******************************************************************************
Global (public) function definitions
******************************************************************************/
/*! @brief A public function that converts a string to a 24-bit RGB value
 * @param rgb Pointer to a uint32_t, which will contain the 24-bit RGB value
 *  if the conversion was successful
 * @param str The string to convert (e.g. Black, Red, etc), or its short form 
 *  (e.g. Bk, Rd, etc)
 * @return ESP_OK if the conversion was successful, ESP_ERR_NOT_FOUND if the 
 *  string was not found
 */
esp_err_t str2rgb(uint32_t *rgb, const char * str);

/*! @brief Simple helper function, converting HSV color space to RGB color space
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 * @param h Hue, 0-360
 * @param s Saturation, 0-100
 * @param v Value, 0-100
 * @param rgb Pointer to the 24-bit Red-Green-Blue value
 */
void hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *rgb);

#undef EXT
#endif /* __colour_H__ */

/****************************** END OF FILE **********************************/
