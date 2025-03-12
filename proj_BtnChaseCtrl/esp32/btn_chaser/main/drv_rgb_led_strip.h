/*****************************************************************************

drv_rgb_led_strip.h

Include file for drv_rgb_led_strip.c

******************************************************************************/
#ifndef __drv_rgb_led_strip_H__
#define __drv_rgb_led_strip_H__

#ifdef __NOT_EXTERN__
#define EXT
#else
#define EXT extern
#endif /* __NOT_EXTERN__ */

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
includes
******************************************************************************/
#include "driver/rmt_tx.h"
#include <stdint.h>
#include "defines.h"
#include "colour.h"

/******************************************************************************
Macros
******************************************************************************/

/******************************************************************************
Struct & Unions
******************************************************************************/

/******************************************************************************
Global (public) variables
******************************************************************************/

/******************************************************************************
Global (public) function definitions
******************************************************************************/

/*! Initialise the LED strip driver
 * @return The total number of LEDs initialised in all strips
 */
uint32_t drv_rgb_led_strip_init(void);

/*! Deinitialize the LED strip driver
 */
void drv_rgb_led_strip_deinit(void);

/*! @brief Sets the colour of the LED at the specified index
 * @param _index The index of the LED to set the colour for
 * @param _rgb The 24-bit RGB value to set the LED to
 */
void drv_rgb_led_strip_set_colour(int led_index, uint32_t rgb);

/*! Returns a pointer to a string with the name of the LED strip type
 * @param[in] led_index The index of the LED to set the colour for
 * @return A pointer to the string with the name of the LED strip type
 */
const char * drv_rgb_led_strip_index2name(int led_index);

/*! Returns the index and the number of LEDs referenced by the name
 * @param[in] name The name of the LED strip. This can be the name of the 
                strip, or the name of the strip and the number of the LED, 
                in the format "<strip_name>[:<nr>]"
 * @param[out] led_index if successfully parsed, contains the index of the 
                LED strip in the list
 * @param[out] led_cnt if successfully parsed, contains the number of LEDs. 
                If ":<nr>" was omitted from the string, this will be the 
                total number of LEDs in the strip"
 * @return true if the name was successfully parsed, false otherwise
 */
 bool drv_rgb_led_strip_name2index(const char * name, int * led_index,  int * led_cnt);

#ifdef __cplusplus
}
#endif

#undef EXT
#endif /* drv_rgb_led_strip_H */


//**************************************************************************//
