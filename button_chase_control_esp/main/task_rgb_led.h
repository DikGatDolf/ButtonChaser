/*****************************************************************************

task_rgb_led.h

Include file for task_rgb_led.c

******************************************************************************/
#ifndef __task_rgb_led_H__
#define __task_rgb_led_H__


/******************************************************************************
includes
******************************************************************************/
#include "defines.h"

/******************************************************************************
Macros
******************************************************************************/
#ifdef __NOT_EXTERN__
#define EXT
#else
#define EXT extern
#endif /* __NOT_EXTERN__ */

/******************************************************************************
variables
******************************************************************************/

/******************************************************************************
Global (public) Function prototypes
******************************************************************************/

/*! Initialises the RGB driving task
 * @return A pointer to the taskInfo structure for the console (if successful)
            otherwise NULL
 */
void * rgb_led_init_task(void);

/*! Turns RGB LEDs off
 * @param[in] address_mask The address mask of the LED to turn off
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t rgb_led_off(uint16_t address_mask);

/*! Turns RGB LEDs on
 * @param[in] address_mask The address mask of the LED to turn on
 * @param[in] rgb_colour The 24-bit RGB colour value to set the LED to
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t rgb_led_on(uint16_t address_mask, uint32_t rgb_colour);

/*! Sets RGB LEDs to blink between two colours
 * IMPORTANT: This function can be called from ANY task, since it will 
  * queue a message to the RGB task and return immediately
 * @param[in] address_mask The address mask of the LED to blink
 * @param[in] period The period of the blink in milliseconds (min 200ms)
 * @param[in] rgb_colour_1 The 24-bit RGB colour value to set the LED to. 
 *              Set to -1 for a randomized colour to be selected
 * @param[in] rgb_colour_2 The 24-bit RGB colour value to set the LED to
 *              Set to -1 to select the complement of colour 1)
 * @return ESP_OK if msg was queued successful, otherwise an error code
 */
esp_err_t rgb_led_blink(uint16_t address_mask, uint32_t  period, uint32_t rgb_colour_1, uint32_t rgb_colour_2);

/*! Sets RGB LEDs to cycle through the rainbow (Demo Mode)
 * @param[in] address_mask The address mask of the LEDs to put in demo-mode
 * @param[in] period The period of the blink in milliseconds (min 200ms)
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t rgb_led_demo(uint16_t address_mask, uint32_t  period);

#undef EXT
#endif /* __task_rgb_led_H__ */

/****************************** END OF FILE **********************************/
