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
typedef enum {
    ledtype_Debug = 0,
    ledtype_Button = 1,
    /* Keep this at the end*/
    ledtype_max,
}rgb_led_strip_type;

/******************************************************************************
Global (public) variables
******************************************************************************/

/******************************************************************************
Global (public) function definitions
******************************************************************************/

uint32_t drv_rgb_led_strip_init(void);
void drv_rgb_led_strip_deinit(void);

uint32_t drv_rgb_led_strip_pixels(rgb_led_strip_type strip);

void drv_rgb_led_strip_set_rgb_pixel(rgb_led_strip_type strip, uint32_t pixel_index, uint32_t rgb);

#ifdef __cplusplus
}
#endif

#undef EXT
#endif /* drv_rgb_led_strip_H */


//**************************************************************************//
