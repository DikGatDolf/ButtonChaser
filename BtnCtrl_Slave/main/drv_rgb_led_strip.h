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

/******************************************************************************
includes
******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

#include "driver/rmt_tx.h"
#include <stdint.h>
#include "defines.h"

/******************************************************************************
Macros
******************************************************************************/

void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b);
void drv_rgb_led_strip_init(void);
void drv_rgb_led_strip_deinit(void);

void drv_rgb_led_strip_set_rgb_pixel(uint32_t index, uint32_t red, uint32_t green, uint32_t blue);
void drv_rgb_led_strip_clear_rgb_pixel(uint32_t index);
//void drv_rgb_led_strip_set_hsv(uint32_t hue, uint32_t saturation, uint32_t value);

#ifdef __cplusplus
}
#endif

#undef EXT
#endif /* drv_rgb_led_strip_H */


//**************************************************************************//
