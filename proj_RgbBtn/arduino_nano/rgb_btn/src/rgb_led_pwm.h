/*****************************************************************************

rgb_led_pwm.h

Include file for rgb_led_pwm.c

******************************************************************************/
#ifndef __rgb_led_pwm_H__
#define __rgb_led_pwm_H__


/******************************************************************************
includes
******************************************************************************/
#include "Arduino.h"

/******************************************************************************
Macros
******************************************************************************/

/******************************************************************************
definitions
******************************************************************************/
/* The order is selected to match a W-R-G-B order in a uint32_t dword so that
 * the colours can be easily indexed using (u32_col_word >> (e_led_colour*8))*/
typedef enum e_led_colour
{
    eLED_Blue   = 0,
    eLED_Green  = 1,
    eLED_Red    = 2,
    eLED_White  = 3,
    eLED_MAX    = 4
}led_colour_type;

/******************************************************************************
Struct & Unions
******************************************************************************/

/******************************************************************************
variables
******************************************************************************/

/******************************************************************************
functions
******************************************************************************/

bool rgb_led_pwm_start(double target_frequency);
void rgb_led_pwm_stop();
bool rgb_led_pwm_enabled();
// double rgb_led_pwm_get_frequency();
// double rgb_led_pwm_get_period();
// uint8_t rgb_led_pwm_get_resolution();
bool rgb_led_pwm_assign_pin(led_colour_type col, int pin);
void rgb_led_pwm_unassign_pin(led_colour_type col);
void rgb_led_pwm_set_duty_cycle(led_colour_type col, uint8_t duty_cycle_target);
// uint8_t rgb_led_pwm_get_duty_cycle(led_colour_type col);
// uint8_t rgb_led_pwm_get_adj_duty_cycle(led_colour_type col);
// void rgb_led_pwm_inc_duty_cycle(led_colour_type col, uint8_t inc_val = 1, bool wrap = false);
// void rgb_led_pwm_dec_duty_cycle(led_colour_type col, uint8_t dec_val = 1, bool wrap = false);

#endif /* __rgb_led_pwm_H__ */

/****************************** END OF FILE **********************************/
