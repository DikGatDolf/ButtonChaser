/*****************************************************************************

halTimer.h

Include file for halTimer.c

******************************************************************************/
#ifndef __HAL_PWM_H__
#define __HAL_PWM_H__


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

bool hal_pwm_start(double target_frequency);
void hal_pwm_stop();
bool hal_pwm_enabled();
// double hal_pwm_get_frequency();
// double hal_pwm_get_period();
// uint8_t hal_pwm_get_resolution();
bool hal_pwm_assign_pin(led_colour_type col, int pin);
void hal_pwm_unassign_pin(led_colour_type col);
void hal_pwm_set_duty_cycle(led_colour_type col, uint8_t duty_cycle_target);
uint8_t hal_pwm_get_duty_cycle(led_colour_type col);
uint8_t hal_pwm_get_adj_duty_cycle(led_colour_type col);

void hal_pwm_inc_duty_cycle(led_colour_type col, uint8_t inc_val = 1, bool wrap = false);
void hal_pwm_dec_duty_cycle(led_colour_type col, uint8_t dec_val = 1, bool wrap = false);

#endif /* __HAL_PWM_H__ */

/****************************** END OF FILE **********************************/
