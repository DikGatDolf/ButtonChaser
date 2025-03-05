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
definitions
******************************************************************************/

/******************************************************************************
Macros
******************************************************************************/

/******************************************************************************
Struct & Unions
******************************************************************************/

/******************************************************************************
variables
******************************************************************************/

/******************************************************************************
functions
******************************************************************************/

bool hal_pwm_start(double period, int8_t percent_inc = 10);
void hal_pwm_stop();
bool hal_pwm_enabled();
// double hal_pwm_get_frequency();
// double hal_pwm_get_period();
// uint8_t hal_pwm_get_resolution();
uint8_t hal_pwm_assign_pin(int pin, int8_t duty_cycle_percent = 50);
void hal_pwm_unassign_pin(int pin);
void hal_pwm_set_duty_cycle_percent(int pin, int8_t duty_cycle_percent);
int8_t hal_pwm_get_duty_cycle_percent(int pin);
int8_t hal_pwm_get_adj_duty_cycle_percent(int pin);

void hal_pwm_inc_duty_cycle(int pin, bool wrap = false);
void hal_pwm_dec_duty_cycle(int pin, bool wrap = false);

#endif /* __HAL_PWM_H__ */

/****************************** END OF FILE **********************************/
