/******************************************************************************
Project:   RGB Button Chaser
Module:     hal_pwm.c
Purpose:    This file contains the pwm routines
Author:     Rudolph van Niekerk
Processor:  Arduino Nano (ATmega328)
Compiler:	Arduino AVR Compiler


This implements timers which has to be polled to check for expiry.
The timer will act as a one-shot timer in normal operation.
To make the timer behave as a recurring timer, reload the interval and start
the timer once it has expired (using TimerStart()).

The General Timer (ST_MS_TIMER type) - 1 kHz granularity
The general Timers enables the program to create a downcounter with a
preloaded value. This timer will then decrement every 1 ms until it has
expired.
Usage:
The module making use of the timers must host a ST_MS_TIMER structure in RAM and
add it to the linked list (TimerAdd) to ensure that it is maintained.
Removing it from the linked list (TimerRemove) will  make it dormant.
The Timer must be polled (TimerPoll) to see when it has expired

 ******************************************************************************/

#define __NOT_EXTERN__
#include "hal_pwm.h"
#undef __NOT_EXTERN__

#include "dev_console.h"
#include "Arduino.h"
#include "std_utils.h"

/*******************************************************************************
local defines
 *******************************************************************************/
#define MAX_PWM_PINS	6
#define MAX_RESOLUTION	100

static st_console_menu_item hal_pwm_menu = { "pwm", NULL, "PWM Commands"};

typedef struct
{
	uint8_t output;			// The pin number
	int8_t duty_cycle;		// The duty cycle in percent
} st_pwm_pin;

typedef struct
{
	st_pwm_pin pin[MAX_PWM_PINS];
	bool active = false;
	uint8_t pin_count = 0;
	uint16_t prescaler = 0;
	int8_t percent_inc = 0;
	int8_t dc_cnt;			// Counter to keep track of the duty cycle
} st_pwm;

 static const PROGMEM uint8_t hal_pwm_brightness[] = { };

/*******************************************************************************
local variables
 *******************************************************************************/
st_pwm hal_pwm;
volatile uint8_t hal_pwm_tcnt2;

char hal_pwm_tag[] = "[PWM]";

/*******************************************************************************
local functions
 *******************************************************************************/
ISR(TIMER2_OVF_vect) {
	TCNT2 = hal_pwm_tcnt2; //Reset the timer

	for (uint8_t i = 0; i < hal_pwm.pin_count; i++)
	{
		if (hal_pwm.pin[i].duty_cycle == hal_pwm.dc_cnt)	//Catches the case where duty-cycle is 0
			quickPinToggle(hal_pwm.pin[i].output, LOW);
		else if (0 == hal_pwm.dc_cnt)						//For all duty cycles > 0
			quickPinToggle(hal_pwm.pin[i].output, HIGH);
	}
    
    hal_pwm.dc_cnt += hal_pwm.percent_inc;
    if (hal_pwm.dc_cnt >= 100)
        hal_pwm.dc_cnt = 0;
}

void hal_pwm_menu_init() 
{
    addMenuItem(&hal_pwm_menu);
}

/*******************************************************************************

Sets the timers interval and starts it.

 *******************************************************************************/

//void usTimerInit(unsigned long ms, void (*f)()) {
//	usTimerInit(ms, 0.001, f);
//}
/*******************************************************************************

 * @param period The period of the PWM signal in seconds
 *   0.01 implies a 10 ms period.
 *   0.001 implies a 1 ms period.
 *   0.0005 implies a 0.5 ms or 500us period.
 *   0.000005 implies a 5us period.
 * @param percent_inc The number of % points each increment/decrement will affect 
 *  the duty cycle. Min 1, Max 100 (which is effectively a ON/OFF signal)
 
 *******************************************************************************/
bool hal_pwm_start(double period, int8_t percent_inc) 
{
	
	uint16_t prescale_multiplier[] = {1, 8, 32, 64, 128, 256, 1024};
	double prescale_min_freq[] = {62500.0, 		7812.5, 	1953.125, 	976.5625, 	488.2813, 	244.14, 	61.0352};
	double prescale_max_freq[] = {16000000.0, 	2000000.0, 	500000.0,	250000.0, 	125000.0, 	62500.0,	15625.0};

	uint16_t prescaler_index = 0;

	if ((hal_pwm.active) || (hal_pwm.pin_count > 0))
	{
		iPrintF(trPWM | trALWAYS, "%sAlready running with %d pin(s)\n", hal_pwm_tag, hal_pwm.pin_count);
		return false;
	} 	//Otherwise we can go ahead and re-initialize the PWM

	hal_pwm.prescaler = 0;

	//Make sure the resolution is within bounds and a factor of 100
	if (percent_inc <= 1) 
		hal_pwm.percent_inc = 1;
	else if (percent_inc >= MAX_RESOLUTION) 
		hal_pwm.percent_inc = MAX_RESOLUTION;
	else 
	{
		//If this is NOT a factor of 100, let's help the caller find the next best resolution, working our way down to 1
		while ((percent_inc > 1) && ((MAX_RESOLUTION%percent_inc) != 0))
			percent_inc--;
		hal_pwm.percent_inc = percent_inc;
	}

    //iPrintF(trPWM | trALWAYS, "%sResolution = %d\n", hal_pwm_tag, hal_pwm_get_resolution());

	double target_frequency = 100.0/period/hal_pwm.percent_inc;
	
    //iPrintF(trPWM, "%sTarget Freq = %s Hz\n", hal_pwm_tag, floatToStr(target_frequency, 2));

	//We want to find the prescaler that will give us the closest match to the desired period
	//Typically one can assume that a lower presacaler will give less error, so we start from the lowest prescaler value
	for (prescaler_index = 0; prescaler_index < 7; prescaler_index++) 
	{
		if ((target_frequency <= prescale_max_freq[prescaler_index]) && (target_frequency >= prescale_min_freq[prescaler_index])) 
		{
			hal_pwm.prescaler = prescale_multiplier[prescaler_index];
			break;
		}
	}

	if (prescaler_index >= 7) 
	{
		//We could not find a prescaler that will give us the desired period
		iPrintF(trPWM | trALWAYS, "%sCould not achieve %s Hz\n", hal_pwm_tag, floatToStr(target_frequency, 2));
		return false;
	}

    //iPrintF(trPWM | trALWAYS, "%sPrescaler = %d\n", hal_pwm_tag, hal_pwm.prescaler);

	TIMSK2 &= ~(1<<TOIE2);					// Make sure the timer is stopped
	TCCR2A &= ~((1<<WGM21) | (1<<WGM20));	// NORMAL MODE
	TCCR2B &= ~(1<<WGM22);					// NORMAL MODE
	ASSR &= ~(1<<AS2);						// Clocked from CLK/IO (thus using the prescaler)
	TIMSK2 &= ~(1<<OCIE2A);					// Prevents the timer from generating an interrupt immediately

	switch (prescaler_index)
	{
		case 0:	//F_CPU/1	(001)
			TCCR2B |= (1<<CS20);
			TCCR2B &= ~((1<<CS22) | (1<<CS21));
			break;
		case 1:	//F_CPU/8 	(010)
			TCCR2B |= (1<<CS21);
			TCCR2B &= ~((1<<CS22) | (1<<CS20));
			break;
		case 2:	//F_CPU/32	(011)
			TCCR2B |= (1<<CS21) | (1<<CS20);
			TCCR2B &= ~(1<<CS22);
			break;
		case 3:	//F_CPU/64	(100)
			TCCR2B |= (1<<CS22);
			TCCR2B &= ~((1<<CS21) | (1<<CS20));
			break;
		case 4:	//F_CPU/128	(101)
			TCCR2B |= (1<<CS22) | (1<<CS20);
			TCCR2B &= ~(1<<CS21);
			break;
		case 5:	//F_CPU/256	(110)
			TCCR2B |= (1<<CS22) | (1<<CS21);
			TCCR2B &= ~(1<<CS20);
			break;
		case 6:	//F_CPU/1024(111)
			TCCR2B |= (1<<CS22) | (1<<CS21) | (1<<CS20);
			break;
		default: //NO CLK	(000)
			TCCR2B &= ~((1<<CS22) | (1<<CS21) | (1<<CS20));
			return false;
			break;
	}

	hal_pwm_tcnt2 = (256 - (uint8_t)((float)F_CPU / target_frequency / hal_pwm.prescaler));
    //iPrintF(trPWM | trALWAYS, "%sTCNT2 = %d\n", hal_pwm_tag, hal_pwm_tcnt2);
	hal_pwm.dc_cnt = 0;

	TCNT2 = hal_pwm_tcnt2;
	TIMSK2 |= (1<<TOIE2);	//Start the timer

	hal_pwm.active = true;
	return true;
}

void hal_pwm_stop() {
	TIMSK2 &= ~(1<<TOIE2);
	hal_pwm.active = false;
}

bool hal_pwm_enabled()
{
	return hal_pwm.active;
}

double hal_pwm_get_period()
{
	return ((float)(256 - hal_pwm_tcnt2)*hal_pwm.prescaler*hal_pwm_get_resolution())/((float)F_CPU);
}

double hal_pwm_get_frequency()
{
	return (1.0/hal_pwm_get_period());
}

uint8_t hal_pwm_get_resolution()
{
	return (100/hal_pwm.percent_inc);
}

uint8_t hal_pwm_assign_pin(int pin, int8_t duty_cycle_percent)
{
	uint8_t pin_index = 0;
	// Do we have space for another pin?
	if (hal_pwm.pin_count == MAX_PWM_PINS)
	{
		iPrintF(trPWM | trALWAYS, "%sMax # of PWM pins assigned (%d)\n", hal_pwm_tag, MAX_PWM_PINS);
		return -1;
	}

	//Round the duty cycle down to the nearest resolution 
	while ((duty_cycle_percent > 0) && ((duty_cycle_percent%hal_pwm.percent_inc) != 0))
		duty_cycle_percent--;

	// Check first if the pin is not already in the list
	for (pin_index = 0; pin_index < hal_pwm.pin_count; pin_index++)
	{
		if (hal_pwm.pin[pin_index].output == pin)
		{
			iPrintF(trPWM, "\n%sPin %d already in the PWM List (%d)\n", hal_pwm_tag, pin, pin_index);
			hal_pwm.pin[pin_index].duty_cycle = duty_cycle_percent;
			return pin_index;
		}
	}

	//First make sure it is set as an output pin
	pinMode(pin, OUTPUT);

	//Now add the pin in the list
	quickPinToggle(pin, (duty_cycle_percent == 0)? LOW : HIGH);
	hal_pwm.pin[pin_index].duty_cycle = duty_cycle_percent;
	hal_pwm.pin[pin_index].output = pin;

	iPrintF(trPWM, "\n%sPin %d added to the PWM List (%d)\n", hal_pwm_tag, pin, pin_index);

	hal_pwm.pin_count++;
	return pin_index;
}

void hal_pwm_unassign_pin(int pin)
{

	bool pin_removed = false;

	// Find the pin in the list
	for (int i = 0; (i < MAX_PWM_PINS) && (hal_pwm.pin_count > 0); i++)
	{
		if (!pin_removed)
		{
			if (hal_pwm.pin[i].output == pin)
			{
				pin_removed = true;
				hal_pwm.pin[i].output = -1;
				hal_pwm.pin[i].duty_cycle = 0;
			}
		}
		else if (i < hal_pwm.pin_count)
		{
			//(pin_removed == true) is implied - means the previous slot is now empty
			hal_pwm.pin[i-1].output = hal_pwm.pin[i].output;
			hal_pwm.pin[i-1].duty_cycle = hal_pwm.pin[i].duty_cycle;
			hal_pwm.pin[i].output = -1;
			hal_pwm.pin[i].duty_cycle = 0;
		}
	}
	if (pin_removed)
		hal_pwm.pin_count--;
}

void hal_pwm_set_duty_cycle_percent(int pin, int8_t duty_cycle_percent)
{
    if (duty_cycle_percent > 100)
        duty_cycle_percent = 100;
    else if (duty_cycle_percent < 0)
        duty_cycle_percent = 0;
    else
    {
        //Round the duty cycle down to the nearest resolution 
        while ((duty_cycle_percent > 0) && ((duty_cycle_percent%hal_pwm.percent_inc) != 0))
            duty_cycle_percent--;
    }

	// Check first if the pin is not already in the list
	for (int i = 0; i < hal_pwm.pin_count; i++)
	{
		if (hal_pwm.pin[i].output == pin)
		{
			hal_pwm.pin[i].duty_cycle = duty_cycle_percent;
			break;
		}
	}
}

int8_t hal_pwm_get_duty_cycle_percent(int pin)
{
	for (int i = 0; i < hal_pwm.pin_count; i++)
	{
		if (hal_pwm.pin[i].output == pin)
		{
			return hal_pwm.pin[i].duty_cycle;
		}
	}
	
	return -1;
}

void hal_pwm_inc_duty_cycle(int pin, bool wrap)
{
	for (int i = 0; i < hal_pwm.pin_count; i++)
	{
		if (hal_pwm.pin[i].output == pin)
		{
            if (hal_pwm.pin[i].duty_cycle >= 100)
            {
                if (wrap)
                    hal_pwm.pin[i].duty_cycle = 0;
            }
            else if (hal_pwm.pin[i].duty_cycle > (100-hal_pwm.percent_inc))
                hal_pwm.pin[i].duty_cycle = 100;
            else
                hal_pwm.pin[i].duty_cycle += hal_pwm.percent_inc;
		}
	}
}

void hal_pwm_dec_duty_cycle(int pin, bool wrap)
{
	for (int i = 0; i < hal_pwm.pin_count; i++)
	{
		if (hal_pwm.pin[i].output == pin)
		{
            if (hal_pwm.pin[i].duty_cycle <= 0)
            {
                if (wrap)
                    hal_pwm.pin[i].duty_cycle = 100;
            }
            else if (hal_pwm.pin[i].duty_cycle < hal_pwm.percent_inc)
                hal_pwm.pin[i].duty_cycle = 0;
            else
                hal_pwm.pin[i].duty_cycle -= hal_pwm.percent_inc;
		}
	}
}

/*************************** END OF FILE *************************************/
