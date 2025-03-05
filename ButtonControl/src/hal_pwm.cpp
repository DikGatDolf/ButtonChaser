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

typedef struct
{
	uint8_t pin_nr;			    // The pin number
	int8_t duty_cycle_target;	// The target duty cycle (in percent)
	int8_t duty_cycle_adj;		// The adjusted duty cycle (in percent)
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

#ifdef CONSOLE_ENABLED
bool menuPWM(void);
#endif /* CONSOLE_ENABLED */
void set_adjusted_duty_cycle(uint8_t index, int8_t target_dc);

/*******************************************************************************
local variables
 *******************************************************************************/
#ifdef CONSOLE_ENABLED
static st_console_menu_item hal_pwm_menu = { "pwm", menuPWM, "PWM Commands"};
#endif /* CONSOLE_ENABLED */

st_pwm hal_pwm;
volatile uint8_t hal_pwm_tcnt2;

char hal_pwm_tag[] = "[PWM]";

/* Human perceived brightness is not linear... so we use a lookup table to 
    create the logarithmic curve for increasing brightness. 
    This lookup table is actually just using the perimeter of a circle: 
        x^2 + y^2 = 100^2
    It contains only 99 entries since 0 and 100 should be... well, 0 and 100.
*/
 static const PROGMEM uint8_t hal_pwm_brightness[] = { 
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 1, 1, 1, 1, 1, 1, 2, 2, 2,
     2, 2, 3, 3, 3, 3, 4, 4, 4, 5,
     5, 5, 6, 6, 6, 7, 7, 8, 8, 8,
     9, 9,10,10,11,11,12,12,13,13,
    14,15,15,16,16,17,18,19,19,20,
    21,22,22,23,24,25,26,27,28,29,
    30,31,32,33,34,35,36,37,39,40,
    41,43,44,46,47,49,51,53,54,56,
    59,61,63,66,69,72,76,80,86
 };

/*******************************************************************************
local functions
 *******************************************************************************/
ISR(TIMER2_OVF_vect) {
	TCNT2 = hal_pwm_tcnt2; //Reset the timer

	for (uint8_t i = 0; i < hal_pwm.pin_count; i++)
	{
		if (hal_pwm.pin[i].duty_cycle_adj == hal_pwm.dc_cnt)	//Catches the case where duty-cycle is 0
			quickPinToggle(hal_pwm.pin[i].pin_nr, LOW);
		else if (0 == hal_pwm.dc_cnt)						//For all duty cycles > 0
			quickPinToggle(hal_pwm.pin[i].pin_nr, HIGH);
	}
    
    hal_pwm.dc_cnt += hal_pwm.percent_inc;
    if (hal_pwm.dc_cnt >= MAX_RESOLUTION)
        hal_pwm.dc_cnt = 0;
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

#ifdef CONSOLE_ENABLED
    addMenuItem(&hal_pwm_menu);
#endif /* CONSOLE_ENABLED */

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

    iPrintF(trALWAYS, "PWM is ");
    PrintF("running at %s Hz (%d%% Resolution)", floatToStr(((float)F_CPU)/((float)(256 - hal_pwm_tcnt2)*hal_pwm.prescaler*(MAX_RESOLUTION/hal_pwm.percent_inc)), 2), hal_pwm.percent_inc);
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

// double hal_pwm_get_period()
// {
// 	return ((float)(256 - hal_pwm_tcnt2)*hal_pwm.prescaler*hal_pwm_get_resolution())/((float)F_CPU);
// }

// double hal_pwm_get_frequency()
// {
// 	return (1.0/hal_pwm_get_period());
// }

// uint8_t hal_pwm_get_resolution()
// {
// 	return (MAX_RESOLUTION/hal_pwm.percent_inc);
// }

uint8_t hal_pwm_assign_pin(int pin, int8_t duty_cycle_percent)
{
	uint8_t pin_index = 0;
	// Do we have space for another pin?
	if (hal_pwm.pin_count == MAX_PWM_PINS)
	{
		iPrintF(trPWM | trALWAYS, "%sMax # of PWM pins assigned (%d)\n", hal_pwm_tag, MAX_PWM_PINS);
		return -1;
	}

	// Check first if the pin is not already in the list
	for (pin_index = 0; pin_index < hal_pwm.pin_count; pin_index++)
	{
		if (hal_pwm.pin[pin_index].pin_nr == pin)
		{
			iPrintF(trPWM, "\n%sPin %d already added (%d)\n", hal_pwm_tag, pin, pin_index);
            //Just change the duty cycle
            set_adjusted_duty_cycle(pin_index, duty_cycle_percent);
			return pin_index;
		}
	}

	//First make sure it is set as an output pin
	pinMode(pin, OUTPUT);

	//Now add the pin in the list
	quickPinToggle(pin, (duty_cycle_percent == 0)? LOW : HIGH);
	hal_pwm.pin[pin_index].duty_cycle_adj = duty_cycle_percent;
	hal_pwm.pin[pin_index].pin_nr = pin;

	iPrintF(trPWM, "\n%sPin %d added (%d)\n", hal_pwm_tag, pin, pin_index);

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
			if (hal_pwm.pin[i].pin_nr == pin)
			{
				pin_removed = true;
				hal_pwm.pin[i].pin_nr = -1;
				hal_pwm.pin[i].duty_cycle_adj = 0;
                hal_pwm.pin[i].duty_cycle_target = 0;
			}
		}
		else if (i < hal_pwm.pin_count)
		{
			//(pin_removed == true) is implied - means the previous slot is now empty
			hal_pwm.pin[i-1].pin_nr = hal_pwm.pin[i].pin_nr;
			hal_pwm.pin[i-1].duty_cycle_adj = hal_pwm.pin[i].duty_cycle_adj;
			hal_pwm.pin[i-1].duty_cycle_target = hal_pwm.pin[i].duty_cycle_target;
			hal_pwm.pin[i].pin_nr = -1;
			hal_pwm.pin[i].duty_cycle_adj = 0;
            hal_pwm.pin[i].duty_cycle_target = 0;
		}
	}
	if (pin_removed)
		hal_pwm.pin_count--;
}

void hal_pwm_set_duty_cycle_percent(int pin, int8_t duty_cycle_percent)
{
	// Check first if the pin is not already in the list
	for (int i = 0; i < hal_pwm.pin_count; i++)
	{
		if (hal_pwm.pin[i].pin_nr == pin)
		{
            set_adjusted_duty_cycle(i, duty_cycle_percent);
			break;
		}
	}
}

void set_adjusted_duty_cycle(uint8_t index, int8_t target_dc)
{
    int8_t res;

    if (index >= hal_pwm.pin_count)
        return;

    if (target_dc >= MAX_RESOLUTION)
    {
        hal_pwm.pin[index].duty_cycle_target = hal_pwm.pin[index].duty_cycle_adj = MAX_RESOLUTION;
    }
    else if (target_dc <= 0)
    {
        hal_pwm.pin[index].duty_cycle_target = hal_pwm.pin[index].duty_cycle_adj = 0;
    }
    else
    {
        hal_pwm.pin[index].duty_cycle_target = target_dc;
        //Do we need to round up/down?
        res = (target_dc%hal_pwm.percent_inc);

        if (res > 0)
        {
            //If the remainder is more than (or equal to) half the resolution, we round UP
            if ((2*res) >= hal_pwm.percent_inc)
                target_dc += hal_pwm.percent_inc;
            //else we round DOWN

            target_dc -= res;
        }

        //Read the adjusted value from the lookup table using the rounded value -1 (since 0 is the first entry and it is already catered for)
        hal_pwm.pin[index].duty_cycle_adj = pgm_read_byte(&hal_pwm_brightness[target_dc-1]);
    }
    //return hal_pwm.pin[index].duty_cycle_adj;
}

int8_t hal_pwm_get_duty_cycle_percent(int pin)
{
	for (int i = 0; i < hal_pwm.pin_count; i++)
	{
		if (hal_pwm.pin[i].pin_nr == pin)
		{
			return hal_pwm.pin[i].duty_cycle_target;
		}
	}
	
	return -1;
}
int8_t hal_pwm_get_adj_duty_cycle_percent(int pin)
{
	for (int i = 0; i < hal_pwm.pin_count; i++)
	{
		if (hal_pwm.pin[i].pin_nr == pin)
		{
			return hal_pwm.pin[i].duty_cycle_adj;
		}
	}
	
	return -1;
}

void hal_pwm_inc_duty_cycle(int pin, bool wrap)
{
	for (int i = 0; i < hal_pwm.pin_count; i++)
	{
		if (hal_pwm.pin[i].pin_nr == pin)
		{
            if (hal_pwm.pin[i].duty_cycle_adj >= MAX_RESOLUTION)
            {
                if (wrap)
                    hal_pwm.pin[i].duty_cycle_adj = 0;
            }
            else if (hal_pwm.pin[i].duty_cycle_adj > (MAX_RESOLUTION-hal_pwm.percent_inc))
                hal_pwm.pin[i].duty_cycle_adj = MAX_RESOLUTION;
            else
                hal_pwm.pin[i].duty_cycle_adj += hal_pwm.percent_inc;
		}
	}
}

void hal_pwm_dec_duty_cycle(int pin, bool wrap)
{
	for (int i = 0; i < hal_pwm.pin_count; i++)
	{
		if (hal_pwm.pin[i].pin_nr == pin)
		{
            if (hal_pwm.pin[i].duty_cycle_adj <= 0)
            {
                if (wrap)
                    hal_pwm.pin[i].duty_cycle_adj = MAX_RESOLUTION;
            }
            else if (hal_pwm.pin[i].duty_cycle_adj < hal_pwm.percent_inc)
                hal_pwm.pin[i].duty_cycle_adj = 0;
            else
                hal_pwm.pin[i].duty_cycle_adj -= hal_pwm.percent_inc;
		}
	}
}

#ifdef CONSOLE_ENABLED
bool menuPWM(void)
{
    char *argStr = paramsGetNext();
    bool help_reqested = help_req();

    if (!help_reqested)
    {
        if (!argStr) // Show the current state of the PWM
        {
            PrintF("PWM is ");
            if(hal_pwm.active)
            {
                PrintF("running at %s Hz (%d%% Resolution)", floatToStr(((float)F_CPU)/((float)(256 - hal_pwm_tcnt2)*hal_pwm.prescaler*(MAX_RESOLUTION/hal_pwm.percent_inc)), 2), hal_pwm.percent_inc);
            }
            else
            {
                PrintF("stopped");
            } 
            PrintF("\n");
            if (hal_pwm.pin_count > 0)
            {
                PrintF("\t%d/%d ", hal_pwm.pin_count, MAX_PWM_PINS);
                PrintF("PWM pins");
                PrintF(":");
                PrintF("\n");
            	for (int i = 0; (i < hal_pwm.pin_count) && (hal_pwm.pin_count > 0); i++)
                {
                    if (hal_pwm.pin[i].pin_nr >= 0)
                        PrintF("\t\t%d) Pin %d : %d%% (%d%%)\n", i, hal_pwm.pin[i].pin_nr, hal_pwm.pin[i].duty_cycle_target, hal_pwm.pin[i].duty_cycle_adj);
                }
            }
            else
            {
                PrintF("\tNO ");
                PrintF("PWM pins");
                PrintF(" assigned");
            }
            PrintF("\n");
        }
        //else got "pwm <something>"
        else if (0 == strcasecmp(argStr, "?")) //is it a ?
        {
            help_reqested = true; //Disregard the rest of the arguments
        }
        else if (isNaturalNumberStr(argStr)) //is it a (pin) number 
        {
            //We are expecting 1 or 2 arguments... the pwm pin number and a duty cycle percentage
            uint8_t pwmPinNum = atoi(argStr);
            st_pwm_pin *pwmPin = NULL;
            int8_t dc_target;
            int8_t dc_adj;

            for (int i = 0; i < hal_pwm.pin_count; i++)
            {
                if (hal_pwm.pin[i].pin_nr == pwmPinNum)
                    pwmPin = &hal_pwm.pin[i];
            }

            if (NULL == pwmPin)
            {
                PrintF("Please specify a valid PWM-assigned pin nr [");
                for (int i = 0; i < hal_pwm.pin_count; i++)
                {
                    PrintF("%d", hal_pwm.pin[i].pin_nr);
                    if (i < (hal_pwm.pin_count-1))
                        PrintF(", ");
                    else
                        PrintF("] (got %d)\n", pwmPinNum);
                }
                PrintF("\n");
                return true;
            }

            PrintF("PWM Pin %d identified\n", pwmPin->pin_nr);
            dc_target = pwmPin->duty_cycle_target;
            dc_adj = pwmPin->duty_cycle_adj;

            argStr = paramsGetNext();
            if (argStr)
            {
                if (isNaturalNumberStr(argStr)) //is it a valid duty cycle percentage
                {
                    int pwm_percent = atoi(argStr);

                    if ((pwm_percent < 0) || (pwm_percent > MAX_RESOLUTION))
                    {
                        PrintF("Please specify a duty cycle percentage from 0 and %d", MAX_RESOLUTION);
                        PrintF(" (got %d)\n", pwm_percent);
                    }
                    else
                    {
                        hal_pwm_set_duty_cycle_percent(pwmPin->pin_nr, pwm_percent);
                            
                        PrintF("PWM Pin %d : %d%% (%d%%)", pwmPin->pin_nr, dc_target, dc_adj);
                        PrintF(" -> %d%% (%d%%)\n", pwmPin->duty_cycle_target, pwmPin->duty_cycle_adj);
                    }
                }
                else
                {
                    PrintF("Invalid argument \"%s\"\n", argStr);
                    PrintF("Please specify a duty cycle percentage from 0 and %d", MAX_RESOLUTION);
                    PrintF("\n");
                }
                PrintF("\n");
                return true;
            }
            //else //No arguments passed, 
            //just display the state of the pin
            PrintF("PWM Pin %d : %d%% (%d%%)", pwmPin->pin_nr, dc_target, dc_adj);
            PrintF("\n");
            PrintF("\n");
            return true;
        }
        else
        {
            help_reqested = true;
            PrintF("Invalid argument \"%s\"\n", argStr);
            PrintF("\n");
        }
    }

    if (help_reqested)
    {
        PrintF("Valid PWM arguments are:\n");
        PrintF(" \"\" - Displays the state of ALL assigned pins\n");
        PrintF(" \"<#>\" - Displays state of pin #\n");
        PrintF(" \"<#> <DUTY_CYCLE>\" - Sets the PWM duty cycle %% (0-100) for pin #\n");
        PrintF("\n");
        return true;
    }
    return false;
}
#endif /* CONSOLE_ENABLED */

/*************************** END OF FILE *************************************/
