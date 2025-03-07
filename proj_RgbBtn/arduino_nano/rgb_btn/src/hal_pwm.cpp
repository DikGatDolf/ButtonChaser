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

The General Timer (Timer_ms_t type) - 1 kHz granularity
The general Timers enables the program to create a downcounter with a
preloaded value. This timer will then decrement every 1 ms until it has
expired.
Usage:
The module making use of the timers must host a Timer_ms_t structure in RAM and
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
#include "str_helper.h"

#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("PWM") /* This must be undefined at the end of the file*/

/*******************************************************************************
local defines
 *******************************************************************************/
/* At most this "driver" should really not be driving more than 4 LEDs: Red, Green, Blue and White.*/
#define MAX_PWM_PINS	            (4U)
#define PWM_PIN_UNASSIGNED          (255U)  /* -1 */
/* This is currently the maximum, dictated by the size of the variables holding the PWM values, which are all uint8_t, i.e. 0 to 255 */
#define PWM_RESOLUTION_BITS	        (8U)   

#define PWM_RESOLUTION	            (BIT_POS(PWM_RESOLUTION_BITS))
#define PWM_MAX_VALUE	            (PWM_RESOLUTION-1)

#ifdef CONSOLE_ENABLED
const static char * _pwm_led_col_str[] = {"Blue", "Green", "Red", "White" };
#endif /* CONSOLE_ENABLED */

typedef struct
{
//	uint8_t pin_nr;			    // The pin number
	uint8_t duty_cycle_target;	// The target duty cycle (0 to 255)
	uint8_t duty_cycle_adj;		// The adjusted duty cycle (0 to 255)
	uint8_t duty_cycle_on;	    // The value at which the pin should be turned on - Used to offset switching on LEDs to avoid inrush current
	uint8_t duty_cycle_off;		// The value at which the pin should be turned off - Used to offset switching on LEDs to avoid inrush current
} pwm_pin_type;

typedef struct
{
	uint8_t      pin;	// The pin number
	pwm_pin_type pwm;   // The target duty cycle (0 to 255)
} rgbw_led_type;

typedef struct
{
    rgbw_led_type led[eLED_MAX];
	bool active = false;
	uint16_t prescaler = 0;
	uint8_t dc_cnt;			// Counter to keep track of the duty cycle
    double act_freq;        // The actual frequency of the PWM signal
} st_led_pwm;

/*******************************************************************************
local function definitions
 *******************************************************************************/
#ifdef CONSOLE_ENABLED
void _hal_pwm_menu_handler(void);
void _print_colour_state(led_colour_type col);

/*******************************************************************************
local variables
 *******************************************************************************/
static console_menu_item_t _hal_pwm_menu_items[] =
{
    { "pwm", _hal_pwm_menu_handler, "PWM Commands"},
};
#endif /* CONSOLE_ENABLED */

st_led_pwm hal_pwm;

volatile uint8_t hal_pwm_tcnt2;

/* Human perceived brightness is not linear... so we use a lookup table to 
    create the logarithmic curve for increasing brightness. 
    This lookup table is actually just using the perimeter of a circle: 
        x^2 + y^2 = 100^2
    It contains only 99 entries since 0 and 100 should be... well, 0 and 100.
*/

#ifndef PWM_USE_CIE_LUT_ROUNDED
/* Blatantly copied from https://blog.mbedded.ninja/programming/firmware/controlling-led-brightness-using-pwm/ */
static const PROGMEM uint8_t CIE_LIGHTNESS_TO_PWM_LUT_256_IN_8BIT_OUT[] = {
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1,    1,    1,    1,    1,    1,
    1,    1,    1,    2,    2,    2,    2,    2,    2,    2,    2,    3,    3,    3,    3,    3,
    3,    3,    4,    4,    4,    4,    4,    5,    5,    5,    5,    5,    6,    6,    6,    6,
    6,    7,    7,    7,    7,    8,    8,    8,    8,    9,    9,    9,   10,   10,   10,   11,
   11,   11,   12,   12,   12,   13,   13,   13,   14,   14,   14,   15,   15,   16,   16,   16,
   17,   17,   18,   18,   19,   19,   20,   20,   21,   21,   22,   22,   23,   23,   24,   24,
   25,   25,   26,   26,   27,   28,   28,   29,   29,   30,   31,   31,   32,   33,   33,   34,
   35,   35,   36,   37,   37,   38,   39,   40,   40,   41,   42,   43,   44,   44,   45,   46,
   47,   48,   49,   49,   50,   51,   52,   53,   54,   55,   56,   57,   58,   59,   60,   61,
   62,   63,   64,   65,   66,   67,   68,   69,   70,   71,   72,   73,   75,   76,   77,   78,
   79,   80,   82,   83,   84,   85,   87,   88,   89,   90,   92,   93,   94,   96,   97,   99,
  100,  101,  103,  104,  106,  107,  108,  110,  111,  113,  114,  116,  118,  119,  121,  122,
  124,  125,  127,  129,  130,  132,  134,  135,  137,  139,  141,  142,  144,  146,  148,  149,
  151,  153,  155,  157,  159,  161,  162,  164,  166,  168,  170,  172,  174,  176,  178,  180,
  182,  185,  187,  189,  191,  193,  195,  197,  200,  202,  204,  206,  208,  211,  213,  215,
  218,  220,  222,  225,  227,  230,  232,  234,  237,  239,  242,  244,  247,  249,  252,  255,
};
#else
/* Blatantly copied from https://blog.mbedded.ninja/programming/firmware/controlling-led-brightness-using-pwm/ */
static const PROGMEM uint8_t CIE_LIGHTNESS_TO_PWM_LUT_256_IN_8BIT_OUT[] = {
    0,    0,    0,    0,    0,    1,    1,    1,    1,    1,    1,    1,    1,    1,    2,    2,
    2,    2,    2,    2,    2,    2,    2,    3,    3,    3,    3,    3,    3,    3,    3,    4,
    4,    4,    4,    4,    4,    5,    5,    5,    5,    5,    6,    6,    6,    6,    6,    7,
    7,    7,    7,    8,    8,    8,    8,    9,    9,    9,   10,   10,   10,   10,   11,   11,
   11,   12,   12,   12,   13,   13,   13,   14,   14,   15,   15,   15,   16,   16,   17,   17,
   17,   18,   18,   19,   19,   20,   20,   21,   21,   22,   22,   23,   23,   24,   24,   25,
   25,   26,   26,   27,   28,   28,   29,   29,   30,   31,   31,   32,   32,   33,   34,   34,
   35,   36,   37,   37,   38,   39,   39,   40,   41,   42,   43,   43,   44,   45,   46,   47,
   47,   48,   49,   50,   51,   52,   53,   54,   54,   55,   56,   57,   58,   59,   60,   61,
   62,   63,   64,   65,   66,   67,   68,   70,   71,   72,   73,   74,   75,   76,   77,   79,
   80,   81,   82,   83,   85,   86,   87,   88,   90,   91,   92,   94,   95,   96,   98,   99,
  100,  102,  103,  105,  106,  108,  109,  110,  112,  113,  115,  116,  118,  120,  121,  123,
  124,  126,  128,  129,  131,  132,  134,  136,  138,  139,  141,  143,  145,  146,  148,  150,
  152,  154,  155,  157,  159,  161,  163,  165,  167,  169,  171,  173,  175,  177,  179,  181,
  183,  185,  187,  189,  191,  193,  196,  198,  200,  202,  204,  207,  209,  211,  214,  216,
  218,  220,  223,  225,  228,  230,  232,  235,  237,  240,  242,  245,  247,  250,  252,  255,
};
#endif

// static const PROGMEM uint8_t hal_pwm_brightness[] = { 
//      0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
//      1, 1, 1, 1, 1, 1, 1, 2, 2, 2,
//      2, 2, 3, 3, 3, 3, 4, 4, 4, 5,
//      5, 5, 6, 6, 6, 7, 7, 8, 8, 8,
//      9, 9,10,10,11,11,12,12,13,13,
//     14,15,15,16,16,17,18,19,19,20,
//     21,22,22,23,24,25,26,27,28,29,
//     30,31,32,33,34,35,36,37,39,40,
//     41,43,44,46,47,49,51,53,54,56,
//     59,61,63,66,69,72,76,80,86
//  };

/*******************************************************************************
local functions
 *******************************************************************************/
ISR(TIMER2_OVF_vect) {
	TCNT2 = hal_pwm_tcnt2; //Reset the timer

    //We only take action on the "EDGE" of the PWM signal....
	for (uint8_t i = 0; i < eLED_MAX; i++)
	{
        if (hal_pwm.led[i].pin == PWM_PIN_UNASSIGNED)
            continue;
        
        if (hal_pwm.led[i].pwm.duty_cycle_off == hal_pwm.dc_cnt)	        //Catches the case where duty-cycle is 0
			quickPinToggle(hal_pwm.led[i].pin, LOW);
		else if (hal_pwm.led[i].pwm.duty_cycle_on == hal_pwm.dc_cnt)	        //Catches the case where duty-cycle is 0
			quickPinToggle(hal_pwm.led[i].pin, HIGH);
	}
    
    hal_pwm.dc_cnt++; //This should wrap around at 256 automagically
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
bool hal_pwm_start(double target_frequency) 
{

    //ASSERTM(PWM_RESOLUTION_BITS <= 8U, "PWM resolution is limited to 8 bits (0 to 255)");

    // 100 Hz x 256 resolution = 25600 => LED flicker =  3.90625 ms => Int. Rate =  39.0625 us
    //  50 Hz x 256 resolution = 12800 => LED flicker =  7.81250 ms => Int. Rate =  78.1250 us
    //  25 Hz x 256 resolution =  6400 => LED flicker = 15.62500 ms => Int. Rate = 156.2500 us
    //  10 Hz x 256 resolution =  2560 => LED flicker = 31.25000 ms => Int. Rate = 312.5000 us
    
    char freq_str[16];
    int pin_count = 0;
	uint16_t prescale_multiplier[] = {1, 8, 32, 64, 128, 256, 1024};
	double prescale_min_freq[] = {62500.0, 		7812.5, 	1953.125, 	976.5625, 	488.2813, 	244.14, 	61.0352};
	double prescale_max_freq[] = {16000000.0, 	2000000.0, 	500000.0,	250000.0, 	125000.0, 	62500.0,	15625.0};

	uint16_t prescaler_index = 0;

#ifdef CONSOLE_ENABLED
    console_add_menu("hal", _hal_pwm_menu_items, ARRAY_SIZE(_hal_pwm_menu_items), "PWM Commands");
#endif /* CONSOLE_ENABLED */

    if (hal_pwm.active)
    {
        for (int pin_index = 0; pin_index < eLED_MAX; pin_index++)
            if (hal_pwm.led[pin_index].pin != PWM_PIN_UNASSIGNED)
                pin_count++;

        if (!float2str(freq_str, hal_pwm.act_freq, 2, 16))
            snprintf(freq_str, 16, "%d.????", (int)(hal_pwm.act_freq));

        if (pin_count != 0)
            iprintln(trPWM | trALWAYS, "#Already running with %d colour(s) @ %s Hz", pin_count, freq_str);
    }
    else
    {
        for (int pin_index = 0; pin_index < eLED_MAX; pin_index++)
            hal_pwm.led[pin_index].pin = PWM_PIN_UNASSIGNED;
    }

    //Otherwise we can go ahead and (re?)initialize the PWM

	hal_pwm.prescaler = 0;

    //iprintln(trPWM | trALWAYS, "#Resolution = %d", hal_pwm_get_resolution());

	//double target_frequency = 100.0/period;
    //double period = 1.0/target_frequency;
    target_frequency = target_frequency*PWM_RESOLUTION;
	
    if (!float2str(freq_str, target_frequency, 2, 16))
        snprintf(freq_str, 16, "%d.????", (int)(target_frequency/PWM_RESOLUTION));
    //iprintln(trPWM, "#Target Freq = %s Hz", freq_str);

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

    //iprintln(trPWM, "#prescaler: %d (value: %d)", prescaler_index, hal_pwm.prescaler);

	if (prescaler_index >= 7) 
	{
        char freq_str[16];
        if (!float2str(freq_str, target_frequency, 2, 16))
            snprintf(freq_str, 16, "%d.????", (int)target_frequency);
		//We could not find a prescaler that will give us the desired period
		iprintln(trPWM | trALWAYS, "#Could not achieve %s Hz", freq_str);
		return false;
	}

    //iprintln(trPWM | trALWAYS, "#Prescaler = %d", hal_pwm.prescaler);

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

	hal_pwm_tcnt2 = (uint8_t)(256 - (unsigned int)((float)F_CPU / target_frequency / hal_pwm.prescaler));
    //iprintln(trPWM | trALWAYS, "#TCNT2 = %d", hal_pwm_tcnt2);
	hal_pwm.dc_cnt = 0;

    //iprintln(trPWM, "#TCNT2 = %d", hal_pwm_tcnt2);
    //Do this before enabling the timer interrupts (avoid accessing hal_pwm_tcnt2 outside the ISR)
    hal_pwm.act_freq = ((float)F_CPU)/((float)(256 - hal_pwm_tcnt2)*hal_pwm.prescaler*(PWM_RESOLUTION));

	TCNT2 = hal_pwm_tcnt2;
	TIMSK2 |= (1<<TOIE2);	//Start the timer

	hal_pwm.active = true;

    if (!float2str(freq_str, hal_pwm.act_freq, 2, 16))
        snprintf(freq_str, 16, "%d.????", (int)(hal_pwm.act_freq));
    iprintln(trPWM|trALWAYS, "#PWM is running at %s Hz (%d bit resolution)", freq_str, PWM_RESOLUTION_BITS);
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

bool hal_pwm_assign_pin(led_colour_type col, int pin)
{
    if (col >= eLED_MAX)
    {
        iprintln(trPWM | trALWAYS, "#Invalid LED colour (%d)", col);
        return false;
    }

    if (pin >= NUM_DIGITAL_PINS)
    {
        iprintln(trPWM, "#Invalid pin number (%d)", pin);
        return false;
    }

    if (hal_pwm.led[col].pin == pin)
    {
        iprintln(trPWM, "#%s already assigned Pin %d", _pwm_led_col_str[col], pin);
        return true;
    }

    //If the pin is already assigned to another colour, we have to "unassign" it there first
    for (int i = 0; i < eLED_MAX; i++)
    {
        if (hal_pwm.led[i].pin == pin)
        {
            iprint(trPWM, "#Pin %d", pin);
            iprint(trPWM, " is being re-assigned: ");
            iprintln(trPWM, "%s -> %s", _pwm_led_col_str[i], _pwm_led_col_str[col]);
            hal_pwm.led[i].pin = PWM_PIN_UNASSIGNED;
        }
    }

    //Should also make sure that a pin is not being "duplicated on another colour"
    if ((hal_pwm.led[col].pin != pin) && (hal_pwm.led[col].pin != PWM_PIN_UNASSIGNED))
    {
        iprint(trPWM, "#%s", _pwm_led_col_str[col]);
        iprint(trPWM, " is being re-assigned: ");
        iprintln(trPWM, "%u -> %d", hal_pwm.led[col].pin, pin);

    }

	//First make sure it is set as an output pin
	pinMode(pin, OUTPUT);

	//Now assign the pin in the list
    hal_pwm.led[col].pin = pin;

	iprintln(trPWM, "#%s assigned Pin %d", _pwm_led_col_str[col], pin);

    return true;
}

void hal_pwm_unassign_pin(led_colour_type col)
{
    if (col >= eLED_MAX)
    {
        iprintln(trPWM, "#Invalid LED colour (%d)", col);
        return;
    }

    if (hal_pwm.led[col].pin == PWM_PIN_UNASSIGNED)
    {
        iprintln(trPWM, "#%s is not assigned", _pwm_led_col_str[col]);
        return;
    }

    // Should we reset the pin to INPUT?
	//pinMode(hal_pwm.led[col].pin, INPUT_PULLUP);

    hal_pwm.led[col].pin = PWM_PIN_UNASSIGNED;
}

void hal_pwm_set_duty_cycle(led_colour_type col, uint8_t duty_cycle_target)
{
    if (col >= eLED_MAX)
        return;
    if (hal_pwm.led[col].pin == PWM_PIN_UNASSIGNED)
        return;

    hal_pwm.led[col].pwm.duty_cycle_target = duty_cycle_target;
    hal_pwm.led[col].pwm.duty_cycle_adj = pgm_read_byte(&CIE_LIGHTNESS_TO_PWM_LUT_256_IN_8BIT_OUT[duty_cycle_target]);
    //Stagger the switchon point of the LEDs to avoid inrush current
    hal_pwm.led[col].pwm.duty_cycle_on = col*(PWM_RESOLUTION/4);
    hal_pwm.led[col].pwm.duty_cycle_off = hal_pwm.led[col].pwm.duty_cycle_on + hal_pwm.led[col].pwm.duty_cycle_adj;
}

uint8_t hal_pwm_get_duty_cycle(led_colour_type col)
{
    if (col >= eLED_MAX)
    {
        iprintln(trPWM, "#Invalid LED colour (%d)", col);
        return 0;
    }

    if (hal_pwm.led[col].pin == PWM_PIN_UNASSIGNED)
    {
        iprintln(trPWM, "#%s is not assigned", _pwm_led_col_str[col]);
        return 0;
    }

    return hal_pwm.led[col].pwm.duty_cycle_target;
}
uint8_t hal_pwm_get_adj_duty_cycle(led_colour_type col)
{
    if (col >= eLED_MAX)
    {
        iprintln(trPWM, "#Invalid LED colour (%d)", col);
        return 0;
    }

    if (hal_pwm.led[col].pin == PWM_PIN_UNASSIGNED)
    {
        iprintln(trPWM, "#%s is not assigned", _pwm_led_col_str[col]);
        return 0;
    }

    return hal_pwm.led[col].pwm.duty_cycle_adj;
}

void hal_pwm_inc_duty_cycle(led_colour_type col, uint8_t inc_val, bool wrap)
{
    if (col >= eLED_MAX)
    {
        iprintln(trPWM, "#Invalid LED colour (%d)", col);
        return;
    }

    if (hal_pwm.led[col].pin == PWM_PIN_UNASSIGNED)
    {
        iprintln(trPWM, "#%s is not assigned", _pwm_led_col_str[col]);
        return;
    }

    if ((inc_val >= (PWM_RESOLUTION-hal_pwm.led[col].pwm.duty_cycle_target)) && (!wrap))
        hal_pwm.led[col].pwm.duty_cycle_target = PWM_MAX_VALUE;
    else //wrap around if bigger, otherwise, no care
        hal_pwm.led[col].pwm.duty_cycle_target += inc_val;
    hal_pwm.led[col].pwm.duty_cycle_adj = pgm_read_byte(&CIE_LIGHTNESS_TO_PWM_LUT_256_IN_8BIT_OUT[hal_pwm.led[col].pwm.duty_cycle_target]);
}

void hal_pwm_dec_duty_cycle(led_colour_type col, uint8_t dec_val, bool wrap)
{
    if (col >= eLED_MAX)
    {
        iprintln(trPWM, "#Invalid LED colour (%d)", col);
        return;
    }

    if (hal_pwm.led[col].pin == PWM_PIN_UNASSIGNED)
    {
        iprintln(trPWM, "#%s is not assigned", _pwm_led_col_str[col]);
        return;
    }

    if ((dec_val >= hal_pwm.led[col].pwm.duty_cycle_target) && (!wrap))
        hal_pwm.led[col].pwm.duty_cycle_target = 0;
    else //wrap around if bigger, otherwise, no care
        hal_pwm.led[col].pwm.duty_cycle_target -= dec_val;
    hal_pwm.led[col].pwm.duty_cycle_adj = pgm_read_byte(&CIE_LIGHTNESS_TO_PWM_LUT_256_IN_8BIT_OUT[hal_pwm.led[col].pwm.duty_cycle_target]);
}

#ifdef CONSOLE_ENABLED
void _hal_pwm_menu_handler(void)
{
    char *argStr;// = console_arg_pop();
    bool help_reqested = false;
    led_colour_type set_col = eLED_MAX;     //Indicates "not set"
    int set_pwm_dc = -1;                //Indicates "not set"


    if (console_arg_cnt() == 0) // Show the current state of the PWM
    {
        iprint(trALWAYS, "PWM is ");
        if(hal_pwm.active)
        {
            char freq_str[16];
            if (!float2str(freq_str, hal_pwm.act_freq, 2, 16))
                snprintf(freq_str, 16, "%d.????", (int)(hal_pwm.act_freq));
            iprintln(trALWAYS, "running at %s Hz (%d bit Resolution)", freq_str, PWM_RESOLUTION_BITS);
        }
        else
        {
            iprintln(trALWAYS, "stopped");
        }
        for (int i = 0; i < eLED_MAX; i++)
            _print_colour_state((led_colour_type)i);
        return;
    }

    //Go through every available argument
    while (console_arg_cnt() > 0)
    {
        argStr = console_arg_pop();
        if ((0 == strcasecmp(argStr, "help")) || (0 == strcasecmp(argStr, "?"))) //is it a ? or help
        {
            help_reqested = true; //Disregard the rest of the arguments
            break; //from while
        }
        else if (is_natural_number_str(argStr, 0)) //is it a duty cycle number 
        {
            int pwm_dc = atoi(argStr);

            if ((pwm_dc < 0) || (pwm_dc > (int)PWM_RESOLUTION))
            {
                iprintln(trALWAYS, "Please specify a duty cycle value from 0 to %d (got %d)", PWM_MAX_VALUE, pwm_dc);
                help_reqested = true; //Disregard the rest of the arguments
                break; //from while
            }
            set_pwm_dc = pwm_dc;
        }
        else //could be colour?
        {
            led_colour_type col = eLED_MAX;
        
            //Get the colour match first
            for (int i = 0; i < eLED_MAX; i++)
            {
                //We only match as much as the user has typed.... not much to be confused beween Red, Green White and Blue
                if (strncasecmp(_pwm_led_col_str[i], argStr, strlen(argStr)) == 0)
                {
                    col = (led_colour_type)i;
                    break; //from the for loop
                }
            }
    
            if (col == eLED_MAX)
            {
                // For "All" we can apply the given duty cycle to all assigned colours/pins
                if (strcasecmp("All", argStr) != 0)
                {
                    iprint(trALWAYS, "Please specify a valid LED colour: ");
                    for (int i = 0; i < eLED_MAX; i++)
                        iprint(trALWAYS, "\"%s\", ", _pwm_led_col_str[i]);
                    iprintln(trALWAYS, " or \"All\" (got \"%s\")", argStr);
                    help_reqested = true; //Disregard the rest of the arguments
                    break; //from while
                }
            }
            set_col = col;
        }
    }

    if (!help_reqested)
    {
            
        for (int i = 0; i < eLED_MAX; i++)
        {    
            if (set_pwm_dc == -1) //just display the state of colour(s)
            {
                if ((set_col == eLED_MAX) || (set_col == i))
                    _print_colour_state((led_colour_type)i);
            }
            else    // 0 <= set_pwm_dc <= PWM_MAX_VALUE
            {
                // Set the duty cycle of the colour(s)
                if (((set_col == eLED_MAX) || (set_col == i)) && (hal_pwm.led[i].pin != PWM_PIN_UNASSIGNED))
                {
                    hal_pwm_set_duty_cycle((led_colour_type)i, set_pwm_dc);
                    _print_colour_state((led_colour_type)i);                            
                }
            }
        }
    }
    
    if (help_reqested)
    {
        iprintln(trALWAYS, " Usage: \"pwm [<colour> [<duty_cycle>]]\":");
        iprintln(trALWAYS, "    <colour>     - Red, Green, Blue, White or All");
        iprintln(trALWAYS, "          if omitted, \"All\" is assumed");
        iprintln(trALWAYS, "    <duty_cycle> - Sets an %u-bit PWM value (0-%u) for <colour>", PWM_RESOLUTION_BITS, PWM_RESOLUTION);
        iprintln(trALWAYS, "          if omitted, the pwm of the (or all) colour are displayed");
    }
}

void _print_colour_state(led_colour_type col)
{
    iprint(trALWAYS, "  %5s - ", _pwm_led_col_str[col]);
    if (hal_pwm.led[col].pin != PWM_PIN_UNASSIGNED)
        iprintln(trALWAYS, "Pin %2d : %3d (%3d)", hal_pwm.led[col].pin, hal_pwm.led[col].pwm.duty_cycle_target, hal_pwm.led[col].pwm.duty_cycle_adj);
    else
        iprintln(trALWAYS, "Unassigned");
}
#endif /* CONSOLE_ENABLED */

#undef PRINTF_TAG
/*************************** END OF FILE *************************************/
