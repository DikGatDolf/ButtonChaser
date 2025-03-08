/******************************************************************************
Project:   RGB Button Chaser
Module:     rgb_led_pwm.c
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
#include "rgb_led_pwm.h"
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
const static char * _led_col_str[] = {"Blue", "Green", "Red", "White" };
#endif /* CONSOLE_ENABLED */

typedef struct
{
//	uint8_t pin_nr;			    // The pin number
	uint8_t target;	    // The target duty cycle (0 to 255)
	uint8_t adjust;		// The adjusted duty cycle (0 to 255)
	uint8_t _on;	    // The value at which the pin should be turned on - Used to offset switching on LEDs to avoid inrush current
	uint8_t _off;		// The value at which the pin should be turned off - Used to offset switching on LEDs to avoid inrush current
} pwm_pin_type;

typedef struct
{
	uint8_t      pin;	// The pin number
	pwm_pin_type pwm;   // The target duty cycle (0 to 255)
} colour_pwm_type;

typedef struct
{
    colour_pwm_type colour[eLED_MAX];
	bool active = false;
	uint16_t prescaler = 0;
	uint8_t pin_cnt;			// Counter to keep track of the duty cycle
	uint8_t dc_cnt;			// Counter to keep track of the duty cycle
    double act_freq;        // The actual frequency of the PWM signal
} rgb_led_pwm_type;

/*******************************************************************************
local function definitions
 *******************************************************************************/
void _set_adjusted_duty_cycle(led_colour_type col);

#ifdef CONSOLE_ENABLED
void _rgb_led_pwm_menu_handler(void);
void _print_colour_state(led_colour_type col);
/*******************************************************************************
local variables
 *******************************************************************************/
static console_menu_item_t _rgb_led_pwm_menu_items[] =
{
    { "pwm", _rgb_led_pwm_menu_handler, "PWM Commands"},
};
#endif /* CONSOLE_ENABLED */

rgb_led_pwm_type _rgb;

volatile uint8_t rgb_led_pwm_tcnt2;

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

/*******************************************************************************
local functions
 *******************************************************************************/
ISR(TIMER2_OVF_vect) {
	TCNT2 = rgb_led_pwm_tcnt2; //Reset the timer

    //We only take action on the "EDGE" of the PWM signal....
	for (uint8_t i = 0; i < eLED_MAX; i++)
	{
        if (_rgb.colour[i].pin == PWM_PIN_UNASSIGNED)
            continue;
        
        //We only bother with turning OFF if the duty cycle is not 100%
        if ((_rgb.colour[i].pwm._off == _rgb.dc_cnt) && (_rgb.colour[i].pwm.adjust < PWM_MAX_VALUE))
			quickPinToggle(_rgb.colour[i].pin, LOW);
        
        //We only bother with turning ON if the duty cycle is not 0%
		else if ((_rgb.colour[i].pwm._on == _rgb.dc_cnt) && (_rgb.colour[i].pwm.adjust > 0))
			quickPinToggle(_rgb.colour[i].pin, HIGH);
	}
    
    /*This should wrap around at 255, and not the "magical" 256=0 point. 
        Why is that? I hear you ask....

    Well, the pwm duty cycle is a value from 0 to 255 (8 bits), so at 0, the 
     PWM is supposed to be ALWAYS off, BUT at 255, the LED is supposed to be 
     ALWAYS on.
    So, if we wrap around at 256, we will have a small drop at 255 where the 
     LED will turn off and then back ON at the next cycle of 0.... 
     ...we do not want that! */
    if ((++_rgb.dc_cnt) >= PWM_MAX_VALUE)
        _rgb.dc_cnt = 0;

    //RVN - Easy to test this on a scope!!!
}


void _set_adjusted_duty_cycle(led_colour_type col)
{
    if (col >= eLED_MAX)
        return;
    if (_rgb.colour[col].pin == PWM_PIN_UNASSIGNED)
        return;
    
    _rgb.colour[col].pwm.adjust = pgm_read_byte(&CIE_LIGHTNESS_TO_PWM_LUT_256_IN_8BIT_OUT[_rgb.colour[col].pwm.target]);
    
    //The switch ON time for each LED has already been staggered through the PWM cycle. 
    //We just need to set the switch OFF time from there
    _rgb.colour[col].pwm._off = _rgb.colour[col].pwm._on + _rgb.colour[col].pwm.adjust;
    // If the OFF time is at 255, move it to 0 since 
    if (_rgb.colour[col].pwm._off >= PWM_MAX_VALUE)
        _rgb.colour[col].pwm._off = 0;
    
    // if (_rgb.colour[col].pwm.adjust == PWM_MAX_VALUE)
    //     quickPinToggle(_rgb.colour[col].pin, HIGH); //Turn ON immediately at 100% duty cycle
    // else if (_rgb.colour[col].pwm.adjust == 0)
    //     quickPinToggle(_rgb.colour[col].pin, LOW);  //Turn OFF immediately at 0% duty cycle
}

#ifdef CONSOLE_ENABLED
void _rgb_led_pwm_menu_handler(void)
{
    char *argStr;// = console_arg_pop();
    bool help_reqested = false;
    //led_colour_type set_col = eLED_MAX;             //Indicates "not set"
    uint8_t col_mask = 0x00;                    //Indicates "not set"
    int col_pwm[eLED_MAX] = {-1, -1, -1, -1};   //Indicates "not set"
    //int set_pwm_dc = -1;                //Indicates "not set"


    if (console_arg_cnt() == 0) // Show the current state of the PWM
    {
        iprint(trALWAYS, "PWM is ");
        if(_rgb.active)
        {
            char freq_str[16];
            if (!float2str(freq_str, _rgb.act_freq, 2, 16))
                snprintf(freq_str, 16, "%d.????", (int)(_rgb.act_freq));
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
        col_mask &= 0x0F; //Clear the high nibble
        if ((0 == strcasecmp(argStr, "help")) || (0 == strcasecmp(argStr, "?"))) //is it a ? or help
        {
            help_reqested = true; //Disregard the rest of the arguments
            break; //from while
        }
        else if ((is_natural_number_str(argStr, 0)) || (0 == strcasecmp(argStr, "+")) || (0 == strcasecmp(argStr, "-")))//is it a duty cycle number 
        {
            //Does the argument start with a plus or minus sign?
            bool increment = false;
            bool decrement = false;
            int pwm_dc = 0;
            if (argStr[0] == '+')
            {
                //We are onlny incrementing the duty cycle
                increment = true;
                argStr++; //Skip the + sign

            }
            else if (argStr[0] == '-')
            {
                //We are only decrementing the duty cycle
                decrement = true;
                argStr++; //Skip the - sign
            }
            if (*argStr != 0) //We have a duty cycle value
                pwm_dc = atoi(argStr); //Should always be > 0 as we skipped the sign character above
            else if (increment || decrement)
                pwm_dc = 1; //Default increment value
            else
                pwm_dc = -1; // Will cause the next check to fail

            if ((pwm_dc < 0) || (pwm_dc > (int)PWM_MAX_VALUE))
            {
                iprintln(trALWAYS, "Please specify a duty cycle value from 0 to %d (got %d)", PWM_MAX_VALUE, pwm_dc);
                help_reqested = true; //Disregard the rest of the arguments
                break; //from while
            }
            if (col_mask == 0x00) //No colour specified, just a duty cycle
                col_mask = 0x0F;

            // Setting ALL the colours to the same duty cycle
            for (int i = 0; i < eLED_MAX; i++)
            {
                if ((col_mask & BIT_POS(i)) == 0)
                    continue;
                if (col_pwm[i] != -1)
                    iprintln(trALWAYS, "%s's duty cycle is being overwrittend: %d -> %d", _led_col_str[i], col_pwm[i], pwm_dc);
                
                if (increment)
                    col_pwm[i] = min(_rgb.colour[i].pwm.target + pwm_dc, (int)PWM_MAX_VALUE);
                else if (decrement)
                    col_pwm[i] = max(_rgb.colour[i].pwm.target - pwm_dc, 0);
                else
                    col_pwm[i] = pwm_dc;
                col_mask &= ~BIT_POS(i); //clear bit in case the user wants to set the same duty cycle to more colours
            }
        }
        else //could be colour?
        {
            led_colour_type col = eLED_MAX;
        
            //Get the colour match first
            for (int i = 0; i < eLED_MAX; i++)
            {
                //We only match as much as the user has typed.... not much to be confused beween Red, Green White and Blue
                if (strncasecmp(_led_col_str[i], argStr, strlen(argStr)) == 0)
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
                        iprint(trALWAYS, "\"%s\", ", _led_col_str[i]);
                    iprintln(trALWAYS, " or \"All\" (got \"%s\")", argStr);
                    help_reqested = true; //Disregard the rest of the arguments
                    break; //from while
                }
                else
                {
                    col_mask |= 0x0F; //Set all colours
                    continue; //to the next argument
                }
            }
            //Are we duplicating a colour?
            if ((col_mask & BIT_POS(col)) != 0)
                iprintln(trALWAYS, "%s duplicated in this action (perhaps an errant \"All\"?)", _led_col_str[col]);

            // if (last_col != eLED_MAX)
            // {
            //     //Means we got a colour without a duty cycle
            //     iprintln(trALWAYS, "Please specify a duty cycle value for %s", _led_col_str[last_col]);
            //     help_reqested = true; //Disregard the rest of the arguments
            //     break; //from while
            // }
            //set_col = col;
            col_mask |= BIT_POS(col);
            // last_col = col;
        }
    }

    if (!help_reqested)
    {
            
        for (int i = 0; i < eLED_MAX; i++)
        {
            if (col_pwm[i] != -1)
            {
                // Set the duty cycle of the colour(s)
                if  (_rgb.colour[i].pin != PWM_PIN_UNASSIGNED)
                    rgb_led_pwm_set_duty_cycle((led_colour_type)i, col_pwm[i]);
                _print_colour_state((led_colour_type)i);                            
            }
        }
        for (int i = 0; i < eLED_MAX; i++)
        {
            if (col_mask & BIT_POS(i))
            {
                //just display the state of colour(s)
                _print_colour_state((led_colour_type)i);
            }
        }
    }
    
    if (help_reqested)
    {
        iprintln(trALWAYS, " Usage: \"pwm [<colour>] [[<+/->]<duty_cycle>]\":");
        iprintln(trALWAYS, "    <colour>     - Red, Green, Blue, White or All");
        iprintln(trALWAYS, "          if <colour> is omitted, \"All\" is assumed");
        iprintln(trALWAYS, "    <+/-><duty_cycle> - Sets a delta %u-bit PWM value (0-%u) for <colour>", PWM_RESOLUTION_BITS, PWM_RESOLUTION);
        iprintln(trALWAYS, "          if <+/-> is omitted, an absolute value is assumed");
        iprintln(trALWAYS, "          if <+/-><duty_cycle> is omitted, the pwm of the colouris displayed");
        //                //          1         2         3         4         5         6         7         8         9
        //                //0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
    }
}

void _print_colour_state(led_colour_type col)
{
    iprint(trALWAYS, "  %5s - ", _led_col_str[col]);
    if (_rgb.colour[col].pin != PWM_PIN_UNASSIGNED)
        iprintln(trALWAYS, "Pin %2d : %3d (%3d) -> ON = %d, OFF = %d", _rgb.colour[col].pin, _rgb.colour[col].pwm.target, _rgb.colour[col].pwm.adjust, _rgb.colour[col].pwm._on, _rgb.colour[col].pwm._off);
    else
        iprintln(trALWAYS, "Unassigned");
}
#endif /* CONSOLE_ENABLED */

/*******************************************************************************
Public functions
 *******************************************************************************/

bool rgb_led_pwm_start(double target_frequency) 
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
    console_add_menu("hal", _rgb_led_pwm_menu_items, ARRAY_SIZE(_rgb_led_pwm_menu_items), "PWM Commands");
#endif /* CONSOLE_ENABLED */

    if (_rgb.active)
    {
        for (int pin_index = 0; pin_index < eLED_MAX; pin_index++)
            if (_rgb.colour[pin_index].pin != PWM_PIN_UNASSIGNED)
                pin_count++;

        if (!float2str(freq_str, _rgb.act_freq, 2, 16))
            snprintf(freq_str, 16, "%d.????", (int)(_rgb.act_freq));

        if (pin_count != 0)
            iprintln(trPWM | trALWAYS, "#Already running with %d colour(s) @ %s Hz", pin_count, freq_str);
    }
    else
    {
        _rgb.pin_cnt = 0;
        for (int pin_index = 0; pin_index < eLED_MAX; pin_index++)
            _rgb.colour[pin_index].pin = PWM_PIN_UNASSIGNED;
    }

    //Otherwise we can go ahead and (re?)initialize the PWM

	_rgb.prescaler = 0;

    //iprintln(trPWM | trALWAYS, "#Resolution = %d", rgb_led_pwm_get_resolution());

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
			_rgb.prescaler = prescale_multiplier[prescaler_index];
			break;
		}
	}

    //iprintln(trPWM, "#prescaler: %d (value: %d)", prescaler_index, _rgb.prescaler);

	if (prescaler_index >= 7) 
	{
        char freq_str[16];
        if (!float2str(freq_str, target_frequency, 2, 16))
            snprintf(freq_str, 16, "%d.????", (int)target_frequency);
		//We could not find a prescaler that will give us the desired period
		iprintln(trPWM | trALWAYS, "#Could not achieve %s Hz", freq_str);
		return false;
	}

    //iprintln(trPWM | trALWAYS, "#Prescaler = %d", _rgb.prescaler);

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

	rgb_led_pwm_tcnt2 = (uint8_t)(256 - (unsigned int)((float)F_CPU / target_frequency / _rgb.prescaler));
    //iprintln(trPWM | trALWAYS, "#TCNT2 = %d", rgb_led_pwm_tcnt2);
	_rgb.dc_cnt = 0;

    //iprintln(trPWM, "#TCNT2 = %d", rgb_led_pwm_tcnt2);
    //Do this before enabling the timer interrupts (avoid accessing rgb_led_pwm_tcnt2 outside the ISR)
    _rgb.act_freq = ((float)F_CPU)/((float)(256 - rgb_led_pwm_tcnt2)*_rgb.prescaler*(PWM_RESOLUTION));

	TCNT2 = rgb_led_pwm_tcnt2;
	TIMSK2 |= (1<<TOIE2);	//Start the timer

	_rgb.active = true;

    if (!float2str(freq_str, _rgb.act_freq, 2, 16))
        snprintf(freq_str, 16, "%d.????", (int)(_rgb.act_freq));
    iprintln(trPWM|trALWAYS, "#PWM is running at %s Hz (%d bit resolution)", freq_str, PWM_RESOLUTION_BITS);
	return true;
}

void rgb_led_pwm_stop() {
	TIMSK2 &= ~(1<<TOIE2);
	_rgb.active = false;
}

bool rgb_led_pwm_enabled()
{
	return _rgb.active;
}

bool rgb_led_pwm_assign_pin(led_colour_type col, int pin)
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

    if (_rgb.colour[col].pin == pin)
    {
        iprintln(trPWM, "#%s already assigned Pin %d", _led_col_str[col], pin);
        return true;
    }

    //If the pin is already assigned to another colour, we have to "unassign" it there first
    for (int i = 0; i < eLED_MAX; i++)
    {
        if (_rgb.colour[i].pin == pin)
        {
            iprint(trPWM, "#Pin %d", pin);
            iprint(trPWM, " is being re-assigned: ");
            iprintln(trPWM, "%s -> %s", _led_col_str[i], _led_col_str[col]);
            _rgb.colour[i].pin = PWM_PIN_UNASSIGNED;
            _rgb.pin_cnt--;
        }
    }

    if (_rgb.pin_cnt >= MAX_PWM_PINS)
    {
        iprintln(trPWM, "#%d pins already assigned, unassign one first", MAX_PWM_PINS);
        for (int i = 0; i < eLED_MAX; i++)
            _print_colour_state((led_colour_type)i);
        return false;
    }

    //Should also make sure that a pin is not being "duplicated on another colour"
    if ((_rgb.colour[col].pin != pin) && (_rgb.colour[col].pin != PWM_PIN_UNASSIGNED))
    {
        iprint(trPWM, "#%s", _led_col_str[col]);
        iprint(trPWM, " is being re-assigned: ");
        iprintln(trPWM, "%u -> %d", _rgb.colour[col].pin, pin);

    }

	//First make sure it is set as an output pin
	pinMode(pin, OUTPUT);

	//Now assign the pin in the list
    _rgb.colour[col].pin = pin;
    _rgb.pin_cnt++;

    //We want to stagger the switchon point of the LEDs to avoid inrush current
    //We do this by spreading the switchon point of the assigned LEDs over the PWM cycle
    int delta = (PWM_RESOLUTION/_rgb.pin_cnt);
    int pin_cnt = 0;
    for (int i = 0; i < eLED_MAX; i++)
    {
        if (_rgb.colour[i].pin != PWM_PIN_UNASSIGNED)
        {
            _rgb.colour[i].pwm._on = pin_cnt*delta;
            pin_cnt++;
        }
    }

	iprintln(trPWM, "#%s assigned Pin %d", _led_col_str[col], pin);

    return true;
}

void rgb_led_pwm_unassign_pin(led_colour_type col)
{
    if (col >= eLED_MAX)
    {
        iprintln(trPWM, "#Invalid LED colour (%d)", col);
        return;
    }

    if (_rgb.colour[col].pin == PWM_PIN_UNASSIGNED)
    {
        iprintln(trPWM, "#%s is not assigned", _led_col_str[col]);
        return;
    }

    if (_rgb.pin_cnt == 0)
    {
        iprintln(trPWM, "#No pins assigned");
        return;
    }

    // Should we reset the pin to INPUT?
	//pinMode(_rgb.colour[col].pin, INPUT_PULLUP);

	iprintln(trPWM, "#Pin %d unassigned from %s", _rgb.colour[col].pin, _led_col_str[col]);

    _rgb.colour[col].pin = PWM_PIN_UNASSIGNED;
    _rgb.pin_cnt--;

    if (_rgb.pin_cnt > 0) //Otherwise we are dividing by zero!!!!
    {
        //We want to stagger the switchon point of the LEDs to avoid inrush current
        //We do this by spreading the switchon point of the assigned LEDs over the PWM cycle
        int delta = PWM_RESOLUTION/_rgb.pin_cnt;
        int pin_cnt = 0;
        for (int i = 0; i < eLED_MAX; i++)
        {
            if (_rgb.colour[i].pin != PWM_PIN_UNASSIGNED)
            {
                _rgb.colour[i].pwm._on = pin_cnt*delta;
                pin_cnt++;
            }
        }
    }
}

void rgb_led_pwm_set_duty_cycle(led_colour_type col, uint8_t duty_cycle_target)
{
    if (col >= eLED_MAX)
    {
        iprintln(trPWM, "#Invalid colour (%d)!", col);
        return;
    }
    
    if (_rgb.colour[col].pin == PWM_PIN_UNASSIGNED)
    {
        iprintln(trPWM, "#%s is not assigned to a pin, no PWM possible!", _led_col_str[col]);
        return;
    }

    _rgb.colour[col].pwm.target = duty_cycle_target;
    _set_adjusted_duty_cycle(col);
}

/*
uint8_t rgb_led_pwm_get_duty_cycle(led_colour_type col)
{
    if (col >= eLED_MAX)
    {
        iprintln(trPWM, "#Invalid LED colour (%d)", col);
        return 0;
    }

    if (_rgb.colour[col].pin == PWM_PIN_UNASSIGNED)
    {
        iprintln(trPWM, "#%s is not assigned", _led_col_str[col]);
        return 0;
    }

    return _rgb.colour[col].pwm.target;
}

uint8_t rgb_led_pwm_get_adj_duty_cycle(led_colour_type col)
{
    if (col >= eLED_MAX)
    {
        iprintln(trPWM, "#Invalid LED colour (%d)", col);
        return 0;
    }

    if (_rgb.colour[col].pin == PWM_PIN_UNASSIGNED)
    {
        iprintln(trPWM, "#%s is not assigned", _led_col_str[col]);
        return 0;
    }

    return _rgb.colour[col].pwm.adjust;
}

void rgb_led_pwm_inc_duty_cycle(led_colour_type col, uint8_t inc_val, bool wrap)
{
    if (col >= eLED_MAX)
    {
        iprintln(trPWM, "#Invalid LED colour (%d)", col);
        return;
    }

    if (_rgb.colour[col].pin == PWM_PIN_UNASSIGNED)
    {
        iprintln(trPWM, "#%s is not assigned", _led_col_str[col]);
        return;
    }

    if ((inc_val >= (PWM_RESOLUTION-_rgb.colour[col].pwm.target)) && (!wrap))
        _rgb.colour[col].pwm.target = PWM_MAX_VALUE;
    else //wrap around if bigger, otherwise, no care
        _rgb.colour[col].pwm.target += inc_val;
    _set_adjusted_duty_cycle(col);
}

void rgb_led_pwm_dec_duty_cycle(led_colour_type col, uint8_t dec_val, bool wrap)
{
    if (col >= eLED_MAX)
    {
        iprintln(trPWM, "#Invalid LED colour (%d)", col);
        return;
    }

    if (_rgb.colour[col].pin == PWM_PIN_UNASSIGNED)
    {
        iprintln(trPWM, "#%s is not assigned", _led_col_str[col]);
        return;
    }

    if ((dec_val >= _rgb.colour[col].pwm.target) && (!wrap))
        _rgb.colour[col].pwm.target = 0;
    else //wrap around if bigger, otherwise, no care
        _rgb.colour[col].pwm.target -= dec_val;
    
    _set_adjusted_duty_cycle(col);
}
*/
#undef PRINTF_TAG
/*************************** END OF FILE *************************************/
