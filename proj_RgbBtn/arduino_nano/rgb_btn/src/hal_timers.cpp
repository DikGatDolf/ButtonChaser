/******************************************************************************
Project:   RGB Button Chaser
Module:     hal_timers.c
Purpose:    This file contains the timer routines (because delay() sucks!)
Author:     Rudolph van Niekerk
Processor:  Arduino Nano (ATmega328)
Compiler:	Arduino AVR Compiler


This implements timers which has to be polled to check for expiry.
The timer will act as a one-shot timer in normal operation.
To make the timer behave as a recurring timer, reload the interval and start
the timer once it has expired (using TimerStart()).

The General Timer (timer_ms_t type) - 1 kHz granularity
The general Timers enables the program to create a downcounter with a
preloaded value. This timer will then decrement every 1 ms until it has
expired.
Usage:
The module making use of the timers must host a timer_ms_t structure in RAM and
add it to the linked list (TimerAdd) to ensure that it is maintained.
Removing it from the linked list (TimerRemove) will  make it dormant.
The Timer must be polled (TimerPoll) to see when it has expired

 ******************************************************************************/

#define __NOT_EXTERN__
#include "hal_timers.h"
#undef __NOT_EXTERN__

#include "Arduino.h"
#include "sys_utils.h"
#include <util/atomic.h>

/*******************************************************************************
local defines
 *******************************************************************************/
#define TMR1_CNT_VAL        (F_CPU/1000/64)
#define TMR1_TCNT_LOADVAL   (0xFFFF - TMR1_CNT_VAL + 1)
#define MAX_CB_TMR_CNT      (6)

/*******************************************************************************
local variables
*******************************************************************************/
typedef struct cb_tmr_s
{
    sys_cb_tmr_exp_t func;
    volatile unsigned long ms_cnt;  // 1ms ~ 49 days.
    unsigned long ms_period;        // 1ms ~ 49 days.
    bool reload_mode; //Reload mode
    bool enabled;     //Is the timer enabled... used to pause the timer operation to 
                      // prevent it from being polled while it is being modified
} cb_tmr_t;

cb_tmr_t cb_tmr[MAX_CB_TMR_CNT];

bool sys_tmr_init_ok = false;
/*******************************************************************************
local functions
 *******************************************************************************/
// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#if CLOCK_CORRECTION_ENABLED == 1
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
//static unsigned char timer0_fract = 0;

volatile float millis_correction = 1.0f; // Default: normal rate
bool correction_enabled = false;

#define BASE_MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000.0)
#define BASE_FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)

ISR(TIMER0_OVF_vect)
{
	// // copy these to local variables so they can be stored in registers
	// // (volatile variables must be read from memory on every access)
	// unsigned long m = timer0_millis;
	// unsigned char f = timer0_fract;

	// m += MILLIS_INC;
	// f += FRACT_INC;
	// if (f >= FRACT_MAX) {
	// 	f -= FRACT_MAX;
	// 	m += 1;
	// }

	// timer0_fract = f;
	// timer0_millis = m;
	// timer0_overflow_count++;

    static float fract_buffer = 0.0f;

    unsigned long m = timer0_millis;

    // Apply correction
    float adjusted_millis_inc = BASE_MILLIS_INC * millis_correction;
    float adjusted_fract_inc = BASE_FRACT_INC * millis_correction;

    fract_buffer += adjusted_fract_inc;
    m += (unsigned long)adjusted_millis_inc;

    if (fract_buffer >= FRACT_MAX) {
        fract_buffer -= FRACT_MAX;
        m += 1;
    }

    timer0_millis = m;
    timer0_overflow_count++;
}
#endif /* CLOCK_CORRECTION_ENABLED */

ISR(TIMER1_OVF_vect)
{
    //Load the timer with the value to start counting from (0x00)
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
    {
        //Set the counter to a value which will cause it to overflow every 1ms
        TCNT1 = TMR1_TCNT_LOADVAL; 
    }
    for (int i = 0; i < MAX_CB_TMR_CNT; i++)
    {
        if ((cb_tmr[i].func) && (cb_tmr[i].enabled)) //Check if the function pointer is not NULL and the timer is enabled
        {
            if ((--cb_tmr[i].ms_cnt) == 0)
            {
                cb_tmr[i].func();       //Call the function
                if(cb_tmr[i].reload_mode)
                    cb_tmr[i].ms_cnt = cb_tmr[i].ms_period; //Reload the timer
                else
                    cb_tmr[i].func = NULL;  //Clear the function pointer to indicate that it is free
            }
        }
    }
}

/*******************************************************************************
Global overridden functions
 *******************************************************************************/
#if CLOCK_CORRECTION_ENABLED == 1
unsigned long sys_millis()
{
	unsigned long m;
	uint8_t oldSREG = SREG;

	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	cli();
	m = timer0_millis;
	SREG = oldSREG;

	return m;    
}
#endif /* CLOCK_CORRECTION_ENABLED */

/*******************************************************************************
Global/Public functions
 *******************************************************************************/
void sys_tmr_init(void)
{
    if (sys_tmr_init_ok)
        return; //Already initialized

    //The poll timer and stopwatch makes use of the sys_millis() function (to 
    // determine the time elapsed), which in turn makes use of the timer0.

#if 0
	// on the ATmega168, timer 0 is also used for fast hardware pwm
	// (using phase-correct PWM would mean that timer 0 overflowed half as often
	// resulting in different sys_millis() behavior on the ATmega8 and ATmega168)
	sbi(TCCR0A, WGM01);
	sbi(TCCR0A, WGM00);
	// set timer 0 prescale factor to 64
	// this combination is for the standard 168/328/1280/2560
	sbi(TCCR0B, CS01);
	sbi(TCCR0B, CS00);
	// enable timer 0 overflow interrupt
	sbi(TIMSK0, TOIE0);
#endif
#if CLOCK_CORRECTION_ENABLED == 1
    sys_time_correction_factor_reset(); //Reset the time correction factor
#endif /* CLOCK_CORRECTION_ENABLED */

    //We are going to repurpose timer1 for our own use, so we need to 
    // reconfigure it
	// Stop the timer 1
	cbi(TIMSK1, TOIE1);
    //Set the timer 1 to normal mode (no PWM, no CTC, no fast PWM)
	cbi(TCCR1A, WGM11);
	cbi(TCCR1A, WGM10);
	//Set timer 1 prescale factor to 64
	sbi(TCCR1B, CS11);
	sbi(TCCR1B, CS10);
    //Load the timer with the value to start counting from (0x00)
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
    {
        //Set the counter to a value which will cause it to overflow every 1ms
        TCNT1 = TMR1_TCNT_LOADVAL; 
    }

    // Now the prescaler is set so that timer1 ticks every 64 clock cycles, and the
    // the overflow handler is called every 250 ticks.

    for (int i = 0; i < MAX_CB_TMR_CNT; i++)
    {
        cb_tmr[i].func = NULL;
        cb_tmr[i].ms_cnt = 0;
    }

	// enable timer 1 overflow interrupt
	sbi(TIMSK1, TOIE1);

    sys_tmr_init_ok = true; //Set the timer as initialized
}

#if CLOCK_CORRECTION_ENABLED == 1
void sys_time_correction_factor_set(float correction)
{
    //Make sure this next step is not (haha) interrupted
	cbi(TIMSK0, TOIE0);     //Disable the timer 0 overflow interrupt
    if (correction_enabled)
        millis_correction *= correction;
    else
        millis_correction = correction;
    //RVN - TODO:To reduce jitter, consider exponentially smoothing the correction over time:
    //millis_correction = millis_correction * 0.95 + correction * 0.05;
	sbi(TIMSK0, TOIE0);     // enable timer 0 overflow interrupt
    correction_enabled = true; //Set the correction enabled flag
}

void sys_time_correction_factor_reset(void)
{
	cbi(TIMSK0, TOIE0);     //Disable the timer 0 overflow interrupt
    millis_correction = 1.0f; //Reset to normal (uncorrected) rate
	sbi(TIMSK0, TOIE0);     // enable timer 0 overflow interrupt
    correction_enabled = false; //Reset the correction enabled flag
}

float sys_time_correction_factor(void)
{
    return millis_correction;
}
#endif /* CLOCK_CORRECTION_ENABLED */

bool sys_cb_tmr_start(void (*cb_tmr_exp)(void), unsigned long interval, bool reload)
{
    if (!sys_tmr_init_ok)
        return false; //Timer not initialized

    //First we need to check if this callback is not already in the list
    for (int i = 0; i < MAX_CB_TMR_CNT; i++)
    {
        if (cb_tmr[i].func == cb_tmr_exp)
        {
            //Already in the list, so we can just reset the timer settings
            cb_tmr[i].enabled = false; //Disable the timer as a first action, so we can change the settings without worring about it being changed by the interrupt
            
            //If the new period is longer than the previously set period, then we increse the count by the difference of the two periods
            if (interval > cb_tmr[i].ms_period)
                cb_tmr[i].ms_cnt += (interval - cb_tmr[i].ms_period);
            //if the new period is shorter than the previously set period, we need to adjust the count.... but only if the current count is longer than the new period
            else if (cb_tmr[i].ms_cnt > interval)
                cb_tmr[i].ms_cnt = interval;
            //The rest of the settings are just set to their new values
            cb_tmr[i].ms_period = interval; // 1ms ~ 49 days.
            cb_tmr[i].reload_mode = reload;
            cb_tmr[i].enabled = true; //Enable the timer again as the very last action!
            return true;
        }
    }

    //Not in the list, so we need to find a free slot
    for (int i = 0; i < MAX_CB_TMR_CNT; i++)
    {
        if (!cb_tmr[i].func)
        {
            cb_tmr[i].ms_period = interval; // 1ms ~ 49 days.
            cb_tmr[i].ms_cnt = interval;
            cb_tmr[i].reload_mode = reload;
            cb_tmr[i].func = cb_tmr_exp;
            cb_tmr[i].enabled = true; //Enable the timer as the very last action!
            return true;
        }
    }
    return false; //No free callback timers available
}

void sys_cb_tmr_stop(void (*cb_tmr_exp)(void))
{
    if (!sys_tmr_init_ok)
        return; //Timer not initialized

    //First we need to check if this callback is not already in the list
    for (int i = 0; i < MAX_CB_TMR_CNT; i++)
    {
        if (cb_tmr[i].func == cb_tmr_exp)
        {
            cb_tmr[i].enabled = false; //Disable the timer as a first action, so we can change the settings without worrying about it being changed by the interrupt
            //In the list, just remove it
            cb_tmr[i].ms_period = 0lu;
            cb_tmr[i].ms_cnt = 0lu;
            cb_tmr[i].func = NULL;
            cb_tmr[i].reload_mode = false;
            return;
        }
    }
}

void sys_poll_tmr_start(timer_ms_t *t, unsigned long interval, bool auto_reload)
{
#if CLOCK_CORRECTION_ENABLED == 1
    t->started = sys_millis();
#else
    t->started = millis();
#endif /* CLOCK_CORRECTION_ENABLED */
    t->enabled = false;
    t->ms_expire = t->started + interval;
    t->ms_period = interval;
    t->expired = false;
    t->reload_mode = auto_reload;
    t->enabled = true;
}

bool sys_poll_tmr_reset(timer_ms_t *t)
{
    if (t->ms_period > 0)
    {
#if CLOCK_CORRECTION_ENABLED == 1
        t->started = sys_millis();
#else
        t->started = millis();        
#endif /* CLOCK_CORRECTION_ENABLED */
        t->enabled = false;
        t->ms_expire = t->started + t->ms_period;
        t->expired = false;
        t->enabled = true;
    }
    return t->enabled;
}

void sys_poll_tmr_stop(timer_ms_t *t)
{
    t->enabled = false;
    //t->reload_mode = false;
}

bool sys_poll_tmr_expired(timer_ms_t *t)
{
#if CLOCK_CORRECTION_ENABLED == 1
    uint64_t now_ms = sys_millis();
#else
    uint64_t now_ms = millis();
#endif /* CLOCK_CORRECTION_ENABLED */
    uint64_t overflow = 0;

    //Is the timer enabled?
    if (!(t->enabled))
        return false;

    //Have we already seen the expiry boundary crossed (not in auto reload mode)?
    if (t->expired)
        return true; // Return here, then we don't need to worry about overflowing the variable boundaries

    //RVN - TODO - You need to think about this before implementing it
    // if (t->started > now_ms)
    //     overflow = (UINT32_MAX - t->started) + now_ms;
    // else
    //     overflow = now_ms - t->started;

    //Have we moved beyond the expiry time?
    if (now_ms >= t->ms_expire)
    {
        //NOW it has expired (could have been multiple times)
        overflow = (now_ms - t->ms_expire);
        //If we are in reload mode, set the next expiry time without raising the expired flag
        if (t->reload_mode)
        {
            t->started = now_ms - (overflow % (t->ms_period));
            t->ms_expire = t->started + t->ms_period;                     /* Timer Period */
        }
        else
            t->expired = true;

        return true;
    }

    return (t->expired);
}

bool sys_poll_tmr_enabled(timer_ms_t *t)
{
    return t->enabled;
}

bool sys_poll_tmr_is_running(timer_ms_t *t)
{
#if CLOCK_CORRECTION_ENABLED == 1
    if (t->enabled) //Is the timer enabled?
        return (sys_millis() < t->ms_expire)? true : false;
#else
    if (t->enabled) //Is the timer enabled?
        return (millis() < t->ms_expire)? true : false;
#endif /* CLOCK_CORRECTION_ENABLED */

    return false;
}

uint64_t sys_poll_tmr_seconds(void)
{
#if CLOCK_CORRECTION_ENABLED == 1
  return (sys_millis()/1000);
#else
  return (millis()/1000);
#endif /* CLOCK_CORRECTION_ENABLED */
}

void sys_stopwatch_ms_start(stopwatch_ms_t* sw, unsigned long max_time)
{
#if CLOCK_CORRECTION_ENABLED == 1
    sw->tick_start = sys_millis();
#else
    sw->tick_start = millis();
#endif /* CLOCK_CORRECTION_ENABLED */
    sw->max_time = max_time;
    sw->running = true;
    sw->max_time_reached = false;
}

unsigned long sys_stopwatch_ms_lap(stopwatch_ms_t* sw)
{
    unsigned long elapsed = 0;
    unsigned long now = 0;
    // ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
    // {
#if CLOCK_CORRECTION_ENABLED == 1
        now = sys_millis();
#else
        now = millis();
#endif /* CLOCK_CORRECTION_ENABLED */
    // }

    if (sw->running)
    {
        if (now >= sw->tick_start)
            elapsed = now - sw->tick_start;
        else //Assume we have wrapped (~49 days ofr uint32_t)
            elapsed = (UINT32_MAX - sw->tick_start) + now;

        //Do we have a max time set?
        if (sw->max_time > 0)
        {
            if (elapsed >= sw->max_time)
                sw->max_time_reached = true;

            if (sw->max_time_reached)
                return sw->max_time;
        }
    }
    
    return elapsed;
}

unsigned long sys_stopwatch_ms_reset(stopwatch_ms_t* sw)
{
    unsigned long elapsed = sys_stopwatch_ms_lap(sw);
    sys_stopwatch_ms_start(sw, sw->max_time);
    return elapsed;
}

unsigned long sys_stopwatch_ms_stop(stopwatch_ms_t* sw)
{
    unsigned long elapsed = sys_stopwatch_ms_lap(sw);
    sw->running = false;
    return elapsed;
}


/*************************** END OF FILE *************************************/
