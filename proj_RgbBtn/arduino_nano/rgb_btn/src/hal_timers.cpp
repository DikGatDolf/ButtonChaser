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
} cb_tmr_t;

cb_tmr_t cb_tmr[MAX_CB_TMR_CNT];

bool sys_tmr_init_ok = false;
/*******************************************************************************
local functions
 *******************************************************************************/
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
        if (cb_tmr[i].func)
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
Global/Public functions
 *******************************************************************************/
void sys_tmr_init(void)
{
    if (sys_tmr_init_ok)
        return; //Already initialized

    //The poll timer and stopwatch makes use of the millis() function (to 
    // determine the time elapsed), which in turn makes use of the timer0.

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

bool sys_cb_tmr_start(void (*cb_tmr_exp)(void), unsigned long interval, bool reload)
{
    if (!sys_tmr_init_ok)
        return false; //Timer not initialized

    //First we need to check if this callback is not already in the list
    for (int i = 0; i < MAX_CB_TMR_CNT; i++)
    {
        if (cb_tmr[i].func == cb_tmr_exp)
        {
            //Already in the list, so just reset the timer
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
            {
                cb_tmr[i].ms_period = interval; // 1ms ~ 49 days.
                cb_tmr[i].ms_cnt = interval;    // 1ms ~ 49 days.
                cb_tmr[i].reload_mode = reload;
            }
            return true;
        }
    }

    //Not in the list, so we need to find a free slot
    for (int i = 0; i < MAX_CB_TMR_CNT; i++)
    {
        if (!cb_tmr[i].func)
        {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
            {
                cb_tmr[i].ms_period = interval; // 1ms ~ 49 days.
                cb_tmr[i].ms_cnt = interval;
                cb_tmr[i].func = cb_tmr_exp;
                cb_tmr[i].reload_mode = reload;
            }
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
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
            {
                //In the list, just remove it
                cb_tmr[i].ms_period = 0lu;
                cb_tmr[i].ms_cnt = 0lu;
                cb_tmr[i].func = NULL;
                cb_tmr[i].reload_mode = false;
            }
            return;
        }
    }
}

void sys_poll_tmr_start(timer_ms_t *t, unsigned long interval, bool auto_reload)
{
    t->started = millis();
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
        t->started = millis();
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
    uint64_t now_ms = millis();
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
    if (t->enabled) //Is the timer enabled?
        return (millis() < t->ms_expire)? true : false;

    return false;
}

uint64_t sys_poll_tmr_seconds(void)
{
  return (millis()/1000);
}

void sys_stopwatch_ms_start(stopwatch_ms_t* sw, unsigned long max_time)
{
    sw->tick_start = millis();
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
        now = millis();
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
