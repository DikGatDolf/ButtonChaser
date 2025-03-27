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

/*******************************************************************************
local defines
 *******************************************************************************/

/*******************************************************************************
local variables
 *******************************************************************************/

/*******************************************************************************
local functions
 *******************************************************************************/

void sys_poll_tmr_start(timer_ms_t *t, unsigned long interval, bool auto_reload)
{
    t->enabled = false;
    t->ms_expire = millis() + interval;
    t->ms_period = interval;
    t->expired = false;
    t->reload_mode = auto_reload;
    t->enabled = true;
}

bool sys_poll_tmr_reset(timer_ms_t *t)
{
    if (t->ms_period > 0)
    {
        t->enabled = false;
        t->ms_expire = millis() + t->ms_period;
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

    //Have we moved beyond the expiry time?
    if (now_ms >= t->ms_expire)
    {
        //NOW it has expired (could have been multiple times)
        overflow = (now_ms - t->ms_expire);
        //If we are in reload mode, set the next expiry time without raising the expired flag
        if (t->reload_mode)
            t->ms_expire = now_ms - (overflow % (t->ms_period))   /* Now, minus The part of the Timer Period that has elapsed already */
                            + t->ms_period;                     /* Timer Period */
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
    unsigned long now = millis();

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
    sys_stopwatch_ms_start(sw);
    return elapsed;
}

unsigned long sys_stopwatch_ms_stop(stopwatch_ms_t* sw)
{
    unsigned long elapsed = sys_stopwatch_ms_lap(sw);
    sw->running = false;
    return elapsed;
}

unsigned long SecondCount(void)
{
  return (millis()/1000l);
}

/*************************** END OF FILE *************************************/
