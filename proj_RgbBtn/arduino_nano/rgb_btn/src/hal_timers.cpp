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
#include "hal_timers.h"
#undef __NOT_EXTERN__

#include "Arduino.h"
#include "std_utils.h"

/*******************************************************************************
local defines
 *******************************************************************************/

/*******************************************************************************
local variables
 *******************************************************************************/

/*******************************************************************************
local functions
 *******************************************************************************/

/*******************************************************************************
 * @brief Sets the timer interval and starts it. If the timer is already
 *  running, it will be restarted. ms-timers need to be polled using 
 * 'msTimerPoll()' to check for expiry. If the interval value is set to 0, the 
 * timer will expire on the first poll.  
 * 
 * @param t Pointer to the Timer Struct being reset
 * @param interval The timer interval in milliseconds
 * @returns true if the timer was started, false if the timer was not started.
 *******************************************************************************/
void msTimerStart(ST_MS_TIMER *t, unsigned long interval)
{

	t->Enabled = false;
	t->msExpire = millis() + interval;
	t->msPeriod = interval;
	t->Expired = false;
	t->Enabled = true;
}

/*******************************************************************************
 * @brief Restarts the timer with a previously set interval. If the period hasn't been set 
 * yet (or it is set to 0), the timer will not be started.
 * ms-timers need to be polled using 'msTimerPoll()' to check for expiry.
 * @param t Pointer to the Timer Struct being reset
 * @returns true if the timer was started, false if the timer was not started.
 *******************************************************************************/
bool msTimerReset(ST_MS_TIMER *t)
{
	if (t->msPeriod > 0)
	{
		t->Enabled = false;
		t->msExpire = millis() + t->msPeriod;
		t->Expired = false;
		t->Enabled = true;
	}
	return t->Enabled;
}
/*******************************************************************************
 * @brief Stops the timer. 
 * IMORTANT: DO NOT POLL A ST_MS_TIMER YOU HAVE STOPPED. When polling a stopped 
 * timer, it will appear like it has not expired and it never will expire (the 
 * Poll will return false, indicating "not expired" ad infinitum).
 * @param t Pointer to the Timer Struct being stopped
 *******************************************************************************/
void msTimerStop(ST_MS_TIMER *t)       //Pointer to the Timer Struct being stopped
{
	t->Enabled = false;
	//  t->Expired = true;
}


/*******************************************************************************
 * @brief Polls the timer to see if it has expired, but only if it is still 
 * enabled. If a timer has been stopped, it will always return false when polled, 
 * appearing as if it doesn't expire.
 * @param t Pointer to the Timer Struct which you want to poll
 * @param restart If true, the timer will be restarted when it expires.
 * @returns true if the timer has expired, false if the timer has not expired.
 *******************************************************************************/
bool msTimerPoll(ST_MS_TIMER *t, bool restart)
{
	unsigned long now_ms = millis();

	//Is the timer enabled?
	if (t->Enabled == false)
		return false;

	//Has the timer expired?
	if (!(t->Expired))
	{
		//have we moved beyond the expiry time?
		if (now_ms >= t->msExpire)
		{
			//NOW it has expired
			if (true == restart)
			{
				msTimerReset(t);
				return true;	// The caller better not miss this!
			}
			else
			{
				t->Expired = true;
			}
		}
	}

	return(t->Expired);
}

/*******************************************************************************
 * @brief Checks if the timer is enabled.
 * @returns true if the timer is enabled.
 *******************************************************************************/
bool msTimerEnabled(ST_MS_TIMER *t)   //Pointer to the Timer Struct which you want to check
{
	return t->Enabled;
}

/*******************************************************************************
 * @brief Returns the number of seconds since System Startup
 * @returns The number of seconds since System Startup
*******************************************************************************/
unsigned long SecondCount(void)
{
  return (millis()/1000);
}

/*************************** END OF FILE *************************************/
