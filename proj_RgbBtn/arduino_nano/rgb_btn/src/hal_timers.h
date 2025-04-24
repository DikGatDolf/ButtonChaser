/*****************************************************************************

hal_timers.h

Include file for hal_timers.c

******************************************************************************/
#ifndef __HAL_TIMERS_H__
#define __HAL_TIMERS_H__


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
typedef void (*sys_cb_tmr_exp_t)(void);

typedef struct
{
    unsigned long started;   // 1ms ~ 49 days.
    volatile unsigned long ms_expire;   // 1ms ~ 49 days.
    volatile bool enabled;
    volatile bool expired;
    unsigned long ms_period;   // 1ms ~ 49 days.
    bool reload_mode; 
} timer_ms_t;

typedef struct stopwatch_ms_s
{
	volatile unsigned long tick_start;   // 1ms ~ 49 days.
	volatile bool running;
    unsigned long max_time;
    bool max_time_reached;
} stopwatch_ms_t;
/******************************************************************************
variables
******************************************************************************/

/******************************************************************************
functions
******************************************************************************/

/*! Initializes the timer module. This function must be called before any other
 *  timer functions. It sets up the timer interrupt and initializes the timer 
 *  structures.
 */
void sys_tmr_init(void);

/*! Starts a timer with a callback function and an interval. This timer is 
 * self-destructive, i.e. it will be destroyed after it has expired.
 * @param cb_tmr_exp  The callback function to call when the timer expires. 
 *  Please Note that this function is executed in the context of the timer 
 *  interrupt, so it must be quick and not block.
 * @param interval    period (in ms) after which to expire (unsigned long)
 * @returns true if the timer could be enabled, false if not.
*/
bool sys_cb_tmr_start(void (*cb_tmr_exp)(void), unsigned long interval, bool reload = false);

/*! Stops a timer with a callback function. This function will remove the 
 *  callback function from the list of timers and stop it.
 * @param cb_tmr_exp  The callback function to stop. 
*/
void sys_cb_tmr_stop(void (*cb_tmr_exp)(void));

/*! Sets the timers interval and starts it. 
 * @param t         a pointer to a static timer_ms_t 
 * @param interval  period (in ms) after which to expire (unsigned long)
 * @param reload    Setting reload to true will ensure that a timer automatically 
 *                  reset itself at the next poll after expiry
 */
void sys_poll_tmr_start(timer_ms_t *t, unsigned long interval, bool reload);

/** Resets the timers to its previously set interval and starts it (again). 
 * Will only succeed if the interval has been set before.
 * @param t a pointer to a static timer_ms_t 
 * @returns true if the timer could be enabled, false if not.
*/
bool sys_poll_tmr_reset(timer_ms_t *t);

/** Stops the timer. 
 * DO NOT POLL A TIMER THAT HAS BEEN STOPPED.
 * When polling a stopped timer, it will appear like it has not expired and it
 * will never expire (the Poll will return false ad infinitum) .
 * @param t a pointer to a static timer_ms_t 
*/
void sys_poll_tmr_stop(timer_ms_t *t);

/** Check the timer to see if it has expired.
 * Returns TRUE if the timer has expired, but only if it is still enabled. 
 * If a timer has been stopped, it will always return false when polled, 
 * appearing as if it doesn't expire.
 * If the auto reload has been enabled, the timer will immediately be set to 
 * expire (again) at the next interval period.  
 * MAKE SURE THE TIMER IS POLLED AT LEAST 2x FASTER THAN THE INTERVAL PERIOD, 
 * if the auto reload has been enabled... for an explanation of this, read up
 * on Nyquist's theorem.
 * @param t a pointer to a static timer_ms_t 
 * @returns true if the timer has expired, false otherwise
*/
bool sys_poll_tmr_expired(timer_ms_t *t);

/** Check if a timer is enabled
 * @param t a pointer to a static timer_ms_t 
 * @returns true if the timer is enabled, false otherwise
*/
bool sys_poll_tmr_enabled(timer_ms_t *t);

/** Check if a timer is running. 
 * NOTE: A timer running in auto-reload mode will ALWAYS appear to be running
 * @param t a pointer to a static timer_ms_t 
 * @returns true if the timer is running, false otherwise
*/
bool sys_poll_tmr_is_running(timer_ms_t *t);

/** Starts a stopwatch (passed as a pointer)
 * @param sw a pointer to a static stopwatch_ms_t
*/
void sys_stopwatch_ms_start(stopwatch_ms_t* sw, unsigned long max_time = 0);

/*! returns the elapsed time in ms, without stopping or resetting the stopwatch
 * @param sw a pointer to a static stopwatch_ms_t
 * @returns the elapsed time in ms
*/
unsigned long sys_stopwatch_ms_lap(stopwatch_ms_t* sw);

/*! returns the elapsed time in ms, and resetting the stopwatch
 * @param sw a pointer to a static stopwatch_ms_t
 * @returns the elapsed time in ms
*/
unsigned long sys_stopwatch_ms_reset(stopwatch_ms_t* sw);

/*1 Stops a stopwatch (passed as a pointer) and returns the elapsed time in ms
 * IMPORTANT: this stopwatch timer wraps at 1193h 2min 47.296s
 * @param sw a pointer to a static stopwatch_ms_t
 * @returns the elapsed time in ms
*/
unsigned long sys_stopwatch_ms_stop(stopwatch_ms_t* sw);

#endif /* __HAL_TIMERS_H__ */

/****************************** END OF FILE **********************************/
