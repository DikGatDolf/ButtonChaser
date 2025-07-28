/*****************************************************************************

sys_timers.h

Include file for sys_poll_timer.c

******************************************************************************/
#ifndef __SYS_TIMERS_H__
#define __SYS_TIMERS_H__


/******************************************************************************
includes
******************************************************************************/


/******************************************************************************
definitions
******************************************************************************/
#ifdef __NOT_EXTERN__
#define EXT
#else
#define EXT extern
#endif /* __NOT_EXTERN__ */

/******************************************************************************
Macros
******************************************************************************/

/******************************************************************************
Struct & Unions
******************************************************************************/
// structs

/** This structure is provided for the sole purpose of enabling the host of the
 * timer to declare a static instace of the Timer. These members shall NOT be
 * read or manipulated directly under any circumstances.
 * Rather use of the following methods to access timer functionality:
 *  sys_poll_tmr_start();
 *  sys_poll_tmr_reset()
 *  sys_poll_tmr_stop()
 *  sys_poll_tmr_expired()
 *  sys_poll_tmr_is_running()
 *  sys_poll_tmr_started()
 */
typedef struct Timer_ms_t
{
	uint64_t ms_expire;   // 1ms ~ 500 million years
	uint32_t ms_period;   // 1ms ~ 49 days.
	bool started;
	bool expired;
	bool reload_mode;
} Timer_ms_t;

typedef struct Stopwatch_ms_t
{
	uint32_t tick_start;   // 1ms ~ 49 days.
	bool running;
    uint32_t max_time;
    bool max_time_reached;
} Stopwatch_ms_t;

/******************************************************************************
variables
******************************************************************************/

/******************************************************************************
functions
******************************************************************************/
/******************************************************************************
Class definition
******************************************************************************/

void sys_timers_init(void);
void sys_timers_deinit(void);
void sys_timers_sleep(void);
void sys_timers_wake(void);

/** returns the error (in ms) between the ESP timer's get_time() and the
 * ms_tmr module
 * @returns the number of seconds since it has been initialised
*/
int32_t sys_poll_tmr_ms_get_error(void);

/** Sets the timers interval and starts it. 
 * @param t         a pointer to a static Timer_ms_t 
 * @param interval  period (in ms) after which to expire (uint32_t)
 * @param reload    Setting reload to true will ensure that a timer automatically 
 *                  reset itself at the next poll after expiry
 */
void sys_poll_tmr_start(Timer_ms_t *t, uint32_t interval, bool reload);

/** Resets the timers to its previously set interval and starts it (again). 
 * Will only succeed if the interval has been set before.
 * @param t a pointer to a static Timer_ms_t 
 * @returns true if the timer could be enabled, false if not.
*/
bool sys_poll_tmr_reset(Timer_ms_t *t);

/** Stops the timer. 
 * DO NOT POLL A TIMER THAT HAS BEEN STOPPED.
 * When polling a stopped timer, it will appear like it has not expired and it
 * will never expire (the Poll will return false ad infinitum) .
 * @param t a pointer to a static Timer_ms_t 
*/
void sys_poll_tmr_stop(Timer_ms_t *t);

/** Check the timer to see if it has expired.
 * Returns TRUE if the timer has expired, but only if it is still enabled. 
 * If a timer has been stopped, it will always return false when polled, 
 * appearing as if it doesn't expire.
 * If the auto reload has been enabled, the timer will immediately be set to 
 * expire (again) at the next interval period.  
 * MAKE SURE THE TIMER IS POLLED AT LEAST 2x FASTER THAN THE INTERVAL PERIOD, 
 * if the auto reload has been enabled... for an explanation of this, read up
 * on Nyquist's theorem.
 * @param t a pointer to a static Timer_ms_t 
 * @returns true if the timer has expired, false otherwise
*/
bool sys_poll_tmr_expired(Timer_ms_t *t);

/** Check if a timer is enabled
 * @param t a pointer to a static Timer_ms_t 
 * @returns true if the timer is enabled, false otherwise
*/
bool sys_poll_tmr_started(Timer_ms_t *t);

/** Check if a timer is running. 
 * NOTE: A timer running in auto-reload mode will ALWAYS appear to be running
 * @param t a pointer to a static Timer_ms_t 
 * @returns true if the timer is running, false otherwise
*/
bool sys_poll_tmr_is_running(Timer_ms_t *t);

/** returns the current time according to the ms_tmr module
 * @returns the number of milliseconds since it has been initialised
*/
uint64_t sys_poll_tmr_ms(void);

/** returns the current time according to the ms_tmr module
 * @returns the number of seconds since it has been initialised
*/
uint64_t sys_poll_tmr_seconds(void);

/** Starts a stopwatch (passed as a pointer)
 * @param sw a pointer to a static Stopwatch_ms_t
*/
void sys_stopwatch_ms_start(Stopwatch_ms_t* sw, uint32_t max_time);

/*! returns the elapsed time in ms, without stopping or resetting the stopwatch
 * @param sw a pointer to a static Stopwatch_ms_t
 * @returns the elapsed time in ms
*/
uint32_t sys_stopwatch_ms_lap(Stopwatch_ms_t* sw);

/*! returns the elapsed time in ms, and resetting the stopwatch
 * @param sw a pointer to a static Stopwatch_ms_t
 * @returns the elapsed time in ms
*/
uint32_t sys_stopwatch_ms_reset(Stopwatch_ms_t* sw);

/** Stops a stopwatch (passed as a pointer) and returns the elapsed time in ms
 * IMPORTANT: this stopwatch timer wraps at 1193h 2min 47.296s
 * @param sw a pointer to a static Stopwatch_ms_t
 * @returns the elapsed time in ms
*/
uint32_t sys_stopwatch_ms_stop(Stopwatch_ms_t* sw);

/** Converts a ms count value to a string in the format "hhhh:mm:ss.000"
 * For ms values longer than 24 hours, the decimal (ms) seconds is discarded
 * @param buff a pointer to a buffer to store the string (max 15 characters)
 * @param ms the time in ms to convert
*/
bool ms2dhms_str(char *buff, uint32_t ms);

#undef EXT
#endif /* __SYS_TIMERS_H__ */

/****************************** END OF FILE **********************************/
