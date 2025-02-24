/******************************************************************************
Module:     sys_timers.c
Purpose:    This file contains polling timer routines
Author:     Rudolph van Niekerk

This implements software timers which has to be polled to check for expiry.
Yeah, I know FreeRTOS has built-in timers, but these are limited by the 
configTICK_RATE_HZ frequency.... about 100Hz. This means we have a timer 
resoluton of 10ms.

This implementation makes use of the the timer thread already running on the 
ESP32, and provides a granulaity of 1ms. Another upside of this implementation 
(iso using a timer callback) is that the timer polling is done in the calling 
thread, not the timer thread.  

The timer will act as a one-shot timer in normal operation. If the Auto-reload 
value is set, it will reload immediately after firing, but this has risk in
the sense that it needs to be guaranteed polled faster than the reload value
otherwise the polling thread is going to "miss" expiry.

The module making use of the timers must statically declare a Timer_ms_t 
structure in RAM and start it 
The Timer must be polled (TimerPoll) to see when it has expired

 ******************************************************************************/
#include "esp_timer.h"

#define __NOT_EXTERN__
#include "sys_timers.h"
#undef __NOT_EXTERN__

/*******************************************************************************
local defines
 *******************************************************************************/
#define PRINTF_TAG ("Timers") /* This must be undefined at the end of the file*/

/*******************************************************************************
local structure
 *******************************************************************************/

typedef struct {
	bool init_done;
}SysTimers_t;

// typedef struct Timer_ms_t_int
// {
//     union {
//         struct {
//         };
// 		uint8_t reserved[]
//         Timer_ms_t reserved;
//     };
// } Timer_ms_t_int;



/*******************************************************************************
local function prototypes
*******************************************************************************/

void sys_poll_1_ms_tick(void* arg);

/*******************************************************************************
local variables
 *******************************************************************************/
static uint64_t _sys_poll_1_ms_ticker;

static esp_timer_create_args_t _sys_poll_1_ms_args = {
    .callback = sys_poll_1_ms_tick,
    //.dispatch_method;   //!< Call the callback from task or from ISR
    .name = "1ms_Ticker",
    //bool skip_unhandled_events;     //!< Skip unhandled events for periodic timers
};

static esp_timer_handle_t _sys_poll_1_ms_handle;
SysTimers_t _sys_tmr = {
	.init_done = false
};

/*******************************************************************************
local functions
 *******************************************************************************/

void sys_poll_1_ms_tick(void* arg)
{
	//Keep this function call as short as possible.
	_sys_poll_1_ms_ticker++;
}

/*******************************************************************************
global functions
 *******************************************************************************/

/* Initializes the sys_timers module.
*/
void sys_timers_init(void)
{
	//Only do this once.
	if (_sys_tmr.init_done)
		return;

	_sys_poll_1_ms_ticker = esp_timer_get_time()/1000;//0L;
    
    ESP_ERROR_CHECK(esp_timer_create(&_sys_poll_1_ms_args, &_sys_poll_1_ms_handle));

    // Start the timer immediately
    ESP_ERROR_CHECK(esp_timer_start_periodic(_sys_poll_1_ms_handle, 1000));

	_sys_tmr.init_done = true;
}

void sys_timers_deinit(void)
{
    ESP_ERROR_CHECK(esp_timer_stop(_sys_poll_1_ms_handle));
    ESP_ERROR_CHECK(esp_timer_delete(_sys_poll_1_ms_handle));
	_sys_tmr.init_done = false;
}

void sys_timers_sleep(void)
{
	//RVN - TBD
}

void sys_timers_wake(void)
{
	//RVN - TBD
}

int64_t sys_poll_tmr_ms_get_error(void)
{
	return (int64_t)((int64_t)_sys_poll_1_ms_ticker -(esp_timer_get_time()/1000));
}

void sys_poll_tmr_start(Timer_ms_t *t, uint64_t interval, bool auto_reload)
{

	t->enabled = false;
	t->ms_expire = _sys_poll_1_ms_ticker + interval;
	t->ms_period = interval;
	t->expired = false;
	t->reload_mode = auto_reload;
	t->enabled = true;
}

bool sys_poll_tmr_reset(Timer_ms_t *t)
{
	if (t->ms_period > 0)
	{
		t->enabled = false;
		t->ms_expire = _sys_poll_1_ms_ticker + t->ms_period;
		t->expired = false;
		t->enabled = true;
	}
	return t->enabled;
}

void sys_poll_tmr_stop(Timer_ms_t *t)
{
	t->enabled = false;
	//t->reload_mode = false;
}


bool sys_poll_tmr_expired(Timer_ms_t *t)
{
	uint64_t now_ms = _sys_poll_1_ms_ticker;
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
            t->ms_expire = now_ms - (overflow % t->ms_period)   /* Now, minus The part of the Timer Period that has elapsed already */
                            + t->ms_period;                     /* Timer Period */
        else
            t->expired = true;

        return true;
	}

	return (t->expired);
}

bool sys_poll_tmr_enabled(Timer_ms_t *t)
{
	return t->enabled;
}

bool sys_poll_tmr_is_running(Timer_ms_t *t)
{
	if (t->enabled) //Is the timer enabled?
		return (_sys_poll_1_ms_ticker < t->ms_expire)? true : false;

	return false;
}

uint64_t sys_poll_tmr_seconds(void)
{
  return (_sys_poll_1_ms_ticker/1000);
}

#undef PRINTF_TAG
/*************************** END OF FILE *************************************/
