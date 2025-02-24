/*******************************************************************************

Module:     task_rgb_btn.c
Purpose:    This file contains the RGB LED driver task
Author:     Rudolph van Niekerk


 *******************************************************************************/

/*******************************************************************************
includes
 *******************************************************************************/
#include <ctype.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>
#include "sys_utils.h"
#include "task_console.h"
#include "drv_rgb_led_strip.h"

#define __NOT_EXTERN__
#include "task_rgb_btn.h"
#undef __NOT_EXTERN__

/*******************************************************************************
local defines and constants
 *******************************************************************************/
#define PRINTF_TAG ("RGB_BTN") /* This must be undefined at the end of the file*/

#define RGB_STACK_SIZE 4096

#define RGB_LED_CNT 5

#define EXAMPLE_CHASE_SPEED_MS      50

typedef enum
{
    eLedOff,
    eLedOn,
    eLedBlinking,
}eLED_State;

/*******************************************************************************
local structure
 *******************************************************************************/
typedef struct
{
	eLED_State state;
	int blink_rate;
} RgbLed_t;

typedef struct
{
    RgbLed_t Led[RGB_LED_CNT];
    int cnt;
	TaskInfo_t task;
} DeviceRgb_t;

/*******************************************************************************
local function prototypes
 *******************************************************************************/
/*! Reads the data on the serial port and parses the line when a carriage return 
 * is encountered.
 */
bool _task_rgb_btn_service(void);

/*******************************************************************************
local variables
 *******************************************************************************/
// ConsoleMenuItem_t _task_rgb_btn_menu_items[] =
// {
// 										    // 01234567890123456789012345678901234567890123456789012345678901234567890123456789
// 		// {"version", _menu_handler_version,   "Displays application version"},
// 		// {"help",  	_menu_handler_help,      "Displays information on the available Console commands"},
// 		// {"trace",   _menu_handler_trace,     "Displays the status, or Enables/Disables print traces"},
// 		// {"rtos", 	_menu_handler_tasks,     "Displays RTOS information for each or a specific task"},
// };

static DeviceRgb_t _task_rgb_btn = {
    .cnt = 0,
	.task.handle = NULL,
	.task.stack_depth = RGB_STACK_SIZE,
	.task.stack_unused = 0,
};

/*******************************************************************************
Local (private) Functions
*******************************************************************************/

/*******************************************************************************

Parses the line received on the Debug port.

 *******************************************************************************/
void _task_rgb_btn_task(void * pvParameters)
{
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    uint16_t hue = 0;

    drv_rgb_led_strip_init();

    dbgPrint(trRGB|trALWAYS, "#Start LED rainbow chase");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) 
    {
        led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);

        drv_rgb_led_strip_set_rgb_pixel(0, red, green, blue);
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));  //vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
    
        // drv_rgb_led_strip_clear_rgb_pixel(0);
        // xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));  //vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
        hue += 1;
            
		/* Inspect our own high water mark on entering the task. */
    	_task_rgb_btn.task.stack_unused = uxTaskGetStackHighWaterMark2( NULL );
    }
}

bool _task_rgb_btn_service(void)
{     
    return false;
}

/*******************************************************************************
Global (public) Functions
*******************************************************************************/

TaskInfo_t * task_rgb_btn_init(void)
{
	//Let's not re-initialise this task by accident
	if (_task_rgb_btn.task.handle)
		return &_task_rgb_btn.task;

	_task_rgb_btn.cnt = 0;

	// Create the task, storing the handle.  Note that the passed parameter ucParameterToPass
	// must exist for the lifetime of the task, so in this case is declared static.  If it was just an
	// an automatic stack variable it might no longer exist, or at least have been corrupted, by the time
	// the new task attempts to access it.
	if (xTaskCreate( _task_rgb_btn_task, PRINTF_TAG, _task_rgb_btn.task.stack_depth, _task_rgb_btn.task.parameter_to_pass, 10, &_task_rgb_btn.task.handle ) != pdPASS)
	{
		dbgPrint(trALWAYS, "#Unable to start Task (%d)!", eTaskIndexRgb);
		return NULL;
	}

    dbgPrint(trALWAYS, "#Init OK (%d LEDs)", _task_rgb_btn.cnt);

	configASSERT(_task_rgb_btn.task.handle);

	return &_task_rgb_btn.task;
}

void task_rgb_btn_deinit(void)
{
	// Use the handle to delete the task.
	if(_task_rgb_btn.task.handle != NULL )
		vTaskDelete(_task_rgb_btn.task.handle);

}

#undef PRINTF_TAG

/*************************** END OF FILE *************************************/
