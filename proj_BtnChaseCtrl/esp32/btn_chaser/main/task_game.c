/*******************************************************************************

Module:     task_game.c
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
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "sys_utils.h"
#include "sys_timers.h"
#include "sys_task_utils.h"
#include "str_helper.h"
#include "task_console.h"
#include "nodes.h"

#define __NOT_EXTERN__
#include "task_game.h"
#undef __NOT_EXTERN__

#include "game_random_chase.h"
#include "game_demo.h"

/*******************************************************************************
Macros and Constants
 *******************************************************************************/
const game_t games_list[] =
{
    {"Demo",            game_demo_main,         game_demo_init,         game_demo_teardown,         game_demo_arg_parser        },
    {"Random Chase",    game_random_chase_main, game_random_chase_init, game_random_chase_teardown, game_random_chase_arg_parser},
};

#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("Game") /* This must be undefined at the end of the file*/

#define RGB_STACK_SIZE 4096

/*******************************************************************************
local defines 
 *******************************************************************************/
typedef struct
{
//    bool is_running;
	TaskInfo_t task;
    int current_game_index; // Index of the current game being run
} GameTask_t;

/*******************************************************************************
local function prototypes
 *******************************************************************************/
void _task_game_mainfunc(void * pvParameters);
void _task_game_setup(void);


/*******************************************************************************
local variables
 *******************************************************************************/

GameTask_t _game_task = {
    //.is_running = false,
    .task = {
        .stack_depth = RGB_STACK_SIZE,
        .parameter_to_pass = NULL,
        .handle = NULL
    },
    .current_game_index = -1,
};

/*******************************************************************************
Local (private) Functions
*******************************************************************************/

void _task_game_mainfunc(void * pvParameters)
{
    // uint16_t hue = 0;

    //TickType_t xLastWakeTime = xTaskGetTickCount();

    if ((_game_task.current_game_index < 0 || _game_task.current_game_index >= games_cnt()))
    {
        iprintln(trGAME|trALWAYS, "#Cannot start game %d", _game_task.current_game_index);
        _game_task.current_game_index = -1;
    }
    else
    {
        games_list[_game_task.current_game_index].init(); //Call the init function for the game
        while (1) 
        {
            TickType_t xLastWakeTime = xTaskGetTickCount();
            games_list[_game_task.current_game_index].main();

            xTaskDelayUntil(&xLastWakeTime, MAX(1, pdMS_TO_TICKS(TASK_GAME_INTERVAL_MS)));  //vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
            /* Inspect our own high water mark on entering the task. */
            _game_task.task.stack_unused = uxTaskGetStackHighWaterMark2( NULL );
        }
    }
    iprintln(trGAME|trALWAYS, "#Game Task Ended prematurely!");
}



/*******************************************************************************
Global (public) Functions
*******************************************************************************/
void * start_game(int index)
{
    if ((index < 0) || (index >= games_cnt()))
    {
        iprintln(trGAME|trALWAYS, "#Invalid game index: %d", index);
        return NULL;
    }

    if (index == _game_task.current_game_index)
    {
        iprintln(trGAME|trALWAYS, "#\"%s\" %d is already running", games_list[index].name, index);
        return &_game_task.task;
    }

	//Let's not restart this task by accident
    if ((_game_task.current_game_index >= 0) && (_game_task.current_game_index < games_cnt()))
    {
        iprintln(trGAME|trALWAYS, "#Stopping \"%s\"", games_list[index].name);
        end_game();
    }

    _game_task.current_game_index = index;

    // Create the task, storing the handle.  Note that the passed parameter ucParameterToPass
	// must exist for the lifetime of the task, so in this case is declared static.  If it was just an
	// an automatic stack variable it might no longer exist, or at least have been corrupted, by the time
	// the new task attempts to access it.
	if ((xTaskCreate( _task_game_mainfunc, PRINTF_TAG, _game_task.task.stack_depth, _game_task.task.parameter_to_pass, 1, &_game_task.task.handle ) != pdPASS) ||
		(_game_task.current_game_index < 0))
	{
		iprintln(trGAME|trALWAYS, "#Unable to start %s Game Task!", PRINTF_TAG);
		return NULL;
	}

    iprintln(trGAME|trALWAYS, "#Started \"%s\" @ %d Hz", games_list[_game_task.current_game_index].name, (1000/TASK_GAME_INTERVAL_MS));

    configASSERT(_game_task.task.handle);

    return &_game_task.task;
}

void end_game(void)
{
    //Stop the task if it is running
    if ((_game_task.current_game_index >= 0) && (_game_task.current_game_index < games_cnt()))
    {        
        if (_game_task.task.handle != NULL)
        {
            iprintln(trGAME|trALWAYS, "#Pausing \"%s\"", games_list[_game_task.current_game_index].name);
            vTaskSuspend(_game_task.task.handle); //Suspend the task first
            games_list[_game_task.current_game_index].teardown();
            vTaskDelete(_game_task.task.handle);
            iprintln(trGAME|trALWAYS, "#\"%s\" Stopped", games_list[_game_task.current_game_index].name);
        }

        _game_task.current_game_index = -1;
    }
}

bool is_game_running(void)
{
    return ((_game_task.current_game_index >= 0) && (_game_task.current_game_index < games_cnt()))? true : false;
}

int current_game(void)
{
    return is_game_running()? _game_task.current_game_index : -1;
}

inline int games_cnt(void)
{
    return ARRAY_SIZE(games_list);
}

const char * game_name(int index)
{
    if ((index < 0) || (index >= games_cnt()))
        return "Invalid game index";

    return games_list[index].name;
}

bool parse_game_args(int game_index, const char **arg_str_array, int arg_cnt)
{
    if ((game_index < 0) || (game_index >= games_cnt()))
    {
        iprintln(trGAME|trALWAYS, "#Invalid game index: %d", game_index);
        return false;
    }

    if (arg_cnt < 1)
        return true; //No arguments to parse, so we return true

    //Valid Array and callback function provided, so we can parse the arguments
    if ((!arg_str_array) || (!games_list[game_index].arg_parse))
        return true;
        
    return games_list[game_index].arg_parse(arg_str_array, arg_cnt);
}

#undef PRINTF_TAG

/*************************** END OF FILE *************************************/
