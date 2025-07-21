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
typedef enum
{
    game_state_idle     = 0,    // Waiting for a game to start
    game_state_node_reg = 1,    // Waiting for rollcall responses and subsequent node registration to complete
    game_state_init     = 2,    // Initialising the game
    game_state_running  = 3,    // A game is currently running
    game_state_paused   = 4,    // A game is paused
} game_state_e;


typedef struct
{
//    bool is_running;
	TaskInfo_t task;
    int current_game; // Index of the current game being run
    game_state_e state; // Current state of the game task
} GameTask_t;
/*******************************************************************************
local function prototypes
 *******************************************************************************/
void _task_game_mainfunc(void * pvParameters);
void _task_game_setup(void);


/*******************************************************************************
local variables
 *******************************************************************************/

GameTask_t _game = {
    //.is_running = false,
    .task = {
        .stack_depth = RGB_STACK_SIZE,
        .parameter_to_pass = NULL,
        .handle = NULL
    },
    .current_game = -1,
};

bool _pause_flag = false; //Flag to indicate that the game is paused
bool _new_params = false; //Flag to indicate that new parameters have been set

/*******************************************************************************
Local (private) Functions
*******************************************************************************/

void _task_game_mainfunc(void * pvParameters)
{
    // uint16_t hue = 0;

    //TickType_t xLastWakeTime = xTaskGetTickCount();

    if ((_game.current_game < 0 || _game.current_game >= games_cnt()))
    {
        iprintln(trGAME|trALWAYS, "#Cannot start game %d", _game.current_game);
        _game.current_game = -1;
    }
    else
    {
        bool all_good = true;

        while (all_good) 
        {
            TickType_t xLastWakeTime = xTaskGetTickCount();

            if ((node_count() <= 0) && (_game.state > game_state_node_reg)) //Check if there are any nodes registered
            {
                iprintln(trNODE, "#Error: No nodes registered");
                all_good = false; //Set the all_good flag to false to exit the loop
                continue; //Skip the rest of the loop and wait for the next iteration
            }

            // node_parse_rx_msg(); //Process any received messages, this will also update the nodes.list[x].btn fields with the responses

            if (_pause_flag)
            {
                _game.state = game_state_paused; //Set the game state to paused
                _pause_flag = false; //Reset the pause flag
            }
            
            switch (_game.state) //Replace with a meaningful condition
            {
                case game_state_node_reg:
                {
                    iprintln(trGAME, "#Starting Registration...");
                    if (!nodes_register_all())
                    {    
                        iprintln(trGAME|trALWAYS, "#Failed to register nodes");
                        all_good = false; //Set the all_good flag to false to exit the loop
                    }
                    _game.state = (games_list[_game.current_game].cb_init)? game_state_init : game_state_running;
                    break;
                }
                
                case game_state_init:
                {
                    iprintln(trGAME, "#Game Initialisation...");
                    if (games_list[_game.current_game].cb_init)
                        games_list[_game.current_game].cb_init(true, _new_params); //Call the game initialisation function if one is defined
                    _new_params = false; //Reset the new parameters flag
                    _game.state = game_state_running;
                    //Start with a clean slate
                    bcst_msg_clear_all();
                    break;
                }
                
                case game_state_running:
                {
                    if (_game.current_game < 0 || _game.current_game >= games_cnt())
                    {
                        iprintln(trGAME|trALWAYS, "#Invalid game index: %d", _game.current_game);
                        all_good = false; //Set the all_good flag to false to exit the loop
                    }
                    else if (!games_list[_game.current_game].cb_main)
                    {
                        iprintln(trGAME|trALWAYS, "#No main function for game %s", games_list[_game.current_game].name);
                        all_good = false; //Set the all_good flag to false to exit the loop
                    }
                    else
                    {
                        if (_new_params)
                        {
                            iprintln(trGAME, "#New parameters...");
                            if (games_list[_game.current_game].cb_init)
                                games_list[_game.current_game].cb_init(false, _new_params); //Call the game initialisation function if one is defined
                            _new_params = false; //Reset the new parameters flag
                        }
                        games_list[_game.current_game].cb_main();
                    }
                    break;
                }
                
                case game_state_paused:
                {
                    //Do nothing here... we *might* get resumed... just wait for the next iteration
                    break;
                }
                
                case game_state_idle:
                default:
                    all_good = false;
                    break;
            }

            xTaskDelayUntil(&xLastWakeTime, MAX(1, pdMS_TO_TICKS(TASK_GAME_INTERVAL_MS)));  //vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
            /* Inspect our own high water mark on entering the task. */
            _game.task.stack_unused = uxTaskGetStackHighWaterMark2( NULL );
        }
    }
    iprintln(trGAME|trALWAYS, "#Game Task Ended prematurely!");
    //game_end(); //End the game if we are not receiving any responses from the nodes

    // RVN - TODO figure out a way for the task to self-destruct if it is not running a game
}



/*******************************************************************************
Global (public) Functions
*******************************************************************************/
void * game_start(int index)
{
    if ((index < 0) || (index >= games_cnt()))
    {
        iprintln(trGAME|trALWAYS, "#Invalid game index: %d", index);
        return NULL;
    }

    if (index == _game.current_game)
    {
        iprintln(trGAME|trALWAYS, "#\"%s\" %d is already running", games_list[index].name, index);
        return &_game.task;
    }

	//Let's not restart this task by accident
    if (game_is_running())
    {
        iprintln(trGAME|trALWAYS, "#\"%s\" (%d) is running... Let's stop that first.", games_list[_game.current_game].name, _game.current_game);
        if ((_game.current_game >= 0) && (_game.current_game < games_cnt()))
        {
            iprintln(trGAME|trALWAYS, "#Stopping \"%s\"", games_list[index].name);
            game_end();
        }
        return NULL; //Nothing to do further
    }

    _game.current_game = index;
    _game.state = game_state_node_reg; //Set the game state to node registration
    _pause_flag = false; //Reset the pause flag

    // Create the task, storing the handle.  Note that the passed parameter ucParameterToPass
	// must exist for the lifetime of the task, so in this case is declared static.  If it was just an
	// an automatic stack variable it might no longer exist, or at least have been corrupted, by the time
	// the new task attempts to access it.
	if ((xTaskCreate( _task_game_mainfunc, PRINTF_TAG, _game.task.stack_depth, _game.task.parameter_to_pass, 1, &_game.task.handle ) != pdPASS) ||
		(_game.current_game < 0))
	{
		iprintln(trGAME|trALWAYS, "#Unable to start \"%s\"!", games_list[_game.current_game].name);
        game_end(); //End the game if we cannot start the task
		return NULL;
	}

    iprintln(trGAME|trALWAYS, "#Started \"%s\" @ %d Hz", games_list[_game.current_game].name, (1000/TASK_GAME_INTERVAL_MS));

    configASSERT(_game.task.handle);

    return &_game.task;
}

void game_end(void)
{
    //Stop the task if it is running
    if ((_game.current_game >= 0) && (_game.current_game < games_cnt()))
    {        
        if (_game.task.handle != NULL)
        {
            game_pause(); //Pause the game first
            if (games_list[_game.current_game].cb_teardown)
                games_list[_game.current_game].cb_teardown();
            _new_params = false; //Reset the new parameters flag
            //Turn off all the LED's, as a matter of courtesy
            bcst_msg_clear_all();
            vTaskDelete(_game.task.handle);
            iprintln(trGAME|trALWAYS, "#\"%s\" Stopped", games_list[_game.current_game].name);
        }

        _game.current_game = -1;
        _game.state = game_state_idle; //Reset the game state to idle
    }
    else
    {
        iprintln(trGAME|trALWAYS, "#No game is currently running, nothing to stop");
    }
}

bool game_is_running(void)
{
    return (_game.state != game_state_idle)? true : false;
    // return ((_game.current_game >= 0) && (_game.current_game < games_cnt()))? true : false;
}

bool game_is_paused(void)
{
    return ((_game.state == game_state_paused) || (_pause_flag))? true : false;
    // return ((_game.current_game >= 0) && (_game.current_game < games_cnt()))? true : false;
}

bool game_pause(void)
{
    if ((_game.task.handle == NULL) || (_game.current_game < 0) || (_game.current_game >= games_cnt()) || (_game.state == game_state_idle))
    {
        //If the task handle is NULL, or the game index is invalid, we cannot pause the game
        //This can happen if the game was never started, or if it has been stopped
        iprintln(trGAME|trALWAYS, "#Cannot pause game, task handle is NULL or invalid game index: %d", _game.current_game);
        return false;
    }

    if ((_game.state == game_state_paused) || (_pause_flag))
    {
        iprintln(trGAME|trALWAYS, "#Game \"%s\" is already paused", games_list[_game.current_game].name);
        return true; //Game is already paused, so we can return true
    }

    //We just set the _pause_flag to true, the game task will check this flag once it has completed all pending communications with nodes and THEN go into a pause state
    if (!_pause_flag)
        _pause_flag = true; //Set the pause flag to indicate that the game is paused

    while (_game.state != game_state_paused)
        vTaskDelay(1); //Allow other tasks to run while we wait for the game to pause

    iprintln(trGAME, "#Paused");

    vTaskSuspend(_game.task.handle); //Suspend the task first
    return true; //Return true to indicate that the game was paused successfully
}

bool game_resume(void)
{
    if ((_game.task.handle == NULL) || (_game.current_game < 0) || (_game.current_game >= games_cnt()))
    {
        //If the task handle is NULL, or the game index is invalid, we cannot resume the game
        //This can happen if the game was never started, or if it has been stopped
        iprintln(trGAME|trALWAYS, "#Cannot resume game, task handle is NULL or invalid game index: %d", _game.current_game);
        return false;
    }
    if ((_pause_flag) && (_game.state != game_state_paused))
    {
        //we might need to wait for the game to properly pause before we can resume it
        vTaskDelay(pdMS_TO_TICKS(TASK_GAME_INTERVAL_MS)); //Allow other tasks to run while we wait for the game to pause
    }
    if (_game.state != game_state_paused)
    {
        //If the game is already paused, we can resume it
        iprintln(trGAME|trALWAYS, "#Game \"%s\" is not paused (flag: %s)", games_list[_game.current_game].name, (_pause_flag) ? "set" : "cleared");
        return true; //Return true to indicate that the game was resumed successfully
    }
    _game.state = game_state_running;
    _pause_flag = false; //Reset the pause flag
    iprintln(trGAME|trALWAYS, "#Resuming \"%s\" (%d)", games_list[_game.current_game].name, _game.current_game);
    vTaskResume(_game.task.handle); //Resume the task
    return true;
}

int current_game(void)
{
    return game_is_running()? _game.current_game : -1;
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

bool game_parse_args(int game_index, const char **arg_str_array, int arg_cnt)
{
    if ((game_index < 0) || (game_index >= games_cnt()))
    {
        iprintln(trGAME|trALWAYS, "#Invalid game index: %d", game_index);
        return false;
    }

    if (arg_cnt < 1)
        return true; //No arguments to parse, so we return true

    //Valid Array and callback function provided, so we can parse the arguments
    if ((!arg_str_array) || (!games_list[game_index].cb_arg_parse))
        return true;
        
    return games_list[game_index].cb_arg_parse(arg_str_array, arg_cnt, &_new_params);
}

#undef PRINTF_TAG

/*************************** END OF FILE *************************************/
