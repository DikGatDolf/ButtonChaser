/*******************************************************************************

Module:     game_demo.c
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
#include "task_game.h"
#include "colour.h"

#define __NOT_EXTERN__
#include "game_demo.h"
#undef __NOT_EXTERN__

/*******************************************************************************
Macros and Constants
 *******************************************************************************/

#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("Demo") /* This must be undefined at the end of the file*/

/*******************************************************************************
local defines 
 *******************************************************************************/
#define GAME_DEMO_PERIOD_MIN_MS     1800    /* Cycling through the spectrum in 1.8s */
#define GAME_DEMO_PERIOD_MAX_MS     (0xFFFF * TASK_GAME_INTERVAL_MS)    /* Cycling through the spectrum in 21m 50.7s */
#define GAME_DEMO_PERIOD_MS_DEF     60000

/*******************************************************************************
local function prototypes
 *******************************************************************************/

/*******************************************************************************
local variables
 *******************************************************************************/

uint32_t _total_cycles;
uint32_t _count;
uint32_t _tmp_cycle_period = GAME_DEMO_PERIOD_MS_DEF;

/*******************************************************************************
Local (private) Functions
*******************************************************************************/

/*******************************************************************************
Global (public) Functions
*******************************************************************************/
void game_demo_main(void)
{
    //This function is going to be called repeatedly within a task's while(1) loop.
    uint32_t rgb = hue2rgb((HUE_MAX *_count/_total_cycles)%HUE_MAX);

    // //Get the flags for each node
    // for (int i = 0; i < nodes_get_count(); i++)
    // {
    //     if (nodes_get_type(i) == nodeTypeRGB)
    //     {
    //         rgb |= nodes_get_address_mask(i);
    //     }
    // }

    init_bcst_msg();
    if (add_bcst_msg_set_rgb(0, rgb))
        bcst_msg_tx_now();
    else
        iprintln(trGAME, "#Failed to create broadcast message");

    if ((++_count) >= _total_cycles)
        _count = 0;
}

void game_demo_init(bool startup, bool new_game_params)
{
    //This function is called once to initialise the game.
    //It can be used to set up any resources needed for the game.

    //The minimum timer value is 40ms (TASK_GAME_INTERVAL_MS).
    //Dividing the period by TASK_GAME_INTERVAL_MS gives us a count of how many task cycles it takes to loop through the 360 degree hue spectrum,
    //No need to start a timer... we use the task cycle interval to update the hue (if needed)
    if (startup)
    {
        _total_cycles = (((new_game_params)? _tmp_cycle_period : GAME_DEMO_PERIOD_MS_DEF) /TASK_GAME_INTERVAL_MS);
        _count = 0;
        iprintln(trGAME|trALWAYS, "#Starting with a period of %.1f s (%d cycles)", _tmp_cycle_period/1000.0f, _total_cycles);
    }
    else
    {
        uint32_t _new_total_cycles = (_tmp_cycle_period /TASK_GAME_INTERVAL_MS);
        if (_total_cycles != _new_total_cycles)
            _count = (_count * _new_total_cycles) / _total_cycles; //Adjust the count to the new total cycles
        _total_cycles = _new_total_cycles; //Update the total cycles to the new value
        iprintln(trGAME, "#Period changed to %.1f s (%d cycles)", _tmp_cycle_period/1000.0f, _total_cycles);
    }
}

void game_demo_teardown(void)
{
    //This function is called once to tear down the game.
    //It can be used to free any resources allocated during the game.
    _tmp_cycle_period = GAME_DEMO_PERIOD_MS_DEF; //Reset the period to the default value
}

bool game_demo_arg_parser(const char **arg_str_array, int arg_cnt, bool * new_game_params)
{
    bool help_requested = false;
    uint32_t value;
    
    const char *arg = *arg_str_array++;

    if ((arg == NULL) || (strlen(arg) == 0))
    {
        iprintln(trALWAYS, "Invalid argument (NULL)");
        help_requested = true;
        // continue; //Skip the rest of the loop and go to the next argument
    }

    else if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
    {
        help_requested = true;
        // break; //from while-loop
    }

    else if (str2uint32(&value, arg, 0)) 
    {
        if ((value < (GAME_DEMO_PERIOD_MIN_MS / 1000) || (value > (GAME_DEMO_PERIOD_MAX_MS / 1000))))
        {
            iprintln(trALWAYS, "Invalid period value (%u s). Options are: %d to %d s", value, GAME_DEMO_PERIOD_MIN_MS/1000, GAME_DEMO_PERIOD_MAX_MS/1000);
            help_requested = true;
        }
    }
    else 
    {

        iprintln(trALWAYS, "Invalid argument (\"%s\")", arg);
        help_requested = true;
        // break; //Skip the rest of the arguments... we only cater for 1
    }

    if (arg_cnt > 1)
    {
        iprint(trALWAYS, "#Ignoring ");
        if (arg_cnt > 2)
        {
            iprint(trALWAYS, "the rest of the arguments (");
            for (int i = 1; i < arg_cnt; i++)
                iprint(trALWAYS, "%s\"%s\"", (i > 1)? ", " : "", arg_str_array[i-1]);
            iprintln(trALWAYS, ")"); //Print the rest of the arguments
        }
        else
        {
            iprintln(trALWAYS, "the argument \"%s\"", arg_str_array[0]);
        }
    }

    // }

    if (!help_requested)
    {
        _tmp_cycle_period = (value > 0) ? value*1000 : GAME_DEMO_PERIOD_MS_DEF;
        if (new_game_params != NULL)
            *new_game_params = true; //Indicate that new parameters have been set
    }

    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        iprintln(trALWAYS, "");
        iprintln(trALWAYS, "%s Parameters:", PRINTF_TAG);
        iprintln(trALWAYS, " <period>: A value indicating the cycle period in s (%d to %d)", GAME_DEMO_PERIOD_MIN_MS / 1000, GAME_DEMO_PERIOD_MAX_MS / 1000);
        iprintln(trALWAYS, "        If omitted, a default period of %d s is used", GAME_DEMO_PERIOD_MS_DEF/1000);
    }
    return true;
}


#undef PRINTF_TAG

/*************************** END OF FILE *************************************/
