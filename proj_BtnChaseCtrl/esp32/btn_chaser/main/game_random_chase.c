/*******************************************************************************

Module:     game_random_chase.c
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
#include "game_random_chase.h"
#undef __NOT_EXTERN__

/*******************************************************************************
Macros and Constants
 *******************************************************************************/

#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("RandomChase") /* This must be undefined at the end of the file*/

/*******************************************************************************
local defines 
 *******************************************************************************/

/*******************************************************************************
local function prototypes
 *******************************************************************************/

/*******************************************************************************
local variables
 *******************************************************************************/

/*******************************************************************************
Local (private) Functions
*******************************************************************************/

/*******************************************************************************
Global (public) Functions
*******************************************************************************/
void game_random_chase_main(void)
{
    //This function is going to be called repeatedly within a task's while(1) loop.
}

void game_random_chase_init(void)
{
    //This function is called once to initialise the game.
    //It can be used to set up any resources needed for the game.
}

void game_random_chase_teardown(void)
{
    //This function is called once to tear down the game.
    //It can be used to free any resources allocated during the game.
}

bool game_random_chase_arg_parser(const char **arg_str_array, int arg_cnt)
{
    return true;
}

#undef PRINTF_TAG

/*************************** END OF FILE *************************************/
