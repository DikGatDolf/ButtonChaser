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
#include "colour.h"

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

#define GAME_RANDOM_CHASE_BLINK_PERIOD_MS_DEF   500
#define GAME_RANDOM_CHASE_BLINK_PERIOD_MS_MIN   50
#define GAME_RANDOM_CHASE_BLINK_COL_1_DEF       colRed
#define GAME_RANDOM_CHASE_BLINK_COL_2_DEF       colGreen
#define GAME_RANDOM_CHASE_OFF_COL_3_DEF         colBlack

#define GAME_RANDOM_CHASE_BTN_TIMEOUT_MAX       (5*60)
#define GAME_RANDOM_CHASE_BTN_TIMEOUT_DEF       10

#define GAME_RANDOM_CHASE_BLINK_UPDATE_CNT      2
/*******************************************************************************
local defines 
 *******************************************************************************/
typedef enum
{
    new_blink_period = 0x01,
    new_blink_col_1 = 0x02,
    new_blink_col_2 = 0x04,
    new_off_col_3 = 0x08,
} _random_chase_new_param_e;

typedef enum
{
    _chase_state_set = 0,
    _chase_state_read,    
} _chase_state_t;
/*******************************************************************************
local function prototypes
 *******************************************************************************/

/*******************************************************************************
local variables
 *******************************************************************************/
//uint8_t _new_params = 0x00; //Flag to indicate that new parameters have been set
uint32_t _blink_period = GAME_RANDOM_CHASE_BLINK_PERIOD_MS_DEF;
uint32_t _col[3] = {
    GAME_RANDOM_CHASE_BLINK_COL_1_DEF,
    GAME_RANDOM_CHASE_BLINK_COL_2_DEF,
    GAME_RANDOM_CHASE_OFF_COL_3_DEF
};
uint32_t _tmp_btn_timeout = GAME_RANDOM_CHASE_BTN_TIMEOUT_DEF;
Timer_ms_t _btn_timer = {0}; // Timer for the button timeout
// uint8_t _blink_update_cnt = 0; // Counter for the number of times we have updated the blink period and colours

uint32_t _btn_timeout_ms;
uint8_t _chase_node = ADDR_BROADCAST; // The address of the node we are chasing
button_t * _chase_node_btn = NULL;
_chase_state_t _chase_state = _chase_state_set; // The current state of the chase game
/*******************************************************************************
Local (private) Functions
*******************************************************************************/

/*******************************************************************************
Global (public) Functions
*******************************************************************************/
void game_random_chase_main(void)
{
    //This function is going to be called repeatedly within a task's while(1) loop.
    switch (_chase_state)
    {
        case _chase_state_set:
        {
            uint8_t _new_node;
            bcst_msg_clear_all();
            do
            {
                _new_node = (uint8_t)(rand() % (node_count())); //Get a random node address
            } while (_new_node == _chase_node || !is_node_valid(_new_node)); //Ensure we don't select the same node or an invalid node

            _chase_node = _new_node; //Get a random node address, including the broadcast address

            init_node_msg(_chase_node); //Initialize the node message with the selected node address
            add_node_msg_set_blink(_chase_node, _blink_period); //Set the node to blink
            add_node_msg_set_rgb(_chase_node, 0, _col[0]); //Set the first RGB colour
            add_node_msg_set_rgb(_chase_node, 1, _col[1]); //Set the second RGB colour
            add_node_msg_set_rgb(_chase_node, 2, _col[2]); //Set the third RGB colour
            add_node_msg_set_active(_chase_node, true); //Set the node as active
            if (!node_msg_tx_now(_chase_node)) //Send the message to the node
            {
                iprintln(trGAME|trALWAYS, "#Error: Could not activate node %d", _chase_node);
                //At this point, the node would have been deregistered, so we need to select a new node
                _chase_state = _chase_state_set; //Move to the new state to select a new node
            }
            else
            {
                // iprint(trGAME, "#Press Button #%d", _chase_node);
                if (_tmp_btn_timeout > 0)
                {
                    //Start the button timeout timer
                    // iprint(trGAME, " within %d s", _btn_timeout_ms / 1000);
                    sys_poll_tmr_start(&_btn_timer, _btn_timeout_ms, false); //Start the timer for the button timeout... remember to convert seconds to milliseconds
                    // _blink_update_cnt = 0;
                }
                // iprintln(trGAME, "");
                _chase_state = _chase_state_read; //Move to the read state to wait for a response
                _chase_node_btn = get_node_button_ptr(_chase_node); //Get the button pointer for the node
                _chase_node_btn->reaction_ms = 0; //Reset the reaction time for the "Active" node
            }

            break;
        }
        case _chase_state_read:
        {
            //We want to read the responses from the active node
            init_node_msg(_chase_node); //Initialize the node message with the selected node address
            add_node_msg_get_reaction(_chase_node); //Get the reaction time from the node
            if (!node_msg_tx_now(_chase_node)) //Send the message to the node
            {
                iprintln(trGAME|trALWAYS, "#Error: Could not read node %d", _chase_node);
                //At this point, the node would have been deregistered, so we need to select a new node
                _chase_state = _chase_state_set; //Move to the new state to select a new node
            }
            else
            {
                if (_chase_node_btn->reaction_ms > 0)
                {
                    //Whoop Whoop! We got a reaction from the node!
                    iprintln(trGAME, "#Node %d: Button pressed in %d ms", _chase_node, _chase_node_btn->reaction_ms);
                    _chase_state = _chase_state_set; //Move to the new state to select a new node
                }
                else if (_btn_timeout_ms > 0)
                {
                    if (sys_poll_tmr_expired(&_btn_timer))
                    {
                        //iprintln(trGAME, "#Node %d: Button-press timeout after %d s", _chase_node, _btn_timeout_ms/1000);
                        init_node_msg(_chase_node); //Initialize the node message with the selected node address
                        add_node_msg_set_blink(_chase_node, 0); //Set the node to blink no more
                        add_node_msg_set_rgb(_chase_node, 0, _col[2]); //Set the third RGB colour
                        add_node_msg_set_active(_chase_node, false); //Set the node as inactive
                        if (!node_msg_tx_now(_chase_node)) //Send the message to the node
                            iprintln(trGAME|trALWAYS, "#Error: Could not de-activate node %d", _chase_node);
                        _chase_state = _chase_state_set; //Move to the new state to select a new node
                    }
                    else
                    {
                        //We want to re-adjust the blink period of the node to gradually increase the blinking rate
                        uint32_t _new_blink_rate = GAME_RANDOM_CHASE_BLINK_PERIOD_MS_MIN + ((_btn_timer.ms_expire - sys_poll_tmr_ms()) * (_blink_period - GAME_RANDOM_CHASE_BLINK_PERIOD_MS_MIN)/_btn_timeout_ms); //Calculate the new blink rate based on the remaining time
                        init_node_msg(_chase_node); //Initialize the node message with the selected node address
                        add_node_msg_set_blink(_chase_node, _new_blink_rate); //Set the node to blink no more
                        if (!node_msg_tx_now(_chase_node)) //Send the message to the node
                            iprintln(trGAME|trALWAYS, "#Error: Could not adjust blink rate of node %d to %d ms", _chase_node, _new_blink_rate);
                        // else 
                        //     iprintln(trGAME, "#Set new blink rate: %u ms", _new_blink_rate);
                        _chase_state = _chase_state_read; //Move to the new state to select a new node
                    }
                }
                else
                {
                    _chase_state = _chase_state_read; //Move to the read state to wait for a response
                }
            }
            break;
        }
        default:
        {
            iprintln(trGAME|trALWAYS, "#Unknown chase state: %d", _chase_state);
            break;
        }
    }
}

void game_random_chase_init(bool startup, bool new_game_params)
{
    //The 1st thing we need to do is select one of the nodes to activate
    _btn_timeout_ms = ((new_game_params)? _tmp_btn_timeout :  GAME_RANDOM_CHASE_BTN_TIMEOUT_DEF) * 1000; //Convert the timeout to milliseconds
    if (startup)
    {
        _chase_state = _chase_state_set; // The current state of the chase game
        iprintln(trGAME|trALWAYS, "#Starting with a button timeout of %d s", _btn_timeout_ms / 1000);
    }
    else
    {
        iprintln(trGAME|trALWAYS, "#Changing button timeout to %d s", _btn_timeout_ms / 1000);
    }

}

void game_random_chase_teardown(void)
{
    //This function is called once to tear down the game.
    //It can be used to free any resources allocated during the game.
    _tmp_btn_timeout = GAME_RANDOM_CHASE_BTN_TIMEOUT_DEF;

}

bool game_random_chase_arg_parser(const char **arg_str_array, int arg_cnt, bool * new_game_params)
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
        if (value > GAME_RANDOM_CHASE_BTN_TIMEOUT_MAX)
        {
            iprintln(trALWAYS, "Invalid timeout value (%u s). Options are: 0 to %d s", value, GAME_RANDOM_CHASE_BTN_TIMEOUT_MAX);
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
        _tmp_btn_timeout = ((value > 0) ? value : GAME_RANDOM_CHASE_BTN_TIMEOUT_DEF);
        if (new_game_params != NULL)
            *new_game_params = true; //Indicate that new parameters have been set
    }

    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        iprintln(trALWAYS, "");
        iprintln(trALWAYS, "%s Parameters:", PRINTF_TAG);
        iprintln(trALWAYS, " <timeout>: The period to wait for a btn to be pressed in s (0 to %d)", GAME_RANDOM_CHASE_BTN_TIMEOUT_MAX);
        iprintln(trALWAYS, "        If omitted, a default period of %d s is used", GAME_RANDOM_CHASE_BTN_TIMEOUT_DEF);
        iprintln(trALWAYS, "        If set to 0, the game will wait indefinitely for a button press");
    }
    return true;
}

#undef PRINTF_TAG

/*************************** END OF FILE *************************************/
