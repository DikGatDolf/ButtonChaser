/*******************************************************************************

Module:     game_memory.c
Purpose:    This file contains the RGB LED driver task
Author:     Rudolph van Niekerk


This game will work as follows:
    1. The game will start with a random sequence of button presses
        In the 1st round, the 1st button of the sequence is displayed and the user needs to press it.
        In the 2nd round, the 1st button and then the second button is displayed and the user needs to press them in the same order.
        This will continue until the user has pressed all buttons in the sequence correctly for the last round.
    2. A round starts with the system blinking the first <ROUND_NR> of buttons in order.
    3. Then flashes blue to indicate the start of the user input stage
    4. The user then needs to press the buttons in the same order as they were displayed.
        If the user presses the wrong button, the round will be reset and the user will have to start over after the buttons has been displayed again.
    5. The game will end when the user has pressed all buttons in the sequence correctly for the last round or if the user has pressed the incorrect sequence of buttons more than 3 times consecutively.
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
#include "esp_random.h"

#define __NOT_EXTERN__
#include "game_memory.h"
#undef __NOT_EXTERN__

/*******************************************************************************
Macros and Constants
 *******************************************************************************/

#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("Memory") /* This must be undefined at the end of the file*/

/*******************************************************************************
local defines 
 *******************************************************************************/
#define GAME_MEMORY_BLINK_PERIOD_MIN_MS             100
#define GAME_MEMORY_BLINK_PERIOD_MAX_MS             1500
#define GAME_MEMORY_BLINK_PERIOD_DEF_MS             400
#define GAME_MEMORY_BLINK_PERIOD_OFF_MS             400

#define GAME_MEMORY_LEVELS                  20

typedef struct
{
    uint8_t btn;
//    uint32_t colour;
    Stopwatch_ms_t sw;
} _memory_round_t;

typedef enum
{
    _mem_state_start = 0,
    _mem_state_start_on,
    _mem_state_blink_off,
    _mem_state_blink_on,
    _mem_state_usr_input_wait,
    _mem_state_user_blink_off,   
    _mem_state_win,   
} _memory_state_t;

/*******************************************************************************
local function prototypes
 *******************************************************************************/

void _memory_blink_all_on_off(uint32_t colour, uint32_t time_ms);
void _memory_blink_node_on_off(uint8_t btn, uint32_t colour, uint32_t time_ms);
_memory_state_t _memory_usr_input_stage_start(void);

/*******************************************************************************
local variables
 *******************************************************************************/
uint32_t _memory_col_list[] = {colNavy, colBlue, colGreen, colTeal, colLime, colCyan, colMaroon, colPurple, colMagenta, colRed, colOrange, colYellow, colWhite};
_memory_round_t _round[GAME_MEMORY_LEVELS] = {0}; // The array of rounds


uint8_t _btn_pressed = 0xff; //The button the user pressed (0 to node_count()-1 or 0xff for no button pressed)
uint8_t _game_level = 0; // The current level of the game
uint8_t _game_level_display = 0; // The current level of the game
uint8_t _user_level = 0; // The current level of the user


uint32_t _tmp_blink_ms;
//uint32_t _tmp_blink_wait_ms;
uint32_t _blink_ms = GAME_MEMORY_BLINK_PERIOD_DEF_MS; // The blink period in milliseconds
_memory_state_t _memory_state = _mem_state_start; // The current state of the memory game
Timer_ms_t _memory_tmr = {0}; // Timer for the memory game
/*******************************************************************************
Local (private) Functions
*******************************************************************************/
void _memory_blink_all_on_off(uint32_t colour, uint32_t time_ms)
{
    init_bcst_msg(); //Initialize the broadcast message
    add_bcst_msg_set_rgb(0, colour); //Set the first RGB colour to green
    bcst_msg_tx_now(); //Send the message to the nodes
    if (time_ms > 0)
        sys_poll_tmr_start(&_memory_tmr, time_ms, false); //Start
}

void _memory_blink_node_on_off(uint8_t btn, uint32_t colour, uint32_t time_ms)
{
    //if (colour != colBlack)
    //iprintln(trGAME, "#Round %d/%d: %d - %s (%d ms)", _game_level_display, _game_level, _round[_game_level_display].btn, rgb2name(colour), time_ms);
    init_node_msg(btn); //Initialize the node message with the selected node address
    add_node_msg_set_rgb(btn, 0, colour); //Set the first RGB colour to black
    node_msg_tx_now(btn); //Send the message to the node
    if (time_ms > 0)
        sys_poll_tmr_start(&_memory_tmr, time_ms, false); //Start the timer for the blink period
}

_memory_state_t _memory_usr_input_stage_start(void)
{
    //Great, this is the start of the user input stage... Actiavate all the buttons and set their 3rd colour to either red or green (for the correct button)
    for (int i = 0; i < node_count(); i++)
    {
        init_node_msg(i); //Initialize the node message with the selected node address
        add_node_msg_set_blink(i, 0);
        add_node_msg_set_rgb(i, 0, colBlack); //Set the first RGB colour to black
        add_node_msg_set_rgb(i, 2, (_round[_user_level].btn == i)? colGreen : colRed); //Set the the colour
        add_node_msg_set_active(i, true); //Set the node as active
        node_msg_tx_now(i); //Send the message to the node
        sys_stopwatch_ms_start(&_round[_user_level].sw, UINT32_MAX); //Start the stopwatch for the button press
    }
    _btn_pressed = 0xff; //Reset the button pressed to no button pressed            
    iprintln(trGAME, "#Round %d/%d, Waiting for user input (%d)", _user_level, _game_level, _round[_user_level].btn);
    return _mem_state_usr_input_wait; //Move to the wait user input state
}
/*******************************************************************************
Global (public) Functions
*******************************************************************************/
void game_memory_main(void)
{
    switch (_memory_state)
    {
        case _mem_state_start:
        {
            if (sys_poll_tmr_started(&_memory_tmr))
            {
                if (!sys_poll_tmr_expired(&_memory_tmr))
                    break;
                // else, happy to fall through now.
            } // else, fall through


            //Make sure all buttons are deactivated and set to black
            for (int i = 0; i < node_count(); i++)
            {
                init_node_msg(i); //Initialize the node message with the selected node address
                add_node_msg_set_blink(i, 0); //Set the node to blink
                add_node_msg_set_rgb(i, 0, colBlack); //Set the first RGB colour
                add_node_msg_set_rgb(i, 1, colBlack); //Set the second RGB colour
                add_node_msg_set_rgb(i, 2, colBlack); //Set the third RGB colour
                add_node_msg_set_active(i, false); //Set the node as inactive
                node_msg_tx_now(i); //Send the message to the node
            }
            _game_level_display = 0; //Reset the game level display to 0
            iprintln(trGAME, "#Starting level %d (%d)", _game_level, _round[_game_level_display].btn);
            //RVN - TODO - Depending ont the retry count, this colour could be green... orange.... red.
            _memory_blink_all_on_off(colGreen, _blink_ms);
            _memory_state =  _mem_state_start_on;
            break;
        }
        case _mem_state_start_on:
        {
            if (sys_poll_tmr_expired(&_memory_tmr)) //Check if the timer has expired
            {
                _memory_blink_all_on_off(colBlack, _blink_ms);
                _memory_state =  _mem_state_blink_off;
            }    
            break;
        }
        case _mem_state_blink_off:
        {
            //This state is only entered if (_game_level_display < _game_level)
            if (!sys_poll_tmr_expired(&_memory_tmr)) //Check if the timer has expired
                break; //If the timer has not expired, we are still waiting for the blink period to end

            //We should clear the buttons which are all displaying green now...
            if (0 == _game_level_display)
            {
                init_bcst_msg();
                add_bcst_msg_set_rgb(0, colBlack);
                bcst_msg_tx_now();
            }

            _memory_blink_node_on_off(_round[_game_level_display].btn, colBlue, _blink_ms);
            _memory_state = _mem_state_blink_on;
            break;
        }
        case _mem_state_blink_on:
        {
            if (sys_poll_tmr_expired(&_memory_tmr)) //OK, blink period expired....  we can turn the button off and then  we need to decide  if we are displayign the next button or are we moving to the user input stage
            {
                //OK, blink period expired.... we can blink the next button
                if (_game_level_display < _game_level)
                {
                    _memory_blink_node_on_off(_round[_game_level_display].btn, colBlack, GAME_MEMORY_BLINK_PERIOD_OFF_MS);
                    _memory_state = _mem_state_blink_off;
                    _game_level_display++; //Increment the game level display count
                }
                else
                {
                    _user_level = 0; //Reset the user level count to 0
                    _memory_state = _memory_usr_input_stage_start(); //Move to the wait user input state
                }
            }
            break; //If the timer has not expired, we are still waiting for the blink ON period to end
        }

        case _mem_state_usr_input_wait:
        {
            uint32_t time_since_button_pressed = 0;
            uint32_t time_to_btn_pressed = UINT32_MAX;

            //Run through all the nodes since more than one button could hav been pressed... we need to know which one was pressed first
            for (int i = 0; i < node_count(); i++)
            {
                init_node_msg(i); //Initialize the node message with the selected node address
                add_node_msg_get_reaction(i); //Read the flags state before we set them active to prevent false positives
                node_msg_tx_now(i); //Send the message to the node

                uint32_t ttbp = get_node_btn_reaction_ms(i);
                if (ttbp == 0) //If the node has no reaction time, it was not pressed
                    continue; //Skip this node

                if (ttbp < time_to_btn_pressed) //If this node has a reaction time shorter than the current shortest reaction time
                {
                    time_to_btn_pressed = ttbp; //Set the new shortest reaction time
                    _btn_pressed = i; //Set the button pressed to the node address
                }
                //else, we have the shortest time already
            }
            
            if (_btn_pressed == 0xff) //If no button was pressed
                break; //Exit the switch

            sys_poll_tmr_start(&_memory_tmr, MAX(1, _blink_ms-time_since_button_pressed), false); //Start the timer for the blink period
            // Make sure that only the button that was pressed is turned on (either green or red depending on if it was the correct button) and ALL other buttons are turned off
            for (int i = 0; i < node_count(); i++)
            {
                if ((i == _btn_pressed))
                    continue; //Skip this node... it is already pressed and activated
                uint32_t _col = (i == _btn_pressed)? ((_btn_pressed == _round[_user_level].btn)? colGreen : colRed) : colBlack;
                init_node_msg(i); //Initialize the node message with the selected node address                
                add_node_msg_set_rgb(i, 0, _col); //Set the 1st RGB colour for the buttons already pressed
                add_node_msg_set_rgb(i, 2, _col); //Set the 3rd RGB colour for the buttons we are going to deactivate now
                add_node_msg_set_active(i, false); //Set all the nodes as inactive
                node_msg_tx_now(i); //Send the message to the node

                //RVN -TODO consider doing this with a very special broadcast message instead of individual messages
            }

            time_since_button_pressed = sys_stopwatch_ms_stop(&_round[_user_level].sw) - time_to_btn_pressed; //Get the time since the button was pressed
            //iprintln(trGAME, "#Button %d pressed %d ms ago (reaction time: %d ms)", _btn_pressed, time_since_button_pressed, time_to_btn_pressed);
            //Make sure the button is lit for a period of time
            sys_poll_tmr_start(&_memory_tmr, MAX(GAME_MEMORY_BLINK_PERIOD_MIN_MS, _blink_ms-time_since_button_pressed), false); //Start the timer for the blink period
            _memory_state = _mem_state_user_blink_off; //Move to the win state
            break;
        }

        case _mem_state_user_blink_off:
        {
            // At this point, the only button lit should be the button that was pressed, and it should be green or red., and the timer should have been started...
            if (sys_poll_tmr_expired(&_memory_tmr)) //OK, blink period expired....  we can turn the button off and then  we need to decide  if we are displayign the next button or are we moving to the user input stage
            {
                _memory_blink_node_on_off(_btn_pressed, colBlack, 0);
                _memory_state = _mem_state_blink_off;
            }

            if (_btn_pressed == _round[_user_level].btn) //If the button pressed is the correct button, it would already be turned the correct colour
            {
                //iprintln(trGAME, "#You beat round %d/%d!", _user_level, _game_level);
                if (_user_level < _game_level)
                {
                    _user_level++; //Increment the user level count
                    _memory_state = _memory_usr_input_stage_start(); //Move to the wait user input state
                    break;
                }
                //User has reached the end of the round
                if (_game_level >= GAME_MEMORY_LEVELS) //If we have reached the last level
                {
                    iprintln(trGAME|trALWAYS, "#You won the GAME!");
                    _game_level = 0; // Reset the game level
                    //Decrease the blink time by 10%
                    _blink_ms = MAX((_blink_ms * 9 / 10), GAME_MEMORY_BLINK_PERIOD_MIN_MS);
                }
                else 
                {
                    iprintln(trGAME, "#You beat level %d!", _game_level);
                    _game_level++; //Increment the game level count
                }
            }
            else
            {
                iprintln(trGAME|trALWAYS, "#Wrong button pressed (%d iso %d ), resetting level %d", _btn_pressed, _round[_user_level].btn, _user_level);
                //RVN - TODO.... this should only be allowed a limited number of times.
            }
            _memory_blink_node_on_off(_btn_pressed, colBlack, GAME_MEMORY_BLINK_PERIOD_OFF_MS);
            _memory_state = _mem_state_start;
            break;
        }

        case _mem_state_win:
        {
            iprintln(trGAME|trALWAYS, "#Starting over...");
            _tmp_blink_ms = _blink_ms; //To re-use the same blink period in the next game
            game_memory_init(true, true);
            break;
        }
    }
}

void game_memory_init(bool startup, bool new_game_params)
{
    //This function is called once to initialise the game.
    //It can be used to set up any resources needed for the game.

    //The minimum timer value is 40ms (TASK_GAME_INTERVAL_MS).
    //Dividing the period by TASK_GAME_INTERVAL_MS gives us a count of how many task cycles it takes to loop through the 360 degree hue spectrum,
    //No need to start a timer... we use the task cycle interval to update the hue (if needed)
    if (startup)
    {
        _game_level = 0; //Reset the level count to 0
        _user_level = 0; //Reset the user level count to 0
        _memory_state = _mem_state_start; //Reset the memory state to start

        _blink_ms = (new_game_params)? _tmp_blink_ms : GAME_MEMORY_BLINK_PERIOD_DEF_MS; //Get the blink period from the parameters or use the default value
        //_blink_wait_ms = (new_game_params)? _blink_ms : GAME_MEMORY_BLINK_WAIT_PERIOD_DEF_MS; //Get the blink wait period from the parameters or use the default value
        iprintln(trGAME, "#Starting up with a blink period of %d ms:", _blink_ms);
        for (int i = 0; i < GAME_MEMORY_LEVELS; i++)
        {
            _round[i].btn = (uint8_t)(esp_random() % node_count()); //Generate a random level value between 0 and the number of nodes
            // _round[i].colour =  _memory_col_list[(esp_random() % ARRAY_SIZE(_memory_col_list))]; //Set the colour for the level
            iprintln(trGAME, "#Level %d: %d", i, _round[i].btn);
        }
        sys_poll_tmr_stop(&_memory_tmr);
    }
    else if (new_game_params)
    {
        _blink_ms = _tmp_blink_ms;
        iprintln(trGAME, "#Changing blink period to %d ms", _blink_ms);
    }
    else
    {
        iprintln(trGAME|trALWAYS, "#Unsolicited call to init()");
    }
}

void game_memory_teardown(void)
{
    //This function is called once to tear down the game.
    //It can be used to free any resources allocated during the game.
    _game_level = 0; //Reset the level count to 0
    _user_level = 0; //Reset the user level count to 0
    _memory_state = _mem_state_start; //Reset the memory state to start
    _tmp_blink_ms = GAME_MEMORY_BLINK_PERIOD_DEF_MS; //Reset the blink period to the default value
    _blink_ms = GAME_MEMORY_BLINK_PERIOD_DEF_MS; //Reset the blink period to the default value
    _btn_pressed = 0xff; //Reset the button pressed to no button pressed
}

bool game_memory_arg_parser(const char **arg_str_array, int arg_cnt, bool * new_game_params)
{
    bool help_requested = false;
    uint32_t value = 0;
    
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

    else if ((!strcasecmp("levels", arg)) || (!strcasecmp("lvls", arg)))
    {
        //The user wants to see the list of levels
        if (_memory_state != _mem_state_start)
        {
            iprintln(trALWAYS, "Memory Game Levels:");
            for (int i = 0; i < GAME_MEMORY_LEVELS; i++)
                iprintln(trGAME, "#%d: %d", i, _round[i].btn);
        }
        else
        {
            iprintln(trALWAYS, "Nothing to display,game not started yet");
        }
        return true; //No need to continue parsing arguments
        // break; //from while-loop
    }

    else if ((!strcasecmp("default", arg)) || (!strcasecmp("def", arg)))
    {
        value = GAME_MEMORY_BLINK_PERIOD_DEF_MS;
    }

    else if (str2uint32(&value, arg, 0)) 
    {
        if ((value < GAME_MEMORY_BLINK_PERIOD_MIN_MS) || (value > GAME_MEMORY_BLINK_PERIOD_MAX_MS))
        {
            iprintln(trALWAYS, "Invalid blink period value (%u ms). Options are: %d to %d ms", value, GAME_MEMORY_BLINK_PERIOD_MIN_MS, GAME_MEMORY_BLINK_PERIOD_MAX_MS);
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
        _tmp_blink_ms = (value > 0) ? value : GAME_MEMORY_BLINK_PERIOD_DEF_MS; //Set the blink period to the value specified or the default value
        if (new_game_params != NULL)
            *new_game_params = true; //Indicate that new parameters have been set
    }

    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        iprintln(trALWAYS, "");
        iprintln(trALWAYS, "%s Parameters:", PRINTF_TAG);
        iprintln(trALWAYS, " <period>: A value indicating the blink period in ms (%d to %d)", GAME_MEMORY_BLINK_PERIOD_MIN_MS, GAME_MEMORY_BLINK_PERIOD_MAX_MS);
        iprintln(trALWAYS, "        If omitted, a default period of %d ms is used", GAME_MEMORY_BLINK_PERIOD_DEF_MS);
    }
    return true;
}


#undef PRINTF_TAG

/*************************** END OF FILE *************************************/
