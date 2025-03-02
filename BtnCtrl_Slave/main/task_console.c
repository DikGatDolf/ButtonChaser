/*******************************************************************************

Module:     task_console.c
Purpose:    This file contains the console interface
Author:     Rudolph van Niekerk

The console device will parse all received input strings (terminated with a
carriage return & newline) on the debug port for functional data. The first
part of the input string is matched in the local structured array to check for a
matching command. All additional data will be treated as parameters.

The Console maintains a list of menu item GROUPS (Tables), not the items 
 themselves, which are added with the task_console_add_menu() function. 
 The menu items are statically declared in the caller task/file/module.
 A single menu item of  type ConsoleMenuItem_t, is a structure containing:
    the command
    a function pointer and 
    a description of the command.

To expand on the functions of the console, add your unique command, function
pointer and help text to the Console Menu structure array. Then add your
function (remember the prototype) to the bottom of the file. The pointer to
the start of the parameters (if any) will be passed as the parameter to your
function. NULL means no parameters followed the command.

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
#include "str_helper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define __NOT_EXTERN__
#include "task_console.h"
#undef __NOT_EXTERN__

/*******************************************************************************
local defines and constants
 *******************************************************************************/
#define PRINTF_TAG ("Console") /* This must be undefined at the end of the file*/

#define CONSOLE_RX_BUFF     80 /* Max string size of string we can RX on console */

#define CONSOLE_STACK_SIZE 4096

#define task_console_MAX_ARGS 15 /* The max amount of console arguments we can 
                                    parse. If more than these are passed, the 
                                    last argument placed on the stack will 
                                    contain the remainder of the argument string 
                                    as it was received on the console */

#define task_console_MAX_MENU_ITEMS_MAX 5 /* The maximum number of menu groups/tables */

const char BACKSPACE_ECHO[] = {0x08, 0x20, 0x08, 0x00};

const char BUILD_TIME_DATA[] = {__TIME__ " " __DATE__}; /* Used in our startup Banner*/

const char ARGUMENT_DELIMITERS[] = {' ', '\t', 0x00};

#if ( configUSE_TRACE_FACILITY == 1 )
const char * taskStateName[] = {
    "Running",
	"Ready",
	"Blocked",
	"Suspended",
	"Deleted",
	"Invalid",
//   0123456789
};
#endif

typedef enum
{
    eTraceON,
    eTraceOFF,
    eTraceTOGGLE,
}eTraceFlagAction;

/*******************************************************************************
local structure
 *******************************************************************************/
typedef struct
{
	const char *name;
	const char *description;
	ConsoleMenuItem_t *table_ptr;
	int item_cnt;
} ConsoleMenuTableGroup_t;

typedef struct
{
	const char * long_name;
	uint8_t mask;
	const char * abbr_name;
}	
sPrintFlagItem;

typedef struct
{
	const char * name;
	eTraceFlagAction default_action;
	uint8_t combo_mask;
}	
sPrintFlagActionItem;

typedef struct
{
	struct
	{
		char buff[CONSOLE_RX_BUFF + 1]; /* The rx buff for data read from the console. */
		int cnt;						/* The index into the rx buff. */
	} rx;

	struct
	{
		ConsoleMenuTableGroup_t group[task_console_MAX_MENU_ITEMS_MAX];
		int cnt;
	} menu_grp_list;

	uint8_t tracemask;

	struct
	{
		char *item[task_console_MAX_ARGS];
		int cnt;
		// char *next_arg;
		int pop_index;
	} args;

	TaskInfo_t task;

} DeviceConsole_t;

/*******************************************************************************
local function prototypes
 *******************************************************************************/
/*! Reads the data on the serial port and parses the line when a carriage return 
 * is encountered.
 */
bool _task_console_service(void);

/*! Parses the line received on the Debug port as "<cmd> <arg 1> <arg 2> ... <arg n>". 
 * Arguments seperated by whitespace are parsed and placed on the argument stack.
 * The command is matched to the list of commands in the menu list, and if found, 
 * the specific function is called with the arguments as parameters.
*/
void _task_console_parse_rx_line(void);

/*! Parses the arguments in the string and places them on the argument stack
 * @param[in] arg_str The string containing the arguments to parse
 */
void _task_console_parse_args(char * arg_str);

/*! Prints all the arguments on the stack, with relative index to the next item 
 * to pop
*/
void _task_console_arg_stack(void);

/*! Finds a pointer to the specific command group (if it has been added to the list)
 * @param[in] _command The command string to search for
 * @return A pointer to the group structure if found, NULL otherwise
*/
ConsoleMenuTableGroup_t * _task_console_find_menu_group(char *_command);


/*! Counts the number of occurrences of a command in the menu list (all groups)
 * @param[in] _command The command string to search for
 * @return The number of commands found in the menu list (all groups)
*/
uint32_t _task_console_count_menu_commands(char *_command);

/*! Searches for a specific occurrences of a command in the menu list (all groups)
 * @param[in] _command The command string to search for
 * @param[in] _index The index of the nth item found. If less the N items are found
 * @param[out] _menuItem A pointer to the menu command type structure pointer, if found. 
 *              Will only be set if a valid pointer is passed
 * @param[out] _group_base A pointer to the group_t structure pointer, if found.
 *              Will only be set if a valid pointer is passed
 * @return ESP_OK if the command was found, 
 *         ESP_ERR_NOT_FOUND if less the _index items were found
 *         ESP_ERR_INVALID_ARG if _index < 0 is NULL
*/
esp_err_t _task_console_find_menu_command(char *_command, uint32_t _index, ConsoleMenuItem_t ** _menuItem, ConsoleMenuTableGroup_t **_group_base);

/*! Finds a pointer to the specific command (if it has been added to the list)
 * @param[in] _command The command string to search for
 * @param[out] _menuItem A pointer to the command structure if found
 * @param[out] _tbl_index If supplied, returns index of the group in which the 
 *              command was found (undefined if _command was not found)
 * @return A pointer to the command structure if found, NULL otherwise
*/

/*! Calls the help function on a specific command (or group)
 * @param[in] cmd The command string to search for
 */
void _task_console_show_help_on_command(char * cmd);

/*! Prints the menu of commands available on the console or a specific group of 
 * commands, or the description of a single menu command 
 */
void _menu_handler_help(void);

/*! CONSOLE MENU ITEM - Handles the toggling of trace flags to enable/disable 
 * print traces
 */
void _menu_handler_trace(void);

/*! CONSOLE MENU ITEM - Prints the help string for either the command in question, 
 * or for the entire list of commands under the group
 */
void _menu_handler_tasks(void);

/*! CONSOLE MENU HANDLER - Displays the application version
 */
void _menu_handler_version(void);

/*! Prints an entire line (or not). A leading '#' is replaced with the PRINTF_TAG, 
 * and the print is concluded with an newline character.
 * The passed traceflags is compared with the system set trace print flag(s) to
 * determine if the print can happen or not.
 * @param[in] traceflags The trace flags to compare with the system trace flags
 * @param[in] tag The tag to print with the message (should be defined at the 
 *              top of c-files and undefined at the end)
 * @param[in] fmt The format string to print
 * @param[in] ... The variable arguments to print
 */
void task_console_print(uint8_t traceflags, const char * tag, const char *fmt, ...);

/*! Does the same as task_console_print, but adds a "\n" at the end of the print 
 *   print string
 * @param[in] traceflags The trace flags to compare with the system trace flags
 * @param[in] tag The tag to print with the message (should be defined at the 
 *              top of c-files and undefined at the end)
 * @param[in] fmt The format string to print
 * @param[in] ... The variable arguments to print
 */
void task_console_printline(uint8_t traceflags, const char * tag, const char *fmt, ...);

/*! Prints the default action passed to the function
 * @param[in] def_act The default action to print
 */ 
void _menu_handler_trace_dbgPrint_i_action(eTraceFlagAction def_act);

/*******************************************************************************
local variables
 *******************************************************************************/
sPrintFlagItem print_trace_flag_name[]= 
{
		{"Application", trAPP,      "app"},
		{"Console", 	trCONSOLE,  "con"},
        {"RGB",        trRGB,       "rgb"},
		/* Not Implemented yet {"", 				PRINT_TR_TBD},*/
};

sPrintFlagActionItem print_trace_combos[]= 
{
		{"all", 	eTraceON,       trALL,    },
		{"app", 	eTraceTOGGLE,   trAPP,    },
		{"con", 	eTraceTOGGLE,   trCONSOLE,},
		{"none", 	eTraceOFF,      trALL,    },
        {"rgb",     eTraceTOGGLE,   trRGB,    },
};

ConsoleMenuItem_t _task_console_menu_items[] =
{
										    // 01234567890123456789012345678901234567890123456789012345678901234567890123456789
		{"version", _menu_handler_version,   "Displays application version"},
		{"help",  	_menu_handler_help,      "Displays information on the available Console commands"},
		{"trace",   _menu_handler_trace,     "Displays the status, or Enables/Disables print traces"},
		{"rtos", 	_menu_handler_tasks,     "Displays RTOS information for each or a specific task"},
};

static DeviceConsole_t _task_console = {
	.task.handle = NULL,
	.task.stack_depth = CONSOLE_STACK_SIZE,
	.task.stack_unused = 0,
};

/*******************************************************************************
Local (private) Functions
*******************************************************************************/

/*******************************************************************************

Parses the line received on the Debug port.

 *******************************************************************************/
void _task_console_task(void * pvParameters)
{
	while (1)
  	{
        TickType_t xLastWakeTime = xTaskGetTickCount();

        //Wait at least 100ms before checking again
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100)); 

    	_task_console_service();
	    
		/* Inspect our own high water mark on entering the task. */
    	_task_console.task.stack_unused = uxTaskGetStackHighWaterMark2( NULL );
	}

}

/*******************************************************************************

Parses the line received on the Debug port.

 *******************************************************************************/
void _task_console_parse_rx_line(void)
{
//	dbgPrint(PRINT_TR_CONSOLE, "Parseline() called for \"%s\"", _task_console.rx.buff);

    ConsoleMenuTableGroup_t *_group = NULL;
    char *_command = NULL;
    char * next_param = str_trim_l(_task_console.rx.buff);
    //Assign the 1st parameter to the command string, if it is NULL, then we have no command or arguments

    _command = next_param;

	if ((_command == NULL) || (strlen(_command) == 0))
	{
		dbgPrint(trALWAYS, "What?!");
		return;
	}

    _task_console_parse_args(next_param);

    //Is the first argument a command or a group?
    _group = _task_console_find_menu_group(_command);
    if (_group != NULL)
    {
        //dbgPrint(trALWAYS, "\"%s\" identified as a group (%s)", _command, _group->description);

        //A group, so we expect the next one to be a command (or help/?), 
        if (task_console_arg_cnt() == 0)
        {
            //No command, so we will show the help for the group
            _task_console_show_help_on_command(_command);
            return;
        }
        //else ... oooh, we have more... the next one should be a command
        _command = task_console_arg_pop();
    }

    //We suspect we have a command now, but if there are multiple commands like this in different groups, we cannot decide
    uint32_t _cmd_cnt = _task_console_count_menu_commands(_command);
    // dbgPrint(trCONSOLE, "#Command = \"%s\" (%d)", _command, _cmd_cnt);
    if ((_cmd_cnt > 1) && (_group == NULL))
    {
        //We have more than one command with this name, we need to find out which one the user wants
        dbgPrint(trALWAYS, "Disambiguation needed for \"%s\":", _command);
        _task_console_show_help_on_command(_command);
        return;
    }
    //Unique case where the user entered <group> help or <group> ?
    if ((task_console_arg_cnt() == 0) && ((strcasecmp(_command, "help") == 0) || (strcasecmp(_command, "?") == 0)) && (_group != NULL))
    {
        //No arguments, so we will show the help for the command
        _task_console_show_help_on_command((char *)_group->name);
        return;
    }

    // _task_console_arg_stack();

    // Righto, so we are going to run through the list of menu table groups to see if this command is not contained in one of them...
    ConsoleMenuItem_t *menu_item = NULL;
    ConsoleMenuTableGroup_t *group_base = NULL;
    for (int i = 0; i < _cmd_cnt; i++)    
    {
        if (ESP_OK == _task_console_find_menu_command(_command, i, &menu_item, &group_base))
        {
            if ((_cmd_cnt > 1) && (group_base != _group))
                continue; // with the for-loop (search for the next command in a different group)
            //else, the group is really not relevant as this command is unique

            dbgPrint(trALWAYS, "");
            if (menu_item->func != NULL)
                menu_item->func();
            dbgPrint(trALWAYS, "");
            return;
        }
    }    
	dbgPrint(trALWAYS, "Unknown Command: \"%s\"", _command);
	dbgPrint(trALWAYS, "");
}

void _task_console_show_help_on_command(char * cmd)
{
    //dbgPrint(trALWAYS, "Help called for group (%s):", cmd);
    //Stick this in the argument list and call help!
    _task_console.args.item[_task_console.args.cnt++] = cmd;
    _task_console.args.item[_task_console.args.cnt] = NULL;
    _menu_handler_help();
    dbgPrint(trALWAYS, "");
}

void _task_console_parse_args(char * arg_str)
{
    _task_console.args.cnt = 0;
	_task_console.args.pop_index = 0;

    //Keep at it while we can fit more arguments in the list, and there is more of the string to parse
    do
    {
        /* next_param is looking at the command on 1st entry */
        arg_str = str_next_word(arg_str);

        _task_console.args.item[_task_console.args.cnt] = NULL;

        if (NULL == arg_str)
            break; //from the do-while loop
        _task_console.args.item[_task_console.args.cnt++] = arg_str;
    }
    while (_task_console.args.cnt < task_console_MAX_ARGS);
}

void _task_console_arg_stack(void)
{
    for (int i = 0; i < task_console_MAX_ARGS; i++)
    {
        if (_task_console.args.item[i])
    		dbgPrint(trCONSOLE, "#%d: \"%s\" (%d)", i, _task_console.args.item[i], i - _task_console.args.pop_index);
        else 
            break;
    }
}

char * task_console_arg_pop(void)
{
    char * arg = task_console_arg_peek(0);

    if (arg)
        _task_console.args.pop_index++;
    // {
    //     dbgPrint(trCONSOLE, "Popped \"%s\" (%d)", arg, _task_console.args.pop_index);
    //     _task_console.args.pop_index++;
    // }

    return arg;
}

char * task_console_arg_peek(int offset)
{
    int real_offset = offset + _task_console.args.pop_index;

    //Make sure we are checking within the bounds of the argument stack
    if ((_task_console.args.cnt == 0) || (real_offset < 0) || (real_offset >= _task_console.args.cnt))
        return NULL;

    return _task_console.args.item[real_offset]; 
}

int task_console_arg_cnt(void)
{
    //Guard against the pop_index being greater than the count of arguments
    return MAX(_task_console.args.cnt - _task_console.args.pop_index, 0);
}

uint32_t _task_console_count_menu_commands(char *_command)
{
    uint32_t _count = 0;

	// Righto, so we are going to run through the list of menu table groups to find the 
	// 1st instance of the command that was passed to us
    //Iterate through each group 
	for (int tbl_idx = 0; tbl_idx < _task_console.menu_grp_list.cnt; tbl_idx++)
	{
        //dbgPrint(trALWAYS, "Checking group %s (\"%s\")", _task_console.menu_grp_list.group[tbl_idx].description, _task_console.menu_grp_list.group[tbl_idx].name);
        //Iterate through each item in the group 
		for (int i = 0; i < _task_console.menu_grp_list.group[tbl_idx].item_cnt; i++)
		{
            //dbgPrint(trALWAYS, "Checking cmd \"%s %s\" -> \"%s\"", _task_console.menu_grp_list.group[tbl_idx].name, _task_console.menu_grp_list.group[tbl_idx].table_ptr[i].command, _command);
			if (!strcasecmp(_task_console.menu_grp_list.group[tbl_idx].table_ptr[i].command, _command))
                _count++;
		}
	}
    
    return _count;
}

esp_err_t _task_console_find_menu_command(char *_command, uint32_t _index, ConsoleMenuItem_t ** _menuItem, ConsoleMenuTableGroup_t **_group_base)
{
    uint32_t _count = 0;
    ConsoleMenuItem_t * _last_found_menuItem = NULL;
    ConsoleMenuTableGroup_t *_last_found_group = NULL;

	// Righto, so we are going to run through the list of menu table groups to find the 
	// 1st instance of the command that was passed to us
    //Iterate through each group 
	for (int tbl_idx = 0; tbl_idx < _task_console.menu_grp_list.cnt; tbl_idx++)
	{
        //Iterate through each item in the group 
		for (int i = 0; i < _task_console.menu_grp_list.group[tbl_idx].item_cnt; i++)
		{
			if (!strcasecmp(_task_console.menu_grp_list.group[tbl_idx].table_ptr[i].command, _command))
			{
                _last_found_group = &_task_console.menu_grp_list.group[tbl_idx];
                _last_found_menuItem = &_task_console.menu_grp_list.group[tbl_idx].table_ptr[i];
                //Does the caller want us to get out now?
                if (_count == _index)
                {
                    if (_group_base)
                        *_group_base = _last_found_group;
                    if (_menuItem)
                        *_menuItem = _last_found_menuItem;
                    return ESP_OK;
                }
                _count++;
			}
		}
	}

    return ESP_ERR_NOT_FOUND;
}

ConsoleMenuTableGroup_t * _task_console_find_menu_group(char *_group)
{
	// Righto, so we are going to run through the list of menu table groups to 
	// find the 1st instance of the string that was passed to us
	for (int tbl_idx = 0; tbl_idx < _task_console.menu_grp_list.cnt; tbl_idx++)
	{
		if (!strcasecmp(_task_console.menu_grp_list.group[tbl_idx].name, _group))
			return &_task_console.menu_grp_list.group[tbl_idx];
	}
	// Not found!?!?!?
	return NULL;
}

void _menu_handler_help(void)
{
    char * arg;
    bool help_requested = false;
	int found_grp = 0;
	int found_cmd = 0;
	bool print_all = false;

    //_task_console_arg_stack();

	if (task_console_arg_cnt() > 0)
	{
        arg = task_console_arg_peek(found_grp + found_cmd);// task_console_arg_pop();

        do
        {
			if (_task_console_find_menu_group(arg))
				found_grp++;
			else if (_task_console_count_menu_commands(arg) > 0)
				found_cmd++;
            else
            {
    		    //Is any argument "all" 
                if (!strcasecmp(arg, "all"))
                    print_all = true;
                else //Not known?
                {
                    help_requested = true;
                    dbgPrint(trALWAYS, "\t\"%s\" is not a valid group or menu item", arg);

                }
                //Either way, we are done looping through the arguments.    
                break; //from do-while-loop
            }
            arg = task_console_arg_peek(found_grp + found_cmd); //task_console_arg_pop();

        } while ((task_console_arg_cnt() >= 0) && (NULL != arg) && (!help_requested));

		if ((!print_all) && ((found_grp+found_cmd) > 0))
        {
			//Great, we can actually parse this
			for (int i = (found_grp + found_cmd); i > 0; i--)
			{
                arg = task_console_arg_pop();
				ConsoleMenuTableGroup_t * grp = _task_console_find_menu_group(arg);
                ConsoleMenuItem_t * cmd;
                uint32_t _cmd_cnt = _task_console_count_menu_commands(arg);

                if (grp)
				{
					//dbgPrint(trALWAYS, "The %s (%s) commands are:", grp->description, grp->name);
					for (int j = 0; j < grp->item_cnt; j++)
						dbgPrint(trALWAYS, "  %s %-8s - %s", grp->name, grp->table_ptr[j].command, grp->table_ptr[j].description);
				}
                else // (_cmd_cnt > 0)
                {
                    for (int j = 0; j < _cmd_cnt; j++)
                        if (ESP_OK == _task_console_find_menu_command(arg, j, &cmd, &grp))
                            dbgPrint(trALWAYS, "  %s %-8s - %s", grp->name, cmd->command, cmd->description);
                }
			}
			return;
		}
		//else, don't even bother, we are going to print everything in any case
	}

	//Reaching this point means we are printing EVERYTHING
	dbgPrint(trALWAYS, "The list of available commands are:");

	for (int i = 0; i < _task_console.menu_grp_list.cnt; i++)
	{
		ConsoleMenuTableGroup_t * grp = &_task_console.menu_grp_list.group[i];
		//dbgPrint(trALWAYS, " * %s - %s *", grp->name, grp->description);
		dbgPrint(trALWAYS, " \"%s\" (%s)", grp->name, grp->description);
		//dbgPrint(trALWAYS, "    |");
		//dbgPrint(trALWAYS, "");
		for (int j = 0; j < grp->item_cnt; j++)
		    dbgPrint(trALWAYS, "    + %-8s - %s", 
                grp->table_ptr[j].command, 
                grp->table_ptr[j].description);
        // if (i < (_task_console.menu_grp_list.cnt - 1))
        //     dbgPrint(trALWAYS, "   |");
        dbgPrint(trALWAYS, "");
	}

	//printf("\n%sNOTE: Enter Arguments after the command, separated by a space.");
	dbgPrint(trALWAYS, "Command strings longer than %d chars are invalid.", CONSOLE_RX_BUFF);
}

void _menu_handler_trace_dbgPrint_i_action(eTraceFlagAction def_act)
{
    switch (def_act)
    {
        case eTraceON:      dbgPrint_i(trALWAYS, "ON");         break;
        case eTraceOFF:     dbgPrint_i(trALWAYS, "OFF");        break;
        case eTraceTOGGLE:  dbgPrint_i(trALWAYS, "Toggle");     break;
        default:            dbgPrint_i(trALWAYS, "No Action");  break;
    }
}

void _menu_handler_trace(void)
{
	/*We are expecting one fo the following arguments
	<nothing> 				- Display the current Print Trace Status
	<"?"> 					- Displaya list of all the valid terms
	<valid term> 			- Toggle those traces
	<valid term> <ON/OFF>	- Toggle those traces
	*/
    bool help_requested = false;
    uint8_t change_mask = 0;
    int un_selected_cnt = 0;
    int valid_traces_cnt = 0;
    uint8_t change_mask_on = 0;
    uint8_t change_mask_off = 0;
    uint8_t change_mask_toggle = 0;
	uint8_t tmp_tracemask = _task_console.tracemask;
    //uint8_t *change_mask = &change_mask_interim;


	if (task_console_arg_cnt() > 0)
	{
        char *arg = task_console_arg_pop();
        do
        {
            uint8_t *change_mask_ptr = NULL;
            if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
            {    
                help_requested = true;
                break; //from do-while-loop
            }
            else if ((!strcasecmp("ON", arg)) || (!strcasecmp("Y", arg)) || (!strcasecmp("1", arg)))
            {
                change_mask_ptr = &change_mask_on;    
            }
            else if ((!strcasecmp("OFF", arg)) || (!strcasecmp("N", arg)) || (!strcasecmp("0", arg)))
            {    
                change_mask_ptr = &change_mask_off;
            }
            else if ((!strcasecmp("Toggle", arg)) || (!strcasecmp("T", arg)) || (!strcasecmp("X", arg)))
            {    
                change_mask_ptr = &change_mask_toggle;
            }
            else  //we'll be looking for a valid trace name or combination trace name
            {
                bool found = false;
                for (int i = 0; i < (sizeof(print_trace_combos)/sizeof(sPrintFlagActionItem)); i++)
                {
                    if(!strcasecmp(print_trace_combos[i].name, arg))
                    {
                        //found one
                        change_mask |= print_trace_combos[i].combo_mask;                        
                        found = true;
                        break; //from for-loop
                    }
                }
                //Increment the unselected cound regardless of whether we found a valid trace name or not (thus ignoring it)
                un_selected_cnt++;
                if (!found)
                    dbgPrint(trALWAYS, "\t\"%s\" is not a valid trace name or action. Argument ignored!!!", arg);
                else
                    valid_traces_cnt++;
                }
            //Did we get a ON/OFF/Toggle command... and do we have flags to perform the action on?
            if ((change_mask) && (change_mask_ptr))
            {
                *change_mask_ptr |= change_mask;
                change_mask = 0;
                un_selected_cnt = 0;
                valid_traces_cnt = 0;
            }
            //Get the next argument
            arg = task_console_arg_pop();

        } while ((task_console_arg_cnt() >= 0) && (NULL != arg));// && (!help_requested));

        if (valid_traces_cnt /*un_selected_cnt*/ > 0)
        {
            //_task_console_arg_stack();
            //We have a valid trace mask, but no state was selected, so we'll toggle it
            dbgPrint(trALWAYS, " No action selected for %d valid tag(s), performing default action(s):", valid_traces_cnt /*un_selected_cnt*/);
            for (int offset = (0-un_selected_cnt); offset < 0; offset++)
            {
                arg = task_console_arg_peek(offset);
                for (int i = 0; (i < (sizeof(print_trace_combos)/sizeof(sPrintFlagActionItem)) && (valid_traces_cnt > 0)); i++)
                {
                    if(!strcasecmp(print_trace_combos[i].name, arg))
                    {
                        valid_traces_cnt--;
                        dbgPrint_i(trALWAYS, "   %10s : ", (const char *)print_trace_combos[i].name);
                        _menu_handler_trace_dbgPrint_i_action(print_trace_combos[i].default_action);
                        dbgPrint_i(trALWAYS, "\n");
                        switch (print_trace_combos[i].default_action)
                        {
                            case eTraceON:      change_mask_on |= print_trace_combos[i].combo_mask;     break;
                            case eTraceOFF:     change_mask_off &=  (~print_trace_combos[i].combo_mask);break;
                            case eTraceTOGGLE:  change_mask_toggle ^= print_trace_combos[i].combo_mask; break;
                            default:                                                                    break;
                        }
                    }
                }
                //Reaching this point with found == false means that the argument was not found in the list of valid trace names... that is a big problem!!!
                // if (!found)
                //     dbgPrint(trALWAYS, "\t\t %10s : Unknown (Ignored)", arg);
            }
        }

        //Righto, we are done with the arguments, let's do the toggling
	    tmp_tracemask = _task_console.tracemask;
        if (change_mask_on)
            _task_console.tracemask |= change_mask_on;
        if (change_mask_off)
            _task_console.tracemask &= (~change_mask_off);
        if (change_mask_toggle)
            _task_console.tracemask ^= change_mask_toggle;

	}

    if (help_requested)
    {
        dbgPrint(trALWAYS, "Usage: \"trace <tag(s)> <action (optional)> \"");
        dbgPrint(trALWAYS, "  <action> can be \"ON\", \"OFF\" or \"Toggle\"");
        dbgPrint(trALWAYS, "  <tag(s)> can be 1 or more of the following (Default Action):");
        // dbgPrint_i(trALWAYS, "\t\t[");
        // for (int i = 0; i < (sizeof(print_trace_combos)/sizeof(sPrintFlagActionItem)); i++)
        //     dbgPrint_i(trALWAYS, "%s%s", (const char *)print_trace_combos[i].name, (const char *)((i < (sizeof(print_trace_combos)/sizeof(sPrintFlagActionItem)-1))? ", ":"]\n"));

        //dbgPrint(trCONSOLE, " ==== Trace Menu ==== ");
        for (int i = 0; i < (sizeof(print_trace_combos)/sizeof(sPrintFlagActionItem)); i++)
        {
            uint8_t combo_mask = print_trace_combos[i].combo_mask;
            dbgPrint_i(trALWAYS, "    %6s - ", (const char *)print_trace_combos[i].name);
            //Run through the active bits and find their relative names in print_trace_flag_name
            bool multi_flag = false;
            for (int j = 0; j < (sizeof(print_trace_flag_name)/sizeof(sPrintFlagItem)); j++)
            {
                if(print_trace_flag_name[j].mask & combo_mask)
                {
                    dbgPrint_i(trALWAYS, "%s%s", ((multi_flag)? "|":""), print_trace_flag_name[j].abbr_name);
                    multi_flag = true;
                }
            }
            dbgPrint_i(trALWAYS, " %12s", "(");
            _menu_handler_trace_dbgPrint_i_action(print_trace_combos[i].default_action);
            dbgPrint_i(trALWAYS, ")\n");
        }
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        dbgPrint(trALWAYS, "");
        dbgPrint(trALWAYS, "  Combination of flags switching can be performed in a single line");
        dbgPrint(trALWAYS, "   by specifying tag(s) then action(s)");
        dbgPrint(trALWAYS, "   e.g. \"trace tag_1 tag_2 ON tag_3 tag_4 OFF...\" etc");
        dbgPrint(trALWAYS, "  If an action is not specified, the default action for the relevant");
        dbgPrint(trALWAYS, "   flag is performed");
        dbgPrint(trALWAYS, "");
    }

	dbgPrint(trALWAYS, "The current state of Print Trace Flags are:");

	for (int i = 0; i < (sizeof(print_trace_flag_name)/sizeof(sPrintFlagItem)); i++)
	{
		uint8_t trace_mask = print_trace_flag_name[i].mask;
		dbgPrint(trALWAYS, " %15s - %s%s",
				print_trace_flag_name[i].long_name,
				(print_trace_flag_name[i].mask & _task_console.tracemask)? "ON  " : "OFF ",
				/* Add an asterisk if this trace flag has been changed in this operation */
				((tmp_tracemask & trace_mask) != (_task_console.tracemask & trace_mask))? "*" : "");
	}
}

void _menu_handler_tasks(void)
{
	/*We are expecting one fo the following arguments
	<nothing> 				- Display the stack usage of all the running tasks
	<"?"> 					- Displaya list of all the running tasks
	<task name> 			- Display the stack usage of the specified task
	*/
#if ( configUSE_TRACE_FACILITY == 1 )
	int task_index = -1;
	int print_start = 0;
	int print_end = eTaskIndexMax;

	char task_list_buff[eTaskIndexMax*40];

	if (arg_cnt > 0)
	{
		if (is_number_str(args[0]))
		{
			int index = atoi(args[0]);
			if ((index >= eTaskIndexMax) || (index < 0))
			{
				dbgPrint(trALWAYS, "Please specify a number from 0 and %d, or alternatively the task name.");
				task_index = -1;
			}
		}
	}

	if (task_index >= 0)
	{
		//We are only printing one task's information
		print_start = task_index;
		print_end = task_index+1;
	}
	// else, keep the values unchanged and print the entire list

	dbgPrint(trALWAYS, "Task Information");
	dbgPrint(trALWAYS, "+---+------------+------------+-------+------------+--------------------+");
	dbgPrint(trALWAYS, "+ # | Name       | Status     | Pr'ty | Runtime    | Stack Usage        |");
	dbgPrint(trALWAYS, "+---+------------+------------+-------+------------+--------------------+");
	for (int i = print_start; i < print_end; i++)
	{
		float s_depth_f;
	    configSTACK_DEPTH_TYPE s_depth_u16;
    	configSTACK_DEPTH_TYPE s_used_u16;

		if (!sys_tasks[i])
			continue;
		vTaskGetInfo(sys_tasks[i]->handle, &sys_tasks[i]->details, pdTRUE, eInvalid);

	    s_depth_u16 = (configSTACK_DEPTH_TYPE)sys_tasks[i]->stack_depth;
    	s_used_u16 = (configSTACK_DEPTH_TYPE)(sys_tasks[i]->stack_depth - sys_tasks[i]->stack_unused);
		s_depth_f = (float)(s_used_u16 * 100.0f)/(s_depth_u16 * 1.0f);

		dbgPrint(trALWAYS, "| %d | %-10s | %-10s | %02d/%02d | %10u | %04u/%04u (%2.2f%%) |",
			i, 
			sys_tasks[i]->details.pcTaskName,
			taskStateName[sys_tasks[i]->details.eCurrentState],
			sys_tasks[i]->details.uxCurrentPriority,
			sys_tasks[i]->details.uxBasePriority,
			sys_tasks[i]->details.ulRunTimeCounter,
			s_used_u16,
			s_depth_u16,
			s_depth_f);
	}
	dbgPrint(trALWAYS, "+---+------------+------------+-------+------------+--------------------+");
	dbgPrint(trALWAYS, "");

	dbgPrint(trALWAYS, "System generated Task List:");
	vTaskList(task_list_buff);
	dbgPrint(trALWAYS, "%s", task_list_buff);
	dbgPrint(trALWAYS, "Done.");
#else
	dbgPrint(trALWAYS, "Not Supported in this build.");
#endif

}

void _menu_handler_version(void)
{
	dbgPrint(trALWAYS, "");
	dbgPrint(trALWAYS, "=====================================================");
	dbgPrint(trALWAYS, "ButtonChaser - Slave Controller");
	dbgPrint(trALWAYS, "[c] 2025 ZeroBadCafe Development (Pty) Ltd");
	dbgPrint(trALWAYS, "Version   %d.%2ds.", PROJECT_VERSION / 0x10, PROJECT_VERSION % 0x10);
	dbgPrint(trALWAYS, "BuildInfo %s.", BUILD_TIME_DATA);
	dbgPrint(trALWAYS, "ESP32-C3 (Clock %lu MHz)", configCPU_CLOCK_HZ / 1000000L);
	dbgPrint(trALWAYS, "=====================================================");
}

bool _task_console_service(void)
{
	int rxData;
	bool retVal = false;

	// read the incoming char:
	while ((rxData = getchar()) != EOF)
	{
		// printf("Received: %c (0x%02X)  - Inptr @ %d\n", rxData, rxData, sttask_console.InPtr);
		retVal = true;

		// Lines are terminated on Carriage returns and/or newlines.
		switch (rxData)
		{
		// ****** Carriage Return ******
		case '\r':
			// Skip Carriage Return characters
			continue;

		// ****** Newline ******
		case '\n':
			printf("\n");
			// Serial.write('\n');
			//  Now parse the line if it is Valid
			if (_task_console.rx.cnt > 0)
				_task_console_parse_rx_line();
			// Start a new line now....
			_task_console.rx.cnt = 0; // Reset index pointer
			break;

			// ****** Backspace ******
		case 0x08:
			// Move one char back in buffer
			if (_task_console.rx.cnt)
			{
				_task_console.rx.cnt--;
				printf(BACKSPACE_ECHO);
			}
			break;

			// ****** Escape ******
		case '\t':
			// RVN TODO - Auto-complete?
			/// Naaah.... too much hassle for this scope.
			printf("\t");
			break;

			// ****** Escape ******
		case 0x1B:
			// Clear the line...
			while (_task_console.rx.cnt)
			{
				printf(BACKSPACE_ECHO);
				_task_console.rx.cnt--;
			}
			_task_console.rx.cnt = 0;
			break;

			// ****** All other chars ******
		default:
			// Must we echo the data on the console?
			printf("%c", rxData);

			// Do we still have space in the rx buffer?
			if (_task_console.rx.cnt < CONSOLE_RX_BUFF) // Wrap index?
			{
				// Add char and Null Terminate string
				_task_console.rx.buff[_task_console.rx.cnt++] = rxData;
			}
			else // The index pointer now wraps, our data is invalid.
			{
				_task_console.rx.cnt = 0; // Reset the index pointer
			}
			break;
		}

		// Always NULL terminate whatever is in the buffer.
		_task_console.rx.buff[_task_console.rx.cnt] = 0; // Null Terminate
	}

	return retVal;
}

/*******************************************************************************
Global (public) Functions
*******************************************************************************/

TaskInfo_t * task_console_init(void)
{
	//Let's not re-initialise this task by accident
	if (_task_console.task.handle)
		return &_task_console.task;

	_task_console.menu_grp_list.cnt = 0;

	// The Console will run on the Debug port
	// Serial.begin(115200, SERIAL_8N1);

	_task_console.tracemask = trCONSOLE|trAPP|trRGB;


	// dbgPrint(trALWAYS,	"ALWAYS  - Print a number %d", 1);
	// dbgPrint(trBAT, 		"BAT     - Print a number %d", 2);
	// dbgPrint(trCONSOLE,	"CONSOLE - Print a number %d", 3);
	// dbgPrint(trGPS, 		"GPS     - Print a number %d", 4);
	// dbgPrint(trAPP, 		"APP     - Print a number %d", 5);
	// dbgPrint(trNONE,   	"NONE    - Print a number %d", 6);

	// Create the task, storing the handle.  Note that the passed parameter ucParameterToPass
	// must exist for the lifetime of the task, so in this case is declared static.  If it was just an
	// an automatic stack variable it might no longer exist, or at least have been corrupted, by the time
	// the new task attempts to access it.
	if (xTaskCreate( _task_console_task, PRINTF_TAG, _task_console.task.stack_depth, _task_console.task.parameter_to_pass, tskIDLE_PRIORITY, &_task_console.task.handle ) != pdPASS)
	{
		dbgPrint(trALWAYS, "#Unable to start Task (%d)!", eTaskIndexConsole);
		return NULL;
	}

    _menu_handler_version();

	task_console_add_menu("con", _task_console_menu_items, ARRAY_SIZE(_task_console_menu_items), "Console Interface");

	dbgPrint(trALWAYS, "#Init OK (Traces: 0x%02X)", _task_console.tracemask);

	configASSERT(_task_console.task.handle);

	return &_task_console.task;
}

void task_console_deinit(void)
{
	// Use the handle to delete the task.
	if(_task_console.task.handle != NULL )
		vTaskDelete(_task_console.task.handle);

}

int task_console_add_menu(const char *_group_name, ConsoleMenuItem_t *_tbl, size_t _cnt, const char *_desc)
{
	// Do not bother if we have not been initialized
	if (!_task_console.task.handle)
	{
		dbgPrint(trALWAYS, "#Not Initialized yet (%s)", _group_name);
		return 0;
	}

	// First we need to decide if we have space?
	if (_task_console.menu_grp_list.cnt >= task_console_MAX_MENU_ITEMS_MAX)
	{
		dbgPrint(trCONSOLE, "#menu list: %s (%d)\n",
			   _group_name,
			   "Out of Space",
			   _task_console.menu_grp_list.cnt);
		return 0;
	}

	//Don't add an empty table or a table with dulicate entries
    if (_cnt == 0)
    {
        dbgPrint(trCONSOLE, "#Empty table for \"%s\" (%d)", _group_name, _cnt);
        return 0;
    }

	for (int i = 0; i < _cnt-1; i++)
	{
        //Check every entry with every other entry in the table
        for (int j = i+1; j < _cnt; j++)
        {
            if (!strcasecmp(_tbl[i].command, _tbl[j].command))
            {
                dbgPrint(trCONSOLE, "#Duplicate command \"%s\" in \"%s\" (%d & %d)", _tbl[i].command, _group_name, i, j);
                return 0;
            }
        }
	}


	int index = 0;

	// dbgPrint(trALWAYS, "#Adding: %s", _group_name);

    // Check if we already have this group in the list
	for (index = 0; index < _task_console.menu_grp_list.cnt; index++)
	{
		// We don't want to add duplicate tables or groups
		if (!strcasecmp(_task_console.menu_grp_list.group[index].name, _group_name))
		{
			dbgPrint(trCONSOLE, "#Group \"%s\" already added @ %d)",
				   _group_name,
				   _cnt);
			return 0;
		}
		
        if (_task_console.menu_grp_list.group[index].table_ptr == _tbl)
		{
			dbgPrint(trCONSOLE, "#Table for group \"%s\" already added @ %d under \"%s\"",
				   _group_name,
				   _cnt,
                   _task_console.menu_grp_list.group[index].name);
			return 0;
		}

		// We also want to add it in alphabetical order....
		// We walk through the existing list until we find a group which is alphabetically "after" our group
		if (strcasecmp(_task_console.menu_grp_list.group[index].name, _group_name) < 0)
			continue; // with the for loop
		// else, this item group is alphabetically larger.

        //dbgPrint(trCONSOLE, "#Adding \"%s\" (%d) @ index %d", _group_name, _cnt, index);
		// Add this items right here
		break; // from the for-loop
	}

	//Depending on the current value of index, we may or may not need to make some space.
	if (index < _task_console.menu_grp_list.cnt)
	{
		//dbgPrint(trCONSOLE, "#Opening up space at position %d/%d", index+1, _task_console.menu_grp_list.cnt);
		// Open up a slot by moving everything else on with one space.
        memmove(&_task_console.menu_grp_list.group[index + 1], &_task_console.menu_grp_list.group[index], sizeof(ConsoleMenuTableGroup_t) * (_task_console.menu_grp_list.cnt - index));
	}

    // dbgPrint(trCONSOLE, "#No need to move anything (%d-%d)", index, _task_console.menu_grp_list.cnt);

	// Reaching this point means we can add our next table at the end.
	_task_console.menu_grp_list.group[index].name = _group_name;
	_task_console.menu_grp_list.group[index].description = _desc;
	_task_console.menu_grp_list.group[index].table_ptr = _tbl;
	_task_console.menu_grp_list.group[index].item_cnt = _cnt;
	_task_console.menu_grp_list.cnt++;
	//dbgPrint(trCONSOLE, "#Added \"%s\" @ position %d/%d", _task_console.menu_grp_list.group[index].name, index+1, _task_console.menu_grp_list.cnt);

    //RVN - I guess we can check if possible duplicate commands exist under different groups
    for (int i = 0; i < _cnt; i++)
    {
        //int tbl_index = 0;
        ConsoleMenuTableGroup_t *_group_base;
        if (ESP_OK == _task_console_find_menu_command(_tbl[i].command, 0, NULL, &_group_base))
            if (strcasecmp(_group_base->name, _group_name) != 0)
                dbgPrint(trCONSOLE, "#\"%s\" is duplicated in \"%s\" (%s) and \"%s\" (%s)", 
                    _tbl[i].command, 
                    _group_base->name, _group_base->description,
                    _task_console.menu_grp_list.group[index].name, _task_console.menu_grp_list.group[index].description);
    }
	return _cnt;
}

void task_console_print(uint8_t traceflags, const char * tag, const char *fmt, ...)
{
	va_list ap;

	if (((trALWAYS | _task_console.tracemask) & traceflags) == trNONE)
		return;

	//A line starts with the tag if the format string starts with "#"
	if (fmt[0] == '#')
	{
		printf("[%s]", tag);
		fmt++; // Now skip the '#'
	}

	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
}

void task_console_printline(uint8_t traceflags, const char * tag, const char *fmt, ...)
{
    task_console_print(traceflags, tag, fmt);
	//Ends with a newline.
	printf("\n");
}


#undef PRINTF_TAG

/*************************** END OF FILE *************************************/
