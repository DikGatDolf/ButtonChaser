/*******************************************************************************

Module:     dev_console.c
Purpose:    This file contains the console interface
Author:     Rudolph van Niekerk

The console device will parse all received input strings (terminated with a
carriage return & newline) on the debug port for functional data. The first
part of the input string is matched in the local structured array to check for a
matching command. All additional data will be treated as parameters.

The Console maintains a list of menu item GROUPS (Tables), not the items 
 themselves, which are added with the console_add_menu() function. 
 The menu items are statically declared in the caller task/file/module.
 A single menu item of  type console_menu_item_t, is a structure containing:
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
#include <Arduino.h>
#include <avr/pgmspace.h>
//#include <SPI.h>
#include "defines.h"

//#include "devMotorControl.h"
//#include "halAMT203.h"
//#include "halTLC5615.h"
//#include "paramCtrl.h"
#include "std_utils.h"
#include "str_helper.h"
#include "hal_timers.h"
#include "hal_pwm.h"

#define __NOT_EXTERN__
#include "dev_console.h"
#undef __NOT_EXTERN__

#ifdef CONSOLE_ENABLED

/*******************************************************************************
local defines and constants
 *******************************************************************************/
#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("Console") /* This must be undefined at the end of the file*/

/* Max string size of string we can RX on console */
#define CONSOLE_RX_BUFF     80 

#define CONSOLE_STACK_SIZE 4096

/* The max amount of console arguments we can parse. If more than these are passed, the 
last argument placed on the stack will contain the remainder of the argument string 
as it was received on the console */
#define dev_console_MAX_ARGS 15 

#define dev_console_MAX_MENU_ITEMS_MAX 5 /* The maximum number of menu groups/tables */

#define CONSOLE_READ_INTERVAL_MS      100     /* Task cycles at a 10Hz rate*/


const char BACKSPACE_ECHO[] = {0x08, 0x20, 0x08, 0x00};

#define PrintBackSpace()   {do{serialputc(0x08, NULL); serialputc(0x20, NULL); serialputc(0x08, NULL);}while (0);}
//Turns out that calling serialputc() in a loop is not a good idea.... uses 4 bytes more RAM and 6 bytes more flash
//#define PrintBackSpace()   {int _x = 0; do{serialputc(BACKSPACE_ECHO[_x++], NULL); }while (BACKSPACE_ECHO[_x]);}
    

const char BUILD_TIME_DATA[] = {__TIME__ " " __DATE__}; /* Used in our startup Banner*/

//const char ARGUMENT_DELIMITERS[] = {' ', '\t', 0x00};

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
	const console_menu_item_t *table_ptr;
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
	uint8_t tracemask;
	struct
	{
		int cnt;
		ConsoleMenuTableGroup_t group[dev_console_MAX_MENU_ITEMS_MAX];
	} menu_grp_list;
	struct
	{
		char buff[CONSOLE_RX_BUFF + 1]; /* The rx buff for data read from the console. */
		int cnt;						/* The index into the rx buff. */
	} rx;


	struct
	{
		char *item[dev_console_MAX_ARGS];
		int cnt;
		// char *next_arg;
		int pop_index;
	} args;

} DeviceConsole_t;

/*******************************************************************************
local function prototypes
 *******************************************************************************/
extern "C" {
	int serialputc(char c, FILE *fp)
  	{
		if(c == '\n')
			Serial.write('\r');
		return Serial.write(c);
	}
}

/*! Deinitialises the Console
 */
void _console_task_deinit(void);

/*! The main function for the Console task
 */
void _console_main_func(void * pvParameters);

/*! Parses the line received on the Debug port as "<cmd> <arg 1> <arg 2> ... <arg n>". 
 * Arguments seperated by whitespace are parsed and placed on the argument stack.
 * The command is matched to the list of commands in the menu list, and if found, 
 * the specific function is called with the arguments as parameters.
*/
void _parse_rx_line(void);

/*! Parses the arguments in the string and places them on the argument stack
 * @param[in] arg_str The string containing the arguments to parse
 */
void _parse_args(char * arg_str);

/*! Prints all the arguments on the stack, with relative index to the next item 
 * to pop
*/
void _print_arg_stack(void);

/*! Finds a pointer to the specific command group (if it has been added to the list)
 * @param[in] _command The command string to search for
 * @return A pointer to the group structure if found, NULL otherwise
*/
ConsoleMenuTableGroup_t * _find_menu_group(char *_command);


/*! Counts the number of occurrences of a command in the menu list (all groups)
 * @param[in] _command The command string to search for
 * @return The number of commands found in the menu list (all groups)
*/
int _count_menu_commands(char *_command);

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
bool _find_menu_command(char *_command, int _index, const console_menu_item_t ** _menuItem, ConsoleMenuTableGroup_t **_group_base);

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
void _show_help_on_command(char * cmd);

/*! Prints the menu of commands available on the console or a specific group of 
 * commands, or the description of a single menu command 
 */
void _console_handler_help(void);

/*! CONSOLE MENU ITEM - Handles the toggling of trace flags to enable/disable 
 * print traces
 */
void _console_handler_trace(void);

/*! CONSOLE MENU HANDLER - Displays the application version
 */
void _console_handler_version(void);

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
void console_print(uint8_t traceflags, const char * tag, const char *fmt, ...);

/*! Does the same as console_print, but adds a "\n" at the end of the print 
 *   print string
 * @param[in] traceflags The trace flags to compare with the system trace flags
 * @param[in] tag The tag to print with the message (should be defined at the 
 *              top of c-files and undefined at the end)
 * @param[in] fmt The format string to print
 * @param[in] ... The variable arguments to print
 */
void console_printline(uint8_t traceflags, const char * tag, const char *fmt, ...);

/*! Prints the default action passed to the function
 * @param[in] def_act The default action to print
 */ 
void _console_handler_trace_dbgPrint_i_action(eTraceFlagAction def_act);

/*******************************************************************************
local variables
 *******************************************************************************/
sPrintFlagItem print_trace_flag_name[]= 
{
		{"Main",        trMAIN,     "main"},
		{"Console", 	trCONSOLE,  "con"},
        {"PWM",         trPWM,      "pwm"},
		/* Not Implemented yet {"", 				PRINT_TR_TBD},*/
};

sPrintFlagActionItem print_trace_combos[]= 
{
		{"all", 	eTraceON,       trALL,    },
		{"app", 	eTraceTOGGLE,   trMAIN,    },
		{"con", 	eTraceTOGGLE,   trCONSOLE,},
		{"none", 	eTraceOFF,      trALL,    },
        {"pwm",     eTraceTOGGLE,   trPWM,    },
};

static console_menu_item_t _console_menu_items[] =
{
										    // 01234567890123456789012345678901234567890123456789012345678901234567890123456789
		{"version", _console_handler_version,   "Displays application version"},
		{"help",  	_console_handler_help,      "Displays information on the available Console commands"},
		{"trace",   _console_handler_trace,     "Displays the status, or Enables/Disables print traces"}
};

DeviceConsole_t _console;
bool _console_init_done = false;
FILE _console_stdiostr;
va_list _console_ap;

/*******************************************************************************
Local (private) Functions
*******************************************************************************/

void _parse_rx_line(void)
{
//	iprintln(PRINT_TR_CONSOLE, "Parseline() called for \"%s\"", _console.rx.buff);

    ConsoleMenuTableGroup_t *_group = NULL;
    //char *_command = NULL;
    char * arg = str_trim_l(_console.rx.buff);
    //Assign the 1st parameter to the command string, if it is NULL, then we have no command or arguments

	if ((arg == NULL) || (*arg == 0))
	{
		iprintln(trALWAYS, "What?!");
		return;
	}

    //This will parse the argumentss pointers into the arg stack and null terminate each of them
    _parse_args(arg);

    //the 1st entry in the arg stack should be the command, let's pop it
    arg = console_arg_pop();

    //Is the first argument a command or a group?
    _group = _find_menu_group(arg);
    if (_group != NULL)
    {
        //iprintln(trALWAYS, "\"%s\" identified as a group (%s)", _command, _group->description);

        //A group, so we expect the next one to be a command (or help/?), 
        if (console_arg_cnt() == 0)
        {
            //No command, so we will show the help for the group
            _show_help_on_command((char *)_group->name);
            return;
        }
        //else ... oooh, we have more... the next one should be a command
        arg = console_arg_pop();
    }

    //We suspect we have a command now, but if there are multiple commands like this in different groups, we cannot decide
    int _cmd_cnt = _count_menu_commands(arg);
    // iprintln(trCONSOLE, "#Command = \"%s\" (%d)", arg, _cmd_cnt);
    if ((_cmd_cnt > 1) && (_group == NULL))
    {
        //We have more than one command with this name, we need to find out which one the user wants
        iprintln(trALWAYS, "Disambiguation needed for \"%s\":", arg);
        _show_help_on_command(arg);
        return;
    }
    //Unique case where the user entered <group> help or <group> ?
    if ((console_arg_cnt() == 0) && ((strcasecmp(arg, "help") == 0) || (strcasecmp(arg, "?") == 0)) && (_group != NULL))
    {
        //No arguments, so we will show the help for the command
        _show_help_on_command((char *)_group->name);
        return;
    }

    //_print_arg_stack();

    // Righto, so we are going to run through the list of menu table groups to see if this command is not contained in one of them...
    const console_menu_item_t *menu_item = NULL;
    ConsoleMenuTableGroup_t *group_base = NULL;
    for (int i = 0; i < _cmd_cnt; i++)    
    {
        if (_find_menu_command(arg, i, &menu_item, &group_base))
        {
            if ((_cmd_cnt > 1) && (group_base != _group))
                continue; // with the for-loop (search for the next command in a different group)
            //else, the group is really not relevant as this command is unique

            iprintln(trALWAYS, "");
            if (menu_item->func != NULL)
                menu_item->func();
            iprintln(trALWAYS, "");
            return;
        }
    }    
	iprintln(trALWAYS, "Unknown Command: \"%s\"", arg);
	iprintln(trALWAYS, "");
}

void _show_help_on_command(char * cmd)
{
    //iprintln(trALWAYS, "Help called for group (%s):", cmd);
    //Stick this in the argument list and call help!
    _console.args.item[_console.args.cnt++] = cmd;
    _console.args.item[_console.args.cnt] = NULL;
    _console_handler_help();
    iprintln(trALWAYS, "");
}

void _parse_args(char * arg_str)
{
    _console.args.cnt = 0;
	_console.args.pop_index = 0;

    //Keep at it while we can fit more arguments in the list, and there is more of the string to parse
    while ((arg_str) && (_console.args.cnt < dev_console_MAX_ARGS))    
    {
        _console.args.item[_console.args.cnt++] = arg_str;
        _console.args.item[_console.args.cnt] = NULL;
        arg_str = str_next_word(arg_str);
    }
    return;
}

void _print_arg_stack(void)
{
    for (int i = 0; i < dev_console_MAX_ARGS; i++)
    {
        if (_console.args.item[i])
    		iprintln(trCONSOLE, "#%d: \"%s\" (%d)", i, _console.args.item[i], i - _console.args.pop_index);
        else 
            break;
    }
}

char * console_arg_pop(void)
{
    char * arg = console_arg_peek(0);

    if (arg)
        _console.args.pop_index++;
    // {
    //     iprintln(trCONSOLE, "Popped \"%s\" (%d)", arg, _console.args.pop_index);
    //     _console.args.pop_index++;
    // }

    return arg;
}

char * console_arg_peek(int offset)
{
    int real_offset = offset + _console.args.pop_index;

    //Make sure we are checking within the bounds of the argument stack
    if ((_console.args.cnt == 0) || (real_offset < 0) || (real_offset >= _console.args.cnt))
        return NULL;

    return _console.args.item[real_offset]; 
}

int console_arg_cnt(void)
{
    //Guard against the pop_index being greater than the count of arguments
    return max(0, _console.args.cnt - _console.args.pop_index);
}

int _count_menu_commands(char *_command)
{
    int _count = 0;

	// Righto, so we are going to run through the list of menu table groups to find the 
	// 1st instance of the command that was passed to us
    //Iterate through each group 
	for (int tbl_idx = 0; tbl_idx < _console.menu_grp_list.cnt; tbl_idx++)
	{
        //iprintln(trALWAYS, "Checking group %s (\"%s\")", _console.menu_grp_list.group[tbl_idx].description, _console.menu_grp_list.group[tbl_idx].name);
        //Iterate through each item in the group 
		for (int i = 0; i < _console.menu_grp_list.group[tbl_idx].item_cnt; i++)
		{
            //iprintln(trALWAYS, "Checking cmd \"%s %s\" -> \"%s\"", _console.menu_grp_list.group[tbl_idx].name, _console.menu_grp_list.group[tbl_idx].table_ptr[i].command, _command);
			if (!strcasecmp(_console.menu_grp_list.group[tbl_idx].table_ptr[i].command, _command))
                _count++;
		}
	}
    
    return _count;
}

bool _find_menu_command(char *_command, int _index, const console_menu_item_t ** _menuItem, ConsoleMenuTableGroup_t **_group_base)
{
    int _count = 0;
    const console_menu_item_t * _last_found_menuItem = NULL;
    ConsoleMenuTableGroup_t *_last_found_group = NULL;

	// Righto, so we are going to run through the list of menu table groups to find the 
	// 1st instance of the command that was passed to us
    //Iterate through each group 
	for (int tbl_idx = 0; tbl_idx < _console.menu_grp_list.cnt; tbl_idx++)
	{
        //Iterate through each item in the group 
		for (int i = 0; i < _console.menu_grp_list.group[tbl_idx].item_cnt; i++)
		{
			if (!strcasecmp(_console.menu_grp_list.group[tbl_idx].table_ptr[i].command, _command))
			{
                _last_found_group = &_console.menu_grp_list.group[tbl_idx];
                _last_found_menuItem = &_console.menu_grp_list.group[tbl_idx].table_ptr[i];
                //Does the caller want us to get out now?
                if (_count == _index)
                {
                    if (_group_base)
                        *_group_base = _last_found_group;
                    if (_menuItem)
                        *_menuItem = _last_found_menuItem;
                    return true;
                }
                _count++;
			}
		}
	}

    return false;
}

ConsoleMenuTableGroup_t * _find_menu_group(char *_group)
{
	// Righto, so we are going to run through the list of menu table groups to 
	// find the 1st instance of the string that was passed to us
	for (int tbl_idx = 0; tbl_idx < _console.menu_grp_list.cnt; tbl_idx++)
	{
		if (!strcasecmp(_console.menu_grp_list.group[tbl_idx].name, _group))
			return &_console.menu_grp_list.group[tbl_idx];
	}
	// Not found!?!?!?
	return NULL;
}

void _console_handler_help(void)
{
    char * arg;
    bool help_requested = false;
	int found_grp = 0;
	int found_cmd = 0;
	bool print_all = false;

    //_print_arg_stack();

	if (console_arg_cnt() > 0)
	{
        arg = console_arg_peek(found_grp + found_cmd);// console_arg_pop();

        do
        {
			if (_find_menu_group(arg))
				found_grp++;
			else if (_count_menu_commands(arg) > 0)
				found_cmd++;
            else
            {
    		    //Is any argument "all" 
                if (!strcasecmp(arg, "all"))
                    print_all = true;
                else //Not known?
                {
                    help_requested = true;
                    iprintln(trALWAYS, "\t\"%s\" is not a valid group or menu item", arg);

                }
                //Either way, we are done looping through the arguments.    
                break; //from do-while-loop
            }
            arg = console_arg_peek(found_grp + found_cmd); //console_arg_pop();

        } while ((NULL != arg) && (!help_requested));

		if ((!print_all) && ((found_grp+found_cmd) > 0))
        {
			//Great, we can actually parse this
			for (int i = (found_grp + found_cmd); i > 0; i--)
			{
                arg = console_arg_pop();
				ConsoleMenuTableGroup_t * grp = _find_menu_group(arg);
                const console_menu_item_t * cmd;
                int _cmd_cnt = _count_menu_commands(arg);

                if (grp)
				{
					//iprintln(trALWAYS, "The %s (%s) commands are:", grp->description, grp->name);
					for (int j = 0; j < grp->item_cnt; j++)
						iprintln(trALWAYS, "  %s %-8s - %s", grp->name, grp->table_ptr[j].command, grp->table_ptr[j].description);
				}
                else // (_cmd_cnt > 0)
                {
                    for (int j = 0; j < _cmd_cnt; j++)
                        if (_find_menu_command(arg, j, &cmd, &grp))
                            iprintln(trALWAYS, "  %s %-8s - %s", grp->name, cmd->command, cmd->description);
                }
			}
			return;
		}
		//else, don't even bother, we are going to print everything in any case
	}

	//Reaching this point means we are printing EVERYTHING
	iprintln(trALWAYS, "The list of available commands are:");

	for (int i = 0; i < _console.menu_grp_list.cnt; i++)
	{
		ConsoleMenuTableGroup_t * grp = &_console.menu_grp_list.group[i];
		//iprintln(trALWAYS, " * %s - %s *", grp->name, grp->description);
		iprintln(trALWAYS, " \"%s\" (%s)", grp->name, grp->description);
		//iprintln(trALWAYS, "    |");
		//iprintln(trALWAYS, "");
		for (int j = 0; j < grp->item_cnt; j++)
		    iprintln(trALWAYS, "    + %-8s - %s", 
                grp->table_ptr[j].command, 
                grp->table_ptr[j].description);
        // if (i < (_console.menu_grp_list.cnt - 1))
        //     iprintln(trALWAYS, "   |");
        iprintln(trALWAYS, "");
	}

	//printf("\n%sNOTE: Enter Arguments after the command, separated by a space.");
	iprintln(trALWAYS, "Command strings longer than %d chars are invalid.", CONSOLE_RX_BUFF);
}

void _console_handler_trace_dbgPrint_i_action(eTraceFlagAction def_act)
{
    switch (def_act)
    {
        case eTraceON:      iprint(trALWAYS, "ON");         break;
        case eTraceOFF:     iprint(trALWAYS, "OFF");        break;
        case eTraceTOGGLE:  iprint(trALWAYS, "Toggle");     break;
        default:            iprint(trALWAYS, "No Action");  break;
    }
}

void _console_handler_trace(void)
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
	uint8_t tmp_tracemask = _console.tracemask;
    //uint8_t *change_mask = &change_mask_interim;


	if (console_arg_cnt() > 0)
	{
        char *arg = console_arg_pop();
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
                for (unsigned int i = 0; i < (sizeof(print_trace_combos)/sizeof(sPrintFlagActionItem)); i++)
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
                    iprintln(trALWAYS, "\t\"%s\" is not a valid trace name or action. Argument ignored!!!", arg);
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
            arg = console_arg_pop();

        } while ((NULL != arg));// && (!help_requested));

        if (valid_traces_cnt /*un_selected_cnt*/ > 0)
        {
            //_print_arg_stack();
            //We have a valid trace mask, but no state was selected, so we'll toggle it
            iprintln(trALWAYS, " No action selected for %d valid tag(s), performing default action(s):", valid_traces_cnt /*un_selected_cnt*/);
            for (int offset = (0-un_selected_cnt); offset < 0; offset++)
            {
                arg = console_arg_peek(offset);
                for (unsigned int i = 0; ((i < ARRAY_SIZE(print_trace_combos)) && (valid_traces_cnt > 0)); i++)
                {
                    if(!strcasecmp(print_trace_combos[i].name, arg))
                    {
                        valid_traces_cnt--;
                        iprint(trALWAYS, "   %10s : ", (const char *)print_trace_combos[i].name);
                        _console_handler_trace_dbgPrint_i_action(print_trace_combos[i].default_action);
                        iprint(trALWAYS, "\n");
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
                //     iprintln(trALWAYS, "\t\t %10s : Unknown (Ignored)", arg);
            }
        }

        //Righto, we are done with the arguments, let's do the toggling
	    tmp_tracemask = _console.tracemask;
        if (change_mask_on)
            _console.tracemask |= change_mask_on;
        if (change_mask_off)
            _console.tracemask &= (~change_mask_off);
        if (change_mask_toggle)
            _console.tracemask ^= change_mask_toggle;

	}

    if (help_requested)
    {
        iprintln(trALWAYS, "Usage: \"trace <tag(s)> <action (optional)> \"");
        iprintln(trALWAYS, "  <action> can be \"ON\", \"OFF\" or \"Toggle\"");
        iprintln(trALWAYS, "  <tag(s)> can be 1 or more of the following (Default Action):");
        // iprint(trALWAYS, "\t\t[");
        // for (int i = 0; i < (sizeof(print_trace_combos)/sizeof(sPrintFlagActionItem)); i++)
        //     iprint(trALWAYS, "%s%s", (const char *)print_trace_combos[i].name, (const char *)((i < (sizeof(print_trace_combos)/sizeof(sPrintFlagActionItem)-1))? ", ":"]\n"));

        //iprintln(trCONSOLE, " ==== Trace Menu ==== ");
        for (unsigned int i = 0; i < ARRAY_SIZE(print_trace_combos); i++)
        {
            uint8_t combo_mask = print_trace_combos[i].combo_mask;
            iprint(trALWAYS, "    %6s - ", (const char *)print_trace_combos[i].name);
            //Run through the active bits and find their relative names in print_trace_flag_name
            bool multi_flag = false;
            for (unsigned int j = 0; j < ARRAY_SIZE(print_trace_flag_name); j++)
            {
                if(print_trace_flag_name[j].mask & combo_mask)
                {
                    iprint(trALWAYS, "%s%s", ((multi_flag)? "|":""), print_trace_flag_name[j].abbr_name);
                    multi_flag = true;
                }
            }
            iprint(trALWAYS, " %12s", "(");
            _console_handler_trace_dbgPrint_i_action(print_trace_combos[i].default_action);
            iprint(trALWAYS, ")\n");
        }
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        iprintln(trALWAYS, "");
        iprintln(trALWAYS, "  Combination of flags switching can be performed in a single line");
        iprintln(trALWAYS, "   by specifying tag(s) then action(s)");
        iprintln(trALWAYS, "   e.g. \"trace tag_1 tag_2 ON tag_3 tag_4 OFF...\" etc");
        iprintln(trALWAYS, "  If an action is not specified, the default action for the relevant");
        iprintln(trALWAYS, "   flag is performed");
        iprintln(trALWAYS, "");
    }

	iprintln(trALWAYS, "The current state of Print Trace Flags are:");

	for (unsigned int i = 0; i < ARRAY_SIZE(print_trace_flag_name); i++)
	{
		uint8_t trace_mask = print_trace_flag_name[i].mask;
		iprintln(trALWAYS, " %15s - %s%s",
				print_trace_flag_name[i].long_name,
				(print_trace_flag_name[i].mask & _console.tracemask)? "ON  " : "OFF ",
				/* Add an asterisk if this trace flag has been changed in this operation */
				((tmp_tracemask & trace_mask) != (_console.tracemask & trace_mask))? "*" : "");
	}
}

void _console_handler_version(void)
{
	iprintln(trALWAYS, "");
	iprintln(trALWAYS, "=====================================================");
	iprintln(trALWAYS, "ButtonChaser - Button Controller");
	iprintln(trALWAYS, "[c] 2025 ZeroBadCafe Development (Pty) Ltd");
	iprintln(trALWAYS, "Version   %d.%02d.", PROJECT_VERSION / 0x10, PROJECT_VERSION % 0x10);
	iprintln(trALWAYS, "BuildInfo %s.", BUILD_TIME_DATA);
	iprintln(trALWAYS, "ESP32-C3 (Clock %lu MHz)", F_CPU / 1000000L);
	iprintln(trALWAYS, "=====================================================");
}

/*******************************************************************************
Global (public) Functions
*******************************************************************************/
void console_init(unsigned long baud, uint8_t config)
{
	//Let's not re-initialise this task by accident
	if (_console_init_done)
		return;

    _console.tracemask = trCONSOLE|trMAIN|trPWM,
    _console.menu_grp_list.cnt = 0;
    for (unsigned int i = 0; i < dev_console_MAX_MENU_ITEMS_MAX; i++)
    {
        _console.menu_grp_list.group[i].item_cnt = 0;
        _console.menu_grp_list.group[i].table_ptr = NULL;
    }
    _console.rx.cnt = 0;

    // The Console will run on the Debug port
    //Serial.begin(115200, SERIAL_8N1);
    Serial.begin(baud, config);
    Serial.flush();

    fdev_setup_stream(&_console_stdiostr, serialputc, NULL, _FDEV_SETUP_WRITE);

    _console_init_done = true;

    _console_handler_version();

	console_add_menu("con", _console_menu_items, ARRAY_SIZE(_console_menu_items), "Console Interface");

	iprintln(trCONSOLE|trALWAYS, "#Init OK (Traces: 0x%02X)", _console.tracemask);
}

bool console_service(void)
{
	int rxData;
	bool retVal = false;

    if (!_console_init_done)
	{
		iprintln(trCONSOLE | trALWAYS, "#Not Initialized yet");
		return false;
	}

	// read the incoming char:
	while (Serial.available() > 0)
	{
		// read the incoming byte:
		rxData = Serial.read();
		//iprintln(trALWAYS, "Received: \'%c\' (0x%02X)  - Inptr @ %d\n", rxData, rxData, _console.rx.cnt);
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
            serialputc('\n', NULL);
			// Serial.write('\n');
			//  Now parse the line if it is Valid
			if (_console.rx.cnt > 0)
				_parse_rx_line();
			// Start a new line now....
			_console.rx.cnt = 0; // Reset index pointer
			break;

			// ****** Backspace ******
		case 0x08:
			// Move one char back in buffer
			if (_console.rx.cnt)
			{
				_console.rx.cnt--;
                PrintBackSpace(); // Serial.print(BACKSPACE_ECHO);
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
			while (_console.rx.cnt)
			{
                PrintBackSpace();
				_console.rx.cnt--;
			}
			break;

			// ****** All other chars ******
		default:
			// Must we echo the data on the console?
			Serial.write(rxData);

			// Do we still have space in the rx buffer?
			if (_console.rx.cnt < CONSOLE_RX_BUFF) // Wrap index?
			{
				// Add char and Null Terminate string
				_console.rx.buff[_console.rx.cnt++] = rxData;
			}
			else // The index pointer now wraps, our data is invalid.
			{
				_console.rx.cnt = 0; // Reset the index pointer
			}
			break;
		}

		// Always NULL terminate whatever is in the buffer.
		_console.rx.buff[_console.rx.cnt] = 0; // Null Terminate
	}

	return retVal;
}

int console_add_menu(const char *_group_name, const console_menu_item_t *_tbl, size_t _cnt, const char *_desc)
{
	// Do not bother if we have not been initialized
	if (!_console_init_done)
	{
		iprintln(trALWAYS, "#Not Initialized yet (%s)", _group_name);
		return 0;
	}

	// First we need to decide if we have space?
	if (_console.menu_grp_list.cnt >= dev_console_MAX_MENU_ITEMS_MAX)
	{
		iprintln(trCONSOLE, "#menu list: %s (%d)\n",
			   _group_name,
			   "Out of Space",
			   _console.menu_grp_list.cnt);
		return 0;
	}

	//Don't add an empty table or a table with dulicate entries
    if (_cnt == 0)
    {
        iprintln(trCONSOLE, "#Empty table for \"%s\" (%d)", _group_name, _cnt);
        return 0;
    }

	for (unsigned int i = 0; i < _cnt-1; i++)
	{
        //Check every entry with every other entry in the table
        for (unsigned int j = i+1; j < _cnt; j++)
        {
            if (!strcasecmp(_tbl[i].command, _tbl[j].command))
            {
                iprintln(trCONSOLE, "#Duplicate command \"%s\" in \"%s\" (%d & %d)", _tbl[i].command, _group_name, i, j);
                return 0;
            }
        }
	}


	int index = 0;

	// iprintln(trALWAYS, "#Adding: %s", _group_name);

    // Check if we already have this group in the list
	for (index = 0; index < _console.menu_grp_list.cnt; index++)
	{
		// We don't want to add duplicate tables or groups
		if (!strcasecmp(_console.menu_grp_list.group[index].name, _group_name))
		{
			iprintln(trCONSOLE, "#Group \"%s\" already added @ %d)",
				   _group_name,
				   _cnt);
			return 0;
		}
		
        if (_console.menu_grp_list.group[index].table_ptr == _tbl)
		{
			iprintln(trCONSOLE, "#Table for group \"%s\" already added @ %d under \"%s\"",
				   _group_name,
				   _cnt,
                   _console.menu_grp_list.group[index].name);
			return 0;
		}

		// We also want to add it in alphabetical order....
		// We walk through the existing list until we find a group which is alphabetically "after" our group
		if (strcasecmp(_console.menu_grp_list.group[index].name, _group_name) < 0)
			continue; // with the for loop
		// else, this item group is alphabetically larger.

        //iprintln(trCONSOLE, "#Adding \"%s\" (%d) @ index %d", _group_name, _cnt, index);
		// Add this items right here
		break; // from the for-loop
	}

	//Depending on the current value of index, we may or may not need to make some space.
	if (index < _console.menu_grp_list.cnt)
	{
		//iprintln(trCONSOLE, "#Opening up space at position %d/%d", index+1, _console.menu_grp_list.cnt);
		// Open up a slot by moving everything else on with one space.
        memmove(&_console.menu_grp_list.group[index + 1], &_console.menu_grp_list.group[index], sizeof(ConsoleMenuTableGroup_t) * (_console.menu_grp_list.cnt - index));
	}

    // iprintln(trCONSOLE, "#No need to move anything (%d-%d)", index, _console.menu_grp_list.cnt);

	// Reaching this point means we can add our next table at the end.
	_console.menu_grp_list.group[index].name = _group_name;
	_console.menu_grp_list.group[index].description = _desc;
	_console.menu_grp_list.group[index].table_ptr = _tbl;
	_console.menu_grp_list.group[index].item_cnt = _cnt;
	_console.menu_grp_list.cnt++;
	//iprintln(trCONSOLE, "#Added \"%s\" @ position %d/%d", _console.menu_grp_list.group[index].name, index+1, _console.menu_grp_list.cnt);

    //RVN - I guess we can check if possible duplicate commands exist under different groups
    for (unsigned int i = 0; i < _cnt; i++)
    {
        //int tbl_index = 0;
        ConsoleMenuTableGroup_t *_group_base;
        if (_find_menu_command((char *)_tbl[i].command, 0, NULL, &_group_base))
            if (strcasecmp(_group_base->name, _group_name) != 0)
                iprintln(trCONSOLE, "#\"%s\" is duplicated in \"%s\" (%s) and \"%s\" (%s)", 
                    _tbl[i].command, 
                    _group_base->name, _group_base->description,
                    _console.menu_grp_list.group[index].name, _console.menu_grp_list.group[index].description);
    }
	return _cnt;
}

void console_print(uint8_t traceflags, const char * tag, const char * fmt, ...)
{
	if (((trALWAYS | _console.tracemask) & traceflags) == trNONE)
		return;

	//A line starts with the tag if the format string starts with "#"
    //Remember, the format string is in PROGMEM
    if (pgm_read_byte(fmt) == '#')
    {
        serialputc('[', NULL); //Serial.write('[');
        for (int i = 0; tag[i]; i++)
            serialputc(tag[i], NULL); //Serial.write(tag[i]);
        serialputc(']', NULL); //Serial.write(']');
        fmt++; // Now skip the '#'
    }

    // fdev_setup_stream(&stdiostr, serialputc, NULL, _FDEV_SETUP_WRITE);
    va_start(_console_ap, fmt);
    vfprintf_P(&_console_stdiostr, fmt, _console_ap);
    va_end(_console_ap);
}

void console_printline(uint8_t traceflags, const char * tag, const char * fmt, ...)
{
    if (((trALWAYS | _console.tracemask) & traceflags) == trNONE)
		return;

	//A line starts with the tag if the format string starts with "#"
    //Remember, the format string is in PROGMEM
    if (pgm_read_byte(fmt) == '#')
    {
        serialputc('[', NULL); //Serial.write('[');
        for (int i = 0; tag[i]; i++)
            serialputc(tag[i], NULL); //Serial.write(tag[i]);
        serialputc(']', NULL); //Serial.write(']');
        fmt++; // Now skip the '#'
    }

    va_start(_console_ap, fmt);
    vfprintf_P(&_console_stdiostr, fmt, _console_ap);
    va_end(_console_ap);

	//Ends with a newline.
    //Serial.write('\r');
    serialputc('\n', NULL); //Serial.write('\n');
}

#undef PRINTF_TAG

#endif // CONSOLE_ENABLED
/*************************** END OF FILE *************************************/
