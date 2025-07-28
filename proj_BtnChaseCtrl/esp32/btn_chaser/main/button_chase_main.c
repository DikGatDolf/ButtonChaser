/*******************************************************************************
Module:     button_chase_main.c
Purpose:    This file contains the main controlling functions for the RGB Button
            Chase project. The idea is to have a number of RGB buttons connected
            to a single ESP32, and to control them via a serial interface.
            The ESP32 will be connected to a RS485 bus, and the buttons will be
            daisy-chained on the bus. The ESP32 will be the master on the bus,
            and will control the buttons via a serial interface.
Author:     Rudolph van Niekerk

 *******************************************************************************/

 /* ---------------- RVN / TODO / NOTES / MUSINGS ---------------- 
    * Consider making a table in common_comms.h that contains the command ID, 
        the respective payload sized for rd and wr... it could potentially 
        simplify the code on the button significantly.

    * Get rid of the "remote console" commands... just use the "set" and "get" 
        commands. The console commands take up valuable space on the button and 
        can be replaced by the "set" and "get" commands.

 */

/*******************************************************************************
includes
 *******************************************************************************/
#include <ctype.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>
#include "sys_utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_random.h"

#include "defines.h"
#include "sys_utils.h"
#include "sys_timers.h"
#include "sys_task_utils.h"

#include "../../../../common/common_comms.h"

#ifdef CONSOLE_ENABLED
  #include "task_console.h"
#endif
#include "task_rgb_led.h"
#include "task_comms.h"
#include "str_helper.h"
#include "nodes.h"
#include "colour.h"
#include "task_game.h"


/*******************************************************************************
 Local defines
*******************************************************************************/
#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("Main") /* This must be undefined at the end of the file*/

#define CMD_TYPE_BROADCAST  0x01
#define CMD_TYPE_DIRECT     0x02
/*******************************************************************************
 Local structures
*******************************************************************************/
typedef enum master_state_e
{
    startup,
    roll_call,      //We are in the process of performing a roll-call
    roll_call_wait, //We are waiting for the roll-call to complete
    registrations, //We are in the process of registering nodes
    idle,           //We have been registered with the master (got a bit-mask address)

    none,          //No state set yet

}master_state_t;

typedef struct
{
    master_command_t cmd;   // The command to be sent
    const char *name_1;     // A name for the parameter
    const char *name_2;     // A name for the parameter
    const char *name_3;     // A name for the parameter
    bool allow_bcst;
}command_t;
/*******************************************************************************
 Local function prototypes
 *******************************************************************************/
bool str_to_bcst_set_cmd(const char * cmd_str, master_command_t * cmd);
bool str_to_node_get_cmd(const char * cmd_str, master_command_t * rd_cmd);
bool str_to_node_set_cmd(const char * cmd_str, master_command_t * wr_cmd);


/*! CONSOLE MENU HANDLER - Performs a system reset
 */
 void _sys_handler_reset(void);
/*! CONSOLE MENU ITEM - Prints the help string for either the command in question, 
 * or for the entire list of commands under the group
 */
void _sys_handler_tasks(void);

void _sys_handler_reg(void);
void _sys_handler_bcst(void);
void _sys_handler_node_get(void);
void _sys_handler_node_set(void);
bool _sys_handler_get_node_data(uint8_t node, uint16_t requests);
void _sys_handler_display_node_data(uint8_t node, uint16_t requests);
void _sys_handler_list(void);
bool _sys_handler_read_node_from_str(const char * arg_str_in, uint8_t * node_inout, bool* help_requested);
void _sys_handler_game(void);
void _sys_handler_sync(void);
void _sys_handler_rand(void);

/*******************************************************************************
 Local variables
 *******************************************************************************/
ConsoleMenuItem_t _task_main_menu_items[] =
{
                                    //01234567890123456789012345678901234567890123456789012345678901234567890123456789
    {"reg",     _sys_handler_reg,     "Sends a rollcall and registers all responding nodes"},
    {"bcst",    _sys_handler_bcst,    "Broadcast a msg to all inactive nodes (no response expacted)"},
    {"get",     _sys_handler_node_get,"Reads information from a specific node (response expected)"},
    {"set",     _sys_handler_node_set,"Writes information to a specific node (response expected)"},
    {"sync",    _sys_handler_sync,    "Time-sync functions for the nodes"},
    // {"list",    _sys_handler_list,    "Retrieves a list of all registered nodes"},
    {"game",    _sys_handler_game,    "Game related actions (start, stop, info, etc)"},
    // {"tasks",   _sys_handler_tasks,   "Displays the stack usage of all tasks or a specific task"},
    {"rand",    _sys_handler_rand,    "Random number generator functions"},
    {"reset",   _sys_handler_reset,   "Perform a system reset"},
};

const command_t get_cmd_table[] = {
    {cmd_get_rgb_0,   "rgb0",    "l0",   "r0",  false},
    {cmd_get_rgb_1,   "rgb1",    "l1",   "r1",  false},
    {cmd_get_rgb_2,   "rgb2",    "l2",   "r2",  false},
    {cmd_get_blink,   "blink",   "bl",   "b",   false},
    {cmd_get_reaction,"react",   "sw",   "r",   false},
    {cmd_get_flags,   "flags",   "fl",   "f",   false},
    {cmd_get_dbg_led, "dbg",     "db",   "d",   false},
    {cmd_get_time,    "time",    "cl",   "t",   false},
    {cmd_get_sync,    "sync",    "cor",  "c",   false},
    {cmd_get_version, "version", "ver",  "v",   false},
};


const command_t set_cmd_table[] = {
    {cmd_set_rgb_0,          "rgb0",    "l0",   "r0",   true},
    {cmd_set_rgb_1,          "rgb1",    "l1",   "r1",   true},
    {cmd_set_rgb_2,          "rgb2",    "l2",   "r2",   true},
    {cmd_set_blink,          "blink",   "bl",   "b",    true},
    {cmd_set_switch,         "switch",  "act",  "a",    false},
    {cmd_set_dbg_led,        "dbg",     "db",   "d",    true},
    {cmd_set_time,           "time",    "cl",   "t",    true},
    {cmd_set_sync,           "sync",    "sy",   "s",    true},
//    {cmd_new_add,            "new",     "addr", "n",    },
};
/*******************************************************************************
 Functions
*******************************************************************************/
void app_main(void)
{
    TickType_t xLastWakeTime;

    //printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    //printf("We are launching %d tasks\n", eTaskIndexMax);

    sys_timers_init();
    sys_task_clear();

#ifdef CONSOLE_ENABLED
    sys_task_add((TaskInfo_t *)console_init_task());
    console_add_menu("sys", _task_main_menu_items, ARRAY_SIZE(_task_main_menu_items), "System");
#endif
    sys_task_add((TaskInfo_t *)rgb_led_init_task());

    comms_init_task();

    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        //If a game is running, this is handled by the game task
        // if (!game_is_running())
        //     node_parse_rx_msg(); //Process any received messages, this will also update the nodes.list[x].btn fields with the responses

        //else the game should be handing this for us

        /* Inspect our own high water mark on entering the task. */
    	//task_main_info.stack_unused = uxTaskGetStackHighWaterMark2( NULL );

		//iprintln(trALWAYS, "#Task Running");

    	//We're not all that busy from this point onwards... might as well check in every 1000ms.
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        xTaskDelayUntil(&xLastWakeTime, MAX(1, pdMS_TO_TICKS(50)));
    }

    sys_timers_deinit();


    // for (int i = 10; i >= 0; i--) {
    //     printf("Restarting in %d seconds...\n", i);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
    
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}

bool str_to_node_get_cmd(const char * cmd_str, master_command_t * rd_cmd)
{
    for (int i = 0; i < ARRAY_SIZE(get_cmd_table); i++)
    {
        //Does the string match the command?
        if ((!strcasecmp(cmd_str, get_cmd_table[i].name_1)) ||
            (!strcasecmp(cmd_str, get_cmd_table[i].name_2)) ||
            (!strcasecmp(cmd_str, get_cmd_table[i].name_3)))
        {  
            if (rd_cmd != NULL)
                *rd_cmd = get_cmd_table[i].cmd; //Return the read command enum value
            return true; //We can broadcast this command
        }
    }
    return false;
}

bool str_to_node_set_cmd(const char * cmd_str, master_command_t * wr_cmd)
{
    for (int i = 0; i < ARRAY_SIZE(set_cmd_table); i++)
    {
        //Does the string match the command?
        if ((!strcasecmp(cmd_str, set_cmd_table[i].name_1)) ||
            (!strcasecmp(cmd_str, set_cmd_table[i].name_2)) ||
            (!strcasecmp(cmd_str, set_cmd_table[i].name_3)))
        {  
            if (wr_cmd != NULL)
                *wr_cmd = set_cmd_table[i].cmd; //Return the read command enum value
            
            return true; //We can broadcast this command
        }
    }
    return false;
}

bool str_to_bcst_set_cmd(const char * cmd_str, master_command_t * cmd)
{
    for (int i = 0; i < ARRAY_SIZE(set_cmd_table); i++)
    {
        //Does the string match the command?
        if ((!strcasecmp(cmd_str, set_cmd_table[i].name_1)) ||
            (!strcasecmp(cmd_str, set_cmd_table[i].name_2)) ||
            (!strcasecmp(cmd_str, set_cmd_table[i].name_3)))
        {  
            //Check if the command is allowed to be broadcast
            if (!set_cmd_table[i].allow_bcst)
                return false; //This command cannot be broadcast
            if (cmd != NULL)
                *cmd = set_cmd_table[i].cmd; //Return the read command enum value
            
            return true; //We can broadcast this command
        }
    }
    return false;
}

/*******************************************************************************
Console Functions
*******************************************************************************/

#ifdef CONSOLE_ENABLED
void _sys_handler_reset(void)
{
    static bool reset_lock = false;
	// if no parameter passed then just open the gate

    char *argStr = console_arg_pop();

	if (!reset_lock)
    {
        if (!argStr)
        {
            reset_lock = true;
            iprintln(trALWAYS, "Now type 'reset Y', IF YOU ARE SURE.\n");
            return;
        }
        iprintln(trALWAYS, "No arguments expected (got \"%s\").\n", argStr);
    }
    else //if (reset_lock)
    {
		if (0 == strcasecmp(argStr, "Y"))
        {
            // ok, do the reset.
            iprintln(trALWAYS, "Resetting. Goodbye, cruel world!\n");
            fflush(stdout);
            esp_restart();
            return;
        }        
        iprintln(trALWAYS, "'reset Y' expected. Starting over.\n");
    }
    reset_lock = false;
}

void _sys_handler_tasks(void)
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
                iprintln(trALWAYS, "Please specify a number from 0 and %d, or alternatively the task name.");
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

    iprintln(trALWAYS, "Task Information");
    iprintln(trALWAYS, "+---+------------+------------+-------+------------+--------------------+");
    iprintln(trALWAYS, "+ # | Name       | Status     | Pr'ty | Runtime    | Stack Usage        |");
    iprintln(trALWAYS, "+---+------------+------------+-------+------------+--------------------+");
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

        iprintln(trALWAYS, "| %d | %-10s | %-10s | %02d/%02d | %10u | %04u/%04u (%2.2f%%) |",
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
    iprintln(trALWAYS, "+---+------------+------------+-------+------------+--------------------+");
    iprintln(trALWAYS, "");

    iprintln(trALWAYS, "System generated Task List:");
    vTaskList(task_list_buff);
    iprintln(trALWAYS, "%s", task_list_buff);
    iprintln(trALWAYS, "Done.");
#else
    iprintln(trALWAYS, "Not Supported in this build.");
#endif
}

void _sys_handler_reg(void)
{
    //These functions (_menu_handler...) are called from the console task, so they should 
    // not send messages directly on the RS485 bus, but instead send messages to the Comms 
    // task via the msg_queue, using _tx_now()
    bool help_requested = false;

    while (console_arg_cnt() > 0)
	{
        char *arg = console_arg_pop();
        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from while-loop
        }

        // if ((!strcasecmp("f", arg)) || (!strcasecmp("force", arg)))
        // {
        //     continue;;
        // }

        iprintln(trALWAYS, "Invalid Argument (\"%s\")", arg);
        help_requested = true;
        continue; //Skip the rest of the loop and go to the next argument
    }

    if (!help_requested)
    {        
        //This call will block the console task until the roll-call timer expires, before we can register nodes
        nodes_register_all();

    }

    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        iprintln(trALWAYS, "");
        iprintln(trALWAYS, "Usage: \"reg\" - Sends a rollcall and registers all responding ");
        // iprintln(trALWAYS, "Usage: \"reg [f|force]\" - Sends a rollcalls and registers all responding ");
        // iprintln(trALWAYS, "                            buttons as nodes");
        // iprintln(trALWAYS, "    [f|force]:  clears all registered nodes before sending the rollcall");
        // iprintln(trALWAYS, "        If omitted, only unregistered (new) buttons will respond");
    }
}

void _sys_handler_bcst(void)
{
    //These functions (_menu_handler...) are called from the console task, so they should 
    // not send messages directly on the RS485 bus, but instead send messages to the Comms 
    // task via the msg_queue, using _tx_now()
    bool help_requested = false;
    // uint32_t _mask = 0x00000000;
    // bool got_mask = false;
    int valid_bcst_cmds = 0;
    uint32_t value;
    master_command_t cmd;
    button_t btn;
    uint8_t changes = 0; /* Bitmask of changes made to the button: 0x01 = RGB led 0, 0x02 = RGB led 1, 0x04 = RGB led 2, 0x08 = blink period, 0x10 = debug LED state */

    while (console_arg_cnt() > 0)
	{
        uint8_t temp_changes = 0;
        char *arg = console_arg_pop();
        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from while-loop
        }

        if (str_to_bcst_set_cmd(arg, &cmd))
        {
            if (cmd == cmd_none)
            {
                iprintln(trALWAYS, "Invalid Command \"%s\"", arg);
                help_requested = true;
                continue; //Skip the rest of the loop and go to the next argument
            }

            //We got a valid command
            if (console_arg_cnt() == 0)
            {
                //If there are no more arguments, we cannot process this command
                iprintln(trALWAYS, "No argument specified for command \"%s\"", cmd_to_str(cmd));
                help_requested = true;
                break; //from while-loop
            }
 
            //We got a valid command
            //Let's read the arguments for this command (if any)
            arg = console_arg_pop();

            if ((cmd == cmd_set_rgb_0) || (cmd == cmd_set_rgb_1) || (cmd == cmd_set_rgb_2))
            {
                //If it is a colour, it could be an RGB hex value, or a colour name
                if (!strcasecmp("off", arg))
                {
                    int rgb_led_index = (cmd - cmd_set_rgb_0); //0 for cmd_set_rgb_0, 1 for cmd_set_rgb_1, 2 for cmd_set_rgb_2
                    btn.rgb_colour[rgb_led_index] = (uint32_t)0; //Set the RGB colour for this LED
                    temp_changes = (0x01 << rgb_led_index); //Set the RGB LED state
                }
                else if (parse_str_to_colour(&value, arg) == ESP_OK)
                {
                    int rgb_led_index = (cmd - cmd_set_rgb_0); //0 for cmd_set_rgb_0, 1 for cmd_set_rgb_1, 2 for cmd_set_rgb_2
                    btn.rgb_colour[rgb_led_index] = (uint32_t)value; //Set the RGB colour for this LED
                    temp_changes = (0x01 << rgb_led_index); //Set the RGB LED state
                }
            }
            else if (cmd == cmd_set_blink)
            {
                //If it is a blink period, we are expecting ONLY an integer value
                if ((!strcasecmp("off", arg)) || (!strcasecmp("stop", arg)))
                {
                    btn.blink_ms = 0; //Set the blink period to 0 (off)
                    temp_changes = 0x08; //Set the blink period
                }
                else if (str2uint32(&value, arg, 0))
                {
                    btn.blink_ms = (uint32_t)value; //Set the blink period
                    temp_changes = 0x08; //Set the blink period
                }
            }
            else if (cmd == cmd_set_dbg_led)
            {
                if (!strcasecmp("off", arg))
                {
                    btn.dbg_led_state = dbg_led_off; //Set the debug LED state to OFF
                    temp_changes = 0x10; //Set the debug LED state to ON
                }
                else if (!strcasecmp("on", arg))
                {
                    btn.dbg_led_state = dbg_led_on; //Set the debug LED state to ON
                    temp_changes = 0x10; //Set the debug LED state to ON
                }
                else if (str2uint32(&value, arg, 0))
                {
                    if (value <= UINT8_MAX)
                    {
                        btn.dbg_led_state = (uint8_t)value; //Set the blink period
                        temp_changes = 0x10; //Set the debug LED state to ON
                    }
                }
            }

            //If tepm_changes is 0, it means we did not change anything for this command
            if (0 == temp_changes)
            {
                iprintln(trALWAYS, "Invalid Argument \"%s\" for command \"%s\"", arg, cmd_to_str(cmd));
                help_requested = true;
                continue; //Skip the rest of the loop and go to the next argument
            }

            //Righto, we have a valid command and a valid argument
            valid_bcst_cmds++;
            changes |= temp_changes; //Add the changes made by this command to the overall changes bitmask
            continue;
        }
        iprintln(trALWAYS, "Invalid Command: \"%s\"", arg);
        help_requested = true;
    }

    if (!help_requested)
    {
        if (valid_bcst_cmds == 0)
        {
            iprintln(trALWAYS, "No commands specified. Please specify at least one command & argument to broadcast.");
            help_requested = true;
        }
        // if (_mask == 0x0000)
        // {
        //     iprintln(trALWAYS, "No INACTIVE nodes found. Please register some nodes first.");
        //     return; //Nothing to do further
        // }
        else
        {
            //iprintln(trALWAYS, "Broadcasting to nodes with mask 0x%04X (%d nodes)", _mask, node_count());
            init_bcst_msg();
            bool msg_success = true;
            for (uint8_t i = 0; i < 8; i++)
            {
                uint8_t mask = BIT_POS(i);
                if ((changes & mask) == 0)
                    continue; //Skip this bit, we are not changing anything for this command

                switch (mask)
                {
                    case 0x01: msg_success = add_bcst_msg_set_rgb(0, btn.rgb_colour[0]); break;
                    case 0x02: msg_success = add_bcst_msg_set_rgb(1, btn.rgb_colour[1]); break;
                    case 0x04: msg_success = add_bcst_msg_set_rgb(2, btn.rgb_colour[2]); break;
                    case 0x08: msg_success = add_bcst_msg_set_blink(btn.blink_ms); break;
                    case 0x10: msg_success = add_bcst_msg_set_dbgled(btn.dbg_led_state); break;
                    default: msg_success = false; break; //Skip any other bits
                }

                if (!msg_success)
                {
                    iprintln(trALWAYS, "Failed to initialize broadcast message (0x%02X)", mask);
                    return; //Nothing to do further
                }
            }
            bcst_msg_tx_now();
            iprintln(trALWAYS, "Broadcasted %d command%s to %d node%s", valid_bcst_cmds, (valid_bcst_cmds > 1) ? "s" : "", node_count() - active_node_count(), ((node_count() - active_node_count()) > 1) ? "s" : "");
        }
    }

    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        iprintln(trALWAYS, "");
        iprintln(trALWAYS, "Usage: \"bcst <cmd 1> <arg 1> [<cmd 2> <arg 2>... <cmd n> <arg n>]\"");
        iprintln(trALWAYS, "    <cmd X> <arg X>: any broadcast COMMAND and a valid ARGUMENT:");
        iprintln(trALWAYS, "        \"rgb0|rgb1|rgb2 <colour>\": sets the RGB colour for the specific RGB LED");
        iprintln(trALWAYS, "        \"blink <period>\": sets the blink period (ms) for the button (0/off to disable)");
        iprintln(trALWAYS, "        \"dbg_led <state>\": sets the debug LED state (off, on, fast, med, slow)");
    }
}

void _sys_handler_node_set(void)
{
    //These functions (_menu_handler...) are called from the console task, so they should 
    // not send messages directly on the RS485 bus, but instead send messages to the Comms 
    // task via the msg_queue, using _tx_now()
    bool help_requested = false;
    // uint32_t _mask = 0x00000000;
    uint8_t _node = 0xff;
    int valid_node_set_cmds = 0;
    uint32_t value;
    master_command_t cmd;
    button_t btn;
    uint8_t changes = 0; /* Bitmask of changes made to the button: 0x01 = RGB led 0, 0x02 = RGB led 1, 0x04 = RGB led 2, 0x08 = blink period, 0x10 = debug LED state */

    while (console_arg_cnt() > 0)
	{
        uint8_t temp_changes = 0;
        char *arg = console_arg_pop();
        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from while-loop
        }

        //Check if the argument is a hex value (node address)
        if (_sys_handler_read_node_from_str(arg, &_node, &help_requested))
            continue; //Skip the rest of the loop and go to the next argument
        else if (help_requested)
            break; //Skip the rest of the loop and go to the next argument

        if (str_to_node_set_cmd(arg, &cmd))
        {
            if (cmd == cmd_none)
            {
                iprintln(trALWAYS, "Invalid Command \"%s\"", arg);
                help_requested = true;
                continue; //Skip the rest of the loop and go to the next argument
            }

            //We got a valid command
            if (console_arg_cnt() == 0)
            {
                //If there are no more arguments, we cannot process this command
                iprintln(trALWAYS, "No argument specified for command \"%s\"", cmd_to_str(cmd));
                help_requested = true;
                break; //from while-loop
            }

            //Let's read the arguments for this command (if any)
            arg = console_arg_pop();

            if ((arg == NULL) || (strlen(arg) == 0))
            {
                //If there is no argument, we cannot process this command
                //This is a special case for commands that require an argument
                //e.g. set_rgb_0, set_rgb_1, set_rgb_2, set_blink, set_dbg_led, set_switch
                iprintln(trALWAYS, "No argument specified for command \"%s\"", cmd_to_str(cmd));
                help_requested = true;
                continue; //Skip the rest of the loop and go to the next argument
            }

            if ((cmd == cmd_set_rgb_0) || (cmd == cmd_set_rgb_1) || (cmd == cmd_set_rgb_2))
            {
                //If it is a colour, it could be an RGB hex value, or a colour name
                if (!strcasecmp("off", arg))
                {
                    int rgb_led_index = (cmd - cmd_set_rgb_0); //0 for cmd_set_rgb_0, 1 for cmd_set_rgb_1, 2 for cmd_set_rgb_2
                    btn.rgb_colour[rgb_led_index] = (uint32_t)0; //Set the RGB colour for this LED
                    temp_changes = (0x01 << rgb_led_index); //Set the RGB LED state
                }
                else if (parse_str_to_colour(&value, arg) == ESP_OK)
                {
                    int rgb_led_index = (cmd - cmd_set_rgb_0); //0 for cmd_set_rgb_0, 1 for cmd_set_rgb_1, 2 for cmd_set_rgb_2
                    btn.rgb_colour[rgb_led_index] = (uint32_t)value; //Set the RGB colour for this LED
                    temp_changes = (0x01 << rgb_led_index); //Set the RGB LED state
                }
            }
            else if (cmd == cmd_set_blink)
            {
                //If it is a blink period, we are expecting ONLY an integer value
                if ((!strcasecmp("off", arg)) || (!strcasecmp("stop", arg)))
                {
                    btn.blink_ms = 0; //Set the blink period to 0 (off)
                    temp_changes = 0x08; //Set the blink period
                }
                else if (str2uint32(&value, arg, 0))
                {
                    btn.blink_ms = (uint32_t)value; //Set the blink period
                    temp_changes = 0x08; //Set the blink period
                }
            }
            else if (cmd == cmd_set_dbg_led)
            {
                if (!strcasecmp("off", arg))
                {
                    btn.dbg_led_state = dbg_led_off; //Set the debug LED state to OFF
                    temp_changes = 0x10; //Set the debug LED state to ON
                }
                else if (!strcasecmp("on", arg))
                {
                    btn.dbg_led_state = dbg_led_on; //Set the debug LED state to ON
                    temp_changes = 0x10; //Set the debug LED state to ON
                }
                else if (str2uint32(&value, arg, 0))
                {
                    if (value <= UINT8_MAX)
                    {
                        btn.dbg_led_state = (uint8_t)value; //Set the blink period
                        temp_changes = 0x10; //Set the debug LED state to ON
                    }
                }
            }
            else if (cmd == cmd_set_switch)
            {
                if ((!strcasecmp("off", arg)) || (!strcasecmp("0", arg)))
                {
                    btn.sw_active = false; //Deactivate the button press timer
                    temp_changes = 0x20; //Set the switch state
                }
                else if ((!strcasecmp("on", arg)) || (!strcasecmp("1", arg)))
                {
                    btn.sw_active = true; //Activate the button press timer
                    temp_changes = 0x20; //Set the switch state
                }
            }
            else if (cmd == cmd_set_time)
            {
                if (str2uint32(&value, arg, 0))
                {
                    btn.time_ms = (uint32_t)value; //Set the time
                    temp_changes = 0x40; //Set the time
                }
            }

            //If temp_changes is 0, it means we did not change anything for this command
            if (0 == temp_changes)
            {
                iprintln(trALWAYS, "Invalid Argument \"%s\" for command \"%s\"", arg, cmd_to_str(cmd));
                help_requested = true;
                continue; //Skip the rest of the loop and go to the next argument
            }

            //Righto, we have a valid command and a valid argument
            valid_node_set_cmds++;
            changes |= temp_changes; //Add the changes made by this command to the overall changes bitmask
            continue;
        }
        iprintln(trALWAYS, "Invalid Command: \"%s\"", arg);
        help_requested = true;
    }

    if (!help_requested)
    {
        if (valid_node_set_cmds == 0)
        {
            iprintln(trALWAYS, "No SET commands specified. Please specify at least one command & argument to write to node %d", _node);
            help_requested = true;
        }
        // if (_mask == 0x0000)
        // {
        //     iprintln(trALWAYS, "No INACTIVE nodes found. Please register some nodes first.");
        //     return; //Nothing to do further
        // }
        else
        {
            //iprintln(trALWAYS, "Broadcasting to nodes with mask 0x%04X (%d nodes)", _mask, node_count());
            init_node_msg(_node);
            bool msg_success = true;
            for (uint8_t i = 0; i < 8; i++)
            {
                uint8_t mask = BIT_POS(i);
                if ((changes & mask) == 0)
                    continue; //Skip this bit, we are not changing anything for this command

                switch (mask)
                {
                    case 0x01: msg_success = add_node_msg_set_rgb(_node, 0, btn.rgb_colour[0]); break;
                    case 0x02: msg_success = add_node_msg_set_rgb(_node, 1, btn.rgb_colour[1]); break;
                    case 0x04: msg_success = add_node_msg_set_rgb(_node, 2, btn.rgb_colour[2]); break;
                    case 0x08: msg_success = add_node_msg_set_blink(_node, btn.blink_ms); break;
                    case 0x10: msg_success = add_node_msg_set_dbgled(_node, btn.dbg_led_state); break;
                    case 0x20: msg_success = add_node_msg_set_active(_node, btn.sw_active); break;
                    case 0x40: msg_success = add_node_msg_set_time(_node, btn.time_ms); break;
                    default: msg_success = false; break; //Skip any other bits
                }

                if (!msg_success)
                {
                    iprintln(trALWAYS, "Failed to load msg for node %d (0x%02X)", _node, mask);
                    return; //Nothing to do further
                }
            }
            //Wait for the response from the node
            if (node_msg_tx_now((uint8_t)_node))
                iprintln(trALWAYS, "Wrote %d command%s to node %d (0x%02X)", valid_node_set_cmds, (valid_node_set_cmds > 1) ? "s" : "", _node, get_node_addr(_node));
        }
    }

    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        iprintln(trALWAYS, "");
        iprintln(trALWAYS, "Usage: \"set <#> <cmd_&_arg_1> [<cmd_&_arg_2>... <cmd_&_arg_n>]\"");
        iprintln(trALWAYS, "    <#>:  Node slot number (0 to %d)", RGB_BTN_MAX_NODES);
        iprintln(trALWAYS, "        IMPORTANT: this MUST be the first argument in the stream");
        iprintln(trALWAYS, "    <cmd_&_arg_X>: any SET COMMAND and a valid ARGUMENT:");
        iprintln(trALWAYS, "        \"rgb0|rgb1|rgb2 <colour>\": sets the RGB colour for the specific RGB LED");
        iprintln(trALWAYS, "        \"blink <period>\": sets the blink period (ms) for the button (0/off to disable)");
        iprintln(trALWAYS, "        \"dbg_led <state>\": sets the debug LED state (off, on, fast, med, slow)");
        iprintln(trALWAYS, "        \"sw (on|off)\": starts or stops the button press timer");
        iprintln(trALWAYS, "        \"time <ms>\": sets the current time (32-bit unsigned ms value)");
    }
}

#define NODE_MAX_GET_CMDS 10 //Maximum number of GET commands we can request from a node
void _sys_handler_node_get(void)
{
    //These functions (_menu_handler...) are called from the console task, so they should 
    // not send messages directly on the RS485 bus, but instead send messages to the Comms 
    // task via the msg_queue, using _tx_now()
    bool help_requested = false;
    // uint32_t _mask = 0x00000000;
    uint8_t _node = 0xff;
    int valid_node_get_cmds = 0;
    master_command_t cmd;
//    button_t btn;
    uint16_t requests = 0; /* Bitmask of read requests made to the button:
                                     0x0001 = RGB led 0, 
                                     0x0002 = RGB led 1, 
                                     0x0004 = RGB led 2, 
                                     0x0008 = blink period, 
                                     0x0010 = debug LED state 
                                     0x0020 = switch reaction time, 
                                     0x0040 = time, 
                                     0x0080 = state flags
                                     0x0100 = correction factor 
                                     0x0200 = Version */

    while (console_arg_cnt() > 0)
	{
        uint16_t temp_request = 0;
        char *arg = console_arg_pop();
        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from while-loop
        }

        //Check if the argument is a hex value (node address)
        if (_sys_handler_read_node_from_str(arg, &_node, &help_requested))
            continue; //Skip the rest of the loop and go to the next argument
        else if (help_requested)
            break; //Skip the rest of the loop and go to the next argument

        // if ((!strcasecmp("addr", arg)) || (!strcasecmp("address", arg)))
        // {    
        //     requests |= 0x80;
        //     continue; //with the while-loop
        // }

        if (!strcasecmp("all", arg))
        {    
            //Set all the bits in the requests bitmask
            requests = 0xFFFF; //Set the RGB LED state
            valid_node_get_cmds = NODE_MAX_GET_CMDS; //We are requesting all commands
            break; //from while-loop
        }

        if (str_to_node_get_cmd(arg, &cmd))
        {
            if (cmd == cmd_none)
            {
                iprintln(trALWAYS, "Invalid Command \"%s\"", arg);
                help_requested = true;
                continue; //Skip the rest of the loop and go to the next argument
            }

            //We got a valid get command

            if ((cmd == cmd_get_rgb_0) || (cmd == cmd_get_rgb_1) || (cmd == cmd_get_rgb_2))
            {
                int rgb_led_index = (cmd - cmd_get_rgb_0); //0 for cmd_set_rgb_0, 1 for cmd_set_rgb_1, 2 for cmd_set_rgb_2
                temp_request = BIT_POS(rgb_led_index); //Set the RGB LED state
            }
            else if (cmd == cmd_get_blink)
            {
                temp_request = BIT_POS(3); //Set the blink period
            }
            else if (cmd == cmd_get_dbg_led)
            {
                temp_request = BIT_POS(4); //Set the debug LED state to ON
            }
            else if (cmd == cmd_get_reaction)
            {
                temp_request = BIT_POS(5); //Get the switch reaction time
            }
            else if (cmd == cmd_get_time)
            {
                temp_request = BIT_POS(6); //Get the time
            }
            else if (cmd == cmd_get_flags)
            {
                temp_request = BIT_POS(7); //Get the node flags
            }
            else if (cmd == cmd_get_sync)
            {
                temp_request = BIT_POS(8); //Get the correction factor
            }
            else if (cmd == cmd_get_version)
            {
                temp_request = BIT_POS(9); //Get the version number
            }
            else
            {
                iprintln(trALWAYS, "Invalid GET command \"%s\" (%s)", arg, cmd_to_str(cmd));
                help_requested = true;
                continue; //Skip the rest of the loop and go to the next argument
            }

            //Righto, we have a valid command and a valid argument (only if we havent requested it in this stream already)
            if ((requests & temp_request) == 0)
                valid_node_get_cmds++;
            requests |= temp_request; //Add the changes made by this command to the overall changes bitmask
            continue;
        }
        iprintln(trALWAYS, "Invalid Command: \"%s\"", arg);
        help_requested = true;
    }

    if (!help_requested)
    {
        if (valid_node_get_cmds == 0)
        {
            iprintln(trALWAYS, "No GET commands specified. Please specify at least one value to read");
            help_requested = true;
        }
        else if (_node == 0xff)
        {
            //Get the node data for every registered node
            for (uint8_t i = 0; i < RGB_BTN_MAX_NODES; i++)
            {
                if (!is_node_valid(i))
                    continue; //Skip unregistered nodes

                if (_sys_handler_get_node_data(i, requests))
                    _sys_handler_display_node_data(i, requests);
            }
        }
        else
        {
            //iprintln(trALWAYS, "Broadcasting to nodes with mask 0x%04X (%d nodes)", _mask, node_count());

            if (_sys_handler_get_node_data(_node, requests))
                _sys_handler_display_node_data(_node, requests);
        }
    }

    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        iprintln(trALWAYS, "");
        iprintln(trALWAYS, "Usage: \"get <#> <cmd 1> [<cmd 2> ... <cmd n>]\"");
        iprintln(trALWAYS, "    <#>:  Node slot number (0 to %d)", RGB_BTN_MAX_NODES);
        iprintln(trALWAYS, "      If ommitted, the command(s) will be sent to all nodes in sequence");
        iprintln(trALWAYS, "    get <cmd X>: any node get COMMAND");
        iprintln(trALWAYS, "      rgb0|rgb1|rgb2: get the RGB colour for the specific RGB LED");
        iprintln(trALWAYS, "      blink: get the blink period (ms) for the button");
        iprintln(trALWAYS, "      sw:    get the button press time (0 if inactive or not pressed yet)");
        iprintln(trALWAYS, "      flags: get the node flags");        
        iprintln(trALWAYS, "      dbg:   get the debug LED state");
        iprintln(trALWAYS, "      time:  get the current time (32-bit unsigned ms value)");
        iprintln(trALWAYS, "      sync:  get the node's time correction factor");
        iprintln(trALWAYS, "      version: get the node's firmware version");
        iprintln(trALWAYS, "      all:   get all node parameters (rgb0-2, blink, dbg, sw, flags, time, sync)");
    }
}

bool _sys_handler_get_node_data(uint8_t node, uint16_t requests)
{
    if (node >= RGB_BTN_MAX_NODES)
    {
        iprintln(trALWAYS, "Invalid node number %d. Must be between 0 and %d", node, RGB_BTN_MAX_NODES - 1);
        return false;
    }
    if (!is_node_valid(node))
    {
        iprintln(trALWAYS, "Node %d is not registered", node);
        return false;
    }
    if (requests == 0)
    {
        iprintln(trALWAYS, "No GET requests made for node %d", node);
        return true;
    }

    init_node_msg(node);
    bool msg_success = true;
    for (uint8_t i = 0; i < NODE_MAX_GET_CMDS; i++)
    {
        uint16_t mask = BIT_POS(i);
        if ((requests & mask) == 0)
            continue; //Skip this bit, we are not request this

        switch (mask)
        {
            case 0x0001: msg_success = add_node_msg_get_rgb(node, 0); break;
            case 0x0002: msg_success = add_node_msg_get_rgb(node, 1); break;
            case 0x0004: msg_success = add_node_msg_get_rgb(node, 2); break;
            case 0x0008: msg_success = add_node_msg_get_blink(node); break;
            case 0x0010: msg_success = add_node_msg_get_dbgled(node); break;
            case 0x0020: msg_success = add_node_msg_get_reaction(node); break;
            case 0x0040: msg_success = add_node_msg_get_time(node); break;
            case 0x0080: msg_success = add_node_msg_get_flags(node); break;
            case 0x0100: msg_success = add_node_msg_get_correction(node); break;
            case 0x0200: msg_success = add_node_msg_get_version(node); break;
            default: msg_success = false; break; //Skip any other bits
        }

        if (!msg_success)
        {
            iprintln(trALWAYS, "Failed to load GET msg for node %d (0x%02X)", node, mask);
            return false; //Nothing to do further
        }
    }

    //Wait for the response from the node
    return node_msg_tx_now((uint8_t)node);
}

void _sys_handler_display_node_data(uint8_t node, uint16_t requests)
{
    if (node >= node_count())
    {
        iprintln(trALWAYS, "Invalid node number %d. Must be between 0 and %d", node, node_count() - 1);
        return;
    }
    if (!is_node_valid(node))
    {
        iprintln(trALWAYS, "Node %d is not registered", node);
        return;
    }
    if (requests == 0)
    {
        iprintln(trALWAYS, "No GET requests made for node %d", node);
        return;
    }
    //Print the responses
    iprintln(trALWAYS, "Node %d Data:", node);
    button_t *btn = get_node_button_ptr(node);
    for (uint8_t i = 0; i < NODE_MAX_GET_CMDS; i++)
    {
        uint16_t mask = BIT_POS(i);
        if ((requests & mask) == 0)
            continue; //Skip this bit, we did notrequest this

        switch (mask)
        {
            case 0x0001: 
            case 0x0002:
            case 0x0004:
                iprint(trALWAYS,   "RGB[%d]:       ", i);
                if (rgb2name(btn->rgb_colour[i]) == NULL)
                    iprintln(trALWAYS, "0x%06X", btn->rgb_colour[i]);
                else
                    iprintln(trALWAYS, "%s", rgb2name(btn->rgb_colour[i]));
                break;
            case 0x0008:
                iprintln(trALWAYS, "Blink period: %d ms", btn->blink_ms);
                break;
            case 0x0010:
                iprint(trALWAYS,   "Debug LED:    ");
                if (btn->dbg_led_state == dbg_led_off)      iprintln(trALWAYS, "OFF");
                else if (btn->dbg_led_state == dbg_led_on)  iprintln(trALWAYS, "ON");
                else                                        iprintln(trALWAYS, "BLINK (%dms)", btn->dbg_led_state*10);
                break;
            case 0x0020:
                iprintln(trALWAYS, "Reaction:     %u ms", btn->reaction_ms);
                break;
            case 0x0040: 
                iprintln(trALWAYS, "Time:         %u ms", btn->time_ms);
                break;
            case 0x0080: 
                iprint(trALWAYS,   "Flags:        0x%02X", btn->flags);
                if (btn->flags != 0)
                {
                    bool first = false;
                    iprint(trALWAYS, " (");
                    for (uint8_t f = 0; f < 8; f++)
                    {
                        if (btn->flags & BIT_POS(f))
                        {
                            switch (BIT_POS(f))
                            {
                                case flag_s_press:      iprint(trALWAYS, "%s%s", (first)? "|" : "", "SHT_PRS"); break;
                                case flag_l_press:      iprint(trALWAYS, "%s%s", (first)? "|" : "", "LNG_PRS"); break;
                                case flag_d_press:      iprint(trALWAYS, "%s%s", (first)? "|" : "", "DBL_PRS"); break;
                                case flag_activated:    iprint(trALWAYS, "%s%s", (first)? "|" : "", "ACTIVATED"); break;
                                case flag_deactivated:  iprint(trALWAYS, "%s%s", (first)? "|" : "", "DEACTIVATED"); break;
                                case flag_sw_stopped:   iprint(trALWAYS, "%s%s", (first)? "|" : "", "STOPPED"); break;
                                case flag_blinking:     iprint(trALWAYS, "%s%s", (first)? "|" : "", "BLINKING"); break;
                                case flag_unreg:        iprint(trALWAYS, "%s%s", (first)? "|" : "", "UNREG"); break;
                            }
                            first = true; //Set the first flag to true, so we can print the comma
                        }
                    }
                    iprint(trALWAYS, ")");
                }
                iprintln(trALWAYS, "");
                break;
            case 0x0100: 
                iprintln(trALWAYS,   "Time Factor:  %.6f", btn->time_factor);
                break;
            case 0x0200: 
                iprintln(trALWAYS,   "Version:      %d.%d.%d.%d", 
                    (btn->version & 0x000000FF), 
                    (btn->version & 0x0000FF00) >> 8, 
                    (btn->version & 0x00FF0000) >> 16,
                    (btn->version & 0xFF000000) >> 24);
                break;
            default: 
                iprintln(trALWAYS, "Unknown GET mask for node %d (0x%02X)", node, mask);
                break; //Skip any other bits
        }
    }
    iprintln(trALWAYS, "------------------------------------------------");
}

void _sys_handler_list(void)
{
    iprintln(trALWAYS, "NOT IMPLEMENTED YET");
}

void _sys_handler_sync(void)
{
    //These functions (_menu_handler...) are called from the console task, so they should 
    // not send messages directly on the RS485 bus, but instead send messages to the Comms 
    // task via the msg_queue, using _tx_now()
    bool help_requested = false;
    // uint32_t _mask = 0x00000000;
    uint8_t _node = 0xff;
    uint16_t _actions = 0x0000;// 0x01 = reset, 0x02 = start, 0x04 = stop

    while (console_arg_cnt() > 0)
	{
        char *arg = console_arg_pop();
        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from while-loop
        }
        if ((!strcasecmp("reset", arg)))
        {    
            _actions |= BIT_POS(0); //Set the action to reset the node's correction factor
            continue; //from while-loop
        }
        if ((!strcasecmp("go", arg)) || (!strcasecmp("start", arg)))
        {    
            _actions |= BIT_POS(1); //Set the action to start the sync process
            continue; //from while-loop
        }
        if ((!strcasecmp("end", arg)) || (!strcasecmp("stop", arg)))
        {    
            _actions |= BIT_POS(2); //Set the action to end the sync process
            continue; //from while-loop
        }

        //Check if the argument is a hex value (node address)
        if (_sys_handler_read_node_from_str(arg, &_node, &help_requested))
            continue; //Skip the rest of the loop and go to the next argument
        else if (help_requested)
            break; //Skip the rest of the loop and go to the next argument        

        iprintln(trALWAYS, "Invalid Command: \"%s\"", arg);
        help_requested = true;
        break;
    }

    if (!help_requested)
    {
        if (_actions == 0)
        {
            iprintln(trALWAYS, "No actions specified. Please specify one action (reset, start or stop)");
            help_requested = true;
        }
        //Check if more than one action is specified
        else if ((_actions & (_actions - 1)) != 0)
        {
            iprintln(trALWAYS, "Multiple actions specified. Please specify only one action (reset, start or stop)");
            help_requested = true;
        }
        else if ((_actions == BIT_POS(2)) && (!is_time_sync_busy()))
        {
            iprintln(trALWAYS, "Sync process is not running");
            return; //Nothing to do further
        }
        else
        {
            //Depending on whether we have a valid node address or not, we will either broadcast the command to all nodes, or send it to a specific node
            if (_node == 0xff)              //Broadcast the command to all nodes
                init_bcst_msg();
            else                            //Send the command to a specific node
                init_node_msg(_node);

            bool msg_success = true;
            switch (_actions)
            {
                case 0x01: msg_success = (_node == 0xff)? add_bcst_msg_sync_reset() : add_node_msg_sync_reset(_node); break;
                case 0x02: msg_success = (_node == 0xff)? add_bcst_msg_sync_start() : add_node_msg_sync_start(_node); break;
                case 0x04: msg_success = (_node == 0xff)? add_bcst_msg_sync_end()   : add_node_msg_sync_end(_node); break;
                default: msg_success = false; break; //Skip any other bits
            }
            if (!msg_success)
            {
                if (_node == 0xff)
                    iprintln(trALWAYS, "Failed to load broadcast msg");
                else
                    iprintln(trALWAYS, "Failed to load msg for node %d", _node);
                return; //Nothing to do further
            }
            else if (_node == 0xff)
            {
                bcst_msg_tx_now();      //Fire and forget the Broadcast the message to all nodes
                iprintln(trALWAYS, "Broadcast msg (0x%02X) sent", _actions);
            }
            else if (node_msg_tx_now((uint8_t)_node))
                iprintln(trALWAYS, "Msg (0x%02X) sent to node %d", _actions, _node);
        }
    }

    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        iprintln(trALWAYS, "");
        iprintln(trALWAYS, "Usage: \"sync <action>  [<#>]\"");
        iprintln(trALWAYS, "    <action>: one of the following:");
        iprintln(trALWAYS, "        \"reset\" : resets the correction factor to 1.0");
        iprintln(trALWAYS, "        \"start\" : starts the synchronization process");
        iprintln(trALWAYS, "        \"stop\"  : ends the synchronization process");
        iprintln(trALWAYS, " IMPORTANT: 1 (and only 1) action must be specified");
        iprintln(trALWAYS, "    <#>:  Node number (0 to %d)", RGB_BTN_MAX_NODES);
        iprintln(trALWAYS, "        if omitted, the command will be broadcast to all nodes");
    }

}

bool _sys_handler_read_node_from_str(const char * arg_str_in, uint8_t * node_inout, bool* help_requested)
{
    uint32_t value;

    //Is this the first time we're getting a node address?
    if (*node_inout == 0xff) //If we have not got a node address yet
    { 
        if (str2uint32(&value, arg_str_in, 0))
        {
            if ((is_node_valid((uint8_t)value)) && (value < RGB_BTN_MAX_NODES))
            {
                *node_inout = (uint8_t)value; //We got a valid node 
                return true;
            }
            iprint(trALWAYS, "Invalid Node (\"%s\"). Options are [:", arg_str_in);
            for (int i = 0; i < RGB_BTN_MAX_NODES; i++)
                if (is_node_valid(i))
                    iprint(trALWAYS, "%s%d", (i > 0) ? ", " : "", i);
            iprintln(trALWAYS, "]");
            *help_requested = true;
        }
    }
    return false;
}

void _sys_handler_game(void)
{
    //These functions (_menu_handler...) are called from the console task, so they should 
    // not send messages directly on the RS485 bus, but instead send messages to the Comms 
    // task via the msg_queue, using _tx_now()
    bool help_requested = false;
    uint32_t value;
    bool got_start = false;
    bool got_stop = false;
    bool got_settings = false;
    bool got_pause = false;
    bool got_resume = false;
    int game_nr = -1;
    char * game_args[10];
    int game_arg_cnt = 0;

    while (console_arg_cnt() > 0)
	{
        char *arg = console_arg_pop();

        if ((got_start && (game_nr >= 0)) || got_settings)
        {
            //These are now considered to be paramters for the game to start, or change
            if (game_arg_cnt >= 10)
            {
                iprintln(trALWAYS, "Too many game arguments specified. Maximum is 10.");
                help_requested = true;
                break; //from while-loop
            }
            game_args[game_arg_cnt++] = arg;
            got_settings = true; //If we got a game argument, we cannot have a settings command
            continue; //Skip the rest of the loop and go to the next argument
        }

        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from while-loop
        }

        if ((str2uint32(&value, arg, 0)) && (game_nr < 0))
        {
            if (value < games_cnt())
            {
                game_nr = (int)value; //We got a valid nr
            }
            else
            {
                iprintln(trALWAYS, "Invalid Game Nr (\"%s\"). Options are: [0 to %d]", arg, games_cnt() - 1);
                help_requested = true;
            }
            continue; //Skip the rest of the loop and go to the next argument
        }
        if ((!strcasecmp("stop", arg)) || (!strcasecmp("end", arg)))
        {
            //If we got a stop command, we cannot have a start or info command
            if (got_start || got_settings || got_pause || got_resume)
            {
                iprintln(trALWAYS, "Invalid combination of actions. Please specify only one action.");
                help_requested = true;
                break;//from while-loop
            }
            got_stop = true;
            continue; //Skip the rest of the loop and go to the next argument
        }
        if ((!strcasecmp("start", arg)) || (!strcasecmp("go", arg)))
        {
            //If we got a start command, we cannot have a stop or info command
            if (got_stop || got_settings || got_pause || got_resume)
            {
                iprintln(trALWAYS, "Invalid combination of actions. Please specify only one action.");
                help_requested = true;
                break;//from while-loop
            }
            got_start = true;
            continue; //Skip the rest of the loop and go to the next argument
        }
        if ((!strcasecmp("set", arg)) || (!strcasecmp("settings", arg)) || (!strcasecmp("setting", arg)))
        {
            if (got_stop || got_pause || got_resume)
            {
                iprintln(trALWAYS, "Invalid combination of actions. Please specify only one action.");
                help_requested = true;
                break;//from while-loop
            }
            //If we got a start command, we cannot have a stop or info command
            got_settings = true;
            continue; //Skip the rest of the loop and go to the next argument
        }
        if (!strcasecmp("pause", arg))
        {
            if (got_stop || got_settings || got_resume || got_start)
            {
                iprintln(trALWAYS, "Invalid combination of actions. Please specify only one action.");
                help_requested = true;
                break;//from while-loop
            }
            //If we got a start command, we cannot have a stop or info command
            got_pause = true;
            continue; //Skip the rest of the loop and go to the next argument
        }
        if (!strcasecmp("resume", arg))
        {
            if (got_stop || got_settings || got_pause || got_start)
            {
                iprintln(trALWAYS, "Invalid combination of actions. Please specify only one action.");
                help_requested = true;
                break;//from while-loop
            }
            //If we got a start command, we cannot have a stop or info command
            got_resume = true;
            continue; //Skip the rest of the loop and go to the next argument
        }

        iprintln(trALWAYS, "Invalid Argument (\"%s\")", arg);
        help_requested = true;
        continue; //Skip the rest of the loop and go to the next argument
    }

    if (!help_requested)
    {
        //Deal with actions that require a game number, but maybe did not get one
        if (game_nr < 0)
        {
            if (got_start)
            {
                iprintln(trALWAYS, "Please specify a game number (0 to %d) to start", games_cnt() - 1);
                return;
            }
            //else ((got_stop || got_settings)
            if  (current_game() < 0)
            {
                iprintln(trALWAYS, "No game is currently running. Please specify a game number (0 to %d) to start", games_cnt() - 1);
                return;
            }
            game_nr = current_game(); //If we did not get a game number, we will use the current game
            //Fall throught
        }
        //submit the game arguments to the game
        if ((game_arg_cnt > 0) && got_settings)
        {
            iprint(trALWAYS, "Game argument%s: ", (game_arg_cnt > 1) ? "s" : "");
            for (int i = 0; i < game_arg_cnt; i++)
            {
                if (i > 0) iprint(trALWAYS, ", ");
                iprint(trALWAYS, "\"%s%s\"", (i > 0)? ", " : "", game_args[i]);
            }
            iprintln(trALWAYS, "");
            if (!game_parse_args(game_nr, (const char**)game_args, game_arg_cnt))
                return;
        }
        if (got_start)
        {
            iprintln(trALWAYS, "Starting \"%s\" (%d)", game_name(game_nr), game_nr);
            game_start(game_nr);
        }
        //All remaining actions require a game to be running
        else if (current_game() < 0)
        {
            iprintln(trALWAYS, "No game is currently running. Nothing to %s.", (got_pause) ? "pause" : (got_resume) ? "resume" : "stop");
        }
        //Deal with actions that DON'T require a game number, but might have gotten one
        else if (got_stop)
        {
            //We can stop the game, so we will do it
            iprintln(trALWAYS, "Stopping \"%s\" (%d)", game_name(game_nr), game_nr);
            game_end();
        }
        else if (got_pause)
        {
            //We can pause the game, so we will do it
            iprintln(trALWAYS, "Pausing \"%s\" (%d)", game_name(game_nr), game_nr);
            game_pause();
        }
        else if (got_resume)
        {
            if (!game_is_paused())
            {
                iprintln(trALWAYS, "Game \"%s\" (%d) is not paused. Nothing to resume.", game_name(game_nr), game_nr);
                return; //Nothing to do further
            }
            //We can resume the game, so we will do it
            iprintln(trALWAYS, "Resuming \"%s\" (%d)", game_name(game_nr), game_nr);
            game_resume();
        }
        //All that is left is NO ACTIONS
        else if (game_nr >= 0)
        {
            iprintln(trALWAYS, "Game % 2d - %s", game_nr, game_name(game_nr));
        }
        else
        {
            iprintln(trALWAYS, "List of Games:");
                for (int i = 0; i < games_cnt(); i++)
                    iprintln(trALWAYS, "  % 2d - %s", i, game_name(i));    
        }
        return;
    }

    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        iprintln(trALWAYS, "");
        iprintln(trALWAYS, "Usage: \"game\" -  Displays a list of available games");
        iprintln(trALWAYS, "       \"game <#>\" - Displays info on the selected game");
        iprintln(trALWAYS, "       \"game <#> start [<param_1 ... param_n>]\" - Starts a game");
        iprintln(trALWAYS, "       \"game stop\" - Stops a currently running game");
        iprintln(trALWAYS, "       \"game set [<param_1 ... param_n>]\" - Sets parameters for a game");
        iprintln(trALWAYS, "       \"game pause\" - Pauses a currently running game");
        iprintln(trALWAYS, "       \"game resume\" - Resumes a currently paused game");
        iprintln(trALWAYS, "          <#>:       Game number (0 to %d)", games_cnt() - 1);
        iprintln(trALWAYS, "          <param_x>: Optional game-specific parameters (\"help\" for more info)");
    }
}

#define MAX_RAND_REPEAT             1000
#define MAX_RAND_VALUE_DEFAULT      10000
void _sys_handler_rand(void)
{
    bool help_requested = false;
    int _max = MAX_RAND_VALUE_DEFAULT;
    uint32_t _repeat = 1; //Default to 1 random number
    bool got_max = false;
    bool got_repeat = false;
    bool got_esp = false;
    uint32_t value = 0;

    while (console_arg_cnt() > 0)
	{
        char *arg = console_arg_pop();

        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from while-loop
        }

        if ((!strcasecmp("e", arg)) || (!strcasecmp("esp", arg)))
        {    
            got_esp = true;
            continue; //with while-loop
        }

        if (got_max && got_repeat)
        {
            //If we got both max and repeat, we cannot have any more arguments
            iprintln(trALWAYS, "Ingoring \"%s\"...", arg);
            continue; //From while-loop
        }
        if (str2uint32(&value, arg, 0))
        {
            if (!got_max)
            {
                if ((value < 1) || (value > INT_MAX))
                {
                    iprintln(trALWAYS, "Please specify a valid max value between (1 and %d) (got \"%s\")", INT_MAX, arg);
                    help_requested = true;
                    break; //From while-loop
                }
                _max = (int)value; //We got a valid max value
                got_max = true; //We got a max value
            }
            else if (!got_repeat)
            {
                if ((value < 1) || (value > MAX_RAND_REPEAT))
                {
                    iprintln(trALWAYS, "Please specify a valid repeat count between(1 and %d) (got \"%s\")", MAX_RAND_REPEAT, arg);
                    help_requested = true;
                    break; //From while-loop
                }
                _repeat = (uint32_t)value; //We got a valid repeat count
                got_repeat = true; //We got a repeat count
            }
            continue; //Skip the rest of the loop and go to the next argument
        }

        iprintln(trALWAYS, "Invalid Argument (\"%s\")", arg);
        help_requested = true;
        continue; //Skip the rest of the loop and go to the next argument
    }

    if (!help_requested)
    {
        if (_repeat == 1)
        {
            int random_value;
            if (got_esp) //Use the ESP32's PRNG
                random_value = esp_random() % (_max + 1); //Generate a random number between 0 and max_value
            else//Use the standard C library's rand() function
                random_value = rand() % (_max + 1); //Generate a random number between 0 and max_value

            iprintln(trALWAYS, "Random number between 0 and %d = %d", _max, random_value);
        }
        else
        {
            uint32_t _total = 0;
            iprintln(trALWAYS, "%d Random numbers between 0 and %d:", _repeat, _max);
            for (int i = 0; i < _repeat; i++)
            {
                int random_value;
                if (got_esp) //Use the ESP32's PRNG
                    random_value = esp_random() % (_max + 1); //Generate a random number between 0 and max_value
                else//Use the standard C library's rand() function
                    random_value = rand() % (_max + 1); //Generate a random number between 0 and max_value
                _total += (uint32_t)random_value; //Add the random value to the total
                iprintln(trALWAYS, "%d) %d", i + 1, random_value);
            }
            iprintln(trALWAYS, "");
            iprintln(trALWAYS, "Average: %.3f", _total / (float)_repeat);
        }
    }

    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        iprintln(trALWAYS, "");
        iprintln(trALWAYS, "Usage: \"rand [<max>] [<repeat>] [\"esp\"]\" -  Generates <repeat> random numbers between 0 and <max>");
        iprintln(trALWAYS, "          <max>:    The maximum possible random value (default: %d)", _max);
        iprintln(trALWAYS, "          <repeat>: The number of random values to generate (default: %d)", _repeat);
        iprintln(trALWAYS, "          \"esp\":   Use the ESP32's PRNG (default: std C library)");
    }
}

#endif

#undef PRINTF_TAG
/****************************** END OF FILE **********************************/