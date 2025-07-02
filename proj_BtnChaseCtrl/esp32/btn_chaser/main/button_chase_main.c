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

#include "defines.h"
#include "sys_utils.h"
#include "sys_timers.h"
#include "sys_task_utils.h"
#ifdef CONSOLE_ENABLED
  #include "task_console.h"
#endif
#include "task_rgb_led.h"
#include "task_comms.h"
#include "str_helper.h"


/*******************************************************************************
 Local defines
*******************************************************************************/
#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("Main") /* This must be undefined at the end of the file*/

#define NODE_CMD_CNT_MAX  (10) // The maximum number of commands we can send in a single message

#define CMD_RESPONSE_TIMEOUT_MS (50LL) // The timeout for a command response in ms
#define MAX_NODE_RETRIES         (3) // The maximum number of retries for a node


/*******************************************************************************
 Local structures
*******************************************************************************/
// typedef enum master_state_e
// {
//     idle,          // We are not in the process of registering
//     un_reg,         //We've not been registered yet
//     roll_call,      //We are in the process of responding to a roll-call
//     waiting,        //We are waiting on a registration from the master
//     idle,           //We have been registered with the master (got a bit-mask address)
// }master_state_t;

typedef struct
{
    master_command_t cmd;
    uint8_t *data;
    uint8_t len;
}cmd_data_t;

typedef struct
{
    cmd_data_t cmd_data[NODE_CMD_CNT_MAX]; // The commands we are waiting for a response to
    // master_command_t cmd[NODE_CMD_CNT_MAX]; // The sent command
    // uint8_t *data[NODE_CMD_CNT_MAX]; // The sent command data
    // uint8_t len[NODE_CMD_CNT_MAX]; // The sent command
    //response_code_t resp[NODE_CMD_CNT_MAX]; // The received response code
    uint8_t cnt; // The number of commands in this command list
    uint8_t retry_cnt; // The number of times we have retried sending a message to this node
    uint64_t expiry;
}slave_node_cmd_t;

/* For clarification, a button is JUST a button until it's address is registered, then it is a node on our network.
 Buttons are reference by their address, and nodes are referenced by their slot in the node_list array.
*/

typedef struct
{
    uint8_t             address; // The address of the node
    uint8_t             seq; // The last sequence number received from the node
    uint8_t             flags; // The status flags for the node
    slave_node_cmd_t    responses;//[NODE_CMD_CNT_MAX]; // The last NODE_CMD_CNT_MAX commands, and associated data and responses sent/received
    bool                active; // The node is active (stopwatch running, probably blinking)
    uint32_t            rgb_colour[3]; // The 3 RGB colours of the LED 
    uint32_t            blink_ms; // The blink period (0 if inactive)
}slave_node_t;

slave_node_t    node_list[RGB_BTN_MAX_NODES] = {0}; // The array of slave nodes
uint8_t         rc_response_list[RGB_BTN_MAX_NODES] = {0}; // The response list for the roll-call
int rc_response_cnt = 0; // The number of slave nodes
/*******************************************************************************
 Local function prototypes
 *******************************************************************************/
size_t msg_cmd_response_payload_size(master_command_t cmd, response_code_t resp, size_t data_len_remaining);
void parse_rx_msg(comms_msg_t * msg, size_t payload_len);
const char * msg_cmd_to_str(master_command_t cmd);

void clear_rc_response_list(void);
bool has_button_resonded_to_rc(uint8_t addr);
void clear_nodes(void);
bool is_addr_a_node(uint8_t addr);
bool get_node(uint8_t addr, int *slot);
bool register_node(uint8_t addr, int slot);
void deregister_node(int node);
int node_count(void);
bool next_unregistered_button(uint8_t *addr);
bool next_free_node(int * next_slot);
bool is_node_valid(uint8_t node);
bool get_node_addr(uint8_t node, uint8_t *addr);
bool add_cmd_to_node_msg(uint8_t node, master_command_t cmd, uint8_t *data, uint8_t data_len, bool restart);//, uint64_t timeout_ms); //Timeout in ms, default is 500ms

bool add_node_msg_register(uint8_t node);
bool add_node_msg_new_addr(uint8_t node, uint8_t new_addr);
bool add_node_msg_set_rgb(uint8_t node, uint8_t index, uint32_t rgb_col);
bool add_node_msg_get_rgb(uint8_t node, uint8_t index);
bool add_node_msg_set_blink(uint8_t node, uint32_t period_ms);
bool add_node_msg_get_blink(uint8_t node);
bool add_node_msg_start_sw(uint8_t node);
bool add_node_msg_get_sw_time(uint8_t node);
bool add_node_msg_get_flags(uint8_t node);

void check_all_pending_node_responses(void);
bool node_response_is_pending(int slot);
bool node_response_timeout(int slot);
bool resend_last_node_cmd(int slot);


/*! CONSOLE MENU HANDLER - Performs a system reset
 */
 void _sys_handler_reset(void);
/*! CONSOLE MENU ITEM - Prints the help string for either the command in question, 
 * or for the entire list of commands under the group
 */
void _sys_handler_tasks(void);

void _sys_handler_rollcall(void);
void _sys_handler_reg(void);
void _sys_handler_bcst(void);
void _sys_handler_node(void);
void _sys_handler_list(void);
bool _sys_handler_wait_for_ok_response(uint8_t node, uint64_t timeout);

/*******************************************************************************
 Local variables
 *******************************************************************************/
ConsoleMenuItem_t _task_main_menu_items[] =
{
                                    //01234567890123456789012345678901234567890123456789012345678901234567890123456789
    {"rc",      _sys_handler_rollcall,      "Trigger a rollcall and displays responses after a short wait"},
    {"reg",     _sys_handler_reg,     "Register a node"},
    {"bcst",    _sys_handler_bcst,    "Broadcast a msg to all inactive nodes (no response)"},
    {"node",    _sys_handler_node,    "Sends a message to a specific node (response expected)"},
    {"list",    _sys_handler_list,    "Retrieves a list of all registered nodes"},
    {"tasks",   _sys_handler_tasks,   "Displays the stack usage of all tasks or a specific task"},
    {"reset",   _sys_handler_reset,   "Perform a system reset"},
    {"rtos", 	_sys_handler_tasks,   "Displays RTOS information for each or a specific task"},
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

    clear_rc_response_list();
    clear_nodes(); //Reset all buttons to unregistered state


    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        comms_msg_t rx_msg;
        size_t rx_msg_size;

        if (comms_msg_rx_read(&rx_msg, &rx_msg_size))
            parse_rx_msg(&rx_msg, rx_msg_size - (sizeof(comms_msg_hdr_t) + sizeof(uint8_t))); //We don't need the CRC for processing

        //Check if we have any pending responses that timed out
        check_all_pending_node_responses();

        /* Inspect our own high water mark on entering the task. */
    	//task_main_info.stack_unused = uxTaskGetStackHighWaterMark2( NULL );

		//iprintln(trALWAYS, "#Task Running");

    	//We're not all that busy from this point onwards.... might as well check in every 1000ms.
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
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

size_t msg_cmd_response_payload_size(master_command_t cmd, response_code_t resp, size_t data_len_remaining)
{
    if ((resp == resp_err_payload_len) || (resp == resp_err_range))
    {
        return 1; //1 byte for the length of the payload
    }
    else if ((resp == resp_err_unknown_cmd) || (data_len_remaining <= 2))
    {
        return 0; //Nothing 
    }
    else //if (resp == resp_ok)
    {
        switch (cmd)
        {
            case cmd_roll_call_all:       //= 0x00, /* Requests a Slave Roll-call - ALL    none             none                
            case cmd_roll_call_unreg:     //= 0x01, /* Requests a Slave Roll-call - ONLY   none             none                
            case cmd_set_bitmask_index:   //= 0x08, /* Registers a slave by assigning it   1 byte           none                
            case cmd_bcast_address_mask:  //= 0x02, /* Indicates the indices of            uint32_t         none
            case cmd_set_rgb_0:           //= 0x10, /* Set the primary LED colour          24 bits          none                */
            case cmd_set_rgb_1:           //= 0x11, /* Set the secondary LED colour        24 bits          none                */
            case cmd_set_rgb_2:           //= 0x12, /* Set the 3rd LED colour              24 bits          none                */
            case cmd_set_blink:           //= 0x13, /* Set the blinking interval           uint32_t         none                */
            case cmd_start_sw:            //= 0x14, /* Starts the button stopwatch         none             none                */
            case cmd_set_dbg_led:         //= 0x16, /* Get the debug LED state             none             none              */
            case cmd_new_add:             //= 0x1F, /* Sets a new device address - MUST    1 byte           none                
            case cmd_wr_console_cont:     //= 0x40, /* Write to the slave device console   buffer           none
            case cmd_none:                //= 0xFF, /* Placeholder */
                return 0; //No payload expected for these commands
                break;

            case cmd_get_rgb_0:           //= 0x20, /* Get the primary LED colour          none             24 bits             */
            case cmd_get_rgb_1:           //= 0x21, /* Get the secondary LED colour        none             24 bits             */
            case cmd_get_rgb_2:           //= 0x22, /* Get the 3rd LED colour              none             24 bits             */
                return 3 * sizeof(uint8_t); //3 bytes for the RGB colour
                break;
            
            case cmd_get_blink:           //= 0x23, /* Get the blinking interval           none             uint32_t            */
            case cmd_get_sw_time:         //= 0x24, /* Requests the elapsed time of the    none             4 bytes             
                return sizeof(uint32_t); //4 bytes for the blink period
                break;
                
            case cmd_get_flags:           //= 0x25, /* Requests the system flags           none             1 bytes             */
            case cmd_get_dbg_led:         //= 0x26, /* Requests the debug LED state        none             1 byte              */
                return sizeof(uint8_t); //1 byte for the flags                
                break;

            case cmd_wr_console_done:     //= 0x41, /* Last part of a write to the slave   buffer           expects a potentially 
            case cmd_debug_0:             //= 0x80, /* Dummy command. just fills the data buffer */
            default:
                return data_len_remaining - 2; //the remainder of data in the message (minus the 2 bytes for the response code and command)
                break;
        }
    }
}

void parse_rx_msg(comms_msg_t * msg, size_t payload_len)
{
    int _data_idx = 0;
    int _cmd_idx = 0;
    master_command_t _cmd;
    response_code_t _resp;
    uint8_t * _resp_data;
    uint8_t _resp_data_len;
    comms_msg_t rx_msg;
    size_t rx_msg_size;

    //start at the beginning of the data
    do
    {
        _cmd = (master_command_t) msg->data[_data_idx++];
        _resp = (response_code_t)msg->data[_data_idx++]; //The 2nd byte is the response code
        _resp_data = &msg->data[_data_idx]; //The rest of the data is the response data
        _resp_data_len = msg_cmd_response_payload_size(_cmd, _resp, payload_len - _data_idx); //The rest of the payload is the data
        _data_idx += _resp_data_len; //Move the data index forward by the command data length

        //We are expecting response messages in only 2 instances:
        // 1) Rollcall responses
        // 2) Responses to commands sent to an ACTIVE node
        if ((_cmd == cmd_roll_call_all) || (_cmd == cmd_roll_call_unreg))
        {
            //Add this address to the response list
            for (int i = 0; i < RGB_BTN_MAX_NODES; i++)
            {
                //Find the first empty slot
                if (rc_response_list[i] == msg->hdr.src) 
                {
                    //This address is already in the list (only possible with "cmd_roll_call_unreg"), so what happened?
                    /* It could be that this node has reset and is now responding to a roll-call again, (more likely, since it starting up in an unregsistered state)
                       or it could be that a new node has joined the network with the same address as a previous node (less likely)

                       I guess the best course of action is to double check if we have comms with the node using this address.... if so, then this is a new button that has joined the network.
                        Otherwise we can assume that this is a node that has reset and is now responding to a roll-call again.

                        For now we are just going to assume the more likely scenario...
                    */
                    int node_slot = -1;
                    if (get_node(msg->hdr.src, &node_slot))
                    {
                        int node_slot = -1;
                        if (get_node(msg->hdr.src, &node_slot))
                        //This address is already registered as a node, so let's delete that node
                        iprintln(trAPP, "#Address 0x%02X already registered as a node (%d)", msg->hdr.src, i);
                        deregister_node(node_slot); //Deregister the node
                        //RVN - TODO For safety, should we consider sending a "deregister" command to the node?
                    }
                    return; //No need to continue, we found the address
                }
                if (rc_response_list[i] == 0) 
                {
                    rc_response_list[i] = msg->hdr.src;
                    rc_response_cnt++;
                    iprintln(trAPP, "#Got RC Reply #%d from 0x%02X (%d)", rc_response_cnt, msg->hdr.src, i);
                    return; //We found a slot, no need to continue
                }
                //else, this slot belongs to another button, so continue searching
            }
            iprintln(trAPP, "#No space for 0x%02X", msg->hdr.src);
            return; //No space for this address in the response list
        }
        else
        {
            slave_node_t *node = NULL;
            int node_slot = -1;
            if (get_node(msg->hdr.src, &node_slot))
            {
                node = &node_list[node_slot]; //Get the node from the list

                //Does this node have cmds waiting on responses
                if (node->responses.cnt > 0)
                {
                    //We should be getting the responses in the same order as we sent the commands
                    if (node->responses.cmd_data[0].cmd != _cmd)
                    {
                        //Oops... this is not right?!?!?!
                        iprintln(trAPP, "#Error: Node 0x%02X sent response (0x%02X) for \"%s\" but we expected \"%s\"", 
                                 msg->hdr.src, _resp, msg_cmd_to_str(_cmd), msg_cmd_to_str(node->responses.cmd_data[0].cmd));
                        continue; //Skip this response, we can't handle it
                    }
                    if (_resp != resp_ok)
                    {
                        //We got an error response, so we can handle it
                        iprintln(trAPP, "#Node 0x%02X sent error response (0x%02X) for \"%s\"", msg->hdr.src, _resp, msg_cmd_to_str(_cmd));
                        if (_resp_data_len > 0)
                        {
                            iprint(trAPP, "# Additional data: ");
                            for (int i = 0; i < _resp_data_len; i++)
                                iprintf("%02X ", _resp_data[i]);
                            iprintln(trAPP, "");
                        }
                    }
                    else
                    {
                        //We got an OK response, so we can drop that command from the list
                        // iprintln(trAPP, "#OK rx'd for \"%s\" from 0x%02X (%d - %d)", msg_cmd_to_str(_cmd), msg->hdr.src, node_slot, node->responses.cnt);
                        if (node->responses.cnt > 1)//Delete the entry at index 0 and move the rest of the entries down
                            memmove(&node->responses.cmd_data[0], &node->responses.cmd_data[1], (node->responses.cnt - 1) * sizeof(cmd_data_t));

                        memset(&node->responses.cmd_data[node->responses.cnt - 1], 0, sizeof(cmd_data_t)); //Reset the last entry

                        node->responses.cnt--;
                        if (node->responses.cnt == 0)
                        {
                            node->responses.retry_cnt = 0; //Reset the retry count
                            node->responses.expiry = 0; //Reset the expiry time
                        }
                    }
                }
            }

            //Response to a command sent directly to a node, but we are not waiting for a response?????
            else 
            {
                if (_resp == resp_ok)
                {
                    iprintln(trAPP, "#UNSOLICITED OK rx'd for \"%s\", (Node Address 0x%02X)", msg_cmd_to_str(_cmd), msg->hdr.src);
                }
                else
                {
                    iprintln(trAPP, "#UNSOLICITED Error %d rx'd for \"%s\", (Node Address 0x%02X)", _resp, msg_cmd_to_str(_cmd), msg->hdr.src);
                    //We can handle the error here, but we don't have to
                    //We can also just ignore it
                }
            }
        }
        _cmd_idx++;
    } while (_data_idx< payload_len);
}

const char * msg_cmd_to_str(master_command_t cmd)
{
    switch (cmd)
    {
        case cmd_roll_call_all:         return "roll_call_all";
        case cmd_roll_call_unreg:       return "roll_call_unreg";
        case cmd_set_bitmask_index:     return "set_bitmask_index";
        case cmd_bcast_address_mask:    return "bcast_address_mask";
        case cmd_set_rgb_0:             return "set_rgb_0";
        case cmd_set_rgb_1:             return "set_rgb_1";
        case cmd_set_rgb_2:             return "set_rgb_2";
        case cmd_set_blink:             return "set_blink";
        case cmd_start_sw:              return "start_sw";
        case cmd_set_dbg_led:           return "set_dbg_led";
        case cmd_new_add:               return "new_add";
        case cmd_get_rgb_0:             return "get_rgb_0";
        case cmd_get_rgb_1:             return "get_rgb_1";
        case cmd_get_rgb_2:             return "get_rgb_2";
        case cmd_get_blink:             return "get_blink";
        case cmd_get_sw_time:           return "get_sw_time";
        case cmd_get_flags:             return "get_flags";
        case cmd_get_dbg_led:           return "get_dbg_led";
        case cmd_wr_console_cont:       return "wr_console_cont";
        case cmd_wr_console_done:       return "wr_console_done";
        case cmd_debug_0:               return "debug_0";
        case cmd_none:                  return "none";
        default:                        return "unknown";
    }
}

void clear_rc_response_list(void)
{
    memset(rc_response_list, 0, sizeof(rc_response_list));
    rc_response_cnt = 0;
}

bool has_button_resonded_to_rc(uint8_t addr)
{
    for (int i = 0; i < RGB_BTN_MAX_NODES; i++)
    {
        if (rc_response_list[i] == addr)
            return true;
    }
    return false; //Address not found in the response list
}

void clear_nodes(void)
{
    memset(node_list, 0, sizeof(node_list)); //Reset all buttons to unregistered state
}

bool is_addr_a_node(uint8_t addr)
{
    for (int i = 0; i < RGB_BTN_MAX_NODES; i++)
    {
        if (node_list[i].address == addr)
            return true; //Found the address in the registered list
    }
    return false; //Address not found in the registered list
}

bool get_node(uint8_t addr, int *slot)
{
    for (int i = 0; i < RGB_BTN_MAX_NODES; i++)
    {
        if (node_list[i].address == addr)
        {
            if (slot != NULL)
                *slot = i; //Return the slot index if requested
            return true;
        }
    }
    return false; //Address not found in the registered list
}

bool register_node(uint8_t addr, int slot)
{
    if (slot < 0 || slot >= RGB_BTN_MAX_NODES)
    {
        iprintln(trALWAYS, "Invalid slot %d for button 0x%02X", slot, addr);
        return false;
    }

    if (node_list[slot].address != 0) 
    {
        if (node_list[slot].address != addr)
        {
            //This slot is already in use for a different button
            iprintln(trALWAYS, "Slot %d is already in use for button 0x%02X", slot, node_list[slot].address);
            return false;
        }
        //The slot is already registered with the same address, so we can fall through without clearing existing information
    }
    else //if (node_list[slot].address == 0)
    {
        //This slot is not in use, so we can register the button here
        memset(&node_list[slot], 0, sizeof(slave_node_t)); //Reset the slot to zero
        node_list[slot].address = addr;
    }

    //iprintln(trALWAYS, "Registered button 0x%02X at slot %d", addr, slot);
    return true;
}

void deregister_node(int node)
{
    if (!is_node_valid(node))
        return; //No button registered at this slot

    iprintln(trALWAYS, "Deregistering node %d (0x%02X)", node, node_list[node].address);
    memset(&node_list[node], 0, sizeof(slave_node_t)); //Reset the slot to zero
}

int node_count(void)
{
    int count = 0;
    for (int i = 0; i < RGB_BTN_MAX_NODES; i++)
    {
        if (node_list[i].address > 0) //This slot is in use
            count++; //Count the number of registered buttons
    }
    return count;
}

bool next_unregistered_button(uint8_t *addr)
{
    for (int i = 0; i < rc_response_cnt; i++)
    {
        if (!is_addr_a_node(rc_response_list[i]))
        {
            *addr = rc_response_list[i];
            return true;
        }
    }
    return false;
}

bool next_free_node(int * next_slot)
{
    for (int i = 0; i < RGB_BTN_MAX_NODES; i++)
    {
        if (node_list[i].address == 0) //This slot is not in use
        {
            *next_slot = i;
            return true; //Found a free slot
        }
    }
    return false; //Could not find a free slot
}

bool is_node_valid(uint8_t node)
{
    if (node >= RGB_BTN_MAX_NODES)
    {
        iprintln(trALWAYS, "Invalid slot %d for getting node address (%d/%d in use)", node, node_count(), RGB_BTN_MAX_NODES);
        return false;
    }
    if (node_list[node].address == 0)
    {
        // iprintln(trALWAYS, "No button registered at slot %d", slot);
        return false;
    }
    if ((node_list[node].address == ADDR_BROADCAST)  || (node_list[node].address == ADDR_MASTER))
    {
        iprintln(trCOMMS, "#Invalid node address, 0x%02X, at node %d... deleting!", node_list[node].address, node);
        //This should be de-registered immediately, since it is not a valid node address
        memset(&node_list[node], 0, sizeof(slave_node_t)); //Reset the slot to zero
        return false;
    }
    return true; //The slot is valid and has a registered button
}

bool get_node_addr(uint8_t node, uint8_t *addr)
{
    if (!is_node_valid(node))
        return false;

    *addr = node_list[node].address;
    return true;
}

void init_cmd_to_node_msg(uint8_t node, bool reset_response_data)
{
    if (!is_node_valid(node))
        return;

    comms_node_msg_init(node_list[node].address); //Initialize the message for this node

    if (!reset_response_data)
        return; //We don't want to reset the response data, so we can just return

    node_list[node].responses.cnt = 0; //Reset the command count for this node
    node_list[node].responses.expiry = 0; //Reset the expiry time
    node_list[node].responses.retry_cnt = 0; //Reset the retry count
    memset(node_list[node].responses.cmd_data, 0, sizeof(node_list[node].responses.cmd_data)); //Reset the command data list
}

bool add_cmd_to_node_msg(uint8_t node, master_command_t cmd, uint8_t *data, uint8_t data_len, bool restart)//, uint64_t timeout_ms)
{
    uint8_t node_addr;
    
    if (!get_node_addr(node, &node_addr))
        return false; //No button registered at this slot

    if (comms_node_msg_append(node_addr, cmd, data, data_len, restart))
    {
        int i = node_list[node].responses.cnt;
        //Found an empty slot
        node_list[node].responses.cmd_data[i].cmd = cmd; //Store the command we are waiting for
        node_list[node].responses.cmd_data[i].data = data; //No data for this command
        node_list[node].responses.cmd_data[i].len = data_len; //No data length for this command
        //node_list[slot].responses.resp[i] = resp_err_none; //No response yet
        node_list[node].responses.cnt++; //Increment the command count for this node

        node_list[node].responses.expiry = (uint64_t)sys_poll_tmr_ms() + CMD_RESPONSE_TIMEOUT_MS;//timeout_ms; //Store the timestamp of when we sent the command
        node_list[node].responses.retry_cnt = 0;
        //if we run out of space.... well, heck, then we will not wait for that response...
        return true; //Command added successfully
    }
    iprintln(trCOMMS, "#Failed to append command %s (0x%02X) for node %d (0x%02X)", msg_cmd_to_str(cmd), cmd, node, node_addr);
    return false; //Failed to append the command
}

bool add_node_msg_register(uint8_t node)
{
    return add_cmd_to_node_msg(node, cmd_set_bitmask_index, &node, sizeof(uint8_t), true);
}

bool add_node_msg_new_addr(uint8_t node, uint8_t new_addr)
{
    uint8_t node_addr;
    
    if (!get_node_addr(node, &node_addr))
        return false; //No button registered at this slot

    //Maybe not a good idea to allow the new address to be the same as the old one?
    if (new_addr == node_addr)
    {
        iprintln(trCOMMS, "#New address is the same as the old address for node %d (0x%02X)", node, node_addr);
        return true; //No need to send a command, the address is already set
    }

    return add_cmd_to_node_msg(node, cmd_new_add, &new_addr, sizeof(uint8_t), true);
}

bool add_node_msg_set_rgb(uint8_t node, uint8_t index, uint32_t rgb_col)
{
    if (index > 2)
    {
        iprintln(trCOMMS, "#Invalid RGB index (%d)", index);
        return false;
    }
    return add_cmd_to_node_msg(node, cmd_set_rgb_0 + index, (uint8_t *)&rgb_col, (3*sizeof(uint8_t)), false);
}
bool add_node_msg_get_rgb(uint8_t node, uint8_t index)
{
    if (index > 2)
    {
        iprintln(trCOMMS, "#Invalid RGB index (%d)", index);
        return false;
    }
    return add_cmd_to_node_msg(node, cmd_get_rgb_0 + index, NULL, 0, false);
}

bool add_node_msg_set_blink(uint8_t node, uint32_t period_ms)
{
    return add_cmd_to_node_msg(node, cmd_set_blink, (uint8_t *)&period_ms, sizeof(uint32_t), false);
}
bool add_node_msg_get_blink(uint8_t node)
{
    return add_cmd_to_node_msg(node, cmd_get_blink, NULL, 0, false);
}

bool add_node_msg_start_sw(uint8_t node)
{
    return add_cmd_to_node_msg(node, cmd_start_sw, NULL, 0, false);
}
bool add_node_msg_get_sw_time(uint8_t node)
{
    return add_cmd_to_node_msg(node, cmd_get_sw_time, NULL, 0, false);
}

bool add_node_msg_get_flags(uint8_t node)
{
    return add_cmd_to_node_msg(node, cmd_get_flags, NULL, 0, false);
}

void check_all_pending_node_responses(void)
{
    //Check if we have any pending responses that timed out
    for (int slot = 0; slot < RGB_BTN_MAX_NODES; slot++)
    {
        if (!node_response_timeout(slot))
            continue;

        if (!resend_last_node_cmd(slot))
            deregister_node(slot); //Deregister the node
    }
}

bool node_response_is_pending(int slot)
{
    if (!is_node_valid(slot)) //Check if the slot is valid and has a registered button
        return false; //Skip this slot

    if (node_list[slot].address == 0) //This slot is not in use
        return false; //Skip this slot

    if (node_list[slot].responses.cnt == 0) //No pending responses for this node
        return false; //Skip this slot

    if (node_list[slot].responses.expiry == 0)
        return false; //No timeout (yet)

    return true;
}

bool node_response_timeout(int slot)
{
    if (!node_response_is_pending(slot))
        return false;
    
        //Check if the response has timed out
    if (node_list[slot].responses.expiry > sys_poll_tmr_ms())
        return false; //No timeout (yet)

    iprintln(trAPP, "#Response timeout for 0x%02X (%d):", node_list[slot].address, slot);
    for (int j = 0; j < node_list[slot].responses.cnt; j++)
        iprintln(trAPP, "  - \"%s\" (%d bytes)", msg_cmd_to_str(node_list[slot].responses.cmd_data[j].cmd), node_list[slot].responses.cmd_data[j].len);

    return true; //Response has timed out
}

bool resend_last_node_cmd(int slot)
{
    //So now what, do we resend this command?
    if (node_list[slot].responses.retry_cnt >= MAX_NODE_RETRIES) //If we have retried this command more than 3 times, we give up
        return false; //Give up on this node

    iprintln(trAPP, "#Resending last command to node 0x%02X (%d)", node_list[slot].address, slot);

    //Resend the command
    node_list[slot].responses.retry_cnt++; //Increment the retry count
    init_cmd_to_node_msg((uint8_t)slot, false);
    for (int j = 0; j < node_list[slot].responses.cnt; j++)
    {
        if (!comms_node_msg_append(node_list[slot].address, node_list[slot].responses.cmd_data[j].cmd, node_list[slot].responses.cmd_data[j].data, node_list[slot].responses.cmd_data[j].len, false))
        {
            //RVN - TODO - Not liking how I am handling this failure.... but what the heck am I supposed to do.... another retry counter for this as well?
            iprintln(trAPP, "#Error: Could not reload \"%s\" (%d bytes) to node %d (0x%02X) during resend", msg_cmd_to_str(node_list[slot].responses.cmd_data[j].cmd), node_list[slot].responses.cmd_data[j].len, slot, node_list[slot].address);
            return false; //Failed to append the command, so we cannot resend it
        }
    }
    comms_msg_tx_now(); //Send the message immediately
    node_list[slot].responses.expiry = sys_poll_tmr_ms() + CMD_RESPONSE_TIMEOUT_MS;
    return true; //Command resent successfully
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

void _sys_handler_rollcall(void)
{
    //These functions (_menu_handler....) are called from the console task, so they should 
    // not send messages directly on the RS485 bus, but instead send messages to the Comms 
    // task via the msg_queue, using _tx_now()
    bool help_requested = false;
    bool reset_all = false;

    while (console_arg_cnt() > 0)
	{
        char *arg = console_arg_pop();
        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from while-loop
        }
        if ((!strcasecmp("f", arg)) || (!strcasecmp("force", arg)))
        {    
            reset_all = true;
            continue; //with while-loop
        }

        iprintln(trALWAYS, "Invalid Argument (\"%s\")", arg);
        help_requested = true;
        continue; //Skip the rest of the loop and go to the next argument
    }

    if (!help_requested)
    {
        if (comms_rollcall(reset_all))
        {
            iprintln(trALWAYS, "Sent rollcall to %s nodes", (reset_all)? "all" : "unregistered");
            if (reset_all)
            {
                clear_rc_response_list();
                clear_nodes(); //Reset all buttons to unregistered state
            }
        }
        else
            iprintln(trALWAYS, "Failed to send rollcall message");
    }

    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        iprintln(trALWAYS, "Usage: \"rc [f|force]\" - Sends a rollcall request on the RS485 bus");        
        iprintln(trALWAYS, "    \"f\"|\"force\":   forces all receiving nodes into an unregistered state");
        iprintln(trALWAYS, "            otherwise (if not specified) only UNregistered nodes will respond");
    }
}

void _sys_handler_reg(void)
{
    //These functions (_menu_handler....) are called from the console task, so they should 
    // not send messages directly on the RS485 bus, but instead send messages to the Comms 
    // task via the msg_queue, using _tx_now()
    bool help_requested = false;
    bool reg_all = false;
    uint8_t _addr = 0x00;
    int _slot = -1;
    int valid_args = 0;
    uint32_t value;
    bool got_arg;

    while (console_arg_cnt() > 0)
	{
        got_arg = false;
        char *arg = console_arg_pop();
        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from while-loop
        }
        if (strcasecmp("all", arg) == 0)
        {
            reg_all = true;
            continue;;
        }

        if ((valid_args == 0) && (hex2u32(&value, arg, 2)))
            got_arg = true;

        if ((valid_args == 1) && (str2int32((int32_t*)&value, arg, 0)))
            got_arg = true;

        if ((valid_args >= 2) || (reg_all)) //If we have more than 2 arguments, or if we are registering all nodes
        {
            valid_args++; //Just count the arguments, but don't do anything with them
            continue;
        }


        if (got_arg)
        {
            valid_args++;
            //First argument is the address, second is the slot #
            if (valid_args == 1)
            {
                _addr = (uint8_t)value;
                continue;
            }
            else //if (valid_args >= 2)
            {
                _slot = (int)value;
                break;
            }
        }
        iprintln(trALWAYS, "Invalid Argument (\"%s\")", arg);
        help_requested = true;
        continue; //Skip the rest of the loop and go to the next argument
    }

    if ((reg_all) && (valid_args > 0))
    {
        iprintln(trALWAYS, "No additional arguments expected for \"reg all\"");
        help_requested = true;
    }
    if (valid_args > 2)
    {
        iprintln(trALWAYS, "Too many arguments (%d) for registration (expected 0, 1 or 2)", valid_args);
        help_requested = true;
    }

    if (!help_requested)
    {
        if (reg_all)
        {
            int reg_cnt = 0;
            //Register all unregistered nodes
            //register_all_unregistered_nodes();
            for (int i = 0; i < rc_response_cnt; i++)
            {
                if (is_addr_a_node(rc_response_list[i]))
                    continue; //Skip this address, it is already registered
                _addr = rc_response_list[i];
                //Find the next free slot for this address
                if (!next_free_node(&_slot))
                {
                    iprintln(trALWAYS, "No Slot available for 0x%02X", _addr);
                    continue; //Skip this address, we cannot register it
                }
                if (register_node((uint8_t)_addr, _slot))
                {
                    init_cmd_to_node_msg((uint8_t)_slot, true);
                    if (add_node_msg_register((uint8_t)_slot))
                    {
                        comms_msg_tx_now(); //Send the message immediately
                        if (_sys_handler_wait_for_ok_response((uint8_t)_slot, 1000LL))
                            iprintln(trALWAYS, "Registered node %d (0x%02X)", _slot, (uint8_t)_addr);
                        else
                            iprintln(trALWAYS, "Failed to register node 0x%02X in slot %d", (uint8_t)_addr, _slot);
                    }
                }
            }
            iprintln(trALWAYS, "Registered %d/%d nodes", node_count(), rc_response_cnt);
            return;
        }
        //Argument Validation done here....
        if (valid_args < 1)
        {
            //No arguments given.... Get the next unregistered button address
            if (!next_unregistered_button(&_addr)) 
            {
                //No unregistered nodes found, but we have a roll-call response list
                if (rc_response_cnt > 0)
                    iprintln(trALWAYS, "All RC nodes are registered (%d/%d)", node_count(), rc_response_cnt);
                // No roll-call response list, so we cannot register any nodes
                else
                    iprintln(trALWAYS, "No unregistered nodes found. Please run \"rc [f|force]\" first.");
                return; //Nothing to do further
            }
            valid_args++;
        }
        if (valid_args < 2)
        {
            //We have an address, but no slot specified...Find the next available slot
            if (!next_free_node(&_slot))
            {
                iprintln(trALWAYS, "No Slot available for 0x%02X", _addr);
                return; //Nothing to do further
            }
            valid_args++;
        }

        if (_addr == ADDR_MASTER)
        {
            iprintln(trALWAYS, "Cannot register MASTER address (0x%02X)", _addr);
            return; //Nothing to do further
        }
        if (_addr == ADDR_BROADCAST)
        {
            iprintln(trALWAYS, "Cannot register BROADCAST address (0x%02X)", _addr);
            return; //Nothing to do further
        }
        if (rc_response_cnt == 0)
        {
            iprintln(trALWAYS, "No roll-call response received yet. Please run \"rc [f|force]\" first.");
            return; //Nothing to do further
        }
        
        //Is this addres in the response list we got from the roll-call?
        if (!has_button_resonded_to_rc(_addr))
        {
            if (node_count() >= rc_response_cnt)
            {
                iprintln(trALWAYS, "All RC nodes are registered (%d/%d)", node_count(), rc_response_cnt);
                return;
            }

            if (is_addr_a_node(_addr))
                iprintln(trALWAYS, "Node 0x%02X is already registered", _addr);
            else
                iprintln(trALWAYS, "Address 0x%02X not found in roll-call response list", _addr);

            iprint(trALWAYS, " Select from available nodes (%d/%d)", node_count(), rc_response_cnt);
            iprint(trALWAYS, "   [");
            bool first = false;
            for (int i = 0; i < rc_response_cnt; i++)
            {
                if (is_addr_a_node(rc_response_list[i]))
                    continue; //Skip registered nodes
                iprint(trALWAYS, "%s0x%02X", (first) ? ", " : "", rc_response_list[i]);
                first = true;
            }
            iprintln(trALWAYS, "]");
            return; //Nothing to do further
        }

        // Make sure the given slot # is within allowed range
        if (_slot > RGB_BTN_MAX_NODES) 
        {
            iprintln(trALWAYS, "Slot # must be < %d (got %d)", RGB_BTN_MAX_NODES+1, _slot);
            return; //Nothing to do further
        }
        //And that it is not in use already
        if (node_list[_slot].address != 0)
        {
            iprintln(trALWAYS, "Invalid slot (%d) selected for 0x%02X, already in use by node 0x%02X", _slot, _addr, node_list[_slot].address);
            iprint(trALWAYS, " Select from: [");
            bool first = false;
            for (int i = 0; i < RGB_BTN_MAX_NODES; i++)
            {
                if (node_list[i].address == 0) //This slot is not in use
                {
                    iprint(trALWAYS, "%s%d", (first) ? ", " : "", i);
                    first = true;
                }
            }
            iprintln(trALWAYS, "]");
            return; //Nothing to do further
        }

        //LEt's assume this registration is going to be successful and register the button in the button array
        if (register_node((uint8_t)_addr, _slot))
        {
            init_cmd_to_node_msg((uint8_t)_slot, true);
            if (add_node_msg_register((uint8_t)_slot))
            {
                comms_msg_tx_now(); //Send the message immediately
                //Wait for the response from the node
                if (_sys_handler_wait_for_ok_response((uint8_t)_slot, 1000LL))
                {
                    iprintln(trALWAYS, "Registered node %d (0x%02X)", _slot, (uint8_t)_addr);
                    return;
                }
            }
        }
        iprintln(trALWAYS, "Failed to register node 0x%02X in slot %d");
    }

    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        iprintln(trALWAYS, "Usage: \"reg [<address>] [<slot>]\" - Registers a button as a node");        
        iprintln(trALWAYS, "       \"reg all\" - Registers ALL available buttons as nodes automatically");
        iprintln(trALWAYS, "    <address>:  Hex address for the button to be registered (0x%02X to 0x%02X)", ADDR_SLAVE_MIN, ADDR_SLAVE_MAX);
        iprintln(trALWAYS, "        If omitted, the next available button will be registered automatically");        
        iprintln(trALWAYS, "    <slot>:     Slot number to assign this node to (0 to %d)", RGB_BTN_MAX_NODES);
        iprintln(trALWAYS, "        If omitted, the next available slot is assigned automatically");
    }
}

bool _sys_handler_wait_for_ok_response(uint8_t node, uint64_t timeout)
{
    //it is important that this function is only called from the _sys_handler_* functions, as it will then run in (and block) the console console task until the response is received or the timeout expires.
    //The responses are handled in the main APP task, so this should NOT be called from the APP task, as it will block the console task and not allow the APP task to process the responses.

    uint64_t expiry_time = sys_poll_tmr_ms() + timeout;

    //iprintln(trALWAYS, "Waiting for response from 0x%02X (%d cmd%s)", node_list[node].address, node_list[node].responses.cnt, (node_list[node].responses.cnt > 1) ? "s" : "");
    while (node_response_is_pending(node))
    {
        if (sys_poll_tmr_ms() > expiry_time)
        {
            iprintln(trALWAYS, "#Timeout waiting for response from 0x%02X (%d cmd%s)", node_list[node].address, node_list[node].responses.cnt, (node_list[node].responses.cnt > 1) ? "s" : "");
            return false; //Timeout expired, no response received
        }
        //Just wait here....
        vTaskDelay(1); //Wait a bit before checking again
    }
    return true; //Response received
}

void _sys_handler_bcst(void)
{
    iprintln(trALWAYS, "NOT IMPLEMENTED YET");
}

void _sys_handler_node(void)
{
    iprintln(trALWAYS, "NOT IMPLEMENTED YET");
}

void _sys_handler_list(void)
{
    iprintln(trALWAYS, "NOT IMPLEMENTED YET");
}

#endif

#undef PRINTF_TAG
/****************************** END OF FILE **********************************/