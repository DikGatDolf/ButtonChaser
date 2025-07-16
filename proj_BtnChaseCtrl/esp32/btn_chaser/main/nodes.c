/*******************************************************************************
Module:     nodes.c
Purpose:    This file contains the _______ functions
Author:     Rudolph van Niekerk


For clarification, a button is JUST a button until it's address is registered, 
 then it is a NODE on our network.
Buttons are reference by their address, and NODES are referenced by their 
  slot #in the node_list array.
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

#define __NOT_EXTERN__
#include "nodes.h"
#undef __NOT_EXTERN__
/*******************************************************************************
Macros and Constants
 *******************************************************************************/
#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("Nodes") /* This must be undefined at the end of the file*/

/*******************************************************************************
local defines 
 *******************************************************************************/

#define CMD_RESPONSE_TIMEOUT_MS (50LL) // The timeout for a command response in ms

#define MAX_NODE_RETRIES         (3) // The maximum number of retries for a node

/*******************************************************************************
 Local structure
 *******************************************************************************/
typedef struct
{
    uint8_t     list[RGB_BTN_MAX_NODES]; // The response list for the roll-call
    int         cnt; // The number of slave nodes
    Timer_ms_t  timer;
}rollcall_t;

typedef struct
{
    master_command_t cmd;
    uint8_t *data;
    uint8_t len;
}cmd_data_t;

typedef struct
{
    cmd_data_t cmd_data[NODE_CMD_CNT_MAX]; // The commands we are waiting for a response to
    uint8_t cnt; // The number of commands in this command list
    uint8_t retry_cnt; // The number of times we have retried sending a message to this node
    uint64_t expiry;
}slave_node_cmd_t;

typedef struct
{
    uint8_t             address; // The address of the node
    uint8_t             seq; // The last sequence number received from the node
    uint8_t             flags; // The status flags for the node
    slave_node_cmd_t    responses;//[NODE_CMD_CNT_MAX]; // The last NODE_CMD_CNT_MAX commands, and associated data and responses sent/received
    bool                active; // The node is active (stopwatch running, probably blinking... but most importantly, it should not get broadcast messages)
    button_t            btn; // The button data for this node
    uint64_t            last_update_time; // The last time we have updated this node's data
    comms_tx_msg_t      msg; // The message we are currently building to send to this node
}slave_node_t;

/*******************************************************************************
 Local function prototypes
 *******************************************************************************/
int rc_response_count(void);
int add_rc_address(uint8_t addr);

bool is_addr_registered(uint8_t addr);

bool _get_adress_node_index(uint8_t addr, int *slot);
void _deregister_node(int node);

bool _add_cmd_to_node_msg(uint8_t node, master_command_t cmd, uint8_t *data, uint8_t data_len, bool restart);
bool _bcst_append(uint8_t cmd, uint8_t * data, uint8_t data_len);
void _rollcall_handler(uint8_t addr);

void _check_all_pending_node_responses(void);
void _response_handler(int slot, master_command_t resp_cmd, response_code_t resp, uint8_t *resp_data, size_t resp_data_len);
size_t _response_payload_size(master_command_t cmd, response_code_t resp, size_t data_len_remaining);
int _responses_pending(int slot);
bool _resend_unresponsive_cmds(int slot);
master_command_t _next_expected_response(int slot);

uint8_t _register_addr(uint8_t addr);

uint32_t _inactive_nodes_mask(void);


/*******************************************************************************
 Local variables
 *******************************************************************************/

slave_node_t    node_list[RGB_BTN_MAX_NODES] = {0}; 
comms_tx_msg_t bcst_msg = {0};

rollcall_t rollcall = {0}; // The structure containing information for all who respond on rollcalls

//RVN - Technically I  should maintain a separate stopwatch for each node, but 
//  holy crap that is adding sooooooooo much more complexity (e.g. a sw is 
//  started for a node and stopped using a broadcast.... or vice versa... 
// .... Aaaaaaargh! Can open..... worms everywhere!)
Stopwatch_ms_t sync_stopwatch;
/*******************************************************************************
 Local (private) Functions
 *******************************************************************************/
bool _get_adress_node_index(uint8_t addr, int *slot)
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

bool is_node_valid(uint8_t node)
{
    if (node >= RGB_BTN_MAX_NODES)
    {
        iprintln(trNODE, "#Invalid slot %d for getting node address (%d/%d in use)", node, node_count(), RGB_BTN_MAX_NODES);
        return false;
    }
    if (node_list[node].address == 0)
    {
        // iprintln(trALWAYS, "No button registered at slot %d", slot);
        return false;
    }
    if ((node_list[node].address == ADDR_BROADCAST)  || (node_list[node].address == ADDR_MASTER))
    {
        iprintln(trNODE, "#Invalid node address, 0x%02X, at node %d... deleting!", node_list[node].address, node);
        //This should be de-registered immediately, since it is not a valid node address
        memset(&node_list[node], 0, sizeof(slave_node_t)); //Reset the slot to zero
        return false;
    }
    return true; //The slot is valid and has a registered button
}

bool _add_cmd_to_node_msg(uint8_t node, master_command_t cmd, uint8_t *data, uint8_t data_len, bool restart)//, uint64_t timeout_ms)
{
    uint8_t node_addr;
  
    if (!is_node_valid(node))
        return false; //No button registered at this slot

    node_addr = get_node_addr(node);

    if (comms_tx_msg_append(&node_list[node].msg, node_addr, cmd, data, data_len, restart))
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
    iprintln(trNODE, "#Failed to append command %s (0x%02X) for node %d (0x%02X)", cmd_to_str(cmd), cmd, node, node_addr);
    return false; //Failed to append the command
}

bool _bcst_append(uint8_t cmd, uint8_t * data, uint8_t data_len)
{
    //Must be preceeded by a comms_bcst_start() command
    if ((bcst_msg.data_length == 0) ||                       /* There should at least be a cmd_bcast_address_mask in the data */ 
        (bcst_msg.msg.hdr.dst != ADDR_BROADCAST) ||          /* Destination MUST be broadcast address */
        (bcst_msg.msg.data[0] != cmd_bcast_address_mask))    /* 1st cmd in the data array should be cmd_bcast_address_mask */
    {
        iprintln(trNODE, "#BCST message NOT initialised (len = %d, dst = 0x%02X, 1st cmd = 0x%02X)", bcst_msg.data_length, bcst_msg.msg.hdr.dst, bcst_msg.msg.data[0]);
        return false;
    }

    if (cmd >= cmd_set_bitmask_index) //If this is a command that can only be sent to a node, then we cannot append it to the broadcast message
    {
        iprintln(trNODE, "#Cannot append command %s (0x%02X) to broadcast message", cmd_to_str(cmd), cmd);
        return false; //Cannot append this command to the broadcast message
    }

    return comms_tx_msg_append(&bcst_msg, ADDR_BROADCAST, cmd, data, data_len, false);
}

bool _resend_unresponsive_cmds(int slot)
{
    //So now what, do we resend this command?
    if (node_list[slot].responses.retry_cnt >= MAX_NODE_RETRIES) //If we have retried this command more than 3 times, we give up
        return false; //Give up on this node

    iprintln(trNODE, "#Resending last command to node 0x%02X (%d)", node_list[slot].address, slot);

    //Resend the command
    node_list[slot].responses.retry_cnt++; //Increment the retry count
    comms_tx_msg_init(&node_list[slot].msg, node_list[slot].address); //Initialize the message for this node
    //init_node_msg((uint8_t)slot, false);
    for (int j = 0; j < node_list[slot].responses.cnt; j++)
    {
        if (!comms_tx_msg_append(&node_list[slot].msg, node_list[slot].address, node_list[slot].responses.cmd_data[j].cmd, node_list[slot].responses.cmd_data[j].data, node_list[slot].responses.cmd_data[j].len, false))
        {
            //RVN - TODO - Not liking how I am handling this failure.... but what the heck am I supposed to do.... another retry counter for this as well?
            iprintln(trNODE, "#Error: Could not reload \"%s\" (%d bytes) to node %d (0x%02X) during resend", cmd_to_str(node_list[slot].responses.cmd_data[j].cmd), node_list[slot].responses.cmd_data[j].len, slot, node_list[slot].address);
            return false; //Failed to append the command, so we cannot resend it
        }
    }
    comms_tx_msg_send(&node_list[slot].msg); //Send the message immediately
    node_list[slot].responses.expiry = sys_poll_tmr_ms() + CMD_RESPONSE_TIMEOUT_MS;
    return true; //Command resent successfully
}

master_command_t _next_expected_response(int slot)
{
    return node_list[slot].responses.cmd_data[0].cmd; //Return the first command in the list
}

void _rollcall_handler(uint8_t addr)
{
    //if we are not busy waiting for a roll-call response, then we can just ignore this
    if (!sys_poll_tmr_is_running(&rollcall.timer))
        return;

    //Add this address to the response list
    for (int i = 0; i < RGB_BTN_MAX_NODES; i++)
    {
        //Find the first empty slot
        if (rollcall.list[i] == addr) 
        {
            //This address is already in the list (only possible with "cmd_roll_call(unreg)"), so what happened?
            /* It could be that this node has reset and is now responding to a roll-call again, (more likely, since it starting up in an unregsistered state)
            or it could be that a new node has joined the network with the same address as a previous node (less likely)

            I guess the best course of action is to double check if we have comms with the node using this address.... if so, then this is a new button that has joined the network.
                Otherwise we can assume that this is a node that has reset and is now responding to a roll-call again.

                For now we are just going to assume the more likely scenario...
            */
            int node_slot = -1;
            if (_get_adress_node_index(addr, &node_slot))
            {
                //This address is already registered as a node, so let's delete that node
                iprintln(trNODE, "#Address 0x%02X already registered as a node %d", addr, i);
                _deregister_node(node_slot); //Deregister the node
                //RVN - TODO For safety, should we consider sending a "deregister" command to the node?
            }
            return; //No need to continue, we found the address
        }
        if (rollcall.list[i] == 0) 
        {
            int cnt = add_rc_address(addr);
            iprintln(trNODE, "#Got RC Reply #%d from 0x%02X (%d)", cnt, addr, i);
            return; //We found a slot, no need to continue
        }
        //else, this slot belongs to another button, so continue searching
    }
    iprintln(trNODE, "#No space for 0x%02X", addr);
}

size_t _response_payload_size(master_command_t cmd, response_code_t resp, size_t data_len_remaining)
{
    if (resp == resp_err_range)
        return 2; //2 bytes payload

    if ((resp == resp_err_payload_len) || (resp == resp_err_reject_cmd))
        return 1; //1 byte payload

    if ((resp == resp_err_unknown_cmd) || (data_len_remaining <= 2))
        return 0; //Nothing 

    switch (cmd)
    {
        case cmd_roll_call:
        case cmd_set_bitmask_index:
        case cmd_bcast_address_mask:
        case cmd_set_rgb_0:
        case cmd_set_rgb_1:
        case cmd_set_rgb_2:
        case cmd_set_blink:
        case cmd_set_switch:
        case cmd_set_dbg_led:
        case cmd_set_time:
        case cmd_new_add:
#if REMOTE_CONSOLE_SUPPORTED == 1
        case cmd_wr_console_cont:
#endif /* REMOTE_CONSOLE_SUPPORTED */
        case cmd_none:
            return 0; //No payload exp
            break;

        case cmd_get_rgb_0:           
        case cmd_get_rgb_1:           
        case cmd_get_rgb_2:           
            return 3 * sizeof(uint8_t);
            break;
        
        case cmd_get_blink:
        case cmd_get_reaction:
        case cmd_get_time:
            return sizeof(uint32_t); 
            break;
            
        case cmd_get_sync:
            return sizeof(float);
            break;

        case cmd_get_flags:
        case cmd_get_dbg_led:
            return sizeof(uint8_t); 
            break;

#if REMOTE_CONSOLE_SUPPORTED == 1    
        case cmd_wr_console_done:
#endif /* REMOTE_CONSOLE_SUPPORTED */
        case cmd_debug_0:
        default:
            return data_len_remaining - 2; //the remainder of data in the message (minus the 2 bytes for the response code and command)
            break;
    }
}

void _deregister_node(int node)
{
    if (!is_node_valid(node))
        return; //No button registered at this slot

    iprintln(trNODE, "#Deregistering node %d (0x%02X)", node, node_list[node].address);
    memset(&node_list[node], 0, sizeof(slave_node_t)); //Reset the slot to zero
}

void _check_all_pending_node_responses(void)
{
    //Check if we have any pending responses that timed out
    for (int node = 0; node < RGB_BTN_MAX_NODES; node++)
    {

        if (0 == _responses_pending(node))
            continue; //No pending responses for this node, so we can skip it
        
        //Check if the response has timed out
        if (node_list[node].responses.expiry > sys_poll_tmr_ms())
            continue; //No timeout (yet)

        iprintln(trNODE, "#Response timeout for 0x%02X (%d):", node_list[node].address, node);
        for (int j = 0; j < node_list[node].responses.cnt; j++)
            iprintln(trNODE, "#  - \"%s\" (%d bytes)", cmd_to_str(node_list[node].responses.cmd_data[j].cmd), node_list[node].responses.cmd_data[j].len);

        if (!_resend_unresponsive_cmds(node))
            _deregister_node(node); //Deregister the node
    }
}

int _responses_pending(int slot)
{
    if (!is_node_valid(slot)) //Check if the slot is valid and has a registered button
        return 0; //Skip this slot

    if (node_list[slot].address == 0) //This slot is not in use
        return 0; //Skip this slot

    if (node_list[slot].responses.cnt == 0) //No pending responses for this node
        return 0; //Skip this slot

    if (node_list[slot].responses.expiry == 0)
        return 0; //No timeout (yet)

    return node_list[slot].responses.cnt;
}

void _response_handler(int slot, master_command_t resp_cmd, response_code_t resp, uint8_t *resp_data, size_t resp_data_len)
{
        //     _cmd = (master_command_t) msg->data[_data_idx++];
        // _resp = (response_code_t)msg->data[_data_idx++]; //The 2nd byte is the response code
        // _resp_data = &msg->data[_data_idx]; //The rest of the data is the response data
        // _resp_data_len = _response_payload_size(_cmd, _resp, payload_len - _data_idx); //The rest of the payload is the data

        //Does this node have cmds waiting on responses
    if (!is_node_valid(slot)) //Check if the slot is valid and has a registered button
        return; //Skip this slot

    if (node_list[slot].responses.cnt == 0) //No pending responses for this node
        return; //Skip this slot

    if (node_list[slot].responses.expiry == 0)
        return; //No timeout (yet)

    int _pending_responses = _responses_pending(slot);
    if (_pending_responses == 0)
        return; //No pending responses for this node, so we can return

    //We should be getting the responses in the same order as we sent the commands

    master_command_t waiting_cmd = _next_expected_response(slot);
    if (resp_cmd != waiting_cmd)
    {
        //Oops... this is not right?!?!?!
        iprintln(trNODE, "#Error: Node %d (0x%02X) sent response (0x%02X) for \"%s\" iso \"%s\"", slot, get_node_addr(slot), resp, cmd_to_str(resp_cmd), cmd_to_str(waiting_cmd));
        return; //Skip this response, we can't handle it
    }

    if (resp != resp_ok)
    {
        //We got an error response, so we can handle it
        iprintln(trNODE, "#Node %d (0x%02X) sent error response (0x%02X) for \"%s\"", slot, get_node_addr(slot), resp, cmd_to_str(resp_cmd));
        if (resp_data_len > 0)
        {
            iprint(trNODE, "# Additional data: ");
            for (int i = 0; i < resp_data_len; i++)
                iprintf("%02X ", resp_data[i]);
            iprintln(trNODE, "");
        }
        return; //Skip this response, we can't handle it
    }
    //else (_resp == resp_ok)

    //If this was a response to a read command, then we want to "save" the data returned from the node
    if (resp_data_len > 0) //Only GET's will have a response data length greater than 0
    {

        unsigned int member_offset = 0;
        switch (resp_cmd)
        {
            case cmd_get_rgb_0:     member_offset = offsetof(button_t, rgb_colour[0]);      break;
            case cmd_get_rgb_1:     member_offset = offsetof(button_t, rgb_colour[1]);      break;
            case cmd_get_rgb_2:     member_offset = offsetof(button_t, rgb_colour[2]);      break;
            case cmd_get_blink:     member_offset = offsetof(button_t, blink_ms);           break;
            case cmd_get_reaction:  member_offset = offsetof(button_t, reaction_ms);        break;
            case cmd_get_flags:     member_offset = offsetof(button_t, flags);              break;
            case cmd_get_dbg_led:   member_offset = offsetof(button_t, dbg_led_state);      break;
            case cmd_get_time:      member_offset = offsetof(button_t, time_ms);            break;
            case cmd_get_sync:      member_offset = offsetof(button_t, time_factor);       break;
            default:
                //We don't have any data to save for these commands
                return; //Skip this response, we can't handle it
                break;
        }
        //Update the button member with the new value
        memcpy(((uint8_t *)&node_list[slot].btn) + (size_t)member_offset, (void *)resp_data, resp_data_len);
        node_list[slot].last_update_time = sys_poll_tmr_ms(); //Update the last update time for this node

        //RVN - TODO - Maybe we should maintain a timestamp for every field?
    }

    //We got an OK response, so we can drop that command from the list

    if (node_list[slot].responses.cnt > 1)//Delete the entry at index 0 and move the rest of the entries down
        memmove(&node_list[slot].responses.cmd_data[0], &node_list[slot].responses.cmd_data[1], (node_list[slot].responses.cnt - 1) * sizeof(cmd_data_t));

    memset(&node_list[slot].responses.cmd_data[node_list[slot].responses.cnt - 1], 0, sizeof(cmd_data_t)); //Reset the last entry

    node_list[slot].responses.cnt--;
    if (node_list[slot].responses.cnt == 0)
    {
        node_list[slot].responses.retry_cnt = 0; //Reset the retry count
        node_list[slot].responses.expiry = 0; //Reset the expiry time
    }
}

/*******************************************************************************
 Global (public) Functions
 *******************************************************************************/

bool is_addr_registered(uint8_t addr)
{
    for (int i = 0; i < RGB_BTN_MAX_NODES; i++)
    {
        if (node_list[i].address == addr)
            return true; //Found the address in the registered list
    }
    return false; //Address not found in the registered list
}

uint8_t get_node_addr(uint8_t node)
{
    // if (!is_node_valid(node))
    //     return ADDR_BROADCAST;

    return node_list[node].address;
}

uint8_t _register_addr(uint8_t addr)
{
    if ((addr == ADDR_BROADCAST)  || (addr == ADDR_MASTER))
        return 0x00;    //Invalid address

    if (is_addr_registered(addr))
        return addr;    //Already registered

    //Find the next free slot in the node_list
    for (int i = 0; i < RGB_BTN_MAX_NODES; i++)
    {
        if ((node_list[i].address == 0) || (node_list[i].address == ADDR_BROADCAST)  || (node_list[i].address == ADDR_MASTER))
        {
            //LEt's assume this is going to be all good
            memset(&node_list[i], 0, sizeof(slave_node_t)); //Reset the slot to zero
            node_list[i].address = addr; //Set the address of this node

            init_node_msg((uint8_t)i/*, true*/);
            if (add_node_msg_register((uint8_t)i))
            {
                if (node_msg_tx_now((uint8_t)i, 1000LL))
                {
                    iprintln(trALWAYS, "Registered 0x%02X @ %d (%d/%d nodes)", node_list[i].address, i, node_count(), rc_response_count());
                    return addr; //Continue to the next address
                }
                else
                {
                    //WTF?!?!
                    iprintln(trALWAYS, "Registration failed for 0x%02X @ %d (%d/%d nodes)", addr, i, node_count(), rc_response_count());
                    memset(&node_list[i], 0, sizeof(slave_node_t)); //Reset the slot to zero
                    return 0x00; //Failed to send the register message
                }
            }
        }
    }
    iprintln(trALWAYS, "No Node Slot available for 0x%02X", addr);
    return 0x00;
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

void init_node_msg(uint8_t node/*, bool reset_response_data*/)
{
    if (!is_node_valid(node))
        return;

    comms_tx_msg_init(&node_list[node].msg, node_list[node].address); //Initialize the message for this node

    // if (!reset_response_data)
    //     return; //We don't want to reset the response data, so we can just return

    node_list[node].responses.cnt = 0; //Reset the command count for this node
    node_list[node].responses.expiry = 0; //Reset the expiry time
    node_list[node].responses.retry_cnt = 0; //Reset the retry count
    memset(node_list[node].responses.cmd_data, 0, sizeof(node_list[node].responses.cmd_data)); //Reset the command data list
}

bool node_msg_tx_now(uint8_t node, uint64_t wait_for_timeout)
{
    if (!is_node_valid(node))
        return false;

    if (!comms_tx_msg_send(&node_list[node].msg)) //Send the message immediately
    {
        iprintln(trNODE, "#Error: Could not send message to node %d (0x%02X)", node, node_list[node].address);
        return false; //Failed to send the message
    }

    if (wait_for_timeout == 0)
        return true; //If we don't want to wait for a response, we can just return true

    //This function will block whichever task it is called from until the response is received or the timeout expires.
    //The responses are handled in the main APP task, so this should NOT be called from the APP task, as it will block the console task and not allow the APP task to process the responses.

    //RVN - TODO - Can we handle the COMMS responses here????

    uint64_t expiry_time = sys_poll_tmr_ms() + wait_for_timeout;

    //iprintln(trALWAYS, "Waiting for response from 0x%02X (%d cmd%s)", node_list[node].address, node_list[node].responses.cnt, (node_list[node].responses.cnt > 1) ? "s" : "");
    while (_responses_pending(node) > 0)
    {
        node_parse_rx_msg(); //Process any received messages, this will update the node_list with the responses

        if (sys_poll_tmr_ms() > expiry_time)
        {
            iprintln(trALWAYS, "#Timeout waiting for response from %d (0x%02X), %d cmd%s", node, get_node_addr(node), _responses_pending(node), (_responses_pending(node) > 1) ? "s" : "");
            return false; //Timeout expired, no response received
        }
        //Just wait here....
        vTaskDelay(1); //Wait a bit before checking again
    }

    return true; //Response received
}

bool add_node_msg_register(uint8_t node)
{
    return _add_cmd_to_node_msg(node, cmd_set_bitmask_index, &node, sizeof(uint8_t), true);
}

bool add_node_msg_new_addr(uint8_t node, uint8_t new_addr)
{
    uint8_t node_addr = get_node_addr(node);
    
    //Maybe not a good idea to allow the new address to be the same as the old one?
    if (new_addr == node_addr)
    {
        iprintln(trNODE, "#New address is the same as the old address for node %d (0x%02X)", node, node_addr);
        return true; //No need to send a command, the address is already set
    }

    /*RVN - TODO - There is so much extra admin with this, I wonder if it is worth having it at all...
    * We need to:
    * 1. Make sure that the address is not already in use
    * 2. Make sure that the address is not the same as the current address
    * 3. Check if the command was received OK, and then change the address in the node_list
    * 4. If the command was not received OK, we need .... what? try again?... how many times?
    * 
    * 
    * Another question to consider.... why do we need to assign a new address to a node/button?
     */

     return _add_cmd_to_node_msg(node, cmd_new_add, &new_addr, sizeof(uint8_t), true);
}

bool add_node_msg_set_rgb(uint8_t node, uint8_t index, uint32_t rgb_col)
{
    if (index > 2)
    {
        iprintln(trNODE, "#Invalid RGB index (%d)", index);
        return false;
    }
    return _add_cmd_to_node_msg(node, cmd_set_rgb_0 + index, (uint8_t *)&rgb_col, (3*sizeof(uint8_t)), false);
}
bool add_node_msg_get_rgb(uint8_t node, uint8_t index)
{
    if (index > 2)
    {
        iprintln(trNODE, "#Invalid RGB index (%d)", index);
        return false;
    }
    return _add_cmd_to_node_msg(node, cmd_get_rgb_0 + index, NULL, 0, false);
}

bool add_node_msg_set_blink(uint8_t node, uint32_t period_ms)
{
    return _add_cmd_to_node_msg(node, cmd_set_blink, (uint8_t *)&period_ms, sizeof(uint32_t), false);
}
bool add_node_msg_get_blink(uint8_t node)
{
    return _add_cmd_to_node_msg(node, cmd_get_blink, NULL, 0, false);
}

bool add_node_msg_set_dbgled(uint8_t node, dbg_blink_state_t state)
{
    return _add_cmd_to_node_msg(node, cmd_set_dbg_led, (uint8_t *)&state, sizeof(dbg_blink_state_t), false);
}
bool add_node_msg_get_dbgled(uint8_t node)
{
    return _add_cmd_to_node_msg(node, cmd_get_dbg_led, NULL, 0, false);
}

bool add_node_msg_set_active(uint8_t node, bool start)
{
    uint8_t payload = start? 0x01 : 0x00; //Default command to start the stopwatch
    return _add_cmd_to_node_msg(node, cmd_set_switch, &payload, sizeof(uint8_t), false);
}
bool add_node_msg_get_reaction(uint8_t node)
{
    return _add_cmd_to_node_msg(node, cmd_get_reaction, NULL, 0, false);
}

bool add_node_msg_get_flags(uint8_t node)
{
    return _add_cmd_to_node_msg(node, cmd_get_flags, NULL, 0, false);
}

bool add_node_msg_set_time(uint8_t node, uint32_t new_time_ms)
{
    return _add_cmd_to_node_msg(node, cmd_set_time, (uint8_t *)&new_time_ms, sizeof(uint32_t), false);
}
bool add_node_msg_get_time(uint8_t node)
{
    return _add_cmd_to_node_msg(node, cmd_get_time, NULL, 0, false);
}

bool add_node_msg_sync_reset(uint8_t node)
{
    uint32_t reset_value = 0xFFFFFFFF; //Reset value for the sync command
    return _add_cmd_to_node_msg(node, cmd_set_sync, (uint8_t *)&reset_value, sizeof(uint32_t), false);
}
bool add_node_msg_sync_start(uint8_t node)
{
    sys_stopwatch_ms_start(&sync_stopwatch, 0xFFFFFFFE); //Start the stopwatch for the sync command
    uint32_t start_value = 0x0; //Start value for the sync command
    return _add_cmd_to_node_msg(node, cmd_set_sync, (uint8_t *)&start_value, sizeof(uint32_t), false);
}
bool add_node_msg_sync_end(uint8_t node)
{
    uint32_t stop_value = sys_stopwatch_ms_stop(&sync_stopwatch); //Stop the sync and provide our elapsed time
    return _add_cmd_to_node_msg(node, cmd_set_sync, (uint8_t *)&stop_value, sizeof(uint32_t), false);
}
bool add_node_msg_get_correction(uint8_t node)
{
    return _add_cmd_to_node_msg(node, cmd_get_sync, NULL, 0, false);
}

const char * cmd_to_str(master_command_t cmd)
{
    switch (cmd)
    {
        case cmd_none:                  return "none";
        case cmd_roll_call:             return "roll_call";
        case cmd_set_bitmask_index:     return "set_bitmask_index";
        case cmd_bcast_address_mask:    return "bcast_address_mask";
        case cmd_set_rgb_0:             return "set_rgb_0";
        case cmd_set_rgb_1:             return "set_rgb_1";
        case cmd_set_rgb_2:             return "set_rgb_2";
        case cmd_set_blink:             return "set_blink";
        case cmd_set_switch:            return "set_switch";
        case cmd_set_dbg_led:           return "set_dbg_led";
        case cmd_set_time:              return "set_time";
        case cmd_new_add:               return "new_add";
        case cmd_get_rgb_0:             return "get_rgb_0";
        case cmd_get_rgb_1:             return "get_rgb_1";
        case cmd_get_rgb_2:             return "get_rgb_2";
        case cmd_get_blink:             return "get_blink";
        case cmd_get_reaction:          return "get_sw_time";
        case cmd_get_flags:             return "get_flags";
        case cmd_get_dbg_led:           return "get_dbg_led";
        case cmd_get_time:              return "get_time";
#if REMOTE_CONSOLE_SUPPORTED == 1    
        case cmd_wr_console_cont:       return "wr_console_cont";
        case cmd_wr_console_done:       return "wr_console_done";
#endif /* REMOTE_CONSOLE_SUPPORTED */
        case cmd_debug_0:               return "debug_0";
        default:                        return "unknown";
    }
}

void init_bcst_msg(int * exclude_nodes, int exclude_count)
{
    uint32_t _bcst_mask = _inactive_nodes_mask(); //Start with no excluded nodes (apart from the active node)
    if ((exclude_nodes != NULL) && (exclude_count > 0))
    {
        for (int i = 0; ((i < exclude_count) && (i < RGB_BTN_MAX_NODES)); i++)
        {
            if (exclude_nodes[i] < RGB_BTN_MAX_NODES) //Check if the node # is valid
                _bcst_mask &= ~(1 << exclude_nodes[i]); //Clear the corresponding bit for this node
        }
    }

    comms_tx_msg_init(&bcst_msg, ADDR_BROADCAST); //Initialize the broadcast message
    comms_tx_msg_append(&bcst_msg, ADDR_BROADCAST, cmd_bcast_address_mask, (uint8_t *)&_bcst_mask, sizeof(uint32_t), true); //Append the cmd_bcast_address_mask command
}

void bcst_msg_tx_now(void)
{
    if (!comms_tx_msg_send(&bcst_msg))
        iprintln(trNODE, "#Error: Could not send broadcast (0x%02X)", bcst_msg.msg.hdr.id);
}

bool add_bcst_msg_set_rgb(uint8_t index, uint32_t rgb_col)
{
    if (index > 2)
    {
        iprintln(trNODE, "#Invalid RGB index (%d)", index);
        return false;
    }
    return _bcst_append(cmd_set_rgb_0 + index, (uint8_t *)&rgb_col, (3*sizeof(uint8_t)));
}

bool add_bcst_msg_set_blink(uint32_t period_ms)
{
    return _bcst_append(cmd_set_blink, (uint8_t *)&period_ms, sizeof(uint32_t));
}

bool add_bcst_msg_set_dbgled(dbg_blink_state_t dbg_blink_state)
{
    return _bcst_append(cmd_set_dbg_led, (uint8_t *)&dbg_blink_state, sizeof(uint8_t));
}

bool add_bcst_msg_set_time_ms(uint32_t new_time_ms)
{
    return _bcst_append(cmd_set_dbg_led, (uint8_t *)&new_time_ms, sizeof(uint32_t));
}

bool add_bcst_msg_sync_reset(void)
{
    uint32_t reset_value = 0xFFFFFFFF; //Reset value for the sync command
    return _bcst_append(cmd_set_sync, (uint8_t *)&reset_value, sizeof(uint32_t));
}
bool add_bcst_msg_sync_start(void)
{
    sys_stopwatch_ms_start(&sync_stopwatch, 0xFFFFFFFE); //Start the stopwatch for the sync command
    uint32_t start_value = 0x0; //Start value for the sync command
    return _bcst_append(cmd_set_sync, (uint8_t *)&start_value, sizeof(uint32_t));
}
bool add_bcst_msg_sync_end(void)
{
    uint32_t stop_value = sys_stopwatch_ms_stop(&sync_stopwatch); //Stop the sync and provide our elapsed time
    return _bcst_append(cmd_set_sync, (uint8_t *)&stop_value, sizeof(uint32_t));
}
bool is_time_sync_busy(void)
{
    //Check if the sync stopwatch is running
    return sync_stopwatch.running;
}

bool bcst_rollcall(bool all)
{
    uint8_t data = 0xFF;

    if (all)
    {
        data = 0x00; //If we are doing a roll-call for all nodes, we set the data to 0x00
        memset(&rollcall, 0, sizeof(rollcall_t)); //Reset the roll-call list
        memset(node_list, 0, sizeof(node_list)); //Reset all buttons to unregistered state
    }

    if (comms_tx_msg_append(&bcst_msg, ADDR_BROADCAST, cmd_roll_call, &data, sizeof(uint8_t), true))
    {
        //This is all that is needed... this command can be sent immediately
        if (comms_tx_msg_send(&bcst_msg))
        {
            //The maximum time we could afford to wait for a response.
            sys_poll_tmr_start(&rollcall.timer, (ADDR_BROADCAST * 2 * BUS_SILENCE_MIN_MS) + 0xFF + BUS_SILENCE_MIN_MS, false);
            return true;
        }
    }
    return false;
}

uint32_t _inactive_nodes_mask(void)
{
    uint32_t mask = 0x00000000; //Start with all nodes inactive
    for (int i = 0; i < RGB_BTN_MAX_NODES; i++)
    {
        if ((node_list[i].address != 0) && (node_list[i].active == false))
            mask |= (1 << i); //Set the bit for this node
    }
    return mask; //Return the mask of inactive nodes (which are valid)
}

int active_node_count(void)
{
    int count = 0;
    for (int i = 0; i < RGB_BTN_MAX_NODES; i++)
    {
        if ((node_list[i].address != 0) && (node_list[i].active == true))
            count++; //Count the number of registered buttons
    }
    if (count > 1)
        iprintln(trNODE, "#ERROR - %d active nodes found", count);

    return count;
}

button_t * get_node_button_ptr(int slot)
{
    if (!is_node_valid(slot)) //Check if the slot is valid and has a registered button
        return NULL; //Skip this slot

    if (node_list[slot].address == 0) //This slot is not in use
        return NULL; //Skip this slot


    return &node_list[slot].btn; //Return a poitner to the button for this node
}

int rc_response_count(void)
{
    return rollcall.cnt; //Return the number of slave nodes that responded to the roll-call
}

int add_rc_address(uint8_t addr)
{
    if (rollcall.cnt >= RGB_BTN_MAX_NODES)
    {
        iprintln(trNODE, "#Error: Roll-call list is full (%d/%d)", rollcall.cnt, RGB_BTN_MAX_NODES);
    }
    else
    {
        for (int i = 0; i < RGB_BTN_MAX_NODES; i++)
        {
            if (rollcall.list[i] == addr)
            {
                iprintln(trNODE, "#Address 0x%02X already in roll-call list", addr);
                break; //From for loop.... Address already in the list
            }

            if (rollcall.list[i] == 0x00)
            {
                //Empty slot found, so we can add the address here
                rollcall.list[rollcall.cnt++] = addr; //Add the address to the list and increment the count
                break; //From for loop
            }
        }
    }
    return rollcall.cnt; //Return the number of addresses in the roll-call list
}

bool waiting_for_rollcall(bool blocking)
{
    //This call will block the calling task until the roll-call timer expires

    if (!sys_poll_tmr_expired(&rollcall.timer))
    {
        if (!blocking)
            return true;
        
        uint64_t wait_time = rollcall.timer.ms_expire - sys_poll_tmr_ms();
        //We could still received roll-call responses, so we cannot register nodes yet
        iprintln(trNODE, "Roll-call will complete in %lu.%03d s.", wait_time / 1000, (int)(wait_time % 1000));

        while (!sys_poll_tmr_expired(&rollcall.timer))
        {
            node_parse_rx_msg(); //Process any received messages, this will update the node_list with the responses
            vTaskDelay(pdMS_TO_TICKS((uint32_t)(MIN(100, BUS_SILENCE_MIN_MS + wait_time)))); //Wait a bit (max 100ms) before we check again
        }
    }

    return false; //Roll-call is complete, we can register nodes now
}

void node_parse_rx_msg(void)
{
    comms_msg_t rx_msg;
    size_t rx_msg_size;

    int _data_idx = 0;
    int _cmd_idx = 0;
    master_command_t _cmd;
    response_code_t _resp;
    uint8_t * _resp_data;
    size_t _resp_data_len;


    if (comms_msg_rx_read(&rx_msg, &rx_msg_size))
    {

        size_t payload_len = rx_msg_size - (sizeof(comms_msg_hdr_t) + sizeof(uint8_t));

        //start at the beginning of the data
        do
        {
            _cmd = (master_command_t) rx_msg.data[_data_idx++];
            _resp = (response_code_t)rx_msg.data[_data_idx++]; //The 2nd byte is the response code
            _resp_data = &rx_msg.data[_data_idx]; //The rest of the data is the response data
            _resp_data_len = _response_payload_size(_cmd, _resp, payload_len - _data_idx); //The rest of the payload is the data
            _data_idx += _resp_data_len; //Move the data index forward by the command data length

            //We are expecting response messages in only 2 instances:
            // 1) Rollcall responses
            // 2) Responses to commands sent to an ACTIVE node
            if (_cmd == cmd_roll_call)
            {
                _rollcall_handler(rx_msg.hdr.src); //Handle the roll-call response
                //RVN - should we return from here? or continue processing the rest of the message?
                return;
            }
            else
            {
                int node_slot = -1;
                if (_get_adress_node_index(rx_msg.hdr.src, &node_slot))
                    _response_handler(node_slot, _cmd, _resp, _resp_data, _resp_data_len);
                else //Response to a command sent directly to a node, but we are not waiting for a response (unless we are in a rollcall stage?)?????
                {
                    if (_resp == resp_ok)
                        iprintln(trNODE, "#UNSOLICITED OK rx'd for \"%s\", (Node Address 0x%02X)", cmd_to_str(_cmd), rx_msg.hdr.src);
                    else
                        iprintln(trNODE, "#UNSOLICITED Error %d rx'd for \"%s\", (Node Address 0x%02X)", _resp, cmd_to_str(_cmd), rx_msg.hdr.src);
                        //We can handle the error here, but we don't have to
                        //We can also just ignore it
                }
            }
            _cmd_idx++;
        } while (_data_idx< payload_len);
    }

    //Check if we have any pending responses that timed out
    _check_all_pending_node_responses();
}

void register_new_buttons(void)
{
    uint8_t _addr = 0x00; //Start with the first address

    //This call will block the console task until the roll-call timer expires, so we can register nodes
    waiting_for_rollcall(true);

    for (int i = 0; i < RGB_BTN_MAX_NODES; i++)
    {
        _addr = rollcall.list[i];
        if (_addr == 0) //This slot is empty, so we can not use it
            continue; //Skip this slot, it is empty

        else if (is_addr_registered(_addr))
            continue; //Skip this address, it is already registered
            
        else if (_register_addr(_addr) == 0x00)
            iprintln(trALWAYS, "Failed to register node 0x%02X in slot %d", (uint8_t)_addr, i);
    }
}

#undef PRINTF_TAG
#undef EXT
/*************************** END OF FILE *************************************/
