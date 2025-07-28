/*******************************************************************************
Module:     nodes.c
Purpose:    This file contains the _______ functions
Author:     Rudolph van Niekerk


For clarification, a button is JUST a button until it's address is registered, 
 then it is a NODE on our network.
Buttons are reference by their address, and NODES are referenced by their 
  slot #in the nodes.list array.
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
#include "str_helper.h"

#define __NOT_EXTERN__
#include "../../../../common/common_comms.h"
#include "nodes.h"
#undef __NOT_EXTERN__

#include "task_comms.h"

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
    uint8_t     list[RGB_BTN_MAX_NODES+1]; // The response list for the roll-call
    int         cnt; // The number of slave nodes
    Timer_ms_t  timer;
}rollcall_t;

typedef struct
{
    master_command_t cmd;
    cmd_payload_u payload; // The payload for the command
    // uint8_t *data;
//    uint8_t len;
}cmd_data_t;

typedef struct
{
    cmd_data_t cmd_data[NODE_CMD_CNT_MAX]; // The commands we are waiting for a response to
    uint8_t cnt; // The number of commands in this command list
    uint8_t retry_cnt; // The number of times we have retried sending a message to this node
    uint64_t expiry;
    uint32_t exp_rx_len; // The length of the response data we are waiting for 
    uint8_t  exp_rx_cnt; // The number of responses we are waiting for
}slave_node_cmd_t;

typedef struct
{
    uint8_t             address; // The address of the node
    uint8_t             seq; // The last sequence number received from the node
    slave_node_cmd_t    responses;//[NODE_CMD_CNT_MAX]; // The last NODE_CMD_CNT_MAX commands, and associated data and responses sent/received
    bool                active; // The node is active (stopwatch running, probably blinking... but most importantly, it should not get broadcast messages)
    button_t            btn; // The button data for this node
    uint64_t            last_update_time; // The last time we have updated this node's data
    comms_tx_msg_t      msg; // The message we are currently building to send to this node
}slave_node_t;

typedef struct
{
    slave_node_t        list[RGB_BTN_MAX_NODES]; // The list of registered nodes
    uint8_t             cnt; // The number of nodes in the list
}nodes_t;

/*******************************************************************************
 Local function prototypes
 *******************************************************************************/
/*! \brief Start a roll-call for all nodes.
 * \param all If true, the function will reset the roll-call list and start a new roll-call for all nodes.
 * This function sends a roll-call command to all nodes and waits for their responses.
 * It is used to discover all (or new) nodes in the network and register them.
 */
bool _bcst_rollcall(bool all);

/*! \brief Wait for the roll-call timer to expire.
 * \param blocking If true, the function will block until the roll-call timer expires.
 * This function blocks the calling task until the roll-call timer expires.
 * It is used to ensure that all roll-call responses are received before proceeding with other functions such as registering nodes.
 */
bool _waiting_for_rollcall(bool blocking);

int _add_rc_address(uint8_t addr);

bool _get_adress_node_index(uint8_t addr, int *slot);
void _deregister_node(int node);

bool _add_cmd_to_node_msg(uint8_t node, master_command_t cmd, uint8_t *data, bool restart);
bool _bcst_append(uint8_t cmd, uint8_t * data);
void _rollcall_handler(uint8_t addr);

void _check_all_pending_node_responses(void);
void _response_handler(int slot, master_command_t resp_cmd, response_code_t resp, uint8_t *resp_data, size_t resp_data_len);
size_t _miso_payload_size(master_command_t cmd, response_code_t resp);
int _responses_pending(int slot);
bool _resend_unresponsive_cmds(int slot);

uint8_t _register_addr(uint8_t addr);

uint32_t _inactive_nodes_mask(void);

void * _get_node_btn_data_generic(int slot, master_command_t cmd);

/*******************************************************************************
 Local variables
 *******************************************************************************/

nodes_t nodes = {0}; // The structure containing all registered nodes

comms_tx_msg_t bcst_msg = {0};

rollcall_t rollcall = {0}; // The structure containing information for all who respond on rollcalls

//RVN - Technically I  should maintain a separate stopwatch for each node, but 
//  holy crap that is adding sooooooooo much more complexity (e.g. a sw is 
//  started for a node and stopped using a broadcast... or vice versa... 
// ... Aaaaaaargh! Can open.... worms everywhere!)
Stopwatch_ms_t sync_stopwatch;
/*******************************************************************************
 Local (private) Functions
 *******************************************************************************/
bool _get_adress_node_index(uint8_t addr, int *slot)
{
    for (int i = 0; i < nodes.cnt; i++)
    {
        if (nodes.list[i].address == addr)
        {
            if (slot != NULL)
                *slot = i; //Return the slot index if requested
            return true; //Address found in the registered list
        }
    }
    return false; //Address not found in the registered list
}

bool is_node_valid(uint8_t node)
{
    if (nodes.cnt == 0)
    {
        iprintln(trNODE, "#No nodes registered yet");
        return false; //No nodes registered
    }
    if (node >= nodes.cnt)
    {
        iprintln(trNODE, "#Invalid slot %d for getting node address (%d/%d in use)", node, node_count(), RGB_BTN_MAX_NODES);
        return false;
    }
    if (nodes.list[node].address == 0)
    {
        // iprintln(trALWAYS, "No button registered at slot %d", slot);
        return false;
    }
    if ((nodes.list[node].address == ADDR_BROADCAST)  || (nodes.list[node].address == ADDR_MASTER))
    {
        iprintln(trNODE, "#Invalid node address, 0x%02X, at node %d... deleting!", nodes.list[node].address, node);
        //RVN - TODO -This should really be de-registered immediately, since it is not a valid node address
        memset(&nodes.list[node], 0, sizeof(slave_node_t)); //Reset the slot to zero
        return false;
    }
    return true; //The slot is valid and has a registered button
}

bool _add_cmd_to_node_msg(uint8_t node, master_command_t cmd, uint8_t *data, bool restart)//, uint64_t timeout_ms)
{
    uint8_t node_addr;
    uint8_t data_len = cmd_mosi_payload_size(cmd);
  
    if (!is_node_valid(node))
        return false; //No button registered at this slot

    node_addr = get_node_addr(node);

    if (nodes.list[node].responses.cnt >= NODE_CMD_CNT_MAX)
    {
        iprintln(trNODE, "#Cannot add command %s (0x%02X) to node %d (0x%02X) - (%d/%d)", cmd_to_str(cmd), cmd, node, node_addr, nodes.list[node].responses.cnt, NODE_CMD_CNT_MAX);
        return false; //Too many commands pending for this node
    }

    if (comms_tx_msg_append(&nodes.list[node].msg, node_addr, cmd, data, data_len, restart))
    {
        int i = nodes.list[node].responses.cnt;
        //Found an empty slot
        nodes.list[node].responses.cmd_data[i].cmd = cmd; //Store the command we are expecting a response for
        //We cannot just copy a pointer to the data since some of them are not static, so we need to copy the data
        if (data_len > 0)
            memcpy(nodes.list[node].responses.cmd_data[i].payload.data, data, data_len); //Copy the data to the payload
        else
            memset(nodes.list[node].responses.cmd_data[i].payload.data, 0, sizeof(nodes.list[node].responses.cmd_data[i].payload.data)); //If no data, then just zero the data buffer
        nodes.list[node].responses.cnt++; //Increment the command count for this node


        //RVN - TODO. I guess we can calculate how long the response will be and if it might be 
        //received over several messages, in which case we will need to extend the message timeout
        uint32_t response_len = _miso_payload_size(cmd, resp_ok);
        response_len = MIN(response_len, RGB_BTN_MSG_MAX_DATA_LEN) + (2 * sizeof(uint8_t)); //Get the expected response length for this command + 2 bytes for the response code and command ID
        //iprintln(trNODE, "#Expecting %d bytes for \"%s\" (%d - %d)", response_len, cmd_to_str(cmd), nodes.list[node].responses.exp_rx_len, nodes.list[node].responses.exp_rx_cnt);

        if ((nodes.list[node].responses.exp_rx_len + response_len) > RGB_BTN_MSG_MAX_DATA_LEN)
        {
            nodes.list[node].responses.exp_rx_cnt++;
            iprintln(trNODE, "#Response will span over %d msgs (%d > %d)", nodes.list[node].responses.exp_rx_cnt, nodes.list[node].responses.exp_rx_len + response_len, RGB_BTN_MSG_MAX_DATA_LEN);            
            nodes.list[node].responses.exp_rx_len = response_len; //Reset the response length to the expected response length
        }
        else
        {
            nodes.list[node].responses.exp_rx_len += response_len; //Add the expected response length to the total response length
        }
        // nodes.list[node].responses.expiry = (uint64_t)sys_poll_tmr_ms() + CMD_RESPONSE_TIMEOUT_MS;//timeout_ms; //Store the timestamp of when we sent the command
        nodes.list[node].responses.retry_cnt = 0;
        //if we run out of space... well, heck, then we will not wait for that response...
        return true; //Command added successfully
    }
    iprintln(trNODE, "#Failed to append command %s (0x%02X) for node %d (0x%02X)", cmd_to_str(cmd), cmd, node, node_addr);
    return false; //Failed to append the command
}

bool _bcst_append(uint8_t cmd, uint8_t * data)
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

    if (cmd == cmd_set_switch)
    {
        // Oooh.... this is a special case, since this command will activate ALL the nodes on the network
    }

    return comms_tx_msg_append(&bcst_msg, ADDR_BROADCAST, cmd, data, cmd_mosi_payload_size(cmd), false);
}

bool _resend_unresponsive_cmds(int slot)
{
    //So now what, do we resend this command?
    if (nodes.list[slot].responses.retry_cnt >= MAX_NODE_RETRIES) //If we have retried this command more than 3 times, we give up
        return false; //Give up on this node

    iprintln(trNODE, "#Resending last command to node 0x%02X (%d)", nodes.list[slot].address, slot);

    //Resend the command
    nodes.list[slot].responses.retry_cnt++; //Increment the retry count
    comms_tx_msg_init(&nodes.list[slot].msg, nodes.list[slot].address); //Initialize the message for this node
    //init_node_msg((uint8_t)slot, false);
    for (int i = 0; i < nodes.list[slot].responses.cnt; i++)
    {
        cmd_data_t *cmd_data = &nodes.list[slot].responses.cmd_data[i];
        if (!comms_tx_msg_append(&nodes.list[slot].msg, nodes.list[slot].address, cmd_data->cmd, &cmd_data->payload.data[0], cmd_mosi_payload_size(cmd_data->cmd), false))
        {
            //RVN - TODO - Not liking how I am handling this failure... but what the heck am I supposed to do... another retry counter for this as well?
            iprintln(trNODE, "#Error: Could not reload \"%s\" (%d bytes) to node %d (0x%02X) during resend", cmd_to_str(cmd_data->cmd), cmd_mosi_payload_size(cmd_data->cmd), slot, nodes.list[slot].address);
            return false; //Failed to append the command, so we cannot resend it
        }
    }
    comms_tx_msg_send(&nodes.list[slot].msg); //Send the message immediately
    nodes.list[slot].responses.expiry = sys_poll_tmr_ms() + CMD_RESPONSE_TIMEOUT_MS;
    return true; //Command resent successfully
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
            //This address is already in the rollcall list (only possible with "cmd_roll_call(unreg)"), so what happened?
            /* It could be that this node has reset and is now responding to a roll-call again, (more likely, since it starting up in an unregsistered state)
            or it could be that a new node has joined the network with the same address as a previous node (less likely)

            I guess the best course of action is to double check if we have comms with the node using this address... if so, then this is a new button that has joined the network.
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
            int cnt = _add_rc_address(addr);
            iprintln(trNODE, "#Got RC Reply #%d from 0x%02X (%d)", cnt, addr, i);
            return; //We found a slot, no need to continue
        }
        //else, this slot belongs to another button, so continue searching
    }
    iprintln(trNODE, "#No space for 0x%02X", addr);
}

size_t _miso_payload_size(master_command_t cmd, response_code_t resp)
{
    if (resp == resp_err_range)
        return 2; //2 bytes payload

    if ((resp == resp_err_payload_len) || (resp == resp_err_reject_cmd))
        return 1; //1 byte payload

    if (resp == resp_err_unknown_cmd)
        return 0; //Nothing 

    //means the response is OK, so we need to return the payload size based on the command
    for (int i = 0; i < ARRAY_SIZE(cmd_table); i++)
    {
        if (cmd_table[i].cmd == cmd)
        {
            //Found the command, so return the payload size
            return cmd_table[i].miso_sz;
        }
    }
    return (size_t)-1; //the remainder of data in the message (minus the 2 bytes for the response code and command)
}

void _deregister_node(int node)
{
    if (!is_node_valid(node))
        return; //No button registered at this slot

    //Is this the last node we are deregistering?
    int last_node_index = node_count() - 1; //Get the last node index
    if (node < last_node_index)
        memmove(&nodes.list[node], &nodes.list[node + 1], sizeof(slave_node_t) * (last_node_index - node));

    memset(&nodes.list[last_node_index], 0, sizeof(slave_node_t)); //Reset the rest of the node list
    nodes.cnt--; //Decrement the node count

    iprintln(trNODE, "#Deregistered node %d (0x%02X) - %d Nodes remain:", node, nodes.list[node].address, nodes.cnt);
    for (int i = 0; i < nodes.cnt; i++)
    {
        iprintln(trNODE, "#Node %d (0x%02X) - %d - %d ms", i, nodes.list[i].address, nodes.list[i].responses.cnt, nodes.list[i].responses.expiry);
    }

}

void _check_all_pending_node_responses(void)
{
    //Check if we have any pending responses that timed out
    for (int node = 0; node < nodes.cnt; node++)
    {

        if (0 == _responses_pending(node))
            continue; //No pending responses for this node, so we can skip it
        
        //Check if the response has timed out
        if (nodes.list[node].responses.expiry > sys_poll_tmr_ms())
            continue; //No timeout (yet)

        if (!_resend_unresponsive_cmds(node))
        {
            iprintln(trNODE, "# %d failed retries for node %d (0x%02X), %d cmds:", nodes.list[node].responses.retry_cnt, node, nodes.list[node].address, nodes.list[node].responses.cnt);
            for (int j = 0; j < nodes.list[node].responses.cnt; j++)
                iprintln(trNODE, "#   %d - \"%s\" (%d bytes)", j+1, 
                    cmd_to_str(nodes.list[node].responses.cmd_data[j].cmd), 
                    cmd_mosi_payload_size(nodes.list[node].responses.cmd_data[j].cmd));
            _deregister_node(node); //Deregister the node
            //move our index back by one, since we just removed a node
            node--;
        }
    }
}

int _responses_pending(int slot)
{
    // if (!is_node_valid(slot)) //Check if the slot is valid and has a registered button
    //     return 0; //Skip this slot

    if (nodes.list[slot].address == 0) //This slot is not in use
        return 0; //Skip this slot

    if (nodes.list[slot].responses.cnt == 0) //No pending responses for this node
        return 0; //Skip this slot

    if (nodes.list[slot].responses.expiry == 0)
        return 0; //No timeout (yet)

    return nodes.list[slot].responses.cnt;
}

void _response_handler(int slot, master_command_t resp_cmd, response_code_t resp, uint8_t *resp_data, size_t resp_data_len)
{
        //     _cmd = (master_command_t) msg->data[_data_idx++];
        // _resp = (response_code_t)msg->data[_data_idx++]; //The 2nd byte is the response code
        // _resp_data = &msg->data[_data_idx]; //The rest of the data is the response data
        // _resp_data_len = _miso_payload_size(_cmd, _resp, payload_len - _data_idx); //The rest of the payload is the data

    //iprintln(trNODE, "#RX 0x%02X for \"%s\" from Node %d. %d bytes", resp, cmd_to_str(resp_cmd), slot, resp_data_len);

    //Does this node have cmds waiting on responses
    if (!is_node_valid(slot)) //Check if the slot is valid and has a registered button
        return; //Skip this slot

    if (nodes.list[slot].responses.cnt == 0) //No pending responses for this node
        return; //Skip this slot

    if (nodes.list[slot].responses.expiry == 0)
        return; //No timeout (yet)

    int _pending_responses = _responses_pending(slot);
    if (_pending_responses == 0)
        return; //No pending responses for this node, so we can return

    //We should be getting the responses in the same order as we sent the commands

    master_command_t waiting_tx_cmd = nodes.list[slot].responses.cmd_data[0].cmd;
    cmd_payload_u *waiting_tx_data = &nodes.list[slot].responses.cmd_data[0].payload; //Get the data that was sent with the command

    if (resp_cmd != waiting_tx_cmd)
    {
        //Oops... this is not right?!?!?!
        iprintln(trNODE, "#Error: Node %d (0x%02X) sent response (0x%02X) for \"%s\" iso \"%s\"", slot, get_node_addr(slot), resp, cmd_to_str(resp_cmd), cmd_to_str(waiting_tx_cmd));
        return; //Skip this response, we can't handle it
    }

    if (resp != resp_ok)
    {
        //We got an error response, so we can handle it
        //iprintln(trNODE, "#Node %d (0x%02X) sent error response (0x%02X) for \"%s\"", slot, get_node_addr(slot), resp, cmd_to_str(resp_cmd));
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
            case cmd_get_sync:      member_offset = offsetof(button_t, time_factor);        break;
            case cmd_get_version:   member_offset = offsetof(button_t, version);            break;
            default:
                //We don't have any data to save for these commands
                return; //Skip this response, we can't handle it
                break;
        }
        // iprint(trNODE, "#%d: \"%s\" = OK. [", slot, cmd_to_str(resp_cmd));
        // for (int i = 0; i < resp_data_len; i++) iprint(trNODE, "%s%02X", (i > 0)? " ":"", resp_data[i]);
        // iprintln(trNODE, "]");
        //Update the button member with the new value
        memcpy(((uint8_t *)&nodes.list[slot].btn) + (size_t)member_offset, (void *)resp_data, resp_data_len);
        nodes.list[slot].last_update_time = sys_poll_tmr_ms(); //Update the last update time for this node

        //RVN - TODO - Maybe we should maintain a timestamp for every field?
    }

    //###########################################################################
    //For certain commands' responses, we need to do some additional processing
    //###########################################################################
    if (resp_cmd == cmd_set_switch)
    {
        //set/clear the active state of the node based on what was sent to the node
        nodes.list[slot].active = (waiting_tx_data->u8_val == CMD_SW_PAYLOAD_ACTIVATE); //Set the active state of the node based on the response data
        //iprintln(trNODE, "#Node %d (0x%02X) is now %s", slot, nodes.list[slot].address, (nodes.list[slot].active) ? "active" : "deactivated");
    }
    if (resp_cmd == cmd_get_reaction)
    {
        //If the slot "was" active and the button read returned a positive reaction time, then we can assume that the button is not active anymore
        if ((nodes.list[slot].btn.reaction_ms != 0) && (nodes.list[slot].active)) //If the reaction time is not zero and the node was active
        {
            nodes.list[slot].active = false; //Set the node to inactive
            //iprintln(trNODE, "#Node %d (0x%02X) deactivated itself", slot, nodes.list[slot].address);
        }

    }


    //We got an OK response, so we can drop that command from the list

    if (nodes.list[slot].responses.cnt > 1)//Delete the entry at index 0 and move the rest of the entries down
        memmove(&nodes.list[slot].responses.cmd_data[0], &nodes.list[slot].responses.cmd_data[1], (nodes.list[slot].responses.cnt - 1) * sizeof(cmd_data_t));

    memset(&nodes.list[slot].responses.cmd_data[nodes.list[slot].responses.cnt - 1], 0, sizeof(cmd_data_t)); //Reset the last entry

    nodes.list[slot].responses.cnt--;
    if (nodes.list[slot].responses.cnt == 0)
    {
        nodes.list[slot].responses.retry_cnt = 0; //Reset the retry count
        nodes.list[slot].responses.expiry = 0; //Reset the expiry time
    }
}

bool _bcst_rollcall(bool all)
{
    uint8_t data = 0xFF;

    if (all)
    {
        data = 0x00; //If we are doing a roll-call for all nodes, we set the data to 0x00
        memset(&rollcall, 0, sizeof(rollcall_t)); //Reset the roll-call list
        memset(nodes.list, 0, sizeof(nodes.list)); //Reset all buttons to unregistered state
        nodes.cnt = 0; //Reset the node count
    }

    if (comms_tx_msg_append(&bcst_msg, ADDR_BROADCAST, cmd_roll_call, &data, sizeof(uint8_t), true))
    {
        //This is all that is needed... this command can be sent immediately
        if (comms_tx_msg_send(&bcst_msg))
        {
            //The maximum time we could afford to wait for a response.
            sys_poll_tmr_start(&rollcall.timer, ROLL_CALL_TIMOUT_MS(ADDR_BROADCAST, ADDR_BROADCAST) + BUS_SILENCE_MIN_MS, false);
            return true;
        }
    }
    return false;
}

void * _get_node_btn_data_generic(int slot, master_command_t cmd)
{
    unsigned int member_offset = 0;

    if (!is_node_valid(slot)) //Check if the slot is valid and has a registered button
        return NULL;

    switch (cmd)
    {
        case cmd_get_rgb_0:     member_offset = offsetof(button_t, rgb_colour[0]);      break;
        case cmd_get_rgb_1:     member_offset = offsetof(button_t, rgb_colour[1]);      break;
        case cmd_get_rgb_2:     member_offset = offsetof(button_t, rgb_colour[2]);      break;
        case cmd_get_blink:     member_offset = offsetof(button_t, blink_ms);           break;
        case cmd_get_reaction:  member_offset = offsetof(button_t, reaction_ms);        break;
        case cmd_get_flags:     member_offset = offsetof(button_t, flags);              break;
        case cmd_get_dbg_led:   member_offset = offsetof(button_t, dbg_led_state);      break;
        case cmd_get_time:      member_offset = offsetof(button_t, time_ms);            break;
        case cmd_get_sync:      member_offset = offsetof(button_t, time_factor);        break;
        case cmd_get_version:   member_offset = offsetof(button_t, version);            break;
        default:
            //We don't have any data to save for these commands
            return NULL; //Skip this response, we can't handle it
            break;
    }

    return ((uint8_t *)&nodes.list[slot].btn) + (size_t)member_offset;
}

/*******************************************************************************
 Global (public) Functions
 *******************************************************************************/

uint8_t get_node_addr(uint8_t node)
{
    // if (!is_node_valid(node))
    //     return ADDR_BROADCAST;

    return nodes.list[node].address;
}

uint8_t _register_addr(uint8_t addr)
{
    if ((addr == ADDR_BROADCAST)  || (addr == ADDR_MASTER))
        return 0x00;    //Invalid address - Should really never happen, but just in case

    //Find the next free slot in the nodes.list
    for (int i = 0; i < nodes.cnt; i++)
    {
        if (nodes.list[i].address == addr) //If this address is already registered, we can just return it
        {
            iprintln(trNODE|trALWAYS, "#Address 0x%02X already registered at slot %d", addr, i);
            return addr; //Already registered
        }
    }

    if (nodes.cnt >= RGB_BTN_MAX_NODES)
    {
        iprintln(trNODE|trALWAYS, "#No free slots available for address 0x%02X", addr);
        return 0x00; //No free slots available
    }

    uint8_t slot_index = nodes.cnt;
    //iprintln(trNODE, "#Registering 0x%02X at slot %d", addr, nodes.cnt);

    //LEt's assume this is going to be all good
    memset(&nodes.list[nodes.cnt], 0, sizeof(slave_node_t)); //Reset the slot to zero
    nodes.list[nodes.cnt].address = addr; //Set the address of this node
    nodes.cnt++; //Increment the node count

    init_node_msg(slot_index); //Initialize the message for this node
    if (add_node_msg_register(slot_index))
    {
        if (node_msg_tx_now(slot_index))
        {
            //iprintln(trNODE, "#Registered 0x%02X @ %d (%d/%d nodes)", nodes.list[slot_index].address, slot_index, nodes.cnt, rollcall.cnt);
            return addr; //Continue to the next address
        }
    }
    memset(&nodes.list[slot_index], 0, sizeof(slave_node_t)); //Reset the slot to zero
    nodes.cnt--; //Decrement the node count... this registration failed
    iprintln(trNODE|trALWAYS, "#Registration failed for 0x%02X @ %d (%d/%d nodes)", addr, slot_index, nodes.cnt, rollcall.cnt);
    return 0x00;
}

int node_count(void)
{
    return nodes.cnt;
    // int count = 0;
    // for (int i = 0; i < nodes.cnt; i++)
    //     count++; //Count the number of registered buttons

    // return count;
}

void init_node_msg(uint8_t node/*, bool reset_response_data*/)
{
    if (!is_node_valid(node))
        return;

    comms_tx_msg_init(&nodes.list[node].msg, nodes.list[node].address); //Initialize the message for this node

    // if (!reset_response_data)
    //     return; //We don't want to reset the response data, so we can just return

    nodes.list[node].responses.cnt = 0; //Reset the command count for this node
    nodes.list[node].responses.expiry = 0; //Reset the expiry time
    nodes.list[node].responses.retry_cnt = 0; //Reset the retry count
    nodes.list[node].responses.exp_rx_len = 0; // The length of the response data we are waiting for 
    nodes.list[node].responses.exp_rx_cnt = 1; // The number response messages we are expecting (minimum is 1)

    memset(nodes.list[node].responses.cmd_data, 0, sizeof(nodes.list[node].responses.cmd_data)); //Reset the command data list
}

bool node_msg_tx_now(uint8_t node)
{
    if (!is_node_valid(node))
        return false;

    //We should only set the expiry time for the message at this point...
    nodes.list[node].responses.expiry = (uint64_t)sys_poll_tmr_ms() + (nodes.list[node].responses.exp_rx_cnt * CMD_RESPONSE_TIMEOUT_MS);

    if (!comms_tx_msg_send(&nodes.list[node].msg)) //Send the message immediately
    {
        iprintln(trNODE, "#Error: Could not send message to node %d (0x%02X)", node, nodes.list[node].address);
        return false; //Failed to send the message
    }

    //This function will block whichever task it is called from until the response is received or the timeout expires.
    //The responses are handled in the main APP task, so this should NOT be called from the APP task, as it will block the console task and not allow the APP task to process the responses.

    //iprintln(trALWAYS, "Waiting for response from 0x%02X (%d cmd%s)", nodes.list[node].address, nodes.list[node].responses.cnt, (nodes.list[node].responses.cnt > 1) ? "s" : "");
    while (_responses_pending(node) > 0)
    {
        //Just wait here... and keep on processing the responses... the response handler will do retries on timeouts
        node_parse_rx_msg(); //Process any received messages, this will also update the nodes.list[x].btn fields with the responses
        //if a timeout occurs after all the retries, the node will be de-registered and the while loop will exit
        vTaskDelay(1); //Wait a bit before checking again
    }

    return true; //Response received
}

bool add_node_msg_register(uint8_t node)
{
    return _add_cmd_to_node_msg(node, cmd_set_bitmask_index, &node, true);
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
    * 3. Check if the command was received OK, and then change the address in the nodes.list
    * 4. If the command was not received OK, we need ... what? try again?... how many times?
    * 
    * 
    * Another question to consider... why do we need to assign a new address to a node/button?
     */

     return _add_cmd_to_node_msg(node, cmd_new_add, &new_addr, true);
}

// bool is_direct_node_cmd(master_command_t cmd)
// {
//     //Check if the command can be broadcast
//     for (int i = 0; i < ARRAY_SIZE(cmd_table); i++)
//         if (cmd_table[i].cmd == cmd)//Found the command, so return true if it can be broadcast
//             return ((cmd_table[i].access_flags & CMD_TYPE_DIRECT) == CMD_TYPE_DIRECT)? true : false;

//     return false; //Command not found
// }

bool add_node_msg_set_rgb(uint8_t node, uint8_t index, uint32_t rgb_col)
{
    if (index > 2)
    {
        iprintln(trNODE, "#Invalid RGB index (%d)", index);
        return false;
    }
    return _add_cmd_to_node_msg(node, cmd_set_rgb_0 + index, (uint8_t *)&rgb_col, false);
}
bool add_node_msg_get_rgb(uint8_t node, uint8_t index)
{
    if (index > 2)
    {
        iprintln(trNODE, "#Invalid RGB index (%d)", index);
        return false;
    }
    return _add_cmd_to_node_msg(node, cmd_get_rgb_0 + index, NULL, false);
}

bool add_node_msg_set_blink(uint8_t node, uint32_t period_ms)
{
    return _add_cmd_to_node_msg(node, cmd_set_blink, (uint8_t *)&period_ms, false);
}
bool add_node_msg_get_blink(uint8_t node)
{
    return _add_cmd_to_node_msg(node, cmd_get_blink, NULL, false);
}

bool add_node_msg_set_dbgled(uint8_t node, uint8_t state)
{
    return _add_cmd_to_node_msg(node, cmd_set_dbg_led, (uint8_t *)&state, false);
}
bool add_node_msg_get_dbgled(uint8_t node)
{
    return _add_cmd_to_node_msg(node, cmd_get_dbg_led, NULL, false);
}

bool add_node_msg_set_active(uint8_t node, bool start)
{
    uint8_t payload = start? 0x01 : 0x00; //Default command to start the stopwatch
    return _add_cmd_to_node_msg(node, cmd_set_switch, &payload, false);
}
bool add_node_msg_get_reaction(uint8_t node)
{
    return _add_cmd_to_node_msg(node, cmd_get_reaction, NULL, false);
}

bool add_node_msg_get_flags(uint8_t node)
{
    return _add_cmd_to_node_msg(node, cmd_get_flags, NULL, false);
}

bool add_node_msg_set_time(uint8_t node, uint32_t new_time_ms)
{
    return _add_cmd_to_node_msg(node, cmd_set_time, (uint8_t *)&new_time_ms, false);
}
bool add_node_msg_get_time(uint8_t node)
{
    return _add_cmd_to_node_msg(node, cmd_get_time, NULL, false);
}

bool add_node_msg_sync_reset(uint8_t node)
{
    uint32_t reset_value = 0xFFFFFFFF; //Reset value for the sync command
    return _add_cmd_to_node_msg(node, cmd_set_sync, (uint8_t *)&reset_value, false);
}
bool add_node_msg_sync_start(uint8_t node)
{
    sys_stopwatch_ms_start(&sync_stopwatch, 0xFFFFFFFE); //Start the stopwatch for the sync command
    uint32_t start_value = 0x0; //Start value for the sync command
    return _add_cmd_to_node_msg(node, cmd_set_sync, (uint8_t *)&start_value, false);
}
bool add_node_msg_sync_end(uint8_t node)
{
    uint32_t stop_value = sys_stopwatch_ms_stop(&sync_stopwatch); //Stop the sync and provide our elapsed time
    return _add_cmd_to_node_msg(node, cmd_set_sync, (uint8_t *)&stop_value, false);
}
bool add_node_msg_get_correction(uint8_t node)
{
    return _add_cmd_to_node_msg(node, cmd_get_sync, NULL, false);
}

bool add_node_msg_get_version(uint8_t node)
{
    return _add_cmd_to_node_msg(node, cmd_get_version, NULL, false);
}

size_t cmd_mosi_payload_size(master_command_t cmd)
{
    //means the response is OK, so we need to return the payload size based on the command
    for (int i = 0; i < ARRAY_SIZE(cmd_table); i++)
    {
        if (cmd_table[i].cmd == cmd)
        {
            //Found the command, so return the payload size
            return cmd_table[i].mosi_sz;
        }
    }
    return 0; //the remainder of data in the message (minus the 2 bytes for the response code and command)
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
        case cmd_set_sync:              return "set_sync";
        case cmd_new_add:               return "new_add";
        case cmd_get_rgb_0:             return "get_rgb_0";
        case cmd_get_rgb_1:             return "get_rgb_1";
        case cmd_get_rgb_2:             return "get_rgb_2";
        case cmd_get_blink:             return "get_blink";
        case cmd_get_reaction:          return "get_sw_time";
        case cmd_get_flags:             return "get_flags";
        case cmd_get_dbg_led:           return "get_dbg_led";
        case cmd_get_time:              return "get_time";
        case cmd_get_sync:              return "get_sync";
        case cmd_get_version:           return "get_version";
#if REMOTE_CONSOLE_SUPPORTED == 1    
        case cmd_wr_console_cont:       return "wr_console_cont";
        case cmd_wr_console_done:       return "wr_console_done";
#endif /* REMOTE_CONSOLE_SUPPORTED */
        case cmd_debug_0:               return "debug_0";
        default:                        return "unknown";
    }
}

// bool is_bcst_cmd(master_command_t cmd)
// {
//     //Check if the command can be broadcast
//     for (int i = 0; i < ARRAY_SIZE(cmd_table); i++)
//         if (cmd_table[i].cmd == cmd)//Found the command, so return true if it can be broadcast
//             return ((cmd_table[i].access_flags & CMD_TYPE_BROADCAST) == CMD_TYPE_BROADCAST)? true : false;

//     return false; //Command not found
// }

void init_bcst_msg(void)
{
    uint32_t _bcst_mask = _inactive_nodes_mask(); //Start with no excluded nodes (apart from the active node)
    comms_tx_msg_init(&bcst_msg, ADDR_BROADCAST); //Initialize the broadcast message
    comms_tx_msg_append(&bcst_msg, ADDR_BROADCAST, cmd_bcast_address_mask, (uint8_t *)&_bcst_mask, sizeof(uint32_t), true); //Append the cmd_bcast_address_mask command
}

void bcst_msg_tx_now(void)
{
    //Apart from Roll-calls, broadcast messages are essentially "fire and forget" messages, so we don't need to wait for a response
    if (!comms_tx_msg_send(&bcst_msg))
        iprintln(trNODE, "#Error: Could not send broadcast (0x%02X)", bcst_msg.msg.hdr.id);

    //If one of the commands was the "activate" command, we need to set the appropriate flag for all the currently inactive nodes... 
}

bool add_bcst_msg_set_rgb(uint8_t index, uint32_t rgb_col)
{
    if (index > 2)
    {
        iprintln(trNODE, "#Invalid RGB index (%d)", index);
        return false;
    }
    return _bcst_append(cmd_set_rgb_0 + index, (uint8_t *)&rgb_col);
}

bool add_bcst_msg_set_blink(uint32_t period_ms)
{
    return _bcst_append(cmd_set_blink, (uint8_t *)&period_ms);
}

// bool add_bcst_msg_activate(bool activate)
// {
//     uint8_t payload = activate? CMD_SW_PAYLOAD_ACTIVATE : CMD_SW_PAYLOAD_DEACTIVATE; //Default command to start the stopwatch
//     //This is a very special case, as this will activate ALL the nodes on the network.... but we don't know if they were really activated... we just have to trust that they were.
//     return _bcst_append(cmd_set_switch, &payload);
// }

bool add_bcst_msg_set_dbgled(uint8_t dbg_blink_state)
{
    return _bcst_append(cmd_set_dbg_led, (uint8_t *)&dbg_blink_state);
}

bool add_bcst_msg_set_time_ms(uint32_t new_time_ms)
{
    return _bcst_append(cmd_set_dbg_led, (uint8_t *)&new_time_ms);
}

bool add_bcst_msg_sync_reset(void)
{
    uint32_t reset_value = 0xFFFFFFFF; //Reset value for the sync command
    return _bcst_append(cmd_set_sync, (uint8_t *)&reset_value);
}
bool add_bcst_msg_sync_start(void)
{
    sys_stopwatch_ms_start(&sync_stopwatch, 0xFFFFFFFE); //Start the stopwatch for the sync command
    uint32_t start_value = 0x0; //Start value for the sync command
    return _bcst_append(cmd_set_sync, (uint8_t *)&start_value);
}
bool add_bcst_msg_sync_end(void)
{
    uint32_t stop_value = sys_stopwatch_ms_stop(&sync_stopwatch); //Stop the sync and provide our elapsed time
    return _bcst_append(cmd_set_sync, (uint8_t *)&stop_value);
}
bool is_time_sync_busy(void)
{
    //Check if the sync stopwatch is running
    return sync_stopwatch.running;
}

uint32_t _inactive_nodes_mask(void)
{
    uint32_t mask = 0x00000000; //Start with all nodes inactive
    for (int i = 0; i < nodes.cnt; i++)
        if (nodes.list[i].active == false)
            mask |= (1 << i); //Set the bit for this node

    return mask; //Return the mask of inactive nodes (which are valid)
}

int active_node_count(void)
{
    int count = 0;
    for (int i = 0; i < nodes.cnt; i++)
        if (nodes.list[i].active == true)
            count++; //Count the number of registered buttons

    if (count > 1)
        iprintln(trNODE|trALWAYS, "#Error - %d active nodes found", count);

    return count;
}

button_t * get_node_button_ptr(int slot)
{
    if (!is_node_valid(slot)) //Check if the slot is valid and has a registered button
        return NULL; //Skip this slot

    return &nodes.list[slot].btn; //Return a poitner to the button for this node
}

uint32_t get_node_btn_version(int slot)
{
    return (!is_node_valid(slot))? 0 : nodes.list[slot].btn.version; //Return the version of the button
}
uint32_t get_node_btn_colour(int slot, int col_index)
{
    return ((!is_node_valid(slot)) || (col_index < 0) || (col_index >= 3))? 0 : nodes.list[slot].btn.rgb_colour[col_index]; //Return the RGB colour for the button
}
uint32_t get_node_btn_blink_per_ms(int slot)
{
    return (!is_node_valid(slot))? 0 : nodes.list[slot].btn.blink_ms; //Return the blink period for the button
}
uint32_t get_node_btn_reaction_ms(int slot)
{
    return (!is_node_valid(slot))? 0 : nodes.list[slot].btn.reaction_ms; //Return the reaction period for the button
}
uint32_t get_node_btn_time_ms(int slot)
{
    return (!is_node_valid(slot))? 0 : nodes.list[slot].btn.time_ms; //Return the time period for the button
}
uint8_t get_node_btn_flags(int slot)
{
    return (!is_node_valid(slot))? 0 : nodes.list[slot].btn.flags; //Return the flags for the button
}
float get_node_btn_correction_factor(int slot)
{
    return (!is_node_valid(slot))? 0.0f : nodes.list[slot].btn.time_factor; //Return the correction factor for the button
}
bool get_node_btn_sw_state(int slot)
{
    return (!is_node_valid(slot))? false : nodes.list[slot].btn.sw_active; //Return the switch state for the button
}
dbg_blink_state_t get_node_btn_dbg_led_state(int slot)
{
    return (!is_node_valid(slot))? dbg_led_off : nodes.list[slot].btn.dbg_led_state; //Return the debug LED state for the button
}


int _add_rc_address(uint8_t addr)
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
                break; //From for loop... Address already in the list
            }

            if (rollcall.list[i] == 0x00)
            {
                //Empty slot found, so we can add the address here
                rollcall.list[rollcall.cnt++] = addr; //Add the address to the list and increment the count
                rollcall.list[rollcall.cnt] = 0x00; //Set the next slot to zero, so we can check for empty slots

                break; //From for loop
            }
        }
    }
    return rollcall.cnt; //Return the number of addresses in the roll-call list
}

bool _waiting_for_rollcall(bool blocking)
{
    //This call will block the calling task until the roll-call timer expires

    if (!sys_poll_tmr_expired(&rollcall.timer))
    {
        if (!blocking)
            return true;
        
        uint64_t wait_time = rollcall.timer.ms_expire - sys_poll_tmr_ms();
        //We could still received roll-call responses, so we cannot register nodes yet
        // iprintln(trNODE, "#Roll-call will complete in %lu.%03d s.", wait_time / 1000, (int)(wait_time % 1000));

        while (!sys_poll_tmr_expired(&rollcall.timer))
        {
            node_parse_rx_msg(); //Process any received messages, this will update the nodes.list with the responses
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
            _resp_data_len = _miso_payload_size(_cmd, _resp); //The rest of the payload is the data
            _resp_data_len = MIN(_resp_data_len, payload_len - _data_idx); //The rest of the payload is the data
            //iprintln(trNODE, "#%d bytes payload for \"%s\" (Resp: 0x%02X, Total: %d, Index: %d)", _resp_data_len, cmd_to_str(_cmd), _resp, payload_len, _data_idx);
            _data_idx += _resp_data_len; //Move the data index forward by the command data length

            //We are expecting response messages in only 2 instances:
            // 1) Rollcall responses
            // 2) Responses to directed commands sent to a node
            if (_cmd == cmd_roll_call)
            {
                _rollcall_handler(rx_msg.hdr.src); //Handle the roll-call response
                //RVN - should we return from here? or continue processing the rest of the message?
                return;
            }
            else
            {
                int node_slot = -1;
                //iprintln(trNODE, "#RX 0x%02X for \"%s\" from Node 0x%02X. %d bytes", _resp, cmd_to_str(_cmd), rx_msg.hdr.src, _resp_data_len);
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

bool nodes_register_all(void)
{
    if (!_bcst_rollcall(true))
    {
        iprintln(trNODE|trALWAYS, "#Failed to send rollcall");
        return false; //Failed to send the roll-call message, so we can't register new buttons
    }

    iprintln(trNODE|trALWAYS, "#Registration will complete in %.03f s.", (rollcall.timer.ms_expire - sys_poll_tmr_ms()) / 1000.0);
    // uint8_t _addr = 0x00; //Start with the first address

    //This call will block the console task until the roll-call timer expires, so we can register nodes
    _waiting_for_rollcall(true);

    for (int i = 0; i < rollcall.cnt; i++)
    {
        if (_register_addr(rollcall.list[i]) != rollcall.list[i])
            iprintln(trNODE|trALWAYS, "#Failed to register node 0x%02X", (uint8_t)rollcall.list[0]);
    }
    iprintln(trNODE, "#Registered %d/%d nodes", node_count(), rollcall.cnt);
    return (node_count() > 0)? true : false; //Return true if we have any registered nodes
}

void bcst_msg_clear_all(void)
{
    init_bcst_msg();
    add_bcst_msg_set_blink(0); //Turn off blinking
    add_bcst_msg_set_rgb(0, 0);
    add_bcst_msg_set_rgb(1, 0);
    add_bcst_msg_set_rgb(2, 0);
    add_bcst_msg_set_dbgled(dbg_led_off); //Turn off the debug LED
    bcst_msg_tx_now();
}

#undef PRINTF_TAG
#undef EXT
/*************************** END OF FILE *************************************/
