/*****************************************************************************

nodes.h

Include file for nodes.c

******************************************************************************/
#ifndef __nodes_H__
#define __nodes_H__

#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************
includes
******************************************************************************/
#include "defines.h"
#include "../../../../common/common_comms.h"

/******************************************************************************
definitions
******************************************************************************/
#ifdef __NOT_EXTERN__
#define EXT
#else
#define EXT extern
#endif /* __NOT_EXTERN__ */

#define NODE_CMD_CNT_MAX  (10) // The maximum number of commands we can send in a single message

/******************************************************************************
Macros
******************************************************************************/

/******************************************************************************
Struct & Unions
******************************************************************************/

/******************************************************************************
Global (public) variables
******************************************************************************/

/******************************************************************************
Global (public) function definitions
******************************************************************************/
/*! \brief Start a roll-call for all nodes.
 * \param all If true, the function will reset the roll-call list and start a new roll-call for all nodes.
 * This function sends a roll-call command to all nodes and waits for their responses.
 * It is used to discover all (or new) nodes in the network and register them.
 */
bool bcst_rollcall(bool all);

/*! \brief Wait for the roll-call timer to expire.
 * \param blocking If true, the function will block until the roll-call timer expires.
 * This function blocks the calling task until the roll-call timer expires.
 * It is used to ensure that all roll-call responses are received before proceeding with other functions such as registering nodes.
 */
bool waiting_for_rollcall(bool blocking);

/*! \brief Register new buttons (nodes) in the system.
 * This function checks the roll-call list for new addresses and registers them as nodes.
 * It will send a register command to each new node found in the roll-call list.
 */
void register_new_buttons(void);

/*! \brief Get the address of a registered node.
 * \param node The index of the node in the node_list.
 * \return The address of the node, or ADDR_BROADCAST if the node is invalid.
 */
uint8_t get_node_addr(uint8_t node);

/*! \brief Get the number of registered nodes.
 * This function counts the number of nodes that have a valid address in the node_list.
 */
int node_count(void);

/*! \brief Check if a node is valid.
 * \param node The index of the node in the node_list.
 * \return True if the node is valid, false otherwise.
 */
bool is_node_valid(uint8_t node);

/*! \brief returns the number of active nodes.
 * This function counts the number of nodes that are currently active (stopwatch running, blinking, etc.).
 * It is used to determine how many nodes are currently participating in the game.
 */
int active_node_count(void);

button_t * get_node_button_ptr(int slot);


bool node_parse_rx_msg(void);

size_t cmd_mosi_payload_size(master_command_t cmd);
const char * cmd_to_str(master_command_t cmd);


/*** Node Message ****/
//bool is_direct_node_cmd(master_command_t cmd);

void init_node_msg(uint8_t node/*, bool reset_response_data*/);

bool add_node_msg_register(uint8_t node);
bool add_node_msg_new_addr(uint8_t node, uint8_t new_addr);
bool add_node_msg_set_rgb(uint8_t node, uint8_t index, uint32_t rgb_col);
bool add_node_msg_set_blink(uint8_t node, uint32_t period_ms);
bool add_node_msg_set_dbgled(uint8_t node, uint8_t state);
bool add_node_msg_set_active(uint8_t node, bool start);
bool add_node_msg_set_time(uint8_t node, uint32_t new_time_ms);
bool add_node_msg_sync_reset(uint8_t node);
bool add_node_msg_sync_start(uint8_t node);
bool add_node_msg_sync_end(uint8_t node);

bool add_node_msg_get_rgb(uint8_t node, uint8_t index);
bool add_node_msg_get_blink(uint8_t node);
bool add_node_msg_get_dbgled(uint8_t node);
bool add_node_msg_get_reaction(uint8_t node);
bool add_node_msg_get_flags(uint8_t node);
bool add_node_msg_get_time(uint8_t node);
bool add_node_msg_get_correction(uint8_t node);
bool add_node_msg_get_version(uint8_t node);

bool node_msg_tx_now(uint8_t node);

/*** Broadcast Message ****/
//bool is_bcst_cmd(master_command_t cmd);
void init_bcst_msg(int * exclude_nodes, int exclude_count);
bool add_bcst_msg_set_rgb(uint8_t index, uint32_t rgb_col);
bool add_bcst_msg_set_blink(uint32_t period_ms);
bool add_bcst_msg_set_dbgled(uint8_t dbg_blink_state);
bool add_bcst_msg_set_time_ms(uint32_t new_time_ms);
bool add_bcst_msg_sync_reset(void);
bool add_bcst_msg_sync_start(void);
bool add_bcst_msg_sync_end(void);
void bcst_msg_tx_now(void);

bool is_time_sync_busy(void);

#ifdef __cplusplus
}
#endif

#undef EXT
#endif /* __nodes_H__ */

/****************************** END OF FILE **********************************/
