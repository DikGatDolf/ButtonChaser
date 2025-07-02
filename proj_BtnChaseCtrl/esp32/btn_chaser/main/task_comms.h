/*****************************************************************************

task_comms.h

Include file for task_comms.c

******************************************************************************/
#ifndef __task_comms_H__
#define __task_comms_H__

#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************
includes
******************************************************************************/
#include "defines.h"
#include "sys_utils.h"
#include "../../../../common/common_comms.h"

/******************************************************************************
definitions
******************************************************************************/
#ifdef __NOT_EXTERN__
#define EXT
#else
#define EXT extern
#endif /* __NOT_EXTERN__ */

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

/*! \brief Initialise the RGB Button driver
 */
void * comms_init_task(void);

/*! \brief Transmit data to the RGB Button device
 */
esp_err_t task_comms_tx(uint8_t *tx_data, size_t tx_size);

bool comms_msg_rx_read(comms_msg_t *msg, size_t *msg_size);

bool comms_rollcall(bool all);

void comms_node_msg_init(uint8_t node_addr);
bool comms_node_msg_append(uint8_t node_addr, master_command_t cmd, uint8_t * data, uint8_t data_len, bool restart);

bool comms_bcst_start(uint32_t exclude_bit_mask_addr);
bool comms_bcst_append(uint8_t cmd, uint8_t * data, uint8_t data_len);

bool comms_msg_tx_now(void);

bool comms_bcst_set_rgb(uint8_t index, uint32_t rgb_col);
bool comms_bcst_set_blink(uint32_t period_ms);

// bool comms_node_register(uint8_t node_addr, uint8_t bit_mask_addr);
// bool comms_node_new_addr(uint8_t node_addr, uint8_t new_addr);

// bool comms_node_set_rgb(uint8_t node_addr, uint8_t index, uint32_t rgb_col);
// bool comms_node_get_rgb(uint8_t node_addr, uint8_t index);

// bool comms_node_set_blink(uint8_t node_addr, uint32_t period_ms);
// bool comms_node_get_blink(uint8_t node_addr);

// bool comms_node_start_sw(uint8_t node_addr);
// bool comms_node_get_sw_time(uint8_t node_addr);

// bool comms_node_get_flags(uint8_t node_addr);

#ifdef __cplusplus
}
#endif

#undef EXT
#endif /* __task_comms_H__ */

/****************************** END OF FILE **********************************/
