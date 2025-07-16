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
typedef enum e_comms_msg_tx_state
{
    tx_idle,            // Waiting for a message to send
    tx_wait_for_echo,   // We are busy transmitting, checking for the rx'd echo
}comms_tx_state_t;

typedef struct {
    comms_msg_t msg;
//  uint8_t buff[(RGB_BTN_MSG_MAX_LEN*2)+2];    //Absolute worst case scenario
#if USE_BUILTIN_RS485_UART == 0    
    uint8_t retry_cnt;
#endif    
    uint8_t seq;
    size_t data_length;
    bool msg_busy;
    comms_tx_state_t state;
}comms_tx_msg_t;

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

void comms_tx_msg_init(comms_tx_msg_t * node_msg, uint8_t node_addr);
bool comms_tx_msg_append(comms_tx_msg_t * node_msg, uint8_t node_addr, master_command_t cmd, uint8_t * data, uint8_t data_len, bool restart);

bool comms_tx_msg_send(comms_tx_msg_t * tx_msg);

// bool comms_bcst_set_rgb(uint8_t index, uint32_t rgb_col);
// bool comms_bcst_set_blink(uint32_t period_ms);

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
