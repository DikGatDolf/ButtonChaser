/*****************************************************************************

drv_rgb_btn.h

Include file for drv_rgb_btn.c

******************************************************************************/
#ifndef __drv_rgb_btn_H__
#define __drv_rgb_btn_H__

#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************
includes
******************************************************************************/
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
void drv_rgb_btn_init(void);

/*! \brief De-initialise the RGB Button driver
 */
void drv_rgb_btn_deinit(void);

/*! \brief Probe the RGB Button device to see if it is present on the bus
 */
esp_err_t drv_rgb_btn_probe(uint8_t id_mask);

/*! \brief Transmit data to the RGB Button device
 */
esp_err_t drv_rgb_btn_tx(uint8_t *tx_data, size_t tx_size);

/*! \brief Transmit and receive data to/from the RGB Button device
 */
esp_err_t drv_rgb_btn_txrx(uint8_t *tx_data, size_t tx_size, uint8_t *rx_data, size_t rx_size);


#ifdef __cplusplus
}
#endif

#undef EXT
#endif /* __drv_rgb_btn_H__ */

/****************************** END OF FILE **********************************/
