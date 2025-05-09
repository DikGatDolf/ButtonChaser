/*****************************************************************************

hal_serial.h

Include file for hal_serial.c

******************************************************************************/
#ifndef __hal_serial_H__
#define __hal_serial_H__

/******************************************************************************
includes
******************************************************************************/
#include "Arduino.h"

/******************************************************************************
definitions
******************************************************************************/
typedef enum
{
    rx_good     = 0,
    rx_overflow = 1,
    rx_framing  = 2,
} hal_serial_rx_result_t;

/******************************************************************************
Macros
******************************************************************************/

/******************************************************************************
Struct & Unions
******************************************************************************/

/******************************************************************************
variables
******************************************************************************/

/******************************************************************************
functions
******************************************************************************/

/*! Initializes the serial port
 * @param[in] cb_rx_irq The callback function to call when a byte is received
 * @note This function must be called before any other serial functions
 */
void hal_serial_init(void (*cb_rx_irq)(uint8_t));

/*! Flushes the serial port. Only returns once all TX 
 */
void hal_serial_flush(void);

/*! Writes a byte to the serial port
 * @param[in] c The byte to write
 * @return The number of bytes written
 */
size_t hal_serial_write(uint8_t c);

#endif /* __hal_serial_H__ */

/****************************** END OF FILE **********************************/
