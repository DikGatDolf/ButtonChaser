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
void hal_serial_init(void);

int hal_serial_available(void);

int hal_serial_read(void);

/*! Returns the amount of time since activity on the bus was detected.
 * Will max out on 60 seconds (60,000ms)
 * @return The amount of time (in ms) since activity on the bus was detected.
 */
uint16_t hal_serial_rx_silence_ms(void);

/*! Flushes the serial port. Only returns once all TX 
 */
void hal_serial_flush(void);

/*! Writes a byte to the serial port
 * @param[in] c The byte to write
 * @return The number of bytes written
 */
size_t hal_serial_write(uint8_t c);

/*! Returns true if the serial port is busy with a transmission
 * @return true if the serial port is busy with a transmission
 */
bool hal_serial_tx_busy(void);

#endif /* __hal_serial_H__ */

/****************************** END OF FILE **********************************/
