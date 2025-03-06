/*****************************************************************************

rgbledstrip.h

Include file for rgbledstrip.c

******************************************************************************/
#ifndef __rgbledstrip_H__
#define __rgbledstrip_H__

#ifdef __NOT_EXTERN__
#define EXT
#else
#define EXT extern
#endif /* __NOT_EXTERN__ */

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
includes
******************************************************************************/
#include "driver/rmt_tx.h"
#include <stdint.h>
#include "defines.h"
#include "colour.h"

/******************************************************************************
Macros
******************************************************************************/
#define RGBLEDSTRIP_MAX (SOC_RMT_GROUPS * SOC_RMT_TX_CANDIDATES_PER_GROUP)

/******************************************************************************
Struct & Unions
******************************************************************************/
// typedef enum {
//     strip_Debug = 0,
//     // strip_Buttons = 1,

//     /* Keep this at the end*/
//     strips_Total = ,
// }rgbledstrip_type;

/******************************************************************************
Global (public) variables
******************************************************************************/

/******************************************************************************
Global (public) function definitions
******************************************************************************/

/*! Initialise the LED strip driver
 * @return The total number of LEDs initialised in all strips
 */
uint32_t rgbledstrip_init(void);

/*! Deinitialize the LED strip driver
 */
void rgbledstrip_deinit(void);

/*! @brief Sets the colour of the LED at the specified index
 * @param _index The index of the LED to set the colour for
 * @param _rgb The 24-bit RGB value to set the LED to
 */
void rgbledstrip_set_colour(int led_index, uint32_t rgb);

/*! Returns a pointer to a string with the name of the LED strip type
 * @param[in] led_index The index of the LED to set the colour for
 * @return A pointer to the string with the name of the LED strip type
 */
const char * rgbledstrip_index2name(int led_index);

/*! Returns the index and the number of LEDs referenced by the name
 * @param[in] name The name of the LED strip. This can be the name of the 
                strip, or the name of the strip and the number of the LED, 
                in the format "<strip_name>[:<nr>]"
 * @param[out] led_index if successfully parsed, contains the index of the 
                LED strip in the list
 * @param[out] led_cnt if successfully parsed, contains the number of LEDs. 
                If ":<nr>" was omitted from the string, this will be the 
                total number of LEDs in the strip"
 * @return true if the name was successfully parsed, false otherwise
 */
 bool rgbledstrip_name2index(const char * name, int * led_index,  int * led_cnt);

#ifdef __cplusplus
}
#endif

#undef EXT
#endif /* rgbledstrip_H */


//**************************************************************************//
