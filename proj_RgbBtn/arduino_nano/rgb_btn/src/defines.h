#ifndef __defines_H__
#define __defines_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "Arduino.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <avr/io.h>


/*******************************************************************************
defines
 *******************************************************************************/

const char BUILD_TIME_AND_DATE[] = {__TIME__ " " __DATE__}; /* Used in our startup Banner*/
 
 //This is the firmware version of this Button Controller. The high nibble denotes 
//the major version, while the low nibble denotes the minor version. 0x10 => V1.0
#define PROJECT_VERSION	    0x10

#define btnChasePin_RX0     (0)
#define btnChasePin_TX0     (1)

#define output_RS485_DE     (3)
#define output_RS485_RE     (4)

#define input_Button        (2)     /* Source for INT0 */

#define output_Led_Red      (13)
#define output_Led_Green    (14)
#define output_Led_Blue     (15)

#define input_ADC           (20)

//#define output_Debug        (12)
// #define btnChasePin_SDA     (18)
// #define btnChasePin_SCL     (19)

/* The "CONSOLE_ENABLED" define includes all the Console's menu interfacing and
 * such like. This also takes up a lot of the available codespace. */
#define CONSOLE_ENABLED         1
#define CONSOLE_ECHO_ENABLED    0

#define REDUCE_CODESIZE     1

/* The "MAIN_DEBUG" define includes/excludes various methods and functions which
 * only have a purpose for debugging. A decision had to be made to conserve
 * codespace and these are the guys who fell under the axe.
 * NOTE: This is only available with the Console Menu enabled */
#ifdef CONSOLE_ENABLED
	#define MAIN_DEBUG
#endif

/* Suppresses a bunch of debug prints and console functionality for the RGB LED driver:
    Usage: RAM: 60 bytes, Flash: 2084 bytes
*/
#define DEV_RGB_DEBUG       (0)
/* Suppresses a bunch of debug prints and console functionality for the Button driver:
    Usage: RAM: 30 bytes, Flash: 310 bytes
*/
#define DEV_BTN_DEBUG       (0)


#ifdef __cplusplus
}
#endif

#endif /* __dev_nvstore_H__ */

/****************************** END OF FILE **********************************/
