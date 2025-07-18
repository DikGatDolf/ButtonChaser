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
 
#define btnChasePin_RX0     (0)
#define btnChasePin_TX0     (1)

#define output_RS485_DE     (3)
#define output_DEBUG_LED    (13)

#define input_Button        (2)     /* Source for INT0 */

#define output_Led_Blue     (14)
#define output_Led_Red      (15)
#define output_Led_Green    (16)

#define input_ADC           (20)

//#define output_Debug        (12)
// #define btnChasePin_SDA     (18)
// #define btnChasePin_SCL     (19)

/* The "CONSOLE_ENABLED" define includes all the Console's menu interfacing and
 * such like. This also takes up a lot of the available codespace. */
#define CONSOLE_ENABLED         1
#define CONSOLE_ECHO_ENABLED    0

#define REDUCE_CODESIZE     1

#define DEBUG_LED_ACTIONS   (0)
/* The "MAIN_DEBUG" define includes/excludes various methods and functions which
 * only have a purpose for debugging. A decision had to be made to conserve
 * codespace and these are the guys who fell under the axe.
 * NOTE: This is only available with the Console Menu enabled */
#ifdef CONSOLE_ENABLED
	#define MAIN_DEBUG
#endif

#define DEV_COMMS_DEBUG       (0)

/* Suppresses a bunch of debug prints and console functionality for the RGB LED driver:
    Usage: RAM: 60 bytes, Flash: 2084 bytes */
#define DEV_RGB_DEBUG       (0)

/* Suppresses a bunch of debug prints and console functionality for the Button driver:
    Usage: RAM: 30 bytes, Flash: 310 bytes */
#define DEV_BTN_DEBUG       (1)

/* Suppresses a bunch of debug prints and console functionality for the NVStore driver:
    Usage: RAM: 134 bytes, Flash: 3,112 bytes */
#define DEV_NVSTORE_DEBUG       (1)

#ifdef __cplusplus
}
#endif

#endif /* __dev_nvstore_H__ */

/****************************** END OF FILE **********************************/
