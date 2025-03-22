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
//#include <io.h>
//#include <SoftwareSerial.h>

/*******************************************************************************
defines
 *******************************************************************************/

const char BUILD_TIME_AND_DATE[] = {__TIME__ " " __DATE__}; /* Used in our startup Banner*/
 
 //This is the firmware version of this Button Controller. The high nibble denotes 
//the major version, while the low nibble denotes the minor version. 0x10 => V1.0
#define PROJECT_VERSION	    0x10

#define btnChasePin_RX0     (0)
#define btnChasePin_TX0     (1)

#define btnChasePin_BTN     (2)     /* Source for INT0 */

#define btnChasePin_RED     (13)
#define btnChasePin_GREEN   (14)
#define btnChasePin_BLUE    (15)

#define btnChasePin_SDA     (18)
#define btnChasePin_SCL     (19)

/* The "CONSOLE_ENABLED" define includes all the Console's menu interfacing and
 * such like. This also takes up a lot of the available codespace. */
#define COMMS_USE_I2C       0

/* The "CONSOLE_ENABLED" define includes all the Console's menu interfacing and
 * such like. This also takes up a lot of the available codespace. */
#define CONSOLE_ENABLED     1

/* The "USE_WAV_GEN" define includes the waveform generator code. The waveform
 * generator provides configurable waveforms on the DAC output: Sine, square,
 * triangle, sawtooth (falling), or sawtooth (rising).
 * NOTE: This is only available with the Console Menu enabled */
#ifdef CONSOLE_ENABLED
	//#define USE_WAV_GEN
#endif

#define REDUCE_CODESIZE     1

/* The "MAIN_DEBUG" define includes/excludes various methods and functions which
 * only have a purpose for debugging. A decision had to be made to conserve
 * codespace and these are the guys who fell under the axe.
 * NOTE: This is only available with the Console Menu enabled */
#ifdef CONSOLE_ENABLED
	#define MAIN_DEBUG
#endif

#ifdef __cplusplus
}
#endif

#endif /* __dev_nvstore_H__ */

/****************************** END OF FILE **********************************/
