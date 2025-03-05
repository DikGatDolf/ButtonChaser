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

//This is the firmware version of this Button Controller. The high nibble denotes 
//the major version, while the low nibble denotes the minor version. 0x10 => V1.0
#define BTN_CTRL_VERSION	0x10

/* The "CONSOLE_ENABLED" define includes all the Console's menu interfacing and
 * such like. This also takes up a lot of the available codespace. */
#define CONSOLE_ENABLED

/* The "USE_WAV_GEN" define includes the waveform generator code. The waveform
 * generator provides configurable waveforms on the DAC output: Sine, square,
 * triangle, sawtooth (falling), or sawtooth (rising).
 * NOTE: This is only available with the Console Menu enabled */
#ifdef CONSOLE_ENABLED
	//#define USE_WAV_GEN
#endif

/* The "MAIN_DEBUG" define includes/excludes various methods and functions which
 * only have a purpose for debugging. A decision had to be made to conserve
 * codespace and these are the guys who fell under the axe.
 * NOTE: This is only available with the Console Menu enabled */
#ifdef CONSOLE_ENABLED
	#define MAIN_DEBUG
#endif

