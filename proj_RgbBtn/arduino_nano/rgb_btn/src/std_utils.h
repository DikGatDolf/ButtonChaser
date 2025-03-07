/*****************************************************************************

std_utils.h

Include file for std_utils.c

******************************************************************************/
#ifndef __STD_UTILS_H__
#define __STD_UTILS_H__


/******************************************************************************
includes
******************************************************************************/
#include <avr/pgmspace.h>
#include "Arduino.h"
#include "defines.h"

/******************************************************************************
definitions
******************************************************************************/

#define sign_f(a)	((a < 0)? -1.0 : 1.0)
#define INPUT_EDGE_NONE		0
#define INPUT_EDGE_FALLING	1
#define INPUT_EDGE_RISING	2

#define CRC_POLYNOMIAL 0x8C

/******************************************************************************
Macros
******************************************************************************/

#define ARRAY_SIZE(_array) (sizeof(_array) / sizeof((_array)[0]))

/******************************************************************************
Struct & Unions
******************************************************************************/
typedef struct
{
	//bool States[5];
	int CurrentState;
	int DebounceCount;
}ST_PIN_DEBOUNCE;

typedef struct
{
	//bool States[5];
	byte flagMask;
	const char * Name;
}ST_PRINT_FLAG_ITEM;

/******************************************************************************
variables
******************************************************************************/
const PROGMEM int pinDEBUG_0 	= 14;
const PROGMEM int pinDEBUG_1 	= 15;
const PROGMEM int pinDEBUG_2 	= 16;
const PROGMEM int pinDEBUG_3 	= 17;
const PROGMEM int pinDEBUG_4 	= 18;
const PROGMEM int pinDEBUG_5 	= 19;

/******************************************************************************
functions
******************************************************************************/
/*! An attempt to manage my outpts without the use of digitalWrite and a subsequent
 *  f@#$ing about with clearing the interrupt flag
 * @param[in] pin The pin to toggle
 * @param[in] state The state to set the pin to
*/
void quickPinToggle(uint8_t pin, bool state);
int quickPinRead(uint8_t pin);
int debounceInput(ST_PIN_DEBOUNCE * input, int level, int count);
unsigned long avgULong(volatile unsigned long * arr, int cnt);
int freeRam (void);

byte crc8_str(const char *str);
byte crc8_str(byte crc_start, const char *str);
byte crc8_str_n(const byte *data, byte len);
byte crc8_str_n(byte crc_start, const byte *data, byte len);
byte crc8(byte crc_start, byte data);

#endif /* __STD_UTILS_H__ */

/****************************** END OF FILE **********************************/
