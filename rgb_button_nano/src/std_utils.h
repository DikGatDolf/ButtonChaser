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

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned long u32;
typedef signed short s16;
typedef signed long s32;

#define sign_f(a)	((a < 0)? -1.0 : 1.0)
#define INPUT_EDGE_NONE		0
#define INPUT_EDGE_FALLING	1
#define INPUT_EDGE_RISING	2


#define CRC_POLYNOMIAL 0x8C

#define trMAIN		0x01
#define trCONSOLE	0x02
#define trRGB		0x04
#define trBUTTON	0x08
#define trI2C		0x10
#define trPWM		0x20
#define trYYY		0x40

#define MAX_TRACE_FLAGS	6

#define trALWAYS	0x80
#define trALL		0xFF
#define trNONE		0x00

#define statusOK			0x01	/* Done */
//#define statusMOVING		0x02	/* Done */
//#define statusDIRECTION		0x04	/* Done */
//#define statusPID_BUSY		0x08	/* Done */
//#define statusPID_DONE		0x10	/* Done */
//#define statusCALIB_BUSY	0x20 	/* Done */
//#define status			0x40 	/* Done */
//#define status 			0x80 	/* Done */

//#define stateIDLE			1
//#define stateCAL_SEARCH		2
//#define stateCAL_GOTO_0		3
//#define state			0x0
//#define state			0x0
//#define state			0x0
//#define state			0x0

//#define status	0x20
//#define status	0x40
//#define status	0x80


/******************************************************************************
Macros
******************************************************************************/

//#ifdef CONSOLE_ENABLED
//	#define iPrintF(traceflags, fmt, ...) _SerialPrintf(traceflags, PSTR(fmt), ##__VA_ARGS__)
//	#define PrintF(fmt, ...) _SerialPrintf(PSTR(fmt), ##__VA_ARGS__)
//#else
//	#define iPrintF(traceflags, fmt, ...) devComms::DoNothing(traceflags, PSTR(fmt), ##__VA_ARGS__) /* {	}  */
//	#define PrintF(fmt, ...) devComms::_SerialPrintf(PSTR(fmt), ##__VA_ARGS__)
//#endif

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
int setFloatParam(float * param, float value, float min_Limit, float max_Limit);

#ifdef CONSOLE_ENABLED
	int setFloatParam(char * name, char * paramStr, char * valueStr, float * param, float min_Limit, float max_Limit);
	int setFloatParam(char * name, char * paramStr, char * valueStr, float * param);
	float FloatMathStr(char * Src, float floatValue);
#endif /* CONSOLE_ENABLED */

float FloatMathStr(char * Src, float value, bool absolute);
char * floatToStr(float val, int precision);
//	char * floatToStr(float fVal, int precision, char * dst, int maxlen);
float FloatMathStr(char * Src, float value);
bool isHexStr(char * str);
byte hexToByte(char * hexDigits);
byte charToNibble(char hexDigit);
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

//char * SatusWordBinStr();
char * TmpStrPrintf(char *dst, int len, const char *fmt, ...);
char * TmpStrPrintf(const char *fmt, ...);

bool isNaturalNumberStr(char * str);
bool isFloatStr(char * str);

void SetStatus(byte statusMask);
void ClearStatus(byte statusMask = 0xFF);
void ToggleStatus(byte statusMask, bool flag);
bool GetStatus(byte statusMask);

#endif /* __STD_UTILS_H__ */

/****************************** END OF FILE **********************************/
