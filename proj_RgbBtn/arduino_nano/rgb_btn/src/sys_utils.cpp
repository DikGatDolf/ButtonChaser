/******************************************************************************
Project:   RGB Button Chaser
Module:     sys_utils.c
Purpose:    This file contains the std utilited
Author:     Rudolph van Niekerk
Processor:  Arduino Nano (ATmega328)
Compiler:	Arduino AVR Compiler


This implements timers which has to be polled to check for expiry.
The timer will act as a one-shot timer in normal operation.
To make the timer behave as a recurring timer, reload the interval and start
the timer once it has expired (using TimerStart()).

The General Timer (Timer_ms_t type) - 1 kHz granularity
The general Timers enables the program to create a downcounter with a
preloaded value. This timer will then decrement every 1 ms until it has
expired.
Usage:
The module making use of the timers must host a Timer_ms_t structure in RAM and
add it to the linked list (TimerAdd) to ensure that it is maintained.
Removing it from the linked list (TimerRemove) will  make it dormant.
The Timer must be polled (TimerPoll) to see when it has expired

 ******************************************************************************/

/*******************************************************************************
includes
 *******************************************************************************/
#include "Arduino.h"
#include <stdarg.h>

#define __NOT_EXTERN__
#include "sys_utils.h"
#undef __NOT_EXTERN__

#ifdef CONSOLE_ENABLED
	#include "dev_console.h"
#else
	#include "devComms.h"
#endif

/*******************************************************************************
local defines
 *******************************************************************************/
#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("SysUtils") /* This must be undefined at the end of the file*/

/*******************************************************************************
local variables
 *******************************************************************************/

 /*******************************************************************************

Global Functions

 *******************************************************************************/
void quickPinToggle(uint8_t pin, bool state)
{
//byte port;

	//PrintF("Toggle pin %d to %s\n", pin, (state)? "HI" : "LO");

	//Assign the correct port....
	if (pin < 8)
	{
		//port = PORTD;
		if (state)
			PORTD |= _BV(pin);
		else
			PORTD &= ~_BV(pin);
	}
	else if (pin < 14)
	{
		//port = PORTB;
		if (state)
			PORTB |= _BV(pin%8);
		else
			PORTB &= ~_BV(pin%8);
	}
	else if (pin < 19)
	{
		//port = PORTC;
		if (state)
			PORTC |= _BV(pin%14);
		else
			PORTC &= ~_BV(pin%14);
	}
}

int quickPinRead(uint8_t pin)
{
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);

	//if (port == NOT_A_PIN) return LOW;

	if (*portInputRegister(port) & bit) return HIGH;
	return LOW;
}

/*******************************************************************************

Returns one of the follwoing EDGE states:
#define INPUT_EDGE_NONE		0
#define INPUT_EDGE_FALLING	1
#define INPUT_EDGE_RISING	2

 *******************************************************************************/
int debounceInput(ST_PIN_DEBOUNCE * input, int level, int count)
{
	//Is this level the same as what was saved previously?
	if (input->CurrentState == level)
	{
		//Yup... nothing to debounce.
		input->DebounceCount = 0;
		return INPUT_EDGE_NONE;
	}

	//This means we have seen a change in level from our current "debounced" level
	input->DebounceCount++;

	//Has it been debounce "count" times?
	if (input->DebounceCount < count)
	{
		//We are still debouncing... no change to report.
		return INPUT_EDGE_NONE;
	}

	//Now we have a stable state... assign it to our current state.
	input->CurrentState = level;

	//A HIGH level implies a rising edge....
	// ...A LOW level implies a falling edge.
	return (level == HIGH)?
			INPUT_EDGE_RISING : INPUT_EDGE_FALLING;
}

/*******************************************************************************

Returns the average of all the values in the passed array

 *******************************************************************************/
// unsigned long avgULong(volatile unsigned long * arr, int cnt)
// {
// unsigned long retVal = 0L;

// 	for (int i = 0; i < cnt; i++)
// 	{
// 		retVal += arr[i];
// 	}
// 	retVal /= cnt;

// 	return retVal;
// }

/*******************************************************************************

Reports the amount of space available between the stack and the heap.
Calls to this function from different locations in the code will return a
different result.

 *******************************************************************************/
int freeRam ()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

/*******************************************************************************

Calculate the CRC for a null terminated string.

 *******************************************************************************/
// byte crc8_str(const char *str) {
// 	char * data = (char *)str;
// 	byte crc = 0x00; //Start with a blank CRC.
// 	while (*data){
// 		byte extract = *data++;
// 		for (byte tempI = 8; tempI; tempI--) {
// 			byte sum = (crc ^ extract) & 0x01;
// 			crc >>= 1;
// 			if (sum) {
// 				crc ^= CRC_POLYNOMIAL;
// 			}
// 			extract >>= 1;
// 		}
// 	}
// 	return crc;
// }

// /*******************************************************************************

// Calculate the CRC for a null terminated string with a starting seed CRC value

//  *******************************************************************************/
// byte crc8_str(byte crc_start, const char *str) {
// 	char * data = (char *)str;
// 	byte crc = crc_start; //Start with the seed CRC.
// 	while (*data){
// 		byte extract = *data++;
// 		for (byte tempI = 8; tempI; tempI--) {
// 			byte sum = (crc ^ extract) & 0x01;
// 			crc >>= 1;
// 			if (sum) {
// 				crc ^= CRC_POLYNOMIAL;
// 			}
// 			extract >>= 1;
// 		}
// 	}
// 	return crc;
// }

// /*******************************************************************************

// Calculate the CRC for an N number of bytes.

//  *******************************************************************************/
// byte crc8_str_n(const byte *data, byte len) {
// 	byte crc = 0x00; //Start with a blank CRC.
// 	while (len--) {
// 		byte extract = *data++;
// 		for (byte tempI = 8; tempI; tempI--) {
// 			byte sum = (crc ^ extract) & 0x01;
// 			crc >>= 1;
// 			if (sum) {
// 				crc ^= CRC_POLYNOMIAL;
// 			}
// 			extract >>= 1;
// 		}
// 	}
// 	return crc;
// }

/*******************************************************************************

Calculate the CRC for an N number of bytes with a starting seed CRC value

 *******************************************************************************/
byte crc8_n(byte crc_start, const byte *data, byte len) {
	byte crc = crc_start; //Start with a seed CRC.
	while (len--) {
		byte extract = *data++;
		for (byte tempI = 8; tempI; tempI--) {
			byte sum = (crc ^ extract) & 0x01;
			crc >>= 1;
			if (sum) {
				crc ^= CRC_POLYNOMIAL;
			}
			extract >>= 1;
		}
	}
	return crc;
}

/*******************************************************************************

Calculate the CRC for a single bytes
Why only a single byte CRC. Because a single byte will serve its purpose (of
ensuring the integrity of the message) very well while still allowing us not
to have to worry about byte-endianness

 *******************************************************************************/
byte crc8(byte crc_start, byte data) {
	byte crc = crc_start; //Start with a blank CRC.
	for (byte tempI = 8; tempI; tempI--) {
		byte sum = (crc ^ data) & 0x01;
		crc >>= 1;
		if (sum) {
			crc ^= CRC_POLYNOMIAL;
		}
		data >>= 1;
	}
	return crc;
}

#undef PRINTF_TAG
/*************************** END OF FILE *************************************/
