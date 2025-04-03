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

The General Timer (timer_ms_t type) - 1 kHz granularity
The general Timers enables the program to create a downcounter with a
preloaded value. This timer will then decrement every 1 ms until it has
expired.
Usage:
The module making use of the timers must host a timer_ms_t structure in RAM and
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
void sys_output_write(uint8_t pin, bool state)
{
//uint8_t port;

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

int sys_input_read(uint8_t pin)
{
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);

	//if (port == NOT_A_PIN) return LOW;

	if (*portInputRegister(port) & bit) return HIGH;
	return LOW;
}

void sys_set_io_mode(uint8_t pin, uint8_t mode)
{
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	volatile uint8_t *reg, *out;

	if (port == NOT_A_PIN) return;

	// JWS: can I let the optimizer do this?
	reg = portModeRegister(port);
	out = portOutputRegister(port);

	if (mode == INPUT) { 
		uint8_t oldSREG = SREG;
                cli();
		*reg &= ~bit;
		*out &= ~bit;
		SREG = oldSREG;
	} else if (mode == INPUT_PULLUP) {
		uint8_t oldSREG = SREG;
                cli();
		*reg &= ~bit;
		*out |= bit;
		SREG = oldSREG;
	} else {
		uint8_t oldSREG = SREG;
                cli();
		*reg |= bit;
		SREG = oldSREG;
	}
}

// uint8_t analog_reference = DEFAULT;

// void analogReference(uint8_t mode)
// {
// 	// can't actually set the register here because the default setting
// 	// will connect AVCC and the AREF pin, which would cause a short if
// 	// there's something connected to AREF.
// 	analog_reference = mode;
// }

int sys_analog_read(uint8_t pin)
{
	if (pin >= 14) pin -= 14; // allow for channel or pin numbers

	// set the analog reference (high two bits of ADMUX) and select the
	// channel (low 4 bits).  this also sets ADLAR (left-adjust result)
	// to 0 (the default).
	ADMUX = (DEFAULT << 6) | (pin & 0x07);

	// without a delay, we seem to read from the wrong channel
	//delay(1);

	// start the conversion
	sbi(ADCSRA, ADSC);

	// ADSC is cleared when the conversion finishes
	while (bit_is_set(ADCSRA, ADSC));

	// ADC macro takes care of reading ADC register.
	// avr-gcc implements the proper reading order: ADCL is read first.
	return ADC;
}

long sys_random(long rand_min, long rand_max)
{
    int adc_read = sys_analog_read(input_ADC); /* Random seed for the address */
    randomSeed(adc_read);
    return random(rand_min, rand_max); //Random number between min and max
}

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

#define FIBONACCI_MAX   (46U)    /* Beyond this, the result overruns a 32-bit unsigned integer */
uint32_t fibonacci(uint8_t n)
{
    uint32_t terms[2] = {0, 1};

    if (n == 0)
        return 0;
    if (n > FIBONACCI_MAX)
        return UINT32_MAX;
    
    for (uint8_t i = 1; i <= n; i++)
    {
        unsigned int result = terms[0] + terms[1];
        terms[0] = terms[1];
        terms[1] = result;
    }

    return terms[1];
}

uint8_t crc8_n(uint8_t crc_start, const uint8_t *data, uint8_t len) {
	uint8_t crc = crc_start; //Start with a seed CRC.
	while (len--) {
		uint8_t extract = *data++;
		for (uint8_t tempI = 8; tempI; tempI--) {
			uint8_t sum = (crc ^ extract) & 0x01;
			crc >>= 1;
			if (sum) {
				crc ^= CRC_POLYNOMIAL;
			}
			extract >>= 1;
		}
	}
	return crc;
}

uint8_t crc8(uint8_t crc_start, uint8_t data) {
	uint8_t crc = crc_start; //Start with a blank CRC.
	for (uint8_t tempI = 8; tempI; tempI--) {
		uint8_t sum = (crc ^ data) & 0x01;
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
