/*******************************************************************************

Module:     sys_utils.c
Purpose:    This file contains the system utility functions
Author:     Rudolph van Niekerk

 *******************************************************************************/


/*******************************************************************************
includes
 *******************************************************************************/
#include <stdarg.h>
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>

#define __NOT_EXTERN__
#include "sys_utils.h"
#undef __NOT_EXTERN__

/*******************************************************************************
local defines
 *******************************************************************************/

/*******************************************************************************
local variables
 *******************************************************************************/


/*******************************************************************************

Calculate the CRC for a null terminated string with a starting seed CRC value

 *******************************************************************************/
uint8_t crc8_str(uint8_t crc_start, const char *str) {
	//const char * data = str;
	uint8_t crc = crc_start; //Start with the seed CRC.
	while (*str)
        crc = crc8(crc, *str++);
	return crc;
}

/*******************************************************************************

Calculate the CRC for an N number of bytes of a string with a starting seed CRC 
value. If N is greater than the length of the string, the CRC will be calculated
for the entire string.

 *******************************************************************************/
uint8_t crc8_str_n(uint8_t crc_start, const uint8_t *data, size_t len) {
	uint8_t crc = crc_start; //Start with a seed CRC.
	while ((len--) && (*data))
        crc = crc8(crc, *data++);

	return crc;
}

/*******************************************************************************

Calculate the CRC for a single bytes
Why only a single byte CRC. Because a single byte will serve its purpose (of
ensuring the integrity of the message) very well while still allowing us not
to have to worry about byte-endianness

 *******************************************************************************/
uint8_t crc8(uint8_t crc_start, uint8_t data) {
	uint8_t crc = crc_start; //Start with a blank CRC.
	for (uint8_t tempI = 8; tempI; tempI--) {
		uint8_t sum = (crc ^ data) & 0x01;
		crc >>= 1;
		if (sum) {
			crc ^= CRC_8_POLYNOMIAL;
		}
		data >>= 1;
	}
	return crc;
}

#undef EXT
/*************************** END OF FILE *************************************/
