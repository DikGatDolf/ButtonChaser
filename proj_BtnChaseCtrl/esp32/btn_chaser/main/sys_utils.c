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
#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("StrHelper") /* This must be undefined at the end of the file*/

/*******************************************************************************
local variables
 *******************************************************************************/


/*******************************************************************************

Calculate the CRC for a null terminated string with a starting seed CRC value

 *******************************************************************************/

uint8_t crc8_n(uint8_t crc_start, const uint8_t *data, size_t len) {
	uint8_t crc = crc_start; //Start with a seed CRC.
	while (len--) {
		uint8_t extract = *data++;
		for (size_t i = 8; i; i--) {
			uint8_t sum = (crc ^ extract) & 0x01;
			crc >>= 1;
			if (sum) {
				crc ^= CRC_8_POLYNOMIAL;
			}
			extract >>= 1;
		}
	}
	return crc;
}

uint8_t crc8(uint8_t crc_start, uint8_t data) {
	uint8_t crc = crc_start; //Start with a blank CRC.
    for (size_t i = 8; i; i--) {
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

#undef PRINTF_TAG

/*************************** END OF FILE *************************************/
