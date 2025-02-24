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
TaskInfo_t * sys_tasks[eTaskIndexMax];

/*******************************************************************************
local variables
 *******************************************************************************/

///*******************************************************************************
//
//Finds the start of the next word after this one.
//Returns NULL if nothing is found.
//
// *******************************************************************************/
//char * nextWord(char * curWord, bool terminate)
//{
//char *nxtWord;
//	//Let's start by checking if there is a space after this word?
//	nxtWord = strchr(curWord, ' ');
//	if (nxtWord)
//	{
//		if (terminate)
//			*nxtWord = 0;
//
//		//Great... now we skip the spaces until we find something else
//		do
//		{
//			nxtWord++;
//		}
//		while (*nxtWord == ' ');
//
//		if (*nxtWord == NULL)
//			return NULL;
//	}
//
//	return nxtWord;
//}

/*******************************************************************************

Finds the start of the next word after this one.
Returns NULL if nothing is found.

 *******************************************************************************/
//char * nextWord_delim(char * curWord, bool terminate, char delim)
//{
//char *nxtWord;
//	//Let's start by checking if there is a space after this word?
//	nxtWord = strchr(curWord, delim);
//	if (nxtWord)
//	{
//		if (terminate)
//			*nxtWord = 0;
//
//		//Great... now we skip the spaces until we find something else
//		do
//		{
//			nxtWord++;
//		}
//		while (*nxtWord == ' ');
//
//		if (*nxtWord == NULL)
//			return NULL;
//	}
//
//	return nxtWord;
//}
/*******************************************************************************

Returns the byte value (lower nibble only) represented by a single HEX character

 *******************************************************************************/
uint8_t charToNibble(char hexDigit)
{
	uint8_t retVal = 0x00;
	if ((hexDigit >= '0') && (hexDigit <= '9'))
		retVal = (hexDigit - '0');
	else if ((hexDigit >= 'A') && (hexDigit <= 'F'))
		retVal = 0x0A + (hexDigit - 'A');
	else if ((hexDigit >= 'a') && (hexDigit <= 'f'))
		retVal = 0x0A + (hexDigit - 'a');

	return retVal;
}

uint8_t hexToByte(char * hexDigits)
{
	uint8_t retVal;

	retVal = charToNibble(*hexDigits);
	retVal <<= 4;
	hexDigits++;
	retVal |= charToNibble(*hexDigits);

	return retVal;
}


/*******************************************************************************

Checks if the string is a decimal number (i.e. -1.04, 10.7, 200.8, -23.5....

 *******************************************************************************/
bool isFloatStr(char * str)
{
	int cnt = 0;
	bool decimal = false; //Only allowed 1 decimal point;

	//Check every character until we reach a delimeter or a null-terminator

	//Allow a + or - only in the first position
	if ((str[cnt] == '-') || (str[cnt] == '+'))
		cnt++;

	while (str[cnt])
	{
		if (isdigit((int)str[cnt]) == 0)
		{
			//Not a digit... could be a decimal point
			if ((str[cnt] == '.') && (!decimal))
			{
				//Ok... let's keep on looking...
				decimal = true;
			}
			else
			{
				//Either a second decimal point or something else completely
				return false;
			}
		}
		cnt++;

	}
	return true;
}
/*******************************************************************************

Checks if the string is a natural number (i.e. 0, 1, 2,....

 *******************************************************************************/
bool isNaturalNumberStr(char * str)
{
	//Check every character until we reach a delimeter or a null-terminator
	while (*str)
	{
		if (isdigit((int)*str) == 0)
			return false;
		str++;

	}
	return true;
}


/*******************************************************************************

Returns a pointer to the first non-whitespace character in the string
If no whitespace is found, the passed pointer is returned.
If the string is empty, NULL is returned.

 *******************************************************************************/
char * str_trim_l(const char *str) 
{
    while (isspace((int)*str))
    {
        str++;
        if (*str == 0)
            break; //from while-loop (end of string)
    }

    if (*str)
        return (char *)str;
        
    return NULL;
}


/*******************************************************************************

returns a pointer to the 1st character of the next word after whitespace in the 
string.

 *******************************************************************************/
char * str_next_word(const char *str) 
{
    //Skip over whitespace
    char *nxtWord = str_trim_l(str);

    if (nxtWord)
    {
        //Skip over the word (non-whitespace characters)
        while (!isspace((int)*nxtWord))
        {
            if (*nxtWord == 0) 
                return NULL;    // end of string reached..... no more words
            nxtWord++;          // Move to the next character
        }
        //We've reached the end of the "current" word... now we need to find the next word (and null terminate whitespace as we go)
        while (isspace((int)*nxtWord))
        {
            *nxtWord = 0;  // Null terminate the whitespace
            nxtWord++;      // Move to the next character
        }
        //We've reached the end of the whitespace "between" words
        if (*nxtWord)
            return nxtWord;
    }
    
    return NULL;
}


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
