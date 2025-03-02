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
#include "str_helper.h"
#undef __NOT_EXTERN__

/*******************************************************************************
 Local defines
*******************************************************************************/
#define PRINTF_TAG ("StrHelper") /* This must be undefined at the end of the file*/

/*******************************************************************************
 Local structure
*******************************************************************************/

/*******************************************************************************
local defines
 *******************************************************************************/

/*******************************************************************************
local variables
 *******************************************************************************/
bool hex2u32(uint32_t *val, const char * str, int expected_len)
{
    uint64_t retVal = 0;
    str = str_trim_l(str);
    
    if (val == NULL)
        return false;

    if (!is_hex_str(str, expected_len))
        return false;

    while (*str)
    {
        retVal <<= 4;
        retVal |= char2nibble(*str);
        str++;
    }

    if (retVal > UINT32_MAX)
        return false;

    *val = (uint32_t)retVal;
    return true;
}

bool str2uint32(uint32_t *val, const char * str, int expected_len)
{
    uint64_t retVal = 0;
    bool isNegative = false;
    str = str_trim_l(str);

    if (val == NULL)
        return false;

    if (!is_natural_number_str(str, expected_len))
        return false;

        //Allow a + or - only in the first position
    if ((*str == '-') || (*str == '+'))
    {
        isNegative = (*str == '-');
        str++;
    }

    while (*str)
    {
        retVal *= 10;
        retVal += ((*str) - '0');
        str++;
    }

    if (retVal > UINT32_MAX)
        return false;

    if (isNegative)
        retVal = -retVal;

    *val = (uint32_t)retVal;
    return true;
}

bool str2int32(int32_t *val, const char * str, int expected_len)
{
    int64_t retVal = 0;
    bool isNegative = false;
    str = str_trim_l(str);

    if (val == NULL)
        return false;

    if (!is_natural_number_str(str, expected_len))
        return false;

        //Allow a + or - only in the first position
    if ((*str == '-') || (*str == '+'))
    {
        isNegative = (*str == '-');
        str++;
    }

    while (*str)
    {
        retVal *= 10;
        retVal += ((*str) - '0');
        str++;
    }

    if (isNegative)
        retVal = -retVal;

    if ((retVal > INT32_MAX) || (retVal < INT32_MIN))
        return false;

    *val = (int32_t)retVal;
    return true;
}

uint8_t char2nibble(char hexDigit)
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

uint8_t hex2byte(char * hexDigits)
{
	uint8_t retVal;

	retVal = char2nibble(*hexDigits);
	retVal <<= 4;
	hexDigits++;
	retVal |= char2nibble(*hexDigits);

	return retVal;
}

bool is_float_str(const char * str)
{
    str = str_trim_l(str);
//	int cnt = 0;
	bool decimal = false; //Only allowed 1 decimal point;

	//Check every character until we reach a delimeter or a null-terminator

	//Allow a + or - only in the first position
	if ((*str == '-') || (*str == '+'))
        str++;

    //We should at least have _something_ in the string (don't want to return true for an empty string)
    if (*str == 0)
        return false;

    while (*str)
	{
		if (isdigit((int)*str) == 0)
		{
			//Not a digit... could be a decimal point
			if ((*str == '.') && (!decimal))
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
		str++;

	}
	return true;
}

bool is_natural_number_str(const char * str, int32_t expected_len)
{
    str = str_trim_l(str);

    //Allow a + or - only in the first position
    if ((*str == '-') || (*str == '+'))
        str++;

    //Are we expecting a specific length?
    if (expected_len)
    {
        if (strlen(str) != expected_len)
            return false;
    }

    //We should at least have _something_ in the string (don't want to return true for an empty string)
    if (*str == 0)
        return false;

    //Check every character until we reach a null-terminator
	while (*str)
	{
		if (isdigit((int)*str) == 0)
			return false;
		str++;
	}
	return true;
}

bool is_hex_str(const char * str, int expected_len)
{
    str = str_trim_l(str);

    //The string may or may not start with "0x", "0X", "x", "X" or "#"
    if ((*str == '0') && ((*(str+1) == 'x') || (*(str+1) == 'X')))
        str += 2;
    else if ((*str == 'x') || (*str == 'X') || (*str == '#'))
        str++;
    
    //Are we expecting a specific length?
    if (expected_len)
    {
        if (strlen(str) != expected_len)
            return false;
    }
    
    //We should at least have _something_ in the string (don't want to return true for an empty string)
    if (*str == 0)
        return false;

    //Check every character until we reach a null-terminator
    while (*str)
    {
        if (!isxdigit((int)*str))
            return false;
        str++;
    }
    return true;
}


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

#undef PRINTF_TAG
#undef EXT
/*************************** END OF FILE *************************************/
