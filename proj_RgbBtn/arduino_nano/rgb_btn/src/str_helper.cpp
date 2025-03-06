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
#include <stdint.h>
#include "dev_console.h"

#define __NOT_EXTERN__
#include "str_helper.h"
#undef __NOT_EXTERN__

#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("StrHelper") /* This must be undefined at the end of the file*/

/*******************************************************************************
 Macro definitions
*******************************************************************************/
#define FMT_TO_STR_BUFF_SIZE    (12)    /* 0123456789A
                                          "%s%ld.%0_ld"*/

/*******************************************************************************
 Local defines
*******************************************************************************/

/*******************************************************************************
 Local structure
*******************************************************************************/

/*******************************************************************************
local defines
 *******************************************************************************/

/*******************************************************************************
local variables
 *******************************************************************************/

/*******************************************************************************
local functions
 *******************************************************************************/

/*******************************************************************************
Global functions
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

bool is_hex_str(const char * str, unsigned int expected_len)
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

#define MS_PER_SEC  1000
#define SEC_PER_MIN 60
#define MIN_PER_HR  60
#define HR_PER_DAY  24

#define _u32_ms2hms0_len    15  /* 0xFFFFFFFF  (uint32_t max)
                                    = 4,294,967,295 
                                    = 1193h 2m 47s 295ms 
                                    => "1193:02:47.295" 
                                    is 15 bytes (with NULL terminator) */
bool ms2dhms_str(char *buff, uint32_t ms)
{
    uint32_t dec = ms%MS_PER_SEC;
    uint32_t sec = ms/MS_PER_SEC;
    uint32_t min = sec/SEC_PER_MIN;
    uint32_t hr  = min/MIN_PER_HR;
    
    if (buff == NULL)
    {    
        return false;
    }

    snprintf(buff, _u32_ms2hms0_len, "%ld:%02ld:%02ld.%03ld", hr, min%MIN_PER_HR, sec%SEC_PER_MIN, dec);

    return true;
}

bool float2str(char *buff, double fVal, unsigned int decimalpoints, size_t max_len)
{
	long int  multiplier_divider = 1;
	long int  workingVal;
	long int mod_delta = 0;
                    // 0123456789A
	char __fmt[FMT_TO_STR_BUFF_SIZE]; //"%s%ld.%0_ld"
	long int intVal;
	long int decVal;
	char negSign[2];

    if (buff == NULL){    
        return false;
    }

	//Limit the requsted decimal points between 0 and 4
    if (decimalpoints > 6){
		decimalpoints = 6;	//"9.99999999" -> "9.9999"
    }
    //iprintln(trALWAYS, "#pts: %u", decimalpoints);

	//We need to get our multiplier right.
	for (unsigned int i = 0; i < decimalpoints; i++){
		multiplier_divider *= 10;
    }
    //iprintln(trALWAYS, "#muldiv: %ld", multiplier_divider);

	// Calculate the 10x product
	workingVal = (long int)(fVal * 10 * ((float)multiplier_divider));
    //iprintln(trALWAYS, "#workingVal: %ld", workingVal);

	//Do we need to round up or down?
	mod_delta = workingVal%10;
	if (mod_delta >= 5){
		//For values 5, 6, 7, 8, and 9, we round up
		workingVal += (10 - mod_delta);
    }else{ //if (mod_delta >= 0)
		//For values 0, 1, 2, 3, and 4 we round down
		workingVal -= mod_delta;
    }
	// Calculate the final product
	workingVal /= 10;
    //iprintln(trALWAYS, "#mod_delta: %ld", mod_delta);
    //iprintln(trALWAYS, "#workingVal: %ld", workingVal);

	//Get the whole number
	intVal = abs(workingVal / multiplier_divider);
	//Get the decimal number
	decVal = abs(workingVal % multiplier_divider);
    //iprintln(trALWAYS, "#intVal: %ld", intVal);
    //iprintln(trALWAYS, "#decVal: %ld", decVal);

    //Start with an empty format string.
	__fmt[0] = 0;
	negSign[0] = 0;

    //determine if we need a "-" sign
	if (((intVal > 0) || (decVal > 0)) && (fVal < 0)){
		strlcat(negSign, "-", 2);
    }
	//signVal[1] = 0;
	//We are going to build up our format string as we go along
	strlcat(__fmt, "%s%ld", FMT_TO_STR_BUFF_SIZE);
	//				01234...
	//Are we planning on having a decimal point?
	if (decimalpoints > 0)
	{
		if (decimalpoints == 1)
		{
			strlcat(__fmt, ".%ld", FMT_TO_STR_BUFF_SIZE);
            //__fmt = "%s%ld.%ld"
			//		   0123456789A
		}
		else
		{
			strlcat(__fmt, ".%0_ld", FMT_TO_STR_BUFF_SIZE);
            //__fmt = "%s%ld.%0_ld"
			//		   0123456789A
			__fmt[8] = '0' + decimalpoints;
		}
	    //And now we print our buffer?
        if (max_len > 0){
    	    snprintf(buff, max_len, (const char*)__fmt, negSign, intVal, decVal);
        }else{
            sprintf(buff, (const char*)__fmt, negSign, intVal, decVal);
        }
	}else{
        //No decimal point
        //__fmt = "%s%ld"
		//		   01234
	    //And now we print our buffer?
        if (max_len > 0){
    	    snprintf(buff, max_len, (const char*)__fmt, negSign, intVal);
        }else{
            sprintf(buff, (const char*)__fmt, negSign, intVal);
        }
    }
    //iprintln(trALWAYS, "#fmt: \"%s\", neg: (%s), int: %ld, dec: %ld, pts: %u", __fmt, negSign, intVal, decVal, decimalpoints);
	return true;
}

#undef PRINTF_TAG
/*************************** END OF FILE *************************************/
