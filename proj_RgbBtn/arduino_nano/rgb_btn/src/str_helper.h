/*****************************************************************************

str_helper.h

Include file for str_helper.c

******************************************************************************/
#ifndef __str_helper_H__
#define __str_helper_H__


/******************************************************************************
includes
******************************************************************************/
#include "defines.h"

/******************************************************************************
definitions
******************************************************************************/
#ifdef __NOT_EXTERN__
#define EXT
#else
#define EXT extern
#endif /* __NOT_EXTERN__ */


/******************************************************************************
Macros
******************************************************************************/

/******************************************************************************
Struct & Unions
******************************************************************************/

/******************************************************************************
variables
******************************************************************************/

/******************************************************************************
functions
******************************************************************************/
bool hex2u32(uint32_t *val, const char * str, int expected_len);
bool str2int32(int32_t *val, const char * str, int expected_len);
bool str2uint32(uint32_t *val, const char * str, int expected_len);
uint8_t char2nibble(char hexDigit);
uint8_t hex2byte(char * hexDigits);
bool is_float_str(const char * str);

bool is_natural_number_str(const char * str, int32_t expected_len);

bool is_hex_str(const char * str, int expected_len);

char * str_trim_l(const char *str);
char * str_next_word(const char *str);

/** Converts a ms count value to a string in the format "hhhh:mm:ss.000"
 * For ms values longer than 24 hours, the decimal (ms) seconds is discarded
 * @param buff a pointer to a buffer to store the string (max 15 characters)
 * @param ms the time in ms to convert
*/
bool ms2dhms_str(char *buff, uint32_t ms);

/*! Converts a float to a string
 * @param buff a pointer to a buffer to store the string (the caller must 
                ensure that the buffer is large enough)
 * @param fVal the float value to convert
 * @param decimalpoints the number of decimal points to display (max: 4)
 * @param max_len the maximum length of the buffer
*/
bool float2str(char *buff, double fVal, unsigned int decimalpoints, size_t max_len);


#undef EXT
#endif /* __SYSUTILS_H__ */

/****************************** END OF FILE **********************************/
