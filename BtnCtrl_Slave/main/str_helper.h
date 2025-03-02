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

#undef EXT
#endif /* __SYSUTILS_H__ */

/****************************** END OF FILE **********************************/
