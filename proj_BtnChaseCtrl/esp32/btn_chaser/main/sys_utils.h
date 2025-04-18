/*****************************************************************************

sys_utils.h

Include file for sys_utils.c

******************************************************************************/
#ifndef __SYSUTILS_H__
#define __SYSUTILS_H__


/******************************************************************************
includes
******************************************************************************/
#include "defines.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/******************************************************************************
definitions
******************************************************************************/
#ifdef __NOT_EXTERN__
#define EXT
#else
#define EXT extern
#endif /* __NOT_EXTERN__ */

#define BIT_POS(pos)			(1U << pos)

#define SET_BIT(x, pos)			(x |= BIT_POS(pos))
#define CLEAR_BIT(x, pos) 		(x &= (~BIT_POS(pos)))
#define TOGGLE_BIT(x, pos) 		(x ^= BIT_POS(pos))

#define BIT_IS_SET(x,pos) 		((x) & BIT_POS(pos))
#define BIT_IS_CLEAR(x,pos) 	(~BIT_IS_SET(x,pos))

#define CRC_8_POLYNOMIAL 0x8C

#ifndef NULL_PTR
#define NULL_PTR (void*)0
#endif

#define ARRAY_SIZE(_array) (sizeof(_array) / sizeof((_array)[0]))

#define SWOP_U64(x, y) do { uint64_t(x) _z = x; x = y; y = _z; } while(0)
#define SWOP_U32(x, y) do { uint32_t(x) _z = x; x = y; y = _z; } while(0)
#define SWOP_U16(x, y) do { uint16_t(x) _z = x; x = y; y = _z; } while(0)
#define SWOP_U8(x, y) do { uint8_t(x) _z = x; x = y; y = _z; } while(0)

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

uint8_t crc8_str(uint8_t crc_start, const char *str);
uint8_t crc8_str_n(uint8_t crc_start, const uint8_t *data, size_t len);
uint8_t crc8(uint8_t crc_start, uint8_t data);

#undef EXT
#endif /* __SYSUTILS_H__ */

/****************************** END OF FILE **********************************/
