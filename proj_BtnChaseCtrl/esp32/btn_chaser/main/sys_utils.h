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

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

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

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

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

uint8_t crc8_n(uint8_t crc_start, const uint8_t *data, size_t len);
uint8_t crc8(uint8_t crc_start, uint8_t data);

#undef EXT
#endif /* __SYSUTILS_H__ */

/****************************** END OF FILE **********************************/
