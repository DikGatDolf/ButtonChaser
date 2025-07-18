/*****************************************************************************

sys_utils.h

Include file for sys_utils.c

******************************************************************************/
#ifndef __sys_utils_H__
#define __sys_utils_H__


/******************************************************************************
includes
******************************************************************************/
#include <avr/pgmspace.h>
#include "Arduino.h"
#include "defines.h"


/******************************************************************************
definitions
******************************************************************************/

/******************************************************************************
Macros
******************************************************************************/
#define ASSERT_CONCAT_(a, b) a##b
#define ASSERT_CONCAT(a, b) ASSERT_CONCAT_(a, b)
/* These can't be used after statements in c89. */
#ifdef __COUNTER__
  #define STATIC_ASSERT(e,m) \
    ;enum { ASSERT_CONCAT(static_assert_, __COUNTER__) = 1/(int)(!!(e)) }
#else
  /* This can't be used twice on the same line so ensure if using in headers
   * that the headers are not included twice (by wrapping in #ifndef...#endif)
   * Note it doesn't cause an issue when used on same line of separate modules
   * compiled with gcc -combine -fwhole-program.  */
  #define STATIC_ASSERT(e,m) \
    ;enum { ASSERT_CONCAT(assert_line_, __LINE__) = 1/(int)(!!(e)) }
#endif

#define BIT_POS(pos)			    (1U << pos)

#define SET_BIT(x, pos)			    (x |= BIT_POS(pos))
#define CLEAR_BIT(x, pos) 		    (x &= (~BIT_POS(pos)))
#define TOGGLE_BIT(x, pos) 		    (x ^= BIT_POS(pos))

#define IS_BIT_SET(value, bit)      (value & BIT_POS(bit))
#define IS_BIT_CLEAR(value, bit)    (!(value & BIT_POS(bit)))

#define sign_f(a)	((a < 0)? -1.0 : 1.0)
#define INPUT_EDGE_NONE		0
#define INPUT_EDGE_FALLING	1
#define INPUT_EDGE_RISING	2

#define CRC_POLYNOMIAL 0x8C

#ifndef NULL_PTR
#define NULL_PTR (void*)0
#endif

#define ARRAY_SIZE(_array) (sizeof(_array) / sizeof((_array)[0]))

#define SWOP_U64(x, y) do { uint64_t _z = x; x = y; y = _z; } while(0)
#define SWOP_U32(x, y) do { uint32_t _z = x; x = y; y = _z; } while(0)
#define SWOP_U16(x, y) do { uint16_t _z = x; x = y; y = _z; } while(0)
#define SWOP_U8(x, y) do { uint8_t _z = x; x = y; y = _z; } while(0)

#ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#ifndef nop
    #define nop() __asm__ __volatile__ ("nop");
#endif
/******************************************************************************
Struct & Unions
******************************************************************************/
typedef struct
{
	//bool States[5];
	int CurrentState;
	int DebounceCount;
}ST_PIN_DEBOUNCE;

typedef struct
{
	//bool States[5];
	uint8_t flagMask;
	const char * Name;
}ST_PRINT_FLAG_ITEM;

/******************************************************************************
variables
******************************************************************************/
const PROGMEM int pinDEBUG_0 	= 14;
const PROGMEM int pinDEBUG_1 	= 15;
const PROGMEM int pinDEBUG_2 	= 16;
const PROGMEM int pinDEBUG_3 	= 17;
const PROGMEM int pinDEBUG_4 	= 18;
const PROGMEM int pinDEBUG_5 	= 19;

/******************************************************************************
functions
******************************************************************************/
/*! An attempt to manage my outpts without the use of digitalWrite and a subsequent
 *  f@#$ing about with clearing the interrupt flag
 * @param[in] pin The pin to toggle
 * @param[in] state The state to set the pin to
*/
void sys_output_write(uint8_t pin, bool state);
int sys_input_read(uint8_t pin);
void sys_set_io_mode(uint8_t pin, uint8_t mode);
int sys_analog_read(uint8_t pin);
long sys_random(long rand_min, long rand_max);

int freeRam (void);

/*! Calculates the nth term of the Fibonacci sequence using the formula 
    F(n)=F(n−1)+F(n−2) for n>1, with initial conditions F(0)=0 and F(1)=1. 
    for n > 46, the result overruns a 32-bit unsigned integer 0xFFFFFFFF is returned
 * @param[in] n The term to calculate
 * @return The nth term
*/
uint32_t fibonacci(uint8_t n);

/*! Calculate the CRC for an N number of bytes with a starting seed CRC value
    * @param[in] crc_start The starting seed CRC value
    * @param[in] data Pointer to the data to calculate the CRC for
    * @param[in] len The number of bytes to calculate the CRC for
    * @return The calculated CRC
*/
uint8_t crc8_n(uint8_t crc_start, const uint8_t *data, uint8_t len);

/*! Calculate the CRC for a single byte with a starting seed CRC value
    * @param[in] crc_start The starting seed CRC value
    * @param[in] data The data to calculate the CRC for
    * @return The calculated CRC
*/
uint8_t crc8(uint8_t crc_start, uint8_t data);

#endif /* __sys_utils_H__ */

/****************************** END OF FILE **********************************/
