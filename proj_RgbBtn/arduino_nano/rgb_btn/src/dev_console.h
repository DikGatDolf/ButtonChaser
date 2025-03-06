/*****************************************************************************

dev_console.h

Include file for dev_console.c

******************************************************************************/
#ifndef __dev_console_H__
#define __dev_console_H__


/******************************************************************************
includes
******************************************************************************/
#include "defines.h"
#include <HardwareSerial.h>

/******************************************************************************
Macros
******************************************************************************/
#ifdef __NOT_EXTERN__
#define EXT
#else
#define EXT extern
extern void console_printline(uint8_t traceflags, const char * tag, const char *fmt, ...);
extern void console_print(uint8_t traceflags, const char * tag, const char *fmt, ...);
#endif /* __NOT_EXTERN__ */

#define BIT_POS(pos)			(1U << pos)

#define SET_BIT(x, pos)			(x |= BIT_POS(pos))
#define CLEAR_BIT(x, pos) 		(x &= (~BIT_POS(pos)))
#define TOGGLE_BIT(x, pos) 		(x ^= BIT_POS(pos))

#define BIT_IS_SET(x,pos) 		((x) & BIT_POS(pos))
#define BIT_IS_CLEAR(x,pos) 	(~BIT_IS_SET(x,pos))

#define trMAIN		((uint8_t)BIT_POS(0))
#define trCONSOLE	((uint8_t)BIT_POS(1))
#define trPWM		((uint8_t)BIT_POS(2))
// #define trRGB		0x04
// #define trBUTTON	0x08
// #define trI2C		0x10
// #define trYYY		0x40


#define trALWAYS	((uint8_t)BIT_POS(7))

#define trALL		((uint8_t)(~trALWAYS))
#define trNONE		((uint8_t)0)

/******************************************************************************
Print an entire line (or not). A leading '#' is replaced with the PRINTF_TAG, 
 and the print is concluded with an newline character.
The passed traceflags is compared with the system set trace print flag(s) to
 determine if the print can happen or not.
******************************************************************************/
#define iprintln(traceflags, fmtstr, ...) console_printline(traceflags, PRINTF_TAG, PSTR(fmtstr), ##__VA_ARGS__)
/******************************************************************************
"Incomplete" version of dbgPrint.... no "\n" added at the end
******************************************************************************/
#define iprint(traceflags, fmtstr, ...) console_print(traceflags, PRINTF_TAG, PSTR(fmtstr), ##__VA_ARGS__)

/******************************************************************************
Struct & Unions
******************************************************************************/
typedef struct
{
    //Menu items will be handled as a single static list of items. This app is
    // not so complex as to warrant sub-menus and all that.
	const char * command;
	void (*func)(void);
	const char *  description;
}console_menu_item_t;

/******************************************************************************
variables
******************************************************************************/

/******************************************************************************
Global (public) Function prototypes
******************************************************************************/

/*! Initialises the Console
 * @param[in] baud The baud rate to use for the console
 * @param[in] config The configuration of the serial port
 */
void console_init(unsigned long baud, uint8_t config);

 /*! Reads the data on the serial port and parses the line when a carriage return 
 * is encountered.
 */
bool console_service(void);

/*! Adds a menu item to the console
 * @param[in] _group_name The name of the group of commands to which this item is added
 * @param[in] _tbl A table of console_menu_item_t items
 * @param[in] _cnt The number of console_menu_item_t items in the table
 * @param[in] _desc A description of the group
 * @return The number of commands in the table
 */
int console_add_menu(const char *, const console_menu_item_t *, size_t, const char *);

/*! Pops the next argument from the stack
 * @return A pointer to the argument on the stack. NULL if there are no arguments to pop
 */
char * console_arg_pop(void);

/*! Returns a pointer to the argument on the stack relative to to the index of 
 * the argument which will be popped from the stack next using ...pop() (if available)
 * IMPORTANT. This DOES NOT decrease the count of arguments left to pop
 * This allows the user to "peek" at arguments on the stack without actually popping
 *  them off
 * @param[in] offset The offset from the current argument to return
 * @return A pointer to the argument on the stack. NULL if the ultimate index is 
 *      outside of the bounds of the current stack.
 */
char * console_arg_peek(int offset);

/*! Returns a count of the argument left on the stack to pop
 * @return The count of arguments left to pop
 */
int console_arg_cnt(void);

#undef EXT
#endif /* __task_console_H__ */

/****************************** END OF FILE **********************************/
