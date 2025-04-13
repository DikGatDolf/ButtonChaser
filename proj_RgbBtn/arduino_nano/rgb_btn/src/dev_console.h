/*****************************************************************************

dev_console.h

Include file for dev_console.c

******************************************************************************/
#ifndef __dev_console_H__
#define __dev_console_H__


/******************************************************************************
includes
******************************************************************************/
//#include "defines.h"
#include "sys_utils.h"

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

#define trMAIN		((uint8_t)BIT_POS(0))
#define trCONSOLE	((uint8_t)BIT_POS(1))
#define trRGB		((uint8_t)BIT_POS(2))
#define trCOMMS	    ((uint8_t)BIT_POS(3))
#define trNVSTORE	((uint8_t)BIT_POS(4))
#define trBUTTON    ((uint8_t)BIT_POS(5))

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
 * @param[in] cb_write The function to call to write a byte to the serial port
 * @param[in] cb_flush The function to call to flush the serial port
 * @param[in] cb_rs485_disable The function to call to disable the RS485 driver
 */
void console_init(size_t (*cb_write)(uint8_t), void (*cb_flush)(void), void (*cb_rs485_disable)(void));

/*! Checks if the console buffer contains any valid commands and parses them
 */
void console_service(void);

/*! Adds a byte to the read buffer (as if it was recieved from the serial port)
 * @param[in] data_byte The byte to add to the buffer
 */
void console_read_byte(uint8_t data_byte);

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

/*! Returns the index of the "help" or "?" argument if it was in the list
 * @return true if the "help" or "?" argument was found, false otherwise
 */
bool console_arg_help_found(void);

/*! Prints a section of RAM
 */
void console_print_ram(int Flags, void * Src, unsigned long Address, int Len);

/*! Prints a section of Flash
 */
void console_print_flash(int Flags, void * Src, unsigned long Address, int Len);

void console_flush(void);

void console_enable_alt_output_stream(size_t (*alt_write_cb)(uint8_t));

#undef EXT
#endif /* __task_console_H__ */

/****************************** END OF FILE **********************************/
