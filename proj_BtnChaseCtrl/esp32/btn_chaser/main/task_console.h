/*****************************************************************************

task_console.h

Include file for task_console.c

******************************************************************************/
#ifndef __task_console_H__
#define __task_console_H__


/******************************************************************************
includes
******************************************************************************/
#include "defines.h"
#include "../../../../common/common_defines.h"

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

#define trAPP		((uint8_t)BIT_POS(0))
#define trCONSOLE	((uint8_t)BIT_POS(1))
#define trLED		((uint8_t)BIT_POS(2))
#define trCOMMS     ((uint8_t)BIT_POS(3))
#define trNODE      ((uint8_t)BIT_POS(4))
#define trGAME      ((uint8_t)BIT_POS(5))

#define trALWAYS	((uint8_t)BIT_POS(7))

#define trALL		((uint8_t)(~trALWAYS))
#define trNONE		((uint8_t)0)

/******************************************************************************
Print an entire line (or not). A leading '#' is replaced with the PRINTF_TAG, 
 and the print is concluded with an newline character.
The passed traceflags is compared with the system set trace print flag(s) to
 determine if the print can happen or not.
******************************************************************************/
#define iprintln(traceflags, fmtstr, ...) console_printline(traceflags, PRINTF_TAG, fmtstr, ##__VA_ARGS__)
/******************************************************************************
"Incomplete" version of iprintln... no "\n" added at the end
******************************************************************************/
#define iprint(traceflags, fmtstr, ...) console_print(traceflags, PRINTF_TAG, fmtstr, ##__VA_ARGS__)

/******************************************************************************
Struct & Unions
******************************************************************************/
typedef struct
{
    //Menu items will be handled as a single static list of items. This app is
    // not so complex as to warrant sub-menus and all that.
	char * command;
	void (*func)(void);
	const char *  description;
}ConsoleMenuItem_t;

/******************************************************************************
variables
******************************************************************************/

/******************************************************************************
Global (public) Function prototypes
******************************************************************************/

/*! Initialises the Console
 * @return A pointer to the taskInfo structure for the console (if successful)
            otherwise NULL
 */
void * console_init_task(void);

/*! Adds a menu item to the console
 * @param[in] _group_name The name of the group of commands to which this item is added
 * @param[in] _tbl A table of ConsoleMenuItem_t items
 * @param[in] _cnt The number of ConsoleMenuItem_t items in the table
 * @param[in] _desc A description of the group
 * @return The number of commands in the table
 */
int console_add_menu(const char *, ConsoleMenuItem_t *, size_t, const char *);

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

/*! Prints a section of memory
 */
void console_print_memory(int Flags, void * Src, unsigned long Address, int Len);


#undef EXT
#endif /* __task_console_H__ */

/****************************** END OF FILE **********************************/
