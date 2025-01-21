/*****************************************************************************

dev_console.h

Include file for dev_console.c

******************************************************************************/
#ifndef __DEV_CONSOLE_H__
#define __DEV_CONSOLE_H__


/******************************************************************************
includes
******************************************************************************/
#include "defines.h"
#include <HardwareSerial.h>

/******************************************************************************
definitions
******************************************************************************/
#ifdef CONSOLE_MENU

#define CONSOLE_RX_BUFF     80 /* must be able to store the max size of string. */

#define iPrintF(traceflags, fmt, ...) _SerialPrintf(traceflags, PSTR(fmt), ##__VA_ARGS__)
#define PrintF(fmt, ...) _SerialPrintf(PSTR(fmt), ##__VA_ARGS__)

/******************************************************************************
Macros
******************************************************************************/

/******************************************************************************
Struct & Unions
******************************************************************************/
typedef struct
{
  char RxBuff[CONSOLE_RX_BUFF + 1];  // The rx buff for data for the console.
  int InPtr;                        // The index into the rx buff.
  //bool Overflow;
}ST_CONSOLE;

typedef struct
{
//	void * OwnerObj;
//	char * Tag;
	const char * Command;
	void (*Func)(char **, int);
//	void (*Func)(void);
	const char * HelpStr;
	void * Prev;
	void * Next;
	void * Parent;
	void * FirstBorn;
    int8_t level;
}st_console_menu_item;

/******************************************************************************
variables
******************************************************************************/
//STATIC ST_TIMER stDev_Print_Timer;

//BYTE DoMutex;

/******************************************************************************
Class definition
******************************************************************************/

bool devConsoleInit(unsigned long baud, byte config);

void _SerialPrintf(const char *fmt, ...);
void _SerialPrintf(int traceflags, const char *fmt, ...);

//st_console_menu_item * addMenuItem(st_console_menu_item * menuItem);
st_console_menu_item * addMenuItem(st_console_menu_item * menuItem, st_console_menu_item * parent = NULL);

bool Read(void);
int paramsTotal(void);
char * paramGetIndex(int index);
char * paramGetNext(void);
int paramsGetCnt(void);

//void _SerialPrintf(int flags, const char *fmt, ...);
//void DoNothing(int flags, const char *fmt, ...);
bool GetTrace(int flagIndex);
int GetTraceIndex(const char * traceName);
void SetTrace(int flagIndex);
void ClearTrace(int flagIndex);
const char * GetTraceName(int flagIndex);
void DumpMem(int Flags, void * Src, unsigned long Address, int Len);
#ifdef MAIN_DEBUG
	void menuTogglePin(char ** args, int arg_cnt);
#endif /* MAIN_DEBUG */

void menuDumpMem(char ** args, int arg_cnt);
void menuTogglePrintFlags(char ** args, int arg_cnt);

#endif /* CONSOLE_MENU */

#endif /* __DEV_CONSOLE_H__ */

/****************************** END OF FILE **********************************/
