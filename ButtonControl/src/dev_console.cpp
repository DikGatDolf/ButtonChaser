/*******************************************************************************

Project:   RGB Button Chaser
Module:     dev_console.cpp
Purpose:    This file contains the console interface
Author:     Rudolph van Niekerk
Processor:  Arduino Nano (ATmega328)
Compiler:	Arduino AVR Compiler

The console device will parse all received input strings (terminated with a
carriage return & newline) on the debug port for functional data. The first
part of the input string is matched in the local structured array to check for a
matching command. All additional data will be treated as parameters.

NOTE TO THE PROGRAMMER:
To expand on the functions of the console, add your unique command, function
pointer and help text to the Console Menu structure array. Then add your
function (remember the prototype) to the bottom of the file. The pointer to
the start of the parameters (if any) will be passed as the parameter to your
function. NULL means no parameters followed the command.

 *******************************************************************************/

/*******************************************************************************
includes
 *******************************************************************************/
#define __NOT_EXTERN__
#include "dev_console.h"
#undef __NOT_EXTERN__

//#include "devMotorControl.h"
//#include "halTLC5615.h"
#include "version.h"
#include "std_utils.h"

#ifdef CONSOLE_ENABLED

/*******************************************************************************
local defines
 *******************************************************************************/
#define MAX_ARGS 10

//map PrintF() to SerialPrintf()  (Can't use printf() as the IDE uses it to include stdio.h and then our define would create problems. )
//#define PrintF SerialPrintf

static st_console_menu_item * FirstMenuItem;
//static st_console_menu_item * LastMenuItem;

//static unsigned long Dev_DumpAddr = 0x10000L;
//static unsigned int Dev_DumpLen = 256;

static bool help_request_flag = false; // can be "help", "help XXXX" or "XXXX help"
//static unsigned int Dev_DumpDev = 0;

/*******************************************************************************
local variables
 *******************************************************************************/

/*******************************************************************************
local function prototypes
 *******************************************************************************/
st_console_menu_item * find_menu_entry_in_siblings(st_console_menu_item *pt, char * commandStr);
bool menuDumpRAM(void);
bool menuDumpFlash(void);
bool menuDump_Generic(char * memType, unsigned long memStart, unsigned long memEnd, unsigned long * memDumpAddr, unsigned int * memDumpLen);
bool menuTogglePrintFlags(void);
bool menuPin(void);
void menuPinToggle(void);
void menuPinHi(void);
void menuPinLo(void);

const char * get_trace_name(int flagIndex);

void show_menu_items(st_console_menu_item * pt, bool useParent);
void parse_line(void);
bool str_in_list(char * str, const char ** list);
int parse_params(char * paramStr, bool terminate);
//char * paramsItem(int index);
bool is_next_arg_help(void);

/*******************************************************************************
local structure
 *******************************************************************************/
#ifdef MAIN_DEBUG
typedef struct
{
	int Pin;
	bool State;
}st_debug_port;
#endif /*MAIN_DEBUG*/


/*******************************************************************************
local variables
 *******************************************************************************/
char devConsole_tag[] = "[CON]";
ST_CONSOLE    stDev_Console;
bool devConsole_initOK = false;

const char u8Dev_BackSpaceEcho[] = {0x08, 0x20, 0x08, 0x00};

static byte _traceMask = trALL;

ST_PRINT_FLAG_ITEM traceFlags[8] 	 = {
		{trMAIN, 		"Main"},	/* 1 - 0x01 */
		{trCONSOLE, 	"Console"},	/* 2 - 0x02 */
		{trRGB, 		"RGB"},		/* 3 - 0x04 */
		{trBUTTON, 		"Button"},	/* 4 - 0x08 */
		{trI2C, 		"I2C"},		/* 5 - 0x10 */
		{trPWM, 		"PWM"},		/* 6 - 0x20 */
		{trYYY, 		""},	/* 7 - 0x40 */
		{trALWAYS, 		"ALWAYS"}	/* 8 - 0x80 */
};

#ifdef MAIN_DEBUG
#define MAX_DEBUG_PINS 6
st_debug_port debugPort[MAX_DEBUG_PINS];
#endif /*MAIN_DEBUG*/

char * _paramStr[MAX_ARGS];  // We can keep pointers to up to 10 parameters in here...
int _paramCnt;
int _paramIndex;

//st_console_menu_item devMenuItem_help 		= {NULL, "read", show_menu_items,	"Duh!"};
//st_console_menu_item devMenuItem_dump 		= {NULL, "write", menuDumpMem,	"Dump memory (as bytes). Dump <ADDR(hex)> <LEN(dec)>"};

//static st_console_menu_item devMenuItem_help 		= { "help", show_menu_items,	"Duh!"};
static st_console_menu_item devMenuItem_dump_RAM    = { "ram", menuDumpRAM,	"RAM information"};
static st_console_menu_item devMenuItem_dump_Flash	= { "flash", menuDumpFlash,	"Dump Flash (as bytes). Dump <ADDR(hex)> <LEN(dec)>"};
static st_console_menu_item devMenuItem_Print 		= { "trace",menuTogglePrintFlags,	"Set/Clear debug print traces"};
#ifdef MAIN_DEBUG
static st_console_menu_item devMenuItem_DbgPin 	    = {"pin",  menuPin, "Get/Set the state of Debug pins"};
// static st_console_menu_item devMenuItem_DbgPin_tg 	= {"toggle",  menuPinToggle, "Toggle Debug pins A(0 to 5)"};
// static st_console_menu_item devMenuItem_DbgPin_hi 	= {"hi",  menuPinHi, "Sets Debug pins A(0 to 5) High"};
// static st_console_menu_item devMenuItem_DbgPin_lo 	= {"lo",  menuPinLo, "Sets Debug pins A(0 to 5) Low"};
#endif /* MAIN_DEBUG */

/*******************************************************************************
Function prototypes
*******************************************************************************/

extern "C" {
	int serialputc(char c, FILE *fp)
  	{
		if(c == '\n')
			Serial.write('\r');
		return Serial.write(c);
	}
}

/*******************************************************************************

Replaces the printf I am so used to.
Override to accommodate flags (to turn certain things on or off).

 *******************************************************************************/
void _SerialPrintf(int traceflags, const char *fmt, ...)
{
FILE stdiostr;
va_list ap;

	if ((_traceMask & traceflags) == trNONE)
		return;

	//RVN TODO - we could check for our custom "%?" flags at the start of the string here and then print the trace banner "[xxxx]" if it is set and then pass on fmt+2 to the vfprintf_P() function.

	fdev_setup_stream(&stdiostr, serialputc, NULL, _FDEV_SETUP_WRITE);

	va_start(ap, fmt);
	vfprintf_P(&stdiostr, fmt, ap);
	va_end(ap);
}

/*******************************************************************************

Replaces the printf I am so used to.

 *******************************************************************************/
void _SerialPrintf(const char *fmt, ...)
{
FILE stdiostr;
va_list ap;

  fdev_setup_stream(&stdiostr, serialputc, NULL, _FDEV_SETUP_WRITE);

  va_start(ap, fmt);
  vfprintf_P(&stdiostr, fmt, ap);
  va_end(ap);
}

/*******************************************************************************

Initialises the Console

 *******************************************************************************/
bool devConsoleInit(unsigned long baud, byte config)
{
	if (devConsole_initOK)
		return true;

	devConsole_initOK = false;

    // The Console will run on the Debug port
	//Serial.begin(115200, SERIAL_8N1);
	Serial.begin(baud, config);
	Serial.flush();

    FirstMenuItem = 0;
    //LastMenuItem = 0;

	devConsole_initOK = true;

	//strncpy(devConsole_tag, "[CON]", 5);
	//devConsole_tag[5] = 0;


	//Assign the "private"
	//addMenuItem(&devMenuItem_help);
	addMenuItem(&devMenuItem_dump_RAM);
	addMenuItem(&devMenuItem_dump_Flash);
	addMenuItem(&devMenuItem_Print);
#ifdef MAIN_DEBUG
	addMenuItem(&devMenuItem_DbgPin);
	// addMenuItem(&devMenuItem_DbgPin_tg, &devMenuItem_DbgPin);
	// addMenuItem(&devMenuItem_DbgPin_hi, &devMenuItem_DbgPin);
	// addMenuItem(&devMenuItem_DbgPin_lo, &devMenuItem_DbgPin);
    
#endif /* MAIN_DEBUG */

#ifdef MAIN_DEBUG
    
	debugPort[0].Pin = pinDEBUG_0;
	debugPort[1].Pin = pinDEBUG_1;
	debugPort[2].Pin = pinDEBUG_2;
	debugPort[3].Pin = pinDEBUG_3;
	debugPort[4].Pin = pinDEBUG_4;
	debugPort[5].Pin = pinDEBUG_5;
	for (int i = 0; i < MAX_DEBUG_PINS; i++)
	{
		debugPort[i].State = false;
		pinMode(debugPort[i].Pin, OUTPUT);
        quickPinToggle(debugPort[i].Pin, LOW);
		//digitalWrite(debugPort[i].Pin, LOW);
		iPrintF(trCONSOLE, "%sDebug Pin %d INIT OK (%d)\n", devConsole_tag, i, debugPort[i].Pin);
	}
#endif /*MAIN_DEBUG*/

	iPrintF(trCONSOLE, "\n%sInit OK\n", devConsole_tag);

	return true;
}
/*******************************************************************************

Sets selected trace Flags

 *******************************************************************************/
void SetTraces(uint8_t flag_mask)
{
	// for (int i = 0; i < MAX_TRACE_FLAGS; i++)
    //     if (flag_mask & _BV(i))
    //         iPrintF(trCONSOLE, "%sTrace %s SET\n", devConsole_tag, get_trace_name(i));
    // iPrintF(trCONSOLE, "%sTraceMask 0x%02X -> ", devConsole_tag, _traceMask);
    _traceMask |= flag_mask;
    // iPrintF(trCONSOLE, "0x%02X\n", _traceMask);
}

/*******************************************************************************

Sets selected print Flags

 *******************************************************************************/
void ClearTraces(uint8_t flag_mask)
{
	// for (int i = 0; i < MAX_TRACE_FLAGS; i++)
    //     if (flag_mask & _BV(i))
    //         iPrintF(trCONSOLE, "%sTrace %s CLEAR (0x%02X)\n", devConsole_tag, get_trace_name(i), _BV(i));
    // iPrintF(trCONSOLE, "%sTraceMask 0x%02X -> ", devConsole_tag, _traceMask);
    _traceMask &= (~flag_mask);
    // iPrintF(trCONSOLE, "0x%02X\n", _traceMask);
}

/*******************************************************************************

Sets selected print Flags

 *******************************************************************************/
uint8_t GetTraceMask(const char * traceName)
{
	for (int i = 0; i < MAX_TRACE_FLAGS; i++)
	{
		if ( strcasecmp(traceName, traceFlags[i].Name) == 0)
			return _BV(i);
	}
	return 0;
}

const char * get_trace_name(int flagIndex)
{
	if (flagIndex < MAX_TRACE_FLAGS)
	{
		return traceFlags[flagIndex].Name;
	}
	return "";
}

/*******************************************************************************

Reads the data on the serial port and parses the line when a carriage return is
encountered.

 *******************************************************************************/
bool Console_Read(void)
{
	//UINT16 rxBytes, cnt;
	//UINT16 rxCnt;
	byte rxData;
	bool retVal = false;

	if (devConsole_initOK == false)
	{
		iPrintF(trCONSOLE | trALWAYS, "%sNot Initialized yet\n", devConsole_tag);
		return false;
	}

	while (Serial.available() > 0)
	{
		// read the incoming byte:
		rxData = Serial.read();
		//printf("Received: %c (0x%02X)  - Inptr @ %d\n", rxData, rxData, stDev_Console.InPtr);
		retVal = true;

		// Skip Newline characters
		if (rxData == '\n')
			continue;

		// Lines are terminated on Carriage returns.
		switch (rxData)
		{
		// ****** Newline ******
		case '\n':
			// Skip Newline characters
			continue;

			// ****** Carriage Return ******
		case '\r':
			PrintF("\n");
			//Serial.write('\n');
			// Now parse the line if it is Valid
			if (stDev_Console.InPtr > 0)
			{
				//printf("parse_line() called\n");
				parse_line();
			}
			// Start a new line now....
			stDev_Console.InPtr = 0;      // Reset index pointer
			break;

			// ****** Backspace ******
		case 0x08:
			// Move one char back in buffer
			if (stDev_Console.InPtr)
			{
				stDev_Console.InPtr--;
				Serial.print(u8Dev_BackSpaceEcho);
			}
			break;

			// ****** Escape ******
		case 0x1B:
			// Clear the line...
			while (stDev_Console.InPtr)
			{
				Serial.print(u8Dev_BackSpaceEcho);
				stDev_Console.InPtr--;
			}
			stDev_Console.InPtr = 0;
			break;

			// ****** All other chars ******
		default:
			// Must we echo the data on the console?
			Serial.write(rxData);

			// Do we still have space in the rx buffer?
			if (stDev_Console.InPtr < CONSOLE_RX_BUFF) //Wrap index?
			{
				// Add char and Null Terminate string
				stDev_Console.RxBuff[stDev_Console.InPtr++] = rxData;
			}
			else  // The index pointer now wraps, our data is invalid.
			{
				stDev_Console.InPtr     = 0;      // Reset the index pointer
			}
			break;
		}

		// Always NULL terminate whatever is in the buffer.
		stDev_Console.RxBuff[stDev_Console.InPtr] = 0;  // Null Terminate
	}

	return retVal;
}

/*******************************************************************************

Parses the line received on the Debug port.

 *******************************************************************************/
void parse_line(void)
{
	st_console_menu_item *pt = FirstMenuItem;
	char *commandStr;

    help_request_flag = false;

    //Let's seperate all the parameters for the guys to use further on (max 10)
    if (parse_params(stDev_Console.RxBuff, true) > 0)
    {
        // is the first parameter a "?" or "help"?
        is_next_arg_help();
    }

    // iPrintF(trCONSOLE, "%s%d parameters:\n", devConsole_tag, paramCnt());
    // for (int i = 0; i < paramCnt(); i++)
    //     iPrintF(trCONSOLE, "%s    \"%s\" \n", devConsole_tag, getParam(i));

	//while ((pt) && (paramsGetCnt() > 0))
	while (pt)
    {
        //Save the last "good" menu item we have found
        st_console_menu_item * first_born = pt;

        //Now we are looking for the next argument passed in the command line
        commandStr = strlwr(paramsGetNext());

        //Checking if the next command is found in the siblings of the current menu item
        //pt = find_menu_entry_in_siblings(pt_last_good, commandStr);

        if (!(pt = find_menu_entry_in_siblings(first_born /*pt_last_good*/, commandStr)))
        {
            ///Could not find the command in the siblings of the current menu item or no command passed
            if ((commandStr != NULL) && (strlen(commandStr) > 0)) 
            {
                pt = first_born;
                int i = (help_request_flag)? 1 : 0;
                int stop = pt->level + i;
                if (i < stop)
                {
                    PrintF("\n\t", commandStr);
                    for (; i < stop; i++)
                        PrintF("%s->", _paramStr[i] /*paramsItem(i)*/);
                }
                PrintF(" ... \"%s\" not found!!!\n\n", commandStr);
            }

            show_menu_items(first_born, true); //Command not found on this level, show the available commands
            return;
        }    
        //iPrintF(trCONSOLE, "%sMatched \"%s\" with \"%s\" (%d)\n", devConsole_tag, commandStr, pt->Command, pt->level);

        //Okay, we found the sibling... What are our options now?
        // 1) This command has no children - Call it's function
        // 2) This command has children, but we are out of arguments (or the next argument is a "?" or "help" - Call "help" on the next level of the menu
        // 3) This command has children and we have more arguments - Iterate through the next level of the menu

        if (!help_request_flag)
            is_next_arg_help();

        if (pt->FirstBorn == NULL)
        {
            // iPrintF(trCONSOLE, "%s\"%s\" called with %d parameters:\n", devConsole_tag, commandStr, paramCnt());
            // for (int i = 0; i < paramCnt(); i++)
            //     iPrintF(trCONSOLE, "%s    %s \n", devConsole_tag, getParam(i));

            // if (help_request_flag)
            //     show_menu_items(pt, false);
            // else
            //iPrintF(trCONSOLE, "%s\"%s\" (%d) called with %d arguments(%d)\n", devConsole_tag, pt->Command, pt->level, paramsGetCnt());
            pt->Func();
            return;
        }
        else // (pt->FirstBorn != NULL)
        {
            if (paramsGetCnt() == 0)
            {

                //RVN - continue from here. Trying to figure out how to a parent function with chilren if it has a function specified

                // 2) This command has children, but we are out of arguments - Call "help" on the next level of the menu
                //Unless it has a function to call, then rather call that function
                if (pt->Func)
                {
                    iPrintF(trCONSOLE, "%s Parent function \"%s\" (%d) called with %d arguments(%d)\n", devConsole_tag, pt->Command, pt->level, paramsGetCnt());
                    pt->Func();
                    //show_menu_items(pt, true);//(char *)pt->Command);
                    return;
                }
                else
                {
                    show_menu_items((st_console_menu_item *)pt->FirstBorn, true);//(char *)pt->Command);
                    return;
                }


                show_menu_items(pt, true);//(char *)pt->Command);
                return;
            }
            pt = (st_console_menu_item *)pt->FirstBorn;
            //else// 3) This command has children and we have more arguments - Iterate through the next level of the menu
        }

    }
}

/*******************************************************************************

Look for the command string within the siblings of the passed menu item

 *******************************************************************************/
bool is_next_arg_help(void)
{   
    char * commandStr = _paramStr[/*index] paramsItem*/(_paramCnt - paramsGetCnt())]; 
    if ((0 == strcasecmp("?", commandStr)) || (0 == strcasecmp("help", commandStr)))
    {
        help_request_flag = true;
        paramsGetNext(); // skip the "?" or "help"
    }
    return help_request_flag;
}

bool help_req(void)
{
    return help_request_flag;
}

/*******************************************************************************

Look for the command string within the siblings of the passed menu item

 *******************************************************************************/
st_console_menu_item * find_menu_entry_in_siblings(st_console_menu_item *pt, char * commandStr)
{    
    if ((commandStr == NULL) || (strlen(commandStr) == 0))
    {
        //This should never happen.....but just in case
        //iPrintF(trCONSOLE, "%sNo command found\n", devConsole_tag);
        return NULL;
    }

    //Now we are looking for the command in the linked list
	while (pt)
	{
		//iPrintF(trCONSOLE, "%sComparing: \"%s\"\n", devConsole_tag, pt->Command);

		if (0 == strcasecmp(pt->Command, commandStr))
            break; //Found a match

        //Let's try the next sibling
		pt = (st_console_menu_item *)pt->Next;
	}

    return pt;
}

/*******************************************************************************

Adds a Menu Item Structure (with a specific callback function pointer) on the
Console Menu linked list.

 *******************************************************************************/
st_console_menu_item * addMenuItem(st_console_menu_item * menuItem, st_console_menu_item * parent)
{
	st_console_menu_item *pt;
	st_console_menu_item *last_list_item = 0;

	//Do not bother if we have not been initialized
	if (devConsole_initOK == false)
	{
		iPrintF(trCONSOLE | trALWAYS, "%sNot Initialized\n", devConsole_tag);
		return NULL;
	}
	// check first if the item is not already in the list
    if (parent == NULL)
    {
        //We start at the first item on the top level
	    pt = FirstMenuItem;
    	//PrintF("%sAdd \"%s\" to the Menu Item List\n", devConsole_tag, menuItem->Command);
    }
    else if ((parent->Prev == NULL) && (parent->Next == NULL))
    {
		iPrintF(trCONSOLE | trALWAYS, "%sParent item \"%s\" not added yet\n", devConsole_tag, parent->Command);
		return NULL;
    }
    else
    {
        //We start at the first child of the parent
        pt = (st_console_menu_item *)parent->FirstBorn; //Will be 0 for the first child added
    }


	while (pt)
	{

		//PrintF("Checking list for presence of %s (%s)\n", menuItem->Command, pt->Command);
		if (pt == menuItem)
		{
			iPrintF(trCONSOLE, "%s\"%s\" already exists in this List\n", devConsole_tag, menuItem->Command);
			break;
		}
        else if (0 == strcasecmp(pt->Command, menuItem->Command))
        {
			iPrintF(trCONSOLE, "%s\"%s\" already in use in this list\n", devConsole_tag, menuItem->Command);
			return NULL;
        }
        last_list_item = pt;
		pt = (st_console_menu_item *)pt->Next;
	}

	if (!pt)
	{
		// append item to linked list
		menuItem->Prev = (void *)last_list_item; //0 for the 1st entry
		menuItem->Next = 0;
        if (last_list_item)
            last_list_item->Next = (void *)menuItem;
        menuItem->Parent = parent;            
		if (parent == NULL) //TOP level menu structure
        {
            menuItem->level = 0;
            // append item to linked list
            if (!FirstMenuItem)
            {
                //Adding menuItem->Command as the first Item in the list
                FirstMenuItem = menuItem;
            }
			//iPrintF(trCONSOLE, "%s\"%s\" added to list\n", devConsole_tag, menuItem->Command);
        }
        else    //SUB level menu structure
        {
            menuItem->level = parent->level+1;
            if (!parent->FirstBorn)
            {
                //Adding menuItem->Command as the first child to the parent
                parent->FirstBorn = menuItem;
            }
			iPrintF(trCONSOLE, "%s\"%s\" added to \"%s\"\n", devConsole_tag, menuItem->Command, parent->Command);
        }
		
	}
	return menuItem;
}
/*******************************************************************************

Finds the parameters in the passed string (seperated by spaces)
Returns the number found.

 *******************************************************************************/
bool str_in_list(char * str, const char ** list)
{
    for (int i = 0; list[i]; i++)
    {
        if (0 == strcasecmp(str, list[i]))
            return true;
    }
    return false;
}
/*******************************************************************************

Finds the parameters in the passed string (seperated by spaces)
Returns the number found.

 *******************************************************************************/
int parse_params(char * paramStr, bool terminate)
{
    char * token;

	//Start by clearing all the previously saved pointers...
	for (int i = 0; i < MAX_ARGS; i++)
		_paramStr[i] = 0;
	_paramCnt = 0;
    _paramIndex = 0;

    while (_paramCnt < (MAX_ARGS-1))
    {
        token = strsep(&paramStr, " ");
        if (token == NULL)
            break;
        if (*token == 0)
            continue;
        _paramStr[_paramCnt++] = token;

    } 
    if (paramStr)
    {
        _paramStr[_paramCnt++] = paramStr;
    }

	return _paramCnt;
}

/*******************************************************************************

Pops the next paramater out of the "stack"

 *******************************************************************************/
char * paramsGetNext(void)
{
    char * param;
    
    if (_paramIndex >= _paramCnt)
        return NULL;

    param = _paramStr[_paramIndex];
    _paramIndex++;

	return param;
}
/*******************************************************************************

Returns the number of parameters left in the "stack"

 *******************************************************************************/
uint8_t paramsGetCnt(void)
{
    return (_paramCnt - _paramIndex);
}


/*******************************************************************************

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

      Add the Console Command functions from here on down in the file.

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

 *******************************************************************************/

/*******************************************************************************

Prints the help string for either the command in question, or for the entire
list of commands.

 *******************************************************************************/
void show_menu_items(st_console_menu_item * pt, bool useParent)//, char * cmd_str)//void)
{
    if ((!pt) || ((useParent) && (pt->level == 0)))
    {
    	PrintF("The list of available commands are:\n\n");
        pt = FirstMenuItem;
    }
    else if (useParent) // pt && pt->level > 0 && useParent
    {
        PrintF("The list of commands available under \"%s\" are:\n",((st_console_menu_item *)(pt->Parent))->Command);
    }
    else if (pt->FirstBorn)// pt && !useParent && pt->FirstBorn
    {
        PrintF("The list of commands available under \"%s\" are:\n",pt->Command);
        pt = (st_console_menu_item *)pt->FirstBorn;
    }
    else // pt && !useParent && !pt->FirstBorn // Level > 0 && !useParent && !pt->FirstBorn
    {
		PrintF("% 12s - %s.\n", pt->Command, pt->HelpStr);
        pt = 0;
    }
    
	// Run through the list and print all the Help strings...
	//pt = FirstMenuItem;
	while (pt)
	{
		//PrintF("% 12s - % 5s%s.\n", pt->Command, ((pt->Tag) ? pt->Tag : ""), pt->HelpStr);
		PrintF("% 12s - %s.\n", pt->Command, pt->HelpStr);
		pt = (st_console_menu_item *)pt->Next;
	}
	PrintF("\nNOTE: Enter arguments after the command.\n");
	PrintF("       Command strings longer than %d chars are invalid.\n", CONSOLE_RX_BUFF);
}

/*******************************************************************************

Dumps a block of RAM memory in Byte Access

*******************************************************************************/
bool menuDumpRAM(void)
{
    static unsigned long RAM_DumpAddr = RAMSTART;
    static unsigned int RAM_DumpLen = 256;

    return menuDump_Generic((char *)"RAM", RAMSTART, RAMEND, &RAM_DumpAddr, &RAM_DumpLen);
}

/*******************************************************************************

Dumps a block of FLASH memory in Byte Access

*******************************************************************************/
bool menuDumpFlash(void)
{
    static unsigned long FLASH_DumpAddr = 0l;//0x10000L;
    static unsigned int FLASH_DumpLen = 256;

    return menuDump_Generic((char *)"FLASH", 0, FLASHEND, &FLASH_DumpAddr, &FLASH_DumpLen);
}

/*******************************************************************************

Dumps a block of RAM/FLASH memory in Byte Access (Generic)

*******************************************************************************/
bool menuDump_Generic(char * memType, unsigned long memStart, unsigned long memEnd, unsigned long * memDumpAddr, unsigned int * memDumpLen)
{
    unsigned long tmp_addr = *memDumpAddr;
    unsigned long tmp_len = (unsigned long)*memDumpLen;
    bool is_ram_not_flash = (memStart == RAMSTART);
    bool help_reqested = help_req();

    char *argStr = paramsGetNext();

    if (!help_reqested)
    {
        if (!argStr)
        {
            //Show RAM status
            if (is_ram_not_flash)
            {
                extern int __heap_start, *__brkval;//, __malloc_heap_start;
                PrintF("  HEAP Start: 0x%06lX\n", (int)&__heap_start);
                PrintF("  HEAP End:   0x%06lX\n", (int)__brkval);
                PrintF("  Free RAM:   %d bytes\n", freeRam());
            }
            else
            {
                PrintF("  FLASH Start: 0x%06X\n", 0);
                PrintF("  FLASH End:   0x%06lX\n", (unsigned long)(FLASHEND));
            }
            PrintF("  Next read:   %d bytes from 0x%06lX\n", *memDumpLen, *memDumpAddr);
            PrintF("\n");
            return true;
        }
        else if (0 == strcasecmp(argStr, "?")) //is it a ?
        {
            help_reqested = true; //Disregard the rest of the arguments
        }
        else if (0 == strcasecmp(argStr, "dump")) // dump
        {
            //Get address
            argStr = paramsGetNext();
            if (argStr) //Address
            {
                if (!isHexStr(argStr))
                {
                    PrintF("Invalid %s Start address (", memType);
                    PrintF("\"%s\"", argStr);
                    PrintF(")\nPlease use address between 0x%06lX and 0x%06lX\n", memStart, memEnd);
                    PrintF("\n");
                    return true;
                }

                //sscanf(strupr(argStr), "%lX", &tmp_addr);
                tmp_addr = strtoul(argStr, NULL, 16);
                PrintF("Got Address: 0x%06X from \"%s\"\n", tmp_addr, argStr);

                if ((tmp_addr < memStart) || (tmp_addr >= memEnd))
                {
                    PrintF("Invalid %s Start address (", memType);
                    PrintF("0x%06X", tmp_addr);
                    PrintF(")\nPlease use address between 0x%06lX and 0x%06lX\n", memStart, memEnd);
                    PrintF("\n");
                    return true;
                }
                argStr = paramsGetNext();
                if ((argStr) && (isNaturalNumberStr(argStr))) //Length
                {
                    tmp_len = (unsigned long)atol(argStr); //sscanf(argStr, "%lu", &tmp_len);
                    PrintF("Got Length: %lu from \"%s\"\n", tmp_len, argStr);

                    //TODO - RVN, you should perform better handling of invalid length values here
                }
            }

            *memDumpAddr = tmp_addr;
            *memDumpLen = (unsigned int)tmp_len;

            //Does the read run over the end of the FLASH?
            if ((tmp_len >= ((memEnd + 1l) - tmp_addr)))
            {
                tmp_len = memEnd - tmp_addr + 1;
                PrintF("%s Dump length limited to %ld bytes: 0x%06lX to 0x%06lX\n", memType, tmp_len, tmp_addr, memEnd);
                PrintF("\n");
            }

            // OK, I honestly did not think the compiler will allow this. Cool.
            ((is_ram_not_flash)? Print_RAM : Print_FLASH)(trCONSOLE | trALWAYS, (void *)*memDumpAddr, (u32)*memDumpAddr, tmp_len);
            PrintF("\n");

            *memDumpAddr += *memDumpLen;
            //If we reach the end of RAM, reset the address
            if (*memDumpAddr > memEnd)
                *memDumpAddr = memStart;

            return true;

        }
        else
        {
            help_reqested = true;
            PrintF("Invalid argument \"%s\"\n", argStr);
            PrintF("\n");
        }
    }
    if (help_reqested)
    {
        PrintF("Valid commands:\n");
        PrintF(" <No Args> - Shows %s summary\n", memType);
        PrintF(" \"dump <ADDR> <LEN>\" - Dumps N bytes of %s starting at <ADDR>\n", memType);
        PrintF(" \"dump\" - Dumps previously set N bytes (default: 256) of %s starting\n", memType);
        PrintF("           at end of last call (default: %s Start)\n", memType);
        PrintF("\n");
        return true;
    }    
    return false;
}
/*******************************************************************************


 *******************************************************************************/
bool menuTogglePrintFlags(void)
{
    uint8_t affected_mask = 0;
    int state_mask = -1; /* -1 = unspecified, 0 = OFF, 1 = ON*/
    bool help_reqested = help_req();

    //str_in_list("ALL", traceFlags);
	//Get pointers to all the arguments

    char *argStr = paramsGetNext();

    if (!argStr)    // Just print all the traces without changing anything
        affected_mask = trALL;

    while ((argStr) && (!help_reqested))
    {
        // Check if the user passed the "?"
        if (0 == strcasecmp(argStr, "?"))
        {
            help_reqested = true;
            break; //disregard the rest of the arguments
        }
        // Check if the user passed the "ALL" keywords
        else if (0 == strcasecmp(argStr, "ALL"))
        {
            affected_mask = trALL;
        }
        // Check if the user passed the "ON" or "OFF" keywords
        else if (0 == strcasecmp(argStr, "ON")) //Should be the last argument evaluated
        {
            if (0 == affected_mask) //Treat as "ALL ON"
                affected_mask = trALL;
            state_mask = 1;
            break; //from while loop, disregard the rest of the arguments
        }
        else if (0 == strcasecmp(argStr, "OFF")) //Should be the last argument evaluated
        {
            if (0 == affected_mask) //Treat as "ALL OFF"
                affected_mask = trALL;
            state_mask = 0;
            break; //from while loop, disregard the rest of the arguments
        }
        else //we have to assume the user passed a flag name
        {
            bool found = false;
            for (int i = 0; i < MAX_TRACE_FLAGS; i++)
            {
                if (0 == strcasecmp(argStr, get_trace_name(i)))
                {
                    affected_mask |=  _BV(i);
                    found = true;
                    break; //... from FOR loop
                }
            }
            if (!found)
            {
                PrintF("Invalid trace name \"%s\"\n", argStr);
                PrintF("\n");
                affected_mask = 0;
                state_mask = -1;
                help_reqested = true;
                break; //from while loop, disregard the rest of the arguments
            }
            //Allright, we found a match, let's see if the user passed a state or more flags
        }
        argStr = paramsGetNext();
    }

    if (0 != affected_mask) //We have some flags to get/set
    {
        if (state_mask != -1) //Set
        {
            if (1 == state_mask)
            {
                SetTraces(affected_mask);
            }
            else
            {
                ClearTraces(affected_mask);
            }
        }
		PrintF("Traces: (0x%02X)\n", _traceMask);
		for (int i = 0; i < MAX_TRACE_FLAGS; i++)
        {
    	    PrintF("% 10s : %s %s\n", get_trace_name(i), (_traceMask & _BV(i))? "ON " : "OFF", ((state_mask != -1) && (affected_mask & _BV(i)))? "*" : "");
            // if (affected_mask & _BV(i))
			//     PrintF("% 10s : %s\n", get_trace_name(i), (_traceMask & _BV(i))? "ON" : "OFF");
        }
    	PrintF("\n");
    }
    if (help_reqested)
    {
        PrintF("Valid commands:\n");
        PrintF(" \"<NAME>\"          - Shows state of <NAME> trace(s)\n");
        PrintF(" \"<NAME> <ON/OFF>\" - Turns <NAME> traces ON or OFF\n");
        PrintF("\n");
        PrintF("   Multiple <NAME>s can be specified.\n");
        PrintF("   Options: ALL");
        for (int i = 0; i < MAX_TRACE_FLAGS; i++)
            PrintF(", %s", get_trace_name(i));
        PrintF("\n");
    	PrintF("\n");
    }    
    return true;
}


/*******************************************************************************

Displays the state of the debug pins A(0 to 5)
    "pin" - Displays the state of ALL the Debug pins A(0 to 5)
    "pin <#>" - Displays the state of the Debug pin A<#>
    "pin <#> Toggle/Hi/Lo/1/0/On/Off" - Sets/Changes the state of the Debug pin A<#>
    "pin ?" - Displays the help for the "pin" command

 *******************************************************************************/
#ifdef MAIN_DEBUG
bool menuPin(void)
{
    char *argStr = paramsGetNext();
    bool help_reqested = help_req();

    if (!help_reqested)
    {
        if ((!argStr) || (0 == strcasecmp(argStr, "ALL")))// got "pin" only or "pin all"
        {
            PrintF("Debug Pin states:\n");
            for (int i = 0; i < MAX_DEBUG_PINS; i++)
                PrintF("\t\tA%d -> %s \n", i, debugPort[i].State? "HI" : "LO");
            PrintF("\n");
        }
        //else got "pin <something>"
        else if (0 == strcasecmp(argStr, "?")) //is it a ?
        {
            help_reqested = true; //Disregard the rest of the arguments
        }
        else if (isNaturalNumberStr(argStr)) //is it a (pin) number 
        {
            //We are expecting one or 2 arguments... the debug pin number and "toggle/hi/lo"
            uint8_t debugPinNum = atoi(argStr);//= (uint8_t)((*argStr) - '0');
    		PrintF("Pin %d identified\n", debugPinNum);
            if (debugPinNum >= MAX_DEBUG_PINS)
            {
                PrintF("Please specify a debug pin from 0 to %d (got %d)\n", MAX_DEBUG_PINS-1, debugPinNum);
                PrintF("\n");
                return true;
            }
            argStr = paramsGetNext();
            if (argStr)
            {
                bool state = debugPort[debugPinNum].State;
                if (strcasecmp(argStr, "toggle") == 0)
                    debugPort[debugPinNum].State = !debugPort[debugPinNum].State;
                else if (strcasecmp(argStr, "hi") == 0)
                    debugPort[debugPinNum].State = true;
                else if (strcasecmp(argStr, "lo") == 0)
                    debugPort[debugPinNum].State = false;
                else
                {
                    help_reqested = true;
                    PrintF("Invalid argument \"%s\"\n", argStr);
                    PrintF("The options for Debug Pin %d are: HI, LO, or TOGGLE\n", debugPinNum);
                    PrintF("\n");
                    return true;
                }
                if (state != debugPort[debugPinNum].State)
                    quickPinToggle(debugPort[debugPinNum].Pin, debugPort[debugPinNum].State);

                PrintF("Debug Pin %d (%d):", debugPinNum, debugPort[debugPinNum].Pin);
                PrintF(" %s", state? "HI" : "LO");
                PrintF(" ->");
                PrintF(" %s", debugPort[debugPinNum].State? "HI" : "LO");
                PrintF("\n");
            }
            else
            { 
                //No arguments passed, 
                PrintF("Debug Pin %d (%d):", debugPinNum, debugPort[debugPinNum].Pin);
                PrintF(" %s", debugPort[debugPinNum].State? "HI" : "LO");
                PrintF("\n");
            }
            
            //just display the state of the pin
            PrintF("\n");
            return true;
        }
        else
        {
            help_reqested = true;
            PrintF("Invalid argument \"%s\"\n", argStr);
            PrintF("Please specify a debug pin from 0 to %d \n", MAX_DEBUG_PINS-1);
            PrintF("\n");
            return true;
        }
    }

    if (help_reqested)
    {
        PrintF("Valid Pin arguments are:\n");
        PrintF(" \"ALL\" or \"\" - Displays the state of ALL pins");
        PrintF(" (0 to %d)\n", MAX_DEBUG_PINS-1);
        PrintF(" \"<#>\" - Displays state of pin");
        PrintF(" (0 to %d)\n", MAX_DEBUG_PINS-1);
        PrintF(" \"<#> <HI/LO>\" - Sets the state of pin");
        PrintF(" (0 to %d)\n", MAX_DEBUG_PINS-1);
        PrintF(" \"<#>\" - Toggles the state of pin");
        PrintF(" (0 to %d)\n", MAX_DEBUG_PINS-1);
        PrintF("\n");
    }
    return false;
}
void menuPinToggle(void)
{
	PrintF("NOT IMPLEMENTED\n");
	PrintF("\n");
}
void menuPinHi(void)
{
	PrintF("NOT IMPLEMENTED\n");
	PrintF("\n");
}
void menuPinLo(void)
{
	PrintF("NOT IMPLEMENTED\n");
	PrintF("\n");
}
#endif /* MAIN_DEBUG */

/*void DumpMem(int Flags,
void * Src,           // ptr to memory to dump
unsigned long Address,       // address to display
unsigned short Len)               // nr of bytes to dump
*/
void Print_RAM(int Flags, void * Src, unsigned long Address, int Len)
{
u8 *s;
u8 x;
u16 cnt;
//RAMSTART     (0x100)
//RAMEND       0x8FF     /* Last On-Chip SRAM Location */

	s = (u8 *)Src;

	while(Len)
	{
		// print offset
		iPrintF(Flags, "%06lX : ", Address);
		// print hex data
		for(x = 0; x < 16; x++)
		{
			if (x < Len)
				iPrintF(Flags, "%02X%c", (byte)s[x], (byte)((x == 7) ? '-' : ' '));
			else
				iPrintF(Flags, "  %c", (byte)((x == 7) ? '-' : ' '));

		}
		// print ASCII data
		iPrintF(Flags, " ");
		for(x = 0; x < 16; x++)
		{
			if (x < Len)
				iPrintF(Flags, "%c", (byte)(((s[x] >= 0x20) && (s[x] <= 0x7f))? s[x] : '.'));
			else
				break;
		}
		// goto next line
		cnt = (Len > 16) ? 16 : Len;
		s       += cnt;
		Len     -= cnt;
		iPrintF(Flags, "\n");
		Address += 16;
	}

//  iprintf(Flags, "\n");
}

void Print_FLASH(int Flags, void * Src, unsigned long Address, int Len)
{
    u8 buf[16];
    u8 *s = (u8 *)Src;

	while(Len)
	{
        for (int i = 0; i < 16; i++)
            buf[i] = pgm_read_byte(s++);
        
        Print_RAM(Flags, buf, Address, 16);
		Address += 16;
		Len     -= 16;
	}

//  iprintf(Flags, "\n");
}

#endif /* CONSOLE_ENABLED */

/*************************** END OF FILE *************************************/
