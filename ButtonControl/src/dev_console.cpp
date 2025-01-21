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

#ifdef CONSOLE_MENU

/*******************************************************************************
local defines
 *******************************************************************************/
#define MAX_ARGS 10

//map PrintF() to SerialPrintf()  (Can't use printf() as the IDE uses it to include stdio.h and then our define would create problems. )
//#define PrintF SerialPrintf

static st_console_menu_item * FirstMenuItem;
//static st_console_menu_item * LastMenuItem;

static unsigned long Dev_DumpAddr = 0x10000L;
static unsigned int Dev_DumpLen = 256;
//static unsigned int Dev_DumpDev = 0;

/*******************************************************************************
local variables
 *******************************************************************************/

/*******************************************************************************
local function prototypes
 *******************************************************************************/
void(*resetFunc)(void) = 0; //declare reset function at address 0
st_console_menu_item * find_menu_entry(st_console_menu_item *pt, char * commandStr);
void menuPrintHelp(st_console_menu_item * pt, char * cmd_str);
void ParseLine(void);
int paramsParse(char * paramStr, bool terminate);

/*******************************************************************************
local structure
 *******************************************************************************/
#ifdef MAIN_DEBUG
typedef struct
{
	int Pin[6];
	bool State[6];
}ST_DEBUG_PORT;
#endif /*MAIN_DEBUG*/


/*******************************************************************************
local variables
 *******************************************************************************/
char devConsole_tag[] = "[CON]";
ST_CONSOLE    stDev_Console;
bool devConsole_initOK = false;

const char u8Dev_BackSpaceEcho[] = {0x08, 0x20, 0x08, 0x00};

byte traceMask = trALL;

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
ST_DEBUG_PORT debugPort;
#endif /*MAIN_DEBUG*/

char * _paramStr[MAX_ARGS];  // We can keep pointers to up to 10 parameters in here...
int _paramCnt;
int _paramIndex;

//st_console_menu_item devMenuItem_help 		= {NULL, "read", menuPrintHelp,	"Duh!"};
//st_console_menu_item devMenuItem_dump 		= {NULL, "write", menuDumpMem,	"Dump memory (as bytes). Dump <ADDR(hex)> <LEN(dec)>"};

//static st_console_menu_item devMenuItem_help 		= { "help", menuPrintHelp,	"Duh!"};
static st_console_menu_item devMenuItem_dump 		= { "dump", menuDumpMem,	"Dump memory (as bytes). Dump <ADDR(hex)> <LEN(dec)>"};
static st_console_menu_item devMenuItem_Print 		= { "trace",menuTogglePrintFlags,	"Toggle print flags"};
#ifdef MAIN_DEBUG
static st_console_menu_item devMenuItem_DbgPin 	= {"pin",  NULL, "Manipulate Debug pins A(0 to 5)"};
// static st_console_menu_item devMenuItem_DbgPin 	= {"toggle",  menuPinToggle, "Toggle Debug pins A(0 to 5)"};
// static st_console_menu_item devMenuItem_DbgPin 	= {"hi",  menuPinHi, "Sets Debug pins A(0 to 5) High"};
// static st_console_menu_item devMenuItem_DbgPin 	= {"lo",  menuPinLo, "Sets Debug pins A(0 to 5) Low"};
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

	if ((traceMask & traceflags) == trNONE)
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
	addMenuItem(&devMenuItem_dump);
	addMenuItem(&devMenuItem_Print);
#ifdef MAIN_DEBUG
	addMenuItem(&devMenuItem_DbgPin);
#endif /* MAIN_DEBUG */

#ifdef MAIN_DEBUG
	debugPort.Pin[0] = pinDEBUG_0;
	debugPort.Pin[1] = pinDEBUG_1;
	debugPort.Pin[2] = pinDEBUG_2;
	debugPort.Pin[3] = pinDEBUG_3;
	debugPort.Pin[4] = pinDEBUG_4;
	debugPort.Pin[5] = pinDEBUG_5;
	for (int i = 0; i < 6; i++)
	{
		debugPort.State[i] = false;
		pinMode(debugPort.Pin[i], OUTPUT);
		digitalWrite(debugPort.Pin[i], LOW);
		//iPrintF(trCONSOLE, "\n%sDebug Pin %d INIT OK\n", devConsole_tag, i);
	}
#endif /*MAIN_DEBUG*/

	iPrintF(trCONSOLE, "\n%sInit OK\n", devConsole_tag);

	return true;
}
/*******************************************************************************

Sets selected trace Flags

 *******************************************************************************/
void SetTrace(int flagIndex)
{
	if (flagIndex < 7)
	{
		traceMask |= (0x01 << flagIndex);
	}
	else if (flagIndex == trALL)
	{
		for (int i = 0; i < 7; i++)
			traceMask |= (0x01 << i);
	}
}

/*******************************************************************************

Sets selected print Flags

 *******************************************************************************/
void ClearTrace(int flagIndex)
{
	if (flagIndex < 7)
	{
		traceMask &= (~traceFlags[flagIndex].flagMask);
		traceMask |= trALWAYS; //Keep the ALWAYS flag on.
	}
	else if (flagIndex == trALL)
	{
		for (int i = 0; i < 7; i++)
			traceMask &= (~(0x01 << i));
		traceMask |= trALWAYS; //Keep the ALWAYS flag on.
	}
}

/*******************************************************************************

Sets selected print Flags

 *******************************************************************************/
bool GetTrace(int flagIndex)
{
	if (flagIndex < 7)
	{
		return ((traceMask >> flagIndex) & 0x01)? true : false;
	}
	return false;
}

int GetTraceIndex(const char * traceName)
{
	for (int i = 0; i < 8; i++)
	{
		if ( strcasecmp(traceName, traceFlags[i].Name) == 0)
			return i;
	}
	return 9;
}

const char * GetTraceName(int flagIndex)
{
	if (flagIndex < 7)
	{
		return traceFlags[flagIndex].Name;
	}
	return "";
}

/*******************************************************************************

Reads the data on the serial port and parses the line when a carriage return is
encountered.

 *******************************************************************************/
bool Read(void)
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
				//printf("Parseline() called\n");
				ParseLine();
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
void ParseLine(void)
{
	st_console_menu_item *pt;
	char *commandStr;
    bool help_reqested = false; // can be "help", "help XXXX" or "XXXX help"

    //Let's seperate all the parameters for the guys to use further on (max 10)
    paramsParse(stDev_Console.RxBuff, true);

    // iPrintF(trCONSOLE, "%s%d parameters:\n", devConsole_tag, paramCnt());
    // for (int i = 0; i < paramCnt(); i++)
    //     iPrintF(trCONSOLE, "%s    \"%s\" \n", devConsole_tag, getParam(i));

    pt = FirstMenuItem;
	while ((pt) && (paramsGetCnt() > 0))
    {
        commandStr = strlwr(paramGetNext());

        //If this is a "?", then we want to run a help on the searched command
        if ((0 == strcasecmp("?", commandStr)) || (0 == strcasecmp("help", commandStr)))
        {
            help_reqested = true;
            if (paramsGetCnt() > 0) //No more params
                commandStr = strlwr(paramGetNext());
            else
                break;
        }

        //RVN - still need to figure out the help functionality properly

        pt = find_menu_entry(pt, commandStr);
        if (!pt)
        {
            PrintF("\"%s\" not found\n", commandStr);
            return;
        }    
        //Okay, we found the dude... What are our options now?
        // 1) This command has no children - Call it's function
        if (pt->FirstBorn == NULL)
        {
            iPrintF(trCONSOLE, "%sMatched with: \"%s\" (%d)\n", devConsole_tag, pt->Command, pt->level);
            // iPrintF(trCONSOLE, "%s\"%s\" called with %d parameters:\n", devConsole_tag, commandStr, paramCnt());
            // for (int i = 0; i < paramCnt(); i++)
            //     iPrintF(trCONSOLE, "%s    %s \n", devConsole_tag, getParam(i));
            if (help_reqested)
            {
                menuPrintHelp(pt, (char *)pt->Command);
                return;
            }
            pt->Func(&_paramStr[_paramIndex], paramsGetCnt());
            return;
        }
        else 
        {
            pt = (st_console_menu_item *)pt->FirstBorn;
            if (paramsGetCnt() == 0)
            {
                // 2) This command has children, but we are out of arguments - Call "help" on the next level of the menu
                help_reqested = true;
                break;
            }
            //else// 3) This command has children and we have more arguments - Iterate through the next level of the menu
        }
        // 2) This command has children, but we are out of arguments - Call "help" on the next level of the menu
        // 3) This command has children and we have more arguments - Iterate through the next level of the menu

    }
    if ((help_reqested) && (pt))
    {
        menuPrintHelp(pt, (char *)pt->Command);
    }
    else
    {
        PrintF("No command found\n");
    }
}

/*******************************************************************************

Parses the line received on the Debug port.

 *******************************************************************************/
st_console_menu_item * find_menu_entry(st_console_menu_item *pt, char * commandStr)
{    
    if (commandStr == NULL)
    {
        iPrintF(trCONSOLE, "%sNo command found\n", devConsole_tag);
        return NULL;
    }

    //Now we are looking for the command in the linked list
	while (pt)
	{
		iPrintF(trCONSOLE, "%sComparing: \"%s\"\n", devConsole_tag, pt->Command);

		if (0 == strcasecmp(pt->Command, commandStr))
		{
			return pt;
		}
		pt = (st_console_menu_item *)pt->Next;
	}
	return NULL;
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
    	PrintF("%sAdd \"%s\" to the Menu Item List\n", devConsole_tag, menuItem->Command);
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
			iPrintF(trCONSOLE, "%s\"%s\" added to list\n", devConsole_tag, menuItem->Command);
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
int paramsParse(char * paramStr, bool terminate)
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

Returns the number of paramaeters found.

 *******************************************************************************/
int paramTotal(void)
{
	return _paramCnt;
}

/*******************************************************************************

Returns the parameter at the specifed index...0 = first parameter, 4 = last

 *******************************************************************************/
char * paramGetItem(int index)
{
	if ((index < _paramCnt) && (index >= 0))
		return _paramStr[index];
	else
		return NULL;
}
/*******************************************************************************

Pops the next paramater out of the "stack"

 *******************************************************************************/
char * paramGetNext(void)
{
    char * param;
    
    if (_paramIndex >= _paramCnt)
        return NULL;

    param = _paramStr[_paramIndex];
    _paramIndex++;

	return param;
}
/*******************************************************************************

Pops the next paramater out of the "stack"

 *******************************************************************************/
int paramsGetCnt(void)
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
void menuPrintHelp(st_console_menu_item * pt, char * cmd_str)//void)
{
	if (pt->level > 0)
    {
    	PrintF("The list of commands available under \"%s\" are:\n\n", cmd_str);
    }        
    else
    {
    	PrintF("The list of available commands are:\n\n");
    }

	// Run through the list and print all the Help strings...
	//pt = FirstMenuItem;
	while (pt)
	{
		//PrintF("% 12s - % 5s%s.\n", pt->Command, ((pt->Tag) ? pt->Tag : ""), pt->HelpStr);
		PrintF("% 12s - %s.\n", pt->Command, pt->HelpStr);
		pt = (st_console_menu_item *)pt->Next;
	}
	PrintF("\nNOTE: Enter Parameters after the command, following a space.\n");
	PrintF("       Command strings longer than %d chars are invalid.\n", CONSOLE_RX_BUFF);
}

/*******************************************************************************

Dumps a block of memory in Byte Access

*******************************************************************************/
void menuDumpMem(char ** args, int arg_cnt)//void)
{

// get the address from the commandline, if any
  if(paramGetItem(0))
  {
    sscanf(strupr(paramGetItem(0)), "%lX", &Dev_DumpAddr);
// get the length from the commandline, if any

    if(paramGetItem(1))
      sscanf(paramGetItem(0), "%d", &Dev_DumpLen);
  }

// OK, dump
  DumpMem(trCONSOLE | trALWAYS, (void *)Dev_DumpAddr, (u32)Dev_DumpAddr, Dev_DumpLen);
// add len to addr, so the next "Dump" without params just continues...
  Dev_DumpAddr += Dev_DumpLen;
}

/*******************************************************************************


 *******************************************************************************/
void menuTogglePrintFlags(char ** args, int arg_cnt)//void)
{
char *argStr_1;
char *argStr_2;
//int argCnt = 0;
int affectedIndex = -1;

	//Get pointers to all the arguments
	argStr_1 = paramGetItem(0);//paramStr;

	if (argStr_1)
	{
		// Check if the user passed the "ALL" or "NONE" keywords
		if (0 == strcasecmp(argStr_1, "ALL"))
		{
			SetTrace(trALL);
			affectedIndex = 7;
		}
		else if (0 == strcasecmp(argStr_1, "NONE"))
		{
			ClearTrace(trALL);
			affectedIndex = 7;
		}
		else //Perhaps the user passed a flag name
		{
			argStr_2 = paramGetItem(1);//paramStr;
			for (int i = 0; i < 7; i++)
			{
				if (0 == strcasecmp(argStr_1, GetTraceName(i)))
				{
					affectedIndex = i;
					if (paramTotal() > 1)
					{
						//Allright, we found a match
						if (0 == strcasecmp(argStr_2, "ON"))
						{
							SetTrace(i);
						}
						else if (0 == strcasecmp(argStr_2, "OFF"))
						{
							//PrintF("Turn the %s (%d) traces OFF\n", GetPrintFlagsName(i), i);
							ClearTrace(i);
						}
						else
						{
							affectedIndex = -1;
						}
					}
					break; //... from FOR loop
				}
			}
		}
	}
	else //No arguments passed
	{
		affectedIndex = 7;
	}
	if (affectedIndex == 7)
	{
		PrintF("Traces: \n");
		for (int i = 0; i < 7; i++)
			PrintF("% 10s : %s \n", GetTraceName(i), (GetTrace(i))? "ON" : "OFF");
	}
	else if (affectedIndex >= 0)
	{
		PrintF("% 10s : %s \n", GetTraceName(affectedIndex), (GetTrace(affectedIndex))? "ON" : "OFF");
	}
	else
	{
		//PrintF("Debug prints: \n");
		//for (int i = 0; i < 7; i++)
		//	PrintF("% 10s : %s \n", GetPrintFlagsName(i), (GetPrintFlagsBit(i))? "ON" : "OFF");

		PrintF("Valid commands:\n");
		PrintF(" \"ALL\"             - Turns ALL traces ON\n");
		PrintF(" \"NONE\"            - Turns ALL traces OFF\n");
		PrintF(" \"<NAME>\"          - Shows state of traces for <NAME>\n");
		PrintF(" \"<NAME> <ON/OFF>\" - Turns <NAME> traces ON or OFF\n");
	}
	PrintF("\n");
}
/*******************************************************************************

Provides access to the debug pins through the console
"pin ?" to see the options.

 *******************************************************************************/
#ifdef MAIN_DEBUG
void menuPinToggle(char ** args, int arg_cnt)//void)
{
char * paramStr;
char *argStr_1;
char *argStr_2;
//char *argStr_3;
int argCnt = 0;

int debugPinNum;

paramStr = paramGetItem(0);
	//Get pointers to all the arguments
	argStr_1 = nextWord(paramStr, true);
	if (argStr_1)
	{
		argCnt++;;
		argStr_2 = nextWord(argStr_1, true);
		if (argStr_2)
		{
			argCnt++;;
		}
	}

	PrintF("\"pin toggle\" %s called with \"%s\"\n", paramStr, ((argCnt >= 1)? argStr_1 : ""));

	if ((paramStr) && (isdigit(*paramStr)))
	{
		debugPinNum = (int)((*paramStr) - '0');

		PrintF("Pin %d identified\n", debugPinNum);

		if ((debugPinNum >= 0) && (debugPinNum <= 5))
		{
			if (argCnt == 1)
			{
				//PrintF("Param = %s\n", argStr_1);

				//we are expecting a "hi" or a "lo"
				if (0 == strcasecmp(argStr_1, "hi"))
					debugPort.State[debugPinNum] = true;
				else if (0 == strcasecmp(argStr_1, "lo"))
					debugPort.State[debugPinNum] = false;
				else
					argCnt = 8;
			}
			if (argCnt == 0)
			{
				//PrintF("No Param, just toggle\n");
				//Just toggle the pin
				debugPort.State[debugPinNum] = !debugPort.State[debugPinNum];
			}
			if (argCnt < 2)
			{
				//if (debug.State[debugPinNum])
				//	PORTC |= _BV(debugPinNum);
				//else
				//	PORTC &= ~_BV(debugPinNum);

				quickPinToggle(debugPort.Pin[debugPinNum], debugPort.State[debugPinNum]);
				//digitalWrite(debug.Port[debugPinNum], (debug.State[debugPinNum])? HIGH : LOW);
				PrintF("Debug Pin A%d -> %s \n", debugPinNum , debugPort.State[debugPinNum]? "HI" : "LO");
				PrintF("\n");
				return;
			}
		}
	}

	PrintF("Valid Toggle commands are:\n");
	PrintF(" \"<#> <HI/LO>\" - Sets the debug pin A(0 to 5) hi or lo\n");
	PrintF(" \"<#>\" - Toggles the debug pin A(0 to 5)\n");
	PrintF("\n");
}
void menuPinHi(char ** args, int arg_cnt)//void)
{
char * paramStr;
char *argStr_1;
char *argStr_2;
//char *argStr_3;
int argCnt = 0;

int debugPinNum;

paramStr = paramGetItem(0);
	//Get pointers to all the arguments
	argStr_1 = nextWord(paramStr, true);
	if (argStr_1)
	{
		argCnt++;;
		argStr_2 = nextWord(argStr_1, true);
		if (argStr_2)
		{
			argCnt++;;
		}
	}

	//PrintF("Toggle %s called with \"%s\"\n", paramStr, ((argCnt >= 1)? argStr_1 : ""));

	// if no parameter passed then assume the user wanted RPM
	if ((paramStr) && (isdigit(*paramStr)))
	{
		debugPinNum = (int)((*paramStr) - '0');

		//PrintF("Pin %d identified\n", debugPinNum);

		if ((debugPinNum >= 0) && (debugPinNum <= 5))
		{
			if (argCnt == 1)
			{
				//PrintF("Param = %s\n", argStr_1);

				//we are expecting a "hi" or a "lo"
				if (0 == strcasecmp(argStr_1, "hi"))
					debugPort.State[debugPinNum] = true;
				else if (0 == strcasecmp(argStr_1, "lo"))
					debugPort.State[debugPinNum] = false;
				else
					argCnt = 8;
			}
			if (argCnt == 0)
			{
				//PrintF("No Param, just toggle\n");
				//Just toggle the pin
				debugPort.State[debugPinNum] = !debugPort.State[debugPinNum];
			}
			if (argCnt < 2)
			{
				//if (debug.State[debugPinNum])
				//	PORTC |= _BV(debugPinNum);
				//else
				//	PORTC &= ~_BV(debugPinNum);

				quickPinToggle(debugPort.Pin[debugPinNum], debugPort.State[debugPinNum]);
				//digitalWrite(debug.Port[debugPinNum], (debug.State[debugPinNum])? HIGH : LOW);
				PrintF("Debug Pin A%d -> %s \n", debugPinNum , debugPort.State[debugPinNum]? "HI" : "LO");
				PrintF("\n");
				return;
			}
		}
	}

	PrintF("Valid Toggle commands are:\n");
	PrintF(" \"<#> <HI/LO>\" - Sets the debug pin A(0 to 5) hi or lo\n");
	PrintF(" \"<#>\" - Toggles the debug pin A(0 to 5)\n");
	PrintF("\n");
}
void menuPinLo(char ** args, int arg_cnt)//void)
{
char * paramStr;
char *argStr_1;
char *argStr_2;
//char *argStr_3;
int argCnt = 0;

int debugPinNum;

paramStr = paramGetItem(0);
	//Get pointers to all the arguments
	argStr_1 = nextWord(paramStr, true);
	if (argStr_1)
	{
		argCnt++;;
		argStr_2 = nextWord(argStr_1, true);
		if (argStr_2)
		{
			argCnt++;;
		}
	}

	//PrintF("Toggle %s called with \"%s\"\n", paramStr, ((argCnt >= 1)? argStr_1 : ""));

	// if no parameter passed then assume the user wanted RPM
	if ((paramStr) && (isdigit(*paramStr)))
	{
		debugPinNum = (int)((*paramStr) - '0');

		//PrintF("Pin %d identified\n", debugPinNum);

		if ((debugPinNum >= 0) && (debugPinNum <= 5))
		{
			if (argCnt == 1)
			{
				//PrintF("Param = %s\n", argStr_1);

				//we are expecting a "hi" or a "lo"
				if (0 == strcasecmp(argStr_1, "hi"))
					debugPort.State[debugPinNum] = true;
				else if (0 == strcasecmp(argStr_1, "lo"))
					debugPort.State[debugPinNum] = false;
				else
					argCnt = 8;
			}
			if (argCnt == 0)
			{
				//PrintF("No Param, just toggle\n");
				//Just toggle the pin
				debugPort.State[debugPinNum] = !debugPort.State[debugPinNum];
			}
			if (argCnt < 2)
			{
				//if (debug.State[debugPinNum])
				//	PORTC |= _BV(debugPinNum);
				//else
				//	PORTC &= ~_BV(debugPinNum);

				quickPinToggle(debugPort.Pin[debugPinNum], debugPort.State[debugPinNum]);
				//digitalWrite(debug.Port[debugPinNum], (debug.State[debugPinNum])? HIGH : LOW);
				PrintF("Debug Pin A%d -> %s \n", debugPinNum , debugPort.State[debugPinNum]? "HI" : "LO");
				PrintF("\n");
				return;
			}
		}
	}

	PrintF("Valid Toggle commands are:\n");
	PrintF(" \"<#> <HI/LO>\" - Sets the debug pin A(0 to 5) hi or lo\n");
	PrintF(" \"<#>\" - Toggles the debug pin A(0 to 5)\n");
	PrintF("\n");
}
#endif /* MAIN_DEBUG */

/*void DumpMem(int Flags,
void * Src,           // ptr to memory to dump
unsigned long Address,       // address to display
unsigned short Len)               // nr of bytes to dump
*/
void DumpMem(int Flags, void * Src, unsigned long Address, int Len)
{
u8 *s;
u8 x;
u16 cnt;

	// Are we even bothering to print this?
	if ((traceMask & Flags) == trNONE)
		return;

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

#ifdef MAIN_DEBUG
/*******************************************************************************

 Display program version info

 *******************************************************************************/
void menuVersion(char ** args, int arg_cnt)//void)
{
	PrintF("VER %s\n", PROG_VERSIONSTRING);
}

/*******************************************************************************

 Reads or Sets the system time.

 *******************************************************************************/
void menuTime(char ** args, int arg_cnt)//void)
{
	PrintF("Running for %lu s\n", ((long)millis() / 1000));
}

/*******************************************************************************

 Resets the system

 *******************************************************************************/
void menuReset(char ** args, int arg_cnt)//void)
{
	char *paramStr;
	// if no parameter passed then just open the gate
	paramStr = paramGetItem(0);

	if (!paramStr)
	{
		PrintF("Now type 'reset Y', IF YOU ARE SURE.\n");
		return;
	}
	if (*(char *)paramStr != 'Y')
	{
		PrintF("'reset Y' expected. Start over.\n");
		return;
	}
	// ok, do the reset.
	PrintF("Resetting. Goodbye, cruel world!\n");
	Serial.flush();

	resetFunc();
}
#endif /* MAIN_DEBUG */
#endif /* CONSOLE_MENU */

/*************************** END OF FILE *************************************/
