/*******************************************************************************

 Project:   RGB Button Chaser
 Purpose:   This file is the main entry point for the application running on the 
            "slave" devices in the RGB Button Chaser project. The slave devices 
            are responsible for:
             1) Controlling the RGB LEDs
             2) Reading the button inputs
             3) Measuring timing
             4) Handling all communication with the master device

 Author:    Rudolph van Niekerk
 Processor: Arduino Nano (ATmega328P)
 Compiler:	Arduino AVR Compiler

 SHORT SUMMARY OF OPERATION GOES HERE

 The included classes are:

 ** dev_console **
 A class which exposes the serial port for debugging purposes and maintains a
 linked list of menu functions for the other classes

 ** CLASS_1 **

 ** CLASS_2 **
 
  //TODO
 * 1 - 
 * 2 - 
 * 3 - 
 * 4 - 

 *******************************************************************************/
 /*******************************************************************************
  includes
  *******************************************************************************/
#include <Arduino.h>
#include <avr/pgmspace.h>
//#include <SPI.h>
#include "defines.h"

//#include "devMotorControl.h"
//#include "halAMT203.h"
//#include "halTLC5615.h"
//#include "paramCtrl.h"
#include "std_utils.h"
#include "hal_timers.h"
#include "hal_pwm.h"
//#include "appPidControl.h"
//#include "appWaveGen.h"
#include "version.h"
#ifdef CONSOLE_ENABLED
#include "dev_console.h"
#else
#include "devComms.h"
#endif
/*******************************************************************************
 local defines
 *******************************************************************************/
#define SYS_STATE_INIT			0	/* */
#define SYS_STATE_FIND_ZERO		1	/* */
#define SYS_STATE_STOPPED		2	/* */
#define SYS_STATE_MOVING		3	/* */
#define SYS_STATE_ERROR			9	/* */

/*******************************************************************************
 Function prototypes
 *******************************************************************************/
//Menu Commands (without any owners)
#ifdef MAIN_DEBUG
#ifdef CONSOLE_ENABLED
void(*resetFunc)(void) = 0; //declare reset function at address 0
bool menuTime(void);
bool menuReset(void);
bool menuVersion(void);
#endif /* CONSOLE_ENABLED */
#endif /* MAIN_DEBUG */

/*******************************************************************************
 local variables
 *******************************************************************************/
//The help menu structures
#ifdef MAIN_DEBUG
#ifdef CONSOLE_ENABLED
static st_console_menu_item devMenuItem_time = { "time", menuTime, "Returns the system time" };
static st_console_menu_item devMenuItem_reset = { "reset", menuReset, "Resets the system" };
static st_console_menu_item devMenuItem_version = { "version", menuVersion, "Display firmware version" };
#endif /* CONSOLE_ENABLED */
#endif /* MAIN_DEBUG */
//int SystemState;		/* Movement State Machine variable */

const char BuildTimeData[] = { __TIME__ " " __DATE__ }; /* Used in our startup Banner*/

ST_MS_TIMER debugTmr;


/*******************************************************************************
 Functions
 *******************************************************************************/

void setup() {
  // put your setup code here, to run once:

  //Set the system state to INIT
  ClearStatus();

	//Initialize serial communication on the standard port with the USB-2-RS232 converter
	devConsoleInit(115200, SERIAL_8N1);
	//devComms::Init(115200, SERIAL_8N1);
	
  // print our sign-on banner
//  extern int	printf_P(const char *__fmt, ...);

	PrintF("\n");
	PrintF("=====================================================\n");
	PrintF("RGB Button Chaser - Button Controller (Slave)\n");
	PrintF("[c] 2025 Zero Bad Cafe Development (Pty) Ltd\n");
	PrintF("Version   %s.\n", PROG_VERSIONSTRING);
	PrintF("BuildInfo %s.\n", BuildTimeData);
	PrintF("Arduino Nano (Clock %lu MHz)\n", F_CPU / 1000000L);
	PrintF("=====================================================\n");

	//Add the Menu Items which has no owners
#ifdef MAIN_DEBUG
	addMenuItem(&devMenuItem_reset);
	addMenuItem(&devMenuItem_time);
	addMenuItem(&devMenuItem_version);
#endif /* MAIN_DEBUG */


    hal_pwm_start(0.01, 1);

	//Set the print traces we want...
	SetTraces(trALL);
	ClearTraces(GetTraceMask("RGB"));
	ClearTraces(GetTraceMask("Button"));
	ClearTraces(GetTraceMask("I2C"));

	SetStatus(statusOK);

    hal_pwm_assign_pin(13);
    
	msTimerStart(&debugTmr, 10L);	//Does something every 100ms

}

void loop() {
	// put your main code here, to run repeatedly:

	//Check for anything coming in on the Serial Port and process it
	Console_Read();

    //static uint8_t dc = 0;
	//devComms::Read();

	if (msTimerPoll(&debugTmr))
	{
		//iPrintF(trMAIN, "Running for %lu s\n", SecondCount());
        // if (dc > 100)
        //     dc = 0;
        // hal_pwm_set_duty_cycle_percent(13, dc++);
        //hal_pwm_inc_duty_cycle(13, true);
		msTimerReset(&debugTmr);
	}
}

#ifdef MAIN_DEBUG
#ifdef CONSOLE_ENABLED
/*******************************************************************************

 Display program version info

 *******************************************************************************/
bool menuVersion(void)
{
	PrintF("VER %s\n", PROG_VERSIONSTRING);
    return true;
}

/*******************************************************************************

 Reads or Sets the system time.

 *******************************************************************************/
bool menuTime(void)
{
	PrintF("Running for %lu s\n", ((long)millis() / 1000));
    return true;
}

/*******************************************************************************

 Resets the system

 *******************************************************************************/
bool menuReset(void)
{
    static bool reset_lock = false;
	// if no parameter passed then just open the gate

    char *argStr = paramsGetNext();

	if (!reset_lock)
    {
        if (!argStr)
        {
            reset_lock = true;
            PrintF("Now type 'reset Y', IF YOU ARE SURE.\n");
            return true;
        }
        PrintF("No arguments expected (got \"%s\").\n", argStr);
    }
    else //if (reset_lock)
    {
		if (0 == strcasecmp(argStr, "Y"))
        {
            // ok, do the reset.
            PrintF("Resetting. Goodbye, cruel world!\n");
            Serial.flush();
            resetFunc();
            return true;
        }        
        PrintF("'reset Y' expected. Starting over.\n");
    }
    reset_lock = false;
    return true;
}
#endif /* CONSOLE_ENABLED */
#endif /* MAIN_DEBUG */
