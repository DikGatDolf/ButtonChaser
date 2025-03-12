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

This (slave) device  is responsible for:
  1) Controlling a single set RGB LED (3 wires, see dev_rgb.c/h)
  2) Reading a single button inputs
  3) Measuring timing
  4) Responding to all communication with the master devices

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
#include "sys_utils.h"
#include "str_helper.h"
#include "hal_timers.h"
#include "dev_rgb.h"


#ifdef CONSOLE_ENABLED
#include "dev_console.h"
#else
#include "devComms.h"
#endif

#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("Main") /* This must be undefined at the end of the file*/

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
void _sys_handler_time(void);
void _sys_handler_reset(void);
//bool menuVersion(void);
#endif /* CONSOLE_ENABLED */
#endif /* MAIN_DEBUG */

/*******************************************************************************
 local variables
 *******************************************************************************/
//The help menu structures
#ifdef MAIN_DEBUG
#ifdef CONSOLE_ENABLED
static console_menu_item_t _main_menu_items[] =
{
	{"reset",   _sys_handler_reset, "Perform a system reset"},
    {"time",    _sys_handler_time,  "Returns the system time" }
};

#endif /* CONSOLE_ENABLED */
#endif /* MAIN_DEBUG */
//int SystemState;		/* Movement State Machine variable */

const char BuildTimeData[] = { __TIME__ " " __DATE__ }; /* Used in our startup Banner*/

Timer_ms_t debugTmr;


/*******************************************************************************
 Functions
 *******************************************************************************/

void setup() {
    // put your setup code here, to run once:
	//Initialize serial communication on the standard port with the USB-2-RS232 converter
	console_init(115200, SERIAL_8N1);
    console_add_menu("main", _main_menu_items, ARRAY_SIZE(_main_menu_items), "Main Menu");

    dev_rgb_start(btnChasePin_RED, 15 /*btnChasePin_GREEN*/, 16 /*btnChasePin_BLUE*/, 17 /*-1*/);
    dev_rgb_set_1col(rgbRed, 128);


    sys_poll_tmr_start(&debugTmr, 10000L, true);	//Does something every 10s

}

void loop() {
	// put your main code here, to run repeatedly:

	//Check for anything coming in on the Serial Port and process it
    console_service();

    //static uint8_t dc = 0;
	//devComms::Read();

	if (sys_poll_tmr_expired(&debugTmr))
	{
		//iprintln(trMAIN, "Running for %lu s", SecondCount());
        // if (dc > 100)
        //     dc = 0;
        // dev_rgb_set_duty_cycle(13, dc++);
        //dev_rgb_inc_duty_cycle(13, true);
	}
}

#ifdef MAIN_DEBUG
#ifdef CONSOLE_ENABLED
void _sys_handler_time(void)
{
	iprintln(trALWAYS, "Running for %lu s", ((long)millis() / 1000));
}

void _sys_handler_reset(void)
{
    static bool reset_lock = false;
	// if no parameter passed then just open the gate

    char *argStr = console_arg_pop();

	if (!reset_lock)
    {
        if (!argStr)
        {
            reset_lock = true;
            iprintln(trALWAYS, "Now type 'reset Y', IF YOU ARE SURE");
            return;
        }
        iprintln(trALWAYS, "No arguments expected (got \"%s\")", argStr);
    }
    else //if (reset_lock)
    {
		if (0 == strcasecmp(argStr, "Y"))
        {
            // ok, do the reset.
            iprintln(trALWAYS, "Resetting. Goodbye, cruel world!");
            Serial.flush();
            resetFunc();
            return;
        }        
        iprintln(trALWAYS, "'reset Y' expected. Starting over");
    }
    reset_lock = false;
}

#endif /* CONSOLE_ENABLED */
#endif /* MAIN_DEBUG */

#undef PRINTF_TAG
