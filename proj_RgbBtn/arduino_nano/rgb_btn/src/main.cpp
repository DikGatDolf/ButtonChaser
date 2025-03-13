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
#include "dev_comms.h"


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
Structures and unions
 *******************************************************************************/
typedef struct
{
    rgb_colour_t colour[2];
    uint32_t blink_ms;
    Stopwatch_ms_t btn_sw;
    unsigned long sw_time;
}rgb_btn_t;

Timer_ms_t blink_tmr;

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

void _handle_master_msg_rx(void);

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
rgb_btn_t rgb_btn;
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

    dev_comms_init();

    sys_poll_tmr_start(&blink_tmr, 10000L, true);	//Does something every 10s

}

void loop() {
	// put your main code here, to run repeatedly:

	//Check for anything coming in on the Serial Port and process it
    console_service();

	//Check for anything coming in on the I2C and process it
    dev_comms_service();
    _handle_master_msg_rx();

	if (sys_poll_tmr_expired(&blink_tmr))
	{
        //Swop the colours on the RGB LED and set the new colour
        uint32_t tmp_wrgb = rgb_btn.colour[0].wrgb;
        dev_rgb_set_wrgb(tmp_wrgb);
        rgb_btn.colour[0].wrgb = rgb_btn.colour[1].wrgb;
        rgb_btn.colour[1].wrgb = tmp_wrgb;
	}
}

#ifdef MAIN_DEBUG
#ifdef CONSOLE_ENABLED
void _sys_handler_time(void)
{
	iprintln(trALWAYS, "Running for %lu s", ((long)millis() / 1000));
}

void _handle_master_msg_rx(void)
{
    uint8_t cmd;
    uint8_t len;


    //Have we received anything?
    len = dev_comms_rx_available();
    if (len == 0)
        return;

    iprintln(trCOMMS, "#Received %d bytes from master", len);

    if (dev_comms_rx_error(NULL))
    {
        iprintln(trCOMMS, "#Error: %s", dev_comms_rx_error_msg());
        dev_comms_rx_reset();
        return;
    }

    //Have we received anything?
    while (dev_comms_rx_available())
    {
        len = 0;
        //Message format is [Cmd][Payload][CRC]
        //The first byte is the command/header
        dev_comms_rx_read((uint8_t *)&cmd, 1);
        switch (cmd)
        {
            case cmd_set_rgb:
                //Should have at least 1 colour (3 bytes)
                len = 4;
                rgb_btn.colour[0].wrgb = 0;
                dev_comms_rx_read((uint8_t *)&rgb_btn.colour[0].wrgb, 4);
                break;

            case cmd_set_blink:
                //Should have at least 2 colours and a timer (3 bytes, 3 bytes, 4 bytes)
                len = 12;
                //Start with the primary colour, so we can swop them right now
                rgb_btn.colour[1].wrgb = 0;
                dev_comms_rx_read((uint8_t *)&rgb_btn.colour[1].wrgb, 4);
                
                rgb_btn.colour[0].wrgb = 0;
                dev_comms_rx_read((uint8_t *)&rgb_btn.colour[0].wrgb, 4);
                
                rgb_btn.blink_ms = 0;
                dev_comms_rx_read((uint8_t *)&rgb_btn.blink_ms, 4);

                //Set the colour and start the timer
                dev_rgb_set_wrgb(rgb_btn.colour[0].wrgb);
                sys_poll_tmr_start(&blink_tmr, (unsigned long)rgb_btn.blink_ms, true);

                break;

            case cmd_sw_start:
                len = 0;
                //Nothing else.... just start the stopwatch
                sys_stopwatch_ms_start(&rgb_btn.btn_sw);
                break;

            case cmd_wr_console: 
                //The next byte "should" be the length of the payload
                dev_comms_rx_read((uint8_t *)&len, 1);
                for (int i = 0; i < len; i++)
                {
                    uint8_t rxData;
                    dev_comms_rx_read((uint8_t *)&rxData, 1);
                    console_add_byte_to_rd_buff((uint8_t)rxData);
                }
                len++; //Add the length byte
                break;

            default:
                break;
        }
        iprintln(trCOMMS, "Handled 1 cmd byte 0x%02X) with %d bytes payload", cmd, len);
    }
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
