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
#include "dev_nvstore.h"
#include "dev_button.h"

#ifdef CONSOLE_ENABLED
#include "dev_console.h"
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
    stopwatch_ms_t stopwatch;
    unsigned long press_time;
    unsigned long press_cnt;
    int last_state;
}rgb_btn_t;

timer_ms_t blink_tmr;

/*******************************************************************************
 Function prototypes
 *******************************************************************************/

void _handle_master_msg_rx(void);

 //Menu Commands (without any owners)
#ifdef MAIN_DEBUG
#ifdef CONSOLE_ENABLED
void(*resetFunc)(void) = 0; //declare reset function at address 0
#if REDUCE_CODESIZE==0
    void _sys_handler_time(void);
#endif
void _sys_handler_reset(void);
void _sys_handler_led(void);
//bool menuVersion(void);
/*! Displays the application version
 */
void _sys_handler_version(void);

/*! Displays RAM information or dumps the RAM contents
 */
void _sys_handler_dump_ram(void);

/*! Displays Flash information or dumps the RAM contents
 */
void _sys_handler_dump_flash(void);

/*! Generig handler for the Dump RAM and Dump FLASH Console menus
 */
void _sys_handler_dump_generic(char * memType, unsigned long memStart, unsigned long memEnd, unsigned long * memDumpAddr, unsigned int * memDumpLen);
#endif /* CONSOLE_ENABLED */
#endif /* MAIN_DEBUG */

void _button_change_irq(void);

/*******************************************************************************
 local variables
 *******************************************************************************/
//The help menu structures
#ifdef MAIN_DEBUG
#ifdef CONSOLE_ENABLED
static console_menu_item_t _main_menu_items[] =
{
	{"reset",   _sys_handler_reset, "Perform a reset"},
    {"rgb",     _sys_handler_led,   "Get/Set the RGB LED colour" },
#if REDUCE_CODESIZE==0
    {"time",     _sys_handler_time,   "Displays the uptime of the program" }
#endif /* REDUCE_CODESIZE */
    {"ram",     _sys_handler_dump_ram,  "Display RAM"},
    {"flash",   _sys_handler_dump_flash,"Display FLASH"},
    {"version", _sys_handler_version,   "Displays app version"},
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
	//console_init(115200);

    rgb_btn.press_cnt = 0lu;
    rgb_btn.press_time = 0lu;
    rgb_btn.last_state = LOW;

    dev_comms_init(RGB_BTN_ADDR_MASTER+1);

    _sys_handler_version();
    console_add_menu("main", _main_menu_items, ARRAY_SIZE(_main_menu_items), "Main Menu");

    dev_nvstore_init();

    dev_rgb_start(btnChasePin_RED, btnChasePin_GREEN, btnChasePin_BLUE);
    //dev_rgb_set_colour(0x00800000);

    dev_button_init(_button_change_irq);

    //sys_poll_tmr_start(&blink_tmr, 10000L, true);	//Does something every 10s
    sys_poll_tmr_stop(&blink_tmr);

}

void loop() {
    
    uint8_t my_addr;

    // put your main code here, to run repeatedly:

    int btn_state = quickPinRead(btnChasePin_BTN);
    if (rgb_btn.last_state != btn_state)
    {
        if (HIGH == btn_state)
            rgb_btn.colour[0].rgb = (uint32_t)colBlue;
        else
            rgb_btn.colour[0].rgb = (uint32_t)colOrange;
        
        dev_rgb_set_colour(rgb_btn.colour[0].rgb);
        rgb_btn.last_state = btn_state;
        iprintln(trCOMMS, "Button is %s (%lu)", (HIGH == btn_state)? "HIGH" : "LOW", rgb_btn.press_cnt);
    }


    //Check if our I2C addres has changed
    if (dev_nvstore_new_data_available())
        dev_nvstore_read(&my_addr, sizeof(uint8_t));

    // //Check for anything coming in on the Serial Port and process it
    //console_service();

	//Check for anything coming in on the I2C and process it
    dev_comms_service();

    _handle_master_msg_rx();
   
    //Do we require blinking on our LED?
	if (sys_poll_tmr_expired(&blink_tmr))
	{
        //Swop the colours on the RGB LED and set the new colour
        uint32_t tmp_rgb = rgb_btn.colour[0].rgb;
        dev_rgb_set_colour(tmp_rgb);
        rgb_btn.colour[0].rgb = rgb_btn.colour[1].rgb;
        rgb_btn.colour[1].rgb = tmp_rgb;
	}
}

void _button_change_irq(void)
{
    rgb_btn.press_cnt++;
    // int btn_state = quickPinRead(btnChasePin_BTN);
    // if (HIGH == btn_state)
    // {
    //     rgb_btn.colour[0].rgb = (uint32_t)colBlue;
    // }
    // else
    // {
    //     rgb_btn.colour[0].rgb = (uint32_t)colOrange;
    // }
    // dev_rgb_set_colour(rgb_btn.colour[0].rgb);

}

void _handle_master_msg_rx(void)
{
//    uint8_t dst_addr;
    uint8_t cmd;
    uint8_t len = 0;
    bool responding = false;

    //Let's assume we will be responding
    dev_comms_response_start();

    //Have we received anything?
    while (dev_comms_cmd_available(&cmd))
    {
        //len = 0;
        switch (cmd & ~RGB_BTN_FLAG_CMD_COMPLETE)
        {
            case cmd_set_rgb_0:
                sys_poll_tmr_stop(&blink_tmr);
                len = dev_comms_cmd_read((uint8_t *)&rgb_btn.colour[0].rgb);
                dev_rgb_set_colour(rgb_btn.colour[0].rgb);
                break;

            case cmd_set_rgb_1:
                len = dev_comms_cmd_read((uint8_t *)&rgb_btn.colour[1].rgb);
                break;

            case cmd_set_blink:
                sys_poll_tmr_stop(&blink_tmr);
                len = dev_comms_cmd_read((uint8_t *)&rgb_btn.blink_ms);
                sys_poll_tmr_start(&blink_tmr, (unsigned long)rgb_btn.blink_ms, true);
                break;

            case cmd_get_btn:
                len = dev_comms_cmd_read(NULL);   // No data to read
                responding = true;
                dev_comms_response_add_data(cmd_get_btn, (uint8_t*)&rgb_btn.press_time, sizeof(unsigned long));
                break;
            
            case cmd_sw_start:
                len = dev_comms_cmd_read(NULL);   // No data to read
                //Nothing else.... just start the stopwatch
                sys_stopwatch_ms_start(&rgb_btn.stopwatch);
                break;

            // case cmd_get_sw_lap:
            //     len = dev_comms_cmd_read(NULL);   // No data to read
            //     responding = true;
            //     unsigned long _time = sys_stopwatch_ms_lap(&rgb_btn.stopwatch);
            //     dev_comms_response_add_data(cmd_get_btn, (uint8_t*)&_time, sizeof(unsigned long));
            //     break;

            case cmd_wr_console:
                uint8_t _data[RGB_BTN_MSG_MAX_LEN - sizeof(rgb_btn_msg_hdr_t) - sizeof(uint8_t)];
                memset(_data, 0, sizeof(_data));
                len = dev_comms_cmd_read(_data);
                if (cmd & RGB_BTN_FLAG_CMD_COMPLETE)
                {
                    //RVN - TODO This is the last section of this console message , this means we need to redirect the console output
                }
                for (int8_t i = 0; (i < ((int8_t)sizeof(_data))) & (_data[i] > 0); i++)
                    console_read_byte(_data[i]);

                //RVN - TODO You still need to figure out how you are going to do this, buddy!
                // Somehow need to trigger the console read with console_read_byte('\n') which will trigger _parse_rx_line()
                break;

            default:
                break;
        }

        iprintln(trCOMMS, "Handled Cmd (0x%02X) with %d bytes payload", cmd, len);
    }
    if (responding)
        dev_comms_response_send();
}

#ifdef MAIN_DEBUG
#ifdef CONSOLE_ENABLED
#if REDUCE_CODESIZE==0
void _sys_handler_time(void)
{
	iprintln(trALWAYS, "Running for %lu s", ((long)millis() / 1000));
}
#endif

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
            console_flush();
            resetFunc();
            return;
        }        
        iprintln(trALWAYS, "'reset Y' expected. Starting over");
    }
    reset_lock = false;
}

const static char * _led_col_str[] = {"Blue", "Green", "Red"/*, "White" */};

void _sys_handler_led(void)
{
    char *argStr;// = console_arg_pop();
    bool help_requested = console_arg_help_found();
    uint32_t rgb = 0;
    uint8_t col_mask = 0x00;                    //Indicates "not set"
    // int col_pwm[rgbMAX] = {-1, -1, -1};   //Indicates "not set"


    if (console_arg_cnt() == 0) // Show the current state of the PWM
        col_mask |= 0x07; //Set all colours

    //Go through every available argument
    while ((console_arg_cnt() > 0) && (!help_requested))
    {
        argStr = console_arg_pop();
        /*if ((0 == strcasecmp(argStr, "help")) || (0 == strcasecmp(argStr, "?"))) //is it a ? or help
        {
            help_reqested = true; //Disregard the rest of the arguments
            break; //from while
        }
        else */
        if (hex2u32(&rgb, argStr, 6)) //is it a hex value?
        {
            sys_poll_tmr_stop(&blink_tmr);
            dev_rgb_set_colour(rgb);
            col_mask |= 0x07;
            continue; //to the next argument
        }
        else //could be colour?
        {
            led_colour_type col = rgbMAX;
        
            //Get the colour match first
            for (int i = 0; i < rgbMAX; i++)
            {
                //We only match as much as the user has typed.... not much to be confused beween Red, Green, and Blue
                if (strncasecmp(_led_col_str[i], argStr, strlen(argStr)) == 0)
                {
                    col = (led_colour_type)i;
                    break; //from the for loop
                }
            }
    
            if (col == rgbMAX)
            {
                // For "All" we can apply the given duty cycle to all assigned colours/pins
                if (strcasecmp("All", argStr) != 0)
                {
                    iprint(trALWAYS, "Please specify a valid LED colour");
                    iprint(trALWAYS, ": ");
                    for (int i = 0; i < rgbMAX; i++)
                        iprint(trALWAYS, "\"%s\", ", _led_col_str[i]);
                    iprintln(trALWAYS, " or \"All\" (got \"%s\")", argStr);
                    help_requested = true; //Disregard the rest of the arguments
                    break; //from while
                }
                else
                {
                    col_mask |= 0x07; //Set all colours
                    continue; //to the next argument
                }
            }
            col_mask |= BIT_POS(col);
        }
    }

    if (!help_requested)
    {

        uint32_t rgb = 0, pwm = 0;
        rgb = dev_rgb_get_colour();
        pwm = dev_rgb_get_pwm();
        for (int i = 0; i < rgbMAX; i++)
        {
            if (col_mask & BIT_POS(i))
            {
                uint8_t led_val = (uint8_t)((rgb >> (i * 8)) & 0xFF);
                uint8_t pwm_val = (uint8_t)((pwm >> (i * 8)) & 0xFF);
                iprintln(trALWAYS, "  %5s: %3d (pwm: %3d)", _led_col_str[i], led_val, pwm_val);
            }
        }
    }
    
    if (help_requested)
    {
        iprintln(trALWAYS, " Usage: \"rgb [<hex_colour>] [<name_1> <name_2> .. <name_n>]\"");
#if REDUCE_CODESIZE==0        
        iprintln(trALWAYS, "    <colour> - A 24-bit hex value denoting the RGB colour value (0xRRGGBB) to set");
        iprintln(trALWAYS, "    <name>   - Displays the status of Red, Green, Blue, or All");
        iprintln(trALWAYS, " Multiple parameters can be passed in a single operation (sepated by spaces)");
        //                //          1         2         3         4         5         6         7         8         9
        //                //0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
#endif /* REDUCE_CODESIZE */
    }
}

void _sys_handler_version(void)
{
	iprintln(trALWAYS, "");
	iprintln(trALWAYS, "=====================================================");
	iprintln(trALWAYS, "ButtonChaser - Button Controller");
	iprintln(trALWAYS, "[c] 2025 ZeroBadCafe Development (Pty) Ltd");
	iprintln(trALWAYS, "Version   %d.%02d.", PROJECT_VERSION / 0x10, PROJECT_VERSION % 0x10);
	iprintln(trALWAYS, "BuildInfo %s.", BUILD_TIME_AND_DATE);
	iprintln(trALWAYS, "ESP32-C3 (Clock %lu MHz)", F_CPU / 1000000L);
	iprintln(trALWAYS, "=====================================================");
}

void _sys_handler_dump_ram(void)
{
    static unsigned long RAM_DumpAddr = RAMSTART;
    static unsigned int RAM_DumpLen = 256;

    _sys_handler_dump_generic((char *)"RAM", RAMSTART, RAMEND, &RAM_DumpAddr, &RAM_DumpLen);
}

void _sys_handler_dump_flash(void)
{
    static unsigned long FLASH_DumpAddr = 0l;//0x10000L;
    static unsigned int FLASH_DumpLen = 256;

    _sys_handler_dump_generic((char *)"FLASH", 0, FLASHEND, &FLASH_DumpAddr, &FLASH_DumpLen);
}

void _sys_handler_dump_generic(char * memType, unsigned long memStart, unsigned long memEnd, unsigned long * memDumpAddr, unsigned int * memDumpLen)
{
    uint32_t tmp_addr = *memDumpAddr;
    uint32_t tmp_len = (unsigned long)*memDumpLen;
    bool is_ram_not_flash = (memStart == RAMSTART);
    bool help_requested = console_arg_help_found();

    if (console_arg_cnt() == 0)
    {
        //Show RAM status
        if (is_ram_not_flash)
        {
            extern int __heap_start, *__brkval;//, __malloc_heap_start;
            iprintln(trALWAYS, "  HEAP Start: 0x%06lX", (int)&__heap_start);
            iprintln(trALWAYS, "  HEAP End:   0x%06lX", (int)__brkval);
            iprintln(trALWAYS, "  Free RAM:   %d bytes\n", freeRam());
        }
        else
        {
            iprintln(trALWAYS, "  FLASH Start: 0x%06X\n", 0);
            iprintln(trALWAYS, "  FLASH End:   0x%06lX\n", (unsigned long)(FLASHEND));
        }
        iprintln(trALWAYS, "  Next read:   %d bytes from 0x%06lX\n", *memDumpLen, *memDumpAddr);
        return;
    }

    while ((console_arg_cnt() > 0) && (!help_requested))
	{
        char *arg = console_arg_pop();
        /*if ((0 == strcasecmp(arg, "help")) || (0 == strcasecmp(arg, "?"))) //is it a ?
        {
            help_reqested = true; //Disregard the rest of the arguments
            break;
        }
        else */
        if (hex2u32(&tmp_addr, arg, 0))
        {
            //sscanf(strupr(argStr), "%lX", &tmp_addr);
            //hex2u32(&tmp_addr, arg, 0);
            iprintln(trALWAYS, "Got Address: 0x%06X from \"%s\"", tmp_addr, arg);

            if ((tmp_addr < memStart) || (tmp_addr >= memEnd))
            {
                iprint(trALWAYS, "Invalid %s Start address (", memType);
                iprintln(trALWAYS, "0x%06X)", tmp_addr);
                iprintln(trALWAYS, "Please use address between 0x%06lX and 0x%06lX", memStart, memEnd);
                help_requested = true; //Disregard the rest of the arguments
                break;
            }    
            *memDumpAddr = tmp_addr;
        }
        else if (str2uint32(&tmp_len, arg, 0)) //Length
        {
            //str2uint32(&tmp_len, arg, 0); //sscanf(argStr, "%lu", &tmp_len);
            iprintln(trALWAYS, "Got Length: %lu from \"%s\"", tmp_len, arg);
            //TODO - RVN, you should perform better handling of invalid length values here
            *memDumpLen = (unsigned int)tmp_len;
        }
        else
        {
            help_requested = true;
            iprintln(trALWAYS, "Invalid argument \"%s\"", arg);
        }
    }

    if (!help_requested)
    {
        //Does the read run over the end of the FLASH?
        if ((tmp_len >= ((memEnd + 1l) - tmp_addr)))
        {
            tmp_len = memEnd - tmp_addr + 1;
            iprintln(trALWAYS, "%s Dump length limited to %ld bytes: 0x%06lX to 0x%06lX", memType, tmp_len, tmp_addr, memEnd);
        }

        // OK, I honestly did not think the compiler will allow this. Cool.
        ((is_ram_not_flash)? console_print_ram : console_print_flash)(trALWAYS, (void *)*memDumpAddr, (uint32_t)*memDumpAddr, tmp_len);

        *memDumpAddr += *memDumpLen;
        //If we reach the end of RAM, reset the address
        if (*memDumpAddr > memEnd)
            *memDumpAddr = memStart;
    }
    
    if (help_requested)
    {
        iprintln(trALWAYS, "Usage: \"%s [<addr>] [<len>]\"", memType);
#if REDUCE_CODESIZE==0        
        iprintln(trALWAYS, " <No Args> - Shows %s summary\n", memType);
        iprintln(trALWAYS, " \"dump <ADDR> <LEN>\" - Dumps N bytes of %s starting at <ADDR>\n", memType);
        iprintln(trALWAYS, " \"dump\" - Dumps previously set N bytes (default: 256) of %s starting\n", memType);
        iprintln(trALWAYS, "           at end of last call (default: %s Start)\n", memType);
#endif /* REDUCE_CODESPACE */
        return;
    }    
}

#endif /* CONSOLE_ENABLED */
#endif /* MAIN_DEBUG */

#undef PRINTF_TAG
