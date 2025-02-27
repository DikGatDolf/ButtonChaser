/*******************************************************************************

Module:     task_rgb_btn.c
Purpose:    This file contains the RGB LED driver task
Author:     Rudolph van Niekerk


 *******************************************************************************/

/*******************************************************************************
includes
 *******************************************************************************/
#include <ctype.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "sys_utils.h"
#include "sys_timers.h"
#include "task_console.h"
#include "drv_rgb_led_strip.h"

#define __NOT_EXTERN__
#include "task_rgb_btn.h"
#undef __NOT_EXTERN__

/*******************************************************************************
local defines and constants
 *******************************************************************************/
#define PRINTF_TAG ("RGB_BTN") /* This must be undefined at the end of the file*/

#define RGB_STACK_SIZE 4096

//#define RGB_LED_CNT 5

//#define EXAMPLE_CHASE_SPEED_MS      50

#define RGB_LED_UPDATE_INTERVAL     20     /* Task cycles at a 50Hz rate*/
#define RGB_LED_BLINK_PERIOD_MS_MIN 100     /* 10Hz/2 = 5Hz... is already fairly fast for a blinking period */
#define RGB_LED_RAINBOW_PER_MIN_MS  1800    /* Cycling through the spectrum in 1.8s */
#define RGB_LED_RAINBOW_PER_MAX_MS  (0xFFFF * RGB_LED_UPDATE_INTERVAL)    /* Cycling through the spectrum in 21m 50.7s */


typedef enum
{
    led_bit_dbg = 0,
    led_bit_btn_0 = 1,
    led_bit_max = 31,
}rgb_led_addr_t;

typedef enum
{
    led_state_off = 0,
    led_state_on = 1,
    led_state_blink = 2,
    led_state_rainbow = 3,
}rgb_led_state_t;

/*******************************************************************************
local structure
 *******************************************************************************/
typedef enum {
    led_action_off = 0,         /* N0 value required */
    led_action_colour = 1,      /* 24 bit RGB value required */
    led_action_blink = 2,       /* blink period in ms (0 to 16,777,215/4h39m37.215s) */
    led_action_rainbow = 3,     /* rotation period in ms (0 to 16,777,215/4h39m37.215s) */
    /* Keep this at the end*/
    led_action_max = 7,
}rgb_led_action_cmd;

/*! The action command msg is built to be sent per LED (since we don't have that many LEDs)
 * This way it can be shrunk down to 2x32 bits and minimize the size of the msg queue
 * So, messages meant for ALL LEDs of a type, will require a total_leds_cnt messages in the queue 
 */
typedef struct {
    union 
    {
        struct 
        {
            uint32_t cmd    : 3;    // 0 = off, 1 = on/colour, 2 = blink, 3 = rainbow (4, 5, 6, 7 spare)
            uint32_t type   : 1;    // 0 = debug, 1 = button
            uint32_t pixel  : 4;    // 0 to 15
            uint32_t colour  :24;   // 24 bit RGB colour (if used by the command)
        };
        uint32_t val_0;         
        /* data */
    };
    uint32_t val_1;         // based on action. Max requirement: 24 bits for colour value
    uint32_t val_2;         // based on action. Max requirement: 24 bits for colour value
} rgb_led_action_msg_t;     // 32 bits

typedef struct {
    uint32_t type;          // 0 = debug, 1 = button
    uint32_t pixel;         // 0 to 15
    uint32_t col_1;         // 24 bit RGB value
    uint32_t col_2;         // 24 bit RGB value (used for blink)
    rgb_led_state_t state;
    Timer_ms_t timer;
} rgb_led_t;     // 32 bits

// typedef struct
// {
// 	eLED_State state;
// 	int blink_rate;
// } RgbLed_t;

typedef struct
{
    int cnt;
    rgb_led_t * array;    
	TaskInfo_t task;
    QueueHandle_t msg_queue;

    Stopwatch_ms_t sw;  /* used for debugging */

} DeviceRgb_t;

//We need to create a message queue for the RGB LED task
//static QueueHandle_t _task_rgb_btn_queue;


/*******************************************************************************
local function prototypes
 *******************************************************************************/
void _task_rgb_btn_init(void);
void _task_rgb_btn_deinit(void);

void _task_rgb_btn_pixel_info(int _index);

void _task_rgb_btn_read_msg_queue(void);
void _task_rgb_btn_service(void);

bool _pixel_addr_to_index(rgb_led_strip_type _type, uint32_t _pixel, uint32_t * _index);
bool _index_to_pixel_addr(uint32_t _index, rgb_led_strip_type * _type, uint32_t * _pixel);

void _menu_handler_led_off(void);
void _menu_handler_led_col(void);
void _menu_handler_led_blink(void);
void _menu_handler_led_rainbow(void);
void _menu_handler_led_col_list(void);
void _menu_handler_led_status(void);

/*******************************************************************************
local variables
 *******************************************************************************/
ConsoleMenuItem_t _task_rgb_btn_menu_items[] =
{
									        // 01234567890123456789012345678901234567890123456789012345678901234567890123456789
        {"off",     _menu_handler_led_off,      "Turns off led strip/specific pixel"},
        {"colour",  _menu_handler_led_col,      "Sets a led strip/pixel to a specific colour"},
        {"blink",   _menu_handler_led_blink,    "Blinks a led strip/pixel at a specified rate"},
        {"rainbow", _menu_handler_led_rainbow,  "Apply rainbow effect on a led strip/pixels"},
        {"cols",    _menu_handler_led_col_list, "Displays a list colours with their RGB hex values"},
		{"status",  _menu_handler_led_status,   "Displays the status of the strip(s)/pixel(s)"},

		// {"btn",     _menu_handler_led,   "Displays application version"},
};

static DeviceRgb_t _task_rgb_btn = {
    .cnt = 0,
    .array = NULL,
	.task = {   .handle = NULL,
	            .stack_depth = RGB_STACK_SIZE,
	            .stack_unused = 0
    },
    .msg_queue = NULL,
};

/*******************************************************************************
Local (private) Functions
*******************************************************************************/

void _task_rgb_btn_init(void)
{
    rgb_led_action_msg_t first_msg;

    _task_rgb_btn.cnt = drv_rgb_led_strip_init();

    dbgPrint(trRGB|trALWAYS, "#LED strips init done (%d pixels in total)", _task_rgb_btn.cnt);

#ifdef CONSOLE_ENABLED
    task_console_add_menu("rgb_btn", _task_rgb_btn_menu_items, ARRAY_SIZE(_task_rgb_btn_menu_items), "RGB Button");
#endif

    //We need to create a message queue for the RGB LED task, let's make it 2x the total pixel count
    _task_rgb_btn.msg_queue = xQueueCreate(_task_rgb_btn.cnt*2, sizeof(rgb_led_action_msg_t));
    if(_task_rgb_btn.msg_queue == 0 )
    {
        // Queue was not created and must not be used.
        dbgPrint(trRGB|trALWAYS, "#Unable to create Msg Queue for RGB LED task");
        assert(0);
    }

    //We also need to create a timer for each RGB pixel
    _task_rgb_btn.array = (rgb_led_t *)malloc(_task_rgb_btn.cnt * sizeof(rgb_led_t));
    if (_task_rgb_btn.array == NULL)
    {
        // Queue was not created and must not be used.
        dbgPrint(trRGB|trALWAYS, "#Unable to create Timers for RGB LED task");
        assert(0);
    }
    else
    {
        //Build the content of the array to match the LED strips/pixels
        int array_index = 0;
        for (rgb_led_strip_type _type = 0; _type < ledtype_max; _type++)
        {
            if (array_index >= _task_rgb_btn.cnt)
            {
                dbgPrint(trRGB|trALWAYS, "#%s() - Array Indexing out of bounds (%d/%d)", __FUNCTION__, array_index, _task_rgb_btn.cnt);
                //This is a bit of a problem
                break;
            }

            int _type_pixel_cnt = drv_rgb_led_strip_pixels(_type);
            for (int _pixel = 0; _pixel < _type_pixel_cnt; _pixel++)
            {
                _task_rgb_btn.array[array_index].type = _type;
                _task_rgb_btn.array[array_index].pixel = _pixel;
                _task_rgb_btn.array[array_index].col_1 = colBlack;
                _task_rgb_btn.array[array_index].col_2 = colBlack;
                _task_rgb_btn.array[array_index].state = led_state_off;      //All Leds start in OFF state
                sys_poll_tmr_stop(&_task_rgb_btn.array[array_index].timer);  //Just in case
                array_index++;
            }
        }
        //memset(_task_rgb_btn.array, 0, _task_rgb_btn.cnt * sizeof(rgb_led_t));
    }

    //Debugging - print contents of Pixel Array
    // for (int i = 0; i < _task_rgb_btn.cnt; i++)
    //     _task_rgb_btn_pixel_info(i);
    
    //Send a message to start the rainbow effect on the Debug LED
    first_msg.cmd = led_action_rainbow;
    first_msg.type = ledtype_Debug;
    first_msg.pixel = 0;
    first_msg.colour = colBlack;   //Black
    first_msg.val_1 = 7200;  //ms
    first_msg.val_2 = 0;

    //Add this message to the queue
    xQueueSend(_task_rgb_btn.msg_queue, &first_msg, 0);
}

void _task_rgb_btn_pixel_info(int _index)
{
    rgb_led_t * rgb_led_pixel = &_task_rgb_btn.array[_index];
    dbgPrint(trRGB, "#RGB Pixel #%d Info: Type: %d, Pixel: %d, Colour: %06X, State: %d", _index, rgb_led_pixel->type, rgb_led_pixel->pixel, rgb_led_pixel->col_1, rgb_led_pixel->state);
}

void _task_rgb_btn_deinit(void)
{
    //Build the content of the array to match the LED strips/pixels
    for (int _pixel = 0; _pixel < _task_rgb_btn.cnt; _pixel++)
    {
        sys_poll_tmr_stop(&_task_rgb_btn.array[_pixel].timer);
    }
    //memset(_task_rgb_btn.array, 0, _task_rgb_btn.cnt * sizeof(rgb_led_t));

    //Free the allocated memory
    if (_task_rgb_btn.array != NULL)
    {
        free(_task_rgb_btn.array);
        _task_rgb_btn.array = NULL;
    }

    drv_rgb_led_strip_deinit();
}

void _task_rgb_btn_task(void * pvParameters)
{
    // uint16_t hue = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    _task_rgb_btn_init();

    dbgPrint(trRGB|trALWAYS, "#RGB LED Task Started. Period %dms (%d LEDs)", RGB_LED_UPDATE_INTERVAL, _task_rgb_btn.cnt);

    while (1) 
    {

        _task_rgb_btn_read_msg_queue();
        
        _task_rgb_btn_service();

        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(RGB_LED_UPDATE_INTERVAL));  //vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
    
        /* Inspect our own high water mark on entering the task. */
    	_task_rgb_btn.task.stack_unused = uxTaskGetStackHighWaterMark2( NULL );
    }
    
    _task_rgb_btn_deinit();

}

void _task_rgb_btn_read_msg_queue(void)
{     
    rgb_led_action_msg_t rx_msg;
    // rgb_led_strip_type _type;
    // uint32_t _pixel;
    uint32_t _index;

    //Check if there is a message in the queue
    while (xQueueReceive(_task_rgb_btn.msg_queue, &rx_msg, 0) == pdTRUE)
    {
        //We have a message, let's process it
        if (_pixel_addr_to_index(rx_msg.type, rx_msg.pixel, &_index))
        {
            switch (rx_msg.cmd)
            {
                case led_action_off:
                    dbgPrint(trRGB, "#Pixel %d turned OFF", _index);
                    //Val_1 and Val_2 are not used
                    _task_rgb_btn.array[_index].col_1 = 0;
                    drv_rgb_led_strip_set_rgb_pixel(rx_msg.type, rx_msg.pixel, 0);
                    _task_rgb_btn.array[_index].state = led_state_off;
                    sys_poll_tmr_stop(&_task_rgb_btn.array[_index].timer);
                    break;
                case led_action_colour:
                    dbgPrint(trRGB, "#Pixel %d turned ON (colour: 0x%06X)", _index, rx_msg.colour);
                    //Val_1 and Val_2 are not used
                    _task_rgb_btn.array[_index].col_1 = rx_msg.colour;
                    drv_rgb_led_strip_set_rgb_pixel(rx_msg.type, rx_msg.pixel, rx_msg.colour);
                    _task_rgb_btn.array[_index].state = (rx_msg.colour == 0)? led_state_off : led_state_on; //If the colour is black, then the state is off, otherwise it is on
                    sys_poll_tmr_stop(&_task_rgb_btn.array[_index].timer);
                    break;
                case led_action_blink:            
                    dbgPrint(trRGB, "#Pixel %d Set to Blink (0x%06X <-> 0x%06X) at %dms", _index, rx_msg.colour, (rx_msg.val_2 & 0x00FFFFFF), rx_msg.val_1);
                    //Val_1 is the period (in ms)
                    //Val_2 is the 2nd/alternating  colour
                    //First make sure the blink timer period is good
                    // Since the task is running at 20ms intervals, we can only set the timer to multiples of 20ms
                    if ((rx_msg.val_1%RGB_LED_UPDATE_INTERVAL) > 0)
                        rx_msg.val_1 = (RGB_LED_UPDATE_INTERVAL * ((rx_msg.val_1/RGB_LED_UPDATE_INTERVAL)+1));
                        // rx_msg.val_1 += (RGB_LED_UPDATE_INTERVAL - (rx_msg.val_1%RGB_LED_UPDATE_INTERVAL));

                    //Blinking faster than 10Hz is not really useful
                    if (rx_msg.val_1 < RGB_LED_BLINK_PERIOD_MS_MIN)
                        rx_msg.val_1 = RGB_LED_BLINK_PERIOD_MS_MIN;

                        //Start the timer and assign the primary colour
                    drv_rgb_led_strip_set_rgb_pixel(rx_msg.type, rx_msg.pixel, rx_msg.colour);
                    //Swop the primary and secondary colours for the next iteration
                    _task_rgb_btn.array[_index].col_1 = (rx_msg.val_2 & 0x00FFFFFF);
                    _task_rgb_btn.array[_index].col_2 = (rx_msg.colour & 0x00FFFFFF);
                    _task_rgb_btn.array[_index].state = led_state_blink;
                    sys_poll_tmr_start(&_task_rgb_btn.array[_index].timer, (uint64_t)rx_msg.val_1/2, true);
                    break;
                case led_action_rainbow:
                    dbgPrint(trRGB, "#Pixel %d Set to Rainbow at %dms", _index, rx_msg.val_1);
                    //Val_1 is the period (in ms)
                    //Val_2 is not used
                    //First make sure the timer period is good
                    // Since the task is running at 20ms intervals, we can only set the timer to multiples of 20ms
                    if ((rx_msg.val_1%RGB_LED_UPDATE_INTERVAL) > 0)
                        rx_msg.val_1 = (RGB_LED_UPDATE_INTERVAL * ((rx_msg.val_1/RGB_LED_UPDATE_INTERVAL)+1));
                        //rx_msg.val_1 += (RGB_LED_UPDATE_INTERVAL - (rx_msg.val_1%RGB_LED_UPDATE_INTERVAL));

                    //Blinking faster than 10Hz is not really useful
                    if (rx_msg.val_1 < RGB_LED_RAINBOW_PER_MIN_MS)
                        rx_msg.val_1 = RGB_LED_RAINBOW_PER_MIN_MS;
                    if (rx_msg.val_1 > RGB_LED_RAINBOW_PER_MAX_MS)
                        rx_msg.val_1 = RGB_LED_RAINBOW_PER_MAX_MS;
                    
                    drv_rgb_led_strip_set_rgb_pixel(rx_msg.type, rx_msg.pixel, 0);
                    //The minimum timer value is 20ms (RGB_LED_UPDATE_INTERVAL).
                    //Dividing the period by RGB_LED_UPDATE_INTERVAL gives us a count of how many task cycles it takes to loop through the 360 degree hue spectrum,
                    //No need to start a timer... we use the task cycle interval to update the hue (if needed)
                    _task_rgb_btn.array[_index].col_1 = 0;
                    _task_rgb_btn.array[_index].col_2 = (rx_msg.val_1/RGB_LED_UPDATE_INTERVAL) & 0xFFFF;
                    dbgPrint(trRGB, "#Pixel %d Rainbow effect at %dms (%d x %d = %d)", 
                        _index, rx_msg.val_1, _task_rgb_btn.array[_index].col_2, RGB_LED_UPDATE_INTERVAL, _task_rgb_btn.array[_index].col_2 * RGB_LED_UPDATE_INTERVAL);
                    _task_rgb_btn.array[_index].state = led_state_rainbow;
                    // sys_stopwatch_ms_start(&_task_rgb_btn.sw);
                    sys_poll_tmr_stop(&_task_rgb_btn.array[_index].timer);
                    break;
                default:
                    break;
            }
        }
        else // Not a valid message
        {
            //Message details corrupted
            dbgPrint(trRGB|trALWAYS, "#Invalid message details (type: %d, pixel: %d)", rx_msg.type, rx_msg.pixel);
        }
    }
}

void _task_rgb_btn_service(void)
{     
    static uint32_t hue = 0;
    for (int _pixel = 0; _pixel < _task_rgb_btn.cnt; _pixel++)
    {
        rgb_led_t * led_pixel = &_task_rgb_btn.array[_pixel];
        switch (led_pixel->state)
        {
            case led_state_blink:
                if (sys_poll_tmr_expired(&led_pixel->timer))
                {
                    uint32_t _temp_col = led_pixel->col_1;
                    //Set the current colour                    
                    drv_rgb_led_strip_set_rgb_pixel(led_pixel->type, led_pixel->pixel, led_pixel->col_1);
                    //Swop the primary and secondary colours for the next iteration
                    led_pixel->col_1 = led_pixel->col_2;
                    led_pixel->col_2 = _temp_col;
                    //Timer should be running on auto-reload, so no need to reset it
                }
                hue = 0;
                break;
            case led_state_rainbow:
                //This is assumed to run at the task interval period of 20ms (not using a timer)
                uint32_t _total = (led_pixel->col_2 & 0x0000FFFF);
                uint32_t _count = (led_pixel->col_2 >> 16) & 0x0000FFFF;
                hue = (360 *_count/_total)%360;
                hsv2rgb(hue, 100, 100, &led_pixel->col_1);
                drv_rgb_led_strip_set_rgb_pixel(ledtype_Debug, 0, led_pixel->col_1);
                //dbgPrint(trRGB, "#Rainbow: %d/%d - H: %d -> 0x%06x)", _count, _total, hue, led_pixel->col_1);
                _count++;
                if (_count >= _total)
                {
                    // uint32_t _lap = sys_stopwatch_ms_reset(&_task_rgb_btn.sw);
                    // dbgPrint(trRGB, "#Rainbow (re)starting after %dms", _lap);
                    _count = 0;
                }
                led_pixel->col_2 = ((((uint32_t)_count) << 16) & 0xFFFF0000) | ((uint32_t)_total & 0x0000FFFF);
                break;
            // case led_state_off:
            // case led_state_on:
            default:
                //Do nothing
                hue = 0;
                break;
        }
    }
}

bool _pixel_addr_to_index(rgb_led_strip_type _type, uint32_t _pixel, uint32_t * _index)
{
    for (int i = 0; i < _task_rgb_btn.cnt; i++)
    {
        if ((_task_rgb_btn.array[i].type == _type) && (_task_rgb_btn.array[i].pixel == _pixel))
        {
            *_index = i;
            return true;
        }
    }
    return false;
}

bool _index_to_pixel_addr(uint32_t _index, rgb_led_strip_type * _type, uint32_t * _pixel)
{
    if (_index < _task_rgb_btn.cnt)
    {
        *_type = _task_rgb_btn.array[_index].type;
        *_pixel = _task_rgb_btn.array[_index].pixel;
        return true;
    }
    return false;
}

void _menu_handler_led_off(void)
{
    //These functions (_menu_handler....) are called from the console task, so they should 
    // not manupiluate the RGB task variables directly, but instead send messages to the RGB 
    // task via the msg_queue
    bool help_requested = false;
    rgb_led_action_msg_t first_msg;

    while (task_console_arg_cnt() > 0)
	{
        char *arg = task_console_arg_pop();
        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from while-loop
        }
    }

    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        dbgPrint(trALWAYS, "Usage: \"off <pixel_addr>\"");
        dbgPrint(trALWAYS, "  <pixel #> (optional): 0 to <pixel_cnt-1>");
        dbgPrint(trALWAYS, "                        \"[Debug|Dbg|Button|Btn]<:#>");
        dbgPrint(trALWAYS, " If <args> are omitted, the command applies to all pixels");
        dbgPrint(trALWAYS, "  ");
        dbgPrint(trALWAYS, "  if <type> or <pixel #> is omitted, applies to all led strips/pixels");
        // dbgPrint(trALWAYS, "  Multiple actions can be performed in a single line");
        // dbgPrint(trALWAYS, "   e.g. \"led COLOUR 0x00FF00 debug 0 BLINK 100 button 1...\" etc");

    }
}

void _menu_handler_led_col(void)
{

}

void _menu_handler_led_blink(void)
{

}

void _menu_handler_led_rainbow(void)
{

}

void _menu_handler_led_status(void)
{
    bool help_requested = false;

    while (task_console_arg_cnt() > 0)
	{
        char *arg = task_console_arg_pop();
        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from while-loop
        }
    }

    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        dbgPrint(trALWAYS, "Usage: \"status <type> <pixel #>\"");
        dbgPrint(trALWAYS, "  <type> (optional):    \"debug|button\". If omitted, lists all");
        dbgPrint(trALWAYS, "  <pixel #> (optional): 0 to <pixel_cnt-1> for type. If omitted, lists all");
        dbgPrint(trALWAYS, "  ");
        dbgPrint(trALWAYS, "  Multiple statusses can be requested in a single line");
        dbgPrint(trALWAYS, "   e.g. \"status debug 0 button 1...\" etc");
    }
}

void _menu_handler_led_col_list(void)
{
    bool help_requested = false;
    /* Possible combinations:
        led <action>
        led <action> <type>
        led <action> <pixel #>
        led <action> <value>
        led <action> <value> <type>
        led <action> <value> <pixel #>
        led <action> <type> <pixel #>
        led <action> <value> <type> <pixel #>
        led <action> <value> <action> <value> <type> <pixel #>
        led <action> <value> <type> <action> <value> <type> <pixel #>
    */

    while (task_console_arg_cnt() > 0)
	{
        char *arg = task_console_arg_pop();
        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from while-loop
        }
    }

    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        dbgPrint(trALWAYS, "Usage: \"led <action> <value> <type> <pixel #>\"");
        dbgPrint(trALWAYS, "  <action>   - <value> can be: ");
        dbgPrint(trALWAYS, "   colour    - colour|0xRRGGBBWW - Sets a strip/pixel to a colour or RGB hex value");
        dbgPrint(trALWAYS, "   blink     - period_ms         - Blinks strip/pixel at period (50%% duty cycle)");
        dbgPrint(trALWAYS, "   rainbow   - period_ms         - Rainbow rotation at period");
        dbgPrint(trALWAYS, "   off       - N/A               - Turn strip/pixel off");
        dbgPrint(trALWAYS, "  <type> (optional):    \"debug|button\". If omitted, applies to all");
        dbgPrint(trALWAYS, "  <pixel #> (optional): 0 to <pixel_cnt-1> for type. If omitted, applies to all");
        dbgPrint(trALWAYS, "  ");
        dbgPrint(trALWAYS, "  Multiple actions can be performed in a single line");
        dbgPrint(trALWAYS, "   e.g. \"led COLOUR 0x00FF00 debug 0 BLINK 100 button 1...\" etc");

    }
}

/*******************************************************************************
Global (public) Functions
*******************************************************************************/

TaskInfo_t * task_rgb_btn_init(void)
{
	//Let's not re-initialise this task by accident
	if (_task_rgb_btn.task.handle)
		return &_task_rgb_btn.task;

	_task_rgb_btn.cnt = 0;

	// Create the task, storing the handle.  Note that the passed parameter ucParameterToPass
	// must exist for the lifetime of the task, so in this case is declared static.  If it was just an
	// an automatic stack variable it might no longer exist, or at least have been corrupted, by the time
	// the new task attempts to access it.
	if (xTaskCreate( _task_rgb_btn_task, PRINTF_TAG, _task_rgb_btn.task.stack_depth, _task_rgb_btn.task.parameter_to_pass, 10, &_task_rgb_btn.task.handle ) != pdPASS)
	{
		dbgPrint(trALWAYS, "#Unable to start Task (%d)!", eTaskIndexRgb);
		return NULL;
	}

	configASSERT(_task_rgb_btn.task.handle);

	return &_task_rgb_btn.task;
}

void task_rgb_btn_deinit(void)
{
	// Use the handle to delete the task.
	if(_task_rgb_btn.task.handle != NULL )
		vTaskDelete(_task_rgb_btn.task.handle);

}

#undef PRINTF_TAG

/*************************** END OF FILE *************************************/
