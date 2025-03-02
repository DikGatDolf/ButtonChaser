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
#include "esp_timer.h"
#include "sys_utils.h"
#include "sys_timers.h"
#include "str_helper.h"
#include "task_console.h"
#include "drv_rgb_led_strip.h"

#define __NOT_EXTERN__
#include "task_rgb_btn.h"
#undef __NOT_EXTERN__

/*******************************************************************************
Macros and Constants
 *******************************************************************************/
#define PRINTF_TAG ("RGB_BTN") /* This must be undefined at the end of the file*/

#define RGB_STACK_SIZE 4096

#define RGB_LED_UPDATE_INTERVAL         20     /* Task cycles at a 50Hz rate*/
#define RGB_LED_BLINK_PERIOD_MS_MIN     100     /* 10Hz/2 = 5Hz... is already fairly fast for a blinking period */
#define RGB_LED_BLINK_PERIOD_MS_DEF     400    /* 1Hz */
#define RGB_LED_RAINBOW_PER_MIN_MS      1800    /* Cycling through the spectrum in 1.8s */
#define RGB_LED_RAINBOW_PER_MAX_MS      (0xFFFF * RGB_LED_UPDATE_INTERVAL)    /* Cycling through the spectrum in 21m 50.7s */
#define RGB_LED_RAINBOW_PERIOD_MS_DEF   3800

#define MAX_LED_NAME_LEN 16
/*******************************************************************************
local defines 
 *******************************************************************************/
typedef enum
{
    led_bit_dbg = 0,        /* Debug LED (only 1) */
    led_bit_btn_0 = 1,      /* Start of Button LEDs */
    /* Keep this at the end*/
    led_bit_max = 16,       /* Allows for up to 16 LEDs in total (dbg + 15) */
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
    led_action_status = 4,      /* No value required */
    /* Keep these at the end*/
    led_action_total = 5,       /* update this value as more actions are added */
    led_action_max = 0xffff,
}rgb_led_action_cmd;

/*! The action command msg is built to be sent per ACTION and not per LED
 * This means, a single action can be sent for any one or more LEDs
 * So, a messages meant for ALL LEDs, will require a single message in the queue 
 * This also ensures that time-based actions (Blinking and Rainbow effect) are 
 * synchronised.
 */

typedef struct {
    union 
    {
        struct 
        {
            uint16_t cmd;       // 0 = off, 1 = on/colour, 2 = blink, 3 = rainbow (4, to 0xFFFF spare)
            uint16_t led_addr;  // bit-map address for the affected LEDs
        };
        /* data */
        uint32_t cmd_and_addr;
    };
    uint32_t val_0;         // based on action. Max requirement: 24 bits for colour value
    uint32_t val_1;         // based on action. Max requirement: 24 bits for colour value
    uint32_t val_2;         // based on action. Max requirement: 24 bits for colour value
} rgb_led_action_msg_t;     // 32 bits

typedef struct {
//    uint32_t id;            // 0 to 15
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

    //Stopwatch_ms_t sw;  /* used for debugging */

} DeviceRgb_t;

//We need to create a message queue for the RGB LED task
//static QueueHandle_t _task_rgb_led_queue;


/*******************************************************************************
local function prototypes
 *******************************************************************************/
void _task_rgb_led_init(void);
void _task_rgb_led_deinit(void);

void _task_rgb_led_info(int _index);

void _task_rgb_led_read_msg_queue(void);
void _task_rgb_led_service(void);

void _task_rgb_led_set_colour(rgb_led_addr_t _index, uint32_t _rgb);
bool _led_addr_to_bit_index(rgb_led_strip_type _type, uint32_t _led_nr, rgb_led_addr_t * _index);
bool _index_to_led_type_addr(rgb_led_addr_t _index, rgb_led_strip_type * _type, uint32_t * _led_nr, char * led_name);
char * _index_to_led_name(rgb_led_addr_t _index);

void _menu_handler_led_off(void);
void _menu_handler_led_col(void);
void _menu_handler_led_blink(void);
void _menu_handler_led_rainbow(void);
void _menu_handler_led_col_list(void);
void _menu_handler_led_status(void);

/*******************************************************************************
local variables
 *******************************************************************************/
ConsoleMenuItem_t _task_rgb_led_menu_items[] =
{
									        // 01234567890123456789012345678901234567890123456789012345678901234567890123456789
        {"off",     _menu_handler_led_off,      "Turns off LEDs"},
        {"on",      _menu_handler_led_col,      "Sets LEDs to a specific colour"},
//        {"ledcol",  _menu_handler_led_col,      "Sets LEDs to a specific colour"},
        {"blink",   _menu_handler_led_blink,    "Blinks LEDs at a specified rate and colour(s)"},
        {"rainbow", _menu_handler_led_rainbow,  "Apply rainbow effect on LEDs"},
        {"listcol", _menu_handler_led_col_list, "Displays a list predefined colours with their RGB hex values"},
		{"status",  _menu_handler_led_status,   "Displays the status of the LEDs"},

		// {"btn",     _menu_handler_led,   "Displays application version"},
};

static DeviceRgb_t _task_rgb_led = {
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

void _task_rgb_led_init(void)
{
    rgb_led_action_msg_t first_msg;

    _task_rgb_led.cnt = drv_rgb_led_strip_init();

    //dbgPrint(trRGB|trALWAYS, "#RGB LED Task init done (%d LEDs in total)", _task_rgb_led.cnt);

#ifdef CONSOLE_ENABLED
    task_console_add_menu("led", _task_rgb_led_menu_items, ARRAY_SIZE(_task_rgb_led_menu_items), "RGB LED Control");
#endif

    //We need to create a message queue for the RGB LED task, let's make it double to the total led_action_total count
    _task_rgb_led.msg_queue = xQueueCreate(led_action_total*2, sizeof(rgb_led_action_msg_t));
    if(_task_rgb_led.msg_queue == 0 )
    {
        // Queue was not created and must not be used.
        dbgPrint(trRGB|trALWAYS, "#Unable to create Msg Queue for RGB LED task");
        assert(0);
    }

    //We also need to create a timer for each RGB LED (for blinking)
    _task_rgb_led.array = (rgb_led_t *)malloc(_task_rgb_led.cnt * sizeof(rgb_led_t));
    if (_task_rgb_led.array == NULL)
    {
        // Queue was not created and must not be used.
        dbgPrint(trRGB|trALWAYS, "#Unable to create Timers for RGB LED task");
        assert(0);
    }
    else
    {
        //Build the content of the array to match the LEDs
        int array_index = 0;
        for (rgb_led_strip_type _type = 0; _type < strips_Total; _type++)
        {
            if (array_index >= _task_rgb_led.cnt)
            {
                dbgPrint(trRGB|trALWAYS, "#%s() - Array Indexing out of bounds (%d/%d)", __FUNCTION__, array_index, _task_rgb_led.cnt);
                //This is a bit of a problem
                break;
            }

            int _type_led_cnt = drv_rgb_led_strip_count(_type);
            for (int _addr = 0; _addr < _type_led_cnt; _addr++)
            {
                _task_rgb_led.array[array_index].col_1 = colBlack;
                _task_rgb_led.array[array_index].col_2 = colBlack;
                _task_rgb_led.array[array_index].state = led_state_off;      //All Leds start in OFF state
                sys_poll_tmr_stop(&_task_rgb_led.array[array_index].timer);  //Just in case
                array_index++;
            }
        }
        //memset(_task_rgb_led.array, 0, _task_rgb_led.cnt * sizeof(rgb_led_t));
    }

    //Debugging - print contents of LED Array
    // for (int i = 0; i < _task_rgb_led.cnt; i++)
    //     _task_rgb_led_info(i);
    
    //Send a message to start the rainbow effect on the Debug LED only
    first_msg.cmd = led_action_rainbow;
    first_msg.led_addr = BIT(led_bit_dbg);
    first_msg.val_0 = colBlack;   //Black
    first_msg.val_1 = 7200;  //ms
    first_msg.val_2 = 0;

    //Add this message to the queue
    xQueueSend(_task_rgb_led.msg_queue, &first_msg, 0);
}

void _task_rgb_led_info(int _index)
{
    rgb_led_t * rgb_led = &_task_rgb_led.array[_index];
    rgb_led_strip_type _strip;
    uint32_t _addr;
    char _led_name[16]; 

    if ((_index >= _task_rgb_led.cnt) || (_index > led_bit_max))
    {
        dbgPrint(trRGB, "#RGB LED #%d Index out of bounds (%d/%d)", _index, _index, _task_rgb_led.cnt);
        return;
    }

    if (_index_to_led_type_addr(_index, &_strip, &_addr, _led_name))
        dbgPrint(trRGB, "#%s LED (%d) Info: Type %d, Address %d, Colour %06X, State %d", _led_name, _index, _strip, _addr, rgb_led->col_1, rgb_led->state);

}

void _task_rgb_led_deinit(void)
{
    for (int _led = 0; _led < _task_rgb_led.cnt; _led++)
        sys_poll_tmr_stop(&_task_rgb_led.array[_led].timer);

    //memset(_task_rgb_led.array, 0, _task_rgb_led.cnt * sizeof(rgb_led_t));

    //Free the allocated memory
    if (_task_rgb_led.array != NULL)
    {
        free(_task_rgb_led.array);
        _task_rgb_led.array = NULL;
    }

    drv_rgb_led_strip_deinit();
}

void _task_rgb_led_task(void * pvParameters)
{
    // uint16_t hue = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    _task_rgb_led_init();

    dbgPrint(trRGB|trALWAYS, "#RGB LED Task Started (%d LEDs) @ %d Hz", _task_rgb_led.cnt, (1000/RGB_LED_UPDATE_INTERVAL));

    while (1) 
    {

        _task_rgb_led_read_msg_queue();
        
        _task_rgb_led_service();

        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(RGB_LED_UPDATE_INTERVAL));  //vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
    
        /* Inspect our own high water mark on entering the task. */
    	_task_rgb_led.task.stack_unused = uxTaskGetStackHighWaterMark2( NULL );
    }
    
    _task_rgb_led_deinit();

}

void _task_rgb_led_read_msg_queue(void)
{     
    rgb_led_action_msg_t rx_msg;

    //Check if there is a message in the queue
    while (xQueueReceive(_task_rgb_led.msg_queue, &rx_msg, 0) == pdTRUE)
    {
        if (rx_msg.led_addr == 0)
        {
            dbgPrint(trRGB, "#Empty address list msg rx'd in queue: action", rx_msg.cmd);
            continue;  //No LEDs are affected by this message
        }

        //We have a message, let's process it for every LED that it affects
        for (uint32_t _led_nr = 0; _led_nr < _task_rgb_led.cnt; _led_nr++)
        {

            if ((rx_msg.led_addr & BIT(_led_nr)) == 0)
                continue;   //This LED is not affected by the message

            //This LED is affected by the message
            switch (rx_msg.cmd)
            {
                case led_action_off:
                    dbgPrint(trRGB, "#%s turned OFF", _index_to_led_name(_led_nr));
                    //Val_0, Val_1 and Val_2 are not used
                    _task_rgb_led.array[_led_nr].col_1 = 0;
                    _task_rgb_led_set_colour(_led_nr, 0);
                    _task_rgb_led.array[_led_nr].state = led_state_off;
                    sys_poll_tmr_stop(&_task_rgb_led.array[_led_nr].timer);
                    break;
                case led_action_colour:
                    dbgPrint(trRGB, "#%s turned ON (colour: 0x%06X)", _index_to_led_name(_led_nr), rx_msg.val_0);
                    //Val_0 is the colour
                    //Val_1 and Val_2 are not used
                    _task_rgb_led.array[_led_nr].col_1 = rx_msg.val_0;
                    _task_rgb_led_set_colour(_led_nr, rx_msg.val_0);
                    _task_rgb_led.array[_led_nr].state = (rx_msg.val_0 == 0)? led_state_off : led_state_on; //If the colour is black, then the state is off, otherwise it is on
                    sys_poll_tmr_stop(&_task_rgb_led.array[_led_nr].timer);
                    break;
                case led_action_blink:            
                    dbgPrint(trRGB, "#%s Set to Blink (0x%06X <-> 0x%06X) at %dms", _index_to_led_name(_led_nr), rx_msg.val_0, (rx_msg.val_2 & 0x00FFFFFF), rx_msg.val_1);
                    //Val_0 is the 1st (primary) colour
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
                    _task_rgb_led_set_colour(_led_nr, rx_msg.val_0);
                    //Swop the primary and secondary colours for the next iteration
                    _task_rgb_led.array[_led_nr].col_1 = (rx_msg.val_2 & 0x00FFFFFF);
                    _task_rgb_led.array[_led_nr].col_2 = (rx_msg.val_0 & 0x00FFFFFF);
                    _task_rgb_led.array[_led_nr].state = led_state_blink;
                    sys_poll_tmr_start(&_task_rgb_led.array[_led_nr].timer, (uint64_t)rx_msg.val_1/2, true);
                    break;
                case led_action_rainbow:
                    dbgPrint(trRGB, "#%s LED: Set to Rainbow at %dms", _index_to_led_name(_led_nr), rx_msg.val_1);
                    //Val_0 is not used
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
                    
                    _task_rgb_led_set_colour(_led_nr, 0);
                    //The minimum timer value is 20ms (RGB_LED_UPDATE_INTERVAL).
                    //Dividing the period by RGB_LED_UPDATE_INTERVAL gives us a count of how many task cycles it takes to loop through the 360 degree hue spectrum,
                    //No need to start a timer... we use the task cycle interval to update the hue (if needed)
                    _task_rgb_led.array[_led_nr].col_1 = 0;
                    _task_rgb_led.array[_led_nr].col_2 = (rx_msg.val_1/RGB_LED_UPDATE_INTERVAL) & 0xFFFF;
                    // dbgPrint(trRGB, "#%s LED: Rainbow effect at %dms (%d x %d = %d)", 
                    //     _index_to_led_name(_led_nr), rx_msg.val_1, _task_rgb_led.array[_led_nr].col_2, RGB_LED_UPDATE_INTERVAL, _task_rgb_led.array[_led_nr].col_2 * RGB_LED_UPDATE_INTERVAL);
                    _task_rgb_led.array[_led_nr].state = led_state_rainbow;
                    // sys_stopwatch_ms_start(&_task_rgb_led.sw);
                    sys_poll_tmr_stop(&_task_rgb_led.array[_led_nr].timer);
                    break;
                case led_action_status:
                {
                    switch (_task_rgb_led.array[_led_nr].state)
                    {
                        case led_state_off:
                            dbgPrint(trALWAYS, "%s LED OFF", _index_to_led_name(_led_nr));
                            break;
                        case led_state_on:
                            dbgPrint(trALWAYS, "%s LED ON, Colour 0x%06X", _index_to_led_name(_led_nr), _task_rgb_led.array[_led_nr].col_1, _task_rgb_led.array[_led_nr].state);
                            break;
                        case led_state_blink:
                            //Remember, the timer period is half the blink period (on/off)
                            dbgPrint(trALWAYS, "%s LED Blinking (0x%06X <-> 0x%06X), %.2f Hz", _index_to_led_name(_led_nr), _task_rgb_led.array[_led_nr].col_1, _task_rgb_led.array[_led_nr].col_2, (500.0f/_task_rgb_led.array[_led_nr].timer.ms_period));
                            break;
                        case led_state_rainbow:
                            //Remember, the bottom 16 bits of col_2 is the total count.... x RGB_LED_UPDATE_INTERVAL gives the period
                            dbgPrint(trALWAYS, "%s LED Rainbow, %dms", _index_to_led_name(_led_nr), (_task_rgb_led.array[_led_nr].col_2 & 0xFFFF) * RGB_LED_UPDATE_INTERVAL);
                            break;
                        default:
                            dbgPrint(trALWAYS, "%s LED UNKNOWN State (%d)", _index_to_led_name(_led_nr), _task_rgb_led.array[_led_nr].state);
                        break;
                    }
                    dbgPrint(trALWAYS, "");
                    break;
                }
                default:
                    break;
            }
        }
    }
}

void _task_rgb_led_service(void)
{     
    static uint32_t hue = 0;

    for (int _led_nr = 0; _led_nr < _task_rgb_led.cnt; _led_nr++)
    {
        rgb_led_t * rgb_led = &_task_rgb_led.array[_led_nr];
        switch (rgb_led->state)
        {
            //We only really need to deal with the blink and rainbow states (timer based)
            case led_state_blink:
                if (sys_poll_tmr_expired(&rgb_led->timer))
                {
                    uint32_t _temp_col = rgb_led->col_1;
                    //Set the current colour                    
                    _task_rgb_led_set_colour(_led_nr, rgb_led->col_1);
                    //Swop the primary and secondary colours for the next iteration
                    rgb_led->col_1 = rgb_led->col_2;
                    rgb_led->col_2 = _temp_col;
                    //Timer should be running on auto-reload, so no need to reset it
                }
                hue = 0;
                break;
            case led_state_rainbow:
                //This is assumed to run at the task interval period of 20ms (not using a timer)
                uint32_t _total = (rgb_led->col_2 & 0x0000FFFF);
                uint32_t _count = (rgb_led->col_2 >> 16) & 0x0000FFFF;
                hue = (HUE_MAX *_count/_total)%HUE_MAX;
                hsv2rgb(hue, SAT_MAX, VAL_MAX, &rgb_led->col_1);
                _task_rgb_led_set_colour(_led_nr, rgb_led->col_1);
                //dbgPrint(trRGB, "#Rainbow: %d/%d - H: %d -> 0x%06x)", _count, _total, hue, rgb_led->col_1);
                _count++;
                if (_count >= _total)
                {
                    // uint32_t _lap = sys_stopwatch_ms_reset(&_task_rgb_led.sw);
                    // dbgPrint(trRGB, "#Rainbow (re)starting after %dms", _lap);
                    _count = 0;
                }
                rgb_led->col_2 = ((((uint32_t)_count) << 16) & 0xFFFF0000) | ((uint32_t)_total & 0x0000FFFF);
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

void _task_rgb_led_set_colour(rgb_led_addr_t _index, uint32_t _rgb)
{
    rgb_led_strip_type _strip;
    uint32_t _addr;

    if (_index_to_led_type_addr(_index, &_strip, &_addr, NULL))
        drv_rgb_led_strip_set_led(_strip, _addr, _rgb);
}

bool _led_addr_to_bit_index(rgb_led_strip_type _strip, uint32_t _addr, rgb_led_addr_t * _index)
{
    uint32_t strip_led_cnt;

    if ((_index == NULL) || (_strip >= strips_Total))
    {
        dbgPrint(trALWAYS|trRGB, "#%s() - Invalid parameters (%d)", __FUNCTION__, _strip);
        return false;
    }

    strip_led_cnt = drv_rgb_led_strip_count(_strip);
    if (_addr >= strip_led_cnt)
    {
        dbgPrint(trALWAYS|trRGB, "#%s() - LED Strip Address out of bounds (%d/%d)", __FUNCTION__, _addr, strip_led_cnt);
        return false;
    }

    *_index = (_strip == strip_Debug)? led_bit_dbg : led_bit_btn_0 + _addr;

    return true;
}

bool _index_to_led_type_addr(rgb_led_addr_t _index, rgb_led_strip_type * _type, uint32_t * _led_nr, char * led_name /* Optional */)
{
    // typedef enum
    // {
    //     led_bit_dbg = 0,        /* Debug LED (only 1) */
    //     led_bit_btn_0 = 1,      /* Start of Button LEDs */
    //     /* Keep this at the end*/
    //     led_bit_max = 16,       /* Allows for up to 16 LEDs in total (dbg + 15) */
    // }rgb_led_addr_t;

    if (_index >= led_bit_max)
    {
        dbgPrint(trALWAYS|trRGB, "#%s() - Index out of bounds (%d/%d)", __FUNCTION__, _index, led_bit_max);
        return false;
    }

    if ((_type == NULL) || (_led_nr == NULL))
    {
        dbgPrint(trALWAYS|trRGB, "#%s() - Invalid parameters", __FUNCTION__);
        return false;
    }

    *_type = (_index == led_bit_dbg)? strip_Debug : strip_Buttons;
    *_led_nr = (_index == led_bit_dbg)? 0 : _index - led_bit_btn_0;

    /* Writing the Name string is optional */
    if (led_name != NULL)
        strcpy(led_name, _index_to_led_name(_index));

    return true;
}

char * _index_to_led_name(rgb_led_addr_t _index)
{
    rgb_led_strip_type _type;
    uint32_t _led_nr;
    static char _led_name[MAX_LED_NAME_LEN];

    if (_index >= led_bit_max)
    {
        dbgPrint(trALWAYS|trRGB, "#%s() - Index out of bounds (%d/%d)", __FUNCTION__, _index, led_bit_max);
        return NULL;
    }

    _type = (_index == led_bit_dbg)? strip_Debug : strip_Buttons;
    _led_nr = (_index == led_bit_dbg)? 0 : _index - led_bit_btn_0;

    strncpy(_led_name, drv_rgb_led_striptype2name(_type), MAX_LED_NAME_LEN);
    if (_type != strip_Debug)
    {
        char * str = _led_name + strlen(_led_name);
        snprintf(str, MAX_LED_NAME_LEN-strlen(_led_name), " #%d", (int)_led_nr);
    }

    return _led_name;
}

#define LED_STRIP_NAMES 6
typedef struct {
    const char * name;
    rgb_led_strip_type type;
} _led_strip_name_t;
const _led_strip_name_t _led_strips[LED_STRIP_NAMES] =
{
    {"Debug",   strip_Debug},
    {"Dbg",     strip_Debug},
    {"Buttons", strip_Buttons},
    {"Button",  strip_Buttons},
    {"Btns",    strip_Buttons},
    {"Btn",     strip_Buttons},
};

esp_err_t _task_rgb_menu_handler_parse_address_str(uint32_t * addr_mask, const char * str)
{
    uint32_t value;

    //Righto, the address can be in one of three valid formats:
    // 1. A single number (0 to 15)
    // 2. A hexadecimal number (0xHHHH), e.g. "0x003", "0x0F0", etc
    // 3. A string containing "Debug|Dbg|Button|Btn" optionally followed with a colon (:) and a number, e.g. "debug", "btn:1", etc
    
    if (str2int32((int32_t*)&value, str, 0))
    {
        //This would be a single led index
        if (value >= led_bit_max)
        {
            dbgPrint(trALWAYS, "Invalid LED Address (\"%s\" -> %d)", str, value);
            return ESP_ERR_INVALID_ARG;
        }
        *addr_mask = BIT(value);
        return ESP_OK;
    }
    
    if (hex2u32(&value, str, 4))
    {
        //This could be several led indices (the whole mask)
        *addr_mask = value;
        return ESP_OK;
    }

    //Let's first check if any of the usual suspects are present at the start of the string.
    for (int i = 0; i < LED_STRIP_NAMES; i++)
    {
        const char * strip_name = _led_strips[i].name;
        int match_len = strlen(strip_name);
        if (strncasecmp(strip_name, str, match_len) == 0)
        {
            rgb_led_strip_type type = _led_strips[i].type; 
            //From here on out we are committed to return either true or false, with an error msg
            str += match_len;
            //We have a match, let's see what is next
            if ((*str == 0) || ((*str == ':') && (*(str+1) == 0)))
            {
                //The entire LED strip.... The address is either bit 0 (0x0001), or all the rest (0xFFFE)
                if (type == strip_Debug)
                    *addr_mask = BIT(led_bit_dbg);
                else
                    for (int i = 0; i < drv_rgb_led_strip_count(type); i++)
                        *addr_mask |= BIT(led_bit_btn_0 + i);
                return ESP_OK;
            }
            //Is it maybe immediately followed by a colon and then a number
            if (*str == ':')
            {
                //Okay, this is looking good
                str++;
                int32_t led_nr;
                if (str2int32(&led_nr, str, 0))
                {
                    if (led_nr >= drv_rgb_led_strip_count(type))
                    {
                        dbgPrint(trALWAYS, "Invalid LED # for \"%s\" strip (%d)", strip_name, led_nr);
                        return ESP_ERR_INVALID_ARG;
                    }
                    *addr_mask = BIT(type + led_nr);
                    return ESP_OK;
                }
                dbgPrint(trALWAYS, "Invalid LED # for \"%s\" strip (\"%s\")", strip_name, str);
                return ESP_ERR_INVALID_ARG;
            }
        }
        //Try next strip name
    }

    //Reaching this point means this is NOT an LED address
    return ESP_ERR_NOT_FOUND;
}

esp_err_t _task_rgb_menu_handler_parse_colour(uint32_t * colour_value, const char * str)
{
    uint32_t rgb_col_24bit;

    //Righto, a colour can be in one of two valid formats:
    // 1. NOT USED: A 6-character hexadecimal string (0xHHHHHH), e.g. "0x000001", "#0F0F0F", etc
    // 2. A string containing any one of the assigned colour names, e.g. "Black", "wh", etc
    // 3. A HUE value in the range 0 <= hue < 360 degrees provided as "Hue:<#>"

    // if (hex2u32(&rgb_col_24bit, str, 6))
    // {
    //     //This could be several led indices (the whole mask)
    //     *colour_value = rgb_col_24bit;
    //     return ESP_OK;
    // }
    if (str2rgb(&rgb_col_24bit, str) == ESP_OK)
    {
        *colour_value = rgb_col_24bit;
        return ESP_OK;
    }
    if (strncasecmp("Hue:", str, 4) == 0)
    {
        //This could be a hue value
        if (str2uint32(&rgb_col_24bit, str+4, 0) == ESP_OK)
        {
            if (rgb_col_24bit > HUE_MAX)
            {
                dbgPrint(trALWAYS, "Invalid Hue value (%d > %d)", rgb_col_24bit, HUE_MAX);
                return ESP_ERR_INVALID_ARG;
            }
            *colour_value = rgb_col_24bit;
            return ESP_OK;
        }
        dbgPrint(trALWAYS, "Invalid Hue value (\"%s\")", str);
        return ESP_ERR_INVALID_ARG;
    }
    //Reaching this point means this is NOT an colour value or name
    return ESP_ERR_NOT_FOUND;
}


void _menu_handler_led_off(void)
{
    //These functions (_menu_handler....) are called from the console task, so they should 
    // not manupiluate the RGB task variables directly, but instead send messages to the RGB 
    // task via the msg_queue
    bool help_requested = false;
    rgb_led_action_msg_t first_msg = {.led_addr = 0};//Indicating "not set yet"
    esp_err_t err = ESP_OK;
    uint32_t parse_value = 0;
//    uint32_t addr_mask = 0;

    if (task_console_arg_cnt() == 0)
    {
        for (int i = 0; i < _task_rgb_led.cnt; i++)
            first_msg.led_addr |= BIT(i);
    }

    while (task_console_arg_cnt() > 0)
	{
        char *arg = task_console_arg_pop();
        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from while-loop
        }
        parse_value = 0;
        err = _task_rgb_menu_handler_parse_address_str(&parse_value, arg);
        if (err == ESP_OK)
        {
            first_msg.led_addr |= parse_value;
            continue;
        }
        if (err == ESP_ERR_NOT_FOUND)   //ESP_ERR_INVALID_ARG has already been handled ito user feedback
            dbgPrint(trALWAYS, "Invalid Argument (\"%s\")", arg);
        // help_requested = true;
        break;
    }

    if ((!help_requested))
    {
        if (first_msg.led_addr == 0)
        {   
            // No LED Address(es) specified... let's apply to all LEDs
            for (int i = 0; i < _task_rgb_led.cnt; i++)
                first_msg.led_addr |= BIT(i);
        }
        first_msg.cmd = led_action_off;
        xQueueSend(_task_rgb_led.msg_queue, &first_msg, 0);

        //Maybe a good idea to also queue a status request right after this?
        first_msg.cmd = led_action_status;
        // for (int i = 0; i < _task_rgb_led.cnt; i++)
        //     first_msg.led_addr |= BIT(i);
        xQueueSend(_task_rgb_led.msg_queue, &first_msg, 0);
    
    }
    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        dbgPrint(trALWAYS, "Usage: \"ledoff [<led_addr>]\"");        
        dbgPrint(trALWAYS, "    <led_addr>: <#>      - a single index (0 to %d), e.g. 0, 1, 2, 3, etc", _task_rgb_led.cnt-1);
        dbgPrint(trALWAYS, "                0xHHHH - a 16 bit mask of the LEDs to affect, e.g. \"0x003\"");
        dbgPrint(trALWAYS, "                \"<strip>[:<nr>]\" - a string and number (seperated by a colon)");
        dbgPrint(trALWAYS, "                            indicating the LED strip and LED # to use");
        dbgPrint(trALWAYS, "                            e.g. \"debug:0\", \"button:1\", etc");
        dbgPrint(trALWAYS, "                  If <nr> is omitted, %s applies to the entire strip", "OFF");
        dbgPrint(trALWAYS, "        If <led_addr> is omitted, %s applies to all leds", "OFF");
        dbgPrint(trALWAYS, " Multiple <led_addr> values can be specified, separated by spaces");
        //        dbgPrint(trALWAYS, "   e.g. \"off 0 1 2 3\", \"off debug:0 button:1...\" etc");
    }
}

void _menu_handler_led_col(void)
{
    //These functions (_menu_handler....) are called from the console task, so they should 
    // not manupiluate the RGB task variables directly, but instead send messages to the RGB 
    // task via the msg_queue
    bool help_requested = false;
    rgb_led_action_msg_t first_msg = {.led_addr = 0, .val_0 = -1};//Indicating "not set yet"
    esp_err_t err = ESP_OK;
    uint32_t parse_value = 0;

    if (task_console_arg_cnt() == 0)
    {
        for (int i = 0; i < _task_rgb_led.cnt; i++)
            first_msg.led_addr |= BIT(i);
    }

    while (task_console_arg_cnt() > 0)
	{
        char *arg = task_console_arg_pop();
        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from while-loop
        }
        parse_value = 0;
        err = _task_rgb_menu_handler_parse_colour(&parse_value, arg);
        if (err == ESP_OK)
        {
            if (first_msg.val_0 != (uint32_t)-1)
                dbgPrint(trALWAYS, "Overwriting previously set colour (%06X) with \"%s\" (%06X)", first_msg.val_0, arg, parse_value);

            first_msg.val_0 = parse_value;
            continue;
        }
        else if (err == ESP_ERR_INVALID_ARG)
        {
            //Looked like a Hue value, but it was invalid
            help_requested = true;
            break;
        }
        err = _task_rgb_menu_handler_parse_address_str(&parse_value, arg);
        if (err == ESP_OK)
        {
            first_msg.led_addr |= parse_value;
            continue;
        }
        else if (err == ESP_ERR_INVALID_ARG)
        {
            //Looked like an address, but it was invalid
            help_requested = true;
            break;
        }
        //else if (err == ESP_ERR_NOT_FOUND) /// fall through to colour check

        dbgPrint(trALWAYS, "Invalid Argument (\"%s\")", arg);
        help_requested = true;
        break;
    }

    if (!help_requested)
    {
        if (first_msg.val_0 == (uint32_t)-1)
        {
            // No colour? We'll assign one using a very silly RNG based on the time since boot in us (mod 360)
            hsv2rgb((uint32_t)(esp_timer_get_time()%360L), 100, 100, &first_msg.val_0);
            dbgPrint(trALWAYS, "No Colour specified, using 0x%06X", first_msg.val_0);
        }
        if (first_msg.led_addr == 0)
        {   
            // No LED Address(es) specified... let's apply to all LEDs
            for (int i = 0; i < _task_rgb_led.cnt; i++)
                first_msg.led_addr |= BIT(i);
        }
        if (first_msg.val_0 == colBlack)
        {
            //This is a special case... we are turning the LED off
            dbgPrint(trALWAYS, "Setting the Colour to \"Black\" is the same as turning the LED OFF");
            first_msg.cmd = led_action_off;
        }
        else
        {
            first_msg.cmd = led_action_colour;
        }
        xQueueSend(_task_rgb_led.msg_queue, &first_msg, 0);
        //Maybe a good idea to also queue a status request right after this?
        first_msg.cmd = led_action_status;
        // for (int i = 0; i < _task_rgb_led.cnt; i++)
        //     first_msg.led_addr |= BIT(i);
        xQueueSend(_task_rgb_led.msg_queue, &first_msg, 0);
    }

    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        dbgPrint(trALWAYS, "Usage: \"ledon <colour> [<led_addr>]\"");        
        // dbgPrint(trALWAYS, "    <colour>:   0xRRGGBB - A 6-character hexadecimal string containing the 24-");
        // dbgPrint(trALWAYS, "                            -bit RGB value, e.g. \"0x000001\", \"#0F0F0F\", etc");
        dbgPrint(trALWAYS, "    <colour>:   String   - Any one of the assigned colour names");
        dbgPrint(trALWAYS, "                            e.g. \"Black\", \"wh\", etc");
        dbgPrint(trALWAYS, "                \"Hue:<#>\" - A hue value in the range 0 to 360 degrees");
        dbgPrint(trALWAYS, "        If <colour> is omitted, one will be selected at random");
        dbgPrint(trALWAYS, "    <led_addr>: <#>      - a single index (0 to %d), e.g. 0, 1, 2, 3, etc", _task_rgb_led.cnt-1);
        dbgPrint(trALWAYS, "                0xHHHH - a 16 bit mask of the LEDs to affect, e.g. \"0x003\"");
        dbgPrint(trALWAYS, "                \"<strip>[:<nr>]\" - a string and number (seperated by a colon)");
        dbgPrint(trALWAYS, "                            indicating the LED strip and LED # to use");
        dbgPrint(trALWAYS, "                            e.g. \"debug:0\", \"button:1\", etc");
        dbgPrint(trALWAYS, "                  If <nr> is omitted, %s applies to the entire strip", "ON");
        dbgPrint(trALWAYS, "        If <led_addr> is omitted, %s applies to all leds", "ON");
        dbgPrint(trALWAYS, " Multiple <led_addr> values can be specified, separated by spaces");
//        dbgPrint(trALWAYS, "   e.g. \"off 0 1 2 3\", \"off debug:0 button:1...\" etc");
    }
}

void _menu_handler_led_blink(void)
{
    //These functions (_menu_handler....) are called from the console task, so they should 
    // not manupiluate the RGB task variables directly, but instead send messages to the RGB 
    // task via the msg_queue
    bool help_requested = false;
    rgb_led_action_msg_t first_msg = {.led_addr = 0, .val_0 = -1, .val_1 = 0, .val_2 = -1};//Indicating "not set yet"
    esp_err_t err = ESP_OK;
    uint32_t parse_value = 0;//addr_mask = 0;

    if (task_console_arg_cnt() == 0)
    {
        for (int i = 0; i < _task_rgb_led.cnt; i++)
            first_msg.led_addr |= BIT(i);
    }

    while (task_console_arg_cnt() > 0)
	{
        char *arg = task_console_arg_pop();
        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from while-loop
        }
        parse_value = 0;
        //Check for period first
        if (str2uint32(&parse_value, arg, 0))
        {
            //This could be the period OR an address
            if (parse_value >= RGB_LED_BLINK_PERIOD_MS_MIN*2)
            {
                //Ooooh... this could be a period
                if (first_msg.val_1 != 0)
                    dbgPrint(trALWAYS, "Overwriting previously set period (%dms) with \"%s\" (%dms)", first_msg.val_1, arg, parse_value);
                first_msg.val_1 = parse_value;
                continue;
            }
            // else if (parse_value >= _task_rgb_led.cnt)
            // {
            //     //This is also NOT an address
            //     dbgPrint(trALWAYS, "Invalid Period/Address value (\"%s\" -> %d)", arg, parse_value);
            //     help_requested = true;
            //     break;
            // }
            //Now we need to assume it is a valid(?) address... fall through to the next check?
        }
        //else not a good period value... maybe it is something
        //Then we check for colour(s)
        err = _task_rgb_menu_handler_parse_colour(&parse_value, arg);
        if (err == ESP_OK)
        {
            if (first_msg.val_0 == (uint32_t)-1)
            {
                first_msg.val_0 = parse_value;
            }
            else
            {
                if (first_msg.val_2 != (uint32_t)-1)
                    dbgPrint(trALWAYS, "Overwriting previously set colour (%06X) with \"%s\" (%06X)", first_msg.val_2, arg, parse_value);            
                first_msg.val_2 = parse_value;
            }
            continue;
        }
        else if (err == ESP_ERR_INVALID_ARG)
        {
            //Looked like an address, but it was invalid
            help_requested = true;
            break;
        }
        //Then we check for LED addressing
        err = _task_rgb_menu_handler_parse_address_str(&parse_value, arg);
        if (err == ESP_OK)
        {
            first_msg.led_addr |= parse_value;
            continue;
        }
        else if (err == ESP_ERR_INVALID_ARG)
        {
            //Looked like an address, but it was invalid
            help_requested = true;
            break;
        }
        //else if (err == ESP_ERR_NOT_FOUND) /// fall through to colour check
        dbgPrint(trALWAYS, "Invalid Argument (\"%s\")", arg);
        help_requested = true;
        break;
    }

    if (!help_requested)
    {
        if (first_msg.val_0 == (uint32_t)-1)
        {
            // No colour? We'll assign both using a very silly RNG based on the time since boot in us (mod 360)
            uint32_t rand_hue = (uint32_t)(esp_timer_get_time()%360L);
            hsv2rgb(rand_hue, SAT_MAX, VAL_MAX, &first_msg.val_0);
            //The second colour is the complementary colour on the other side (+180 degrees) of the colour wheel
            hsv2rgb(rand_hue+(HUE_MAX/2), SAT_MAX, VAL_MAX, &first_msg.val_2);
            dbgPrint(trALWAYS, "No Colours specified, using 0x%06X and 0x%06X", first_msg.val_0, first_msg.val_2);
        }
        else if (first_msg.val_2 == (uint32_t)-1)
        {
            // No colour? We'll assign one using a very silly RNG based on the time since boot in us (mod 360)
            hsv2rgb((uint32_t)(esp_timer_get_time()%360L), 100, 100, &first_msg.val_2);
            dbgPrint(trALWAYS, "No Colour specified, using 0x%06X", first_msg.val_2);
        }
        

        if (first_msg.val_1 == 0)
        {
            // No period? We'll assign one using a very silly RNG based on the time since boot in us (mod 360)
            first_msg.val_1 = RGB_LED_BLINK_PERIOD_MS_DEF;
            dbgPrint(trALWAYS, "Using default period of %dms", first_msg.val_1);
        }
        if (first_msg.led_addr == 0)
        {   
            // No LED Address(es) specified... let's apply to all LEDs
            for (int i = 0; i < _task_rgb_led.cnt; i++)
                first_msg.led_addr |= BIT(i);
        }
        first_msg.cmd = led_action_blink;
        xQueueSend(_task_rgb_led.msg_queue, &first_msg, 0);
        //Maybe a good idea to also queue a status request right after this?
        first_msg.cmd = led_action_status;
        // for (int i = 0; i < _task_rgb_led.cnt; i++)
        //     first_msg.led_addr |= BIT(i);
        xQueueSend(_task_rgb_led.msg_queue, &first_msg, 0);
    }

    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        dbgPrint(trALWAYS, "Usage: \"ledblink <period> [<colour_1>] [<colour_2>] [<led_addr>]\"");        
        dbgPrint(trALWAYS, "    <period>:   A value indicating the period of the blink in ms (min: %dms)", RGB_LED_BLINK_PERIOD_MS_MIN*2);
        dbgPrint(trALWAYS, "        If <period> is omitted, a default period of %dms is used", RGB_LED_BLINK_PERIOD_MS_DEF);
        dbgPrint(trALWAYS, "    <colour>:   String   - Any one of the assigned colour names");
        dbgPrint(trALWAYS, "                            e.g. \"Black\", \"wh\", etc");
        dbgPrint(trALWAYS, "                \"Hue:<#>\" - A hue value in the range 0 to 360 degrees");
        dbgPrint(trALWAYS, "        If one or both <colour> are omitted, they will be selected at random");
        dbgPrint(trALWAYS, "    <led_addr>: <#>      - a single index (0 to %d), e.g. 0, 1, 2, 3, etc", _task_rgb_led.cnt-1);
        dbgPrint(trALWAYS, "                0xHHHH - a 16 bit mask of the LEDs to affect, e.g. \"0x003\"");
        dbgPrint(trALWAYS, "                \"<strip>[:<nr>]\" - a string and number (seperated by a colon)");
        dbgPrint(trALWAYS, "                            indicating the LED strip and LED # to use");
        dbgPrint(trALWAYS, "                            e.g. \"debug:0\", \"button:1\", etc");
        dbgPrint(trALWAYS, "                  If <nr> is omitted, %s applies to the entire strip", "BLINK");
        dbgPrint(trALWAYS, "        If <led_addr> is omitted, %s applies to all leds", "BLINK");
        dbgPrint(trALWAYS, " Multiple <led_addr> values can be specified, separated by spaces");
//        dbgPrint(trALWAYS, "   e.g. \"off 0 1 2 3\", \"off debug:0 button:1...\" etc");
    }

    //Print a status of all LEDs?
}

void _menu_handler_led_rainbow(void)
{
    //These functions (_menu_handler....) are called from the console task, so they should 
    // not manupiluate the RGB task variables directly, but instead send messages to the RGB 
    // task via the msg_queue
    bool help_requested = false;
    rgb_led_action_msg_t first_msg = {.led_addr = 0, .val_1 = 0};//Indicating "not set yet"
    esp_err_t err = ESP_OK;
    uint32_t parse_value = 0;//addr_mask = 0;

    if (task_console_arg_cnt() == 0)
    {
        for (int i = 0; i < _task_rgb_led.cnt; i++)
            first_msg.led_addr |= BIT(i);
    }

    while (task_console_arg_cnt() > 0)
	{
        char *arg = task_console_arg_pop();
        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from while-loop
        }
        parse_value = 0;
        //Check for period first
        if (str2uint32(&parse_value, arg, 0))
        {
            //This could be the period OR an address
            if (parse_value >= RGB_LED_BLINK_PERIOD_MS_MIN*2)
            {
                //Ooooh... this could be a period
                if (first_msg.val_1 != 0)
                    dbgPrint(trALWAYS, "Overwriting previously set period (%dms) with \"%s\" (%dms)", first_msg.val_1, arg, parse_value);
                first_msg.val_1 = parse_value;
                continue;
            }
            //Now we need to assume it is an address... fall through to the next check?
        }
        //else not a good period value... maybe it is something
        //Then we check for LED addressing
        err = _task_rgb_menu_handler_parse_address_str(&parse_value, arg);
        if (err == ESP_OK)
        {
            first_msg.led_addr |= parse_value;
            continue;
        }
        else if (err == ESP_ERR_INVALID_ARG)
        {
            //Looked like an address, but it was invalid
            help_requested = true;
            break;
        }
        //else if (err == ESP_ERR_NOT_FOUND) /// fall through to colour check
        dbgPrint(trALWAYS, "Invalid Argument (\"%s\")", arg);
        help_requested = true;
        break;
    }

    if (!help_requested)
    {
        if (first_msg.val_1 == 0)
        {
            // No period? We'll assign one using a very silly RNG based on the time since boot in us (mod 360)
            first_msg.val_1 = RGB_LED_RAINBOW_PERIOD_MS_DEF;
            dbgPrint(trALWAYS, "Using default period of %dms", first_msg.val_1);
        }

        if (first_msg.led_addr == 0)
        {   
            // No LED Address(es) specified... let's apply to all LEDs
            for (int i = 0; i < _task_rgb_led.cnt; i++)
                first_msg.led_addr |= BIT(i);
        }
        first_msg.cmd = led_action_rainbow;
        xQueueSend(_task_rgb_led.msg_queue, &first_msg, 0);
        //Maybe a good idea to also queue a status request right after this?
        first_msg.cmd = led_action_status;
        // for (int i = 0; i < _task_rgb_led.cnt; i++)
        //     first_msg.led_addr |= BIT(i);
        xQueueSend(_task_rgb_led.msg_queue, &first_msg, 0);
    }

    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        dbgPrint(trALWAYS, "Usage: \"ledblink <period> [<colour_1>] [<colour_2>] [<led_addr>]\"");        
        dbgPrint(trALWAYS, "    <period>:   A value indicating the period of the rainbow cycle in ms");
        dbgPrint(trALWAYS, "                            range %d to %d", RGB_LED_RAINBOW_PER_MIN_MS, RGB_LED_RAINBOW_PER_MAX_MS);
        dbgPrint(trALWAYS, "        If <period> is omitted, a default period of %dms is used", RGB_LED_RAINBOW_PERIOD_MS_DEF);
        dbgPrint(trALWAYS, "    <led_addr>: <#>      - a single index (0 to %d), e.g. 0, 1, 2, 3, etc", _task_rgb_led.cnt-1);
        dbgPrint(trALWAYS, "                0xHHHH - a 16 bit mask of the LEDs to affect, e.g. \"0x003\"");
        dbgPrint(trALWAYS, "                \"<strip>[:<nr>]\" - a string and number (seperated by a colon)");
        dbgPrint(trALWAYS, "                            indicating the LED strip and LED # to use");
        dbgPrint(trALWAYS, "                            e.g. \"debug:0\", \"button:1\", etc");
        dbgPrint(trALWAYS, "                  If <nr> is omitted, %s applies to the entire strip", "RAINBOW");
        dbgPrint(trALWAYS, "        If <led_addr> is omitted, %s applies to all leds", "RAINBOW");
        dbgPrint(trALWAYS, " Multiple <led_addr> values can be specified, separated by spaces");
//        dbgPrint(trALWAYS, "   e.g. \"off 0 1 2 3\", \"off debug:0 button:1...\" etc");
    }
}

void _menu_handler_led_status(void)
{
    //These functions (_menu_handler....) are called from the console task, so they should 
    // not manupiluate the RGB task variables directly, but instead send messages to the RGB 
    // task via the msg_queue
    bool help_requested = false;
    rgb_led_action_msg_t first_msg = {.led_addr = 0};//Indicating "not set yet"
    esp_err_t err = ESP_OK;
    uint32_t parse_value = 0;
//    uint32_t addr_mask = 0;

    if (task_console_arg_cnt() == 0)
    {
        for (int i = 0; i < _task_rgb_led.cnt; i++)
            first_msg.led_addr |= BIT(i);
    }

    while (task_console_arg_cnt() > 0)
	{
        char *arg = task_console_arg_pop();
        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from while-loop
        }
        parse_value = 0;
        err = _task_rgb_menu_handler_parse_address_str(&parse_value, arg);
        if (err == ESP_OK)
        {
            first_msg.led_addr |= parse_value;
            continue;
        }
        if (err == ESP_ERR_NOT_FOUND)   //ESP_ERR_INVALID_ARG has already been handled ito user feedback
            dbgPrint(trALWAYS, "Invalid Argument (\"%s\")", arg);
        // help_requested = true;
        break;
    }

    if ((!help_requested))
    {
        if (first_msg.led_addr == 0)
        {   
            // No LED Address(es) specified... let's apply to all LEDs
            for (int i = 0; i < _task_rgb_led.cnt; i++)
                first_msg.led_addr |= BIT(i);
        }
        first_msg.cmd = led_action_status;
        // for (int i = 0; i < _task_rgb_led.cnt; i++)
        //     first_msg.led_addr |= BIT(i);
        xQueueSend(_task_rgb_led.msg_queue, &first_msg, 0);
    
    }
    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        dbgPrint(trALWAYS, "Usage: \"ledstat [<led_addr>]\"");        
        dbgPrint(trALWAYS, "    <led_addr>: <#>      - a single index (0 to %d), e.g. 0, 1, 2, 3, etc", _task_rgb_led.cnt-1);
        dbgPrint(trALWAYS, "                0xHHHH - a 16 bit mask of the LEDs to affect, e.g. \"0x003\"");
        dbgPrint(trALWAYS, "                \"<strip>[:<nr>]\" - a string and number (seperated by a colon)");
        dbgPrint(trALWAYS, "                            indicating the LED strip and LED # to use");
        dbgPrint(trALWAYS, "                            e.g. \"debug:0\", \"button:1\", etc");
        dbgPrint(trALWAYS, "                  If <nr> is omitted, %s applies to the entire strip", "STATUS");
        dbgPrint(trALWAYS, "        If <led_addr> is omitted, %s applies to all leds", "STATUS");
        dbgPrint(trALWAYS, " Multiple <led_addr> values can be specified, separated by spaces");
        //        dbgPrint(trALWAYS, "   e.g. \"off 0 1 2 3\", \"off debug:0 button:1...\" etc");
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
	if (_task_rgb_led.task.handle)
		return &_task_rgb_led.task;

	_task_rgb_led.cnt = 0;

	// Create the task, storing the handle.  Note that the passed parameter ucParameterToPass
	// must exist for the lifetime of the task, so in this case is declared static.  If it was just an
	// an automatic stack variable it might no longer exist, or at least have been corrupted, by the time
	// the new task attempts to access it.
	if (xTaskCreate( _task_rgb_led_task, PRINTF_TAG, _task_rgb_led.task.stack_depth, _task_rgb_led.task.parameter_to_pass, 10, &_task_rgb_led.task.handle ) != pdPASS)
	{
		dbgPrint(trALWAYS, "#Unable to start Task (%d)!", eTaskIndexRgb);
		return NULL;
	}

	configASSERT(_task_rgb_led.task.handle);

	return &_task_rgb_led.task;
}

void task_rgb_btn_deinit(void)
{
	// Use the handle to delete the task.
	if(_task_rgb_led.task.handle != NULL )
		vTaskDelete(_task_rgb_led.task.handle);

}

#undef PRINTF_TAG

/*************************** END OF FILE *************************************/
