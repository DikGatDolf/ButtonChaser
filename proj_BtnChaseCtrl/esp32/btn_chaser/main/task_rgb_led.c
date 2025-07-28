/*******************************************************************************

Module:     task_rgb_led.c
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
#include "sys_task_utils.h"
#include "str_helper.h"
#include "task_console.h"
#include "drv_rgb_led_strip.h"
#include "esp_random.h"

#define __NOT_EXTERN__
#include "task_rgb_led.h"
#undef __NOT_EXTERN__

/*******************************************************************************
Macros and Constants
 *******************************************************************************/

#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("LED Task") /* This must be undefined at the end of the file*/

#define RGB_STACK_SIZE 4096

#define LED_UPDATE_INTERVAL_MS      20     /* Task cycles at a 50Hz rate*/
#define LED_BLINK_PERIOD_MS_MIN     100     /* 10Hz/2 = 5Hz... is already fairly fast for a blinking period */
#define LED_BLINK_PERIOD_MS_DEF     400    /* 1Hz */
#define LED_RAINBOW_PER_MIN_MS      1800    /* Cycling through the spectrum in 1.8s */
#define LED_RAINBOW_PER_MAX_MS      (0xFFFF * LED_UPDATE_INTERVAL_MS)    /* Cycling through the spectrum in 21m 50.7s */
#define LED_RAINBOW_PERIOD_MS_DEF   3800

/*******************************************************************************
local defines 
 *******************************************************************************/
typedef enum
{
    led_bit_dbg = 0,        /* Debug LED (only 1) */
    led_bit_btn_0 = 1,      /* Start of Button LEDs */
    /* Keep this at the end*/
    led_bit_max = 16,       /* Allows for up to 16 LEDs in total (dbg + 15) */
}led_addr_bit_index;

typedef enum
{
    led_state_off = 0,
    led_state_on = 1,
    led_state_blink = 2,
    led_state_rainbow = 3,
}rgb_led_state_t;

typedef enum {
    led_action_nop,         /* No value required */
    led_action_off,         /* No value required */
    led_action_colour,      /* 24 bit RGB value required */
    led_action_blink,       /* blink period in ms (0 to 16,777,215/4h39m37.215s) */
    led_action_rainbow,     /* rotation period in ms (0 to 16,777,215/4h39m37.215s) */
    led_action_status,      /* No value required */
    /* Keep this at the end*/
    led_action_total,
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
    bool init_done;
    int cnt;
    rgb_led_t * array;    
	TaskInfo_t task;
    QueueHandle_t msg_queue;

    //Stopwatch_ms_t sw;  /* used for debugging */

} DeviceRgb_t;

//We need to create a message queue for the LED task
//static QueueHandle_t _queue;


/*******************************************************************************
local function prototypes
 *******************************************************************************/
/*! Deinitialises the RGB driving
 */
void _led_task_deinit(void);

/*! The main function for the LED task
 */
void _led_main_func(void * pvParameters);

/*! @brief Initialises the LED task
 */
void _led_setup(void);

/*! @brief Deinitialises the LED task
 */
void _led_teardown(void);

/*! @brief Displays information about a specific LED (Only for debug purposes)
 * @param _index The index of the LED to display information about
 */
void _led_info_print(int _index);

/*! @brief Reads the message queue for the LED task
 */
void _read_msg_queue(void);

/*! @brief The main service() function for the LED task
 */
void _led_service(void);

/*! @brief Sets the colour of the LED at the specified index
 * @param _index The index of the LED to set the colour for
 * @param _rgb The 24-bit RGB value to set the LED to
 */
void _set_colour(led_addr_bit_index _index, uint32_t _rgb);

/*! @brief Parses the arguments in the string and converts it to set bits in the address mask
 * @param[out] addr_mask The address mask if successful, otherwise unchanged
 * @param[in] arg_str The string containing the arguments to parse
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t _handler_parse_address_str(uint32_t * addr_mask, const char * str);

/*! @brief Common action handler for LED actions
 */
void _led_handler_common_action(void);

/*! @brief Console menu handler to display the RGB hex values
 */
void _led_handler_col_list(void);

/*! @brief Console menu handler to display the status of LEDs
 */
void _led_handler_status(void);

/*! @brief Console menu handler to reset the state of the LEDs
 */
void _led_handler_reset(void);

/*******************************************************************************
local variables
 *******************************************************************************/
ConsoleMenuItem_t _led_menu_items[] =
{
									        // 01234567890123456789012345678901234567890123456789012345678901234567890123456789
        {"off",     _led_handler_common_action, "Turns off LEDs"},
        {"on",      _led_handler_common_action, "Sets LEDs to a specific colour"},
        {"blink",   _led_handler_common_action, "Blinks LEDs at a specified rate and colour(s)"},
        {"rainbow", _led_handler_common_action, "Apply rainbow effect on LEDs"},
        {"rgb",     _led_handler_col_list,      "Displays the RGB hex values for predefined colours"},
		{"status",  _led_handler_status,        "Displays the status of the LEDs"},
		{"reset",   _led_handler_reset,         "Resets the state of the LEDs"},
};

static DeviceRgb_t _rgb_led = {
    .cnt = 0,
    .array = NULL,
	.task = {   
        .init_done = false,
        .handle = NULL,
            .stack_depth = RGB_STACK_SIZE,
            .stack_unused = 0,
            // .init_func = rgb_led_init_task,
            // .deinit_func = _led_task_deinit,
    },
    .msg_queue = NULL,
};

/*******************************************************************************
Local (private) Functions
*******************************************************************************/

void _led_task_deinit(void)
{
	// Use the handle to delete the task.
	if(_rgb_led.task.handle != NULL )
		vTaskDelete(_rgb_led.task.handle);
}

void _led_main_func(void * pvParameters)
{
    // uint16_t hue = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    //Wait until the initialisation is done
    while (!_rgb_led.task.init_done)
        xTaskDelayUntil(&xLastWakeTime, 1);//pdMS_TO_TICKS(LED_UPDATE_INTERVAL_MS));

    iprintln(trLED|trALWAYS, "#Task Started (%d). Running @ %d Hz", _rgb_led.cnt, (1000/LED_UPDATE_INTERVAL_MS));

    while (1) 
    {

        _read_msg_queue();
        
        _led_service();

        xTaskDelayUntil(&xLastWakeTime, MAX(1, pdMS_TO_TICKS(LED_UPDATE_INTERVAL_MS)));  //vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
    
        /* Inspect our own high water mark on entering the task. */
    	_rgb_led.task.stack_unused = uxTaskGetStackHighWaterMark2( NULL );
    }

    _led_teardown();

    iprintln(trLED|trALWAYS, "#LED Task Complete!");
}

void _led_setup(void)
{
    if (_rgb_led.cnt > 0)
    {
        iprintln(trLED|trALWAYS, "#Already initialised");
        return; //Already initialised
    }

    _rgb_led.cnt = drv_rgb_led_strip_init();

    //iprintln(trLED|trALWAYS, "#LED Task init done (%d LEDs in total)", _rgb_led.cnt);

#ifdef CONSOLE_ENABLED
    console_add_menu("led", _led_menu_items, ARRAY_SIZE(_led_menu_items), "LED Control");
#endif

    //We need to create a message queue for the LED task, let's make it double to the total led_action_total count
    _rgb_led.msg_queue = xQueueCreate(led_action_total*2, sizeof(rgb_led_action_msg_t));
    if(_rgb_led.msg_queue == 0 )
    {
        // Queue was not created and must not be used.
        iprintln(trLED|trALWAYS, "#Unable to create Msg Queue for LED task");
        assert(0);
    }

    //We also need to create a timer for each LED (for blinking)
    _rgb_led.array = (rgb_led_t *)malloc(_rgb_led.cnt * sizeof(rgb_led_t));
    if (_rgb_led.array == NULL)
    {
        // Queue was not created and must not be used.
        iprintln(trLED|trALWAYS, "#Unable to create Timers for LED task");
        assert(0);
    }
    else
    {
        //Build the content of the array to match the LEDs
        for (int i = 0; i < _rgb_led.cnt; i++)
        {
            _rgb_led.array[i].col_1 = colBlack;
            _rgb_led.array[i].col_2 = colBlack;
            _rgb_led.array[i].state = led_state_off;
            sys_poll_tmr_stop(&_rgb_led.array[i].timer);
        }
        //memset(_rgb_led.array, 0, _rgb_led.cnt * sizeof(rgb_led_t));
    }

    _rgb_led.task.init_done = true;

    //Debugging - print contents of LED Array
    // for (int i = 0; i < _rgb_led.cnt; i++)
    //     _led_info_print(i);
    
    //Send a message to start the rainbow effect on the Debug LED only
    rgb_led_demo(BIT(led_bit_dbg), LED_RAINBOW_PERIOD_MS_DEF);

}

void _led_teardown(void)
{
    for (int _led = 0; _led < _rgb_led.cnt; _led++)
        sys_poll_tmr_stop(&_rgb_led.array[_led].timer);

    //memset(_rgb_led.array, 0, _rgb_led.cnt * sizeof(rgb_led_t));

    //Free the allocated memory
    if (_rgb_led.array != NULL)
    {
        free(_rgb_led.array);
        _rgb_led.array = NULL;
    }

    _rgb_led.cnt = 0;

    drv_rgb_led_strip_deinit();

}

void _read_msg_queue(void)
{     
    rgb_led_action_msg_t rx_msg;

    //Check if there is a message in the queue
    while (xQueueReceive(_rgb_led.msg_queue, &rx_msg, 0) == pdTRUE)
    {
        if (rx_msg.led_addr == 0)
        {
            iprintln(trLED, "#Empty address list msg rx'd in queue: action", rx_msg.cmd);
            continue;  //No LEDs are affected by this message
        }

        //We have a message, let's process it for every LED that it affects
        for (uint32_t _led_nr = 0; _led_nr < _rgb_led.cnt; _led_nr++)
        {

            if ((rx_msg.led_addr & BIT(_led_nr)) == 0)
                continue;   //This LED is not affected by the message

            //This LED is affected by the message
            switch (rx_msg.cmd)
            {
                case led_action_nop:
                    //Does nothing except use up a space in the msg queue
                    break;
                case led_action_off:
                    iprintln(trLED, "#%s -> OFF", drv_rgb_led_strip_index2name(_led_nr));
                    //Val_0, Val_1 and Val_2 are not used
                    _rgb_led.array[_led_nr].col_1 = 0;
                    drv_rgb_led_strip_set_colour(_led_nr, 0);
                    _rgb_led.array[_led_nr].state = led_state_off;
                    sys_poll_tmr_stop(&_rgb_led.array[_led_nr].timer);
                    break;
                case led_action_colour:
                    iprintln(trLED, "#%s -> ON (colour: %06X)", drv_rgb_led_strip_index2name(_led_nr), rx_msg.val_0);
                    //Val_0 is the colour
                    //Val_1 and Val_2 are not used
                    _rgb_led.array[_led_nr].col_1 = rx_msg.val_0;
                    drv_rgb_led_strip_set_colour(_led_nr, rx_msg.val_0);
                    _rgb_led.array[_led_nr].state = (rx_msg.val_0 == 0)? led_state_off : led_state_on; //If the colour is black, then the state is off, otherwise it is on
                    sys_poll_tmr_stop(&_rgb_led.array[_led_nr].timer);
                    break;
                case led_action_blink:            
                    iprintln(trLED, "#%s -> Set to Blink (%06X <-> %06X) at %dms", drv_rgb_led_strip_index2name(_led_nr), rx_msg.val_0, (rx_msg.val_2 & 0x00FFFFFF), rx_msg.val_1);
                    //Val_0 is the 1st (primary) colour
                    //Val_1 is the period (in ms)
                    //Val_2 is the 2nd/alternating  colour
                    //First make sure the blink timer period is good
                    // Since the task is running at 20ms intervals, we can only set the timer to multiples of 20ms
                    if ((rx_msg.val_1%LED_UPDATE_INTERVAL_MS) > 0)
                        rx_msg.val_1 = (LED_UPDATE_INTERVAL_MS * ((rx_msg.val_1/LED_UPDATE_INTERVAL_MS)+1));
                        // rx_msg.val_1 += (LED_UPDATE_INTERVAL_MS - (rx_msg.val_1%LED_UPDATE_INTERVAL_MS));

                    //Blinking faster than 10Hz is not really useful
                    if (rx_msg.val_1 < LED_BLINK_PERIOD_MS_MIN)
                        rx_msg.val_1 = LED_BLINK_PERIOD_MS_MIN;

                        //Start the timer and assign the primary colour
                    drv_rgb_led_strip_set_colour(_led_nr, rx_msg.val_0);
                    //Swop the primary and secondary colours for the next iteration
                    _rgb_led.array[_led_nr].col_1 = (rx_msg.val_2 & 0x00FFFFFF);
                    _rgb_led.array[_led_nr].col_2 = (rx_msg.val_0 & 0x00FFFFFF);
                    _rgb_led.array[_led_nr].state = led_state_blink;
                    sys_poll_tmr_start(&_rgb_led.array[_led_nr].timer, (uint64_t)rx_msg.val_1/2, true);
                    break;
                case led_action_rainbow:
                    iprintln(trLED, "#%s LED -> Rainbow at %dms", drv_rgb_led_strip_index2name(_led_nr), rx_msg.val_1);
                    //Val_0 is not used
                    //Val_1 is the period (in ms)
                    //Val_2 is not used
                    //First make sure the timer period is good
                    // Since the task is running at 20ms intervals, we can only set the timer to multiples of 20ms
                    if ((rx_msg.val_1%LED_UPDATE_INTERVAL_MS) > 0)
                        rx_msg.val_1 = (LED_UPDATE_INTERVAL_MS * ((rx_msg.val_1/LED_UPDATE_INTERVAL_MS)+1));
                        //rx_msg.val_1 += (LED_UPDATE_INTERVAL_MS - (rx_msg.val_1%LED_UPDATE_INTERVAL_MS));

                    //Blinking faster than 10Hz is not really useful
                    if (rx_msg.val_1 < LED_RAINBOW_PER_MIN_MS)
                        rx_msg.val_1 = LED_RAINBOW_PER_MIN_MS;
                    if (rx_msg.val_1 > LED_RAINBOW_PER_MAX_MS)
                        rx_msg.val_1 = LED_RAINBOW_PER_MAX_MS;
                    
                    drv_rgb_led_strip_set_colour(_led_nr, 0);
                    //The minimum timer value is 20ms (LED_UPDATE_INTERVAL_MS).
                    //Dividing the period by LED_UPDATE_INTERVAL_MS gives us a count of how many task cycles it takes to loop through the 360 degree hue spectrum,
                    //No need to start a timer... we use the task cycle interval to update the hue (if needed)
                    _rgb_led.array[_led_nr].col_1 = 0;
                    _rgb_led.array[_led_nr].col_2 = (rx_msg.val_1/LED_UPDATE_INTERVAL_MS) & 0xFFFF;
                    // iprintln(trLED, "#%s LED: Rainbow effect at %dms (%d x %d = %d)", 
                    //     drv_rgb_led_strip_index2name(_led_nr), rx_msg.val_1, _rgb_led.array[_led_nr].col_2, LED_UPDATE_INTERVAL_MS, _rgb_led.array[_led_nr].col_2 * LED_UPDATE_INTERVAL_MS);
                    _rgb_led.array[_led_nr].state = led_state_rainbow;
                    // sys_stopwatch_ms_start(&_rgb_led.sw);
                    sys_poll_tmr_stop(&_rgb_led.array[_led_nr].timer);
                    break;
                case led_action_status:
                {
                    _led_info_print(_led_nr);
                    if (_led_nr == (_rgb_led.cnt - 1))
                        iprintln(trALWAYS, "");
                    break;
                }
                default:
                    break;
            }
        }
    }
}

void _led_service(void)
{     
    static uint32_t hue = 0;

    for (int _led_nr = 0; _led_nr < _rgb_led.cnt; _led_nr++)
    {
        rgb_led_t * rgb_led = &_rgb_led.array[_led_nr];
        switch (rgb_led->state)
        {
            //We only really need to deal with the blink and rainbow states (timer based)
            case led_state_blink:
                if (sys_poll_tmr_expired(&rgb_led->timer))
                {
                    uint32_t _temp_col = rgb_led->col_1;
                    //Set the current colour                    
                    drv_rgb_led_strip_set_colour(_led_nr, rgb_led->col_1);
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
                rgb_led->col_1 = hue2rgb(hue);
                drv_rgb_led_strip_set_colour(_led_nr, rgb_led->col_1);
                //iprintln(trLED, "#Rainbow: %d/%d - H: %d -> 0x%06x)", _count, _total, hue, rgb_led->col_1);
                _count++;
                if (_count >= _total)
                {
                    // uint32_t _lap = sys_stopwatch_ms_reset(&_rgb_led.sw);
                    // iprintln(trLED, "#Rainbow (re)starting after %dms", _lap);
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

void _led_info_print(int _index)
{
    rgb_led_t * rgb_led = &_rgb_led.array[_index];
    switch (rgb_led->state)
    {
        case led_state_off:
            iprintln(trALWAYS, "%s LED - OFF", drv_rgb_led_strip_index2name(_index));
            break;
        case led_state_on:
            iprintln(trALWAYS, "%s LED - ON (%06X)", drv_rgb_led_strip_index2name(_index), rgb_led->col_1, rgb_led->state);
            break;
        case led_state_blink:
            //Remember, the timer period is half the blink period (on/off)
            iprintln(trALWAYS, "%s LED - Blinking (%06X <-> %06X), %.2f Hz", drv_rgb_led_strip_index2name(_index), rgb_led->col_1, rgb_led->col_2, (500.0f/rgb_led->timer.ms_period));
            break;
        case led_state_rainbow:
            //Remember, the bottom 16 bits of col_2 is the total count... x LED_UPDATE_INTERVAL_MS gives the period
            iprintln(trALWAYS, "%s LED - Rainbow, %dms", drv_rgb_led_strip_index2name(_index), (rgb_led->col_2 & 0xFFFF) * LED_UPDATE_INTERVAL_MS);
            break;
        default:
            iprintln(trALWAYS, "%s LED - UNKNOWN State (%d)", drv_rgb_led_strip_index2name(_index), rgb_led->state);
        break;
    }

}

esp_err_t _handler_parse_address_str(uint32_t * addr_mask, const char * str)
{
    uint32_t value;
    int index;
    int count;

    //Righto, the address can be in one of three valid formats:
    // 1. A single number (0 to 15)
    // 2. A hexadecimal number (0xHHHH), e.g. "0x003", "0x0F0", etc
    // 3. A string containing "Debug|Dbg|Button|Btn" optionally followed with a colon (:) and a number, e.g. "debug", "btn:1", etc
    
    if (str2int32((int32_t*)&value, str, 0))
    {
        //This would be a single led index
        if (value >= led_bit_max)
        {
            iprintln(trALWAYS, "Invalid LED Address (\"%s\" -> %d)", str, value);
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

    //The strip name parsing is handled by the driver
    if (drv_rgb_led_strip_name2index(str, &index, &count))
    {        
        //This could be a single led index or a whole mask
        for (int i = index; i < (index + count); i++)
            *addr_mask |= BIT(i);
        return ESP_OK;
    }

    //Reaching this point means this is NOT an LED address
    return ESP_ERR_NOT_FOUND;
}

void _led_handler_common_action(void/*rgb_led_action_cmd action*/)
{
    //These functions (_menu_handler...) are called from the console task, so they should 
    // not manupiluate the RGB task variables directly, but instead send messages to the RGB 
    // task via the msg_queue
    bool help_requested = false;
    rgb_led_action_msg_t led_msg = {.led_addr = 0, .val_0 = -1, .val_1 = 0, .val_2 = -1};//Indicating "not set yet"
    esp_err_t err = ESP_OK;
    uint32_t parse_value = 0;//addr_mask = 0;
    rgb_led_action_cmd action = led_action_nop;
    char *arg = console_arg_peek(-1);

    if (strcasecmp(arg, "off") == 0)
        action = led_action_off;
    else if (strcasecmp(arg, "on") == 0)
        action = led_action_colour;
    else if (strcasecmp(arg, "blink") == 0)
        action = led_action_blink;
    else if (strcasecmp(arg, "rainbow") == 0)
        action = led_action_rainbow;
    else
        return; //Nothing to do... (how did we get here?)

    while (console_arg_cnt() > 0)
	{
        char *arg = console_arg_pop();
        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from while-loop
        }
        parse_value = 0;

        //Check for period first - BLINK and RAINBOW
        if ((action == led_action_blink) || (action == led_action_rainbow))
        {
            if (str2uint32(&parse_value, arg, 0))
            {
                //This could be the period OR an address
                if (((parse_value >= LED_BLINK_PERIOD_MS_MIN*2) && (action == led_action_blink)) ||
                    ((parse_value >= LED_RAINBOW_PER_MIN_MS) && (action == led_action_rainbow)))
                {
                    //Ooooh... this could be a period
                    if (led_msg.val_1 != 0)
                        iprintln(trALWAYS, "Overwriting previously set period (%dms) with \"%s\" (%dms)", led_msg.val_1, arg, parse_value);
                    led_msg.val_1 = parse_value;
                    continue;
                }
            }
            //else not a good period value... maybe it is a colour or a LED address
        }

        //Then we check for colour(s) - ON and BLINK
        if ((action == led_action_colour) || (action == led_action_blink))
        {
            err = parse_str_to_colour(&parse_value, arg);
            if (err == ESP_OK)
            {
                if (action == led_action_colour)
                {
                    if (led_msg.val_0 != (uint32_t)-1)
                        iprintln(trALWAYS, "Overwriting previously set colour (%06X) with \"%s\" (%06X)", led_msg.val_0, arg, parse_value);
    
                    led_msg.val_0 = parse_value;
                }
                else if (action == led_action_blink)
                {
                    if (led_msg.val_0 == (uint32_t)-1)
                    {
                        led_msg.val_0 = parse_value;
                    }
                    else
                    {
                        if (led_msg.val_2 != (uint32_t)-1)
                            iprintln(trALWAYS, "Overwriting previously set colour (%06X) with \"%s\" (%06X)", led_msg.val_2, arg, parse_value);            
                        led_msg.val_2 = parse_value;
                    }
                }
                continue;
            }
            else if (err == ESP_ERR_INVALID_ARG)
            {
                //Looked like a Hue value, but it was invalid
                help_requested = true;
                break;
            }
        }

        //Then we check for LED addressing - ALL
        err = _handler_parse_address_str(&parse_value, arg);
        if (err == ESP_OK)
        {
            led_msg.led_addr |= parse_value;
            continue;
        }
        else if (err == ESP_ERR_INVALID_ARG)
        {
            //Looked like an address, but it was invalid
            help_requested = true;
            break;
        }
        //else if (err == ESP_ERR_NOT_FOUND) /// fall through to colour check
        iprintln(trALWAYS, "Invalid Argument (\"%s\")", arg);
        help_requested = true;
        break;
    }

    if (!help_requested)
    {
        if (action == led_action_colour)
        {
            if (led_msg.val_0 == (uint32_t)-1)
            {
                // No colour? We'll assign both using a very silly RNG based on the time since boot in us (mod 360)
                led_msg.val_0 = hue2rgb((uint32_t)(esp_random()%360L));
                iprintln(trALWAYS, "No Colour specified, using %06X", led_msg.val_0);
            }
        }
        else if (action == led_action_blink)
        {
            if (led_msg.val_0 == (uint32_t)-1)
            {
                // No colour? We'll assign both using a very silly RNG based on the time since boot in us (mod 360)
                uint32_t rand_hue = (uint32_t)(esp_random()%360L);
                led_msg.val_0 = hue2rgb(rand_hue);
                //The second colour is the complementary colour on the other side (+180 degrees) of the colour wheel
                led_msg.val_2 = hue2rgb(rand_hue+(HUE_MAX/2));
                iprintln(trALWAYS, "No Colours specified, using %06X and %06X", led_msg.val_0, led_msg.val_2);
            }
            else if (led_msg.val_2 == (uint32_t)-1)
            {
                uint32_t hue, sat, val;
                rgb2hsv(led_msg.val_0, &hue, &sat, &val);
                // No 2nd colour? We'll assign a complimentary colour from the one selected in val_0
                led_msg.val_2 = hsv2rgb(hue+(HUE_MAX/2), sat, val);
                iprintln(trALWAYS, "No Colour specified, using %06X", led_msg.val_2);
            }
        }        

        if ((action == led_action_blink) || (action == led_action_rainbow))
        {
            if (led_msg.val_1 == 0)
            {
                // No period? We'll assign one using a very silly RNG based on the time since boot in us (mod 360)
                led_msg.val_1 = (action == led_action_rainbow) ? LED_RAINBOW_PERIOD_MS_DEF : LED_BLINK_PERIOD_MS_DEF;
                iprintln(trALWAYS, "Using default period of %dms", led_msg.val_1);
            }
        }

        if (led_msg.led_addr == 0)
            led_msg.led_addr = BIT(0);             // No LED Address(es) specified... let's apply debug

        led_msg.cmd = action;
        xQueueSend(_rgb_led.msg_queue, &led_msg, 0);

        //Maybe a good idea to also queue a status request right after this?
        led_msg.cmd = led_action_status;
        xQueueSend(_rgb_led.msg_queue, &led_msg, 0);
    }

    if (help_requested)
    {
        //Opening line
        iprint(trALWAYS, "Usage: \"%s ", arg);        
        if (action == led_action_colour)    iprint(trALWAYS, "[<colour>] ");
        if ((action == led_action_rainbow) || (action == led_action_blink))   iprint(trALWAYS, "[<period>] ");
        if (action == led_action_blink)     iprint(trALWAYS, "[<colour_1>] [<colour_2>]");        
        iprintln(trALWAYS, " [<led_addr>]\"");


        // Describe "period"
        if ((action == led_action_blink) || (action == led_action_rainbow))
        {
            iprintln(trALWAYS, "    <period>:   A value indicating the period of the %s cycle in ms", arg);
            iprintln(trALWAYS, "                            (min: %dms)", (action == led_action_blink)? LED_BLINK_PERIOD_MS_DEF*2 : LED_RAINBOW_PER_MIN_MS);
            iprintln(trALWAYS, "        If <period> is omitted, a default period of %dms is used", (action == led_action_blink)? LED_BLINK_PERIOD_MS_DEF*2 : LED_RAINBOW_PER_MIN_MS);
        }

        // Describe "colour"
        if ((action == led_action_colour) || (action == led_action_blink))
        {
            iprintln(trALWAYS, "    <colour>:   String   - Any one of the assigned colour names");
            iprintln(trALWAYS, "                            e.g. \"Black\", \"wh\", etc");
            iprintln(trALWAYS, "                \"HSV:<csv>\" - An HSV string in the format \"HSV:<h>[,<s>[,<v>]]\"");
            iprintln(trALWAYS, "                            e.g. \"HSV:120\", \"HSV:180,50,50\", etc");
            iprintln(trALWAYS, "                  <h> must be in the range 0 to %d degrees", HUE_MAX-1);
            iprintln(trALWAYS, "                  If <s> or <v> is omitted, it will be set to 100%%");
            if (action == led_action_blink)
                iprintln(trALWAYS, "        If one or both <colour> are omitted, they will be selected at random");
            else
                iprintln(trALWAYS, "        If <colour> is omitted, one will be selected at random");
        }

        // Describe "led_addr"
        iprintln(trALWAYS, "    <led_addr>: <#>      - a single index (0 to %d), e.g. 0, 1, 2, 3, etc", _rgb_led.cnt-1);
        iprintln(trALWAYS, "                0xHHHH   - a 16 bit mask of the LEDs to affect, e.g. \"0x003\"");
        iprintln(trALWAYS, "                \"<strip>[:<nr>]\" - a string and number (seperated by a colon)");
        iprintln(trALWAYS, "                            indicating the LED strip and LED # to use");
        iprintln(trALWAYS, "                            e.g. \"debug\", \"button:1\", \"button:2\", etc");
        iprintln(trALWAYS, "                  If <nr> is omitted, %s applies to the entire strip", arg);
        iprintln(trALWAYS, "        If <led_addr> is omitted, \"debug\" is assumed");
        iprintln(trALWAYS, " Multiple <led_addr> values can be specified, separated by spaces");
    }
}

void _led_handler_status(void)
{
    //These functions (_menu_handler...) are called from the console task, so they should 
    // not manupiluate the RGB task variables directly, but instead send messages to the RGB 
    // task via the msg_queue
    bool help_requested = false;
    rgb_led_action_msg_t led_msg = {.led_addr = 0};//Indicating "not set yet"
    esp_err_t err = ESP_OK;
    uint32_t parse_value = 0;
//    uint32_t addr_mask = 0;

    while (console_arg_cnt() > 0)
	{
        char *arg = console_arg_pop();
        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from while-loop
        }
        parse_value = 0;
        err = _handler_parse_address_str(&parse_value, arg);
        if (err == ESP_OK)
        {
            led_msg.led_addr |= parse_value;
            continue;
        }
        if (err == ESP_ERR_NOT_FOUND)   //ESP_ERR_INVALID_ARG has already been handled ito user feedback
            iprintln(trALWAYS, "Invalid Argument (\"%s\")", arg);
        // help_requested = true;
        break;
    }

    if ((!help_requested))
    {
        if (led_msg.led_addr == 0)
            led_msg.led_addr = BIT(0);             // No LED Address(es) specified... let's apply debug

        led_msg.cmd = led_action_status;
        // for (int i = 0; i < _rgb_led.cnt; i++)
        //     led_msg.led_addr |= BIT(i);
        xQueueSend(_rgb_led.msg_queue, &led_msg, 0);
    
    }
    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        iprintln(trALWAYS, "Usage: \"ledstat [<led_addr>]\"");        
        iprintln(trALWAYS, "    <led_addr>: <#>      - a single index (0 to %d), e.g. 0, 1, 2, 3, etc", _rgb_led.cnt-1);
        iprintln(trALWAYS, "                0xHHHH   - a 16 bit mask of the LEDs to affect, e.g. \"0x003\"");
        iprintln(trALWAYS, "                \"<strip>[:<nr>]\" - a string and number (seperated by a colon)");
        iprintln(trALWAYS, "                            indicating the LED strip and LED # to use");
        iprintln(trALWAYS, "                            e.g. \"debug\", \"button:1\", \"button:2\", etc");
        iprintln(trALWAYS, "                  If <nr> is omitted, %s applies to the entire strip", "STATUS");
        iprintln(trALWAYS, "        If <led_addr> is omitted, \"debug\" is assumed");
        iprintln(trALWAYS, " Multiple <led_addr> values can be specified, separated by spaces");
        //        iprintln(trALWAYS, "   e.g. \"off 0 1 2 3\", \"off debug:0 button:1...\" etc");
    }
}

void _led_handler_col_list(void)
{
    bool help_requested = false;
    bool show_all = false;
    bool show_hues = false;
    // uint32_t col_list_size = 0;
    uint32_t parse_value = 0;
    esp_err_t err = ESP_OK;

    // if (console_arg_cnt() > 0)
    // {
    //     //Let's get an idea of how many colours we need to list
    //     while (colour_list_item(col_list_size) != NULL)
    //         col_list_size++;
    // }
    if (console_arg_cnt() == 0)
    {
        //No arguments... let's list all the colours
        show_all = true;
        help_requested = true;

    }

    //First check that all arguments are valid
    for (int i = 0; i < console_arg_cnt(); i++)
    {
        char *arg = console_arg_peek(i);
        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from for-loop
        }
        if ((!strcasecmp("all", arg)) || (!strcasecmp("list", arg)))
        {    
            show_all = true;
        }
        if (!strcasecmp("hues", arg))
        {    
            show_hues = true;
        }
        // if (hex2u32(&rgb_col_24bit, str, 6))
        // {
        // }
        err = parse_str_to_colour(&parse_value, arg);
        if (err == ESP_ERR_NOT_FOUND)
        {
            //No a parsable colour
            iprintln(trALWAYS, "Invalid Argument (\"%s\")", arg);
            help_requested = true;
            break;
        }

    }

    if (show_hues)
    {
        iprintln(trALWAYS, "The RGB value for all 359 HUEs (100%% Sat and Val) are:");
        for (int i = 0; i < HUE_MAX; i++)
        {
            const char * col_name = NULL;
            uint32_t rgb_val = 0;
            uint32_t hue, sat, val;
            rgb_val = hue2rgb(i);
            rgb2hsv(rgb_val, &hue, &sat, &val);
            iprint(trALWAYS, " Hue: % 3d -> %06X", i, rgb_val);
            col_name = rgb2name(rgb_val);
            // iprint(trALWAYS, " - Hue: %3d", hue);
            if (col_name != NULL)
                iprint(trALWAYS, " (%s)", col_name);
            iprintln(trALWAYS, "");
        }
        return;
    }
    
    if (show_all)
    {
        iprintln(trALWAYS, "The RGB and HSV values for the named colours are:");
        int i = 0;
        const char *col_name = colour_list_item(i);
        while (col_name != NULL)
        {
            uint32_t rgb_val;
            if (str2rgb(&rgb_val, col_name) != ESP_OK)
                iprintln(trALWAYS, " % 2d: %s -> Invalid Colour", i, col_name); //Should never happen
            else
            {
                uint32_t hue, sat, val;
                rgb2hsv(rgb_val, &hue, &sat, &val);
                iprintln(trALWAYS, " % 8s -> %06X - HSV: %3d, %3d, %3d", col_name, rgb_val, hue, sat, val);
            }
            col_name = colour_list_item(++i);
        }
        return;
    }
    if (!help_requested)
    {
        //All the arguments are valid... let's list each one of them 
        //There are two options... 
        // 1) the used provides a Colour name (to get the RGB value)
        // 2) the user provides a Hue value (to get the RGB value)
        while (console_arg_cnt() > 0)
        {
            char *arg = console_arg_pop();
            if ((!strcasecmp("all", arg)) || (!strcasecmp("list", arg)))
                continue;   //Skip... already handled
            if (!strcasecmp("hues", arg))
                continue;   //Skip... already handled
            if (ESP_OK == parse_str_to_colour(&parse_value, arg))
            {
                iprint(trALWAYS, " % 8s -> %06X", arg, parse_value);
                const char *col_name = rgb2name(parse_value);
                if (col_name != NULL)
                    iprint(trALWAYS, " (%s)", col_name);
                iprintln(trALWAYS, "");
            }
        }
        return;
    }

    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        iprintln(trALWAYS, "Usage: \"rgb [<colour>]\"");        
        iprintln(trALWAYS, "    <colour>:   String   - Any one of the assigned colour names");
        iprintln(trALWAYS, "                            e.g. \"Black\", \"wh\", etc");
        iprintln(trALWAYS, "                \"HSV:<csv>\" - An HSV string in the format \"HSV:<h>[,<s>[,<v>]]\"");
        iprintln(trALWAYS, "                            e.g. \"HSV:120\", \"HSV:180,50,50\", etc");
        iprintln(trALWAYS, "                  <h> must be in the range 0 to %d degrees", HUE_MAX-1);
        iprintln(trALWAYS, "                  If <s> or <v> is omitted, it will be set to 100%%");
        iprintln(trALWAYS, "                \"all|list\" - Lists ALL the available named colours");
        iprintln(trALWAYS, "                \"hues\" - Lists the RGB values for all 359 HUE values");
        iprintln(trALWAYS, "        If <colour> is omitted \"rgb all\" is assumed");
        iprintln(trALWAYS, " Multiple <colour> values can be specified, separated by spaces");
    }
}

void _led_handler_reset(void)
{
    //These functions (_menu_handler...) are called from the console task, so they should 
    // not manupiluate the RGB task variables directly, but instead send messages to the RGB 
    // task via the msg_queue
    bool help_requested = false;
    rgb_led_action_msg_t led_msg = {.led_addr = 0, .val_1 = 0};//Indicating "not set yet"
    esp_err_t err = ESP_OK;
    uint32_t parse_value = 0;//addr_mask = 0;

    while (console_arg_cnt() > 0)
	{
        char *arg = console_arg_pop();
        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from while-loop
        }
        parse_value = 0;
        //Check for LED addressing
        err = _handler_parse_address_str(&parse_value, arg);
        if (err == ESP_OK)
        {
            led_msg.led_addr |= parse_value;
            continue;
        }
        else if (err == ESP_ERR_INVALID_ARG)
        {
            //Looked like an address, but it was invalid
            help_requested = true;
            break;
        }
        //else if (err == ESP_ERR_NOT_FOUND) /// fall through to colour check
        iprintln(trALWAYS, "Invalid Argument (\"%s\")", arg);
        help_requested = true;
        break;
    }

    if (!help_requested)
    {
        if (led_msg.led_addr == 0)
            led_msg.led_addr = BIT(0);             // No LED Address(es) specified... let's apply debug

        led_msg.val_1 = LED_RAINBOW_PERIOD_MS_DEF;
        led_msg.cmd = led_action_rainbow;
        xQueueSend(_rgb_led.msg_queue, &led_msg, 0);
        //Maybe a good idea to also queue a status request right after this?
        led_msg.cmd = led_action_status;
        // for (int i = 0; i < _rgb_led.cnt; i++)
        //     led_msg.led_addr |= BIT(i);
        xQueueSend(_rgb_led.msg_queue, &led_msg, 0);
    }

    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        iprintln(trALWAYS, "Usage: \"reset [<led_addr>]\"");        
        iprintln(trALWAYS, "    <led_addr>: <#>      - a single index (0 to %d), e.g. 0, 1, 2, 3, etc", _rgb_led.cnt-1);
        iprintln(trALWAYS, "                0xHHHH   - a 16 bit mask of the LEDs to affect, e.g. \"0x003\"");
        iprintln(trALWAYS, "                \"<strip>[:<nr>]\" - a string and number (seperated by a colon)");
        iprintln(trALWAYS, "                            indicating the LED strip and LED # to use");
        iprintln(trALWAYS, "                            e.g. \"debug\", \"button:1\", \"button:2\", etc");
        iprintln(trALWAYS, "                  If <nr> is omitted, %s applies to the entire strip", "RESET");
        iprintln(trALWAYS, "        If <led_addr> is omitted, \"debug\" is assumed");
        iprintln(trALWAYS, " Multiple <led_addr> values can be specified, separated by spaces");
//        iprintln(trALWAYS, "   e.g. \"off 0 1 2 3\", \"off debug:0 button:1...\" etc");
    }
}
/*******************************************************************************
Global (public) Functions
*******************************************************************************/
void * rgb_led_init_task(void)
{
	//Let's not re-initialise this task by accident
	if (_rgb_led.task.handle)
		return &_rgb_led.task;

	// Create the task, storing the handle.  Note that the passed parameter ucParameterToPass
	// must exist for the lifetime of the task, so in this case is declared static.  If it was just an
	// an automatic stack variable it might no longer exist, or at least have been corrupted, by the time
	// the new task attempts to access it.
	if (xTaskCreate( _led_main_func, PRINTF_TAG, _rgb_led.task.stack_depth, _rgb_led.task.parameter_to_pass, 1, &_rgb_led.task.handle ) != pdPASS)
	{
		iprintln(trALWAYS, "#Unable to start %s Task!", PRINTF_TAG);
		return NULL;
	}

	configASSERT(_rgb_led.task.handle);

    _led_setup();

    return &_rgb_led.task;
}

esp_err_t rgb_led_off(uint16_t address_mask)
{
    //This function can be called from ANY task
    rgb_led_action_msg_t led_msg = {
        .led_addr = 0, 
        .cmd = led_action_nop, 
        .val_0 = -1, 
    };//Indicating "not set yet"

    if (!_rgb_led.task.init_done)
        return ESP_ERR_INVALID_STATE;

    if (address_mask == 0)
        return ESP_OK;  //Technically not an error... but nothing to do

    for (int i = 0; i < _rgb_led.cnt; i++)
        led_msg.led_addr |= (BIT(i) & address_mask);

    led_msg.val_0 = colBlack;
    led_msg.cmd = led_action_off;
    if (xQueueSend(_rgb_led.msg_queue, &led_msg, 0) == pdTRUE)
        return ESP_OK;

    iprintln(trLED, "#Failed to queue message (%s)", __FUNCTION__);
    return ESP_FAIL;
}

esp_err_t rgb_led_on(uint16_t address_mask, uint32_t rgb_colour)
{
    //This function can be called from ANY task
    rgb_led_action_msg_t led_msg = {
        .led_addr = 0, 
        .cmd = led_action_nop, 
        .val_0 = -1, 
    };//Indicating "not set yet"

    if (!_rgb_led.task.init_done)
        return ESP_ERR_INVALID_STATE;

    if (address_mask == 0)
        return ESP_OK;  //Technically not an error... but nothing to do

    if (rgb_colour == (uint32_t)-1)
    {
        rgb_colour = hue2rgb((uint32_t)(esp_random()%360L));
    }
    
    if (rgb_colour > RGB_MAX)
    {
        iprintln(trLED, "#Invalid RGB value (%d: 0x%08X)", 1, rgb_colour);
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < _rgb_led.cnt; i++)
        led_msg.led_addr |= (BIT(i) & address_mask);

    led_msg.val_0 = rgb_colour;
    led_msg.cmd = (rgb_colour == colBlack)? led_action_off : led_action_colour;
    if (xQueueSend(_rgb_led.msg_queue, &led_msg, 0) == pdTRUE)
        return ESP_OK;

    iprintln(trLED, "#Failed to queue message (%s)", __FUNCTION__);
    return ESP_FAIL;
}

esp_err_t rgb_led_blink(uint16_t address_mask, uint32_t period, uint32_t rgb_colour_1, uint32_t rgb_colour_2)
{
    rgb_led_action_msg_t led_msg = {
        .led_addr = 0, 
        .cmd = led_action_nop, 
        .val_0 = -1, 
        .val_1 = 0, 
        .val_2 = -1
    };//Indicating "not set yet"

    if (!_rgb_led.task.init_done)
        return ESP_ERR_INVALID_STATE;

    if (address_mask == 0)
        return ESP_OK;  //Technically not an error... but nothing to do

    if (period < (LED_BLINK_PERIOD_MS_MIN*2))
    {
        iprintln(trLED, "#Min blink period = %dms (got %dms)", (LED_BLINK_PERIOD_MS_MIN*2), period);
        return ESP_ERR_INVALID_ARG;
    }

    if (rgb_colour_1 == (uint32_t)-1)
    {
        rgb_colour_1 = hue2rgb((uint32_t)(esp_random()%360L));
    }
    
    if (rgb_colour_1 > RGB_MAX)
    {
        iprintln(trLED, "#Invalid RGB value (%d: 0x%08X)", 1, rgb_colour_1);
        return ESP_ERR_INVALID_ARG;
    }

    if (rgb_colour_2 == (uint32_t)-1)
    {
        uint32_t hue, sat, val;
        //get the first colour's hue value from the RGB value
        rgb2hsv(rgb_colour_1, &hue, &sat, &val);
        //The complementary colour sits on the other side (+180 degrees) of the colour wheel
        hue = (hue + (HUE_MAX/2)) % HUE_MAX;
        //We retain the same satuaration and value
        rgb_colour_2 = hsv2rgb(hue, sat, val);
    }

    if (rgb_colour_2 > RGB_MAX)
    {
        iprintln(trLED, "#Invalid RGB value (%d: 0x%08X)", 2, rgb_colour_2);
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < _rgb_led.cnt; i++)
        led_msg.led_addr |= (BIT(i) & address_mask);

    led_msg.val_0 = rgb_colour_1;
    led_msg.val_1 = period;
    led_msg.val_2 = rgb_colour_2;
    led_msg.cmd = led_action_blink;
    if (xQueueSend(_rgb_led.msg_queue, &led_msg, 0) == pdTRUE)
        return ESP_OK;

    iprintln(trLED, "#Failed to queue message (%s)", __FUNCTION__);
    return ESP_FAIL;
}

esp_err_t rgb_led_demo(uint16_t address_mask, uint32_t period)
{
    if (!_rgb_led.task.init_done)
        return ESP_ERR_INVALID_STATE;

    rgb_led_action_msg_t led_msg = {
        .led_addr = 0, 
        .cmd = led_action_nop, 
        .val_1 = 0, 
    };//Indicating "not set yet"

    if (address_mask == 0)
        return ESP_OK;  //Technically not an error... but nothing to do

    if (period < (LED_BLINK_PERIOD_MS_MIN*2))
    {
        iprintln(trLED, "#Min blink period = %dms (got %dms)", (LED_BLINK_PERIOD_MS_MIN*2), period);
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < _rgb_led.cnt; i++)
        led_msg.led_addr |= (BIT(i) & address_mask);

    led_msg.val_1 = period;
    led_msg.cmd = led_action_rainbow;
    if (xQueueSend(_rgb_led.msg_queue, &led_msg, 0) == pdTRUE)
        return ESP_OK;

    iprintln(trLED, "#Failed to queue message (%s)", __FUNCTION__);
    return ESP_FAIL;
}

#undef PRINTF_TAG

/*************************** END OF FILE *************************************/
