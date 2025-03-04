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
#include "str_helper.h"
#include "task_console.h"
#include "drv_rgb_led_strip.h"

#define __NOT_EXTERN__
#include "task_rgb_led.h"
#undef __NOT_EXTERN__

/*******************************************************************************
Macros and Constants
 *******************************************************************************/

#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("RGB_LED") /* This must be undefined at the end of the file*/

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

typedef struct {
    const char * name;
    rgb_led_strip_type type;
} _led_strip_name_t;

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
    int cnt;
    rgb_led_t * array;    
	TaskInfo_t task;
    QueueHandle_t msg_queue;

    //Stopwatch_ms_t sw;  /* used for debugging */

} DeviceRgb_t;

//We need to create a message queue for the RGB LED task
//static QueueHandle_t _queue;


/*******************************************************************************
local function prototypes
 *******************************************************************************/
/*! @brief Initialises the RGB LED task
 */
void _init(void);

/*! @brief Deinitialises the RGB LED task
 */
void _deinit(void);

/*! @brief Displays information about a specific LED (Only for debug purposes)
 * @param _index The index of the LED to display information about
 */
void _info(int _index);

/*! @brief Reads the message queue for the RGB LED task
 */
void _read_msg_queue(void);

/*! @brief The main service() function for the RGB LED task
 */
void _led_service(void);

/*! @brief Sets the colour of the LED at the specified index
 * @param _index The index of the LED to set the colour for
 * @param _rgb The 24-bit RGB value to set the LED to
 */
void _set_colour(rgb_led_addr_t _index, uint32_t _rgb);

/*! @brief Converts the LED address to a bit index for LED addressing
 * @param _type The type of LED strip
 * @param _led_nr The LED number
 * @param _index Pointer containing the address value if successful
 * @return true if successful, otherwise false
 */
bool _led_addr_to_bit_index(rgb_led_strip_type _type, uint32_t _led_nr, rgb_led_addr_t * _index);

/*! @brief Converts the bit index to the LED type and address
 * @param _index The bit index of the LED address
 * @param _type Pointer to the LED strip type if successful
 * @param _led_nr Pointer to the LED number if successful
 * @param led_name Optional pointer to the name of the LED if successful
 * @return true if successful, otherwise false
 */
bool _led_index2type_nr(rgb_led_addr_t _index, rgb_led_strip_type * _type, uint32_t * _led_nr, char * led_name);

/*! @brief Converts the bit index to the LED name
 * @param _index The bit index of the LED address
 * @return A pointer to the name of the LED
 */
char * _led_index2name(rgb_led_addr_t _index);

/*! @brief Parses the arguments in the string and converts it to set bits in the address mask
 * @param[out] addr_mask The address mask if successful, otherwise unchanged
 * @param[in] arg_str The string containing the arguments to parse
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t _handler_parse_address_str(uint32_t * addr_mask, const char * str);

/*! @brief Parses the arguments in the string and converts it to a 24-bit RGB value
 * @param[out] colour_value The 24-bit RGB value if successful, otherwise unchanged
 * @param[in] str The string containing the arguments to parse, either the name 
 *  of the colour (see colour.c) an HSV formatted string
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t _handler_parse_colour(uint32_t * colour_value, const char * str);

/*! @brief Parses the arguments in the string into H, S or V values
 * @param[in] str The string containing the arguments to parse (in the 
 *  format  "HSV:<H>[,<S>[,<V>]]")
 * @param[out] value The 24-bit RGB value if successful or an empty string 
 *  was received, otherwise unchanged
 * @param[in] limit The maximum value allowed for the element (H=360, S=V=100)
 * @param[in] type_str The type of element being parsed
 * @return ESP_OK if successful (or an empty string was received), otherwise an error code
 */
esp_err_t _handler_parse_colour_hsv(char * str, uint32_t *value, uint32_t limit, const char * type_str);

/*! @brief Console menu handler to turn LEDs OFF
 */
void _led_handler_off(void);

/*! @brief Console menu handler to turn LEDs ON
 */
void _led_handler_col(void);

/*! @brief Console menu handler to blink LEDs
 */
void _led_handler_blink(void);

/*! @brief Console menu handler to turn demo mode on LEDs on
 */
void _led_handler_rainbow(void);

/*! @brief Console menu handler to display the RGB hex values
 */
void _led_handler_col_list(void);

/*! @brief Console menu handler to display the status of LEDs
 */
void _led_handler_status(void);

/*! @brief Console menu handler to reset the state of the LEDs
 */
void _led_handler_reset(void);

/*! @brief Common action handler for LED actions
 */
void _led_handler_common_action(void);


/*******************************************************************************
local variables
 *******************************************************************************/
const _led_strip_name_t _led_strips[] =
{
    {"Debug",   strip_Debug},
    {"Dbg",     strip_Debug},
    {"Buttons", strip_Buttons},
    {"Button",  strip_Buttons},
    {"Btns",    strip_Buttons},
    {"Btn",     strip_Buttons},
};

ConsoleMenuItem_t _led_menu_items[] =
{
									        // 01234567890123456789012345678901234567890123456789012345678901234567890123456789
        {"off",     _led_handler_off,      "Turns off LEDs"},
        {"on",      _led_handler_col,      "Sets LEDs to a specific colour"},
        {"blink",   _led_handler_blink,    "Blinks LEDs at a specified rate and colour(s)"},
        {"rainbow", _led_handler_rainbow,  "Apply rainbow effect on LEDs"},
        {"rgb",     _led_handler_col_list, "Displays the RGB hex values for predefined colours"},
		{"status",  _led_handler_status,   "Displays the status of the LEDs"},
		{"reset",   _led_handler_reset,    "Resets the state of the LEDs"},
};

static DeviceRgb_t _rgb_led = {
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

void _init(void)
{
    rgb_led_action_msg_t led_msg;

    _rgb_led.cnt = drv_rgb_led_strip_init();

    //dbgPrint(trRGB|trALWAYS, "#RGB LED Task init done (%d LEDs in total)", _rgb_led.cnt);

#ifdef CONSOLE_ENABLED
    task_console_add_menu("led", _led_menu_items, ARRAY_SIZE(_led_menu_items), "RGB LED Control");
#endif

    //We need to create a message queue for the RGB LED task, let's make it double to the total led_action_total count
    _rgb_led.msg_queue = xQueueCreate(led_action_total*2, sizeof(rgb_led_action_msg_t));
    if(_rgb_led.msg_queue == 0 )
    {
        // Queue was not created and must not be used.
        dbgPrint(trRGB|trALWAYS, "#Unable to create Msg Queue for RGB LED task");
        assert(0);
    }

    //We also need to create a timer for each RGB LED (for blinking)
    _rgb_led.array = (rgb_led_t *)malloc(_rgb_led.cnt * sizeof(rgb_led_t));
    if (_rgb_led.array == NULL)
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
            if (array_index >= _rgb_led.cnt)
            {
                dbgPrint(trRGB|trALWAYS, "#%s() - Array Indexing out of bounds (%d/%d)", __FUNCTION__, array_index, _rgb_led.cnt);
                //This is a bit of a problem
                break;
            }

            int _type_led_cnt = drv_rgb_led_strip_count(_type);
            for (int _addr = 0; _addr < _type_led_cnt; _addr++)
            {
                _rgb_led.array[array_index].col_1 = colBlack;
                _rgb_led.array[array_index].col_2 = colBlack;
                _rgb_led.array[array_index].state = led_state_off;      //All Leds start in OFF state
                sys_poll_tmr_stop(&_rgb_led.array[array_index].timer);  //Just in case
                array_index++;
            }
        }
        //memset(_rgb_led.array, 0, _rgb_led.cnt * sizeof(rgb_led_t));
    }

    //Debugging - print contents of LED Array
    // for (int i = 0; i < _rgb_led.cnt; i++)
    //     _info(i);
    
    //Send a message to start the rainbow effect on the Debug LED only
    led_msg.cmd = led_action_rainbow;
    led_msg.led_addr = BIT(led_bit_dbg);
    led_msg.val_0 = colBlack;   //Black
    led_msg.val_1 = RGB_LED_RAINBOW_PERIOD_MS_DEF;  // 7200ms
    led_msg.val_2 = 0;

    //Add this message to the queue
    xQueueSend(_rgb_led.msg_queue, &led_msg, 0);
}

void _info(int _index)
{
    rgb_led_t * rgb_led = &_rgb_led.array[_index];
    rgb_led_strip_type _strip;
    uint32_t _addr;
    char _led_name[16]; 

    if ((_index >= _rgb_led.cnt) || (_index > led_bit_max))
    {
        dbgPrint(trRGB, "#RGB LED #%d Index out of bounds (%d/%d)", _index, _index, _rgb_led.cnt);
        return;
    }

    if (_led_index2type_nr(_index, &_strip, &_addr, _led_name))
        dbgPrint(trRGB, "#%s LED (%d) Info: Type %d, Address %d, Colour %06X, State %d", _led_name, _index, _strip, _addr, rgb_led->col_1, rgb_led->state);

}

void _deinit(void)
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

    drv_rgb_led_strip_deinit();
}

void _led_task(void * pvParameters)
{
    // uint16_t hue = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    _init();

    dbgPrint(trRGB|trALWAYS, "#RGB LED Task Started (%d LEDs) @ %d Hz", _rgb_led.cnt, (1000/RGB_LED_UPDATE_INTERVAL));

    while (1) 
    {

        _read_msg_queue();
        
        _led_service();

        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(RGB_LED_UPDATE_INTERVAL));  //vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
    
        /* Inspect our own high water mark on entering the task. */
    	_rgb_led.task.stack_unused = uxTaskGetStackHighWaterMark2( NULL );
    }
    
    _deinit();

}

void _read_msg_queue(void)
{     
    rgb_led_action_msg_t rx_msg;

    //Check if there is a message in the queue
    while (xQueueReceive(_rgb_led.msg_queue, &rx_msg, 0) == pdTRUE)
    {
        if (rx_msg.led_addr == 0)
        {
            dbgPrint(trRGB, "#Empty address list msg rx'd in queue: action", rx_msg.cmd);
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
                    dbgPrint(trRGB, "#%s -> OFF", _led_index2name(_led_nr));
                    //Val_0, Val_1 and Val_2 are not used
                    _rgb_led.array[_led_nr].col_1 = 0;
                    _set_colour(_led_nr, 0);
                    _rgb_led.array[_led_nr].state = led_state_off;
                    sys_poll_tmr_stop(&_rgb_led.array[_led_nr].timer);
                    break;
                case led_action_colour:
                    dbgPrint(trRGB, "#%s -> ON (colour: 0x%06X)", _led_index2name(_led_nr), rx_msg.val_0);
                    //Val_0 is the colour
                    //Val_1 and Val_2 are not used
                    _rgb_led.array[_led_nr].col_1 = rx_msg.val_0;
                    _set_colour(_led_nr, rx_msg.val_0);
                    _rgb_led.array[_led_nr].state = (rx_msg.val_0 == 0)? led_state_off : led_state_on; //If the colour is black, then the state is off, otherwise it is on
                    sys_poll_tmr_stop(&_rgb_led.array[_led_nr].timer);
                    break;
                case led_action_blink:            
                    dbgPrint(trRGB, "#%s -> Set to Blink (0x%06X <-> 0x%06X) at %dms", _led_index2name(_led_nr), rx_msg.val_0, (rx_msg.val_2 & 0x00FFFFFF), rx_msg.val_1);
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
                    _set_colour(_led_nr, rx_msg.val_0);
                    //Swop the primary and secondary colours for the next iteration
                    _rgb_led.array[_led_nr].col_1 = (rx_msg.val_2 & 0x00FFFFFF);
                    _rgb_led.array[_led_nr].col_2 = (rx_msg.val_0 & 0x00FFFFFF);
                    _rgb_led.array[_led_nr].state = led_state_blink;
                    sys_poll_tmr_start(&_rgb_led.array[_led_nr].timer, (uint64_t)rx_msg.val_1/2, true);
                    break;
                case led_action_rainbow:
                    dbgPrint(trRGB, "#%s LED -> Rainbow at %dms", _led_index2name(_led_nr), rx_msg.val_1);
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
                    
                    _set_colour(_led_nr, 0);
                    //The minimum timer value is 20ms (RGB_LED_UPDATE_INTERVAL).
                    //Dividing the period by RGB_LED_UPDATE_INTERVAL gives us a count of how many task cycles it takes to loop through the 360 degree hue spectrum,
                    //No need to start a timer... we use the task cycle interval to update the hue (if needed)
                    _rgb_led.array[_led_nr].col_1 = 0;
                    _rgb_led.array[_led_nr].col_2 = (rx_msg.val_1/RGB_LED_UPDATE_INTERVAL) & 0xFFFF;
                    // dbgPrint(trRGB, "#%s LED: Rainbow effect at %dms (%d x %d = %d)", 
                    //     _led_index2name(_led_nr), rx_msg.val_1, _rgb_led.array[_led_nr].col_2, RGB_LED_UPDATE_INTERVAL, _rgb_led.array[_led_nr].col_2 * RGB_LED_UPDATE_INTERVAL);
                    _rgb_led.array[_led_nr].state = led_state_rainbow;
                    // sys_stopwatch_ms_start(&_rgb_led.sw);
                    sys_poll_tmr_stop(&_rgb_led.array[_led_nr].timer);
                    break;
                case led_action_status:
                {
                    switch (_rgb_led.array[_led_nr].state)
                    {
                        case led_state_off:
                            dbgPrint(trALWAYS, "%s LED - OFF", _led_index2name(_led_nr));
                            break;
                        case led_state_on:
                            dbgPrint(trALWAYS, "%s LED - ON (0x%06X)", _led_index2name(_led_nr), _rgb_led.array[_led_nr].col_1, _rgb_led.array[_led_nr].state);
                            break;
                        case led_state_blink:
                            //Remember, the timer period is half the blink period (on/off)
                            dbgPrint(trALWAYS, "%s LED - Blinking (0x%06X <-> 0x%06X), %.2f Hz", _led_index2name(_led_nr), _rgb_led.array[_led_nr].col_1, _rgb_led.array[_led_nr].col_2, (500.0f/_rgb_led.array[_led_nr].timer.ms_period));
                            break;
                        case led_state_rainbow:
                            //Remember, the bottom 16 bits of col_2 is the total count.... x RGB_LED_UPDATE_INTERVAL gives the period
                            dbgPrint(trALWAYS, "%s LED - Rainbow, %dms", _led_index2name(_led_nr), (_rgb_led.array[_led_nr].col_2 & 0xFFFF) * RGB_LED_UPDATE_INTERVAL);
                            break;
                        default:
                            dbgPrint(trALWAYS, "%s LED - UNKNOWN State (%d)", _led_index2name(_led_nr), _rgb_led.array[_led_nr].state);
                        break;
                    }
                    if (_led_nr == (_rgb_led.cnt - 1))
                        dbgPrint(trALWAYS, "");
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
                    _set_colour(_led_nr, rgb_led->col_1);
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
                _set_colour(_led_nr, rgb_led->col_1);
                //dbgPrint(trRGB, "#Rainbow: %d/%d - H: %d -> 0x%06x)", _count, _total, hue, rgb_led->col_1);
                _count++;
                if (_count >= _total)
                {
                    // uint32_t _lap = sys_stopwatch_ms_reset(&_rgb_led.sw);
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

void _set_colour(rgb_led_addr_t _index, uint32_t _rgb)
{
    rgb_led_strip_type _strip;
    uint32_t _addr;

    if (_led_index2type_nr(_index, &_strip, &_addr, NULL))
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

bool _led_index2type_nr(rgb_led_addr_t _index, rgb_led_strip_type * _type, uint32_t * _led_nr, char * led_name /* Optional */)
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
        strcpy(led_name, _led_index2name(_index));

    return true;
}

char * _led_index2name(rgb_led_addr_t _index)
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

esp_err_t _handler_parse_address_str(uint32_t * addr_mask, const char * str)
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
    for (int i = 0; i < ARRAY_SIZE(_led_strips); i++)
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

esp_err_t _handler_parse_colour(uint32_t * colour_value, const char * str)
{
    // uint32_t rgb_col_24bit;
    uint32_t hue_value = HUE_MAX, sat_value = SAT_MAX, val_value = VAL_MAX;
    const char * hue_str = NULL;
    char * sat_str = NULL;
    char * val_str = NULL;
    esp_err_t err = ESP_OK;

    //Righto, a colour can be in one of two valid formats:
    // 1. NOT USED: A 6-character hexadecimal string (0xHHHHHH), e.g. "0x000001", "#0F0F0F", etc
    // 2. A string containing any one of the assigned colour names, e.g. "Black", "wh", etc
    // 3. A HSV string degrees provided as HSV:<h>[,<s>[,<v>]]"

    if (str2rgb(colour_value, str) == ESP_OK)
        return ESP_OK;

    if (strncasecmp("HSV:", str, 4) == 0)
    {
        hue_str = str+4;
        sat_str = strchr(hue_str, ',');
        if (sat_str != NULL)
        {
            *sat_str = 0;
            sat_str++;
            val_str = strchr(sat_str, ',');
            if (val_str != NULL)
            {
                *val_str = 0;
                val_str++;
            }
        }
        
        err = _handler_parse_colour_hsv((char *)hue_str, &hue_value, HUE_MAX, "Hue");
        if (err != ESP_OK)
            return err;
        err = _handler_parse_colour_hsv(sat_str, &sat_value, SAT_MAX, "Saturation %%");
        if (err != ESP_OK)
            return err;
        err = _handler_parse_colour_hsv(val_str, &val_value, VAL_MAX, "Value %%");
        if (err != ESP_OK)
            return err;

        hsv2rgb(hue_value, sat_value, val_value, colour_value);
        return ESP_OK;
    }

    //Reaching this point means this is NOT a hex RGB colour or an HSV value
    return ESP_ERR_NOT_FOUND;
}

esp_err_t _handler_parse_colour_hsv(char * str, uint32_t *value, uint32_t limit, const char * type_str)
{
    if (value == NULL)
    {
        dbgPrint(trALWAYS, "Invalid dst ptr passed (NULL) - %s", type_str);
        return ESP_ERR_INVALID_ARG;
    }

    //Special case: if no string pointer is passed, then we return OK, but change nothing.
    if (str == NULL)
        return ESP_OK;

    if (str2uint32(value, str, 0))
    {
        if (*value < limit)
            return ESP_OK;

        //Value is over the limit
        dbgPrint(trALWAYS, "Invalid %s value (%d > %d)", type_str, *value, limit);
        return ESP_ERR_INVALID_ARG;
    }
    //Could not parse the string as a number
    dbgPrint(trALWAYS, "Invalid %s value (\"%s\")", type_str, str);
    return ESP_ERR_INVALID_ARG;
}

void _led_handler_off(void)
{
    //These functions (_menu_handler....) are called from the console task, so they should 
    // not manupiluate the RGB task variables directly, but instead send messages to the RGB 
    // task via the msg_queue
    bool help_requested = false;
    rgb_led_action_msg_t led_msg = {.led_addr = 0};//Indicating "not set yet"
    esp_err_t err = ESP_OK;
    uint32_t parse_value = 0;
//    uint32_t addr_mask = 0;

    while (task_console_arg_cnt() > 0)
	{
        char *arg = task_console_arg_pop();
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
            dbgPrint(trALWAYS, "Invalid Argument (\"%s\")", arg);
        // help_requested = true;
        break;
    }

    if ((!help_requested))
    {
        if (led_msg.led_addr == 0)
            led_msg.led_addr = BIT(0);             // No LED Address(es) specified... let's apply debug

        led_msg.cmd = led_action_off;
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
        dbgPrint(trALWAYS, "Usage: \"off [<led_addr>]\"");        
        dbgPrint(trALWAYS, "    <led_addr>: <#>      - a single index (0 to %d), e.g. 0, 1, 2, 3, etc", _rgb_led.cnt-1);
        dbgPrint(trALWAYS, "                0xHHHH   - a 16 bit mask of the LEDs to affect, e.g. \"0x003\"");
        dbgPrint(trALWAYS, "                \"<strip>[:<nr>]\" - a string and number (seperated by a colon)");
        dbgPrint(trALWAYS, "                            indicating the LED strip and LED # to use");
        dbgPrint(trALWAYS, "                            e.g. \"debug\", \"button:1\", \"button:2\", etc");
        dbgPrint(trALWAYS, "                  If <nr> is omitted, %s applies to the entire strip", "OFF");
        dbgPrint(trALWAYS, "        If <led_addr> is omitted, \"debug\" is assumed");
        dbgPrint(trALWAYS, " Multiple <led_addr> values can be specified, separated by spaces");
        //        dbgPrint(trALWAYS, "   e.g. \"off 0 1 2 3\", \"off debug:0 button:1...\" etc");
    }
}

void _led_handler_col(void)
{
    //These functions (_menu_handler....) are called from the console task, so they should 
    // not manupiluate the RGB task variables directly, but instead send messages to the RGB 
    // task via the msg_queue
    bool help_requested = false;
    rgb_led_action_msg_t led_msg = {.led_addr = 0, .val_0 = -1};//Indicating "not set yet"
    esp_err_t err = ESP_OK;
    uint32_t parse_value = 0;

    while (task_console_arg_cnt() > 0)
	{
        char *arg = task_console_arg_pop();
        if ((!strcasecmp("?", arg)) || (!strcasecmp("help", arg)))
        {    
            help_requested = true;
            break; //from while-loop
        }
        parse_value = 0;
        err = _handler_parse_colour(&parse_value, arg);
        if (err == ESP_OK)
        {
            if (led_msg.val_0 != (uint32_t)-1)
                dbgPrint(trALWAYS, "Overwriting previously set colour (%06X) with \"%s\" (%06X)", led_msg.val_0, arg, parse_value);

            led_msg.val_0 = parse_value;
            continue;
        }
        else if (err == ESP_ERR_INVALID_ARG)
        {
            //Looked like a Hue value, but it was invalid
            help_requested = true;
            break;
        }
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

        dbgPrint(trALWAYS, "Invalid Argument (\"%s\")", arg);
        help_requested = true;
        break;
    }

    if (!help_requested)
    {
        if (led_msg.val_0 == (uint32_t)-1)
        {
            // No colour? We'll assign one using a very silly RNG based on the time since boot in us (mod 360)
            hsv2rgb((uint32_t)(esp_timer_get_time()%360L), 100, 100, &led_msg.val_0);
            dbgPrint(trALWAYS, "No Colour specified, using 0x%06X", led_msg.val_0);
        }

        if (led_msg.led_addr == 0)
            led_msg.led_addr = BIT(0);             // No LED Address(es) specified... let's apply debug

        // Setting the Colour to Black is the same as turning the LED OFF
        led_msg.cmd = (led_msg.val_0 == colBlack) ? led_action_off : led_action_colour;

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
        dbgPrint(trALWAYS, "Usage: \"on <colour> [<led_addr>]\"");        
        // dbgPrint(trALWAYS, "    <colour>:   0xRRGGBB - A 6-character hexadecimal string containing the 24-");
        // dbgPrint(trALWAYS, "                            -bit RGB value, e.g. \"0x000001\", \"#0F0F0F\", etc");
        dbgPrint(trALWAYS, "    <colour>:   String   - Any one of the assigned colour names");
        dbgPrint(trALWAYS, "                            e.g. \"Black\", \"wh\", etc");
        dbgPrint(trALWAYS, "                \"HSV:<csv>\" - An HSV string in the format \"HSV:<h>[,<s>[,<v>]]\"");
        dbgPrint(trALWAYS, "                            e.g. \"HSV:120\", \"HSV:180,50,50\", etc");
        dbgPrint(trALWAYS, "                  <h> must be in the range 0 to %d degrees", HUE_MAX-1);
        dbgPrint(trALWAYS, "                  If <s> or <v> is omitted, it will be set to 100%%");
        dbgPrint(trALWAYS, "        If <colour> is omitted, one will be selected at random");
        dbgPrint(trALWAYS, "    <led_addr>: <#>      - a single index (0 to %d), e.g. 0, 1, 2, 3, etc", _rgb_led.cnt-1);
        dbgPrint(trALWAYS, "                0xHHHH   - a 16 bit mask of the LEDs to affect, e.g. \"0x003\"");
        dbgPrint(trALWAYS, "                \"<strip>[:<nr>]\" - a string and number (seperated by a colon)");
        dbgPrint(trALWAYS, "                            indicating the LED strip and LED # to use");
        dbgPrint(trALWAYS, "                            e.g. \"debug\", \"button:1\", \"button:2\", etc");
        dbgPrint(trALWAYS, "                  If <nr> is omitted, %s applies to the entire strip", "ON");
        dbgPrint(trALWAYS, "        If <led_addr> is omitted, \"debug\" is assumed");
        dbgPrint(trALWAYS, " Multiple <led_addr> values can be specified, separated by spaces");
//        dbgPrint(trALWAYS, "   e.g. \"off 0 1 2 3\", \"off debug:0 button:1...\" etc");
    }
}

void _led_handler_common_action(void/*rgb_led_action_cmd action*/)
{
    rgb_led_action_cmd action = led_action_nop;
    char *arg = task_console_arg_peek(-1);
    
    //These functions (_menu_handler....) are called from the console task, so they should 
    // not manupiluate the RGB task variables directly, but instead send messages to the RGB 
    // task via the msg_queue
    bool help_requested = false;
    rgb_led_action_msg_t led_msg = {.led_addr = 0, .val_0 = -1, .val_1 = 0, .val_2 = -1};//Indicating "not set yet"
    esp_err_t err = ESP_OK;
    uint32_t parse_value = 0;//addr_mask = 0;

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
                if (led_msg.val_1 != 0)
                    dbgPrint(trALWAYS, "Overwriting previously set period (%dms) with \"%s\" (%dms)", led_msg.val_1, arg, parse_value);
                led_msg.val_1 = parse_value;
                continue;
            }
        }
        //else not a good period value... maybe it is a colour or a LED address
        //Then we check for colour(s)
        err = _handler_parse_colour(&parse_value, arg);
        if (err == ESP_OK)
        {
            if (led_msg.val_0 == (uint32_t)-1)
            {
                led_msg.val_0 = parse_value;
            }
            else
            {
                if (led_msg.val_2 != (uint32_t)-1)
                    dbgPrint(trALWAYS, "Overwriting previously set colour (%06X) with \"%s\" (%06X)", led_msg.val_2, arg, parse_value);            
                led_msg.val_2 = parse_value;
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
        dbgPrint(trALWAYS, "Invalid Argument (\"%s\")", arg);
        help_requested = true;
        break;
    }

    if (!help_requested)
    {
        if (led_msg.val_0 == (uint32_t)-1)
        {
            // No colour? We'll assign both using a very silly RNG based on the time since boot in us (mod 360)
            uint32_t rand_hue = (uint32_t)(esp_timer_get_time()%360L);
            hsv2rgb(rand_hue, SAT_MAX, VAL_MAX, &led_msg.val_0);
            //The second colour is the complementary colour on the other side (+180 degrees) of the colour wheel
            hsv2rgb(rand_hue+(HUE_MAX/2), SAT_MAX, VAL_MAX, &led_msg.val_2);
            dbgPrint(trALWAYS, "No Colours specified, using 0x%06X and 0x%06X", led_msg.val_0, led_msg.val_2);
        }
        else if (led_msg.val_2 == (uint32_t)-1)
        {
            // No colour? We'll assign one using a very silly RNG based on the time since boot in us (mod 360)
            hsv2rgb((uint32_t)(esp_timer_get_time()%360L), 100, 100, &led_msg.val_2);
            dbgPrint(trALWAYS, "No Colour specified, using 0x%06X", led_msg.val_2);
        }
        

        if (led_msg.val_1 == 0)
        {
            // No period? We'll assign one using a very silly RNG based on the time since boot in us (mod 360)
            led_msg.val_1 = RGB_LED_BLINK_PERIOD_MS_DEF;
            dbgPrint(trALWAYS, "Using default period of %dms", led_msg.val_1);
        }
        if (led_msg.led_addr == 0)
            led_msg.led_addr = BIT(0);             // No LED Address(es) specified... let's apply debug

        led_msg.cmd = led_action_blink;
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
        dbgPrint(trALWAYS, "Usage: \"blink <period> [<colour_1>] [<colour_2>] [<led_addr>]\"");        
        dbgPrint(trALWAYS, "    <period>:   A value indicating the period of the blink in ms (min: %dms)", RGB_LED_BLINK_PERIOD_MS_MIN*2);
        dbgPrint(trALWAYS, "        If <period> is omitted, a default period of %dms is used", RGB_LED_BLINK_PERIOD_MS_DEF);
        dbgPrint(trALWAYS, "    <colour>:   String   - Any one of the assigned colour names");
        dbgPrint(trALWAYS, "                            e.g. \"Black\", \"wh\", etc");
        dbgPrint(trALWAYS, "                \"HSV:<csv>\" - An HSV string in the format \"HSV:<h>[,<s>[,<v>]]\"");
        dbgPrint(trALWAYS, "                            e.g. \"HSV:120\", \"HSV:180,50,50\", etc");
        dbgPrint(trALWAYS, "                  <h> must be in the range 0 to %d degrees", HUE_MAX-1);
        dbgPrint(trALWAYS, "                  If <s> or <v> is omitted, it will be set to 100%%");
        dbgPrint(trALWAYS, "        If one or both <colour> are omitted, they will be selected at random");
        dbgPrint(trALWAYS, "    <led_addr>: <#>      - a single index (0 to %d), e.g. 0, 1, 2, 3, etc", _rgb_led.cnt-1);
        dbgPrint(trALWAYS, "                0xHHHH   - a 16 bit mask of the LEDs to affect, e.g. \"0x003\"");
        dbgPrint(trALWAYS, "                \"<strip>[:<nr>]\" - a string and number (seperated by a colon)");
        dbgPrint(trALWAYS, "                            indicating the LED strip and LED # to use");
        dbgPrint(trALWAYS, "                            e.g. \"debug\", \"button:1\", \"button:2\", etc");
        dbgPrint(trALWAYS, "                  If <nr> is omitted, %s applies to the entire strip", "BLINK");
        dbgPrint(trALWAYS, "        If <led_addr> is omitted, \"debug\" is assumed");
        dbgPrint(trALWAYS, " Multiple <led_addr> values can be specified, separated by spaces");
//        dbgPrint(trALWAYS, "   e.g. \"off 0 1 2 3\", \"off debug:0 button:1...\" etc");
    }
}

void _led_handler_blink(void)
{
    //These functions (_menu_handler....) are called from the console task, so they should 
    // not manupiluate the RGB task variables directly, but instead send messages to the RGB 
    // task via the msg_queue
    bool help_requested = false;
    rgb_led_action_msg_t led_msg = {.led_addr = 0, .val_0 = -1, .val_1 = 0, .val_2 = -1};//Indicating "not set yet"
    esp_err_t err = ESP_OK;
    uint32_t parse_value = 0;//addr_mask = 0;

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
                if (led_msg.val_1 != 0)
                    dbgPrint(trALWAYS, "Overwriting previously set period (%dms) with \"%s\" (%dms)", led_msg.val_1, arg, parse_value);
                led_msg.val_1 = parse_value;
                continue;
            }
        }
        //else not a good period value... maybe it is a colour or a LED address
        //Then we check for colour(s)
        err = _handler_parse_colour(&parse_value, arg);
        if (err == ESP_OK)
        {
            if (led_msg.val_0 == (uint32_t)-1)
            {
                led_msg.val_0 = parse_value;
            }
            else
            {
                if (led_msg.val_2 != (uint32_t)-1)
                    dbgPrint(trALWAYS, "Overwriting previously set colour (%06X) with \"%s\" (%06X)", led_msg.val_2, arg, parse_value);            
                led_msg.val_2 = parse_value;
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
        dbgPrint(trALWAYS, "Invalid Argument (\"%s\")", arg);
        help_requested = true;
        break;
    }

    if (!help_requested)
    {
        if (led_msg.val_0 == (uint32_t)-1)
        {
            // No colour? We'll assign both using a very silly RNG based on the time since boot in us (mod 360)
            uint32_t rand_hue = (uint32_t)(esp_timer_get_time()%360L);
            hsv2rgb(rand_hue, SAT_MAX, VAL_MAX, &led_msg.val_0);
            //The second colour is the complementary colour on the other side (+180 degrees) of the colour wheel
            hsv2rgb(rand_hue+(HUE_MAX/2), SAT_MAX, VAL_MAX, &led_msg.val_2);
            dbgPrint(trALWAYS, "No Colours specified, using 0x%06X and 0x%06X", led_msg.val_0, led_msg.val_2);
        }
        else if (led_msg.val_2 == (uint32_t)-1)
        {
            // No colour? We'll assign one using a very silly RNG based on the time since boot in us (mod 360)
            hsv2rgb((uint32_t)(esp_timer_get_time()%360L), 100, 100, &led_msg.val_2);
            dbgPrint(trALWAYS, "No Colour specified, using 0x%06X", led_msg.val_2);
        }
        

        if (led_msg.val_1 == 0)
        {
            // No period? We'll assign one using a very silly RNG based on the time since boot in us (mod 360)
            led_msg.val_1 = RGB_LED_BLINK_PERIOD_MS_DEF;
            dbgPrint(trALWAYS, "Using default period of %dms", led_msg.val_1);
        }
        if (led_msg.led_addr == 0)
            led_msg.led_addr = BIT(0);             // No LED Address(es) specified... let's apply debug

        led_msg.cmd = led_action_blink;
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
        dbgPrint(trALWAYS, "Usage: \"blink <period> [<colour_1>] [<colour_2>] [<led_addr>]\"");        
        dbgPrint(trALWAYS, "    <period>:   A value indicating the period of the blink in ms (min: %dms)", RGB_LED_BLINK_PERIOD_MS_MIN*2);
        dbgPrint(trALWAYS, "        If <period> is omitted, a default period of %dms is used", RGB_LED_BLINK_PERIOD_MS_DEF);
        dbgPrint(trALWAYS, "    <colour>:   String   - Any one of the assigned colour names");
        dbgPrint(trALWAYS, "                            e.g. \"Black\", \"wh\", etc");
        dbgPrint(trALWAYS, "                \"HSV:<csv>\" - An HSV string in the format \"HSV:<h>[,<s>[,<v>]]\"");
        dbgPrint(trALWAYS, "                            e.g. \"HSV:120\", \"HSV:180,50,50\", etc");
        dbgPrint(trALWAYS, "                  <h> must be in the range 0 to %d degrees", HUE_MAX-1);
        dbgPrint(trALWAYS, "                  If <s> or <v> is omitted, it will be set to 100%%");
        dbgPrint(trALWAYS, "        If one or both <colour> are omitted, they will be selected at random");
        dbgPrint(trALWAYS, "    <led_addr>: <#>      - a single index (0 to %d), e.g. 0, 1, 2, 3, etc", _rgb_led.cnt-1);
        dbgPrint(trALWAYS, "                0xHHHH   - a 16 bit mask of the LEDs to affect, e.g. \"0x003\"");
        dbgPrint(trALWAYS, "                \"<strip>[:<nr>]\" - a string and number (seperated by a colon)");
        dbgPrint(trALWAYS, "                            indicating the LED strip and LED # to use");
        dbgPrint(trALWAYS, "                            e.g. \"debug\", \"button:1\", \"button:2\", etc");
        dbgPrint(trALWAYS, "                  If <nr> is omitted, %s applies to the entire strip", "BLINK");
        dbgPrint(trALWAYS, "        If <led_addr> is omitted, \"debug\" is assumed");
        dbgPrint(trALWAYS, " Multiple <led_addr> values can be specified, separated by spaces");
//        dbgPrint(trALWAYS, "   e.g. \"off 0 1 2 3\", \"off debug:0 button:1...\" etc");
    }

    //Print a status of all LEDs?
}

void _led_handler_rainbow(void)
{
    //These functions (_menu_handler....) are called from the console task, so they should 
    // not manupiluate the RGB task variables directly, but instead send messages to the RGB 
    // task via the msg_queue
    bool help_requested = false;
    rgb_led_action_msg_t led_msg = {.led_addr = 0, .val_1 = 0};//Indicating "not set yet"
    esp_err_t err = ESP_OK;
    uint32_t parse_value = 0;//addr_mask = 0;

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
                if (led_msg.val_1 != 0)
                    dbgPrint(trALWAYS, "Overwriting previously set period (%dms) with \"%s\" (%dms)", led_msg.val_1, arg, parse_value);
                led_msg.val_1 = parse_value;
                continue;
            }
            //Now we need to assume it is an address... fall through to the next check?
        }
        //else not a good period value... maybe it is something
        //Then we check for LED addressing
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
        dbgPrint(trALWAYS, "Invalid Argument (\"%s\")", arg);
        help_requested = true;
        break;
    }

    if (!help_requested)
    {
        if (led_msg.val_1 == 0)
        {
            // No period? We'll assign one using a very silly RNG based on the time since boot in us (mod 360)
            led_msg.val_1 = RGB_LED_RAINBOW_PERIOD_MS_DEF;
            dbgPrint(trALWAYS, "Using default period of %dms", led_msg.val_1);
        }

        if (led_msg.led_addr == 0)
            led_msg.led_addr = BIT(0);             // No LED Address(es) specified... let's apply debug

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
        dbgPrint(trALWAYS, "Usage: \"rainbow <period> [<led_addr>]\"");        
        dbgPrint(trALWAYS, "    <period>:   A value indicating the period of the rainbow cycle in ms");
        dbgPrint(trALWAYS, "                            range %d to %d", RGB_LED_RAINBOW_PER_MIN_MS, RGB_LED_RAINBOW_PER_MAX_MS);
        dbgPrint(trALWAYS, "        If <period> is omitted, a default period of %dms is used", RGB_LED_RAINBOW_PERIOD_MS_DEF);
        dbgPrint(trALWAYS, "    <led_addr>: <#>      - a single index (0 to %d), e.g. 0, 1, 2, 3, etc", _rgb_led.cnt-1);
        dbgPrint(trALWAYS, "                0xHHHH   - a 16 bit mask of the LEDs to affect, e.g. \"0x003\"");
        dbgPrint(trALWAYS, "                \"<strip>[:<nr>]\" - a string and number (seperated by a colon)");
        dbgPrint(trALWAYS, "                            indicating the LED strip and LED # to use");
        dbgPrint(trALWAYS, "                            e.g. \"debug\", \"button:1\", \"button:2\", etc");
        dbgPrint(trALWAYS, "                  If <nr> is omitted, %s applies to the entire strip", "RAINBOW");
        dbgPrint(trALWAYS, "        If <led_addr> is omitted, \"debug\" is assumed");
        dbgPrint(trALWAYS, " Multiple <led_addr> values can be specified, separated by spaces");
//        dbgPrint(trALWAYS, "   e.g. \"off 0 1 2 3\", \"off debug:0 button:1...\" etc");
    }
}

void _led_handler_status(void)
{
    //These functions (_menu_handler....) are called from the console task, so they should 
    // not manupiluate the RGB task variables directly, but instead send messages to the RGB 
    // task via the msg_queue
    bool help_requested = false;
    rgb_led_action_msg_t led_msg = {.led_addr = 0};//Indicating "not set yet"
    esp_err_t err = ESP_OK;
    uint32_t parse_value = 0;
//    uint32_t addr_mask = 0;

    while (task_console_arg_cnt() > 0)
	{
        char *arg = task_console_arg_pop();
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
            dbgPrint(trALWAYS, "Invalid Argument (\"%s\")", arg);
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
        dbgPrint(trALWAYS, "Usage: \"ledstat [<led_addr>]\"");        
        dbgPrint(trALWAYS, "    <led_addr>: <#>      - a single index (0 to %d), e.g. 0, 1, 2, 3, etc", _rgb_led.cnt-1);
        dbgPrint(trALWAYS, "                0xHHHH   - a 16 bit mask of the LEDs to affect, e.g. \"0x003\"");
        dbgPrint(trALWAYS, "                \"<strip>[:<nr>]\" - a string and number (seperated by a colon)");
        dbgPrint(trALWAYS, "                            indicating the LED strip and LED # to use");
        dbgPrint(trALWAYS, "                            e.g. \"debug\", \"button:1\", \"button:2\", etc");
        dbgPrint(trALWAYS, "                  If <nr> is omitted, %s applies to the entire strip", "STATUS");
        dbgPrint(trALWAYS, "        If <led_addr> is omitted, \"debug\" is assumed");
        dbgPrint(trALWAYS, " Multiple <led_addr> values can be specified, separated by spaces");
        //        dbgPrint(trALWAYS, "   e.g. \"off 0 1 2 3\", \"off debug:0 button:1...\" etc");
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

    // if (task_console_arg_cnt() > 0)
    // {
    //     //Let's get an idea of how many colours we need to list
    //     while (colour_list_item(col_list_size) != NULL)
    //         col_list_size++;
    // }
    if (task_console_arg_cnt() == 0)
    {
        //No arguments... let's list all the colours
        show_all = true;
        help_requested = true;

    }

    //First check that all arguments are valid
    for (int i = 0; i < task_console_arg_cnt(); i++)
    {
        char *arg = task_console_arg_peek(i);
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
        err = _handler_parse_colour(&parse_value, arg);
        if (err == ESP_ERR_NOT_FOUND)
        {
            //No a parsable colour
            dbgPrint(trALWAYS, "Invalid Argument (\"%s\")", arg);
            help_requested = true;
            break;
        }

    }

    if (show_hues)
    {
        dbgPrint(trALWAYS, "The RGB value for all 359 HUEs (100%% Sat and Val) are:");
        for (int i = 0; i < HUE_MAX; i++)
        {
            const char * col_name = NULL;
            uint32_t rgb_val = 0;
            uint32_t hue, sat, val;
            hsv2rgb(i, SAT_MAX, VAL_MAX, &rgb_val);
            rgb2hsv(rgb_val, &hue, &sat, &val);
            dbgPrint_i(trALWAYS, " Hue: % 3d -> 0x%06X", i, rgb_val);
            col_name = rgb2name(rgb_val);
            dbgPrint_i(trALWAYS, " - HSV: %3d, %3d, %3d", hue, sat, val);
            if (col_name != NULL)
                dbgPrint_i(trALWAYS, " (%s)", col_name);
            dbgPrint_i(trALWAYS, "\n");
        }
        return;
    }
    
    if (show_all)
    {
        dbgPrint(trALWAYS, "The RGB and HSV values for the named colours are:");
        int i = 0;
        const char *col_name = colour_list_item(i);
        while (col_name != NULL)
        {
            uint32_t rgb_val;
            if (str2rgb(&rgb_val, col_name) != ESP_OK)
                dbgPrint(trALWAYS, " % 2d: %s -> Invalid Colour", i, col_name); //Should never happen
            else
            {
                uint32_t hue, sat, val;
                rgb2hsv(rgb_val, &hue, &sat, &val);
                dbgPrint(trALWAYS, " % 8s -> 0x%06X - HSV: %3d, %3d, %3d", col_name, rgb_val, hue, sat, val);
            }
            col_name = colour_list_item(++i);
        }
        return;
    }
    if (!help_requested)
    {
        //All the arguments are valid... let's list each one of them 
        //There are two options.... 
        // 1) the used provides a Colour name (to get the RGB value)
        // 2) the user provides a Hue value (to get the RGB value)
        while (task_console_arg_cnt() > 0)
        {
            char *arg = task_console_arg_pop();
            if ((!strcasecmp("all", arg)) || (!strcasecmp("list", arg)))
                continue;   //Skip.... already handled
            if (!strcasecmp("hues", arg))
                continue;   //Skip.... already handled
            if (ESP_OK == _handler_parse_colour(&parse_value, arg))
            {
                dbgPrint_i(trALWAYS, " % 8s -> 0x%06X", arg, parse_value);
                const char *col_name = rgb2name(parse_value);
                if (col_name != NULL)
                    dbgPrint_i(trALWAYS, " (%s)", col_name);
                dbgPrint_i(trALWAYS, "\n");
            }
        }
        return;
    }

    if (help_requested)
    {
        //                  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        dbgPrint(trALWAYS, "Usage: \"rgb [<colour>]\"");        
        dbgPrint(trALWAYS, "    <colour>:   String   - Any one of the assigned colour names");
        dbgPrint(trALWAYS, "                            e.g. \"Black\", \"wh\", etc");
        dbgPrint(trALWAYS, "                \"HSV:<csv>\" - An HSV string in the format \"HSV:<h>[,<s>[,<v>]]\"");
        dbgPrint(trALWAYS, "                            e.g. \"HSV:120\", \"HSV:180,50,50\", etc");
        dbgPrint(trALWAYS, "                  <h> must be in the range 0 to %d degrees", HUE_MAX-1);
        dbgPrint(trALWAYS, "                  If <s> or <v> is omitted, it will be set to 100%%");
        dbgPrint(trALWAYS, "                \"all|list\" - Lists ALL the available named colours");
        dbgPrint(trALWAYS, "                \"hues\" - Lists the RGB values for all 359 HUE values");
        dbgPrint(trALWAYS, "        If <colour> is omitted \"rgb all\" is assumed");
        dbgPrint(trALWAYS, " Multiple <colour> values can be specified, separated by spaces");
    }
}

void _led_handler_reset(void)
{
    //These functions (_menu_handler....) are called from the console task, so they should 
    // not manupiluate the RGB task variables directly, but instead send messages to the RGB 
    // task via the msg_queue
    bool help_requested = false;
    rgb_led_action_msg_t led_msg = {.led_addr = 0, .val_1 = 0};//Indicating "not set yet"
    esp_err_t err = ESP_OK;
    uint32_t parse_value = 0;//addr_mask = 0;

    while (task_console_arg_cnt() > 0)
	{
        char *arg = task_console_arg_pop();
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
        dbgPrint(trALWAYS, "Invalid Argument (\"%s\")", arg);
        help_requested = true;
        break;
    }

    if (!help_requested)
    {
        if (led_msg.led_addr == 0)
            led_msg.led_addr = BIT(0);             // No LED Address(es) specified... let's apply debug

        led_msg.val_1 = RGB_LED_RAINBOW_PERIOD_MS_DEF;
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
        dbgPrint(trALWAYS, "Usage: \"reset [<led_addr>]\"");        
        dbgPrint(trALWAYS, "    <led_addr>: <#>      - a single index (0 to %d), e.g. 0, 1, 2, 3, etc", _rgb_led.cnt-1);
        dbgPrint(trALWAYS, "                0xHHHH   - a 16 bit mask of the LEDs to affect, e.g. \"0x003\"");
        dbgPrint(trALWAYS, "                \"<strip>[:<nr>]\" - a string and number (seperated by a colon)");
        dbgPrint(trALWAYS, "                            indicating the LED strip and LED # to use");
        dbgPrint(trALWAYS, "                            e.g. \"debug\", \"button:1\", \"button:2\", etc");
        dbgPrint(trALWAYS, "                  If <nr> is omitted, %s applies to the entire strip", "RESET");
        dbgPrint(trALWAYS, "        If <led_addr> is omitted, \"debug\" is assumed");
        dbgPrint(trALWAYS, " Multiple <led_addr> values can be specified, separated by spaces");
//        dbgPrint(trALWAYS, "   e.g. \"off 0 1 2 3\", \"off debug:0 button:1...\" etc");
    }
}
/*******************************************************************************
Global (public) Functions
*******************************************************************************/

TaskInfo_t * task_rgb_led_init(void)
{
	//Let's not re-initialise this task by accident
	if (_rgb_led.task.handle)
		return &_rgb_led.task;

	_rgb_led.cnt = 0;

	// Create the task, storing the handle.  Note that the passed parameter ucParameterToPass
	// must exist for the lifetime of the task, so in this case is declared static.  If it was just an
	// an automatic stack variable it might no longer exist, or at least have been corrupted, by the time
	// the new task attempts to access it.
	if (xTaskCreate( _led_task, PRINTF_TAG, _rgb_led.task.stack_depth, _rgb_led.task.parameter_to_pass, 10, &_rgb_led.task.handle ) != pdPASS)
	{
		dbgPrint(trALWAYS, "#Unable to start Task (%d)!", eTaskIndexRgb);
		return NULL;
	}

	configASSERT(_rgb_led.task.handle);

	return &_rgb_led.task;
}

void task_rgb_led_deinit(void)
{
	// Use the handle to delete the task.
	if(_rgb_led.task.handle != NULL )
		vTaskDelete(_rgb_led.task.handle);

}

esp_err_t task_rgb_led_off(uint16_t address_mask)
{
    //This function can be called from ANY task
    rgb_led_action_msg_t led_msg = {
        .led_addr = 0, 
        .cmd = led_action_nop, 
        .val_0 = -1, 
    };//Indicating "not set yet"

    if (address_mask == 0)
        return ESP_OK;  //Technically not an error... but nothing to do

    for (int i = 0; i < _rgb_led.cnt; i++)
        led_msg.led_addr |= (BIT(i) & address_mask);

    led_msg.val_0 = colBlack;
    led_msg.cmd = led_action_off;
    if (xQueueSend(_rgb_led.msg_queue, &led_msg, 0) == pdTRUE)
        return ESP_OK;

    dbgPrint(trRGB, "#Failed to queue message (%s)", __FUNCTION__);
    return ESP_FAIL;
}

esp_err_t task_rgb_led_on(uint16_t address_mask, uint32_t rgb_colour)
{
    //This function can be called from ANY task
    rgb_led_action_msg_t led_msg = {
        .led_addr = 0, 
        .cmd = led_action_nop, 
        .val_0 = -1, 
    };//Indicating "not set yet"

    if (address_mask == 0)
        return ESP_OK;  //Technically not an error... but nothing to do

    if (rgb_colour == (uint32_t)-1)
    {
        uint32_t hue;
        hue = (uint32_t)(esp_timer_get_time()%360L);
        //Always use max saturation and value for "random" colours
        hsv2rgb(hue, SAT_MAX, VAL_MAX, &rgb_colour);
    }
    
    if (rgb_colour > RGB_MAX)
    {
        dbgPrint(trRGB, "#Invalid RGB value (%d: 0x%06X)", 1, rgb_colour);
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < _rgb_led.cnt; i++)
        led_msg.led_addr |= (BIT(i) & address_mask);

    led_msg.val_0 = rgb_colour;
    led_msg.cmd = (rgb_colour == colBlack)? led_action_off : led_action_colour;
    if (xQueueSend(_rgb_led.msg_queue, &led_msg, 0) == pdTRUE)
        return ESP_OK;

    dbgPrint(trRGB, "#Failed to queue message (%s)", __FUNCTION__);
    return ESP_FAIL;
}

esp_err_t task_rgb_led_blink(uint16_t address_mask, uint32_t period, uint32_t rgb_colour_1, uint32_t rgb_colour_2)
{
    rgb_led_action_msg_t led_msg = {
        .led_addr = 0, 
        .cmd = led_action_nop, 
        .val_0 = -1, 
        .val_1 = 0, 
        .val_2 = -1
    };//Indicating "not set yet"

    if (address_mask == 0)
        return ESP_OK;  //Technically not an error... but nothing to do

    if (period < (RGB_LED_BLINK_PERIOD_MS_MIN*2))
    {
        dbgPrint(trRGB, "#Min blink period = %dms (got %dms)", (RGB_LED_BLINK_PERIOD_MS_MIN*2), period);
        return ESP_ERR_INVALID_ARG;
    }

    if (rgb_colour_1 == (uint32_t)-1)
    {
        uint32_t hue;
        hue = (uint32_t)(esp_timer_get_time()%360L);
        //Always use max saturation and value for "random" colours
        hsv2rgb(hue, SAT_MAX, VAL_MAX, &rgb_colour_1);
    }
    
    if (rgb_colour_1 > RGB_MAX)
    {
        dbgPrint(trRGB, "#Invalid RGB value (%d: 0x%06X)", 1, rgb_colour_1);
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
        hsv2rgb(hue, sat, val, &rgb_colour_2);
    }

    if (rgb_colour_2 > RGB_MAX)
    {
        dbgPrint(trRGB, "#Invalid RGB value (%d: 0x%06X)", 2, rgb_colour_2);
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

    dbgPrint(trRGB, "#Failed to queue message (%s)", __FUNCTION__);
    return ESP_FAIL;
}

esp_err_t task_rgb_led_demo(uint16_t address_mask, uint32_t period)
{
    rgb_led_action_msg_t led_msg = {
        .led_addr = 0, 
        .cmd = led_action_nop, 
        .val_1 = 0, 
    };//Indicating "not set yet"

    if (address_mask == 0)
        return ESP_OK;  //Technically not an error... but nothing to do

    if (period < (RGB_LED_BLINK_PERIOD_MS_MIN*2))
    {
        dbgPrint(trRGB, "#Min blink period = %dms (got %dms)", (RGB_LED_BLINK_PERIOD_MS_MIN*2), period);
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < _rgb_led.cnt; i++)
        led_msg.led_addr |= (BIT(i) & address_mask);

    led_msg.val_1 = period;
    led_msg.cmd = led_action_rainbow;
    if (xQueueSend(_rgb_led.msg_queue, &led_msg, 0) == pdTRUE)
        return ESP_OK;

    dbgPrint(trRGB, "#Failed to queue message (%s)", __FUNCTION__);
    return ESP_FAIL;
}

#undef PRINTF_TAG

/*************************** END OF FILE *************************************/
