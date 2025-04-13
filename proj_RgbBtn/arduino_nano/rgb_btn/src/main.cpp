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
typedef enum registration_state_e
{
    no_init,
    un_reg,         //We've not been registered yet
    roll_call,      //We are in the process of responding to a roll-call
    waiting,        //We are waiting on a registration from the master
    idle,           //We have been registered with the master (got a bit-mask address)
}registration_state_t;

/*******************************************************************************
Structures and unions
 *******************************************************************************/

/*******************************************************************************
 Function prototypes
 *******************************************************************************/

void btn_blink(void);
void check_button_press(void);
void address_update(void);

void msg_process(void);
bool _rx_handler_rollcall_msg(uint8_t _cmd, uint8_t _src, uint8_t _dst);
bool _read_cmd_payload(uint8_t cmd, uint8_t * dst);
uint8_t _read_msg_data(uint8_t * dst, uint8_t len);
uint8_t _read_cmd_payload_len(uint8_t cmd);

void send_response(void);
void _rx_handler_state_un_reg(uint8_t _cmd);

bool _is_bcast_msg_for_me(uint32_t bit_mask);


 //Menu Commands (without any owners)
#ifdef MAIN_DEBUG
#ifdef CONSOLE_ENABLED
void(*resetFunc)(void) = 0; //declare reset function at address 0
#if REDUCE_CODESIZE==0
    void _sys_handler_time(void);
#endif
void _sys_handler_reset(void);
#if (DEV_RGB_DEBUG == 1)
void _sys_handler_led(void);
#endif
//bool menuVersion(void);
/*! Displays the application version
 */
void _sys_handler_version(void);

/*! Calculates the CRC-8 of the given data
 */
void _sys_handler_crc(void);

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
#if (DEV_RGB_DEBUG == 1)
    {"rgb",     _sys_handler_led,   "Get/Set the RGB LED colour" },
#endif
#if REDUCE_CODESIZE==0
    {"time",     _sys_handler_time,   "Displays the uptime of the program" }
#endif /* REDUCE_CODESIZE */
    {"crc",     _sys_handler_crc,       "CRC-8 calculator"},
    {"ram",     _sys_handler_dump_ram,  "Display RAM"},
    {"flash",   _sys_handler_dump_flash,"Display FLASH"},
    {"version", _sys_handler_version,   "Displays app version"},
};
#endif /* CONSOLE_ENABLED */
#endif /* MAIN_DEBUG */
//int SystemState;		/* Movement State Machine variable */

const char BuildTimeData[] = { __TIME__ " " __DATE__ }; /* Used in our startup Banner*/

/* We retain 3 colours:
 0) Primary colour - used for solid colour display (not blinking) or one of the blinking colours
 1) Secondary colour - used for the other blinking colour
 2) Tertiary colour - used as a fallback (primary) colour once the button is pressed
*/
rgb_colour_t colour[3];
timer_ms_t blink_tmr;

registration_state_t reg_state = no_init; //We are not registered yet
int8_t my_mask_index = -1; //My address bit mask index, valid values 0 to 31

stopwatch_ms_s roll_call_sw;
uint32_t roll_call_time_ms = 0; //The time we have to wait for the roll-call to finish

bool response_msg_due = false;
//uint8_t send_ack_nak = 0x00;

typedef struct {
    uint8_t src;    // The source address of the message
    uint8_t dst;    // The destination address of the message
    uint8_t data[RGB_BTN_MSG_MAX_LEN - sizeof(comms_msg_hdr_t) - sizeof(uint8_t)];
    uint8_t len;
    uint8_t rd_index;
} rx_msg_t;

rx_msg_t msg = {0};

/*******************************************************************************
 Functions
 *******************************************************************************/

void setup() {

    // put your setup code here, to run once:

    dev_comms_init();

    _sys_handler_version();
    console_add_menu("main", _main_menu_items, ARRAY_SIZE(_main_menu_items), "Main Menu");

    dev_nvstore_init();
   
    dev_rgb_start(output_Led_Red, output_Led_Green, output_Led_Blue);

    dev_button_init();
    colour[0].rgb = (uint32_t)colOrange;
    colour[1].rgb = (uint32_t)colGreen;
    colour[2].rgb = (uint32_t)colTeal;

    sys_poll_tmr_stop(&blink_tmr);

    address_update();

    reg_state = un_reg; //We are not registered yet

    //This can only be set by the master device
    my_mask_index = -1;

    //Testing only
    // sys_poll_tmr_start(&blink_tmr, 200lu, true);
    // dev_button_measure_start();
}

void loop() {

    // put your main code here, to run repeatedly:

    check_button_press();

    btn_blink();

    //Check if our communication address has changed
    address_update();

	//Check for anything we may have received on the main comms channnel...
    //int8_t available = dev_comms_rx_msg_available();
    msg.len = dev_comms_rx_msg_available(&msg.src, &msg.dst, msg.data);
    if (msg.len > 0)
        msg_process(); //... process it

    //If anything requires sending, now is the time to do it
    send_response();
   
    // //Check for anything coming in on the Serial Port and process it
    console_service();

}

void check_button_press(void)
{
    //Has the button seen any action?
    uint8_t btn_state = dev_button_get_state();
    //We only process btn_pressed and only if the button is/was active!
    if ((btn_state & (btn_active | btn_pressed)) == (btn_active | btn_pressed))
    {
        //iprintln(trBUTTON, "#Button Pressed %lu ms", dev_button_get_reaction_time_ms());
        sys_poll_tmr_stop(&blink_tmr);
        dev_rgb_set_colour(colour[2].rgb);
    }
}

void btn_blink(void)
{
    //Do we require blinking on our LED?
	if (sys_poll_tmr_expired(&blink_tmr))
	{
        //Swop the colours on the RGB LED and set the new colour
        SWOP_U32(colour[0].rgb, colour[1].rgb);
        dev_rgb_set_colour(colour[0].rgb);
	}
}

void address_update(void)
{
    uint8_t stored_addr;
    uint8_t current_addr;

    //Check if our communication address has changed
    if (!dev_nvstore_new_data_available())
        return;

    current_addr = dev_comms_addr_get();

    //Read the address from the NV store
    dev_nvstore_read(&stored_addr, sizeof(uint8_t));

    //Check if the address in the NV store is different from the one we are using
    if (stored_addr == current_addr)
        return; //No change

    //iprintln(trALWAYS, "#New Address read from NV Store: 0x%02X (old: 0x%02X)", stored_addr, current_addr);
    //Reset the comms address to that which we got from the NVstore
    if (dev_comms_addr_set(stored_addr))
        return; //All good, we have a new address

    //Not sure why this one failed, but let's just reset the address to the valid address already in use by the device
    dev_nvstore_write(&current_addr, sizeof(uint8_t));                
    //Should raise the flag to get it reset for the next cycle
    
}
void msg_process(void)
{
    uint8_t _u8_val = 0;
    uint32_t _u32_val = 0;
    uint8_t _cnt = 0;
    bool accept_msg = false; //We start off NOT accepting broadcast messages (yet)
    master_command_t _cmd;
    uint8_t _myAddr = dev_comms_addr_get();

    msg.rd_index = 0; //Set the read pointer to the start of the data array

    //We are accepting direct messages IF we are WAITING or IDLE
    if ((msg.src == ADDR_MASTER) && (msg.dst == _myAddr) && (reg_state >= waiting))
        accept_msg = true;

    dev_comms_response_start(); //Start the response message

    //Have we received anything?
    while (_read_msg_data((uint8_t *)&_cmd, 1) > 0)
    {
        //Check for a roll-call first
        if (_rx_handler_rollcall_msg(_cmd, msg.src, msg.dst))
            return; //Handled the roll-call message already

        // if (reg_state != idle)
        //     continue; //We are not in the idle state, so we don't care about anything else
        
        if ((!accept_msg) && ((_cnt > 0) || (_cmd != cmd_bcast_address_mask)))
            return; // No point in processing the rest of the message if we are not accepting it

        switch (_cmd)
        {
            case cmd_bcast_address_mask:
                if (_read_cmd_payload(_cmd, (uint8_t *)&_u32_val))
                {
                    iprint(trALWAYS, "#BCST 0x%08X & 0x%08X : ", BIT_POS(my_mask_index), _u32_val);
                    //We are being registered by the master, so we need to set our address bit mask
                    if (_is_bcast_msg_for_me(_u32_val))                    
                        accept_msg = true;                        

                    iprintln(trALWAYS, "%s", accept_msg? "YES" : "NO");
                }
                //else //read failure already handled in _read_cmd_payload()
                break;
            
            case cmd_set_bitmask_index:
                //We are being registered by the master, so we need to set our address bit mask
                if (_read_cmd_payload(_cmd, &_u8_val))
                {
                    if (((int8_t)_u8_val < 0) || ((int8_t)_u8_val > RGB_BTN_MAX_NODES))
                    {
                        iprintln(trALWAYS, "#Invalid index (%d)", _u8_val);
                        dev_comms_response_add(_cmd, btn_cmd_err_range, &_u8_val, 1);
                        break; // from switch... continue with the next command
                    }
                    my_mask_index = (int8_t)_u8_val; //Set the address bit mask for this device
                    iprintln(trALWAYS, "#State: IDLE (Index: %d)", my_mask_index);
                    reg_state = idle; //We are now in the "idle" state
                    dev_comms_response_add(_cmd, btn_cmd_err_ok, NULL, 0);
                }
                //else //read failure already handled in _read_cmd_payload()
                break;

            case cmd_set_rgb_0:
                if (_read_cmd_payload(_cmd, (uint8_t *)&_u32_val))
                {
                    sys_poll_tmr_stop(&blink_tmr);
                    colour[0].rgb = _u32_val;
                    dev_rgb_set_colour(colour[0].rgb);
                    dev_comms_response_add(_cmd, btn_cmd_err_ok, NULL, 0);
                }
                //else //read failure already handled in _read_cmd_payload()
                break;
    
            case cmd_set_rgb_1:
            case cmd_set_rgb_2:
                if (_read_cmd_payload(_cmd, (uint8_t *)&_u32_val))
                {
                    if (_cmd == cmd_set_rgb_1)
                        colour[1].rgb = _u32_val;
                    else
                        colour[2].rgb = _u32_val;
                    dev_comms_response_add(_cmd, btn_cmd_err_ok, NULL, 0);
                }
                //else //read failure already handled in _read_cmd_payload()
                break;
    
            case cmd_set_blink:
                if (_read_cmd_payload(_cmd, (uint8_t *)&_u32_val))
                {
                    sys_poll_tmr_stop(&blink_tmr);
                    if (_u32_val > 0) //Only restart the timer if the period is > 0
                        sys_poll_tmr_start(&blink_tmr, (unsigned long)_u32_val, true);
                    dev_comms_response_add(_cmd, btn_cmd_err_ok, NULL, 0);
                }
                break;
    
            case cmd_get_btn:
                //We're not even bothering with reading a payload....
                //_read_cmd_payload(_cmd, NULL);   // No data to read
                //RVN - Should add the button state?
                //dev_comms_response_add_data(cmd_get_btn, (uint8_t*)&press_time, sizeof(unsigned long));
                _u32_val = dev_button_get_reaction_time_ms();
                dev_comms_response_add(_cmd, btn_cmd_err_ok, (uint8_t*)&_u32_val, sizeof(uint32_t));
                break;
            
            case cmd_sw_start:
                //We're not even bothering with reading a payload....
                //_read_cmd_payload(_cmd, NULL);   // No data to read
                //Nothing else.... just start the stopwatch
                dev_button_measure_start();
                dev_comms_response_add(_cmd, btn_cmd_err_ok, NULL, 0);
                break;
    
            case cmd_wr_console_cont:
            case cmd_wr_console_done:
                uint8_t _data[RGB_BTN_MSG_MAX_LEN - sizeof(comms_msg_hdr_t) - sizeof(uint8_t)];
                memset(_data, 0, sizeof(_data));
                _read_cmd_payload(_cmd, _data);
                for (int8_t i = 0; (i < ((int8_t)sizeof(_data))) && (_data[i] > 0); i++)
                    console_read_byte(_data[i]);
                if (_cmd == cmd_wr_console_done)
                {
                    //iprintln(trALWAYS, "#Console: \"%s\"", _data);
                    //RVN - TODO - This is the last part of a console message, so we need to send the response back to the master device
                    console_enable_alt_output_stream(dev_comms_response_add_byte);
                    dev_comms_response_add(cmd_wr_console_cont, btn_cmd_err_ok, NULL, 0);
                    console_read_byte('\n');
                    console_service(); //This *should* pipe the output to the alternate stream and reset it back to stdouot once it is done
                    dev_comms_transmit_now();
                    dev_comms_response_add(cmd_wr_console_done, btn_cmd_err_ok, NULL, 0);
                }
                break;
    
            //case cmd_set_address:
            // case cmd_roll_call:             //Already handled
            default:
                //Unkown/unhandled command.... return error (NAK?) to master
                dev_comms_response_add(_cmd, btn_cmd_err_unknown_cmd, NULL, 0);
                break;
        }
        _cnt++;

    }

    //If we have a response ready for a direct message, we need to send it immediately, (without waiting for bus silence)
    if ((msg.src == ADDR_MASTER) && (msg.dst == _myAddr) && (reg_state >= waiting))
        dev_comms_response_send();
}

uint8_t _read_msg_data(uint8_t * dst, uint8_t len)
{
    uint8_t data_len = min(msg.len, sizeof(msg.data));

    // iprintln(trMAIN, "data_rd_index = %d", _comms.rx.data_rd_index);
    // iprintln(trMAIN, "data_length   = %d", _comms.rx.data_length);
    if (len == 0)
        return 0; //Nothing to do

    if (msg.rd_index >= data_len)
        return 0; //No data available

    if (len > (data_len - msg.rd_index))
        return 0; //Requesting more data than available

    if ((dst) && (len > 0))// Don't copy to NULL
        memcpy(dst, &msg.data[msg.rd_index], len);
    
    msg.rd_index += len;

    return len;
}

bool _read_cmd_payload(uint8_t cmd, uint8_t * dst)
{
    //RVN - Cleanup needed
    uint8_t len[2];
    len[0] = _read_cmd_payload_len(cmd);
    len[1] = _read_msg_data(dst, len[0]);
    if (len[0] != len[1])
    {
        iprintln(trALWAYS, "#Error reading %d bytes for cmd 0x%02X (%d)", len[0], cmd, len[0]);
        dev_comms_response_add((master_command_t)cmd, btn_cmd_err_payload_len, len, 2);
        return false;
    }
    return true;//dev_comms_read_payload(dst, len);
}

uint8_t _read_cmd_payload_len(uint8_t cmd)
{
    switch (cmd)
    {
        case cmd_bcast_address_mask:
        case cmd_set_blink:
            return sizeof(uint32_t);

        case cmd_set_rgb_0:
        case cmd_set_rgb_1:
        case cmd_set_rgb_2:
            return  (3*sizeof(uint8_t));

//        case cmd_roll_call:
        case cmd_set_bitmask_index:
            return sizeof(uint8_t);

        case cmd_wr_console_cont: // The remainder of the data paylod belongs to this guy
        case cmd_wr_console_done:
            return  (msg.len - msg.rd_index);

        default:
            //Everything else
            return 0;
    }

}

void send_response(void)
{

    //This will also "clear" the response message if we are not in the roll-call state
    // int8_t tx_bus_state = dev_comms_tx_ready(); //Check if the bus is busy or not (-1 = error, 0 = busy, 1 = ready)

    switch (reg_state)
    {
        case roll_call:
            //Check if we need to send a response
            if (sys_stopwatch_ms_lap(&roll_call_sw) < roll_call_time_ms)// && (roll_call_sw.running == true))
                return; //Not yet, so wait a bit longer

            sys_stopwatch_ms_stop(&roll_call_sw); //Stop the stopwatch for the roll-call response

            //We wait until we are fairly certain that the bus is ready before we send our rollcall response
            // if (tx_bus_state == 0) //busy
            //     return;
            //We are going to try and send the response NOW!
            if(!dev_comms_response_start()) //Start the response message
            {
                //Could NOT send it now.... errors/bus is busy... we'll try again in a bit (between 2 and 50 ms)
                roll_call_time_ms = sys_random(2, 50);
                //RVN - TODO - This is a bit of a thumbsuck time period.... maybe come back to this later?
                sys_stopwatch_ms_start(&roll_call_sw, 0); //Restart the stopwatch for the rollcall response
                iprintln(trALWAYS, "#ROLL-CALL - wait some more (%u ms)", (uint8_t)roll_call_time_ms);
                return;
            }

            dev_comms_response_add(cmd_roll_call, btn_cmd_err_ok, NULL, 0);
            dev_comms_response_send();
            //Now we wait for the master to register us
            reg_state = waiting; //We are now in the "waiting for registration" state
            iprintln(trALWAYS, "#State: WAIT");
        
            /* Fall through to the next case to send the response message */
            
        case waiting:
        case idle:
            dev_comms_tx_service();
            break;
    
        case no_init:
        case un_reg:
        default:
            /* Do nothing... waiting for that precious rollcall */
            break; //This will prevent transmission of any messages until we are (at least)
    }
}

bool _rx_handler_rollcall_msg(uint8_t _cmd, uint8_t _src, uint8_t _dst)
{
    if (_cmd == cmd_roll_call)
    {
        if ((_src == ADDR_MASTER) && (_dst == ADDR_BROADCAST))
        {    
            //New ROLL-CALL command from the master, so we need to start the roll-call response timer again
            //RVN - TODO - NArrow the responsse window down a bit.
            //Means we will respond to the roll-call command between 20 and 2550 ms later
            roll_call_time_ms = (1+((uint32_t)dev_comms_addr_get())) * 10;
            iprintln(trALWAYS, "#ROLL-CALL - Answer in %lu ms", roll_call_time_ms);
            sys_stopwatch_ms_start(&roll_call_sw, 0); //Start the stopwatch for the roll-call response
            reg_state = roll_call; //We are in the process of responding to a roll-call
        }
    
        if ((_src != ADDR_MASTER) && (_dst == ADDR_MASTER))
        {
            //ROLL-CALL response from another node, so we blacklist its address...
            dev_comms_blacklist_add(_src);
    
            //... and generate a new address for ourselves if there is a conflict
            if (_src == dev_comms_addr_get())
                dev_comms_addr_new();            
        }
        return true; //Handled the roll-call message already 
    }
    return false;
}

bool _is_bcast_msg_for_me(uint32_t bit_mask)
{
    if ((my_mask_index < 0) || (my_mask_index > 31))
        return false; //Not registered yet or.... either way, this is not a valid address

    return ((bit_mask & (1 << my_mask_index)) == 0)? false : true;
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

#if (DEV_RGB_DEBUG == 1)
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
#endif /* DEV_RGB_DEBUG */

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

void _sys_handler_crc(void)
{
    char *argStr;// = console_arg_pop();
    char *tmp_mem_buffer = console_arg_peek(0);
    bool help_requested = console_arg_help_found();

    unsigned int bytes_to_parse = 0;
    unsigned int bytes_parsed = 0;

    uint8_t block_crc = 0;

    char * src;

    if (tmp_mem_buffer)
        tmp_mem_buffer--;

    while ((console_arg_cnt() > 0) && (!help_requested))
    {
        argStr = console_arg_pop();\

        if (is_hex_str(argStr,0))
        {
            //Remove leading "0x", "x", or "#" if present
            if ((argStr[0] == '0') && ((argStr[1] == 'x') || (argStr[1] == 'X')))
                src = argStr + 2;
            else if ((argStr[0] == 'x') || (argStr[0] == 'X') || (argStr[0] == '#'))
                src = argStr + 1;
            else
                src = argStr;                

            //How many bytes do we need to write?
            bytes_to_parse = strlen(src)/2;
            if (strlen(src) % 2)
                bytes_to_parse++; //In case the string length is odd

            //We start from the LSB (far right of the string)
            src += (strlen(src) - 2);
            while (bytes_to_parse > 0)
            {
                //Check if we have a valid hex char in the msb (this is only needed if the string length was odd)
                if (!isxdigit(*src))
                    *src = '0'; //If not, replace it with a '0'
                block_crc = crc8(block_crc, hex2byte(src));//_tmp_data[NVSTORE_BLOCK_DATA_SIZE-space_left] = hex2byte(src);
                tmp_mem_buffer[bytes_parsed] = hex2byte(src);
                bytes_parsed++;
                *src = 0;   //Null terminal the string right here
                src -= 2;   //Move src ptr back 2chars (1 byte)
                bytes_to_parse--;   
                //space_left--;   //Decrement the space left
            }
            continue;
        }
        //Is this a string?
        else if ((((argStr[0] == '\"') && (argStr[strlen(argStr)-1] == '\"'))   ||
                  ((argStr[0] == '\'') && (argStr[strlen(argStr)-1] == '\'')))  && (strlen(argStr) > 2))
        {
            //Remove quotes from string if present
            argStr[strlen(argStr)-1] = 0;
            src = argStr + 1;

            bytes_to_parse = strlen(src);

            block_crc = crc8_n(block_crc, (uint8_t*)src, bytes_to_parse);//strncpy((char *)&_tmp_data[NVSTORE_BLOCK_DATA_SIZE-space_left], argStr, space_left);
            memcpy((char *)&tmp_mem_buffer[bytes_parsed], src, bytes_to_parse);
            bytes_parsed += bytes_to_parse;
            //space_left -= bytes_to_parse;

            continue; // with the next argument
        }
        //Is this an integer value?
        else if (is_natural_number_str(argStr, 0))
        {
            int64_t _val = 0l;
            int64_t _max_val = INT8_MAX;
            int64_t _min_val = INT8_MIN;
            str2int64(&_val, argStr, 0);
            bytes_to_parse = 1;

            //We check for enough space....
            while (((_val > _max_val) || (_val < _min_val)))// && (bytes_to_parse < space_left))
            {
                //Doesn't fit into a n byte(s), so we need to add another byte
                bytes_to_parse++;
                ////Which will increase the range of max and min values
                _max_val = (_max_val << 8) | 0xFF;
                _min_val = (_min_val << 8) & 0x00;
            } 

            //Great, the number will fit into the space left
            while ((_val != 0))// && (space_left > 0))
            {
                block_crc = crc8(block_crc, (uint8_t)(_val & 0xFF));//_tmp_data[NVSTORE_BLOCK_DATA_SIZE-space_left] = (uint8_t)(_val & 0xFF);
                tmp_mem_buffer[bytes_parsed] = hex2byte(src);
                bytes_parsed++;
                _val >>= 8;
                //space_left--;
            }
            continue; // with the next argument
        }
        //else what is this????
        iprintln(trALWAYS, "Invalid argument \"%s\"", argStr);
        help_requested = true;
        break;
    }

    if (!help_requested)
    {
        if (bytes_parsed > 0)
        {
            iprintln(trALWAYS, "CRC: 0x%02X (%d bytes)", block_crc, bytes_parsed);
            console_print_ram(trALWAYS, tmp_mem_buffer, 0l, bytes_parsed);
        }
        else
            help_requested = true; //Disregard the rest of the arguments
    }

    if (help_requested)
    {
        iprintln(trALWAYS, " Usage: \"crc [<element_1> <element_2> ... <element_n>]\"");
#if REDUCE_CODESIZE==0        
        iprintln(trALWAYS, "    <element> - data elements as hex values (0x..), strings (\"..\") or integers");
        // iprintln(trALWAYS, "    <element> - up to %d data elements in any of the following formats:", NVSTORE_BLOCK_DATA_SIZE);
        // iprintln(trALWAYS, "              * hex value strings (preceded '0x...')");
        // iprintln(trALWAYS, "              * string(s) (enclosed in \"...\" or '...'");
        // iprintln(trALWAYS, "              * integer values  (e.g \"11 255 -71 23 1290 91\")");
        iprintln(trALWAYS, "     Multiple elements can be given, seperated by spaces, e.g \"write 11 0xff -7 '23' 1290\"");
        iprintln(trALWAYS, "     Hex and integer data is stored in big-endian format (i.e. LSB to MSB)");
        //                //          1         2         3         4         5         6         7         8         9
        //                //0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
#endif /* REDUCE_CODESIZE */
    }
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
