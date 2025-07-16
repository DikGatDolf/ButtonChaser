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
    un_reg,         //1Hz blink - We've not been registered yet
    roll_call,      //10 Hz blink - We are in the process of responding to a roll-call
    waiting,        //2.5 Hz blink - We are waiting on a registration from the master
    idle,           //dbg led off - We have been registered with the master (got a bit-mask address)
}registration_state_t;

/*******************************************************************************
Structures and unions
 *******************************************************************************/

/*******************************************************************************
 Function prototypes
 *******************************************************************************/

void blink_start(unsigned long _period_ms);
// void blink_pause(void);
// void blink_resume(void);
void blink_stop(void);
void blink_action(void);

void deactivate_button(uint8_t method);

void address_update(void);

void msg_process(void);
bool rollcall_msg_handler(uint8_t _cmd, uint8_t _src, uint8_t _dst);
void send_roll_call_response(void);
bool read_cmd_payload(uint8_t cmd, uint8_t * dst, uint8_t len);
uint8_t read_msg_data(uint8_t * dst, uint8_t len);

void state_machine_handler(void);
void _rx_handler_state_un_reg(uint8_t _cmd);

bool is_bcast_msg_for_me(uint32_t bit_mask);

void button_long_press(void);
void button_double_press(void);
void button_press(void);

void button_down(void);
//void button_release(void);

void dbg_led(dbg_blink_state_t state);
void tmr_debug_led(void);

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
#if REDUCE_CODESIZE==0
void _sys_handler_version(void);
#endif /* REDUCE_CODESIZE */



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
    {"crc",     _sys_handler_crc,       "CRC-8 calculator"},
#endif /* REDUCE_CODESIZE */
    {"ram",     _sys_handler_dump_ram,  "Display RAM"},
    {"flash",   _sys_handler_dump_flash,"Display FLASH"},
  #if REDUCE_CODESIZE==0
    {"version", _sys_handler_version,   "Displays app version"},
  #endif /* REDUCE_CODESIZE */
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

volatile unsigned long blink_period_ms;   // 1ms ~ 49 days.
registration_state_t reg_state = no_init; //We are not registered yet
int8_t my_mask_index = -1; //My address bit mask index, valid values 0 to 31

stopwatch_ms_s roll_call_sw;
stopwatch_ms_s sync_sw;
uint32_t roll_call_time_ms = 0; //The time we have to wait for the roll-call to finish

//bool response_msg_due = false;

uint8_t my_version = PROJECT_VERSION;

//uint8_t send_ack_nak = 0x00;

typedef struct {
    uint8_t src;    // The source address of the message
    uint8_t dst;    // The destination address of the message
    uint8_t data[RGB_BTN_MSG_MAX_LEN - sizeof(comms_msg_hdr_t) - sizeof(uint8_t)];
    uint8_t len;
    uint8_t rd_index;
} rx_msg_t;

rx_msg_t msg = {0};

stopwatch_ms_t reaction_time_sw;
unsigned long reaction_time_ms;

uint8_t system_flags = flag_unreg; //The state of the button (pressed or not pressed)

uint8_t dbg_led_state = 0; //0 = off, 1 = on, 2 = blinking fast, 3 = blinking slow
bool dbg_led_output; //The output state for the debug LED pin

uint32_t time_ms_offset = 0lu; //The time offset for the system time (in ms)
/*******************************************************************************
 Functions
 *******************************************************************************/

void setup() {

    // put your setup code here, to run once:
 
    dev_comms_init();

#if REDUCE_CODESIZE==0
    _sys_handler_version();
#endif /* REDUCE_CODESIZE */
    console_add_menu("main", _main_menu_items, ARRAY_SIZE(_main_menu_items), "Main Menu");

    dev_nvstore_init();

    //If this is the first run of this firmware, we need to save the comms address
    if (!dev_nvstore_new_data_available())
    {
        uint8_t current_addr = dev_comms_addr_get();
        iprintln(trMAIN, "#1st run - address: 0x%02X", current_addr);
        dev_nvstore_write(&current_addr, sizeof(uint8_t));                
    }
   
    dev_rgb_start(output_Led_Red, output_Led_Green, output_Led_Blue);

    reaction_time_ms = 0lu;
    sys_stopwatch_ms_stop(&reaction_time_sw);

    dev_button_init(button_down, NULL /*button_release*/, button_press, button_long_press, button_double_press);

    colour[0].rgb = (uint32_t)colYellow;
    colour[1].rgb = (uint32_t)colBlue;
    colour[2].rgb = (uint32_t)colTeal;

    blink_stop(); //Make sure we are NOT blinking

    address_update();

    sys_set_io_mode(output_DEBUG_LED, OUTPUT);
    sys_output_write(output_DEBUG_LED, LOW);
    sys_cb_tmr_start(&tmr_debug_led, 5000lu);

    reg_state = un_reg; //We are not registered yet
    system_flags |= flag_unreg;
    dbg_led(dbg_led_blink_500ms); //Start blinking the debug LED at 200ms intervals

    //This can only be set by the master device
    my_mask_index = -1;

    //Testing only
    // sys_poll_tmr_start(&blink_tmr, 200lu, true);

    //sys_poll_tmr_stop(&debug_tmr);
    // _tmr_expired = false;
}

void dbg_led(dbg_blink_state_t state)
{
    if (dbg_led_state == state)
        return; //No change, so do nothing

    sys_cb_tmr_stop(tmr_debug_led);
    dbg_led_output = ((state == dbg_led_off)? LOW : HIGH);
    sys_output_write(output_DEBUG_LED, dbg_led_output);
    dbg_led_state = state;

    switch (state)
    {
        case dbg_led_off:
        case dbg_led_on:
            break;
        case dbg_led_blink_50ms:
            sys_cb_tmr_start(tmr_debug_led, 50lu, true);
            break;
        case dbg_led_blink_200ms:
            sys_cb_tmr_start(tmr_debug_led, 200lu, true);
            break;
        case dbg_led_blink_500ms:
            sys_cb_tmr_start(tmr_debug_led, 500lu, true);
            break;
        default:
            //Do nothing
            break;
    }
}

void tmr_debug_led() 
{
    //IMPORTANT: DO NOT CALL THIS FUNCTION DIRECTLY FROM ANYWHERE ELSE!
    // This function is called from the timer ISR, so it must be quick and not block!
    dbg_led_output = !(dbg_led_output);
    sys_output_write(output_DEBUG_LED, dbg_led_output);
}

void loop() {

    // if (_tmr_expired)
    // {
    //     _tmr_expired = false;
    //     iprintln(trALWAYS, "#Timer expired");
    //     sys_cb_tmr_start(&tmr_debug, 2000lu);
    // }

    // put your main code here, to run repeatedly:

    dev_button_service(); //Check if the button has been pressed and handle it

    //Check if our communication address has changed
    address_update();

	//Check for anything we want to send or may have received on the main comms channnel...
    msg_process(); //... process it

    //Handle the state machine  transitions for the device
    state_machine_handler(); //RVN - TODO - This might not be needed at all?
  
    // //Check for anything coming in on the Serial Port and process it
    console_service();
}

void button_long_press(void)
{
    system_flags |= flag_l_press;
    iprintln(trMAIN, "#Btn: Long Press");
#if DEBUG_LED_ACTIONS == 1
    sys_stopwatch_ms_start(&reaction_time_sw);
    blink_stop();
    dev_rgb_set_colour(colRed);
#endif    
}

void button_double_press(void)
{
    system_flags |= flag_d_press;
    iprintln(trMAIN, "#Btn: Dbl Press");
#if DEBUG_LED_ACTIONS == 1
    if (blink_period_ms > 0)
    {
        //Stop the blinking timer, as we are not interested in it anymore
        blink_stop();
        dev_rgb_set_colour(colBlack); //Turn off the LED
    }
    else //Start the blinking timer
        blink_start(200lu); //Start blinking at 200ms intervals
#endif    
}

void button_press(void)
{
    system_flags |= flag_s_press;
    iprintln(trMAIN, "#Btn: Short Press");
}

void button_down(void)
{
    //Button is down
    deactivate_button(flag_sw_stopped);
    iprintln(trMAIN, "#Btn: Down");
}

void deactivate_button(uint8_t method)
{
    //If we are measuring the reaction time, get the elapsed time now
    if (reaction_time_sw.running)
    {
        reaction_time_ms = sys_stopwatch_ms_stop(&reaction_time_sw);
        system_flags |= method;
        blink_stop(); //Stop blinking if we are measuring the reaction time
        dev_rgb_set_colour(colour[2].rgb); //Set the tertiary colour as the new colour
        iprintln(trMAIN, "#Time: %lu ms (%d)", reaction_time_ms, method);
    }
}

// void button_release(void)
// {
//     iprintln(trMAIN, "#Btn: Release");
// }

void blink_start(unsigned long _period_ms)
{
    blink_period_ms = _period_ms;
    if (_period_ms > 0)
    {
        system_flags |= flag_blinking; //Clear the blinking flag
        sys_cb_tmr_start(blink_action, blink_period_ms, true);
    }
    else
        blink_stop();
}

// void blink_pause(void)
// {
//     //If we are not blinking, this will have no effect
//     system_flags &= ~flag_blinking; //Clear the blinking flag
//     sys_cb_tmr_stop(blink_action);
// }

// void blink_resume(void)
// {
//     //If we are not blinking, this will have no effect
//     if (blink_period_ms > 0)
//     {
//         system_flags |= flag_blinking; //Clear the blinking flag
//         sys_cb_tmr_start(blink_action, blink_period_ms, true);
//     }
// }

void blink_stop(void)
{
    system_flags &= ~flag_blinking; //Clear the blinking flag
    blink_period_ms = 0lu;
    sys_cb_tmr_stop(blink_action);
}

void blink_action(void)
{
    //Swop the colours on the RGB LED
    SWOP_U32(colour[0].rgb, colour[1].rgb);
    //Set the new colour
    dev_rgb_set_colour(colour[0].rgb);
    //Restart the timer    
    if (blink_period_ms == 0)
        sys_cb_tmr_stop(blink_action); // will self-destruct after exiting this function
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
    //We probably need to make sure that we are not reading an invalid address from the NV store
    if (dev_comms_verify_addr(stored_addr) == false)
    {
        iprintln(trCOMMS, "#Invalid address read: 0x%02X", stored_addr);
        //Write the current address back to the NV store and restart
        dev_nvstore_write(&current_addr, sizeof(uint8_t)); 
        while (1)
        {    
            resetFunc();
        }
        return; //We cannot use this address, so we will not change the current address
    }

    //Check if the address in the NV store is different from the one we are using
    if (stored_addr == current_addr)
        return; //No change

    dev_comms_addr_set(stored_addr);
    iprintln(trCOMMS, "#Address change: 0x%02X -> 0x%02X", current_addr, dev_comms_addr_get());
    //Reset the comms address to that which we got from the NVstore
}

void msg_process(void)
{
    uint8_t _u8_val = 0;
    uint16_t _u16_val = 0;
    uint32_t _u32_val = 0;
    float _fl_val = 0.0f;
    uint8_t _cnt = 0;
    bool accept_msg = false; //We start off NOT accepting broadcast messages (yet)
    master_command_t _cmd;
    uint8_t _myAddr = dev_comms_addr_get();
    bool _can_respond = false; //We are not processing a broadcast message yet
   
    //If anything requires sending, now is the time to do it
    if (reg_state == roll_call)
        send_roll_call_response();
        //Fall through to ensure we read the other nodes' responses to populate our blacklist

    //Have we received anything?
    msg.len = dev_comms_rx_msg_available(&msg.src, &msg.dst, msg.data);
    if (msg.len == 0)
        return; //Nothing to process

    msg.rd_index = 0; //Set the read pointer to the start of the data array

    if (msg.dst == _myAddr) 
        _can_respond = true; //We are accepting messages addressed to us

    //Have we received anything?
    while (read_msg_data((uint8_t *)&_cmd, 1) > 0)
    {
        //Check for a roll-call first
        if (rollcall_msg_handler(_cmd, msg.src, msg.dst))
            return; //Handled the roll-call message already (or ignored it)

        if (msg.src != ADDR_MASTER)
            return;//   //Going forward, we only care about messages from the master device
        
        //We are accepting direct messages IF we are WAITING or IDLE
        if (_can_respond) 
            accept_msg = (reg_state >= waiting)? true : false;

        //If we are not accepting messages, we need to check if this is a broadcast message
        if (msg.dst == ADDR_BROADCAST)
        {
            //Even for broadcast msgs there are limits... e.g. no information can be "READ", and no console commands can be executed
            if (_cmd >= cmd_set_bitmask_index)
            {
                iprintln(trALWAYS, "!Invalid bcst (0x%02X)", _cmd);
                return; //We cannot process this message... and since it is a broadcast, we don't want to respond
            }
        }

        //If the accept_msg flag is still false (i.e a message  NOT for our address) after processing the 1st command in the message, then we can stop processing the rest of the message
        if ((!accept_msg) && ((_cnt > 0) || (_cmd != cmd_bcast_address_mask)))
        {
            //Not really an error....
            //iprintln(trALWAYS, "!Msg not for us (0x%02X)", msg.dst);
            return;
        }
        
        // if (reg_state != idle)
        //     continue; //We are not in the idle state, so we don't care about anything else
        
        //iprintln(trALWAYS, "#Parsing: 0x%02X for 0x%02X", _cmd, msg.dst);

        switch (_cmd)
        {
            case cmd_bcast_address_mask:
            {
                if (read_cmd_payload(_cmd, (uint8_t *)&_u32_val, sizeof(uint32_t)))
                {
                    //iprint(trALWAYS, "#BCST 0x%08X & 0x%08X : ", BIT_POS(my_mask_index), _u32_val);
                    //We need to check if the broadcast message applies to us or not
                    accept_msg = is_bcast_msg_for_me(_u32_val)? true : false;
                    //iprintln(trALWAYS, "%s", accept_msg? "YES" : "NO");
                }
                //else //read failure already handled in read_cmd_payload()
                break;
            }
                        
            case cmd_set_rgb_0:
            case cmd_set_rgb_1:
            case cmd_set_rgb_2:
            {
                if (read_cmd_payload(_cmd, (uint8_t *)&_u32_val, (3*sizeof(uint8_t))))
                {
                    //If we are not blinking, this will have no effect
                    sys_cb_tmr_stop(blink_action);

                    //blink_pause(); //pause blinking if we are changing LED colours
                    colour[_cmd - cmd_set_rgb_0].rgb = _u32_val;
                    //If we are changing the primary colour, we need to set it now
                    if (_cmd == cmd_set_rgb_0)
                        dev_rgb_set_colour(colour[0].rgb);

                    //If we are not blinking, this will have no effect
                    if (blink_period_ms > 0)
                        sys_cb_tmr_start(blink_action, blink_period_ms, true);

                    if (_can_respond) 
                        dev_comms_response_append(_cmd, resp_ok, NULL, 0);
                }
                //else //read failure already handled in read_cmd_payload()
                break;
            }
            
            case cmd_set_blink:
            {
                if (read_cmd_payload(_cmd, (uint8_t *)&_u32_val, sizeof(uint32_t)))
                {
                    blink_stop(); //Stop blinking if we are measuring the reaction time
                    if (_u32_val > 0) //Only restart the timer if the period is > 0
                        blink_start(_u32_val); //Start blinking at the specified intervals
                    if (_can_respond) 
                        dev_comms_response_append(_cmd, resp_ok, NULL, 0);
                }
                break;
            }   
            
            case cmd_set_switch:
            {
                if (read_cmd_payload(_cmd, &_u8_val, sizeof(uint8_t)))
                {
                    if (_u8_val > 1)
                    {
                        iprintln(trALWAYS, "#Invalid value (%d > %d)", _u8_val, 1);
                        if (_can_respond) 
                        {
                            _u16_val = (0x0001) | (_u8_val << 8);
                            dev_comms_response_append(_cmd, resp_err_range, (uint8_t *)&_u16_val, sizeof(uint16_t));
                        }
                        break; // from switch... continue with the next command
                    }
                    else if (_u8_val == 1)
                    {
                        system_flags |= flag_activated;
                        reaction_time_ms = 0lu;
                        sys_stopwatch_ms_start(&reaction_time_sw);
                    }
                    else //if (_u8_val == 0)
                    {
                        //Stop measuring the reaction time
                        //if the stopwatch was running, we return the elapsed time.
                        deactivate_button(flag_deactivated);
                    }
                    if (_can_respond) 
                        dev_comms_response_append(_cmd, resp_ok, NULL, 0);
                }
                break;
            }
            
            case cmd_set_dbg_led:
            {
                if (read_cmd_payload(_cmd, (uint8_t*)&_u8_val, sizeof(uint8_t)))
                {
                    if (_u8_val >= dbg_led_state_limit)
                    {
                        iprintln(trALWAYS, "#Invalid index (%d > %d)", _u8_val, RGB_BTN_MAX_NODES);
                        if (_can_respond)
                        {
                            _u16_val = (0x00FF & (dbg_led_state_limit - 1)) | (_u8_val << 8);
                            dev_comms_response_append(_cmd, resp_err_range, (uint8_t *)&_u16_val, sizeof(uint16_t));
                        }
                        break;
                    }
                    dbg_led((dbg_blink_state_t)_u8_val); //Set the debug LED state
                    if (_can_respond) 
                        dev_comms_response_append(_cmd, resp_ok, NULL, 0);
                }
                //else //read failure already handled in read_cmd_payload()
                break;
            }

            case cmd_set_time:
            {
                if (read_cmd_payload(_cmd, (uint8_t *)&_u32_val, sizeof(uint32_t)))
                {
                    time_ms_offset = sys_millis() - _u32_val; //Set the time offset to the specified value
                    if (_can_respond)
                        dev_comms_response_append(_cmd, resp_ok, NULL, 0);
                }
                break;
            }   

            case cmd_set_sync:
            {
                if (read_cmd_payload(_cmd, (uint8_t *)&_u32_val, sizeof(uint32_t)))
                {
                    if (_u32_val == 0xFFFFFFFF)
                    {
                        //This forces a reset of the correction value
                        sys_time_correction_factor_reset(); //Set the time correction factor
                        iprintln(trALWAYS, "#Correction Factor reset");
                    }
                    else if (_u32_val == 0)
                    {
                        //This indicates the start of the sync process
                        sys_stopwatch_ms_start(&sync_sw); //Start the stopwatch for the sync process
                        iprintln(trALWAYS, "#Sync started");
                    }
                    else
                    {
                        char buff[16];
                        if (!sync_sw.running)
                        {
                            iprintln(trALWAYS, "#Not Sync'ing");
                            if (_can_respond) 
                            {
                                _u8_val = 0x01; //Indicate that we are not syncing
                                dev_comms_response_append(_cmd, resp_err_reject_cmd, (uint8_t *)&_u8_val, sizeof(uint8_t));
                            }
                            break;
                        }
                        uint32_t my_elapsed_time_ms = sys_stopwatch_ms_stop(&sync_sw); //Stop the stopwatch for the sync process
                        iprintln(trALWAYS, "#Sync stopped after %lu ms (%lu ms)", my_elapsed_time_ms, _u32_val);
                        //This is now the end of the sync process.... the value is the time in milliseconds measured by the master
                        _fl_val = (float)_u32_val / (float)my_elapsed_time_ms; //Calculate the time correction factor
                        sys_time_correction_factor_set(_fl_val); //Set the time correction factor
                        iprintln(trALWAYS, "#Time correction factor: %s", float2str(buff, (double)_fl_val, 8, 16));
                    }
                    if (_can_respond)
                        dev_comms_response_append(_cmd, resp_ok, NULL, 0);
                }
                break;
            }

            case cmd_set_bitmask_index:
            {
                //We are being registered by the master, so we need to set our address bit mask
                if (read_cmd_payload(_cmd, &_u8_val, sizeof(uint8_t)))
                {
                    if (((int8_t)_u8_val < 0) || ((int8_t)_u8_val > RGB_BTN_MAX_NODES))
                    {
                        iprintln(trALWAYS, "#Invalid index (%d > %d)", _u8_val, RGB_BTN_MAX_NODES);
                        _u16_val = (0x00FF & RGB_BTN_MAX_NODES) | (_u8_val << 8);
                        dev_comms_response_append(_cmd, resp_err_range, (uint8_t *)&_u16_val, sizeof(uint16_t));
                        break; // from switch... continue with the next command
                    }
                    my_mask_index = (int8_t)_u8_val; //Set the address bit mask for this device
                    iprintln(trALWAYS, "#State: IDLE (Index: %d)", my_mask_index);
                    reg_state = idle; //We are now in the "idle" state
                    dbg_led(dbg_led_off); //Turn off the debug LED blinking
                    dev_comms_response_append(_cmd, resp_ok, NULL, 0);
                    // sys_poll_tmr_start(&debug_tmr, 1000lu, true);
                }
                //else //read failure already handled in read_cmd_payload()
                break;
            }

            case cmd_new_add:
            {
                //The master is assigning a new address to us, so we need to set it
                if (read_cmd_payload(_cmd, (uint8_t *)&_u8_val, sizeof(uint8_t)))
                {
#if (DEV_COMMS_DEBUG == 1)
                    //Special case: 0xFF is used to indicate that the master wants us to reset our address
                    if ((_u8_val == 0xFF) && (msg.dst == ADDR_BROADCAST))
                    {
                        //This is a broadcast message that does not get a response
                        //Start by resetting the blacklist of addresses
                        dev_comms_blacklist_clear();
                        _u8_val = dev_comms_addr_new();
                        dev_nvstore_write(&_u8_val, sizeof(uint8_t)); //Save the new address to the NV store
                        //Reset right now
                        iprintln(trALWAYS, "#Resetting device (new address: 0x%02X)", _u8_val);
                        console_flush(); //Flush the console output before resetting
                        resetFunc(); //This will reset the device and re-initialise the comms
                    }
#endif
                    if (!dev_comms_verify_addr(_u8_val))
                    {
                        iprintln(trALWAYS, "#Invalid address (%d)", _u8_val);
                        _u16_val = (0x00FF & dev_comms_addr_get() ) | (_u8_val << 8);
                        dev_comms_response_append(_cmd, resp_err_range, (uint8_t *)&_u16_val, sizeof(uint16_t));
                        break;
                    }
                    dev_nvstore_write(&_u8_val, sizeof(uint8_t)); //Save the new address to the NV store
                    dev_comms_response_append(_cmd, resp_ok, NULL, 0);
                    //This response will still be sent using the old address... it will be updated in the next loop iteration
                    iprintln(trALWAYS, "#New Address set from Master: 0x%02X", _u8_val);
                }
                //else //read failure already handled in read_cmd_payload()
                break;
            }
            
            case cmd_get_rgb_0:
            case cmd_get_rgb_1:
            case cmd_get_rgb_2:
            {
                _u32_val = colour[_cmd-cmd_get_rgb_0].rgb;
                dev_comms_response_append(_cmd, resp_ok, (uint8_t*)&_u32_val, (3*sizeof(uint8_t)));
                break;
            }

            case cmd_get_blink:
            {
                _u32_val = blink_period_ms; //Get the period of the timer
                dev_comms_response_append(_cmd, resp_ok, (uint8_t*)&_u32_val, sizeof(uint32_t));
                break;
            }

            case cmd_get_reaction:
            {
                //We're not even bothering with reading a payload....
                _u32_val = reaction_time_ms;
                dev_comms_response_append(_cmd, resp_ok, (uint8_t*)&_u32_val, sizeof(uint32_t));
                break;
            }
            
            case cmd_get_flags:
            {
                //We're not even bothering with reading a payload....
                //read_cmd_payload(_cmd, NULL);   // No data to read
                dev_comms_response_append(_cmd, resp_ok, (uint8_t*)&system_flags, sizeof(uint8_t));
                //Now we can clear the edge-detect flags that we have read
                system_flags &= ~(flag_s_press | flag_l_press | flag_d_press | flag_activated | flag_deactivated | flag_sw_stopped);
                break;
            }

            case cmd_get_dbg_led:
            {
                dev_comms_response_append(_cmd, resp_ok, (uint8_t*)&dbg_led_state, sizeof(uint8_t));
                break;
            }

            case cmd_get_time:
            {
                //We're not even bothering with reading a payload....
                _u32_val = sys_millis() - time_ms_offset; //Get the current time in milliseconds
                dev_comms_response_append(_cmd, resp_ok, (uint8_t*)&_u32_val, sizeof(uint32_t));
                break;
            }

            case cmd_get_sync:
            {
                //We're not even bothering with reading a payload....
                _fl_val = sys_time_correction_factor(); //Get the current time correction factor
                dev_comms_response_append(_cmd, resp_ok, (uint8_t*)&_fl_val, sizeof(float));
                break;
            }

#if REMOTE_CONSOLE_SUPPORTED == 1    
            case cmd_wr_console_cont:
            case cmd_wr_console_done:
            {
                uint8_t _data[RGB_BTN_MSG_MAX_LEN - sizeof(comms_msg_hdr_t) - sizeof(uint8_t)];
                memset(_data, 0, sizeof(_data));
                if (read_cmd_payload(_cmd, _data, (msg.len - msg.rd_index)))
                {
                    for (int8_t i = 0; (i < ((int8_t)sizeof(_data))) && (_data[i] > 0); i++)
                        console_read_byte(_data[i]);
                    if (_cmd == cmd_wr_console_done)
                    {
                        //iprintln(trALWAYS, "#Console: \"%s\"", _data);
                        //Maybe wise to turn OFF all COMMS and HAL_SERIAL traces while this is happening?
                        console_enable_alt_output_stream(dev_comms_response_add_byte);
                        dev_comms_response_append(cmd_wr_console_cont, resp_ok, NULL, 0);
                        console_read_byte('\n');
                        console_service(); //This *should* pipe the output to the alternate stream and reset it back to stdout once it is done
                        if (!dev_comms_transmit_now()) //Send the response message now
                            iprintln(trALWAYS, "!Sending console response msg");
                        dev_comms_response_append(cmd_wr_console_done, resp_ok, NULL, 0);
                    }
                }
                //else //read failure already handled in read_cmd_payload()
                break;
            }
#endif /* REMOTE_CONSOLE_SUPPORTED */

            // case cmd_roll_call_all:             //Already handled in rollcall_msg_handler()
            default:
            {
                iprintln(trALWAYS, "!Unknown Cmd: 0x%02X", _cmd);
                //Unkown/unhandled command.... return error (NAK?) to master
                if (_can_respond)
                    dev_comms_response_append(_cmd, resp_err_unknown_cmd, NULL, 0);
                break;
            }
        }
        _cnt++;

    }

    //If we have a response ready for a direct message, we need to send it immediately, (without waiting for bus silence)
    if ((msg.src == ADDR_MASTER) && (_can_respond) && (reg_state >= waiting))
        if (!dev_comms_transmit_now()) //Send the response message now
            iprintln(trALWAYS, "!Tx response");
}

uint8_t read_msg_data(uint8_t * dst, uint8_t len)
{
    uint8_t data_len = min(msg.len, sizeof(msg.data));

    // iprintln(trMAIN, "data_rd_index = %d", _comms.rx.data_rd_index);
    // iprintln(trMAIN, "data_length   = %d", _comms.rx.data_length);
    if (len == 0)
        return 0; //Nothing to do

    //iprintln(trCOMMS, "#Rd %d from %d/%d", len, msg.rd_index, data_len);

    if (msg.rd_index >= data_len)
        return 0; //No data available

    if (len > (data_len - msg.rd_index))
        return 0; //Requesting more data than available

    if ((dst) && (len > 0))// Don't copy to NULL
        memcpy(dst, &msg.data[msg.rd_index], len);
    

    msg.rd_index += len;

    return len;
}

bool read_cmd_payload(uint8_t cmd, uint8_t * dst, uint8_t len)
{
    if (read_msg_data(dst, len) != len)
    {
        iprintln(trALWAYS, "!Rd %d bytes for cmd 0x%02X", len, cmd);
        dev_comms_response_append((master_command_t)cmd, resp_err_payload_len, &len, 1);
        return false;
    }
    return true;//dev_comms_read_payload(dst, len);
}

void send_roll_call_response(void)
{
    //function is only called in roll_call state, so this is not neeeded
    // if (reg_state != roll_call)
    //     return; //We are not in the roll-call state, so we don't need to send a response

    if (sys_stopwatch_ms_lap(&roll_call_sw) < roll_call_time_ms)// && (roll_call_sw.running == true))
        return; //Not yet, so wait a bit longer

    sys_stopwatch_ms_stop(&roll_call_sw); //Stop the stopwatch for the roll-call response

    //We wait until we are fairly certain that the bus is ready before we send our rollcall response
    // if (tx_bus_state == 0) //busy
    //     return;
    //We are going to try and send the response NOW!
    if(!dev_comms_tx_ready()) //Start the response message
    {
        //Could NOT send it now.... errors/bus is busy... we'll try again in a bit (between 2 and 50 ms)
        roll_call_time_ms = sys_random(BUS_SILENCE_MIN_MS, BUS_SILENCE_MIN_MS*10);
        //RVN - TODO - This is a bit of a thumbsuck time period.... maybe come back to this later?
        sys_stopwatch_ms_start(&roll_call_sw, 0); //Restart the stopwatch for the rollcall response
        iprintln(trALWAYS, "#ROLL-CALL - wait some more (%u ms)", (uint8_t)roll_call_time_ms);
        return;
    }

    dev_comms_response_append(cmd_roll_call, resp_ok, &my_version, 1, true);
    if (dev_comms_transmit_now()) //Send the response message now
    {
        //Now we wait for the master to register us
        reg_state = waiting; //We are now in the "waiting for registration" state
        iprintln(trALWAYS, "#State: WAIT");
        dbg_led(dbg_led_blink_200ms); //Start blinking the debug LED at 200ms intervals
    }
    else
    {
        iprintln(trALWAYS, "!Tx roll-call response");
        reg_state = un_reg; //RVN - TODO - we need to go back to the state we were in before the roll-call started (no necessarily un_reg)
        dbg_led(dbg_led_blink_500ms); //Start blinking the debug LED at 500ms intervals
    }
}

void state_machine_handler(void)
{

    //This will also "clear" the response message if we are not in the roll-call state
    // int8_t tx_bus_state = dev_comms_tx_ready(); //Check if the bus is busy or not (-1 = error, 0 = busy, 1 = ready)

    switch (reg_state)
    {
        case no_init:
        case un_reg:
        case roll_call:
        case waiting:
            system_flags |= flag_unreg;
            break;

        case idle:
            system_flags &= ~flag_unreg;
            break;
    
        default:
            /* Do nothing... waiting for that precious rollcall msg*/
            break; //This will prevent transmission of any messages until we are (at least)
    }
    
    //dev_comms_check_error();
}

bool rollcall_msg_handler(uint8_t _cmd, uint8_t _src, uint8_t _dst)
{
    uint8_t _u8_val = 0;
    if (_cmd != cmd_roll_call)
        return false; //Not a roll-call message, so we don't care about it

    //Was this a roll-call message from the master device?
    if ((_src == ADDR_MASTER) && (_dst == ADDR_BROADCAST))
    {    
        //Now we need to read the payload of the roll-call message
        if (read_cmd_payload(_cmd, &_u8_val, sizeof(uint8_t)))
        {
            //The payload of the roll-call message is a single byte, which can be:
            // 0 - Roll call for ALL devices on the bus
            // 1 - Roll call for UNREGISTERED devices only
            if (_u8_val != 0)   // Roll call for unregistered devices only
            {
                // If we are registered and this was meant for unregistered devices, we do NOT respond
                if (reg_state == idle)
                {
                    iprintln(trALWAYS, "#ROLL-CALL Ignored (registered)");
                    return true; //Handled the roll-call message already (ignored it)
                }
            }
            else// if (_u8_val == 1)   // Roll call for ALL devices
            {
                //Clear the blacklist of addresses, as we are starting a new roll-call
                dev_comms_blacklist_clear();

            }
        }
        else //read failure already handled in read_cmd_payload()
        {
            iprintln(trALWAYS, "!Tx roll-call");
            return true; //Handled the roll-call message already (ignored it)
        }        
        //New ROLL-CALL command from the master, so we need to start the roll-call response timer (again?)
        //The random time is based on our address, so that we don't all respond at the same time
        //A random jitter is added between 0 and 255ms to try to eliminate/reduce the probability of bus 
        //  collisions between nodes with the same addresses.
        //Means we will respond to the roll-call command between 10 and 2795 ms later
        roll_call_time_ms = (((uint32_t)dev_comms_addr_get() * 2 * BUS_SILENCE_MIN_MS)) + sys_random(0, 0xFF);

        /*With a maximum of 31 nodes on the bus, and a randomly generated address between 1 and 254, the 
           probability of a collision between two devices is ~31.4%
          
          See https://en.wikipedia.org/wiki/Birthday_problem#Generalizations for more on this.

          We add another randomly generated time jitter (0 to 255ms) to the response time (Address × 10 ms) to 
           reduce the probability of collisions further.

          We are not really concerned about collisions between devices with DIFFERENT addresses, 
           as these will be detected by the device and then just retried at a later (randomly generated) time
         Our concern is collisions between devices with the SAME address, which will NOT be detected by the devices or the master.

          With a 173.6 µs minimum safe spacing (twice the byte length @ 115200 baud), we can now estimate timing-based collisions with more precision. 
            Here's a breakdown for both scenarios:

            ===== 1 - Probability of a Collision Between ANY Two Nodes =====
            Each node's response time is:
            T = (Address × 10 ms) + jitter

            Where:
            - Address ∈ [1, 254]
            - Jitter ∈ [0, 255 ms], uniformly distributed in 1 ms increments
            - Response time spread per address: [A×10 ms, A×10 ms + 255 ms]
            So every node's response is in a 256 ms-wide window, shifted by 10 ms per address.
            Now let’s get concrete:
            There are 31 nodes, so you have:
            - 465 unique node pairs = C(31, 2)
            Assuming all response times are uniformly random across a domain of [10 ms, 2540 ms + 255 ms] ≈ [10 ms, 2795 ms], we discretize this into 173.6 µs bins:
            # of bins ≈ (2795 ms − 10 ms) / 0.1736 ms ≈ 16,047 bins

            Each response lands uniformly into one of these bins. So the probability that a second node lands in the same bin as the first is:
            P(pairwise collision) ≈ 1 / 16,047

            With 465 pairs:
            P(any collision) ≈ 1 − (1 − 1/16,047)^465 ≈ 2.82%


            So:
            - P(any bus timing collision) ≈ 2.8%

            ===== 2 - Probability of a Collision from Nodes with the SAME Address AND SAME Jitter =====
            As before, we combine:
            - P(same address) — for 31 random draws from 254 values: ≈ 31.4%
            - P(same jitter) — uniform 0–255 → 1/256
            - P(timing overlap) — they will definitely land in the same 173.6 µs bin if they have the same base time and jitter
            So, collision probability for a specific address-duplicate pair:
            P = P(same address) × P(same jitter) ≈ 0.314 × 1/256 ≈ 0.123%


            But not every node pair collides on address — only a small subset does. 
            We could refine this more with the expected number of duplicate-address pairs, but even with 31 nodes:
            - You’re likely to have ~1 pair with a shared address (on average),
            - So P(collision due to same address + same jitter) ≈ 0.1–0.2%
            This is a very low probability, and it’s acceptable for our use case.        

            TL;DR
            +------------------------------------------------------------+-------------+
            | Event Type                                                 | Estimated   |
            |                                                            | Probability | 
            +------------------------------------------------------------+-------------+
            | Any two nodes overlap in response timing (≤173.6 µs apart) |     ~2.8%   | 
            | Two nodes overlap due to same address and same jitter      | ~0.1–0.2%   | 
            +------------------------------------------------------------+-------------+

            This is only appliccable to Rollcall responses.... once registered, the devices 
             will be either addressed individually by the master (if requiring responses) or a 
             broadcast to all devices will be sent (requiring no responses).
        */

        iprintln(trALWAYS, "#ROLL-CALL - Answer in %lu ms", roll_call_time_ms);
        sys_stopwatch_ms_start(&roll_call_sw, 0); //Start the stopwatch for the roll-call response
        reg_state = roll_call; //We are in the process of responding to a roll-call
        dbg_led(dbg_led_blink_50ms); //Start blinking the debug LED at 50ms intervals
    }

    // or was this a roll-call response from another device?
    if ((_src != ADDR_MASTER) && (_dst == ADDR_MASTER))
    {
        //ROLL-CALL response from another node, so we blacklist its address...
        dev_comms_blacklist_add(_src);

        //... and generate a new address for ourselves if there is a conflict and we have not responded with our address yet
        if ((reg_state != waiting) && (reg_state != idle) && (_src == dev_comms_addr_get()))
        {
            uint8_t new_addr = dev_comms_addr_new();
            iprintln(trALWAYS, "#Address conflict 0x%02X -> 0x%02X", _src, new_addr);
            dev_nvstore_write(&new_addr, sizeof(uint8_t)); //Save the new address to the NV store
        }
    }
    return true; //Handled the roll-call message already 
}

bool is_bcast_msg_for_me(uint32_t bit_mask)
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
	iprintln(trALWAYS, "Running for %lu s", ((long)sys_millis() / 1000));
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

#if REDUCE_CODESIZE==0
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
#endif /* REDUCE_CODESIZE */

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
