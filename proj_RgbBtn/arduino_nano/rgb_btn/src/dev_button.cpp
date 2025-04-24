/******************************************************************************
Project:    RGB Button Chaser
Module:     dev_button.c
Purpose:    This file contains the routines for the button input
Author:     Rudolph van Niekerk
Processor:  Arduino Nano (ATmega328)
Compiler:	Arduino AVR Compiler

 ******************************************************************************/

#include "Arduino.h"

#define __NOT_EXTERN__
#include "dev_button.h"
#undef __NOT_EXTERN__

#include "dev_console.h"
#include "hal_timers.h"

/******************************************************************************
Macros
******************************************************************************/
#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("Button") /* This must be undefined at the end of the file*/

#define BTN_DEBOUNCE_TIME_MS       (50)	/* Debounce time in ms */
#define BTN_LONG_PRESS_TIME_MS     (500)	/* Btn Dwn hold time to trigger a long-press event */
#define BTN_DBL_PRESS_TIME_MS      (650)	/* Period within which 2 btn presses will register a double-press */

/******************************************************************************
Structs and Unions
******************************************************************************/
enum button_event_e
{
    btn_down        = BIT_POS(0),   // Falling Edge
    btn_release     = BIT_POS(1),   // Rising Edge
    btn_long_press  = BIT_POS(2),   // Implies Release
    btn_shrt_press  = BIT_POS(3),   // Implies Release
    btn_dbl_press   = BIT_POS(4),   // Implies Short Press and Release
};

/******************************************************************************
Local function definitions
******************************************************************************/
void _debounce_cb(void);
void _dbl_press_cb(void);
void _long_press_cb(void);

/******************************************************************************
Local variables
******************************************************************************/

volatile uint8_t button_event;
volatile uint8_t _last_pin_state = HIGH;
volatile bool _dbl_press_flag = false;
volatile bool _long_press_flag = false;


void (*_btn_down_cb)(void) = NULL;
void (*_btn_release_cb)(void) = NULL;
void (*_btn_short_press_cb)(void) = NULL;
void (*_btn_long_press_cb)(void) = NULL;
void (*_btn_dbl_press_cb)(void) = NULL;

/******************************************************************************
Local functions
******************************************************************************/

ISR(INT0_vect) // INT0
{
    //Just start the debounce timer, and let the timer ISR handle the rest
    //sys_poll_tmr_start(&debounce_tmr, BTN_DEBOUNCE_TIME_MS, false);
    sys_cb_tmr_start(&_debounce_cb, BTN_DEBOUNCE_TIME_MS);
}

//This function is called from the timer ISR.
void _debounce_cb(void)
{
    //Whatever the state of the pin, it has been stable for the debounce period
    uint8_t _pin_state = sys_input_read(input_Button);
    if (_pin_state == _last_pin_state)
        return; //No change in state, so just return the current state

    //Save the current state 
    _last_pin_state = _pin_state; //Save the last state

    //We have either a button press or release event
    if (_pin_state == LOW)
    {
        //Button pressed
        button_event |= btn_down;

        //We need to start the measurement for the long press time now.
        _long_press_flag = true; //Set the flag to indicate we are waiting for a long-press
        sys_cb_tmr_start(&_long_press_cb, BTN_LONG_PRESS_TIME_MS);
    }
    else
    {
        //Button released
        button_event |= btn_release;

        //Stop the long press timer, as we are not interested in it anymore
        if (_long_press_flag)
        {
            _long_press_flag = false; //Reset the flag for the next time
            sys_cb_tmr_stop(&_long_press_cb);
            button_event |= btn_shrt_press;
        }
        //else, the long press event should already have been triggered, so we don't need to do anything
        
        //Are we still in time for a double-press event?
        if (_dbl_press_flag)
        {
            //We have a double-press event, so we need to stop the timer
            button_event |= btn_dbl_press;
            sys_cb_tmr_stop(&_dbl_press_cb);
            _dbl_press_flag = false; //Reset the flag for the next time
        }
        else // Nope.... this is a normal single press event
        {    
            //But we we can start the measurement for a double-press now
            _dbl_press_flag = true; //Set the flag to indicate we are waiting for a double-press
            sys_cb_tmr_start(&_dbl_press_cb, BTN_DBL_PRESS_TIME_MS);
        }
    }
}

// If/when this function is called, we have a long-press event
void _long_press_cb(void)
{
    //for safety, just check if the button is still pressed
    if ((_last_pin_state == LOW) && (_long_press_flag))
        button_event |= btn_long_press;

    _long_press_flag = false; //No long-press event, so reset the flag
}

void _dbl_press_cb(void)
{
    _dbl_press_flag = false; //Double-press timeout

}
/******************************************************************************
Local functions
******************************************************************************/

/******************************************************************************
Global functions
******************************************************************************/

void dev_button_init(
    void (*_cb_btn_down)(void), 
    void (*_cb_btn_release)(void), 
    void (*_cb_short_press)(void), 
    void (*_cb_long_press)(void), 
    void (*_cb_dbl_press)(void))
{
    sys_tmr_init(); //Initialise the timer system

    sys_set_io_mode(input_Button, INPUT_PULLUP);

    EICRA = (EICRA & ~((1 << ISC00) | (1 << ISC01))) | (CHANGE << ISC00);
    EIMSK |= (1 << INT0);

    if (_cb_btn_down != NULL)
        _btn_down_cb = _cb_btn_down;
    if (_cb_btn_release != NULL)
        _btn_release_cb = _cb_btn_release;  
    if (_cb_short_press != NULL)
        _btn_short_press_cb = _cb_short_press;
    if (_cb_long_press != NULL)
        _btn_long_press_cb = _cb_long_press;
    if (_cb_dbl_press != NULL)
        _btn_dbl_press_cb = _cb_dbl_press;

    button_event = 0;

    //sys_poll_tmr_start(&debounce_tmr, BTN_DEBOUNCE_TIME_MS, false);
    sys_cb_tmr_start(&_debounce_cb, BTN_DEBOUNCE_TIME_MS);

    iprintln(trBUTTON, "#Initialised (%d)", sys_input_read(input_Button));
}

void dev_button_service(void)
{

    if (button_event == 0)
        return; //No button event, so just return
    

    if (button_event & btn_down)
    {
        if (_btn_down_cb != NULL)
            _btn_down_cb();
        button_event &= ~btn_down; //Clear the button down event
    }

    if (button_event & btn_release)
    {
        if (_btn_release_cb != NULL)
            _btn_release_cb();
        button_event &= ~btn_release; //Clear the button release event
    }

    if (button_event & btn_shrt_press)
    {
        if (_btn_short_press_cb != NULL)
            _btn_short_press_cb();
        button_event &= ~btn_shrt_press; //Clear the short press event
    }

    if (button_event & btn_long_press)
    {
        if (_btn_long_press_cb != NULL)
            _btn_long_press_cb();
        button_event &= ~btn_long_press; //Clear the long press event
    }
    if (button_event & btn_dbl_press)
    {
        if (_btn_dbl_press_cb != NULL)
            _btn_dbl_press_cb();
        button_event &= ~btn_dbl_press; //Clear the double press event
    }
}

#undef PRINTF_TAG

/*************************** END OF FILE *************************************/
