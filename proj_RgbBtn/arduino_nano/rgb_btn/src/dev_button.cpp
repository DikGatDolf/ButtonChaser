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

#define BTN_DEBOUNCE_TIME_MS    (50)	/* Debounce time in ms */

/******************************************************************************
Structs and Unions
******************************************************************************/

/******************************************************************************
Local function definitions
******************************************************************************/

/******************************************************************************
Local variables
******************************************************************************/

void (*int0_irq_cb)(void) = NULL;
timer_ms_t debounce_tmr;
//int pin_state;
uint8_t button_state;
stopwatch_ms_t button_press_sw;
unsigned long press_time;

#if (DEV_BTN_DEBUG == 1)
stopwatch_ms_t debug_sw;
#endif

/******************************************************************************
Local functions
******************************************************************************/

ISR(INT0_vect) // INT0
{
    sys_poll_tmr_start(&debounce_tmr, BTN_DEBOUNCE_TIME_MS, false);
}

/******************************************************************************
Local functions
******************************************************************************/

/******************************************************************************
Global functions
******************************************************************************/

void dev_button_init(void)
{
    quickPinMode(btnChasePin_BTN, INPUT_PULLUP);

#if defined(EICRA) && defined(ISC00) && defined(EIMSK)
    EICRA = (EICRA & ~((1 << ISC00) | (1 << ISC01))) | (CHANGE << ISC00);
    EIMSK |= (1 << INT0);
#elif defined(MCUCR) && defined(ISC00) && defined(GICR)
    MCUCR = (MCUCR & ~((1 << ISC00) | (1 << ISC01))) | (CHANGE << ISC00);
    GICR |= (1 << INT0);
#elif defined(MCUCR) && defined(ISC00) && defined(GIMSK)
    MCUCR = (MCUCR & ~((1 << ISC00) | (1 << ISC01))) | (CHANGE << ISC00);
    GIMSK |= (1 << INT0);
#else
    #error attachInterrupt not finished for this CPU (case 0)
#endif
    button_state = btn_up;
    press_time = 0lu;
    sys_stopwatch_ms_start(&button_press_sw);

    sys_poll_tmr_start(&debounce_tmr, BTN_DEBOUNCE_TIME_MS, false);

    iprintln(trBUTTON, "#Initialised (%d)", quickPinRead(btnChasePin_BTN));
}

void dev_button_measure_start(void)
{
    press_time = 0lu;
    sys_stopwatch_ms_start(&button_press_sw);
}

unsigned long dev_button_measure_lap(void)
{
    return sys_stopwatch_ms_lap(&button_press_sw);
}

uint8_t dev_button_get_state(void)
{
    //If the release or pressed flags are set when entering, clear them... those are one-shot flags
    if ((button_state & (btn_released | btn_pressed)) != 0)
        button_state &= ~(btn_released | btn_pressed);

    uint8_t _tmp_btn_state = 0; //No state assumed
    int _pin_state_0 = quickPinRead(btnChasePin_BTN);

    if (!sys_poll_tmr_expired(&debounce_tmr))
        return button_state;

    //To check if the button has not been pressed while we check the timer
    int _pin_state_1 = quickPinRead(btnChasePin_BTN);
    if (_pin_state_1 != _pin_state_0)
        return button_state;

    //State has been stable for the debounce time
    _tmp_btn_state = (_pin_state_0 == LOW)? btn_down : btn_up;
    if (button_state == _tmp_btn_state)
        return button_state;

    // Means the button state has changed
    button_state = _tmp_btn_state;
#if (DEV_BTN_DEBUG == 1)
    press_time = sys_stopwatch_ms_stop(&debug_sw);
    sys_stopwatch_ms_start(&debug_sw);
#endif
    if (_tmp_btn_state == btn_down)
    {
        button_state |= btn_pressed;

        if (button_press_sw.running)
            press_time = sys_stopwatch_ms_stop(&button_press_sw);
    }
    else
    {
        button_state |= btn_released;
    }
#if (DEV_BTN_DEBUG == 1)
    iprintln(trBUTTON, "#%s (%lu ms)", (button_state & btn_pressed)? "Pressed" : "Released", press_time);
#endif
    return button_state;
}

#undef PRINTF_TAG

/*************************** END OF FILE *************************************/
