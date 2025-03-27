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

/******************************************************************************
Macros
******************************************************************************/
#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("Button") /* This must be undefined at the end of the file*/

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
/******************************************************************************
Local functions
******************************************************************************/

ISR(INT0_vect) // INT0
{
    if (int0_irq_cb)
        int0_irq_cb();
}

/******************************************************************************
Local functions
******************************************************************************/

/******************************************************************************
Global functions
******************************************************************************/

void dev_button_init(void (*cb_func_ptr)(void))
{
    pinMode(btnChasePin_BTN, INPUT_PULLUP);

    if (cb_func_ptr)
        int0_irq_cb = cb_func_ptr;
    // attachInterrupt(0, _dev_button_irq, FALLING);

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

    iprintln(trBUTTON, "#Initialised with CB @ 0x%08X (State: %d)", (uint32_t)cb_func_ptr, quickPinRead(btnChasePin_BTN));
}

#undef PRINTF_TAG

/*************************** END OF FILE *************************************/
