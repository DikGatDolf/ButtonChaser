/*****************************************************************************

dev_button.h

Include file for dev_button.c

******************************************************************************/
#ifndef __dev_button_H__
#define __dev_button_H__

/******************************************************************************
includes
******************************************************************************/
#include "Arduino.h"
#include "sys_utils.h"

/******************************************************************************
definitions
******************************************************************************/
enum button_state_e
{
    btn_up          = BIT_POS(0),
    btn_down        = BIT_POS(1),
    btn_pressed     = BIT_POS(2),   //Implies btn_down
    btn_released    = BIT_POS(3),   //Implies btn_up
    // btnLongPress    = BIT_POS(4),
    // btnShortPress   = BIT_POS(5),
    //btnDoublePress  = BIT_POS(6),
    btn_active      = BIT_POS(7),
};
/******************************************************************************
Macros
******************************************************************************/

/******************************************************************************
Struct & Unions
******************************************************************************/

/******************************************************************************
variables
******************************************************************************/

/******************************************************************************
functions
******************************************************************************/
void dev_button_init(void);

void dev_button_measure_start(void);

unsigned long dev_button_get_reaction_time_ms(void);

uint8_t dev_button_get_state(void);

#endif /* __dev_button_H__ */

/****************************** END OF FILE **********************************/
