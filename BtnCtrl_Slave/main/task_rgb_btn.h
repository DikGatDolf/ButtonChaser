/*****************************************************************************

task_rgb_btn.h

Include file for task_rgb_btn.c

******************************************************************************/
#ifndef __task_rgb_led_H__
#define __task_rgb_led_H__


/******************************************************************************
includes
******************************************************************************/
#include "defines.h"

/******************************************************************************
Macros
******************************************************************************/
#ifdef __NOT_EXTERN__
#define EXT
#else
#define EXT extern
#endif /* __NOT_EXTERN__ */

/******************************************************************************
variables
******************************************************************************/

/******************************************************************************
Global (public) Function prototypes
******************************************************************************/

/*! Initialises the RGB driving task
 * @return A pointer to the taskInfo structure for the console (if successful)
            otherwise NULL
 */
TaskInfo_t * task_rgb_btn_init(void);

/*! Deinitialises the Console
 */
void task_rgb_btn_deinit(void);

#undef EXT
#endif /* __task_rgb_led_H__ */

/****************************** END OF FILE **********************************/
