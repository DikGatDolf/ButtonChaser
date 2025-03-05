/*******************************************************************************

Module:     sys_utils.c
Purpose:    This file contains the system utility functions
Author:     Rudolph van Niekerk

 *******************************************************************************/


/*******************************************************************************
includes
 *******************************************************************************/
#include <stdarg.h>
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>
#include "sys_utils.h"
#include "esp_err.h"

#define __NOT_EXTERN__
#include "sys_task_utils.h"
#undef __NOT_EXTERN__

/*******************************************************************************
local defines
 *******************************************************************************/
typedef enum
{
    eTaskIndexConsole = 0,
    eTaskIndexRgb,

    /* Keep this at the end */
    eTaskIndexMax
}eTaskIndex;


/*******************************************************************************
local variables
 *******************************************************************************/
TaskInfo_t * sys_tasks[eTaskIndexMax];
int sys_task_count = 0;

/*******************************************************************************
Global functions
 *******************************************************************************/
void sys_task_clear(void) 
{
    //Get everything ready
    for (int i = 0; i < eTaskIndexMax; i++)
    	sys_tasks[i] = NULL;
    sys_task_count = 0;
}

void sys_task_add(TaskInfo_t * _task) 
{
    if (_task->handle == NULL) {
        return;
    }
    
    sys_tasks[sys_task_count++] = _task;
}

// bool sys_task_running(TaskInfo_t * _task)
// {
//     if (_task->handle == NULL) {
//         return false;
//     }
//     return (eTaskGetState(_task->handle) == eRunning);
// }

#undef EXT
/*************************** END OF FILE *************************************/
