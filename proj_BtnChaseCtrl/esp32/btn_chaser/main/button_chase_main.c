/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <ctype.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>
#include "sys_utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

#include "defines.h"
#include "sys_utils.h"
#include "sys_timers.h"
#include "sys_task_utils.h"
#ifdef CONSOLE_ENABLED
  #include "task_console.h"
#endif
#include "task_rgb_led.h"
#include "drv_rgb_btn.h"

#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("Main") /* This must be undefined at the end of the file*/

/*! CONSOLE MENU HANDLER - Performs a system reset
 */

 void _sys_handler_reset(void);
/*! CONSOLE MENU ITEM - Prints the help string for either the command in question, 
 * or for the entire list of commands under the group
 */
void _sys_handler_tasks(void);


ConsoleMenuItem_t _task_main_menu_items[] =
{
                                    //01234567890123456789012345678901234567890123456789012345678901234567890123456789
	{"reset",   _sys_handler_reset, "Perform a system reset"},
    {"rtos", 	_sys_handler_tasks,     "Displays RTOS information for each or a specific task"},
};

void app_main(void)
{
    TickType_t xLastWakeTime;

    //printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    //printf("We are launching %d tasks\n", eTaskIndexMax);

    sys_timers_init();
    sys_task_clear();

#ifdef CONSOLE_ENABLED
    sys_task_add((TaskInfo_t *)console_init_task());
    console_add_menu("sys", _task_main_menu_items, ARRAY_SIZE(_task_main_menu_items), "System");
#endif
    sys_task_add((TaskInfo_t *)rgb_led_init_task());

    drv_rgb_btn_init();

    xLastWakeTime = xTaskGetTickCount();

    //This task will basically just monitor the stack usage (to what end???)
    while (1){
		/* Inspect our own high water mark on entering the task. */
    	//task_main_info.stack_unused = uxTaskGetStackHighWaterMark2( NULL );


		//iprintln(trALWAYS, "#Task Running");

    	//We're not all that busy from this point onwards.... might as well check in every 1000ms.
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }

    sys_timers_deinit();


    // for (int i = 10; i >= 0; i--) {
    //     printf("Restarting in %d seconds...\n", i);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
    
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}

#ifdef CONSOLE_ENABLED
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
            iprintln(trALWAYS, "Now type 'reset Y', IF YOU ARE SURE.\n");
            return;
        }
        iprintln(trALWAYS, "No arguments expected (got \"%s\").\n", argStr);
    }
    else //if (reset_lock)
    {
		if (0 == strcasecmp(argStr, "Y"))
        {
            // ok, do the reset.
            iprintln(trALWAYS, "Resetting. Goodbye, cruel world!\n");
            fflush(stdout);
            esp_restart();
            return;
        }        
        iprintln(trALWAYS, "'reset Y' expected. Starting over.\n");
    }
    reset_lock = false;
}

void _sys_handler_tasks(void)
{
    /*We are expecting one fo the following arguments
    <nothing> 				- Display the stack usage of all the running tasks
    <"?"> 					- Displaya list of all the running tasks
    <task name> 			- Display the stack usage of the specified task
    */
#if ( configUSE_TRACE_FACILITY == 1 )
    int task_index = -1;
    int print_start = 0;
    int print_end = eTaskIndexMax;

    char task_list_buff[eTaskIndexMax*40];

    if (arg_cnt > 0)
    {
        if (is_number_str(args[0]))
        {
            int index = atoi(args[0]);
            if ((index >= eTaskIndexMax) || (index < 0))
            {
                iprintln(trALWAYS, "Please specify a number from 0 and %d, or alternatively the task name.");
                task_index = -1;
            }
        }
    }

    if (task_index >= 0)
    {
        //We are only printing one task's information
        print_start = task_index;
        print_end = task_index+1;
    }
    // else, keep the values unchanged and print the entire list

    iprintln(trALWAYS, "Task Information");
    iprintln(trALWAYS, "+---+------------+------------+-------+------------+--------------------+");
    iprintln(trALWAYS, "+ # | Name       | Status     | Pr'ty | Runtime    | Stack Usage        |");
    iprintln(trALWAYS, "+---+------------+------------+-------+------------+--------------------+");
    for (int i = print_start; i < print_end; i++)
    {
        float s_depth_f;
        configSTACK_DEPTH_TYPE s_depth_u16;
        configSTACK_DEPTH_TYPE s_used_u16;

        if (!sys_tasks[i])
            continue;
        vTaskGetInfo(sys_tasks[i]->handle, &sys_tasks[i]->details, pdTRUE, eInvalid);

        s_depth_u16 = (configSTACK_DEPTH_TYPE)sys_tasks[i]->stack_depth;
        s_used_u16 = (configSTACK_DEPTH_TYPE)(sys_tasks[i]->stack_depth - sys_tasks[i]->stack_unused);
        s_depth_f = (float)(s_used_u16 * 100.0f)/(s_depth_u16 * 1.0f);

        iprintln(trALWAYS, "| %d | %-10s | %-10s | %02d/%02d | %10u | %04u/%04u (%2.2f%%) |",
            i, 
            sys_tasks[i]->details.pcTaskName,
            taskStateName[sys_tasks[i]->details.eCurrentState],
            sys_tasks[i]->details.uxCurrentPriority,
            sys_tasks[i]->details.uxBasePriority,
            sys_tasks[i]->details.ulRunTimeCounter,
            s_used_u16,
            s_depth_u16,
            s_depth_f);
    }
    iprintln(trALWAYS, "+---+------------+------------+-------+------------+--------------------+");
    iprintln(trALWAYS, "");

    iprintln(trALWAYS, "System generated Task List:");
    vTaskList(task_list_buff);
    iprintln(trALWAYS, "%s", task_list_buff);
    iprintln(trALWAYS, "Done.");
#else
    iprintln(trALWAYS, "Not Supported in this build.");
#endif
}

#endif

#undef PRINTF_TAG
/****************************** END OF FILE **********************************/