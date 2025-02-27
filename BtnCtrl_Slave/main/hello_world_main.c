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
#ifdef CONSOLE_ENABLED
  #include "task_console.h"
#endif
#include "task_rgb_btn.h"

#define PRINTF_TAG ("Main") /* This must be undefined at the end of the file*/

/*! CONSOLE MENU HANDLER - Performs a system reset
 */
void _menu_handler_reset(void);


ConsoleMenuItem_t _task_main_menu_items[] =
{
                                    //01234567890123456789012345678901234567890123456789012345678901234567890123456789
	{"reset",   _menu_handler_reset, "Perform a system reset"},
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

    printf("We are launching %d tasks\n", eTaskIndexMax);

    sys_timers_init();

    //Get everything ready
    for (int i = 0; i < eTaskIndexMax; i++)
    	sys_tasks[i] = NULL;

#ifdef CONSOLE_ENABLED
    sys_tasks[eTaskIndexConsole]    = task_console_init();
#endif
    sys_tasks[eTaskIndexRgb]        = task_rgb_btn_init();
    
        //typeof(_task_main_menu_items)
#ifdef CONSOLE_ENABLED
    task_console_add_menu("main", _task_main_menu_items, ARRAY_SIZE(_task_main_menu_items), "System");
#endif

    xLastWakeTime = xTaskGetTickCount();

    //This task will basically just monitor the stack usage (to what end???)
    while (1){
		/* Inspect our own high water mark on entering the task. */
    	//task_main_info.stack_unused = uxTaskGetStackHighWaterMark2( NULL );


		//dbgPrint(trALWAYS, "#Task Running");

    	//We're not all that busy from this point onwards.... might as well check in every 1000ms.
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }

    task_console_deinit();

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
void _menu_handler_reset(void)
{
    static bool reset_lock = false;
	// if no parameter passed then just open the gate

    char *argStr = task_console_arg_pop();

	if (!reset_lock)
    {
        if (!argStr)
        {
            reset_lock = true;
            dbgPrint(trALWAYS, "Now type 'reset Y', IF YOU ARE SURE.\n");
            return;
        }
        dbgPrint(trALWAYS, "No arguments expected (got \"%s\").\n", argStr);
    }
    else //if (reset_lock)
    {
		if (0 == strcasecmp(argStr, "Y"))
        {
            // ok, do the reset.
            dbgPrint(trALWAYS, "Resetting. Goodbye, cruel world!\n");
            fflush(stdout);
            esp_restart();
            return;
        }        
        dbgPrint(trALWAYS, "'reset Y' expected. Starting over.\n");
    }
    reset_lock = false;
}
#endif

#undef PRINTF_TAG
/****************************** END OF FILE **********************************/