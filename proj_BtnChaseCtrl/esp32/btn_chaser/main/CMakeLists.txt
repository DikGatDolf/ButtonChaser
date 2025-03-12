idf_component_register(SRCS "button_chase_main.c" 
                            "sys_task_utils.c" 
                            "str_helper.c" 
                            "template.c" 
                            "colour.c"
                            "sys_utils.c"
                            "sys_timers.c"
							"task_console.c"
							"task_rgb_led.c"
                            "drv_rgb_btn.c" 
                            "drv_rgb_led_strip.c" 
                    PRIV_REQUIRES spi_flash esp_timer esp_driver_rmt esp_driver_i2c
                    INCLUDE_DIRS "")
