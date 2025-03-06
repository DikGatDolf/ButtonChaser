#idf_component_register(SRCS "hello_world_main.c"
idf_component_register(SRCS "button_chase_main.c" 
                            "sys_task_utils.c" 
                            "str_helper.c" 
                            "template.c" 
                            "colour.c"
                            "sys_utils.c"
                            "sys_timers.c"
							"task_console.c"
							"task_rgb_led.c"
                            "rgbledstrip.c" 
                    PRIV_REQUIRES spi_flash esp_timer esp_driver_rmt
                    INCLUDE_DIRS "")
