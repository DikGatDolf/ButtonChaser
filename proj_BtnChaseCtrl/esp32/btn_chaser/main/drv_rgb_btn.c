/*******************************************************************************
Module:     drv_rgb_btn.c
Purpose:    This file contains the driver interface for a single RGB Button which 
             is currently implemented an Arduino Nano.
            Communication is done via an I2C serial interface. The idea is that 
             up to 16 RGB button devices can be daisy-chained on a single I2C 
             bus and be controlled via this interface. The upper 4 bits of the 
             I2C address is used to identify the RGB button type, and the lower 
             4 bits are used to address the individual devices.
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

#include "task_console.h"

//#include "sys_utils.h"
#include "driver/i2c_master.h"
//#include "../../../../common/common_comms.h"

#define __NOT_EXTERN__
#include "drv_rgb_btn.h"
#undef __NOT_EXTERN__

/*******************************************************************************
Macros and Constants
 *******************************************************************************/
#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("RgbBtn") /* This must be undefined at the end of the file*/

#define I2C_DEFAULT_TIMEOUT_MS  (1000)
/*******************************************************************************
local defines 
 *******************************************************************************/

/*******************************************************************************
 Local structure
 *******************************************************************************/
typedef struct rgb_button_st{
    union {
        struct {
            struct {
                uint8_t red;
                uint8_t green;
                uint8_t blue;
            }colour;
            uint8_t btn;
        };
        uint32_t btn_colour;    // 1 byte button, 3 bytes colour
    };
}rgb_button_t;

typedef struct{
    i2c_master_bus_config_t config;
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
}drv_rgb_btn_i2c_master_data_t;

typedef struct{
    bool init_done;
    drv_rgb_btn_i2c_master_data_t i2c_master;
    i2c_device_config_t i2c_slave_cfg;
    rgb_button_t rgb_btn;
}dev_rgb_button_t;

//i2c_master_bus_handle_t bus_handle;
//ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

//i2c_master_dev_handle_t dev_handle;
//ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
/*******************************************************************************
 Local function prototypes
 *******************************************************************************/

/*******************************************************************************
 Local variables
 *******************************************************************************/
dev_rgb_button_t _rgb_btn = {
    .init_done = false,
    .i2c_master.config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,              /*-1 for auto select */
        .scl_io_num = pinI2C_SCL,
        .sda_io_num = pinI2C_SDA,
        .glitch_ignore_cnt = 7,
        //.intr_priority = 1,
        .flags.enable_internal_pullup = true,
    },
    .i2c_slave_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = RGB_BTN_I2C_TYPE_ID, /*0x58,*/
        .scl_speed_hz = 100000,
    },
};

/*******************************************************************************
 Local (private) Functions
 *******************************************************************************/

/*******************************************************************************
 Global (public) Functions
 *******************************************************************************/

void drv_rgb_btn_init(void)
{

    ESP_ERROR_CHECK(i2c_new_master_bus(&_rgb_btn.i2c_master.config, &_rgb_btn.i2c_master.bus_handle));

    ESP_ERROR_CHECK(i2c_master_bus_add_device(_rgb_btn.i2c_master.bus_handle, &_rgb_btn.i2c_slave_cfg, &_rgb_btn.i2c_master.dev_handle));

    _rgb_btn.init_done = true;
    iprintln(trBTN, "#Driver initialised");
    // iprintln(trBTN, "#bus_handle: 0x%08x", _rgb_btn.i2c_master.bus_handle);
    // iprintln(trBTN, "#dev_handle: 0x%08x", _rgb_btn.i2c_master.dev_handle);
}

void drv_rgb_btn_deinit(void)
{
    if (_rgb_btn.init_done)
    {
        ESP_ERROR_CHECK(i2c_master_bus_rm_device(_rgb_btn.i2c_master.dev_handle));
        _rgb_btn.init_done = false;
        iprintln(trBTN, "#Driver de-initialised");
    }
}

esp_err_t drv_rgb_btn_tx(uint8_t *tx_data, size_t tx_size)
{
    if (!_rgb_btn.init_done)
    {
        iprintln(trBTN, "#Driver not initialised");
        return ESP_FAIL;
    }

//    return i2c_master_transmit(_rgb_btn.i2c_master.dev_handle, tx_data, tx_size, -1);
    return i2c_master_transmit_receive(_rgb_btn.i2c_master.dev_handle, tx_data, tx_size, NULL, 0, 1000);
}

esp_err_t drv_rgb_btn_txrx(uint8_t *tx_data, size_t tx_size, uint8_t *rx_data, size_t rx_size)
{
    if (!_rgb_btn.init_done)
    {
        iprintln(trBTN, "#Driver not initialised");
        return ESP_FAIL;
    }

    return i2c_master_transmit_receive(_rgb_btn.i2c_master.dev_handle, tx_data, tx_size, rx_data, rx_size, I2C_DEFAULT_TIMEOUT_MS);
}

esp_err_t drv_rgb_btn_probe(uint8_t id_mask)
{
    if (!_rgb_btn.init_done)
    {
        iprintln(trBTN, "#Driver not initialised");
        return ESP_FAIL;
    }

    return i2c_master_probe(_rgb_btn.i2c_master.bus_handle, 
                            ((RGB_BTN_I2C_TYPE_ID & RGB_BTN_I2C_TYPE_MASK) | (id_mask & RGB_BTN_I2C_ADDR_MASK)), I2C_DEFAULT_TIMEOUT_MS);
}
#undef PRINTF_TAG
#undef EXT
/*************************** END OF FILE *************************************/
