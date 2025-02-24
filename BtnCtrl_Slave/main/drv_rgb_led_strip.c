/*******************************************************************************

Module:     drv_rgb_led_strip.c
Purpose:    This file contains the RGB LED driver task
Author:     Rudolph van Niekerk


 *******************************************************************************/

/*******************************************************************************
includes
 *******************************************************************************/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "led_strip_encoder.h"
 
#include "sys_utils.h"
#include "task_console.h"
//#include "esp_check.h"
#include "esp_err.h"

#define __NOT_EXTERN__
#include "drv_rgb_led_strip.h"
#undef __NOT_EXTERN__

/*******************************************************************************
local defines and constants
 *******************************************************************************/
#define PRINTF_TAG ("RGB") /* This must be undefined at the end of the file*/


#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)

#define RMT_LED_STRIP_GPIO_NUM      GPIO_NUM_8

#define EXAMPLE_LED_NUMBERS         1

#define RGB_LED_STRIP_CNT_MAX 5

/**
 * Macro which can be used to check the condition. If the condition is not 'true', it prints the message,
 * sets the local variable 'ret' to the supplied 'err_code', and then exits by jumping to 'goto_tag'.
 */
#define GOTO_ON_FALSE(a, err_code, goto_tag, format, ...) do {                              \
    if (unlikely(!(a))) {                                                                   \
        dbgPrint(trRGB, "#%s(%d): " format, __FUNCTION__, __LINE__, ##__VA_ARGS__);         \
        ret = err_code;                                                                     \
        goto goto_tag;                                                                      \
    }                                                                                       \
} while (0)

/**
 * Macro which can be used to check the error code. If the code is not ESP_OK, it prints the message,
 * sets the local variable 'ret' to the code, and then exits by jumping to 'goto_tag'.
 */
#define GOTO_ON_ERROR(x, goto_tag, format, ...) do {                                        \
    esp_err_t err_rc_ = (x);                                                                \
    if (unlikely(err_rc_ != ESP_OK)) {                                                      \
        dbgPrint(trRGB, "#%s(%d): " format, __FUNCTION__, __LINE__, ##__VA_ARGS__);         \
        ret = err_rc_;                                                                      \
        goto goto_tag;                                                                      \
    }                                                                                       \
} while(0)

/*******************************************************************************
local structure
 *******************************************************************************/
typedef struct {
//    int bytesPerPixel;
    rmt_bytes_encoder_config_t enc_cfg;     //RVN - bytes_encoder_config
    char pixel_order[5]; /* 4 bytes (max) + null terminator. Also determines the number of bytes per pixel.
                        e.g.    "rgb" - 3 bytes per pixel, ordered Red, Green, Blue
                                "grbw" - 4 bytes per pixel, ordered Green, Red, Blue, White
                                */        
 } rgb_drv_config_t;
  
typedef enum {
    SK6812_V1,
    SM16703_V1,
    SK6812W_V1,
}rgb_drv_types;
  
const rgb_drv_config_t rgb_led_drv_cfg[] = {  // Still must match order of `rgb_drv_types`
    [SK6812_V1]  = {.enc_cfg = {.bit0 = { .level0 = 1, .duration0 = 300, /*T0H*/ .level1 = 0, .duration1 = 900, /*T0L*/ },
                                .bit1 = { .level0 = 1, .duration0 = 700, /*T1H*/ .level1 = 0, .duration1 = 300, /*T1L*/ },
                    .flags.msb_first = 1 /*SK6812 transfer bit order: G7...G0R7...R0B7...B0 */}, .pixel_order = "grb" },

    [SM16703_V1] = {.enc_cfg = {.bit0 = { .level0 = 1, .duration0 = 300, /*T0H*/ .level1 = 0, .duration1 = 900, /*T0L*/ },
                                .bit1 = { .level0 = 1, .duration0 = 900, /*T1H*/ .level1 = 0, .duration1 = 300, /*T1L*/ },
                    .flags.msb_first = 1 /*SM16703 transfer bit order: R7...R0G7...G0B7...B0 */}, .pixel_order = "rgb" },

    [SK6812W_V1] = {.enc_cfg = {.bit0 = { .level0 = 1, .duration0 = 300, /*T0H*/ .level1 = 0, .duration1 = 900, /*T0L*/ },
                                .bit1 = { .level0 = 1, .duration0 = 900, /*T1H*/ .level1 = 0, .duration1 = 300, /*T1L*/ },
                    .flags.msb_first = 1 /*SK6812W transfer bit order: R7...R0G7...G0B7...B0W7...W0 */}, .pixel_order = "rgbw" },
};

typedef struct {
    rgb_drv_types drv_type;
    uint8_t pixel_cnt;
    uint8_t * pixel_buf; // Pointer to the pixel buffer, should be 3 or 4 bytes per pixel x pixel_cnt
    rmt_channel_handle_t chan;                  //RVN - led_chan
    rmt_tx_channel_config_t chan_config;
    rmt_encoder_handle_t rmt_encoder;           //RVN - led_encoder_handle
    // led_strip_encoder_t * led_strip_encoder;
    gpio_num_t rmt_tx_gpio_num;        /*!< GPIO number used by RMT TX channel. Set to -1 if unused */
    // led_strip_encoder_config_t encoder_config;
} led_strip_t;

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} led_strip_encoder_t;

// typedef struct {
//     uint32_t resolution; /*!< Encoder resolution, in Hz */
// } led_strip_encoder_config_t;

/*******************************************************************************
local function prototypes
 *******************************************************************************/

 
/*! Create RMT encoder for encoding LED strip pixels into RMT symbols
 * @param[in] config Encoder configuration
 * @param[out] ret_encoder Returned encoder handle
 * @return
 *      - ESP_ERR_INVALID_ARG for any invalid arguments
 *      - ESP_ERR_NO_MEM out of memory when creating led strip encoder
 *      - ESP_OK if creating encoder successfully
 */
esp_err_t rmt_new_led_strip_encoder(/*const led_strip_encoder_config_t *config,*/ rmt_encoder_handle_t *ret_encoder);

/*******************************************************************************
local variables
 *******************************************************************************/
// ConsoleMenuItem_t _task_rgb_btn_menu_items[] =
// {
// 										    // 01234567890123456789012345678901234567890123456789012345678901234567890123456789
// 		// {"version", _menu_handler_version,   "Displays application version"},
// 		// {"help",  	_menu_handler_help,      "Displays information on the available Console commands"},
// 		// {"trace",   _menu_handler_trace,     "Displays the status, or Enables/Disables print traces"},
// 		// {"rtos", 	_menu_handler_tasks,     "Displays RTOS information for each or a specific task"},
// };

static uint8_t led_strip_pixels[EXAMPLE_LED_NUMBERS * 3];

//led_strip_t

// different led strip might have its own timing requirements, following parameter is for WS2812
rmt_bytes_encoder_config_t bytes_encoder_config = {
    .bit0 = {
        .level0 = 1,
        .duration0 = 0.3 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000, // T0H=0.3us
        .level1 = 0,
        .duration1 = 0.9 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000, // T0L=0.9us
    },
    .bit1 = {
        .level0 = 1,
        .duration0 = 0.7 /*0.9*/ * RMT_LED_STRIP_RESOLUTION_HZ / 1000000, // T1H=0.9us
        .level1 = 0,
        .duration1 = 0.3 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000, // T1L=0.3us
    },
    .flags.msb_first = 1 // WS2812 transfer bit order: G7...G0R7...R0B7...B0
};

rmt_channel_handle_t led_chan = NULL;
rmt_tx_channel_config_t tx_chan_config = {
    .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
    .gpio_num = RMT_LED_STRIP_GPIO_NUM,
    .mem_block_symbols = 64, // increase the block size can make the LED less flickering
    .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
    .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
};

rmt_encoder_handle_t led_encoder_handle = NULL;
// led_strip_encoder_config_t encoder_config = {
//     .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
// };


static uint8_t led_strip_pixels[EXAMPLE_LED_NUMBERS * 3];
led_strip_t dbg_led = {
    .drv_type = SK6812_V1,
    .pixel_cnt = 1,
    .pixel_buf = NULL,
    .chan = NULL,
    //.chan_config = {.}, //RVN tx_chan_config, //
    .rmt_encoder = NULL, //RVN led_encoder_handle,
    .rmt_tx_gpio_num = RMT_LED_STRIP_GPIO_NUM,
    // .encoder_config = encoder_config,
};

/*******************************************************************************
Local (private) Functions
*******************************************************************************/

static size_t rmt_encode_led_strip(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    led_strip_encoder_t *led_strip_encoder = __containerof(encoder, led_strip_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = led_strip_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = led_strip_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    switch (led_strip_encoder->state) {
    case 0: // send RGB data
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_strip_encoder->state = 1; // switch to next state when current encoding session finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space for encoding artifacts
        }
    // fall-through
    case 1: // send reset code
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_strip_encoder->reset_code,
                                                sizeof(led_strip_encoder->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_strip_encoder->state = RMT_ENCODING_RESET; // back to the initial encoding session
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space for encoding artifacts
        }
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder)
{
    led_strip_encoder_t *led_strip_encoder = __containerof(encoder, led_strip_encoder_t, base);
    rmt_del_encoder(led_strip_encoder->bytes_encoder);
    rmt_del_encoder(led_strip_encoder->copy_encoder);
    free(led_strip_encoder);
    return ESP_OK;
}

static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t *encoder)
{
    led_strip_encoder_t *led_strip_encoder = __containerof(encoder, led_strip_encoder_t, base);
    rmt_encoder_reset(led_strip_encoder->bytes_encoder);
    rmt_encoder_reset(led_strip_encoder->copy_encoder);
    led_strip_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

esp_err_t rmt_new_led_strip_encoder(/*const led_strip_encoder_config_t *config,*/ rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    led_strip_encoder_t *led_strip_encoder = NULL;
    GOTO_ON_FALSE(/*config &&*/ ret_encoder, ESP_ERR_INVALID_ARG, err, "invalid argument");
    led_strip_encoder = rmt_alloc_encoder_mem(sizeof(led_strip_encoder_t));
    GOTO_ON_FALSE(led_strip_encoder, ESP_ERR_NO_MEM, err, "no mem for led strip encoder");
    led_strip_encoder->base.encode = rmt_encode_led_strip;
    led_strip_encoder->base.del = rmt_del_led_strip_encoder;
    led_strip_encoder->base.reset = rmt_led_strip_encoder_reset;
    GOTO_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &led_strip_encoder->bytes_encoder), err, "create bytes encoder failed");
    rmt_copy_encoder_config_t copy_encoder_config = {};
    GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &led_strip_encoder->copy_encoder), err, "create copy encoder failed");

    uint32_t reset_ticks = RMT_LED_STRIP_RESOLUTION_HZ /*config->resolution*/ / 1000000 * 50 / 2; // reset code duration defaults to 50us
    led_strip_encoder->reset_code = (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = reset_ticks,
        .level1 = 0,
        .duration1 = reset_ticks,
    };
    *ret_encoder = &led_strip_encoder->base;
    return ESP_OK;
err:
    if (led_strip_encoder) {
        if (led_strip_encoder->bytes_encoder) {
            rmt_del_encoder(led_strip_encoder->bytes_encoder);
        }
        if (led_strip_encoder->copy_encoder) {
            rmt_del_encoder(led_strip_encoder->copy_encoder);
        }
        free(led_strip_encoder);
    }
    return ret;
}

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

/*******************************************************************************
Global (public) Functions
*******************************************************************************/

void drv_rgb_led_strip_init(void)
{
    dbgPrint(trRGB, "#Create RMT TX channel");
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    dbgPrint(trRGB, "#Install led strip encoder");
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(/*&encoder_config,*/ &led_encoder_handle));

    dbgPrint(trRGB, "#Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));
}

void drv_rgb_led_strip_deinit(void)
{
    dbgPrint(trRGB, "#Disable RMT TX channel");
    ESP_ERROR_CHECK(rmt_disable(led_chan));

    dbgPrint(trRGB, "#Uninstall led strip encoder");
    ESP_ERROR_CHECK(rmt_del_encoder(led_encoder_handle));

    dbgPrint(trRGB, "#Destroy RMT TX channel");
    ESP_ERROR_CHECK(rmt_del_channel(led_chan));
}

static rmt_transmit_config_t rmt_tx_config = {
    .loop_count = 0, // no transfer loop
};

void drv_rgb_led_strip_set_rgb_pixel(uint32_t index, uint32_t red, uint32_t green, uint32_t blue)
{
    index = index % EXAMPLE_LED_NUMBERS;

    // Build RGB pixels
    led_strip_pixels[index * 3 + 0] = green;
    led_strip_pixels[index * 3 + 1] = blue;
    led_strip_pixels[index * 3 + 2] = red;

    // Flush RGB values to LEDs
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder_handle, led_strip_pixels, sizeof(led_strip_pixels), &rmt_tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));

}
void drv_rgb_led_strip_clear_rgb_pixel(uint32_t index)
{
    index = index % EXAMPLE_LED_NUMBERS;
    memset(&led_strip_pixels[index], 0, (sizeof(led_strip_pixels)/EXAMPLE_LED_NUMBERS));
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder_handle, led_strip_pixels, sizeof(led_strip_pixels), &rmt_tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
}

#undef PRINTF_TAG

/*************************** END OF FILE *************************************/