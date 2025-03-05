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
#include "esp_err.h"
 
#include "sys_utils.h"
#include "task_console.h"
#include "colour.h"

#define __NOT_EXTERN__
#include "drv_rgb_led_strip.h"
#undef __NOT_EXTERN__

/*******************************************************************************
local defines and constants
 *******************************************************************************/
#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("LedStrip") /* This must be undefined at the end of the file*/


#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)

#define RMT_ticks_per_us        (RMT_LED_STRIP_RESOLUTION_HZ / 1000000L)

#define RMT_ns2ticks(_ns)       ( (_ns) * RMT_ticks_per_us / 1000 )
#define RMT_ticks2ns(_ticks)    ( (_ticks) * 1000 / RMT_ticks_per_us )

#define ColourBuffer(_type)     ((uint8_t*)led_strip_list[_type]->colour_buf)
#define ColOrder(_type)         ((const char *)rgb_led_drv_cfg[led_strip_list[_type]->drv_type].col_order)

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
    rmt_bytes_encoder_config_t enc_cfg;     //RVN - bytes_encoder_config
    const char col_order[5]; /* 4 bytes (max) + null terminator. Also determines the number of bytes per led.
                        e.g.    "rgb" - 3 bytes per led, ordered Red, Green, Blue
                                "grbw" - 4 bytes per led, ordered Green, Red, Blue, White
                                */        
    const char name[11];
} rgb_drv_config_t;
  
typedef enum {
    SK6812_V1,
    SM16703_V1,
    SK6812W_V1,
    /* Keep this at the end */
    RGB_DRV_MAX
}rgb_drv_types;
  
const rgb_drv_config_t rgb_led_drv_cfg[] = {  // Still must match order of `rgb_drv_types`
    [SK6812_V1]  = {.enc_cfg = {.bit0 = {   .level0 = 1, .duration0 = RMT_ns2ticks(300), /*T0H*/ 
                                            .level1 = 0, .duration1 = RMT_ns2ticks(900), /*T0L*/ },
                                .bit1 = {   .level0 = 1, .duration0 = RMT_ns2ticks(700), /*T1H*/ 
                                            .level1 = 0, .duration1 = RMT_ns2ticks(300), /*T1L*/ },
                    .flags.msb_first = 1 /*SK6812 transfer bit order: G7...G0R7...R0B7...B0 */}, .col_order = "grb", .name = "SK6812"},

    [SM16703_V1] = {.enc_cfg = {.bit0 = {   .level0 = 1, .duration0 = RMT_ns2ticks(300), /*T0H*/ 
                                            .level1 = 0, .duration1 = RMT_ns2ticks(900), /*T0L*/ },
                                .bit1 = {   .level0 = 1, .duration0 = RMT_ns2ticks(900), /*T1H*/ 
                                            .level1 = 0, .duration1 = RMT_ns2ticks(300), /*T1L*/ },
                    .flags.msb_first = 1 /*SM16703 transfer bit order: R7...R0G7...G0B7...B0 */}, .col_order = "rgb", .name = "SM16703"},

    [SK6812W_V1] = {.enc_cfg = {.bit0 = {   .level0 = 1, .duration0 = RMT_ns2ticks(300), /*T0H*/ 
                                            .level1 = 0, .duration1 = RMT_ns2ticks(900), /*T0L*/ },
                                .bit1 = {   .level0 = 1, .duration0 = RMT_ns2ticks(900), /*T1H*/ 
                                            .level1 = 0, .duration1 = RMT_ns2ticks(300), /*T1L*/ },
                    .flags.msb_first = 1 /*SK6812W transfer bit order: R7...R0G7...G0B7...B0W7...W0 */}, .col_order = "rgbw", .name = "SK6812W"},
};

typedef struct {
    rgb_drv_types drv_type;                     // Type of LED strip driver (see rgb_led_drv_cfg[]), NEEDS TO BE SETUP BEFORE RMT initialisation
    uint8_t led_cnt;                            // Number of leds in the strip, NEEDS TO BE SETUP BEFORE RMT initialisation
    uint8_t * colour_buf;                       // Pointer to the colour buffer, should be bytes_per_led x led_cnt NB: Allocated at initialisation!
    rmt_channel_handle_t chan;                  // Pointer to the RMT Channel Config NB: Allocated at RMT initialisation!
    rmt_tx_channel_config_t chan_config;        // RMT Channel Config, NEEDS TO BE SETUP BEFORE RMT initialisation
    rmt_encoder_handle_t rmt_encoder;           // Pointer to the RMT Encoder Config NB: Allocated at RMT initialisation!
    esp_err_t init_result;                      // Result of the initialisation, set to ESP_OK if successful
    const char * name;                          // Name of the LED strip, assigned at initialisation
} led_strip_t;

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} led_strip_encoder_t;

/*******************************************************************************
local function prototypes
 *******************************************************************************/
/*! Initialize the LED strip driver
 * @param[in] led_strip LED strip handle
 * @param[in] name Name of the LED strip
 * @return number of leds in the strip if initialisation is successful, 0 otherwise
 */
uint32_t _rgb_led_strip_init(led_strip_t * led_strip);

/*! Deinitialize the LED strip driver
 * @param[in] led_strip LED strip handle
 * @param[in] name Name of the LED strip
 */
void _rgb_led_strip_deinit(led_strip_t * led_strip);

/*! Create RMT encoder for encoding LED strip leds into RMT symbols
 * @param[in] config Encoder configuration
 * @param[out] ret_encoder Returned encoder handle
 * @return
 *      - ESP_ERR_INVALID_ARG for any invalid arguments
 *      - ESP_ERR_NO_MEM out of memory when creating led strip encoder
 *      - ESP_OK if creating encoder successfully
 */
esp_err_t rmt_new_led_strip_encoder(rgb_drv_types type, rmt_encoder_handle_t *ret_encoder);

/*******************************************************************************
local variables
 *******************************************************************************/

static led_strip_t dbg_led = {
    .drv_type = SK6812_V1,                              /* Must be set here (see rgb_led_drv_cfg[]) */
    .led_cnt = 1,                                       /* Must be set here - Only 1 led on the Debug RGB LED */
    .colour_buf = NULL,                                 /* Set to NULL, will be alocated at initialisation */
    .chan = NULL,                                       /* Set to NULL, will be assigned at initialisation */
    .chan_config = {                                    /* RMT TX Chan Config */
        .clk_src = RMT_CLK_SRC_DEFAULT,                 /* select source clock */
        .gpio_num = GPIO_NUM_8,                         /*< GPIO number used by RMT TX channel. Set to -1 if unused */
        .mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL, /* DO NOT CHANGE THIS */
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4,                         /* set the number of transactions that can be pending in the background*/
    },
    .rmt_encoder = NULL,                                /* Set to NULL, will be assigned at initialisation */
    .init_result = ESP_FAIL,                            /* Set to ESP_FAIL, will be assigned at initialisation, hopefully to ESP_OK */
    .name = "Debug",
};

static led_strip_t button_led = {
    .drv_type = SM16703_V1,                             /* Must be set here (see rgb_led_drv_cfg[]) */
    .led_cnt = 5,                                       /* Must be set here - Let's start with 5 leds on the Button RGB LEDs */
    .colour_buf = NULL,                                 /* Set to NULL, will be alocated at initialisation */
    .chan = NULL,                                       /* Set to NULL, will be assigned at initialisation */
    .chan_config = {                                    /* RMT TX Chan Config */
        .clk_src = RMT_CLK_SRC_DEFAULT,                 /* select source clock */
        .gpio_num = GPIO_NUM_9,                         /*< GPIO number used by RMT TX channel. Set to -1 if unused */
        .mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL, /* DO NOT CHANGE THIS */
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4,                         /* set the number of transactions that can be pending in the background*/
    },
    .rmt_encoder = NULL,                                /* Set to NULL, will be assigned at initialisation */
    .init_result = ESP_FAIL,                            /* Set to ESP_FAIL, will be assigned at initialisation, hopefully to ESP_OK */
    .name = "Button",
};

led_strip_t * led_strip_list[strips_Total] = {
    [strip_Debug] = &dbg_led,
    [strip_Buttons] = &button_led,
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

esp_err_t rmt_new_led_strip_encoder(rgb_drv_types type, rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;

    if (type >= RGB_DRV_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    led_strip_encoder_t *led_strip_encoder = NULL;
    GOTO_ON_FALSE(/*config &&*/ ret_encoder, ESP_ERR_INVALID_ARG, err, "invalid argument");
    led_strip_encoder = rmt_alloc_encoder_mem(sizeof(led_strip_encoder_t));
    GOTO_ON_FALSE(led_strip_encoder, ESP_ERR_NO_MEM, err, "no mem for led strip encoder");
    led_strip_encoder->base.encode = rmt_encode_led_strip;
    led_strip_encoder->base.del = rmt_del_led_strip_encoder;
    led_strip_encoder->base.reset = rmt_led_strip_encoder_reset;
    GOTO_ON_ERROR(rmt_new_bytes_encoder(&rgb_led_drv_cfg[type].enc_cfg, &led_strip_encoder->bytes_encoder), err, "create bytes encoder failed");
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

uint32_t _rgb_led_strip_init(led_strip_t * led_strip)
{    
    //dbgPrint(trRGB, "#Initialise %s RGB LED RMT Channel", led_strip->name);
    ESP_ERROR_CHECK(rmt_new_tx_channel(&led_strip->chan_config, &led_strip->chan));
    
    //dbgPrint(trRGB, "#Install %s RGB LED Strip encoder", led_strip->name);
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(led_strip->drv_type, &led_strip->rmt_encoder));

    //dbgPrint(trRGB, "#Enable %s RGB LED RMT TX channel", led_strip->name);
    ESP_ERROR_CHECK(rmt_enable(led_strip->chan));

    //allocate memory for led_strip->colour_buf
    int bytes_per_led = strlen(rgb_led_drv_cfg[led_strip->drv_type].col_order);
    led_strip->colour_buf = (uint8_t *)malloc(led_strip->led_cnt * bytes_per_led);
    if (led_strip->colour_buf == NULL) {
        dbgPrint(trRGB|trALWAYS, "#No mem for %s RGB LED buffer", led_strip->name);
        led_strip->init_result = ESP_ERR_NO_MEM;
    }
    else
    {
        //dbgPrint(trRGB, "#Allocated %d bytes for %s RGB LED strip (%d leds x %d bytes)", bytes_per_led*led_strip->led_cnt, led_strip->name, led_strip->led_cnt, bytes_per_led);
        led_strip->init_result = ESP_OK;
    }
    ESP_ERROR_CHECK(led_strip->init_result);

    return (led_strip->init_result == ESP_OK)? led_strip->led_cnt : 0;
}

void _rgb_led_strip_deinit(led_strip_t * led_strip)
{    
    if (led_strip->colour_buf != NULL) {
        int bytes_per_led = strlen(rgb_led_drv_cfg[led_strip->drv_type].col_order);
        dbgPrint(trRGB, "#Freed %d bytes for %s RGB LED strip (%d leds x %d bytes)", bytes_per_led*led_strip->led_cnt, led_strip->name, led_strip->led_cnt, bytes_per_led);
        free(led_strip->colour_buf);
        led_strip->colour_buf = NULL;
    }

    dbgPrint(trRGB, "#Disable %s RGB LED RMT TX channel", led_strip->name);
    ESP_ERROR_CHECK(rmt_disable(led_strip->chan));

    dbgPrint(trRGB, "#Uninstall %s RGB LED Strip encoder", led_strip->name);
    ESP_ERROR_CHECK(rmt_del_encoder(led_strip->rmt_encoder));

    dbgPrint(trRGB, "#Destroy %s RGB LED RMT Channel", led_strip->name);
    ESP_ERROR_CHECK(rmt_del_channel(led_strip->chan));

    led_strip->init_result = ESP_FAIL;
}

void _print_strip_config(rgb_led_strip_type strip)
{
    led_strip_t * led_strip = led_strip_list[strip];

    dbgPrint(trRGB, "#%s RGB LED Strip Config:", led_strip->name);
    dbgPrint(trRGB, "#  Driver Type: %s", rgb_led_drv_cfg[led_strip->drv_type].name);
    dbgPrint(trRGB, "#    Colour Order: %s (%d bytes per led)", ColOrder(strip), strlen(ColOrder(strip)));
    dbgPrint(trRGB, "#    Bit Timing:");
    dbgPrint(trRGB, "#      bit0: level %d, %d ticks (%dns)", rgb_led_drv_cfg[led_strip->drv_type].enc_cfg.bit0.level0, 
                                                                rgb_led_drv_cfg[led_strip->drv_type].enc_cfg.bit0.duration0, 
                                                                RMT_ticks2ns(rgb_led_drv_cfg[led_strip->drv_type].enc_cfg.bit0.duration0));
    dbgPrint(trRGB, "#            level %d, %d ticks (%dns)", rgb_led_drv_cfg[led_strip->drv_type].enc_cfg.bit0.level1, 
                                                                rgb_led_drv_cfg[led_strip->drv_type].enc_cfg.bit0.duration1, 
                                                                RMT_ticks2ns(rgb_led_drv_cfg[led_strip->drv_type].enc_cfg.bit0.duration1));
    dbgPrint(trRGB, "#      bit1: level %d, %d ticks (%dns)", rgb_led_drv_cfg[led_strip->drv_type].enc_cfg.bit1.level0, 
                                                                rgb_led_drv_cfg[led_strip->drv_type].enc_cfg.bit1.duration0, 
                                                                RMT_ticks2ns(rgb_led_drv_cfg[led_strip->drv_type].enc_cfg.bit1.duration0));
    dbgPrint(trRGB, "#            level %d, %d ticks (%dns)", rgb_led_drv_cfg[led_strip->drv_type].enc_cfg.bit1.level1, 
                                                                rgb_led_drv_cfg[led_strip->drv_type].enc_cfg.bit1.duration1, 
                                                                RMT_ticks2ns(rgb_led_drv_cfg[led_strip->drv_type].enc_cfg.bit1.duration1));
    dbgPrint(trRGB, "#  LED Count: %d/%d", (led_strip->init_result == ESP_OK)?led_strip->led_cnt : 0, led_strip->led_cnt);
    dbgPrint(trRGB, "#  Colour Buffer: %p (%d bytes)", ColourBuffer(strip), strlen(ColOrder(strip)) * led_strip->led_cnt);
    dbgPrint(trRGB, "#  RMT Channel: %p", led_strip->chan);
    dbgPrint(trRGB, "#  RMT Encoder: %p", led_strip->rmt_encoder);
    dbgPrint(trRGB, "#  Init Result: %s", esp_err_to_name(led_strip->init_result));
    dbgPrint(trRGB, "");
}

/*******************************************************************************
Global (public) Functions
*******************************************************************************/

uint32_t drv_rgb_led_strip_init(void)
{
    uint32_t led_cnt = 0;

    for (rgb_led_strip_type i = 0; i < strips_Total; i++)
    {
        led_cnt += _rgb_led_strip_init(led_strip_list[i]);
        //_print_strip_config(i);
    }
    // led_cnt += _rgb_led_strip_init(led_strip_list[strip_Debug]);
    // _print_strip_config(strip_Debug);

    // led_cnt += _rgb_led_strip_init(&button_led);    
    // _print_strip_config(strip_Buttons);

    return led_cnt;
}

void drv_rgb_led_strip_deinit(void)
{
    for (rgb_led_strip_type i = strips_Total; i > 0; i--)
    {
        _rgb_led_strip_deinit(led_strip_list[i-1]);
    }

    // _rgb_led_strip_deinit(&dbg_led);
    // _rgb_led_strip_deinit(&button_led);
}

static rmt_transmit_config_t rmt_tx_config = {
    .loop_count = 0, // no transfer loop
};

void drv_rgb_led_strip_set_led(rgb_led_strip_type strip, uint32_t led_nr, uint32_t rgb)
{
    if (strip >= strips_Total) {
        return;
    }

    led_strip_t* led_strip = led_strip_list[strip];
    
    if (led_strip->init_result != ESP_OK) 
    {
        dbgPrint(trRGB, "# %s RGB LED strip not initialised", led_strip->name);
        return;
    }

    uint8_t * rgb_buf = ColourBuffer(strip);
    const char * col_order = ColOrder(strip);
    int bytes_per_led = strlen(col_order);
    led_nr = led_nr % led_strip->led_cnt;
    
    // Build RGB colours in the order as they are expeted for the particular driver
    for (int i = 0; i < bytes_per_led; i++)
    {
        switch (col_order[i])
        {
        case 'R':
        case 'r':
            rgb_buf[led_nr * bytes_per_led + i] = RED_from_WRGB(rgb);
            break;
        case 'G':
        case 'g':
            rgb_buf[led_nr * bytes_per_led + i] = GREEN_from_WRGB(rgb);
            break;
        case 'B':
        case 'b':
            rgb_buf[led_nr * bytes_per_led + i] = BLUE_from_WRGB(rgb);
            break;
        case 'W':
        case 'w':
            rgb_buf[led_nr * bytes_per_led + i] = WHITE_from_WRGB(rgb);
            break;
        default:
            dbgPrint(trRGB, "#Unsupported char ('%c')in colour order str of %s RGB LED strip", col_order[i], led_strip->name);
            break;
        }
    }
    // Flush RGB values to LEDs
    ESP_ERROR_CHECK(rmt_transmit(led_strip->chan, led_strip->rmt_encoder, rgb_buf, bytes_per_led * led_strip->led_cnt, &rmt_tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_strip->chan, portMAX_DELAY));
};

uint32_t drv_rgb_led_strip_count(rgb_led_strip_type strip)
{
    if ((strip >= strips_Total) || (led_strip_list[strip]->init_result != ESP_OK))
        return 0;

    return led_strip_list[strip]->led_cnt;
}

const char * drv_rgb_led_striptype2name(rgb_led_strip_type _type)
{
    if (_type >= strips_Total)
        return NULL;

    return led_strip_list[_type]->name;
}

#undef PRINTF_TAG

/*************************** END OF FILE *************************************/