/*******************************************************************************
Module:     task_comms.c
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
#include <ctype.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>
#include "sys_utils.h"
#include "sys_timers.h"
#include "str_helper.h"
#include "sys_task_utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "task_console.h"

#include "driver/uart.h"
#include "esp_timer.h"

#include "driver/gpio.h"

#define __NOT_EXTERN__
#include "task_comms.h"
#undef __NOT_EXTERN__

/*******************************************************************************
Macros and Constants
 *******************************************************************************/
#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("Comms") /* This must be undefined at the end of the file*/

#define USE_BUILTIN_RS485_UART  (1)    /* Use RS485 UART for comms */


#define output_RS485_TX     (GPIO_NUM_4)
#define input_RS485_RX      (GPIO_NUM_5)
#define output_RS485_DE     (GPIO_NUM_6)


#define COMMS_STACK_SIZE 4096
#define COMMS_BAUD_RATE             (115200)    /* 115200 baud rate */

#define COMMS_READ_INTERVAL_MS      20     /* Task cycles at a period of 20ms */

// #define WAIT_BUS_SILENCE_MAX_MS     (15)


// #define RX_BUFF_SIZE_MAX            (WBUS_SILENCE_MIN_MS * (sizeof(comms_msg_hdr_t) + sizeof(uitn8_t)))
// sizeof(comms_msg_hdr_t)

// #define RS485_BUS_SILENCE_MAX_MS    (UINT16_MAX)


#define COMMS_MSG_TX_Q_LEN           (1)
#define COMMS_MSG_RX_Q_LEN           (32)

#define RESPONSE_MSG_SIZE_MIN_SIZE (sizeof(comms_msg_hdr_t) + sizeof(uint8_t) + 2) // Minimum size of a response message is the header + 1 byte crc, cmd and response, respectively

/*******************************************************************************
local defines 
 *******************************************************************************/

 //RGB_BTN_MSG_MAX_LEN
 /*******************************************************************************
 Local structure
 *******************************************************************************/
typedef struct rgb_button_st{
    uint8_t address;        // I2C address (8bits)
    uint8_t state;          // I2C address (8bits)
    uint32_t colour[3];     // 2 colours (8+24bits)
    uint32_t blink_timer;   // blink timer (32bits)
    uint32_t button_time;   // button stopwatch time (32bits)
}rgb_button_t;

const uart_port_t uart_num = UART_NUM_1;

typedef struct{
    uart_config_t uart_config;
    QueueHandle_t rx_queue;
    QueueHandle_t tx_msg_queue;
    QueueHandle_t rx_msg_queue;
#if USE_BUILTIN_RS485_UART == 0    
    gpio_config_t io_config;
#endif
}task_comms_rs485_t;

typedef struct{
	TaskInfo_t task;
    task_comms_rs485_t rs485;
    rgb_button_t btn[32];
}comms_t;

typedef enum e_comms_msg_rx_state
{
    rx_listen,          // Waiting for a message start (ETX)
    rx_busy,            // We are busy receiving a message (between ETX and STX)
    rx_escaping,        // The last byte was a DLE, so we need to xor the next byte with DLE
}comms_rx_state_t;

typedef struct {
    comms_msg_t msg;
    size_t length;
    size_t data_length;
    // int8_t data_rd_index;
    comms_rx_state_t state;
}comms_rx_msg_t;

typedef struct {
    comms_msg_t msg;
    size_t msg_size;
    //RVN - TODO might add a timestamp here
} comms_msg_queue_item_t;


/*******************************************************************************
 Local function prototypes
 *******************************************************************************/
void _comms_main_func(void * pvParameters);
void _rx_msg_handler(uart_event_t *rx_event);
void _tx_msg_handler(comms_msg_queue_item_t *tx_event);
void _comms_deinit(void);
bool _rx_data_process(uint8_t rx_data);
void _bus_silence_expired(void *arg);

void _comms_handler_rc(void);
void _comms_handler_node(void);
/*******************************************************************************
 Local variables
 *******************************************************************************/
// ConsoleMenuItem_t _comms_menu_items[] =
// {
// 									        // 01234567890123456789012345678901234567890123456789012345678901234567890123456789
// //        {"rc",      _comms_handler_rc,      "Triggers a rollcall"},
//         // {"tx",      _comms_handler_tx,      "Transmit a raw message"},
// 		{"node",    _comms_handler_node,    "Sends a message to a specific node"},
// };

comms_t _comms = {
    .task = {
        .init_done = false,
        .handle = NULL,
        .stack_depth = COMMS_STACK_SIZE,
        .stack_unused = 0
        // .init_func = console_init_task,
        // .deinit_func = _console_task_deinit,
    },
    .rs485 = {
        .uart_config = {
            .baud_rate = COMMS_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
            .source_clk = UART_SCLK_DEFAULT,//UART_SCLK_APB,
        },
#if USE_BUILTIN_RS485_UART == 0    
        .io_config = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << output_RS485_DE),
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
        },
#endif
        .rx_queue = NULL
    },
//    .btn = {0} // 32 buttons
};

static esp_timer_create_args_t _bus_silence_tmr_args = {
    .callback = _bus_silence_expired,
    //.dispatch_method;   //!< Call the callback from task or from ISR
    .name = "bus_silence_tmr",  //!< Name of the timer, used for debugging
    //bool skip_unhandled_events;     //!< Skip unhandled events for periodic timers
};

static esp_timer_handle_t _bus_silence_tmr = NULL;


const int comms_uart_buffer_size = (1024);//(RGB_BTN_MSG_MAX_LEN);// + 2; //Absolute worst case scenario.... every character in the msg escaped, plus STX and ETX

comms_rx_msg_t _rx = {0};

//comms_tx_msg_t _tx = {0};
uint8_t _tx_seq = 0; //Sequence number for the next message to be sent

/*******************************************************************************
 Local (private) Functions
 *******************************************************************************/
void _bus_silence_expired(void *arg)
{
    //If the RX state is not in listen mode after 15ms of silence, we force it to  listen mode. 
    // Obviously a transmission was interrupted and we need to start over again.
    if (_rx.state != rx_listen)
    {
        iprintln(trCOMMS, "#Bus silent for 15ms (0x%02X)", _rx.state);
        _rx.state = rx_listen;
    }
}

void _comms_main_func(void * pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    comms_msg_queue_item_t tx_q_msg;


    // Setup UART buffered IO with event queue
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, comms_uart_buffer_size, 0, (2 * RGB_BTN_MSG_MAX_LEN) + 2, &_comms.rs485.rx_queue, 0));
    /* From the function prototype description:     tx_buffer_size -- UART TX ring buffer size. If set to zero, driver will not use TX buffer, 
                                                                        TX function will block task until all data have been sent out.  
        But this does not seem to ring true in pratical terms.... can be tested by setting USE_BUILTIN_RS485_UART to 0*/
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &_comms.rs485.uart_config));

#if USE_BUILTIN_RS485_UART == 0    
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, output_RS485_TX, input_RS485_RX, -1 /* RS485 DE line */, -1 /*output_RS485_CTS*/));
    ESP_ERROR_CHECK(uart_set_mode(UART_NUM_1, UART_MODE_UART)); // Set UART mode
    ESP_ERROR_CHECK(gpio_config(&_comms.rs485.io_config)); // Set GPIO as output
    ESP_ERROR_CHECK(gpio_set_level(output_RS485_DE, 0)); // Set RS485 DE pin to low
    //gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);//(1ULL << output_RS485_TX) | (1ULL << input_RS485_RX) | (1ULL << output_RS485_DE) | (1ULL << 20) | (1ULL << 21));
#else    
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, output_RS485_TX, input_RS485_RX, output_RS485_DE /* RS485 DE line */, -1 /*output_RS485_CTS*/));
    ESP_ERROR_CHECK(uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX /*UART_MODE_UART*/)); // Set UART mode
#endif

    //We are also creating a transmit queue for the RS485 interface, but with only a single item in it.
    _comms.rs485.tx_msg_queue = xQueueCreate(1, sizeof(comms_msg_queue_item_t));
    //and a received queue for the RS485 interface, with space for 32 msg's
    _comms.rs485.rx_msg_queue = xQueueCreate(32, sizeof(comms_msg_queue_item_t));

    ESP_ERROR_CHECK(esp_timer_create(&_bus_silence_tmr_args, &_bus_silence_tmr));

    //Wait until the initialisation is done
    while (!_comms.task.init_done)
        xTaskDelayUntil(&xLastWakeTime, 1 /* 1 Tick */);

    iprintln(trCOMMS|trALWAYS, "#Task Started (%d). Running @ %d Hz", UART_NUM_MAX, (1000/COMMS_READ_INTERVAL_MS));

    ESP_ERROR_CHECK(esp_timer_start_once(_bus_silence_tmr, BUS_SILENCE_MIN_MS));
	while (1)
  	{
        uart_event_t rx_event;

        // Have we received something?
        vTaskDelay(1); //Wait for 1 tick (10ms?) - Max priority, max speed

        //Did we receive something on the RS485 bus?
        if (xQueueReceive(_comms.rs485.rx_queue, (void *)&rx_event, 0) == pdTRUE)//1))//portMAX_DELAY))// pdMS_TO_TICKS(COMMS_READ_INTERVAL_MS) /*(TickType_t)portMAX_DELAY*/) )
        {
            //Restart or start the bus silence timer, depending on its current state
            ESP_ERROR_CHECK(((esp_timer_is_active(_bus_silence_tmr))? esp_timer_restart : esp_timer_start_once)(_bus_silence_tmr, BUS_SILENCE_MIN_MS));
            _rx_msg_handler(&rx_event); //Handle the received message
        }
        //are we waiting for a message to be sent?
#if USE_BUILTIN_RS485_UART == 0    
        else if (_tx.state == tx_wait_for_echo)
        {
            //If the recieve state machine is in listen mode, the bust has been silent for 15ms or the message was corrupted in a way we cannot figure out
            if (_rx.state == rx_listen)
            { 
                //This means 15ms of bus silence has passed and we have not received an echo of the message we just sent
                if ( _tx.retry_cnt < 5)
                {
                    // tx_q_msg should still contain the message we want to (re)send.
                    iprintln(trCOMMS, "#No ECHO Rx'd (seq %d)", tx_q_msg.msg.hdr.id);
                    _tx_msg_handler(&tx_q_msg); //Handle the message to be transmitted
                }
                else
                {
                    //We have retried this message too many times, so we need to give up and move on
                    iprintln(trCOMMS, "#TX Abandonded after %d tries (0x%02X)", _tx.retry_cnt, tx_q_msg.msg.hdr.id);
                    _tx.state = tx_idle; //Return to an idle state, so that we can start a new message
                    ESP_ERROR_CHECK(gpio_set_level(output_RS485_DE, 0)); // Set RS485 DE pin to low
                }
            }
        }
#endif
        //Do we have something to transmit?
        else if  (xQueueReceive(_comms.rs485.tx_msg_queue, &tx_q_msg, 0) == pdTRUE)
        {
#if USE_BUILTIN_RS485_UART == 0    
            _tx.retry_cnt = 0;      //Reset the retry count
#endif
            _tx_msg_handler(&tx_q_msg); //Handle the message to be transmitted
        }
        //Okay, se it seems like we have not received anything nor do we have a message to send
        // but if we are waiting for an echo, we might be in a place where we need to retry the message?
    	_comms.task.stack_unused = uxTaskGetStackHighWaterMark2( NULL );
	}

    _comms_deinit();
}

void _rx_msg_handler(uart_event_t *rx_event)
{
    switch (rx_event->type) 
    {
        case UART_DATA:
            // Data received
            iprintln(trCOMMS, "#Data received: (%d bytes)", rx_event->size);
            for (int i = 0; i < rx_event->size; i++)
            {
                uint8_t data = 0;
                uart_read_bytes(UART_NUM_1, &data, 1, 0);
                if (_rx_data_process(data))
                {
                    //We have a message available, but we need to check if it is valid first
                    uint8_t crc = crc8_n(0, (uint8_t *)&_rx.msg, _rx.length);
                    if (crc != 0)
                    {
                        iprintln(trCOMMS, "#RX Error: CRC (0x%02X vs 0x%02X)", ((uint8_t*)&_rx.msg)[_rx.length-1], (uint8_t)crc8_n(0, (uint8_t *)&_rx.msg, _rx.length-1));
                    }
                    else if (_rx.msg.hdr.version > RGB_BTN_MSG_VERSION)
                    {
                        iprintln(trCOMMS, "#RX Error: Msg version > %d (%d)", RGB_BTN_MSG_VERSION, _rx.msg.hdr.version);
                    }
                    else if (_rx.length < RESPONSE_MSG_SIZE_MIN_SIZE)
                    {
                        iprintln(trCOMMS, "#RX Error: Msg too short > %d (%d)", RESPONSE_MSG_SIZE_MIN_SIZE, _rx.length);
                    }
                    else //CRC is good, Version is Good, Sync # is good - I guess we are done?
                    {
                        comms_msg_queue_item_t _rx_msg_q_item = {0};
                        // iprintln(trCOMMS, "#RX Msg: ");
                        // console_print_memory(trCOMMS, (uint8_t *)&_rx.msg, 0, _rx.length);
                        memcpy(&_rx_msg_q_item.msg, &_rx.msg, _rx.length); //Copy the message to the queue item
                        _rx_msg_q_item.msg_size = _rx.length; //Set the message size
                        //This message can be passed up the queue to the application
                        if (xQueueSend(_comms.rs485.rx_msg_queue, (void *)&_rx_msg_q_item, 0) != pdTRUE)
                        {
                            iprintln(trCOMMS, "#Msg Lost (queue full): ");
                            console_print_memory(trCOMMS, (uint8_t *)&_rx.msg, 0, _rx.length);                            
                        }
                    }
                }
            }
            break;
        case UART_FRAME_ERR:
            // UART frame error detected
            iprintln(trCOMMS, "#Frame error detected");
            break;                    
        case UART_FIFO_OVF:
            // UART RX FIFO overflow
            iprintln(trCOMMS, "#RX FIFO overflow");
            uart_flush_input(UART_NUM_1);
            break;
        case UART_BUFFER_FULL:
            // UART RX buffer full
            iprintln(trCOMMS, "#RX buffer full");
            uart_flush_input(UART_NUM_1);
            break;
        case UART_BREAK:
            // UART RX break detected
            iprintln(trCOMMS, "#RX break detected");
            break;
        default:
            iprintln(trCOMMS, "#??? Event: 0x%04X", rx_event->type);
            break;
    }
}

void _tx_msg_handler(comms_msg_queue_item_t *tx_q_msg)
{
    uint8_t tx_data[2 + (2*RGB_BTN_MSG_MAX_LEN)];
    size_t tx_data_len = 0;
    //No data received on the RS485 bus, but we have a message to send
#if USE_BUILTIN_RS485_UART == 0    
    _tx.retry_cnt++;
    iprintln(trCOMMS, "#TX: %d bytes (%d), attempt %d", tx_q_msg->msg_size, tx_q_msg->msg.hdr.id, _tx.retry_cnt);
#else
    iprintln(trCOMMS, "#TX: %d bytes (%d)", tx_q_msg->msg_size, tx_q_msg->msg.hdr.id);
    console_print_memory(trCOMMS, (uint8_t *)&tx_q_msg->msg, 0, tx_q_msg->msg_size);
#endif

    //wait here for the bus to go silent!
    while (_rx.state != rx_listen)
    {
        //Wait for the bus to be free (if not already free, it should happen in no more than 15ms)
        vTaskDelay(1); //Wait for 1 tick
    }

    //Handle the framing and escaping of the message here.....
    tx_data_len = 0;
    tx_data[tx_data_len++] = STX; //Start of message

    for (uint8_t i = 0; i < tx_q_msg->msg_size; i++)
    {
        uint8_t _d = ((uint8_t *)&tx_q_msg->msg)[i];
        if ((_d == STX) || (_d == DLE) || (_d == ETX))
        {
            tx_data[tx_data_len++] = DLE; //Escape the STX and ETX bytes
            _d ^= DLE;
        }
        tx_data[tx_data_len++] = _d;
    }
    tx_data[tx_data_len++] = ETX; //End of message

#if USE_BUILTIN_RS485_UART == 0    
    _tx.state = tx_wait_for_echo; //From this point on, we are waiting for an echo of the message we just sent
    ESP_ERROR_CHECK(gpio_set_level(output_RS485_DE, 1)); // Set RS485 DE pin to low
#endif    
    uart_write_bytes(UART_NUM_1, tx_data, tx_data_len); //Send the message
    //Since we provided no tx buffer size, this action is supposed to block until the message is sent out
    //However, measuring the data line switching on the scope, it seems like the message is sent through a buffer 
    // since the next line executes before the first byte is even sent.
    //ESP_ERROR_CHECK(gpio_set_level(output_RS485_DE, 0)); // Set RS485 DE pin to low
}

void _comms_deinit(void)
{
    if (_comms.task.init_done)
    {
        uart_driver_delete(UART_NUM_1);
        _comms.task.init_done = false;
        iprintln(trCOMMS, "#Driver de-initialised");
    }
}

bool _rx_data_process(uint8_t rx_data)
{    
    bool return_value = false;

    
    //This is a callback which is called from the serial interrupt handler
    if (rx_data == STX)
    {
        _rx.length = 0;
        _rx.state = rx_busy;
    }    
    else if (_rx.state == rx_busy) 
    {
        if (rx_data == ETX)
        {
#if USE_BUILTIN_RS485_UART == 0
            // Is this a half-duplex echo of what we were busy transmitting just now?
            if (_tx.state == tx_wait_for_echo)
            {
                // if we were sending just now, we want to check if our rx and tx buffers match
                if (memcmp((uint8_t *)&_rx.msg, (uint8_t *)&_tx.msg, min(_rx.length, RGB_BTN_MSG_MAX_LEN)) == 0)
                {
                    //No need to check this message in the application.... we are done with it
                    _tx.state = tx_idle;
                    ESP_ERROR_CHECK(gpio_set_level(output_RS485_DE, 0)); // Set RS485 DE pin to low
                }
                else // only reason this would happen is if we had a bus collision
                    return_value = true;
                // If we remain in this state (tx_wait_for_echo) we will get a timeout once the bus is free again and our retry mechanism will kick in
            }
            else 
                return_value = true;
#else
            return_value = true;
#endif
            _rx.state = rx_listen; 
        }
        else if (rx_data == DLE)
        {
            _rx.state = rx_escaping;
        }
        else 
        {
            if (_rx.length < RGB_BTN_MSG_MAX_LEN)
                ((uint8_t *)&_rx.msg)[_rx.length++] = rx_data;
            // _rx_state = rx_busy;
            //iprintln(trCOMMS, "#RX: 0x%02X", rx_data);
        }
    }
    else if (_rx.state == rx_escaping)
    {
        if (_rx.length < RGB_BTN_MSG_MAX_LEN)
            ((uint8_t *)&_rx.msg)[_rx.length++] = rx_data^DLE;
        //iprintln(trCOMMS, "#RX: 0x%02X *", rx_data);
        _rx.state = rx_busy;
    }
    // else (_rx.state == rx_listen) -> Do nothing, we ignore data outside of STX-ETX
    return return_value;
}

bool _tx_now(comms_tx_msg_t * tx_msg)
{
    comms_msg_queue_item_t _tx_msg_q_item = {0};
    Stopwatch_ms_t _sw = {0};
    uint32_t _elapsed_ms = 0;

    if ((tx_msg->data_length == 0) || (!tx_msg->msg_busy))
        return true; //Nothing to send, but the user might as well think all is well

    //If the transmit queue is not empty, we cannot send a new message
    sys_stopwatch_ms_start(&_sw, UINT16_MAX); /* Do not count past 0xFFFF */
    while (uxQueueMessagesWaiting(_comms.rs485.tx_msg_queue) > 0)
    {
        //Wait for the queue to be free
        _elapsed_ms = sys_stopwatch_ms_lap(&_sw);
        if (_elapsed_ms > (2 * BUS_SILENCE_MIN_MS))
        {
            //We have waited long enough, this TX is not going to happen
            iprintln(trCOMMS, "#TX FAIL: queue busy for %dms", _elapsed_ms);
            return false;
        }
        vTaskDelay(max(1, pdMS_TO_TICKS(BUS_SILENCE_MIN_MS))); //Wait for 1 bus silence period or 1 tick (whichever is longer)
    }

    tx_msg->msg.hdr.id = _tx_seq++; //Set the message ID to the current sequence number
    //This gets incremented every time we send a message, even on retries
    
    //Now we calculate the CRC over the message header and the data
    tx_msg->msg.data[tx_msg->data_length] = crc8_n(0, ((uint8_t *)&tx_msg->msg), sizeof(comms_msg_hdr_t) + tx_msg->data_length); //Calculate the CRC for the newly added data in the message

    _tx_msg_q_item.msg_size = (sizeof(comms_msg_hdr_t) + tx_msg->data_length + sizeof(uint8_t));
    memcpy((uint8_t *)&_tx_msg_q_item.msg, (uint8_t *)&tx_msg->msg, _tx_msg_q_item.msg_size); //Copy the message to the buffer


    //Send the message to the queue
    xQueueSend(_comms.rs485.tx_msg_queue, (void *)&_tx_msg_q_item, 0); 

    //Clear the counters and flags indicating that we are busy with a message
    tx_msg->data_length = 0;
    tx_msg->msg_busy = false; 

    return true; //Message was queued successfully
}

/*******************************************************************************
 Global (public) Functions
 *******************************************************************************/

void * comms_init_task(void)
{

	//Let's not re-initialise this task by accident
	if (_comms.task.handle)
		return &_comms.task;

	// The Console will run on the Debug port
	// Serial.begin(115200, SERIAL_8N1);

	// iprintln(trALWAYS,	"ALWAYS  - Print a number %d", 1);
	// iprintln(trBAT, 		"BAT     - Print a number %d", 2);
	// iprintln(trCONSOLE,	"CONSOLE - Print a number %d", 3);
	// iprintln(trGPS, 		"GPS     - Print a number %d", 4);
	// iprintln(trAPP, 		"APP     - Print a number %d", 5);
	// iprintln(trNONE,   	"NONE    - Print a number %d", 6);

	// Create the task, storing the handle.  Note that the passed parameter ucParameterToPass
	// must exist for the lifetime of the task, so in this case is declared static.  If it was just an
	// an automatic stack variable it might no longer exist, or at least have been corrupted, by the time
	// the new task attempts to access it.
	if (xTaskCreate(_comms_main_func, PRINTF_TAG, _comms.task.stack_depth, _comms.task.parameter_to_pass, 8, &_comms.task.handle ) != pdPASS)
	{
		iprintln(trALWAYS, "#Unable to start %s Task!", PRINTF_TAG);
		return NULL;
	}

	configASSERT(_comms.task.handle);

    _comms.task.init_done = true;

#ifdef CONSOLE_ENABLED
    // console_add_menu("comms", _comms_menu_items, ARRAY_SIZE(_comms_menu_items), "Comms Control");
#endif    

    iprintln(trALWAYS, "#Init OK");

    return &_comms.task;
}

esp_err_t task_comms_tx(uint8_t *tx_data, size_t tx_size)
{
    if (!_comms.task.init_done)
    {
        iprintln(trCOMMS, "#Driver not initialised");
        return ESP_FAIL;
    }
    // Write data to UART.
    int tx_cnt = uart_write_bytes(uart_num, (const char*)tx_data, tx_size);
    if (tx_cnt < ((int)tx_size))
    {
        iprintln(trCOMMS, "#Failed to write ALL data: %d/%d", tx_cnt, ((int)tx_size));
        return ESP_FAIL;
    }

    //RVN - TODO - We need to wake the task up from here....
    return ESP_OK;
}

bool comms_msg_rx_read(comms_msg_t *msg, size_t *msg_size)
{
    comms_msg_queue_item_t rx_q_msg;
    if(xQueueReceive(_comms.rs485.rx_msg_queue, &rx_q_msg, 0) == pdTRUE)
    {
        if (msg != NULL)
            memcpy(msg, &rx_q_msg.msg, rx_q_msg.msg_size);

        if (msg_size != NULL)
            *msg_size = rx_q_msg.msg_size;

        return true; //Message was read successfully
    }
    return false; //No message was available to read
}

void comms_tx_msg_init(comms_tx_msg_t * tx_msg, uint8_t node_addr)
{
    memset(tx_msg, 0, sizeof(comms_tx_msg_t)); //Clear the message structure
    tx_msg->data_length = 0;//sizeof(comms_msg_hdr_t);         //Reset to the beginning of the data
    tx_msg->msg_busy = true; //We are busy building a message

    tx_msg->msg.hdr.version = RGB_BTN_MSG_VERSION;  //Superfluous, but just in case
    tx_msg->msg.hdr.src = ADDR_MASTER;              //Our Address (might have changed since our last message)
    tx_msg->msg.hdr.dst = node_addr;                //We only ever talk to the master!!!!!
}

bool comms_tx_msg_append(comms_tx_msg_t * tx_msg, uint8_t node_addr, master_command_t cmd, uint8_t * data, uint8_t data_len, bool restart)
{
    // We cannot send a message if we are not ready to send now, otherwise we will overwrite data being sent
    if ((!tx_msg->msg_busy) || (restart))
        comms_tx_msg_init(tx_msg, node_addr); //We are NOT busy building a message (or restarting a new one)

    if ((tx_msg->msg_busy) && (node_addr != tx_msg->msg.hdr.dst))
    {
        iprintln(trCOMMS, "#ERROR: Node addr (0x%02X) different than msg init (0x%02X). Cmd = 0x%02X", node_addr, tx_msg->msg.hdr.dst, cmd);
        return false; //We are not busy building a message, so we cannot add anything to it
    }

    //We *know* we will be adding at least 2 bytes (cmd and crc)
    if ((uint8_t)(tx_msg->data_length + data_len + 2) > sizeof(tx_msg->msg.data))
    {
        iprintln(trCOMMS, "#ERROR: Not enough space for cmd 0x%02X, len = %d (dst: 0x%02X, available space = %d)", cmd, data_len, node_addr, (sizeof(tx_msg->msg.data) - tx_msg->data_length));
        return false; //This is NEVER gonna fit!!!
    }
    
    //The master should not bother with "breaking" the message up into smaller chunks. 
    // Currently the system does not support fragmented messages to the nodes
    
    tx_msg->msg.data[tx_msg->data_length++] = cmd;
    if ((data) && (data_len > 0))
    {
        memcpy(&tx_msg->msg.data[tx_msg->data_length], data, data_len);
        //Now we increment the datalenth
        tx_msg->data_length += data_len; // for the payload
    }

    return true;
}

bool comms_tx_msg_send(comms_tx_msg_t * tx_msg)
{
    //We are not busy building a message, so we cannot send anything
    if (!tx_msg->msg_busy)
    {
        iprintln(trCOMMS, "#ERROR: No message to send");
        return false;
    }

    //This is all that is needed... this command can be sent immediately
    return _tx_now(tx_msg);
}

#undef PRINTF_TAG
#undef EXT
/*************************** END OF FILE *************************************/
