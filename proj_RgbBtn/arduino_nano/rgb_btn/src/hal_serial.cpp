/******************************************************************************
Project:    RGB Button Chaser
Module:     hal_serial.c
Purpose:    This file contains the a modified version of the serial routines 
              found in the Arduino HardwareSerial library. 
Author:     Rudolph van Niekerk
Processor:  Arduino Nano (ATmega328)
Compiler:	Arduino AVR Compiler

It has been modified in the following was:
 1) The receive buffer has been removed... instead a callback is assigned at 
    initialisation which provides the upper layer an ability to parse/buffer 
    the incoming byte.
 2) Line controls have been added to enable/disable an RS485 transceiver. By 
    default it will be enabled, but once disabled it will remain disabled 
    until the next TX complete interrupt is executed. This means that caller 
    MUST wait for the transmission to complete before a new transmission can 
    be queued in RS485 mode.

From Arduinos' HardwareSerial.cpp:
  HardwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
  Modified 14 August 2012 by Alarus
  Modified 3 December 2013 by Matthijs Kooijman

 ******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <util/atomic.h>
#include "Arduino.h"

#include "defines.h"
#include "sys_utils.h"

#define __NOT_EXTERN__
#include "hal_serial.h"
#undef __NOT_EXTERN__

#include "hal_timers.h"

/******************************************************************************
Macros
******************************************************************************/
#ifdef PRINTF_TAG
#undef PRINTF_TAG
#endif
#define PRINTF_TAG ("Serial") /* This must be undefined at the end of the file*/


#ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#if !defined(HAL_SERIAL_TX_BUFFER_SIZE)
  #if ((RAMEND - RAMSTART) < 1023)
    #define HAL_SERIAL_TX_BUFFER_SIZE 16
  #else
    #define HAL_SERIAL_TX_BUFFER_SIZE 64
  #endif
#endif

#define HAL_SERIAL_BAUDRATE         (115200L)

#define HAL_BUS_SILENCE_MAX_MS          (UINT16_MAX)

/******************************************************************************
Warnings
******************************************************************************/
#if (HAL_BUS_SILENCE_MAX_MS > UINT16_MAX)
  #error "HAL_BUS_SILENCE_MAX_MS > UINT16_MAX - not supported"
#endif

#if (HAL_SERIAL_TX_BUFFER_SIZE>256)
  #error "HAL_SERIAL_TX_BUFFER_SIZE > 256 - not supported"
  //typedef uint16_t tx_buffer_index_t;
#else
  #define TX_BUFFER_ATOMIC
  typedef uint8_t tx_buffer_index_t;
#endif


/******************************************************************************
Structs and Unions
******************************************************************************/

/******************************************************************************
Local function definitions
******************************************************************************/
//void _hal_serial_rx_complete_irq(void);

void _hal_serial_tx_udr_empty_irq(void);

/******************************************************************************
Local variables
******************************************************************************/

//This flag ensures that we don't accidentally enable the RS485 transceiver in 
// the TX complete ISR when we are about to start transmitting 
volatile bool _tx_busy = false; 

void (*_rx_irq)(uint8_t) = NULL;
stopwatch_ms_t bus_tmr;

volatile tx_buffer_index_t _tx_buffer_head;
volatile tx_buffer_index_t _tx_buffer_tail;

// Don't put any members after these buffers, since only the first
// 32 bytes of this struct can be accessed quickly using the ldd
// instruction.
unsigned char _tx_buffer[HAL_SERIAL_TX_BUFFER_SIZE];

/******************************************************************************
Local functions
******************************************************************************/

ISR(USART_RX_vect) // USART Rx Complete
{
    unsigned char c;
    //_hal_serial_rx_complete_irq();
    if (bit_is_clear(UCSR0A /**_ucsra*/, UPE0)) 
    {
        // No Parity error, read byte and store it in the buffer if there is
        // room
        c = UDR0 /**_udr*/;
        //If we have a rx callback registered we really do not need to buffer any received data?
        if (_rx_irq)
            _rx_irq(c);
    } 
    else 
    {
        // Parity error, read byte but discard it
        UDR0 /**_udr*/;
    };

    //This calls millis() underneath.... which calls cli()... 
    sys_stopwatch_ms_start(&bus_tmr, HAL_BUS_SILENCE_MAX_MS); //~65 seconds

    /* Receiver Error Flags
        The USART receiver has three error flags: Frame error (FEn), data overrun (DORn) and parity error (UPEn). All can be
        accessed by reading UCSRnA. Common for the error flags is that they are located in the receive buffer together with the
        frame for which they indicate the error status. Due to the buffering of the error flags, the UCSRnA must be read before the
        receive buffer (UDRn), since reading the UDRn I/O location changes the buffer read location. Another equality for the error
        flags is that they can not be altered by software doing a write to the flag location. However, all flags must be set to zero when
        the UCSRnA is written for upward compatibility of future USART implementations. None of the error flags can generate
        interrupts.
        The frame error (FEn) flag indicates the state of the first stop bit of the next readable frame stored in the receive buffer. The
        FEn flag is zero when the stop bit was correctly read (as one), and the FEn flag will be one when the stop bit was incorrect
        (zero). This flag can be used for detecting out-of-sync conditions, detecting break conditions and protocol handling. The FEn
        flag is not affected by the setting of the USBSn bit in UCSRnC since the receiver ignores all, except for the first, stop bits. For
        compatibility with future devices, always set this bit to zero when writing to UCSRnA.
        The data overrun (DORn) flag indicates data loss due to a receiver buffer full condition. A data overrun occurs when the
        receive buffer is full (two characters), it is a new character waiting in the receive shift register, and a new start bit is detected.
        If the DORn flag is set there was one or more serial frame lost between the frame last read from UDRn, and the next frame
        read from UDRn. For compatibility with future devices, always write this bit to zero when writing to UCSRnA. The DORn flag
        is cleared when the frame received was successfully moved from the shift register to the receive buffer.
        The parity error (UPEn) flag indicates that the next frame in the receive buffer had a parity error when received. If parity
        check is not enabled the UPEn bit will always be read zero. For compatibility with future devices, always set this bit to zero
        when writing to UCSRnA. For more details see Section 19.4.1 “Parity Bit Calculation” on page 148 and Section 19.7.5 “Parity
        Checker” on page 154.
    */    
}

ISR(USART_UDRE_vect) // USART, Data Register Empty
{
    _hal_serial_tx_udr_empty_irq();
}

ISR( USART_TX_vect) // USART Tx Complete
{
    if (_tx_buffer_head == _tx_buffer_tail)
        _tx_busy = false;
}

/******************************************************************************
Local functions
******************************************************************************/

void _hal_serial_tx_udr_empty_irq(void)
{
    // IMPORTANT: This function is also called from the Data Register Empty Interrupt Handler

    // If interrupts are enabled, there must be more data in the output
    // buffer. Send the next byte
    unsigned char c = _tx_buffer[_tx_buffer_tail];
    _tx_buffer_tail = (_tx_buffer_tail + 1) % HAL_SERIAL_TX_BUFFER_SIZE;

    UDR0 /**_udr*/ = c;

    // clear the TXC bit -- "can be cleared by writing a one to its bit
    // location". This makes sure flush() won't return until the bytes
    // actually got written. Other r/w bits are preserved, and zeroes
    // written to the rest.

#ifdef MPCM0
    UCSR0A /**_ucsra*/ = ((UCSR0A /**_ucsra*/) & ((1 << U2X0) | (1 << MPCM0))) | (1 << TXC0);
#else
    UCSR0A /**_ucsra*/ = ((UCSR0A /**_ucsra*/) & ((1 << U2X0) | (1 << TXC0)));
#endif

    if (_tx_buffer_head == _tx_buffer_tail) 
    {
        // Buffer empty, so disable interrupts
        cbi(UCSR0B /**_ucsrb*/, UDRIE0);
    }

    /* Transmitter Flags and Interrupts
        The USART transmitter has two flags that indicate its state: USART data register empty (UDREn) and transmit complete
        (TXCn). Both flags can be used for generating interrupts.
        The data register empty (UDREn) flag indicates whether the transmit buffer is ready to receive new data. This bit is set when
        the transmit buffer is empty, and cleared when the transmit buffer contains data to be transmitted that has not yet been
        moved into the shift register. For compatibility with future devices, always write this bit to zero when writing the UCSRnA
        register.
        When the data register empty interrupt enable (UDRIEn) bit in UCSRnB is written to one, the USART data register empty
        interrupt will be executed as long as UDREn is set (provided that global interrupts are enabled). UDREn is cleared by writing
        UDRn. When interrupt-driven data transmission is used, the data register empty interrupt routine must either write new data
        to UDRn in order to clear UDREn or disable the data register empty interrupt, otherwise a new interrupt will occur once the
        interrupt routine terminates.
        The transmit complete (TXCn) flag bit is set one when the entire frame in the transmit shift register has been shifted out and
        there are no new data currently present in the transmit buffer. The TXCn flag bit is automatically cleared when a transmit
        complete interrupt is executed, or it can be cleared by writing a one to its bit location. The TXCn flag is useful in half-duplex
        communication interfaces (like the RS-485 standard), where a transmitting application must enter receive mode and free the
        communication bus immediately after completing the transmission.
        */
}

// Public Methods //////////////////////////////////////////////////////////////

void hal_serial_init(void (*cb_rx_irq)(uint8_t))
{
        // Try u2x mode first
    uint16_t baud_setting = (F_CPU / 4 / HAL_SERIAL_BAUDRATE - 1) / 2;
    UCSR0A /**_ucsra*/ = 1 << U2X0;

    // hardcoded exception for 57600 for compatibility with the bootloader
    // shipped with the Duemilanove and previous boards and the firmware
    // on the 8U2 on the Uno and Mega 2560. Also, The baud_setting cannot
    // be > 4095, so switch back to non-u2x mode if the baud rate is too
    // low.
    if (((F_CPU == 16000000UL) && (HAL_SERIAL_BAUDRATE == 57600)) || (baud_setting >4095))
    {
        UCSR0A /**_ucsra*/ = 0;
        baud_setting = (F_CPU / 8 / HAL_SERIAL_BAUDRATE - 1) / 2;
    }

    // assign the baud_setting, a.k.a. ubrr (USART Baud Rate Register)
    UBRR0H /**_ubrrh*/ = baud_setting >> 8;
    UBRR0L /**_ubrrl*/ = baud_setting;

    //set the data bits, parity, and stop bits
#if defined(__AVR_ATmega8__)
    config |= 0x80; // select UCSRC register (shared with UBRRH)
#endif
    UCSR0C /**_ucsrc*/ = SERIAL_8N1;// 0x06;// config;
    
    sbi(UCSR0B /**_ucsrb*/, RXEN0);     // Enable RX
    sbi(UCSR0B /**_ucsrb*/, TXEN0);     // Enable TX
    sbi(UCSR0B /**_ucsrb*/, TXCIE0);    // Enable TX complete interrupt
    sbi(UCSR0B /**_ucsrb*/, RXCIE0);    // Enable RX interrupt
    cbi(UCSR0B /**_ucsrb*/, UDRIE0);    // Disable Data Register Empty interrupt

    if (cb_rx_irq)
        _rx_irq = cb_rx_irq;
    
    sys_stopwatch_ms_start(&bus_tmr, HAL_BUS_SILENCE_MAX_MS);
}

uint16_t hal_serial_rx_silence_ms(void)
{
    return (uint16_t)(sys_stopwatch_ms_lap(&bus_tmr));
}

void hal_serial_flush()
{
    if (!_tx_busy)
        return;

    // If we get here, nothing is queued anymore (DRIE is disabled) and
    // the hardware finished transmission (TXC is set).

    while (_tx_busy)
    {
        //nop -  Wait for the TX complete interrupt to be called and the buffer is empty
    }
}

size_t hal_serial_write(uint8_t c)
{
    _tx_busy = true; //Will be cleared in the TX complete ISR if the buffer is empty

    // If the buffer and the data register is empty, just write the byte
    // to the data register and be done. This shortcut helps
    // significantly improve the effective datarate at high (>
    // 500kbit/s) bitrates, where interrupt overhead becomes a slowdown.
    if (_tx_buffer_head == _tx_buffer_tail && bit_is_set(UCSR0A /**_ucsra*/, UDRE0)) 
    {
        // If TXC is cleared before writing UDR and the previous byte
        // completes before writing to UDR, TXC will be set but a byte
        // is still being transmitted causing flush() to return too soon.
        // So writing UDR must happen first.
        // Writing UDR and clearing TC must be done atomically, otherwise
        // interrupts might delay the TXC clear so the byte written to UDR
        // is transmitted (setting TXC) before clearing TXC. Then TXC will
        // be cleared when no bytes are left, causing flush() to hang
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
        {
            UDR0 /**_udr*/ = c;
#ifdef MPCM0
            UCSR0A /**_ucsra*/ = ((UCSR0A /**_ucsra*/) & ((1 << U2X0) | (1 << MPCM0))) | (1 << TXC0);
#else
            UCSR0A /**_ucsra*/ = ((UCSR0A /**_ucsra*/) & ((1 << U2X0) | (1 << TXC0)));
#endif
        }
        return 1;
    }
    tx_buffer_index_t i = (_tx_buffer_head + 1) % HAL_SERIAL_TX_BUFFER_SIZE;
        
    // If the output buffer is full, there's nothing for it other than to 
    // wait for the interrupt handler to empty it a bit
    while (i == _tx_buffer_tail) 
    {
        if (bit_is_clear(SREG, SREG_I)) 
        {
            // Interrupts are disabled, so we'll have to poll the data
            // register empty flag ourselves. If it is set, pretend an
            // interrupt has happened and call the handler to free up
            // space for us.
            if(bit_is_set(UCSR0A /**_ucsra*/, UDRE0))
                _hal_serial_tx_udr_empty_irq();
        } 
        else 
        {
            // nop, the interrupt handler will free up space for us
        }
    }

    _tx_buffer[_tx_buffer_head] = c;

    // make atomic to prevent execution of ISR between setting the
    // head pointer and setting the interrupt flag resulting in buffer
    // retransmission
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
    {
        _tx_buffer_head = i;
        sbi(UCSR0B /**_ucsrb*/, UDRIE0);
    }
    
    return 1;
}

#undef PRINTF_TAG

/*************************** END OF FILE *************************************/
