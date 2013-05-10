/*   Copyright (c) 2008, Swedish Institute of Computer Science
 *  All rights reserved. 
 *
 *  Additional fixes for AVR contributed by:
 *
 *	Colin O'Flynn coflynn@newae.com
 *	Eric Gnoske egnoske@gmail.com
 *	Blake Leverett bleverett@gmail.com
 *	Mike Vidales mavida404@gmail.com
 *	Kevin Brown kbrown3@uccs.edu
 *	Nate Bohlmann nate@elfwerks.com
 *
 *	Adaptions for ARM7 by: Joerg Wolf <gwynpen@googlemail.com>
 *
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of the copyright holders nor the names of
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *    \addtogroup hal
 *    @{
 */

/**
 *  \file
 *  \brief This file contains low-level radio driver code.
 *
 *   $Id: hal.h,v 1.5 2010/12/03 20:42:01 dak664 Exp $
*/

#ifndef HAL_STM32F103_H
#define HAL_STM32F103_H
/*============================ INCLUDE =======================================*/
#include <stdint.h>
#include <stdbool.h>
//#include <util/crc16.h>
#include "contiki-conf.h"
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/cm3/nvic.h>

/*============================ MACROS ========================================*/
/** \name Macros for radio operation.
 * \{
 */
#define HAL_BAT_LOW_MASK       ( 0x80 ) /**< Mask for the BAT_LOW interrupt. */
#define HAL_TRX_UR_MASK        ( 0x40 ) /**< Mask for the TRX_UR interrupt. */
#define HAL_ED_DONE_MASK       ( 0x10 ) /**< Mask for the ED_DONE interrupt */
#define HAL_IRQ_AMI_MASK	   ( 0x20 ) /**< Mask for the address match interrupt. */
#define HAL_TRX_END_MASK       ( 0x08 ) /**< Mask for the TRX_END interrupt. */
#define HAL_RX_START_MASK      ( 0x04 ) /**< Mask for the RX_START interrupt. */
#define HAL_PLL_UNLOCK_MASK    ( 0x02 ) /**< Mask for the PLL_UNLOCK interrupt. */
#define HAL_PLL_LOCK_MASK      ( 0x01 ) /**< Mask for the PLL_LOCK interrupt. */

#define HAL_MIN_FRAME_LENGTH   ( 0x03 ) /**< A frame should be at least 3 bytes. */
#define HAL_MAX_FRAME_LENGTH   ( 0x7F ) /**< A frame should no more than 127 bytes. */

//#define nop()           do { __asm__ volatile ("nop"); } while (0)
#define HAL_TRX_CMD_RW         (0xC0) /**<  Register Write (short mode). */
#define HAL_TRX_CMD_RR         (0x80) /**<  Register Read (short mode). */
#define HAL_TRX_CMD_FW         (0x60) /**<  Frame Transmit Mode (long mode). */
#define HAL_TRX_CMD_FR         (0x20) /**<  Frame Receive Mode (long mode). */
#define HAL_TRX_CMD_SW         (0x40) /**<  SRAM Write. */
#define HAL_TRX_CMD_SR         (0x00) /**<  SRAM Read. */
#define HAL_TRX_CMD_RADDRM     (0x7F) /**<  Register Address Mask. */

#define ENTER_CRITICAL_REGION()		__asm__ volatile ("cpsid i")
#define LEAVE_CRITICAL_REGION()		__asm__ volatile ("cpsie i")

/* Enables the transceiver interrupts. */
#define hal_enable_int()                 (nvic_enable_irq(NVIC_EXTI4_IRQ))
/* Disables the transceiver interrupts. */
#define hal_disable_int()                 (nvic_disable_irq(NVIC_EXTI4_IRQ))
/* Clears the transceiver interrupts. */
#define CLEAR_TRX_IRQ()                   ( nvic_clear_pending_irq(NVIC_EXTI4_IRQ))

#define RADIO_IRQ_PIN					(GPIO4) /* PORT C 4*/
/* RESET pin */
#define RST                             (GPIO0)//PB0
/* Sleep Transceiver pin */
#define SLP_TR                          (GPIO5)//PC5
/* Slave select pin */
#define SEL                             (GPIO4)//PA4
/* SPI Bus Master Input/Slave Output pin */
#define MISO                            (GPIO6)//PA6
/* SPI Bus Master Output/Slave Input pin */
#define MOSI                            (GPIO7)//PA7
/* SPI serial clock pin */
#define SCK                             (GPIO5)//PA5
/*
 * Set TRX GPIO pins.
 */
#define hal_set_rst_low()	gpio_clear(GPIOB,RST) /**< This macro pulls the RST pin low. */
#define hal_set_rst_high()	gpio_set(GPIOB,RST) /**< This macro pulls the RST pin high. */
#define hal_set_slptr_high()	gpio_set(GPIOC,SLP_TR)      /**< This macro pulls the SLP_TR pin high. */
#define hal_set_slptr_low()	gpio_clear(GPIOC,SLP_TR)    /**< This macro pulls the SLP_TR pin low. */
#define hal_get_slptr()		gpio_get(GPIOC,SLP_TR)  /**< Read current state of the SLP_TR pin (High/Low). */

/*
 * Slave select made low
 */
#define hal_set_ss_low()                        gpio_clear(GPIOA,SEL)
/*
 * Slave select made high
 */
#define hal_set_ss_high()                       gpio_set(GPIOA,SEL)
/*
 * Dummy value to be written in SPI transmit register to retrieve data form it
 */
#define SPI_DUMMY_VALUE                 (0x00)
/**
 * @brief Block for a given time
 * @param value timeout given in microseconds
 */
//inline void delay_us(unsigned int value)
// execute "no-operation" instructions for some time
void delay_us(volatile int i);
void delay_ms(volatile unsigned int i);
#ifndef RF231_CONF_RX_BUFFERS
#define RF231_CONF_RX_BUFFERS 3
#endif

/*============================ TYPDEFS =======================================*/

/** \struct hal_rx_frame_t
 *  \brief  This struct defines the rx data container.
 *
 *  \see hal_frame_read
 */
typedef struct{
    uint8_t length;                       /**< Length of frame. */
    uint8_t data[ HAL_MAX_FRAME_LENGTH ]; /**< Actual frame data. */
    uint8_t lqi;                          /**< LQI value for received frame. */
    bool crc;                             /**< Flag - did CRC pass for received frame? */
} hal_rx_frame_t;

typedef int (*hal_irq_callback_t)(void);
typedef int (*hal_ed_callback_t)(void);

extern hal_irq_callback_t hal_irq_callback;
/*============================ PROTOTYPES ====================================*/
void 	hal_init( void );
uint8_t hal_register_read( uint8_t address );
void 	hal_register_write( uint8_t address, uint8_t value );
uint8_t hal_subregister_read( uint8_t address, uint8_t mask, uint8_t position );
void 	hal_subregister_write( uint8_t address, uint8_t mask, uint8_t position, uint8_t value );
void	hal_frame_write( uint8_t *write_buffer, uint8_t length );
void	hal_frame_read(hal_rx_frame_t *rx_frame);
#endif
/** @} */
/*EOF*/
