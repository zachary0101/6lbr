/*   Copyright (c) 2012, Swedish Institute of Computer Science
 *  All rights reserved. 
 *
 *   All rights reserved.
 *
 *   Adapted for rf231 by: Joerg Wolf <gwynpen@googlemail.com>
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
 *
 * 
*/

/**
 *   \addtogroup wireless
 *  @{
*/

/**
 *   \defgroup hal RF231 hardware level drivers
 *   @{
 */

/**
 *  \file
 *  This file contains low-level radio driver code.
 *  This version is optimized for use with the "barebones" RF231bb driver,
 *  which communicates directly with the contiki core MAC layer.
 *  It is optimized for speed at the expense of generality.
 */

/*============================ INCLUDE =======================================*/
#include "contiki-conf.h"

#if DEBUGFLOWSIZE
	extern uint8_t debugflowsize,debugflow[DEBUGFLOWSIZE];
	#define DEBUGFLOW(c) if (debugflowsize<(DEBUGFLOWSIZE-1)) debugflow[debugflowsize++]=c
#else
	#define DEBUGFLOW(c)
#endif

#include <stdlib.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include "rf231hal.h"
#include "dev/models.h"
#include "at86rf231_registermap.h"

hal_irq_callback_t hal_irq_callback = 0;

extern uint8_t rf231_last_correlation, rf231_last_rssi, rf231_smallest_rssi;

/*============================ MACROS =====================================*/

#define SCBR_FIELD_POS_IN_CSR_REG          (8)

/* Value in us used for delay between poll attempts for transceiver access. */
#define TRX_POLL_WAIT_TIME_US       (100)

/* Ratio between max time of TR1 / transceiver poll delay */
#define P_ON_TO_CLKM_ATTEMPTS       ((uint8_t) \
                                     (P_ON_TO_CLKM_AVAILABLE_MAX_US / TRX_POLL_WAIT_TIME_US))

/* Ratio between max time of TR2 / transceiver poll delay */
#define SLEEP_TO_TRX_OFF_ATTEMPTS   ((uint8_t) \
                                     (SLEEP_TO_TRX_OFF_MAX_US / TRX_POLL_WAIT_TIME_US))
/* These link to the RF231BB driver in rf231bb.c */
void* rf231_interrupt(void);

extern hal_rx_frame_t rxframe[RF231_CONF_RX_BUFFERS];
extern uint8_t rxframe_head, rxframe_tail;

/* rf231interruptflag can be printed in the main idle loop for debugging */
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
volatile char rf231interruptflag;
#define INTERRUPTDEBUG(arg) rf231interruptflag=arg
#else
#define PRINTF(...)
#define INTERRUPTDEBUG(arg)
#endif

/** \brief This is a file internal variable that contains the 16 MSB of the
 *         system time.
 *
 *         The system time (32-bit) is the current time in microseconds. For the
 *         AVR microcontroller implementation this is solved by using a 16-bit
 *         timer (Timer1) with a clock frequency of 1MHz. The hal_system_time is
 *         incremented when the 16-bit timer overflows, representing the 16 MSB.
 *         The timer value it self (TCNT1) is then the 16 LSB.
 *
 *  \see hal_get_system_time
 */
static uint16_t hal_system_time = 0;
/**
 * Current state of the transceiver.
 */
uint8_t tal_trx_status;
/**
 * @brief Initializes the transceiver
 *
 * This function is called to initialize the transceiver.
 *
 * @return MAC_SUCCESS  if the transceiver state is changed to TRX_OFF and the
 *                 current device part number and version number are correct;
 *         FAILURE otherwise
 */
static uint8_t trx_init(void)
{
    uint8_t trx_status;
    uint8_t poll_counter = 0;

    hal_set_rst_high();
    hal_set_slptr_low();
    /* Wait typical time of timer TR1. */
    delay_us(P_ON_TO_CLKM_AVAILABLE_TYP_US);
    /* Apply reset pulse */
    hal_set_rst_low();
    delay_us(RST_PULSE_WIDTH_US);
    hal_set_rst_high();
    /* Verify that TRX_OFF can be written */
    do
    {
        /* Wait not more than max. value of TR1. */
        if (poll_counter == P_ON_TO_CLKM_ATTEMPTS)
        {
            return(-1/* FAILURE*/);
        }
        /* Wait a short time interval. */
        delay_us(TRX_POLL_WAIT_TIME_US);
        poll_counter++;
        /* Check if AT86RF231 is connected; omit manufacturer id check */
    } while ((hal_register_read(RG_VERSION_NUM) != AT86RF231_VERSION_NUM) ||
             (hal_register_read(RG_PART_NUM) != AT86RF231_PART_NUM));

    /* Verify that TRX_OFF can be written */
    hal_register_write(RG_TRX_STATE, CMD_TRX_OFF);

    /* Verify that the trx has reached TRX_OFF. */
    poll_counter = 0;
    do
    {
        /* Wait a short time interval. */
        delay_us(TRX_POLL_WAIT_TIME_US);

        trx_status = (uint8_t)hal_subregister_read(SR_TRX_STATUS);

        /* Wait not more than max. value of TR2. */
        if (poll_counter == SLEEP_TO_TRX_OFF_ATTEMPTS)
        {
            return(-1/* FAILURE*/);
        }
        poll_counter++;
    } while (trx_status != TRX_OFF);

    tal_trx_status = TRX_OFF;

    return(0);
}

/* *************** HAL API functions **************************************** */
void hal_init( void ) {
	int ret = 0;
	/* Reset variables used in file. */
	hal_system_time = 0;
	//  hal_reset_flags();
	/* Enable GPIOA clock. Enable AFIO clock. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR,
			RCC_APB2ENR_IOPAEN |
			RCC_APB2ENR_IOPBEN |
			RCC_APB2ENR_IOPCEN |
			RCC_APB2ENR_SPI1EN |
			RCC_APB2ENR_AFIOEN );
	/* The following pins are output pins.  */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,RST);		//reset
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,SLP_TR);	//sleep
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,SEL);		//cs
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
			SCK | MOSI | MISO);		//sck mosi miso
	spi_disable(RF_SPI);
	SPI2_I2SCFGR = 0;
	/* Setup SPI parameters. */
	spi_init_master(RF_SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_16, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
			SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
	spi_set_unidirectional_mode(RF_SPI);
	spi_set_full_duplex_mode(RF_SPI); /* Not receive-only */
	spi_enable_software_slave_management(RF_SPI);
	spi_set_nss_high(RF_SPI);
	spi_enable_ss_output(RF_SPI); /* Required, see NSS, 25.3.1 section. */
	/* Finally enable the SPI. */
	spi_enable(RF_SPI);


	/* Set GPIO4 (in GPIO port C) to 'input float'. */
	gpio_set_mode(RF_IRQ_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, RF_IRQ_PIN);
	gpio_clear(RF_IRQ_PORT, RF_IRQ_PIN);
	/* Configure the EXTI subsystem. */
	exti_select_source(EXTI4, RF_IRQ_PORT);
	exti_set_trigger(EXTI4, EXTI_TRIGGER_RISING);
	exti_enable_request(EXTI4);
	exti_reset_request(EXTI4);
	PRINTF("Enabling interrupts\r\n");
	/* Enable EXTI0 interrupt. */
	nvic_enable_irq(NVIC_EXTI4_IRQ);
	nvic_set_priority(NVIC_EXTI4_IRQ,4);
//@@@!?	timer_init();
//	ret = trx_init();
//	if(ret!=0)
//	{
//		PRINTF("rf231:hal init failed\r\n");
//	}else
//	{
//		PRINTF("rf231:hal init success\r\n");
//	}

}
static u8 spi_readwrite(u32 spi, u8 data)
{
	while (!(SPI_SR(spi) & SPI_SR_TXE))
		;
	SPI_DR(spi) = data;
	while (!(SPI_SR(spi) & SPI_SR_RXNE))
		;
	return SPI_DR(spi);
}
/*----------------------------------------------------------------------------*/
/** \brief  This function reads data from one of the radio transceiver's registers.
 *  \param  address Register address to read from. See datasheet for register map.
 *  \see Look at the at86rf231_registermap.h file for register address definitions.
 *  \returns The actual value of the read register.
 */
uint8_t
hal_register_read( uint8_t address ) {
	// derived frompal_trx_access
	uint8_t register_value;
	ENTER_CRITICAL_REGION();
	// Prepare the command byte
	address |= HAL_TRX_CMD_RR;
	// Start SPI transaction by pulling SEL low
	hal_set_ss_low();
	// Send the write command byte
	spi_readwrite(RF_SPI,address);
	register_value = spi_readwrite(RF_SPI,0x0);
	// Stop the SPI transaction by setting SEL high
	hal_set_ss_high();

	LEAVE_CRITICAL_REGION();
	return (register_value);
}

/*----------------------------------------------------------------------------*/
/** \brief  This function writes a new value to one of the radio transceiver's
 *          registers.
 *  \see Look at the at86rf231_registermap.h file for register address definitions.
 *  \param  address Address of register to write.
 *  \param  value   Value to write.
 */
void
hal_register_write( uint8_t address, uint8_t value ) {

	ENTER_CRITICAL_REGION();

    // Prepare the command byte
    address |= HAL_TRX_CMD_RW;

    // Start SPI transaction by pulling SEL low
    hal_set_ss_low();

    // Send the Read command byte
    spi_readwrite(RF_SPI,address);

    // Write the byte in the transceiver data register
    spi_readwrite(RF_SPI,value);

    // Stop the SPI transaction by setting SEL high
    hal_set_ss_high();

    LEAVE_CRITICAL_REGION();
}

/*----------------------------------------------------------------------------*/
/** \brief  This function reads the value of a specific subregister.
 *
 *  \see Look at the at86rf231_registermap.h file for register and subregister
 *       definitions.
 *
 *  \param  address  Main register's address.
 *  \param  mask  Bit mask of the subregister.
 *  \param  position   Bit position of the subregister
 *  \retval Value of the read subregister.
 */
uint8_t
hal_subregister_read( uint8_t address, uint8_t mask, uint8_t position ) {
	/* Read current register value and mask out subregister. */
	uint8_t register_value = hal_register_read(address);
	register_value &= mask;
	register_value >>= position; /* Align subregister value. */

	return(register_value);
}

/*----------------------------------------------------------------------------*/
/** \brief  This function writes a new value to one of the radio transceiver's
 *          subregisters.
 *
 *  \see Look at the at86rf231_registermap.h file for register and subregister
 *       definitions.
 *
 *  \param  address  Main register's address.
 *  \param  mask  Bit mask of the subregister.
 *  \param  position  Bit position of the subregister
 *  \param  value  Value to write into the subregister.
 */
void
hal_subregister_write( uint8_t address, uint8_t mask, uint8_t position, uint8_t value) {
	/* Read current register value and mask area outside the subregister. */
	volatile uint8_t register_value = hal_register_read(address);
	register_value &= ~mask;

	/* Start preparing the new subregister value. shift in place and mask. */
	value <<= position;
	value &= mask;

	value |= register_value; /* Set the new subregister value. */

	/* Write the modified register value. */
	hal_register_write(address, value);
}

/*----------------------------------------------------------------------------*/
/** \brief  This function will upload a frame from the radio transceiver's frame
 *          buffer.
 *
 *          If the frame currently available in the radio transceiver's frame buffer
 *          is out of the defined bounds. Then the frame length, lqi value and crc
 *          be set to zero. This is done to indicate an error.
 *          This version is optimized for use with contiki RF230BB driver.
 *          The callback routine and CRC are left out for speed in reading the rx buffer.
 *          Any delays here can lead to overwrites by the next packet!
 *
 *  \param  rx_frame    Pointer to the data structure where the frame is stored.
 *  \param  rx_callback Pointer to callback function for receiving one byte at a time.
 */
void
hal_frame_read( hal_rx_frame_t *rx_frame)
{
	uint8_t dummy_rx_data;

	uint8_t phy_status;
	uint8_t frame_length;
	uint8_t *rx_data;

    ENTER_CRITICAL_REGION();
    /* Start SPI transaction by pulling SEL low */
    hal_set_ss_low();
    /* Send the command byte */
    phy_status  = spi_readwrite(RF_SPI,HAL_TRX_CMD_FR);
    frame_length = spi_readwrite(RF_SPI,0);
     /*Check for correct frame length.*/
     if ((frame_length >= HAL_MIN_FRAME_LENGTH) &&
         (frame_length <= HAL_MAX_FRAME_LENGTH)){
       rx_data = rx_frame->data;
       rx_frame->length = frame_length;
       do {
         *rx_data++  = spi_readwrite(RF_SPI,0);
       } while (--frame_length > 0);
       rx_frame->lqi = spi_readwrite(RF_SPI,0);
       rx_frame->crc = 1;
     } else {
		rx_frame->length = 0;
		rx_frame->lqi    = 0;
		rx_frame->crc    = 0;
     }
    /* Stop the SPI transaction by setting SEL high. */
    hal_set_ss_high();
    LEAVE_CRITICAL_REGION();
}

/*----------------------------------------------------------------------------*/
/** \brief  This function will download a frame to the radio transceiver's frame
 *          buffer.
 *
 *  \param  write_buffer    Pointer to data that is to be written to frame buffer.
 *  \param  length          Length of data. The maximum length is 127 bytes.
 */
void
hal_frame_write( uint8_t *write_buffer, uint8_t length )
{
	uint8_t tmp;
	ENTER_CRITICAL_REGION();

	/* Start SPI transaction by pulling SEL low */
	hal_set_ss_low();
	/* Download to the Frame Buffer.
	* When the FCS is autogenerated there is no need to transfer the last two bytes
	* since they will be overwritten.
	*/
	#if !RF231_CONF_CHECKSUM
		length -= 2;
	#endif
	/* Send the command byte */
	spi_readwrite(RF_SPI,HAL_TRX_CMD_FW);
	/* Length */
	spi_readwrite(RF_SPI,length);
	do {
		tmp = *write_buffer++;
		spi_readwrite(RF_SPI, tmp);
		--length;
	} while (length > 0);
	/* Stop the SPI transaction by setting SEL high. */
	hal_set_ss_high();
	LEAVE_CRITICAL_REGION();
}

void exti4_isr(void)
{
	exti_reset_request(EXTI4);
    /* Call interrupt handler */
    if (hal_irq_callback) {
    	(void)hal_irq_callback();
    	PRINTF("exti4_isr\r\n ");
    }
}

void delay_us(volatile int i)
{
	unsigned int a;
	while(i--)
	{
		a=6;
		while(a--);
	}
}
void delay_ms(volatile unsigned int i)
{
	unsigned int a;
	while(i--)
	{
		a=7200;
		while(a--);
	}
}
/* EOF */
