/*
 * Copyright (c) 2007, Swedish Institute of Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 * @(#)$Id: rf231bb.c,v 1.24 2010/12/22 20:10:00 dak664 Exp $
 */
/*
 * This code is almost device independent and should be easy to port.
 * Ported to Atmel RF231 21Feb2010 by dak
 *
 * Renamed and adapted to rf231 by Joerg Wolf <gwynpen@googlemail.com>
 *
 */

#include <stdio.h>
#include <string.h>

#include "contiki.h"

#include "rimestats.h"

#include "dev/leds.h"
//#include "dev/spi.h"
#include "rf231bb.h"
#include "rf231hal.h"

#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/netstack.h"
#include "sys/timetable.h"
#define WITH_SEND_CCA 0

/* Timestamps have not been tested */
#if RF231_CONF_TIMESTAMPS
#include "net/rime/timesynch.h"
#define TIMESTAMP_LEN 3
#else /* RF231_CONF_TIMESTAMPS */
#define TIMESTAMP_LEN 0
#endif /* RF231_CONF_TIMESTAMPS */
/* Nonzero FOOTER_LEN has not been tested */
#define FOOTER_LEN 0

/* RF231_CONF_CHECKSUM=0 for automatic hardware checksum */
#ifndef RF231_CONF_CHECKSUM
#define RF231_CONF_CHECKSUM 0
#endif

/* Autoack setting ignored in non-extended mode */
#ifndef RF231_CONF_AUTOACK
#define RF231_CONF_AUTOACK 1
#endif

/* We need to turn off autoack in promiscuous mode */
#if RF231_CONF_AUTOACK
static bool is_promiscuous;
#endif

/* RF231_CONF_AUTORETRIES is 1 plus the number written to the hardware. */
/* Valid range 1-16, zero disables extended mode. */
#ifndef RF231_CONF_AUTORETRIES
#define RF231_CONF_AUTORETRIES 3
#endif

/* RF231_CONF_CSMARETRIES is number of random-backoff/CCA retries. */
/* The hardware will accept 0-7, but 802.15.4-2003 only allows 5 maximum */
#ifndef RF231_CONF_CSMARETRIES
#define RF231_CONF_CSMARETRIES 5
#endif

//Automatic and manual CRC both append 2 bytes to packets 
#if RF231_CONF_CHECKSUM || defined(RF231BB_HOOK_TX_PACKET)
#include "lib/crc16.h"
#endif
#define CHECKSUM_LEN 2

/* Note the AUX_LEN is equal to the CHECKSUM_LEN in any tested configurations to date! */
#define AUX_LEN (CHECKSUM_LEN + TIMESTAMP_LEN + FOOTER_LEN)
#if AUX_LEN != CHECKSUM_LEN
#warning RF231 Untested Configuration!
#endif

struct timestamp {
  uint16_t time;
  uint8_t authority_level;
};

#define FOOTER1_CRC_OK      0x80
#define FOOTER1_CORRELATION 0x7f

/* Leave radio on when USB powered or for testing low power protocols */
/* This allows DEBUGFLOW indication of packets received when the radio is "off" */
#if RF231_CONF_RADIOALWAYSON
#define RADIOALWAYSON 1
#else
#define RADIOALWAYSON 0
#define RADIOSLEEPSWHENOFF 1
#endif

/* RS232 delays will cause 6lowpan fragment overruns! Use DEBUGFLOW instead. */
#define DEBUG 0
#if DEBUG
//#define PRINTF(FORMAT,args...) printf_P(PSTR(FORMAT),##args)
//#define PRINTSHORT(FORMAT,args...) printf_P(PSTR(FORMAT),##args)
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTSHORT(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#define PRINTSHORT(...)
#endif
#if DEBUG>1
/* Output format is suitable for text2pcap to convert to wireshark pcap file.
 * Use $text2pcap -e 0x809a (these_outputs) capture.pcap
 * Since the hardware calculates and appends the two byte checksum to Tx packets,
 * we just add two zero bytes to the packet dump. Don't forget to enable wireshark
 * 802.15.4 dissection even when the checksum is wrong!
 */
#endif

/* See clock.c and httpd-cgi.c for RADIOSTATS code */
#if RF231_CONF_RADIOSTATS
#define RADIOSTATS 1
#endif
#if RADIOSTATS
uint16_t RF231_sendpackets,RF231_receivepackets,RF231_sendfail,RF231_receivefail;
#endif

#if RADIO_CONF_CALIBRATE_INTERVAL
/* Set in clock.c every 256 seconds */
/* The calibration is automatic when the radio turns on, so not needed when duty cycling */
uint8_t rf231_calibrate;
uint8_t rf231_calibrated; //for debugging, prints from main loop when calibration occurs
#endif

/* Track flow through mac, rdc and radio drivers, see contiki-raven-main.c for example of use */
#if DEBUGFLOWSIZE
extern uint8_t debugflowsize,debugflow[DEBUGFLOWSIZE];
#define DEBUGFLOW(c) if (debugflowsize<(DEBUGFLOWSIZE-1)) debugflow[debugflowsize++]=c
#else
#define DEBUGFLOW(c)
#endif

/* XXX hack: these will be made as Chameleon packet attributes */
rtimer_clock_t rf231_time_of_arrival, rf231_time_of_departure;

int rf231_authority_level_of_sender;

#if RF231_CONF_TIMESTAMPS
static rtimer_clock_t setup_time_for_transmission;
static unsigned long total_time_for_transmission, total_transmission_len;
static int num_transmissions;
#endif

uint8_t volatile rf231_pending;

/* RF231 hardware delay times, from datasheet */
// TODO @@@jwg: check values, see at86rf231.h
typedef enum{
    TIME_TO_ENTER_P_ON               = 510, /**<  Transition time from VCC is applied to P_ON - most favorable case! */
    TIME_P_ON_TO_TRX_OFF             = 510, /**<  Transition time from P_ON to TRX_OFF. */
    TIME_SLEEP_TO_TRX_OFF            = 880, /**<  Transition time from SLEEP to TRX_OFF. */
    TIME_RESET                       = 6,   /**<  Time to hold the RST pin low during reset */
    TIME_ED_MEASUREMENT              = 140, /**<  Time it takes to do a ED measurement. */
    TIME_CCA                         = 140, /**<  Time it takes to do a CCA. */
    TIME_PLL_LOCK                    = 150, /**<  Maximum time it should take for the PLL to lock. */
    TIME_FTN_TUNING                  = 25,  /**<  Maximum time it should take to do the filter tuning. */
    TIME_NOCLK_TO_WAKE               = 6,   /**<  Transition time from *_NOCLK to being awake. */
    TIME_CMD_FORCE_TRX_OFF           = 1,   /**<  Time it takes to execute the FORCE_TRX_OFF command. */
    TIME_TRX_OFF_TO_PLL_ACTIVE       = 180, /**<  Transition time from TRX_OFF to: RX_ON, PLL_ON, TX_ARET_ON and RX_AACK_ON. */
    TIME_STATE_TRANSITION_PLL_ACTIVE = 1,   /**<  Transition time from PLL active state to another. */
}radio_trx_timing_t;



/*---------------------------------------------------------------------------*/
PROCESS(rf231_process, "RF231 driver");
/*---------------------------------------------------------------------------*/

int rf231_on(void);
int rf231_off(void);

//static int rf231_read(void *buf, unsigned short bufsize);

static int rf231_prepare(const void *data, unsigned short len);
static int rf231_transmit(unsigned short len);
static int rf231_send(const void *data, unsigned short len);

static int rf231_receiving_packet(void);
//static int rf231_pending_packet(void);
static int rf231_cca(void);

uint8_t rf231_last_correlation,rf231_last_rssi,rf231_smallest_rssi;
volatile int cw_on;

volatile int ed_done;

const struct radio_driver rf231_driver = {
	rf231_init,
	rf231_prepare,
	rf231_transmit,
	rf231_send,
	rf231_read,
	rf231_cca,
	rf231_receiving_packet,
	rf231_pending_packet,
	rf231_on,
	rf231_off
};

uint8_t RF231_receive_on,RF231_sleeping;
static uint8_t channel;

/* Received frames are buffered to rxframe in the interrupt routine in hal.c */
uint8_t rxframe_head,rxframe_tail;
hal_rx_frame_t rxframe[RF231_CONF_RX_BUFFERS];

/*----------------------------------------------------------------------------*/
/** \brief  This function return the Radio Transceivers current state.
 *
 *  \retval     P_ON               When the external supply voltage (VDD) is
 *                                 first supplied to the transceiver IC, the
 *                                 system is in the P_ON (Poweron) mode.
 *  \retval     BUSY_RX            The radio transceiver is busy receiving a
 *                                 frame.
 *  \retval     BUSY_TX            The radio transceiver is busy transmitting a
 *                                 frame.
 *  \retval     RX_ON              The RX_ON mode enables the analog and digital
 *                                 receiver blocks and the PLL frequency
 *                                 synthesizer.
 *  \retval     TRX_OFF            In this mode, the SPI module and crystal
 *                                 oscillator are active.
 *  \retval     PLL_ON             Entering the PLL_ON mode from TRX_OFF will
 *                                 first enable the analog voltage regulator. The
 *                                 transceiver is ready to transmit a frame.
 *  \retval     BUSY_RX_AACK       The radio was in RX_AACK_ON mode and received
 *                                 the Start of Frame Delimiter (SFD). State
 *                                 transition to BUSY_RX_AACK is done if the SFD
 *                                 is valid.
 *  \retval     BUSY_TX_ARET       The radio transceiver is busy handling the
 *                                 auto retry mechanism.
 *  \retval     RX_AACK_ON         The auto acknowledge mode of the radio is
 *                                 enabled and it is waiting for an incomming
 *                                 frame.
 *  \retval     TX_ARET_ON         The auto retry mechanism is enabled and the
 *                                 radio transceiver is waiting for the user to
 *                                 send the TX_START command.
 *  \retval     RX_ON_NOCLK        The radio transceiver is listening for
 *                                 incomming frames, but the CLKM is disabled so
 *                                 that the controller could be sleeping.
 *                                 However, this is only true if the controller
 *                                 is run from the clock output of the radio.
 *  \retval     RX_AACK_ON_NOCLK   Same as the RX_ON_NOCLK state, but with the
 *                                 auto acknowledge module turned on.
 *  \retval     BUSY_RX_AACK_NOCLK Same as BUSY_RX_AACK, but the controller
 *                                 could be sleeping since the CLKM pin is
 *                                 disabled.
 *  \retval     STATE_TRANSITION   The radio transceiver's state machine is in
 *                                 transition between two states.
 */
static uint8_t
radio_get_trx_state(void)
{
    return hal_subregister_read(SR_TRX_STATUS);
}

/*----------------------------------------------------------------------------*/
/** \brief  This function checks if the radio transceiver is sleeping.
 *
 *  \retval     true    The radio transceiver is in SLEEP or one of the *_NOCLK
 *                      states.
 *  \retval     false   The radio transceiver is not sleeping.
 */
static bool radio_is_sleeping(void)
{
    bool sleeping = false;
    gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN,SLP_TR);
    /* The radio transceiver will be at SLEEP or one of the *_NOCLK states only if */
    /* the SLP_TR pin is high. */
    if (hal_get_slptr() != 0){
        sleeping = true;
    }
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,SLP_TR);
    return sleeping;
}
/*----------------------------------------------------------------------------*/
/** \brief  This function will reset the state machine (to TRX_OFF) from any of
 *          its states, except for the SLEEP state.
 */
static void
radio_reset_state_machine(void)
{
    if (radio_is_sleeping()) DEBUGFLOW('"');
    hal_set_slptr_low();
    delay_us(TIME_NOCLK_TO_WAKE);
    hal_subregister_write(SR_TRX_CMD, CMD_FORCE_TRX_OFF);
    delay_us(TIME_CMD_FORCE_TRX_OFF);
}
/*---------------------------------------------------------------------------*/
static char
rf231_isidle(void)
{
  uint8_t radio_state;
  if (radio_is_sleeping()) {
    DEBUGFLOW(']');
	return 1;
  } else {
	  radio_state = hal_subregister_read(SR_TRX_STATUS);
	  if (radio_state != BUSY_TX_ARET &&
	      radio_state != BUSY_RX_AACK &&
	      radio_state != STATE_TRANSITION_IN_PROGRESS &&
	      radio_state != BUSY_RX && 
	      radio_state != BUSY_TX) {
	    return(1);
	  } else {
	    PRINTF(".%u\r\n",radio_state);
	    return(0);
	  }
  }
}
  
static void
rf231_waitidle(void)
{
  while (1) {
    if (rf231_isidle()) break;
    delay_us(100);
  }
}

/*----------------------------------------------------------------------------*/
/** \brief  This function will change the current state of the radio
 *          transceiver's internal state machine.
 *
 *  \param     new_state        Here is a list of possible states:
 *             - RX_ON        Requested transition to RX_ON state.
 *             - TRX_OFF      Requested transition to TRX_OFF state.
 *             - PLL_ON       Requested transition to PLL_ON state.
 *             - RX_AACK_ON   Requested transition to RX_AACK_ON state.
 *             - TX_ARET_ON   Requested transition to TX_ARET_ON state.
 *
 *  \retval    RADIO_SUCCESS          Requested state transition completed
 *                                  successfully.
 *  \retval    RADIO_INVALID_ARGUMENT Supplied function parameter out of bounds.
 *  \retval    RADIO_WRONG_STATE      Illegal state to do transition from.
 *  \retval    RADIO_BUSY_STATE       The radio transceiver is busy.
 *  \retval    RADIO_TIMED_OUT        The state transition could not be completed
 *                                  within resonable time.
 */
static radio_status_t
radio_set_trx_state(uint8_t new_state)
{
    uint8_t original_state;

    /*Check function paramter and current state of the radio transceiver.*/
    if (!((new_state == TRX_OFF)    ||
          (new_state == RX_ON)      ||
          (new_state == PLL_ON)     ||
          (new_state == RX_AACK_ON) ||
          (new_state == TX_ARET_ON))){
        return RADIO_INVALID_ARGUMENT;
    }

	if (radio_is_sleeping()) {
        return RADIO_WRONG_STATE;
    }

    /* Wait for radio to finish previous operation */
    rf231_waitidle();
 //   for(;;)
 //   {
        original_state = radio_get_trx_state();
  //      if (original_state != BUSY_TX_ARET &&
  //          original_state != BUSY_RX_AACK &&
  //          original_state != BUSY_RX && 
  //          original_state != BUSY_TX)
  //          break;
  //  }

    if (new_state == original_state){
        return RADIO_SUCCESS;
    }


    /* At this point it is clear that the requested new_state is: */
    /* TRX_OFF, RX_ON, PLL_ON, RX_AACK_ON or TX_ARET_ON. */

    /* The radio transceiver can be in one of the following states: */
    /* TRX_OFF, RX_ON, PLL_ON, RX_AACK_ON, TX_ARET_ON. */
    if(new_state == TRX_OFF){
        radio_reset_state_machine(); /* Go to TRX_OFF from any state. */
    } else {
        /* It is not allowed to go from RX_AACK_ON or TX_AACK_ON and directly to */
        /* TX_AACK_ON or RX_AACK_ON respectively. Need to go via RX_ON or PLL_ON. */
        if ((new_state == TX_ARET_ON) &&
            (original_state == RX_AACK_ON)){
            /* First do intermediate state transition to PLL_ON, then to TX_ARET_ON. */
            /* The final state transition to TX_ARET_ON is handled after the if-else if. */
            hal_subregister_write(SR_TRX_CMD, PLL_ON);
            delay_us(TIME_STATE_TRANSITION_PLL_ACTIVE);
        } else if ((new_state == RX_AACK_ON) &&
                 (original_state == TX_ARET_ON)){
            /* First do intermediate state transition to RX_ON, then to RX_AACK_ON. */
            /* The final state transition to RX_AACK_ON is handled after the if-else if. */
            hal_subregister_write(SR_TRX_CMD, RX_ON);
            delay_us(TIME_STATE_TRANSITION_PLL_ACTIVE);
        }

        /* Any other state transition can be done directly. */
        hal_subregister_write(SR_TRX_CMD, new_state);

        /* When the PLL is active most states can be reached in 1us. However, from */
        /* TRX_OFF the PLL needs time to activate. */
        if (original_state == TRX_OFF){
            delay_us(TIME_TRX_OFF_TO_PLL_ACTIVE);
        } else {
            delay_us(TIME_STATE_TRANSITION_PLL_ACTIVE);
        }
    } /*  end: if(new_state == TRX_OFF) ... */

    /*Verify state transition.*/
    radio_status_t set_state_status = RADIO_TIMED_OUT;

    if (radio_get_trx_state() == new_state){
        set_state_status = RADIO_SUCCESS;
    }

    return set_state_status;
}

void
rf231_set_promiscuous_mode(bool isPromiscuous) {
#if RF231_CONF_AUTOACK
    is_promiscuous = isPromiscuous;
/* TODO: Figure out when to pass promisc state to 802.15.4 */
    radio_set_trx_state(is_promiscuous?RX_ON:RX_AACK_ON);
    hal_subregister_write(SR_AACK_PROM_MODE, 1);
#endif
}

bool
rf231_is_ready_to_send() {
	switch(radio_get_trx_state()) {
		case BUSY_TX:
		case BUSY_TX_ARET:
			return false;
	}
	
	return true;
}


static void
flushrx(void)
{
  rxframe[rxframe_head].length=0;
}
/*---------------------------------------------------------------------------*/
static uint8_t locked, lock_on, lock_off;

static void
on( void ) {
	ENERGEST_ON(ENERGEST_TYPE_LISTEN);
	RF231_receive_on = 1;
	PRINTF("rf231_on\r\n");
#ifdef RF231BB_HOOK_RADIO_ON
	RF231BB_HOOK_RADIO_ON();
#endif

	if ( radio_is_sleeping() ) {
	ENERGEST_ON(ENERGEST_TYPE_LED_RED);

		ENTER_CRITICAL_REGION();
		//   DEBUGFLOW('0');
		hal_set_slptr_low();
		delay_us(TIME_SLEEP_TO_TRX_OFF);
		delay_us(TIME_SLEEP_TO_TRX_OFF);//extra delay for now, wake time depends on board capacitance
		//	SREG=sreg;
		LEAVE_CRITICAL_REGION();
	}
	rf231_waitidle();

#if RF231_CONF_AUTOACK
	// radio_set_trx_state(is_promiscuous?RX_ON:RX_AACK_ON);
	radio_set_trx_state(RX_AACK_ON);
	//DEBUGFLOW('a');
#else
	radio_set_trx_state(RX_ON);
	DEBUGFLOW('b');
#endif

	RF231_receive_on = 1;
}
static void
off(void)
{
  PRINTF("rf231_off\r\n");
#ifdef RF231BB_HOOK_RADIO_OFF
  RF231BB_HOOK_RADIO_OFF();
#endif

  /* Wait any transmission to end */
  rf231_waitidle();
  RF231_receive_on = 0;
#if RADIOALWAYSON
/* Do not transmit autoacks when stack thinks radio is off */
  radio_set_trx_state(RX_ON);
//DEBUGFLOW('c');
#else 
  /* Force the device into TRX_OFF. */   
  radio_reset_state_machine();
#if RADIOSLEEPSWHENOFF
  /* Sleep Radio */
  hal_set_slptr_high();
  ENERGEST_OFF(ENERGEST_TYPE_LED_RED);
#endif

#endif /* RADIOALWAYSON */

	RF231_receive_on = 0;
  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
}
/*---------------------------------------------------------------------------*/
#define GET_LOCK() locked = 1
static void RELEASE_LOCK(void) {
  if(lock_on) {
    on();
    lock_on = 0;
  }
  if(lock_off) {
    off();
    lock_off = 0;
  }
  locked = 0;
}
/*---------------------------------------------------------------------------*/
static void
set_txpower(uint8_t power)
{
  if (power > TX_PWR_17_2DBM){
    power=TX_PWR_17_2DBM;
  }
  if (radio_is_sleeping()) {
    DEBUGFLOW('f');
    PRINTF("rf231_set_txpower:Sleeping");  //happens with cxmac
  } else {
    DEBUGFLOW('g');
    hal_subregister_write(SR_TX_PWR, power);
  }
}
/*----------------------------------------------------------------------------*/
/**
    \brief Calibrate the internal RC oscillator

    This function calibrates the internal RC oscillator, based
    on an external 32KHz crystal connected to TIMER2. In order to
    verify the calibration result you can program the CKOUT fuse
    and monitor the CPU clock on an I/O pin.
*/
#define AVR_ENTER_CRITICAL_REGION( ) {uint8_t volatile saved_sreg = SREG; cli( )
#define AVR_LEAVE_CRITICAL_REGION( ) SREG = saved_sreg;}
    uint8_t osccal_original,osccal_calibrated;
/*---------------------------------------------------------------------------*/
int
rf231_init( void ) {

	uint8_t i;
	DEBUGFLOW('I');
	/* Initialize Hardware Abstraction Layer */
	hal_init();
	/* Wait in case VCC just applied */
	delay_us(TIME_TO_ENTER_P_ON);

	/* Calibrate oscillator */
	// printf_P(PSTR("\nBefore calibration OSCCAL=%x\n"),OSCCAL);
	// calibrate_rc_osc_32k();
	// printf_P(PSTR("After calibration OSCCAL=%x\n"),OSCCAL);
	// power on reset TODO:@@@zachary
	hal_set_rst_low();
	hal_set_slptr_low();
	delay_us(RST_PULSE_WIDTH_US);
	hal_set_rst_high();

	/* Set receive buffers empty and point to the first */
	for ( i = 0; i < RF231_CONF_RX_BUFFERS; i++ ) {
		rxframe[i].length = 0;
	}

	rxframe_head = 0;
	rxframe_tail = 0;

	/* Do full rf231 Reset */
	hal_set_rst_low();
	hal_set_slptr_low();
#if 0
  /* On powerup a TIME_RESET delay is needed here, however on some other MCU reset
   * (JTAG, WDT, Brownout) the radio may be sleeping. It can enter an uncertain
   * state (sending wrong hardware FCS for example) unless the full wakeup delay
   * is done.
   * Wake time depends on board capacitance; use 2x the nominal delay for safety.
   * See www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=78725
   */
  delay_us(2*TIME_SLEEP_TO_TRX_OFF);
#else
  delay_us(TIME_RESET);
#endif

	hal_set_rst_high();

	/* Force transition to TRX_OFF */
	hal_subregister_write(SR_TRX_CMD, CMD_FORCE_TRX_OFF);
	delay_us(TIME_P_ON_TO_TRX_OFF);

	/* Verify that it is a supported version */
	/* Note gcc optimizes this away if DEBUG is not set! */
	//ATMEGA128RFA1 - version 4, ID 31
	uint8_t tvers = hal_register_read(RG_VERSION_NUM);
	uint8_t tmanu = hal_register_read(RG_MAN_ID_0);

	if ( (tvers != RF231_REVA) && (tvers != RF231_REVB) ) {
		PRINTF("rf231: Unsupported version %u\r\n",tvers);
	}
	if ( tmanu != SUPPORTED_MANUFACTURER_ID ) {
		PRINTF("rf231: Unsupported manufacturer ID %u\r\n",tmanu);
	}

	PRINTF("rf231: Version %u, ID %u\r\n", tvers, tmanu);

	rf231_warm_reset();

	/* Start the packet receive process */
	process_start(&rf231_process, NULL);  // TODO @@@jwg: moved back from radio_driver.c
	/* Limit tx power for testing miniature Raven mesh */
	#define RF231_MAX_TX_POWER 0
	#ifdef RF231_MAX_TX_POWER
	  set_txpower(RF231_MAX_TX_POWER);  //0=3dbm 15=-17.2dbm
	#endif
	/* Leave radio in on state !*/
	on();

	return 0;
}

/*---------------------------------------------------------------------------*/
/* Used to reinitialize radio parameters without losing pan and mac address, channel, power, etc. */
void rf231_warm_reset(void) {
//	uint8_t i;
  hal_register_write(RG_IRQ_MASK, RF231_SUPPORTED_INTERRUPT_MASK);
  PRINTF("TRX_CTRL_0: %x\r\n", hal_register_read(RG_TRX_CTRL_0));
  /* Set up number of automatic retries 0-15 (0 implies PLL_ON sends instead of the extended TX_ARET mode */
  hal_subregister_write(SR_MAX_FRAME_RETRIES, RF231_CONF_AUTORETRIES );
  PRINTF("max_frame_retries: %x %x\r\n",RF231_CONF_AUTORETRIES, hal_subregister_read(SR_MAX_FRAME_RETRIES));
 /* Set up carrier sense/clear channel assesment parameters for extended operating mode */
  hal_subregister_write(SR_MAX_CSMA_RETRIES, 5 );//highest allowed retries TODO @@@zachary
  hal_register_write(RG_CSMA_BE, 0x80);       //min backoff exponent 0, max 8 (highest allowed) TODO @@@zachary
  hal_register_write(RG_CSMA_SEED_0,hal_register_read(RG_PHY_RSSI) );//upper two RSSI reg bits RND_VALUE are random in rf231
 // hal_register_write(CSMA_SEED_1,42 );

  /* CCA Mode Mode 1=Energy above threshold  2=Carrier sense only  3=Both 0=Either (RF231 only) */
//hal_subregister_write(SR_CCA_MODE,1);  //1 is the power-on default

  /* Carrier sense threshold (not implemented in RF231 or RF231) */
// hal_subregister_write(SR_CCA_CS_THRES,1);

  /* CCA energy threshold = -91dB + 2*SR_CCA_ED_THRESH. Reset defaults to -77dB */
  /* Use RF231 base of -91;  RF231 base is -90 according to datasheet */
#ifdef RF231_CONF_CCA_THRES
#if RF231_CONF_CCA_THRES < -91
#warning
#warning RF231_CONF_CCA_THRES below hardware limit, setting to -91dBm
#warning
  hal_subregister_write(SR_CCA_ED_THRES,0);  
#elif RF231_CONF_CCA_THRES > -61
#warning
#warning RF231_CONF_CCA_THRES above hardware limit, setting to -61dBm
#warning
  hal_subregister_write(SR_CCA_ED_THRES,15);  
#else
  hal_subregister_write(SR_CCA_ED_THRES,(RF231_CONF_CCA_THRES+91)/2);
#endif
#endif

  /* Use automatic CRC unless manual is specified */
#if RF231_CONF_CHECKSUM
  hal_subregister_write(SR_TX_AUTO_CRC_ON, 0);
#else
  hal_subregister_write(SR_TX_AUTO_CRC_ON, 1);
#endif

#define RF231_CONF_DIVERSITY 1
#if (RF231_CONF_DIVERSITY == 1)
  enable_antenna_diversity(1);
#else
  disable_antenna_diversity();
#endif
#if 0
  int i=0;
  for (i=0; i<32; i++) {
    PRINTF("reg %02d : %04x\r\n", (int)i, (unsigned int)hal_register_read(i));
  }
#endif
/* Limit tx power for testing miniature Raven mesh */
#ifdef RF231_MAX_TX_POWER
  set_txpower(RF231_MAX_TX_POWER);  //0=3dbm 15=-17.2dbm
#endif
  /* Call radio interrupt when needed */
  hal_irq_callback = rf231_interrupt;

}
/*---------------------------------------------------------------------------*/
static uint8_t buffer[RF231_MAX_TX_FRAME_LENGTH+AUX_LEN];

static int
rf231_transmit(unsigned short payload_len)
{
  int txpower;
  uint8_t total_len;
  uint8_t radiowason;
  uint8_t tx_result;
#if RF231_CONF_TIMESTAMPS
  struct timestamp timestamp;
#endif /* RF231_CONF_TIMESTAMPS */

  GET_LOCK();

  /* Save receiver state */
  radiowason=RF231_receive_on;

  /* If radio is sleeping we have to turn it on first */
  /* This automatically does the PLL calibrations */
  if (radio_is_sleeping()) {
    hal_set_slptr_low();
	DEBUGFLOW('j');
    delay_us(TIME_SLEEP_TO_TRX_OFF);
	delay_us(TIME_SLEEP_TO_TRX_OFF); //extra delay depends on board capacitance
  } else {
#if RADIO_CONF_CALIBRATE_INTERVAL
  /* If nonzero, do periodic calibration. See clock.c */
    if (rf231_calibrate) {
	  DEBUGFLOW('k');
      hal_subregister_write(SR_PLL_CF_START,1);   //takes 80us max
      hal_subregister_write(SR_PLL_DCU_START,1); //takes 6us, concurrently
      rf231_calibrate=0;
      rf231_calibrated=1;
      delay_us(80); //?
    }
#endif
  }
 
  /* Wait for any previous operation or state transition to finish */
  rf231_waitidle();
  if(RF231_receive_on) {
    ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
  }
  /* Prepare to transmit */
#if RF231_CONF_AUTORETRIES
  radio_set_trx_state(TX_ARET_ON);
  DEBUGFLOW('t');
#else
  radio_set_trx_state(PLL_ON);
  DEBUGFLOW('T');
#endif

  txpower = 0;
  
  if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
    /* Remember the current transmission power */
    txpower = rf231_get_txpower();
    /* Set the specified transmission power */
    set_txpower(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) - 1);
  }

  total_len = payload_len + AUX_LEN;

#if RF231_CONF_TIMESTAMPS

  rtimer_clock_t txtime = timesynch_time();
#endif /* RF231_CONF_TIMESTAMPS */

	ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);

	PRINTF("<<< rf231(seq: %u, len: %u) ", buffer[3], total_len);

	/* Note the dumped packet will have a zero checksum unless compiled with RF231_CONF_CHECKSUM
	* since we don't know what it will be if calculated by the hardware.
	*/
	{
#if 0
		uint8_t i;
		PRINTF("0000");       //Start a new wireshark packet
		for (i=1;i<total_len;i++) PRINTF(" %02x",buffer[i]);	// TODO @@@jwg: buffer[0] contains frame length!
#endif
		PRINTF("\r\n");
	}

 /* Toggle the SLP_TR pin to initiate the frame transmission */
  hal_set_slptr_high();
  delay_us(1);
  hal_set_slptr_low();
  hal_frame_write(buffer, total_len);

#if 0
  PRINTF("<<< rf231(%u): ", total_len);
#if DEBUG//>1
/* Note the dumped packet will have a zero checksum unless compiled with RF231_CONF_CHECKSUM
 * since we don't know what it will be if calculated by the hardware.
 */
  {
    uint8_t i;
    PRINTF("0000");       //Start a new wireshark packet
    for (i=0;i<total_len;i++) PRINTF(" %02x",buffer[i]);
    PRINTF("\r\n");
  }
#endif
#endif

#if RADIOSTATS
  RF231_sendpackets++;
#endif
 
 /* We wait until transmission has ended so that we get an
     accurate measurement of the transmission time.*/
  rf231_waitidle();

 /* Get the transmission result */  
#if RF231_CONF_AUTORETRIES
  tx_result = hal_subregister_read(SR_TRAC_STATUS);
#else
  tx_result=RADIO_TX_OK;
#endif

#ifdef ENERGEST_CONF_LEVELDEVICE_LEVELS
  ENERGEST_OFF_LEVEL(ENERGEST_TYPE_TRANSMIT,rf231_get_txpower());
#endif

 /* Restore the transmission power */
 if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
    set_txpower(txpower & 0xff);
  }
 
  /* Restore receive mode */
  if(radiowason) {
    DEBUGFLOW('l');
    on();
  } else {
    off();
  }

#if RF231_CONF_TIMESTAMPS
  setup_time_for_transmission = txtime - timestamp.time;

  if(num_transmissions < 10000) {
    total_time_for_transmission += timesynch_time() - txtime;
    total_transmission_len += total_len;
    num_transmissions++;
  }

#endif /* RF231_CONF_TIMESTAMPS */

  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
  if(RF231_receive_on) {
    ENERGEST_ON(ENERGEST_TYPE_LISTEN);
	on();
  } else {
#if RADIOALWAYSON
    /* Enable reception */
    on();
#else
    off();
    PRINTF("rf231_transmit: turning radio off\n");

#endif
  }

  RELEASE_LOCK();
#if RF231_INSERTACK
   ack_pending = 0;
#endif
  if (tx_result==1) {               //success, data pending from adressee
		tx_result = RADIO_TX_OK;        //Just show success?
	  if (tx_result==RADIO_TX_OK) {
		RIMESTATS_ADD(lltx);
		if(packetbuf_attr(PACKETBUF_ATTR_RELIABLE))
		  RIMESTATS_ADD(ackrx);		//ack was requested and received
	#if RF231_INSERTACK
	  /* Not PAN broadcast to FFFF, and ACK was requested and received */
	  if (!((buffer[5]==0xff) && (buffer[6]==0xff)) && (buffer[0]&(1<<6)))
		ack_pending=1;
	#endif
	  } else if (tx_result==3) {        //CSMA channel access failure
		DEBUGFLOW('m');
		RIMESTATS_ADD(contentiondrop);
		PRINTF("rf231_transmit: Transmission never started\r\n");
		tx_result = RADIO_TX_COLLISION;
	  } else if (tx_result==5) {        //Expected ACK, none received
		DEBUGFLOW('n');
		tx_result = RADIO_TX_NOACK;
		PRINTF("rf231_transmit: ACK not received\n");
		RIMESTATS_ADD(badackrx);		//ack was requested but not received
	  } else if (tx_result==7) {        //Invalid (Can't happen since waited for idle above?)
		DEBUGFLOW('o');
		tx_result = RADIO_TX_ERR;
	  }
  }
  return tx_result;
}
/*---------------------------------------------------------------------------*/
static int
rf231_prepare(const void *payload, unsigned short payload_len)
{
  int ret = 0;
  uint8_t total_len,*pbuf;
#if RF231_CONF_TIMESTAMPS
  struct timestamp timestamp;
#endif /* RF231_CONF_TIMESTAMPS */
#if RF231_CONF_CHECKSUM
  uint16_t checksum;
#endif /* RF231_CONF_CHECKSUM */
#if RF231_INSERTACK
/* The sequence number is needed to construct the ack packet */
  ack_seqnum=*(((uint8_t *)payload)+2);
#endif

  GET_LOCK();
  DEBUGFLOW('p');

  PRINTF("rf231: sending %d bytes\r\n", payload_len);
//  PRINTSHORT("s%d ",payload_len);

  RIMESTATS_ADD(tx);

#if RF231_CONF_CHECKSUM
  checksum = crc16_data(payload, payload_len, 0);
#endif
 
  /* Copy payload to RAM buffer */
  total_len = payload_len + AUX_LEN;
  if (total_len > RF231_MAX_TX_FRAME_LENGTH){
#if RADIOSTATS
    RF231_sendfail++;
#endif
#if DEBUG
//    printf_P(PSTR("rf231_prepare: packet too large (%d, max: %d)\r\n"),total_len,RF231_MAX_TX_FRAME_LENGTH);
    PRINTF("rf231_prepare: packet too large (%d, max: %d)\r\n" , total_len, RF231_MAX_TX_FRAME_LENGTH);
#endif
    ret = -1;
	goto bail;
  }
  pbuf=&buffer[0];	// TODO @@@jwg: set back to 0!!!!
  memcpy(pbuf,payload,payload_len);
  pbuf+=payload_len;
 // buffer[0] = payload_len;	// TODO @@@jwg:

#if RF231_CONF_CHECKSUM
  memcpy(pbuf,&checksum,CHECKSUM_LEN);
  pbuf+=CHECKSUM_LEN;
#endif

#if RF231_CONF_TIMESTAMPS
  timestamp.authority_level = timesynch_authority_level();
  timestamp.time = timesynch_time();
  memcpy(pbuf,&timestamp,TIMESTAMP_LEN);
  pbuf+=TIMESTAMP_LEN;
#endif
/*------------------------------------------------------------*/  

#ifdef RF231BB_HOOK_TX_PACKET
#if !RF231_CONF_CHECKSUM
  { // Add a checksum before we log the packet out
    uint16_t checksum;
    checksum = crc16_data(payload, payload_len, 0);
    memcpy(buffer+total_len-CHECKSUM_LEN,&checksum,CHECKSUM_LEN);
  }
#endif /* RF231_CONF_CHECKSUM */
  RF231BB_HOOK_TX_PACKET(buffer,total_len);
#endif
  
  PRINTF("rf231_prepare ok\r\n");
bail:
  RELEASE_LOCK();
  return ret;
}
/*---------------------------------------------------------------------------*/
static int
rf231_send(const void *payload, unsigned short payload_len)
{
	int ret = 0;

#ifdef RF231BB_HOOK_IS_SEND_ENABLED
	if(!RF231BB_HOOK_IS_SEND_ENABLED()) {
		goto bail;
	}
#endif
	
//	if((ret=rf231_prepare(payload, payload_len))) {		ORIG
		if((ret=rf231_prepare(payload, payload_len += 2))) {	// FIXME @@@jwg
#if DEBUG
//		printf_P(PSTR("rf231_send: Unable to send, prep failed (%d)\r\n"),ret);
		PRINTF("rf231_send: Unable to send, prep failed (%d)\r\n", ret);
#endif
		goto bail;
	}

	ret = rf231_transmit(payload_len);
	PRINTF("rf231_send ok (%d)\r\n", ret);
bail:
#if RADIOSTATS
    if (ret) RF231_sendfail++;
#endif
	return ret;
}
/*---------------------------------------------------------------------------*/
int
rf231_off(void)
{
  /* Don't do anything if we are already turned off. */
  if(RF231_receive_on == 0) {
    return 0;
  }

  /* If we are called when the driver is locked, we indicate that the
     radio should be turned off when the lock is unlocked. */
  if(locked) {
    lock_off = 1;
    return 1;
  }

  /* If we are currently receiving a packet
     we don't actually switch the radio off now, but signal that the
     driver should switch off the radio once the packet has been
     received and processed, by setting the 'lock_off' variable. */
  if (!rf231_isidle()) {
    lock_off = 1;
	PRINTF("rf231_off: busy receiving\r\n");
    return 1;
  }

  off();
  return 0;
}
/*---------------------------------------------------------------------------*/
int
rf231_on(void)
{
  if(RF231_receive_on) {
    DEBUGFLOW('q');
    return 1;
  }
  if(locked) {
    lock_on = 1;
	DEBUGFLOW('r');
    return 1;
  }

  on();
  return 1;
}
/*---------------------------------------------------------------------------*/
uint8_t
rf231_get_channel( void ) {
	//jackdaw reads zero channel, raven reads correct channel?
	return(hal_subregister_read(SR_CHANNEL));
//	return channel;
}
/*---------------------------------------------------------------------------*/
void
rf231_set_channel(uint8_t c) {
	/* Wait for any transmission to end. */
	rf231_waitidle();
	channel = c;
	hal_subregister_write(SR_CHANNEL,c) ;
	PRINTF("rf231: Set Channel %u\r\n",rf231_get_channel());
}
/*---------------------------------------------------------------------------*/
void
rf231_listen_channel(uint8_t c)
{
 /* Same as set channel but forces RX_ON state for sniffer or energy scan */
  PRINTF("rf231: Listen Channel %u\r\n",c);
  rf231_set_channel(c);
  radio_set_trx_state(RX_ON);
}
/*---------------------------------------------------------------------------*/
void
rf231_set_pan_addr(unsigned pan,
                    unsigned addr,
                    const uint8_t ieee_addr[8])
//rf231_set_pan_addr(uint16_t pan,uint16_t addr,uint8_t *ieee_addr)
{
  PRINTF("rf231: PAN=%x Short Addr=%x\r\n",pan,addr);
  
  uint8_t abyte;
  abyte = pan & 0xFF;
  hal_register_write(RG_PAN_ID_0,abyte);
  abyte = (pan >> 8*1) & 0xFF;
  hal_register_write(RG_PAN_ID_1, abyte);

  abyte = addr & 0xFF;
  hal_register_write(RG_SHORT_ADDR_0, abyte);
  abyte = (addr >> 8*1) & 0xFF;
  hal_register_write(RG_SHORT_ADDR_1, abyte);  

  if (ieee_addr != NULL) {
    PRINTF("rf231: MAC=%x",*ieee_addr);
    hal_register_write(RG_IEEE_ADDR_7, *ieee_addr++);
    PRINTF(":%x",*ieee_addr);
    hal_register_write(RG_IEEE_ADDR_6, *ieee_addr++);
    PRINTF(":%x",*ieee_addr);
    hal_register_write(RG_IEEE_ADDR_5, *ieee_addr++);
    PRINTF(":%x",*ieee_addr);
    hal_register_write(RG_IEEE_ADDR_4, *ieee_addr++);
    PRINTF(":%x",*ieee_addr);
    hal_register_write(RG_IEEE_ADDR_3, *ieee_addr++);
    PRINTF(":%x",*ieee_addr);
    hal_register_write(RG_IEEE_ADDR_2, *ieee_addr++);
    PRINTF(":%x",*ieee_addr);
    hal_register_write(RG_IEEE_ADDR_1, *ieee_addr++);
    PRINTF(":%x",*ieee_addr);
    hal_register_write(RG_IEEE_ADDR_0, *ieee_addr);
    PRINTF("\r\n");
  }
}
/*---------------------------------------------------------------------------*/
/*
 * Interrupt leaves frame intact in FIFO.
 */
#if RF231_CONF_TIMESTAMPS
static volatile rtimer_clock_t interrupt_time;
static volatile int interrupt_time_set;
#endif /* RF231_CONF_TIMESTAMPS */
#if RF231_TIMETABLE_PROFILING
#define rf231_timetable_size 16
TIMETABLE(rf231_timetable);
TIMETABLE_AGGREGATE(aggregate_time, 10);
#endif /* RF231_TIMETABLE_PROFILING */
void*
rf231_interrupt(void)
{
	uint8_t irq_status;
	uint8_t trx_state;
//	uint8_t tmp;
	PRINTF("rf231_interrupt\r\n");
	/* When the carrier wave test mode is active, we don't want to do anything
	 here. */
	if (cw_on)
		return 0;
	/* There has to be something bad going on with SPI. Without this
	read, we get a bogus value from the IRQ status register...
	*/
//	tmp=hal_register_read(0x01);
//	PRINTF("Reg 0x01: %02x\r\n", (unsigned int)tmp);

	irq_status = hal_register_read(RG_IRQ_STATUS);
	PRINTF("IRQ status: %02x\r\n", (unsigned int)irq_status);

	/* The datasheet says "By using the Extended Operating Mode, it is
	recommended to mask IRQ_2 (RX_START)" so we might want to disable
	this completely. At least, we don't want to do anything here.  */
	if (irq_status & HAL_RX_START_MASK) {
		PRINTF("*RX_START ");
	}
	if (irq_status & HAL_IRQ_AMI_MASK) {
		PRINTF("*address match ");
	}

	if (irq_status & HAL_TRX_END_MASK) {
		PRINTF("*TRX_END ");

		trx_state = hal_subregister_read(SR_TRX_STATUS);
		/* The TRX_END interrupt has different meanings depending on which
		state (or mode) the radio is in. If it's in one of the RX
		states, a TRX_END interrupt means that we have received a
		packet. If we are in one of the TX states, it means we have
		transmitted one. */
		if ((trx_state == BUSY_RX_AACK) || (trx_state == RX_ON) ||
		(trx_state == BUSY_RX) || (trx_state == RX_AACK_ON)) {
		/* RX */
			PRINTF("*RX\r\n");
			#if RF231_CONF_AUTOACK
			rf231_last_rssi = hal_subregister_read(SR_ED_LEVEL);
			#endif
			hal_frame_read(&rxframe[rxframe_tail]);
			if (rxframe[rxframe_tail].length > 0) {
				PRINTF("len: %d\r\n", rxframe[rxframe_tail].length);
				rxframe_tail++;
				if (rxframe_tail >= RF231_CONF_RX_BUFFERS) {
					rxframe_tail = 0;
				}
				PRINTF("tail: %d\r\n", rxframe_tail);

				/* Poll the receive process, unless the stack thinks the radio is off */
				#if RADIOALWAYSON
				if (RF231_receive_on) {
					DEBUGFLOW('+');
					#endif
					#if RF231_CONF_TIMESTAMPS
					interrupt_time = timesynch_time();
					interrupt_time_set = 1;
					#endif /* RF231_CONF_TIMESTAMPS */

					process_poll(&rf231_process);

					#if RF231_TIMETABLE_PROFILING
					timetable_clear(&rf231_timetable);
					TIMETABLE_TIMESTAMP(rf231_timetable, "interrupt");
					#endif /* RF231_TIMETABLE_PROFILING */

					rf231_pending = 1;

					#if RADIOSTATS //TODO:This will double count buffered packets
					RF231_receivepackets++;
					#endif
					RIMESTATS_ADD(llrx);
					#if RADIOALWAYSON
					} else {
						DEBUGFLOW('-');
						PRINTF("RF231_receive_on off?");
						rxframe[rxframe_head].length=0;
					}
				#endif
			}
		} else if ((trx_state == TX_ARET_ON) || (trx_state == BUSY_TX_ARET)) {
		/* TX */
			PRINTF("*TX\r\n");
		} else {
			PRINTF("trx_state: %02x\r\n", (unsigned int)trx_state);
			PRINTF("UNKNOWN INTERRUPT\r\n");
		}
	}
	if (irq_status & HAL_TRX_UR_MASK) {
		PRINTF("*TRX_END ");
	}
	if (irq_status & HAL_PLL_UNLOCK_MASK) {
		PRINTF("*PLL_UNLOCK ");
	}
		if (irq_status & HAL_PLL_LOCK_MASK) {
		PRINTF("*PLL_LOCK ");
	}
	if (irq_status & HAL_ED_DONE_MASK) {
		ed_done = 1;
		PRINTF("*ED_DONE ");
	}
	/*
	{
		printf("*UNHANDLED %x\r\n", (unsigned int)irq_status);
	}
	*/
//	PRINTF("\r\n");
	return (void*)NULL/*1*/;
}
/*---------------------------------------------------------------------------*/
/* Process to handle input packets
 * Receive interrupts cause this process to be polled
 * It calls the core MAC layer which calls rf231_read to get the packet
 * rf231processflag can be printed in the main idle loop for debugging
 */
#if 0
uint8_t rf231processflag;
#define RF231PROCESSFLAG(arg) rf231processflag=arg
#else
#define RF231PROCESSFLAG(arg)
#endif

PROCESS_THREAD(rf231_process, ev, data)
{
  int len;
  PROCESS_BEGIN();
  RF231PROCESSFLAG(99);
  PRINTF("rf231_process started\r\n");PRINTF("\r\n");

  while(1) {

	  PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    RF231PROCESSFLAG(42);
#if RF231_TIMETABLE_PROFILING
    TIMETABLE_TIMESTAMP(rf231_timetable, "poll");
#endif /* RF231_TIMETABLE_PROFILING */
    do {
    	rf231_pending = 0;
      packetbuf_clear();

      hal_disable_int();
      len = rf231_read(packetbuf_dataptr(), PACKETBUF_SIZE);
      hal_enable_int();
		PRINTF("rf231_read: %u bytes lqi %u\n",len,rf231_last_correlation);

      if (len > 0) {
        packetbuf_set_datalen(len);

        NETSTACK_RDC.input();
#if RF231_TIMETABLE_PROFILING
        TIMETABLE_TIMESTAMP(rf231_timetable, "end");
        timetable_aggregate_compute_detailed(&aggregate_time,
                                             &rf231_timetable);
        timetable_clear(&rf231_timetable);
#endif /* RF231_TIMETABLE_PROFILING */
      } else if (len < 0) {
#if RADIOSTATS
        RF231_receivefail++;
#endif
      }
    } while(len > 0);
  }

  PROCESS_END();
}
/* Get packet from Radio if any, else return zero.
 * The two-byte checksum is appended but the returned length does not include it.
 * Frames are buffered in the interrupt routine so this routine
 * does not access the hardware or change its status
 */
/*---------------------------------------------------------------------------*/
/*static*/ int
rf231_read( void *buf, unsigned short bufsize ) {
	uint8_t len, *framep;
#if FOOTER_LEN
	uint8_t footer[FOOTER_LEN];
#endif /* FOOTER_LEN */
#if RF231_CONF_CHECKSUM
	uint16_t checksum;
#endif /* RF231_CONF_CHECKSUM */
#if RF231_CONF_TIMESTAMPS
	struct timestamp t;
#endif /* RF231_CONF_TIMESTAMPS */

	/* The length includes the twp-byte checksum but not the LQI byte */
	len = rxframe[rxframe_head].length;

	if ( len==0 ) {
#if RADIOALWAYSON && DEBUGFLOWSIZE
		if (RF231_receive_on==0) {if (debugflow[debugflowsize-1]!='z') DEBUGFLOW('z');} //cxmac calls with radio off?
#endif
		return(0);
	}

#if RADIOALWAYSON
	if ( RF231_receive_on ) {
#else
	if ( radio_is_sleeping() ) {
		DEBUGFLOW('!');
		return(0);
	}
	if ( !RF231_receive_on ) {
		DEBUGFLOW('[');
		return(0);
	}
#endif

#if RF231_CONF_TIMESTAMPS
  if(interrupt_time_set) {
    rf231_time_of_arrival = interrupt_time;
    interrupt_time_set = 0;
  } else {
    rf231_time_of_arrival = 0;
  }
  rf231_time_of_departure = 0;
#endif /* RF231_CONF_TIMESTAMPS */

// PRINTSHORT("r%d",rxframe[rxframe_head].length);  
  PRINTF(">>> rf231(seq: %u, len: %u) ", rxframe[rxframe_head].data[2], rxframe[rxframe_head].length);
#if DEBUG//>1
 {
#if 0
    uint8_t i;
    PRINTF("0000");
    for (i=0;i<rxframe[rxframe_head].length;i++) PRINTF(" %02x",rxframe[rxframe_head].data[i]);
#endif
    PRINTF("\r\n");
  }
#endif

 GET_LOCK();

//if(len > RF231_MAX_PACKET_LEN) {
  if(len > RF231_MAX_TX_FRAME_LENGTH) {
    /* Oops, we must be out of sync. */
    DEBUGFLOW('u');
    flushrx();
    RIMESTATS_ADD(badsynch);
    RELEASE_LOCK();
    return 0;
  }

  if(len <= AUX_LEN) {
    DEBUGFLOW('s');
    PRINTF("len <= AUX_LEN\r\n");
    flushrx();
    RIMESTATS_ADD(tooshort);
    RELEASE_LOCK();
    return 0;
  }

  if(len - AUX_LEN > bufsize) {
    DEBUGFLOW('v');
    PRINTF("len - AUX_LEN > bufsize\r\n");
    flushrx();
    RIMESTATS_ADD(toolong);
    RELEASE_LOCK();
    return 0;
  }
 /* Transfer the frame, stripping the footer, but copying the checksum */
  framep=&(rxframe[rxframe_head].data[0]);
  memcpy(buf,framep,len-AUX_LEN+CHECKSUM_LEN);
  rf231_last_correlation = rxframe[rxframe_head].lqi;

  /* Clear the length field to allow buffering of the next packet */
  rxframe[rxframe_head].length=0;
  rxframe_head++;if (rxframe_head >= RF231_CONF_RX_BUFFERS) rxframe_head=0;
  /* If another packet has been buffered, schedule another receive poll */
  if (rxframe[rxframe_head].length) rf231_interrupt();
  
 /* Point to the checksum */
  framep+=len-AUX_LEN; 
#if RF231_CONF_CHECKSUM
  memcpy(&checksum,framep,CHECKSUM_LEN);
#endif /* RF231_CONF_CHECKSUM */
  framep+=CHECKSUM_LEN;
#if RF231_CONF_TIMESTAMPS
  memcpy(&t,framep,TIMESTAMP_LEN);
#endif /* RF231_CONF_TIMESTAMPS */
  framep+=TIMESTAMP_LEN;
#if FOOTER_LEN
  memcpy(footer,framep,FOOTER_LEN);
#endif
#if RF231_CONF_CHECKSUM
  if(checksum != crc16_data(buf, len - AUX_LEN, 0)) {
    DEBUGFLOW('w');
    PRINTF("checksum failed 0x%04x != 0x%04x\r\n",
      checksum, crc16_data(buf, len - AUX_LEN, 0));
  }
#if FOOTER_LEN
  if(footer[1] & FOOTER1_CRC_OK &&
     checksum == crc16_data(buf, len - AUX_LEN, 0)) {
#endif
#endif /* RF231_CONF_CHECKSUM */

/* Get the received signal strength for the packet, 0-84 dB above rx threshold */
#if 0   //more general
    rf231_last_rssi = rf231_get_raw_rssi();
#else   //faster
#if RF231_CONF_AUTOACK
 //   rf231_last_rssi = hal_subregister_read(SR_ED_LEVEL);  //0-84 resolution 1 dB
    rf231_last_rssi = hal_register_read(RG_PHY_ED_LEVEL);  //0-84, resolution 1 dB
#else
/* last_rssi will have been set at RX_START interrupt */
//  rf231_last_rssi = 3*hal_subregister_read(SR_RSSI);    //0-28 resolution 3 dB
#endif
#endif /* speed vs. generality */

  /* Save the smallest rssi. The display routine can reset by setting it to zero */
  if ((rf231_smallest_rssi==0) || (rf231_last_rssi<rf231_smallest_rssi))
     rf231_smallest_rssi=rf231_last_rssi;

 //   rf231_last_correlation = rxframe[rxframe_head].lqi;
    packetbuf_set_attr(PACKETBUF_ATTR_RSSI, rf231_last_rssi);
    packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, rf231_last_correlation);

    RIMESTATS_ADD(llrx);

#if RF231_CONF_TIMESTAMPS
    rf231_time_of_departure =
      t.time +
      setup_time_for_transmission +
      (total_time_for_transmission * (len - 2)) / total_transmission_len;

    rf231_authority_level_of_sender = t.authority_level;

    packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, t.time);
#endif /* RF231_CONF_TIMESTAMPS */

#if RF231_CONF_CHECKSUM
#if FOOTER_LEN
  } else {
    DEBUGFLOW('x');
    PRINTF("bad crc");
    RIMESTATS_ADD(badcrc);
    len = AUX_LEN;
  }
#endif
#endif

#ifdef RF231BB_HOOK_RX_PACKET
  RF231BB_HOOK_RX_PACKET(buf,len);
#endif

  /* Here return just the data length. The checksum is however still in the buffer for packet sniffing */
  return(len - AUX_LEN);

#if RADIOALWAYSON
} else {
   DEBUGFLOW('y');  //Stack thought radio was off
   return 0;
}
#endif
}

/*---------------------------------------------------------------------------*/
void
rf231_set_txpower(uint8_t power)
{
  GET_LOCK();
  set_txpower(power);
  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
uint8_t
rf231_get_txpower(void)
{
	uint8_t power = TX_PWR_UNDEFINED;
	if (radio_is_sleeping()) {
		PRINTF("rf231_get_txpower:Sleeping");
	} else {
		power = hal_subregister_read(SR_TX_PWR);
	}
	return power;
}

/*---------------------------------------------------------------------------*/
uint8_t
rf231_get_raw_rssi(void)
{
  uint8_t rssi,state;
  bool radio_was_off = 0;

  /*The RSSI measurement should only be done in RX_ON or BUSY_RX.*/
  if(!RF231_receive_on) {
    radio_was_off = 1;
    rf231_on();
  }

/* The energy detect register is used in extended mode (since RSSI will read 0) */
/* The rssi register is multiplied by 3 to a consistent value from either register */
  state=radio_get_trx_state();
  if ((state==RX_AACK_ON) || (state==BUSY_RX_AACK)) {
 //  rssi = hal_subregister_read(SR_ED_LEVEL);  //0-84, resolution 1 dB
     rssi = hal_register_read(RG_PHY_ED_LEVEL);  //0-84, resolution 1 dB
  } else {
#if 0   // 3-clock shift and add is faster on machines with no hardware multiply
/* avr-gcc may have an -Os bug that uses the general subroutine for multiplying by 3 */
     rssi = hal_subregister_read(SR_RSSI);      //0-28, resolution 3 dB
     rssi = (rssi << 1)  + rssi;                //*3
#else  // 1 or 2 clock multiply, or compiler with correct optimization
     rssi = 3 * hal_subregister_read(SR_RSSI);
#endif

  }

  if(radio_was_off) {
    rf231_off();
  }
  return rssi;
}

/*---------------------------------------------------------------------------*/
static int
rf231_cca( void ) {
	uint8_t cca = 0;
	uint8_t radio_was_off = 0;
	int ret = 0;
	/* If the radio is locked by an underlying thread (because we are
	 being invoked through an interrupt), we preted that the coast is
	 clear (i.e., no packet is currently being transmitted by a
	 neighbor). */
	if ( locked ) {
		DEBUGFLOW('|');
		//   return 1; rf231 hangs on occasion?
		return 0;
	}

	/* Don't allow interrupts! */
	//cli();
	ENTER_CRITICAL_REGION();

	/* Turn radio on if necessary. If radio is currently busy return busy channel */
	/* This may happen when testing radio duty cycling with RADIOALWAYSON */

	if ( RF231_receive_on ) {
		if ( radio_is_sleeping() ) { //should not be sleeping!
			DEBUGFLOW('<');
			goto busyexit;
		}
		else {
			if ( !rf231_isidle() ) {
				DEBUGFLOW('2');
				goto busyexit;
			}
		}
	}
	else {
		DEBUGFLOW('3');
		radio_was_off = 1;
		rf231_on();
	}

	/* CCA Mode Mode 1=Energy above threshold  2=Carrier sense only  3=Both 0=Either (RF231 only) */
	/* Use the current mode. Note triggering a manual CCA is not recommended in extended mode */
	//hal_subregister_write(SR_CCA_MODE,1);

	/* Start the CCA, wait till done, return result */
	/* Note reading the TRX_STATUS register clears both CCA_STATUS and CCA_DONE bits */

	hal_subregister_write(SR_CCA_REQUEST, 1);
	delay_us(TIME_CCA);

	while ( (cca & 0x80) == 0 ) {
		cca = hal_register_read(RG_TRX_STATUS);
	}

	if ( radio_was_off ) {
		rf231_off();
	}
//  if (cca & 0x40) {/*DEBUGFLOW('3')*/;} else {rf231_pending=1;DEBUGFLOW('4');}
	if ( cca & 0x40 ) {
		//   DEBUGFLOW('5');
		// 	 SREG=saved_sreg;
		//			LEAVE_CRITICAL_REGION();
		ret = 1;
	}
	else {
		//  DEBUGFLOW('6');
		busyexit:
		//	 SREG=saved_sreg;
		//			LEAVE_CRITICAL_REGION();
		ret = 0;
	}

	LEAVE_CRITICAL_REGION();

	return(ret);
}

/*---------------------------------------------------------------------------*/
int
rf231_receiving_packet(void)
{
  uint8_t radio_state;
  if (radio_is_sleeping()) {
    DEBUGFLOW('=');
  } else {  
    radio_state = hal_subregister_read(SR_TRX_STATUS);
    if ((radio_state==BUSY_RX) || (radio_state==BUSY_RX_AACK)) {
//      DEBUGFLOW('8');
//	  rf231_pending=1;
      return 1;
    }
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
/*static*/ int
rf231_pending_packet( void )
{
	if ( rf231_pending ) {
		DEBUGFLOW('@');
	}

	return(rf231_pending);
}

int rf231_cw_off(void)
{
  int i;
  /*
    19 Register Access 0x1C W 0x00 Disable Continuous Transmission Test Mode
   */
  hal_register_write(0x1c, 0x00);

  /*
    20 RESET Reset AT86RF231
   */
  hal_set_rst_low();

  for(i=0; i<50; i++) {
    volatile uint32_t dummy; (void)dummy;
  }

  hal_set_rst_high();
  return 0;
}

/* Continous wave mode - see datasheet, Appendix A */
int rf231_cw_on(int ant)
{
  int i;
  uint8_t reg;
  uint8_t buf[127];
  int len;

  len = 127;

  cw_on = 1;

  memset(buf, 0xff, len);

  hal_set_rst_low();

  for(i=0; i<50; i++) {
    volatile uint32_t dummy; (void)dummy;
  }

  hal_set_rst_high();
  /*
    2 Register Access 0X0E W 0x01 Set IRQ mask register, enable IRQ_0 (PLL_LOCK)
  */
  //hal_register_write(RG_IRQ_MASK, 0x01);
  hal_register_write(0x0e, 0x01);
  /*
    3 Register Access 0x04 W 0x00 Disable TX_AUTO_CRC_ON
   */
  //hal_register_write(RG_TRX_CTRL_1, 0);
  hal_register_write(0x04, 0);
  /*
    4 Register Access 0x02 W 0x03 Set radio transceiver state TRX_OFF
   */
  //hal_register_write(RG_TRX_STATE, 0x03);
  hal_register_write(0x02, 0x03);
  /*
    5 Register Access 0x03 W 0x01 Set clock at pin 17 (CLKM)
   */
  //hal_register_write(RG_TRX_CTRL_0, 0x01);
  hal_register_write(0x03, 0x01);
  /*
    6 Register Access 0x08 W 0x33 Set IEEE 802.15.4 CHANNEL, e.g. 19
   */
  //hal_register_write(RG_PHY_CC_CCA, 0x33);
  hal_register_write(0x08, 0x3A);
  /*
    7 Register Access 0x05 W 0x00 Set TX output power, e.g. to Pmax
   */
  //hal_register_write(RG_PHY_TX_PWR, 0x00);
  hal_register_write(0x05, 0x00);
  /*
    8 Register Access 0x01 R 0x08 Verify TRX_OFF state
   */
  for (i=0; i<1000; i++) {
    reg = hal_register_read(0x01);
    if (0x08 == reg) {
      break;
    }
  }

  if (0x08 != reg) {
    PRINTF("8 (%02x)\r\n", reg);
    return -1;
  }
  //hal_register_read(RG_TRX_STATUS, 0x08);
  /*
    9 Register Access 0x036 W 0x0F Enable Continuous Transmission Test Mode
    - step # 1
   */
  //hal_register_write(RG_TRX_STATUS, 0x08);
  hal_register_write(0x36, 0x0f);
  /*
    10(1) Register Access 0x0C W 0x03 Enable High Data Rate Mode, 2 Mb/s
   */
  hal_register_write(0x0c, 0x03);
  /*
    11(1) Register Access 0x0A W 0xA7 Configure High Data Rate Mode
   */
  hal_register_write(0x0c, 0x03);
  /*
    12(2) Frame Buffer W - Write PHR and PSDU data (even for CW mode),
    refer to Write Access Table 20-2 on page 176.
   */
  hal_frame_write(buf, len);
  /*
    13 Register Access 0x1C W 0x54 Enable Continuous Transmission Test Mode -
    step # 2
  */
  hal_register_write(0x1c, 0x54);
  /*
    14 Register Access 0x1C W 0x46 Enable Continuous Transmission Test Mode -
    step # 3
   */
  hal_register_write(0x1c, 0x46);
  /*
    15 Register Access 0x02 W 0x09 Enable PLL_ON state
   */
  hal_register_write(0x02, 0x09);
  /*
    16 Interrupt event 0x0F R 0x01 Wait for IRQ_0 (PLL_LOCK)
   */
  for (i=0; i<1000; i++) {
    reg = hal_register_read(0x0f);
    if (0x01 == reg) {
      break;
    }
  }
  if (0x01 != reg) {
    PRINTF("16\r\n");
    return -1;
  }
  enable_antenna_diversity(ant);
   /*
    17 Register Access 0x02 W 0x02 Initiate Transmission, enter BUSY_TX state
  */
  hal_register_write(0x02, 0x02);
  /*
    18 Measurement Perform measurement
   */
  for(i=0; i<50000000; i++) {
    volatile uint32_t dummy; (void)dummy;
  }



  return 0;
}

static int rf231_channel_ed(uint8_t channel, uint8_t *rssi)
{
    int i;


    //PRINTF("Scanning channel %d\r\n", channel);
    hal_subregister_write(SR_CHANNEL, channel);
    rf231_waitidle();

    ed_done = 0;
    hal_register_write(RG_PHY_ED_LEVEL, 0);
    for (i=0; i<1000000; i++) {
      if (ed_done)
        break;
    }
    if (ed_done) {
      *rssi = hal_register_read(RG_PHY_ED_LEVEL);
      return 0;
    }

    PRINTF("Scan never happened\r\n");
    return -1;
}

int rf231_scan(uint8_t ed_val[16])
{
  int i;
  int res;
  for (i=0; i<16; i++) {
    res = rf231_channel_ed(11+i, &ed_val[i]);
    if (0 != res) {
      break;
    }
  }
  if (0 != res) {
    return -1;
  }
  return 0;
}
void enable_antenna_diversity(int antenna)//TODO @@@zachary
{
	  /* Use one antenna */
	  /*
	    ANT_CTRL
	    0 Reserved
	    1 Antenna 1: DIG1 = L, DIG2 = H
	    2 Antenna 0: DIG1 = H, DIG2 = L
	    3 Default value for ANT_EXT_SW_EN = 0.
	    Writing 1 to ANT_CTRL selects the antenna path with the PCB trace
	    antenna. Writing 2 selects the other path.
	  */
	  hal_subregister_write(SR_ANT_CTRL, antenna);	// Use antenna on board
	/* SR_ANT_EXT_SW_EN : AD RF-switch control using differential pin pair DIG1/DIG2
		0: Output pins DIG1 and DIG2 are pulled-down to digital ground
		1: Output pins DIG1 and DIG2 are enabled thus providing a differential control signal for an AD switch.*/
	  hal_subregister_write(ANT_EXT_SW_EN, 1);
	  /*SR_ANT_DIV_EN : control self-contained AD algorithm which automatically chooses the receive antenna
		0: Receive/transmit antenna configured in SR_ANT_CTRL is selected
		1: Receive antenna automatically chosen*/
	  hal_subregister_write(ANT_DIV_EN, 1);


	  /* Set this to 3 when using diversity and 7 if not. Datasheet says so. */
	  hal_subregister_write(SR_PDT_THRES, 3);
	  PRINTF("SR_ANT_CTRL: 0x%x\r\n", (unsigned int)hal_subregister_read(SR_ANT_CTRL));
	  PRINTF("ANT_DIV_EN: 0x%x\r\n", (unsigned int)hal_subregister_read(ANT_DIV_EN));
	  PRINTF("ANT_SEL: 0x%x\r\n", (unsigned int)hal_subregister_read(ANT_SEL));
}
void disable_antenna_diversity(void)//TODO @@@zachary
{
	  /* Use one antenna */
	  /*
	    ANT_CTRL
	    0 Reserved
	    1 Antenna 1: DIG1 = L, DIG2 = H
	    2 Antenna 0: DIG1 = H, DIG2 = L
	    3 Default value for ANT_EXT_SW_EN = 0.
	    Writing 1 to ANT_CTRL selects the antenna path with the PCB trace
	    antenna. Writing 2 selects the other path.
	  */
	  hal_subregister_write(SR_ANT_CTRL, 3);	// Default value for ANT_EXT_SW_EN = 0.
	/* SR_ANT_EXT_SW_EN : AD RF-switch control using differential pin pair DIG1/DIG2
		0: Output pins DIG1 and DIG2 are pulled-down to digital ground
		1: Output pins DIG1 and DIG2 are enabled thus providing a differential control signal for an AD switch.*/
	  hal_subregister_write(ANT_EXT_SW_EN, 0);
	  /*SR_ANT_DIV_EN : control self-contained AD algorithm which automatically chooses the receive antenna
		0: Receive/transmit antenna configured in SR_ANT_CTRL is selected
		1: Receive antenna automatically chosen*/
	  hal_subregister_write(ANT_DIV_EN, 0);


	  /* Set this to 3 when using diversity and 7 if not. Datasheet says so. */
	  hal_subregister_write(SR_PDT_THRES, 7);

	  PRINTF("ANTENNA_DIVERSITY_DISABLED\r\n");
}
