#include "enc28j60.h"
#include "enc28j60-drv.h"
#include "contiki-net.h"
#include "net/uip-neighbor.h"
#include "net/uip-ds6.h"
#include "net/uip-nd6.h"
#include "string.h"
#include "sicslow-ethernet.h"

#include "cetic-6lbr.h"
#include "nvm-config.h"
#include "packet-filter.h"

#define DEBUG 1
#include "net/uip-debug.h"

PROCESS(eth_drv_process, "ENC28J60 driver");

#if UIP_CONF_LLH_LEN == 0
uint8_t ll_header[ETHERNET_LLH_LEN];
#endif

extern void eth_input(void);

//TEMPORARY
uint16_t
uip_ipchksum(void)
{
  return 0;
}

/*---------------------------------------------------------------------------*/

void
eth_drv_send(void)		//length, dst mac, src mac, protocol id, data
{
  PRINTF
    ("ENC28 send: %d bytes : %x:%x:%x:%x:%x:%x %x:%x:%x:%x:%x:%x  %x:%x %x %x %x %x %x %x\r\n",
     uip_len, ll_header[0], ll_header[1], ll_header[2], ll_header[3],
     ll_header[4], ll_header[5], ll_header[6], ll_header[7], ll_header[8],
     ll_header[9], ll_header[10], ll_header[11], ll_header[12], ll_header[13],
     uip_buf[0], uip_buf[1], uip_buf[2], uip_buf[3], uip_buf[4], uip_buf[5]);
  	  ENTER_CRITICAL_REGION();
  	  enc28j60PacketSend(uip_len + sizeof(struct uip_eth_hdr), uip_buf);
  	  LEAVE_CRITICAL_REGION();
}

/*
 * Placeholder - switching off enc28 chip wasn't yet considered
 */
void
eth_drv_exit(void)
{
}

/*
 * Wrapper for lowlevel enc28j60 init code
 * in current configuration it reads the Ethernet driver MAC address
 * from EEPROM memory
 */
void
eth_drv_init()
{
  PRINTF("ENC28J60 init\r\n");
  enc28j60_init(eth_mac_addr);
}

/*---------------------------------------------------------------------------*/
void
enc28j60_pollhandler(void)
{
  //process_poll(&enc28j60_process);
	ENTER_CRITICAL_REGION();
	uip_len = enc28j60PacketReceive(UIP_BUFSIZE, uip_buf);
	LEAVE_CRITICAL_REGION();
	if(uip_len > 0) {
    eth_input();
  }
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(eth_drv_process, ev, data)
{
  //PROCESS_POLLHANDLER(enc28j60_pollhandler());

  PROCESS_BEGIN();

  printf("ENC-28J60 Process started\r\n");
  eth_drv_init();

  ethernet_ready = 1;

  while(1) {
    enc28j60_pollhandler();
    PROCESS_PAUSE();
  }

  eth_drv_exit();

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
