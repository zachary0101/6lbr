#include <stdint.h>
#include <stdio.h>
#include <sys/process.h>
#include <sys/procinit.h>
#include <etimer.h>
#include <sys/autostart.h>
#include <clock.h>
#include <string.h>
#include "loader/symbols-def.h"
#include "loader/symtab.h"
#include "rf231bb.h"
#include "net/mac/frame802154.h"
#include "net/mac/framer-802154.h"
#include "net/sicslowpan.h"

#include "net/rime.h"
#include "contiki.h"
#include "contiki-net.h"
#include "contiki-lib.h"

#include "contiki-version.h"
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/cm3/nvic.h>
//#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <errno.h>
#include "crc16.h"
#include "stm32-id.h"
#include "dev/models.h"
#include "dev/leds.h"
#include "debug-uart.h"

#define PRINTA(...) printf(__VA_ARGS__)
/* Put default MAC address in EEPROM */
#if WEBSERVER
extern uint8_t mac_address[8];     //These are defined in httpd-fsdata.c via makefsdata.h
extern uint8_t server_name[16];
extern uint8_t domain_name[30];
#elif MAC_ADDRESS_LAST_BYTE
uint8_t mac_address[8] = {0x02, 0x11, 0x22, 0xff, 0xfe, 0x33, 0x44,
MAC_ADDRESS_LAST_BYTE};
#else
uint8_t mac_address[8] = {0x02, 0x11, 0x22, 0xff, 0xfe, 0x33, 0x44, 0x66};
//uint8_t mac_address[8] EEMEM = {0x00, 0x1a, 0x4c, 0x00, 0x13, 0xb8, 0xdd, 0xb6};
#endif

static void
contiki_print_banner(void)
{
  const char *mhz = "MHz";
  uint16_t uid96[6];
  uint16_t pseudoid16;
  uint16_t flash_kb;
  uint16_t device;
  uint16_t revision;
  stm32_id_uid96(uid96);
  pseudoid16 = crc16_data((unsigned char *)uid96, 12, 0);
  stm32_id_chipinfo(&device, &revision);
  stm32_id_flash_size_kb(&flash_kb);

  printf("\r\n\r\nInitialising %s\r\n", CONTIKI_VERSION_STRING);
  printf("Platform u101-stm32f (dev %04x, rev %04x)\r\n", device, revision);
  printf("UID96:       %04x %04x %04x %04x %04x %04x\r\n",
         uid96[0], uid96[1], uid96[2], uid96[3], uid96[4], uid96[5]);
  printf("PseudoID16:  %04x\r\n", pseudoid16);
}
static void
contiki_set_address(rimeaddr_t *addr)
{
  // uint16_t uid96[6];
  // uint16_t pseudoid16;
  // uint8_t *uidaddr;
   int i;

  // stm32_id_uid96(uid96);
  // pseudoid16 = crc16_data((unsigned char *)uid96, 12, 0);

  // /* Assume the last 64 bits bits are unique (may very well not be the case) */
  // uidaddr = (uint8_t *)&uid96[1];

  /* Set rime addr based on last bits of UID96*/
//  for(i=0; i<RIMEADDR_CONF_SIZE; i++) {
//    addr->u8[i] = uidaddr[i];
//  }
  for(i=0; i<RIMEADDR_CONF_SIZE; i++) {
    addr->u8[i] = mac_address[i];
  }
}

static void
contiki_print_processes(struct process * const processes[])
{
  printf("Starting:\r\n");
  while(*processes != NULL) {
    printf("     %s\r\n", (*processes)->name);
    processes++;
  }
  printf("\r\n");
}
void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

  /* Enable clocks for GPIO port A, B and USART1. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR,
      RCC_APB2ENR_IOPAEN |
      RCC_APB2ENR_IOPBEN |
      RCC_APB2ENR_IOPCEN |
      RCC_APB2ENR_AFIOEN | 
     /* RCC_APB2ENR_USART1EN |*/
      RCC_APB2ENR_SPI1EN);
	rcc_peripheral_enable_clock(&RCC_APB1ENR,
			RCC_APB1ENR_USART2EN/* |
			RCC_APB1ENR_SPI2EN*/);
}

int _write(int file, char *ptr, int len) {
	int sent = -1;
	if (file == 1 || file == 2) {
		sent = dbg_send_bytes((const unsigned char*) ptr, len);
	}
	return sent;
}
void
uip_log(char *msg)
{
	printf("%s\r\n", msg);
}
unsigned int idle_count = 0;

static void
contiki_init(void)
{
	int r;
	rimeaddr_t addr;
	
	clock_setup();

	leds_init();
	leds_on(LEDS_GREEN);
	
	dbg_setup_uart();
	
	clock_init();
	contiki_print_banner();
	stm32_flash_init();
	/* rtimers needed for radio cycling */
	rtimer_init();
	/* Initialize process subsystem */
	process_init();
	/* etimers must be started before ctimer_init */
	process_start(&etimer_process, NULL);

	ctimer_init();
	/* Start radio and radio receive process */
	NETSTACK_RADIO.init();
	/* Set addresses BEFORE starting tcpip process */
	memset(&addr, 0, sizeof(rimeaddr_t));
	contiki_set_address(&addr);
	memcpy(&uip_lladdr.addr, &addr.u8, 8);
	rf231_set_pan_addr(IEEE802154_PANID, 0, (uint8_t *)&addr.u8);
	rf231_set_channel(CHANNEL_802_15_4);
	rimeaddr_set_node_addr(&addr);
	printf("MAC address %x:%x:%x:%x:%x:%x:%x:%x\n",addr.u8[0],addr.u8[1],addr.u8[2],addr.u8[3],addr.u8[4],addr.u8[5],addr.u8[6],addr.u8[7]);
	/* Initialize stack protocols */
	queuebuf_init();
	NETSTACK_RDC.init();
	NETSTACK_MAC.init();
	NETSTACK_NETWORK.init();
	
	printf("%s %s, channel %u",NETSTACK_MAC.name, NETSTACK_RDC.name,rf231_get_channel());
	if (NETSTACK_RDC.channel_check_interval) {//function pointer is zero for sicslowmac
	unsigned short tmp;
	tmp=CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval == 0 ? 1:\
								   NETSTACK_RDC.channel_check_interval());
	if (tmp<65535) printf(", check rate %u Hz",tmp);
	}
	printf("\r\n");
	/* Flush USART output */
	dbg_drain();

	printf("Starting tcpip\r\n");
	process_start(&tcpip_process, NULL);
#if WITH_UIP6
	printf("Tentative link-local IPv6 address ");
	{
		int i, a;
		for(a = 0; a < UIP_DS6_ADDR_NB; a++) {
			if (uip_ds6_if.addr_list[a].isused) {
			for(i = 0; i < 7; ++i) {
					printf("%02x%02x:",
					uip_ds6_if.addr_list[a].ipaddr.u8[i * 2],
					uip_ds6_if.addr_list[a].ipaddr.u8[i * 2 + 1]);
				}
				printf("%02x%02x\r\n",
				uip_ds6_if.addr_list[a].ipaddr.u8[14],
				uip_ds6_if.addr_list[a].ipaddr.u8[15]);
			}
		}
	}
#endif
	NETSTACK_MAC.on();
	NETSTACK_RADIO.on();
	NETSTACK_RDC.on();

	contiki_print_processes(autostart_processes);
	autostart_start(autostart_processes);
	printf("Processes running\r\n");
}

int
main()
{
	contiki_init();
	while(1) {
		do {
		} while(process_run() > 0);
    idle_count++;
    /* Idle! */
    /* Stop processor clock */
    /* asm("wfi"::); */ 
	}
	return 0;
}

