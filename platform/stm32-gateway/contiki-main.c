#include <stdint.h>
#include <sys/unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
/* contiki */
#include <contiki.h>
#include <net/netstack.h>
#include "net/rime/rimeaddr.h"
#include <net/mac/frame802154.h>
#include <contiki-version.h>
#include <sys/process.h>
#include <sys/procinit.h>
#include <etimer.h>
#include <sys/autostart.h>
#include <clock.h>
#include "dev/button-sensor.h"
#include "crc16.h"
/*libopencm3*/
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
/*stm32_node*/
#include "stm32-id.h"
#include "stm32-eeprom.h"
#include "dev/models.h"
#include "dev/leds.h"
#include "debug-uart.h"
#include "rf231bb.h"
#include "platform_prints.h"
/* debug */
#define DEBUG DEBUG_FULL
#include "net/uip-debug.h"
SENSORS(&button_sensor);
#if MAC_ADDRESS_LAST_BYTE
uint8_t mac_address[8] = {0x02, 0x11, 0x22, 0xff, 0xfe, 0x33, 0x44,
MAC_ADDRESS_LAST_BYTE};
#else
uint8_t mac_address[8] = {0x02, 0x11, 0x22, 0xff, 0xfe, 0x33, 0x44, 0x88};
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

  printf("\r\n\r\n\r\nInitialising %s\r\n", CONTIKI_VERSION_STRING);
  printf("Platform u101-stm32f (dev %04x, rev %04x)\r\n", device, revision);
  printf("UID96:       %04x %04x %04x %04x %04x %04x\r\n",
         uid96[0], uid96[1], uid96[2], uid96[3], uid96[4], uid96[5]);
  printf("PseudoID16:  %04x\r\n", pseudoid16);
}
static void
contiki_set_address(rimeaddr_t *addr)
{
  uint16_t uid96[6];
  uint16_t pseudoid16;
  uint8_t *uidaddr;
  int i;

  stm32_id_uid96(uid96);
  pseudoid16 = crc16_data((unsigned char *)uid96, 12, 0);

  /* Assume the last 64 bits bits are unique (may very well not be the case) */
  uidaddr = (uint8_t *)&uid96[2];

  /* Set rime addr based on last bits of UID96*/
  for(i=0; i<RIMEADDR_CONF_SIZE; i++) {
//    addr->u8[i] = uidaddr[i];
    addr->u8[i] = mac_address[i];
  }

#if WITH_UIP6
  /* Construct link-local address based on rime address */
  memcpy(&uip_lladdr.addr, addr->u8, sizeof(uip_lladdr.addr));
#endif

  rimeaddr_set_node_addr(addr);

  printf("Rime address ");
  for(i = 0; i < sizeof(addr->u8); i++) {
    printf("%02x%s", addr->u8[i], ((i<sizeof(addr->u8)-1) ? ":" : "\r\n"));
  }
}

void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	rcc_osc_on(LSI);
	rcc_wait_for_osc_ready(LSI);
  /* Enable clocks for GPIO port A, B and USART1. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR,
      RCC_APB2ENR_IOPAEN |
      RCC_APB2ENR_IOPBEN |
      RCC_APB2ENR_IOPCEN |
      RCC_APB2ENR_AFIOEN | 
      RCC_APB2ENR_USART1EN |
      RCC_APB2ENR_SPI1EN);
	rcc_peripheral_enable_clock(&RCC_APB1ENR,
			RCC_APB1ENR_USART2EN |
			RCC_APB1ENR_SPI2EN);
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
gpio_init(void)
{
	gpio_set_mode(BEEP_GPIO, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, BEEP_PIN);
	gpio_clear(BEEP_GPIO,  BEEP_PIN);
}
static void
contiki_init(void)
{
	int r;
	rimeaddr_t addr;

	leds_init();
	gpio_init();
	leds_on(LEDS_RED);
	stm32_flash_init();
	clock_init();

	/* Initialize process subsystem */
	process_init();

	/* etimers must be started before ctimer_init */
	process_start(&etimer_process, NULL);
	/* rtimers needed for radio cycling */
	rtimer_init();

	ctimer_init();

	contiki_set_address(&addr);

	/* Init network stuff */
	NETSTACK_RADIO.init();
	rf231_set_channel(CHANNEL_802_15_4);
	rf231_set_pan_addr(IEEE802154_PANID, 0, (uint8_t *)&addr.u8);

	/* Flush USART output */
	dbg_drain();

#if WITH_UIP6
	memcpy(&uip_lladdr.addr, &rimeaddr_node_addr.u8, sizeof(uip_lladdr.addr));
	queuebuf_init();
	NETSTACK_RDC.init();
	NETSTACK_MAC.init();
	NETSTACK_NETWORK.init();
  #if DEBUG_ANNOTATE
	print_netstack();
  #endif
#if ! CETIC_6LBR
	process_start(&tcpip_process, NULL);
#endif
  #if DEBUG_ANNOTATE
	print_lladdrs();
  #endif
#endif /* endif WITH_UIP6 */

	NETSTACK_MAC.on();
	NETSTACK_RADIO.on();
	NETSTACK_RDC.on();
	process_start(&sensors_process, NULL);
	print_processes(autostart_processes);
	autostart_start(autostart_processes);
	printf("Processes running\r\n");
}

int
main()
{
	clock_setup();
	dbg_setup_uart();
	contiki_print_banner();

	contiki_init();
#if WITH_UIP6
	/* initialize uip variables */
	memset(uip_buf, 0, UIP_CONF_BUFFER_SIZE);
	uip_len = 0;
#endif
//	watchdog_init();
//	watchdog_start();
	while(1) {
		do {
//			watchdog_periodic();
		} while(process_run() > 0);
    idle_count++;
 //   printf("idle_count:%i\r\n",idle_count);
    /* Idle! */
    /* Stop processor clock */
    /* asm("wfi"::); */ 
	}
	return 0;
}

