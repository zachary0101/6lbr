#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int8_t s8_t;
typedef int16_t s16_t;
typedef int32_t s32_t;
typedef unsigned int clock_time_t;
typedef unsigned int uip_stats_t;

#define CCIF
#define CLIF
#define AUTOSTART_ENABLE	1
#define CLOCK_CONF_SECOND 1000
#define WITH_UIP6 1
//#define WITH_UIP 0
#define WITH_ASCII 1


/* uIP configuration */
#define uip_ipaddr_copy(dest, src) memcpy(dest, src, sizeof(*dest))

#define RF231_CONF_RADIOALWAYSON 1

/* RF230BB must be used with low power protocols */

#define SICSLOWPAN_CONF_CONVENTIONAL_MAC  1     //for barebones driver, sicslowpan calls radio->read function
//#undef PACKETBUF_CONF_HDR_SIZE                  //RF230BB takes the packetbuf default for header size


#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS 2
#if WITH_UIP6
#define RIMEADDR_CONF_SIZE              8
/* Network setup for IPv6 */
#define NETSTACK_CONF_NETWORK           sicslowpan_driver
#define NETSTACK_CONF_MAC               nullmac_driver
#define NETSTACK_CONF_RDC               sicslowmac_driver
#define NETSTACK_CONF_RADIO             rf231_driver
#define NETSTACK_CONF_FRAMER            framer_802154
#define RF231_CONF_AUTOACK        1
#define RF231_CONF_AUTORETRIES    2
#define SICSLOWPAN_CONF_FRAG      1
//Most browsers reissue GETs after 3 seconds which stops frag reassembly, longer MAXAGE does no good
#define SICSLOWPAN_CONF_MAXAGE    3
#define QUEUEBUF_CONF_NUM         1
#define QUEUEBUF_CONF_REF_NUM     1
#define CHANNEL_802_15_4                26
#else /* WITH_UIP6 */

/* Network setup for non-IPv6 (rime). */
#define RIMEADDR_CONF_SIZE              2

#define NETSTACK_CONF_NETWORK           rime_driver
#define NETSTACK_CONF_MAC               nullmac_driver
#define NETSTACK_CONF_RDC               nullrdc_driver
#define NETSTACK_CONF_FRAMER            framer_802154
#define NETSTACK_CONF_RADIO             rf231_driver

#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE      8

#define CHANNEL_802_15_4                26
#define RF231_CONF_AUTOACK              1
#define RF231_CONF_AUTORETRIES          3

#endif
/*
 * UIP SECTION
 */


#ifdef WITH_UIP6

#define UIP_CONF_LL_802154              1
#define UIP_CONF_LLH_LEN                0

#define UIP_CONF_MAX_CONNECTIONS 2
#define UIP_CONF_MAX_LISTENPORTS 2
#define UIP_CONF_UDP_CONNS       2

#define UIP_CONF_IP_FORWARD      0
#define UIP_CONF_FWCACHE_SIZE    0

#define DUIP_CONF_IPV6_RPL		1
#define UIP_CONF_IPV6                   1
#define UIP_CONF_IPV6_CHECKS            1
#define UIP_CONF_IPV6_QUEUE_PKT  		1
#define UIP_CONF_IPV6_REASSEMBLY        0
#define UIP_CONF_NETIF_MAX_ADDRESSES    3
#define UIP_CONF_ND6_MAX_PREFIXES       3
#define UIP_CONF_ND6_MAX_NEIGHBORS      4
#define UIP_CONF_ND6_MAX_DEFROUTERS     2

#define SICSLOWPAN_CONF_COMPRESSION_IPV6        0
#define SICSLOWPAN_CONF_COMPRESSION_HC1         1
#define SICSLOWPAN_CONF_COMPRESSION_HC01        2
#define SICSLOWPAN_CONF_COMPRESSION             SICSLOWPAN_COMPRESSION_HC06
#ifndef SICSLOWPAN_CONF_FRAG
#define SICSLOWPAN_CONF_FRAG                    1
#define SICSLOWPAN_CONF_MAXAGE                  8
#endif /* SICSLOWPAN_CONF_FRAG */
#define SICSLOWPAN_CONF_CONVENTIONAL_MAC        1
#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS       2

#define UIP_CONF_ICMP6                  1


#define UIP_CONF_LOGGING                1
#else /* WITH_UIP6 */


/* uIP configuration */
#define UIP_CONF_LLH_LEN                0
#define UIP_CONF_BROADCAST              1
#define UIP_CONF_LOGGING                1
#define UIP_CONF_BUFFER_SIZE            1500
#define UIP_CONF_ROUTER                 1
#define UIP_CONF_TCP_FORWARD            1
#define UIP_CONF_BROADCAST              1

#endif

/*********************/

/* Prefix for relocation sections in ELF files */
#define REL_SECT_PREFIX ".rel"

#define CC_BYTE_ALIGNED __attribute__ ((packed, aligned(1)))

//#define RAND_MAX 0x7fff
#endif /* __CONTIKI_CONF_H__CDBB4VIH3I__ */
