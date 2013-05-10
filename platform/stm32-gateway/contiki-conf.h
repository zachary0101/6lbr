#ifndef __CONTIKI_CONF_H__1
#define __CONTIKI_CONF_H__1
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

#define CLOCK_CONF_SECOND 100
#define CCIF
#define CLIF
#define AUTOSTART_ENABLE	1

/* start of conitki config. */
#define PLATFORM_HAS_LEDS 1
#define PLATFORM_HAS_BUTTON 1

/* Core rtimer.h defaults to 16 bit timer unless RTIMER_CLOCK_LT is defined */
//typedef unsigned long rtimer_clock_t;
//#define RTIMER_CLOCK_LT(a,b)     ((signed long)((a)-(b)) < 0)

#define WITH_UIP 1
#define WITH_ASCII 1

/* uIP configuration */
#define uip_ipaddr_copy(dest, src) memcpy(dest, src, sizeof(*dest))

#define RF231_CONF_RADIOALWAYSON 0

#if WITH_UIP6

/* Network setup for IPv6 */
#define NETSTACK_CONF_NETWORK           sicslowpan_driver
#define NETSTACK_CONF_MAC               nullmac_driver
#define NETSTACK_CONF_RDC               nullrdc_driver
#define NETSTACK_CONF_RADIO             rf231_driver
#define NETSTACK_CONF_FRAMER            framer_802154

#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE      8
#define RIME_CONF_NO_POLITE_ANNOUCEMENTS 0
#define CXMAC_CONF_ANNOUNCEMENTS         0
#define XMAC_CONF_ANNOUNCEMENTS          0



#define CHANNEL_802_15_4                26
#else /* WITH_UIP6 */

/* Network setup for non-IPv6 (rime). */
#define RIMEADDR_CONF_SIZE              2

#define NETSTACK_CONF_NETWORK           rime_driver
#define NETSTACK_CONF_MAC               csma_driver
#define NETSTACK_CONF_RDC               sicslowmac_driver
#define NETSTACK_CONF_RADIO             rf231_driver
#define NETSTACK_CONF_FRAMER            framer_802154


#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE      8
#define COLLECT_CONF_ANNOUNCEMENTS       1
#define RIME_CONF_NO_POLITE_ANNOUCEMENTS 0
#define CXMAC_CONF_ANNOUNCEMENTS         0
#define XMAC_CONF_ANNOUNCEMENTS          0
#define CONTIKIMAC_CONF_ANNOUNCEMENTS    0

#define CONTIKIMAC_CONF_COMPOWER         0
#define XMAC_CONF_COMPOWER               0
#define CXMAC_CONF_COMPOWER              0

#define COLLECT_NEIGHBOR_CONF_MAX_NEIGHBORS      32
#define CHANNEL_802_15_4                26
#define RF231_CONF_AUTOACK              1
#define RF231_CONF_AUTORETRIES          3

#endif/* WITH_UIP6 */


#define QUEUEBUF_CONF_NUM          16

#define PACKETBUF_CONF_ATTRS_INLINE 1
#ifndef RF_CHANNEL
#define RF_CHANNEL                      26
#endif/* RF_CHANNEL */

#define CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT 0

#define IEEE802154_CONF_PANID       0xABCD

#define PROFILE_CONF_ON 0
#define ENERGEST_CONF_ON 0

#define AODV_COMPLIANCE
#define AODV_NUM_RT_ENTRIES 32

#define PROCESS_CONF_NUMEVENTS 8
#define PROCESS_CONF_STATS 1
/*
 * UIP SECTION
 */


#ifdef WITH_UIP6

#define RIMEADDR_CONF_SIZE              8

#define UIP_CONF_LL_802154              1

#define UIP_CONF_ROUTER                 1
#define UIP_CONF_IPV6_RPL               1

/* Max number of items in the neighbor cache */
#define UIP_CONF_DS6_NBR_NBU            30
/* Max number of items in the routing table cache */
#define UIP_CONF_DS6_ROUTE_NBU          30

#define UIP_CONF_ND6_SEND_RA            0
#define UIP_CONF_ND6_REACHABLE_TIME     600000
#define UIP_CONF_ND6_RETRANS_TIMER      10000


#define UIP_CONF_IPV6                   1
#define UIP_CONF_IPV6_QUEUE_PKT         0
#define UIP_CONF_IPV6_CHECKS            1
#define UIP_CONF_IPV6_REASSEMBLY        0
#define UIP_CONF_NETIF_MAX_ADDRESSES    3
#define UIP_CONF_ND6_MAX_PREFIXES       3
#define UIP_CONF_ND6_MAX_NEIGHBORS      4
#define UIP_CONF_ND6_MAX_DEFROUTERS     2
#define UIP_CONF_IP_FORWARD             0
#define UIP_CONF_BUFFER_SIZE            1300
#define SICSLOWPAN_CONF_FRAG            1
#define SICSLOWPAN_CONF_MAXAGE          8

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

#else /* WITH_UIP6 */
#define UIP_CONF_IP_FORWARD      1
#define UIP_CONF_BUFFER_SIZE     1300
#endif /* WITH_UIP6 */


#define UIP_CONF_ICMP_DEST_UNREACH 1

#define UIP_CONF_DHCP_LIGHT
#define UIP_CONF_LLH_LEN         0
#define UIP_CONF_RECEIVE_WINDOW  48
#define UIP_CONF_TCP_MSS         48
#define UIP_CONF_MAX_CONNECTIONS 4
#define UIP_CONF_MAX_LISTENPORTS 8
#define UIP_CONF_UDP_CONNS       12
#define UIP_CONF_FWCACHE_SIZE    30
#define UIP_CONF_BROADCAST       1
#define UIP_ARCH_IPCHKSUM        1
#define UIP_CONF_UDP             1
#define UIP_CONF_UDP_CHECKSUMS   1
#define UIP_CONF_PINGADDRCONF    0
#define UIP_CONF_LOGGING         0

#define UIP_CONF_TCP_SPLIT       0

/* include the project config */
/* PROJECT_CONF_H might be defined in the project Makefile */
#ifdef PROJECT_CONF_H
#include PROJECT_CONF_H
#endif /* PROJECT_CONF_H */

/*********************/

/* Prefix for relocation sections in ELF files */
#define REL_SECT_PREFIX ".rel"

#define CC_BYTE_ALIGNED __attribute__ ((packed, aligned(1)))

//#define RAND_MAX 0x7fff
#endif /* __CONTIKI_CONF_H__CDBB4VIH3I__ */
