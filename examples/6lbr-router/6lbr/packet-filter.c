#include "contiki-net.h"
#include "net/uip-neighbor.h"
#include "net/uip-ds6.h"
#include "net/uip-nd6.h"
#include "string.h"
#include "sicslow-ethernet.h"

#include "cetic-6lbr.h"
#include "nvm-config.h"

#include "eth-drv.h"

extern const rimeaddr_t rimeaddr_null;

#define DEBUG 0//DEBUG_NONE //DEBUG_PRINT
#include "net/uip-debug.h"

static int eth_output(uip_lladdr_t * src, uip_lladdr_t * dest);

/*---------------------------------------------------------------------------*/


static outputfunc_t wireless_outputfunc;
static inputfunc_t tcpip_inputfunc;

#define BUF ((struct uip_eth_hdr *)&ll_header[0])

#define UIP_IP_BUF ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_ICMP_BUF                      ((struct uip_icmp_hdr *)&uip_buf[uip_l2_l3_hdr_len])
#define UIP_ND6_NS_BUF            ((uip_nd6_ns *)&uip_buf[uip_l2_l3_icmp_hdr_len])
#define UIP_ND6_NA_BUF            ((uip_nd6_na *)&uip_buf[uip_l2_l3_icmp_hdr_len])

#define IS_EUI48_ADDR(a) ((a) != NULL && (a)->addr[3] == CETIC_6LBR_ETH_EXT_A && (a)->addr[4] ==CETIC_6LBR_ETH_EXT_B )
#define IS_BROADCAST_ADDR(a) ((a)==NULL || rimeaddr_cmp((rimeaddr_t *)(a), &rimeaddr_null) != 0)

/*---------------------------------------------------------------------------*/

static void
send_to_uip(void)
{
  if(tcpip_inputfunc != NULL) {
    tcpip_inputfunc();		//tcpip_inputfunc
  } else {
    PRINTF("No input function set\r\n");
  }
}
/*---------------------------------------------------------------------------*/

static void
wireless_input(void)
{
  int processFrame = 0;
  int forwardFrame = 0;

 // PRINTF("wireless_input\r\n");
  //Destination filtering
  //---------------------
  if(IS_BROADCAST_ADDR(packetbuf_addr(PACKETBUF_ADDR_RECEIVER))) {      //Broadcast
//    PRINTF("wireless_input : broadcast\r\n");
    forwardFrame = 1;
    processFrame = 1;
  } else {                      //unicast
    PRINTF("wireless_input: dest: ");
    PRINTLLADDR((uip_lladdr_t *) packetbuf_addr(PACKETBUF_ADDR_RECEIVER));
    PRINTF("\r\n");
    if(rimeaddr_cmp
       (packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
        (rimeaddr_t *) & wsn_mac_addr) != 0) {
      processFrame = 1;         //For us
    } else {                    //For another host
    }
  }
  //Handle packet
  //-------------
  if(processFrame) {
    //PRINTF("wireless_input: processing frame\r\n");
    send_to_uip();
  } else {
    //Drop packet
    uip_len = 0;
  }
}

uint8_t
wireless_output(uip_lladdr_t * src, uip_lladdr_t * dest)
{
  int ret;

  //Packet filtering
  //----------------
  //Filter out Ethernet segment traffic
  if(IS_EUI48_ADDR(dest)) {
    return 0;
  }
  //Filter out RA/RS towards WSN
  if(UIP_IP_BUF->proto == UIP_PROTO_ICMP6 &&
     (UIP_ICMP_BUF->type == ICMP6_RS || UIP_ICMP_BUF->type == ICMP6_RA) &&
     (nvm_data.mode & CETIC_MODE_FILTER_NDP_MASK) != 0) {
    return 0;
  }
  //Packet sending
  //--------------
  if(wireless_outputfunc != NULL) {

    PRINTF("wireless_output: sending packet\r\n");
    ret = wireless_outputfunc(dest);
  } else {
    ret = 0;
  }
  return ret;
}

/*---------------------------------------------------------------------------*/

void
eth_input(void)
{
  uip_lladdr_t destAddr;
  int processFrame = 0;
  int forwardFrame = 0;

  //Packet type filtering
  //---------------------
  //Keep only IPv6 traffic
  if(BUF->type != UIP_HTONS(UIP_ETHTYPE_IPV6)) {
    PRINTF("eth_input: Dropping packet type=0x%04x\r\n", uip_ntohs(BUF->type));
    uip_len = 0;
    return;
  }
  //Packet source Filtering
  //-----------------------
  /* IPv6 uses 33-33-xx-xx-xx-xx prefix for multicast ND stuff */
  if((BUF->dest.addr[0] == 0x33) && (BUF->dest.addr[1] == 0x33)) {
    forwardFrame = 1;
    processFrame = 1;
    rimeaddr_copy((rimeaddr_t *) & destAddr, &rimeaddr_null);
  } else if((BUF->dest.addr[0] == 0xFF)
            && (BUF->dest.addr[1] == 0xFF)
            && (BUF->dest.addr[2] == 0xFF)
            && (BUF->dest.addr[3] == 0xFF)
            && (BUF->dest.addr[4] == 0xFF)
            && (BUF->dest.addr[5] == 0xFF)) {
    /* IPv6 does not use broadcast addresses, hence this should not happen */
    PRINTF("eth_input: Dropping broadcast packet\n\r");
    uip_len = 0;
    return;
  } else {
    /* Complex Address Translation */
    if(mac_createSicslowpanLongAddr(&(BUF->dest.addr[0]), &destAddr) == 0) {
      PRINTF("eth_input: Address translation failed\n\r");
      uip_len = 0;
      return;
    }
  }

  //Packet content rewriting
  //------------------------
  //Some IP packets have link layer in them, need to change them around!
  uint8_t transReturn = mac_translateIPLinkLayer(ll_802154_type);

  if(transReturn != 0) {
    PRINTF("eth_input: IPTranslation returns %d\n\r", transReturn);
  }
  //Destination filtering
  //---------------------
  if(memcmp((uint8_t *) & eth_mac_addr, BUF->dest.addr, 6) == 0) {
    processFrame = 1;
  } else {
  }

  //Handle packet
  //-------------
  if(processFrame) {
   // PRINTF("eth_input: Processing frame\r\n");
    send_to_uip();
  } else {
    //Drop packet
    uip_len = 0;
  }
}

static int
eth_output(uip_lladdr_t * src, uip_lladdr_t * dest)
{
  PRINTF("eth_output: ");
  if(IS_BROADCAST_ADDR(dest)) {
    PRINTF("Broadcast");
  } else {
    PRINTLLADDR(dest);
  }
  PRINTF("\r\n");
  //Packet filtering
  //----------------
  if(uip_len == 0) {
    PRINTF("eth_output: uip_len = 0\r\n");
    return 0;
  }
  //Filter out traffic not targeted to Ethernet segment
  if(!IS_EUI48_ADDR(dest) && !IS_BROADCAST_ADDR(dest)) {
    PRINTF("eth_output: Not ethernet destination : ");
    PRINTLLADDR(dest);
    PRINTF("\r\n");
    return 0;
  }
  //Filter out RPL (broadcast) traffic
  if(UIP_IP_BUF->proto == UIP_PROTO_ICMP6 &&
     UIP_ICMP_BUF->type == ICMP6_RPL &&
     (nvm_data.mode & CETIC_MODE_FILTER_RPL_MASK) != 0) {
    PRINTF("eth_output: Filtering RPL traffic\r\n");
    return 0;
  }
  //IP packet alteration
  //--------------------

  //Modify source address
  if((nvm_data.mode & CETIC_MODE_REWRITE_ADDR_MASK) != 0
     && uip_is_addr_link_local(&UIP_IP_BUF->srcipaddr)
     && uip_ds6_is_my_addr(&UIP_IP_BUF->srcipaddr)) {
    PRINTF("eth_output: Update src address\r\n");
    uip_ipaddr_copy(&UIP_IP_BUF->srcipaddr, &eth_ip_local_addr);
  }
  //Some IP packets have link layer in them, need to change them around!
  mac_translateIPLinkLayer(ll_8023_type);

  //IP header alteration
  //--------------------
  //Remove Hop-by-hop extension header
  if(uip_ext_len > 0) {
    extern void remove_ext_hdr(void);
    uint8_t proto = *((uint8_t *) UIP_IP_BUF + 40);

    remove_ext_hdr();
    UIP_IP_BUF->proto = proto;
  }
  //Create packet header
  //--------------------
  //Packet type
  BUF->type = uip_htons(UIP_ETHTYPE_IPV6);

  //Destination address
  if(IS_BROADCAST_ADDR(dest)) {
    BUF->dest.addr[0] = 0x33;
    BUF->dest.addr[1] = 0x33;
    BUF->dest.addr[2] = UIP_IP_BUF->destipaddr.u8[12];
    BUF->dest.addr[3] = UIP_IP_BUF->destipaddr.u8[13];
    BUF->dest.addr[4] = UIP_IP_BUF->destipaddr.u8[14];
    BUF->dest.addr[5] = UIP_IP_BUF->destipaddr.u8[15];
  } else {
    mac_createEthernetAddr(BUF->dest.addr, dest);
  }
  //Source address
  memcpy(BUF->src.addr, eth_mac_addr, 6);

  //Sending packet
  //--------------
  PRINTF("eth_output: Sending packet to ethernet\r\n");
  eth_drv_send();

  return 1;
}

/*---------------------------------------------------------------------------*/

static uint8_t
bridge_output(uip_lladdr_t * a)
{
  if(uip_len == 0) {
    printf("ERROR: Trying to send empty packet\r\n");
    return 0;
  }
  PRINTF("bridge_output: Sending packet to ");
  if(!IS_BROADCAST_ADDR(a)) {
    PRINTLLADDR(a);
  } else {
    PRINTF("Broadcast");
  }
  PRINTF("\r\n");
  if(IS_BROADCAST_ADDR(a)) {
    //Obviously we can not guess the target segment for a multicast packet
    //So we have to check the packet source prefix (and match it on the Ethernet segment prefix)
    //or, in case of link-local packet, check packet type and/or packet data
    if((UIP_IP_BUF->proto == UIP_PROTO_ICMP6
        && UIP_ICMP_BUF->type == ICMP6_RA)
       || (UIP_IP_BUF->proto == UIP_PROTO_ICMP6
           && UIP_ICMP_BUF->type == ICMP6_NS
           && uip_ipaddr_prefixcmp(&eth_net_prefix,
                                   &UIP_ND6_NS_BUF->tgtipaddr, 64))
       || uip_ipaddr_prefixcmp(&eth_net_prefix, &UIP_IP_BUF->srcipaddr, 64)) {
      eth_output(NULL, a);
    } else {
      //ret = wireless_output(NULL, a);
      wireless_output(NULL, a);
    }
  } else {
    if(IS_EUI48_ADDR(a)) {
      eth_output(NULL, a);
    } else {
      wireless_output(NULL, a);
    }
  }
  return 0;
}

/*---------------------------------------------------------------------------*/

void
packet_filter_init(void)
{
  wireless_outputfunc = tcpip_get_outputfunc();
  tcpip_set_outputfunc(bridge_output);

  tcpip_inputfunc = tcpip_get_inputfunc();

  tcpip_set_inputfunc(wireless_input);
  printf("packet_filter_init\r\n");
}
/*---------------------------------------------------------------------------*/
