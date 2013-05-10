all: er-example-server

CONTIKI=../..
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

# variable for root Makefile.include
WITH_UIP6=1
# for some platforms
UIP_CONF_IPV6=1
UIP_CONF_RPL=1

# REST framework, requires WITH_COAP
${info INFO: compiling with CoAP-08}
CFLAGS += -DWITH_COAP=7
CFLAGS += -DREST=coap_rest_implementation
CFLAGS += -DUIP_CONF_TCP=0
APPS += er-coap-07
APPS += erbium

ADDR = 0x22
CFLAGS += -DMAC_ADDRESS_LAST_BYTE=$(ADDR)

include $(CONTIKI)/Makefile.include

