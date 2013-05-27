CONTIKI_PROJECT = cetic-6lbr

# Default mode selection

CETIC_6LBR_SMARTBRIDGE=0
CETIC_6LBR_TRANSPARENTBRIDGE=0
CETIC_6LBR_ROUTER=1

WITH_WEBSERVER_LITE=0
WITH_WEBSERVER=1
# End of user configurable section

all: $(CONTIKI_PROJECT)

UIP_CONF_IPV6=1

CONTIKI = ../..

PROJECTDIRS += 6lbr platform/stm32_node apps/webserver apps/udp-server apps/node-info

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

CFLAGS +=-DCETIC_6LBR_SMARTBRIDGE=$(CETIC_6LBR_SMARTBRIDGE) -DCETIC_6LBR_TRANSPARENTBRIDGE=$(CETIC_6LBR_TRANSPARENTBRIDGE) -DCETIC_6LBR_ROUTER=$(CETIC_6LBR_ROUTER)

ifneq ($(WITH_WEBSERVER_LITE),0)
WITH_WEBSERVER=0
endif
#.PHONY : force
ifneq ($(WITH_WEBSERVER),0)
CFLAGS += -DWEBSERVER=1
PROJECT_SOURCEFILES += webserver.c httpd-simple.c 
#PROJECT_SOURCEFILES += $(WEBCSOURCE)httpd-fs.c
#FORCE=force
#WEBDIR=$(CONTIKI)/examples/6lbr-router/apps/webserver/www/
#WEBCSOURCE=$(CONTIKI)/examples/6lbr-router/apps/webserver/
#$(WEBCSOURCE)httpd-fs.c: $(WEBCSOURCE)httpd-fsdata.c
#$(WEBCSOURCE)httpd-fsdata.c : $(FORCE) $(WEBDIR)/*.*
#	@echo Generating initial web content for full flash coffee file system
#	$(CONTIKI)/tools/makefsdata -C -A PROGMEM -c -f 20 -d $(WEBDIR) -o $(WEBCSOURCE)httpd-fsdata.c
endif

ifneq ($(WITH_WEBSERVER_LITE),0)
CFLAGS += -DWEBSERVER=1
PROJECT_SOURCEFILES += webserver-lite.c httpd-simple.c 
endif

WITH_UDPSERVER=0
ifneq ($(WITH_UDPSERVER),0)
CFLAGS += -DUDPSERVER=1
PROJECT_SOURCEFILES += udp-server.c 
endif

WITH_NODE_INFO=1
ifneq ($(WITH_NODE_INFO),0)
CFLAGS += -DCETIC_NODE_INFO=1
PROJECT_SOURCEFILES += node-info.c 
endif

PROJECT_SOURCEFILES += rio.c packet-filter.c sicslow-ethernet.c nvm-config.c node-info.c

PROJECT_SOURCEFILES += stm32-init.c stm32-nvm.c enc28j60.c enc28j60-drv.c

VERSION=$(shell echo $(word 3,$(shell awk '/CETIC_6LBR_VERSION/' project-conf.h)))

include $(CONTIKI)/Makefile.include
