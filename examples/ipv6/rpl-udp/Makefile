all: udp-client udp-server udp-audioTransmit udp-audioReceive transmit-audio-adpcm receive-audio-adpcm
APPS=servreg-hack
CONTIKI=../../..

UIP_CONF_IPV6=1

#CFLAGS += -DSEC_NODE_ENABLED=1

CFLAGS+= -DUIP_CONF_IPV6_RPL

SMALL=1

CFLAGS += -DPROJECT_CONF_H=\"transmit-audio-conf.h\"

ifdef WITH_COMPOWER
APPS+=powertrace
CFLAGS+= -DCONTIKIMAC_CONF_COMPOWER=1 -DWITH_COMPOWER=1 -DQUEUEBUF_CONF_NUM=4
endif

ifdef SERVER_REPLY
CFLAGS+=-DSERVER_REPLY=$(SERVER_REPLY)
endif
ifdef PERIOD
CFLAGS+=-DPERIOD=$(PERIOD)
endif

PROJECT_SOURCEFILES += ADPCM_codec.c

include $(CONTIKI)/Makefile.include
