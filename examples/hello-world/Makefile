CONTIKI_PROJECT = hello-world hello-world2 measure-cc2420 measure-cc2420-r measure-audio measure-audio-adpcm adpcm_test measure-linklayer
all: $(CONTIKI_PROJECT)

UIP_CONF_IPV6=1

#CFLAGS += -DPROJECT_CONF_H=\"measure-cc2420-conf.h\"
CFLAGS += -DPROJECT_CONF_H=\"measure-linklayer-conf.h\"

SMALL=1

CONTIKI = ../..

PROJECT_SOURCEFILES += ADPCM_codec.c nullnetwork.c

include $(CONTIKI)/Makefile.include
