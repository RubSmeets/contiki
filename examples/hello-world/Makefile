CONTIKI_PROJECT = hello-world hello-world2 measure-cc2420 measure-cc2420-r measure-audio
all: $(CONTIKI_PROJECT)

UIP_CONF_IPV6=1

CFLAGS += -DPROJECT_CONF_H=\"measure-cc2420-conf.h\"

SMALL=1

CONTIKI = ../..
include $(CONTIKI)/Makefile.include
