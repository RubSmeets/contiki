/*
 * sec-arp.c
 *
 *  Created on: Aug 5, 2013
 *      Author: crea
 */
#include "net/sec-arp-client.h"
#include "net/sec_data.h"
#include "net/rime/rimeaddr.h"
#include "net/uip-ds6.h"
#include "dev/cc2420.h"
#include "dev/xmem.h"
#include "contiki-conf.h"

#if ENABLE_CBC_LINK_SECURITY

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif

#define LINKLAYER_OFFSET	2
#define APPLAYER_OFFSET		16
#define KEY_SIZE			16

uint8_t hasKeys;
struct  device_sec_data devices[MAX_DEVICES];

static void  init_security_data(uint8_t *buf);

/*-----------------------------------------------------------------------------------*/
/**
 * Sec_arp_init
 *
 *
 */
/*-----------------------------------------------------------------------------------*/
void __attribute__((__far__))
sec_arp_init(void)
{
	uint8_t sum, i;
	uint8_t temp_buf[KEY_SIZE*3];

	/* Read security data from Flash mem */
	xmem_pread(temp_buf, (KEY_SIZE*3), MAC_SECURITY_DATA);

	/* Check if we have a network key */
	sum = 0;
	for(i=KEY_SIZE; i>0; i--) {sum |= temp_buf[i-1];}
	if(!(sum))	{
		hasKeys = 0;
	} else {
		PRINTF("sec-arp: Key OK\n");

		hasKeys = 1;
		/* Set security data */
		init_security_data(temp_buf);
	}
}

/*-----------------------------------------------------------------------------------*/
/**
 * Create Hello packet CODE
 *
 * format: | message_type(1) | device id(16) |
 */
/*-----------------------------------------------------------------------------------*/
void __attribute__((__far__))
create_hello(uint8_t *buf)
{
	uint8_t state, i;

	/* hello-packet header */
	buf[0] = HELLO_PACKET;

	/* Get device-id */
	for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
		state = uip_ds6_if.addr_list[i].state;
		if(uip_ds6_if.addr_list[i].isused && state == ADDR_PREFERRED) {
			memcpy(&buf[1], &uip_ds6_if.addr_list[i].ipaddr.u8[0], 16);
		}
	}

	PRINTF("sec-arp: create\n");
}

/*-----------------------------------------------------------------------------------*/
/**
 * Init security data
 */
/*-----------------------------------------------------------------------------------*/
static void
init_security_data(uint8_t *buf)
{
	/* write network key to cc2420 reg */
	CC2420_WRITE_RAM_REV(&buf[0], CC2420RAM_KEY0, KEY_SIZE);

	/* Set sensor key and security app data */
	devices[0].msg_cntr = 0;
	memcpy(&devices[0].remote_device_id.u8[0], &buf[APPLAYER_OFFSET], 16);
	memcpy(&devices[0].session_key[0], &buf[APPLAYER_OFFSET+16], KEY_SIZE);
}
#endif
