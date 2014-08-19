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
#include "dev/slip.h"
#include "dev/cc2420.h"
#include "dev/xmem.h"
#include "dev/leds.h"
#include "contiki-conf.h"
#include "dev/watchdog.h"	/* include to soft restart µP */
#include "platform-conf.h"	/* include for xmem address */

#if ENABLE_CBC_LINK_SECURITY & SEC_CLIENT

#define DEBUG 0
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

static short parse_hello_reply(uint8_t *buf);
static void create_hello(void);
static void  init_security_data(uint8_t *buf);

/*-----------------------------------------------------------------------------------*/
/**
 * Sec_arp_init
 *
 *
 */
/*-----------------------------------------------------------------------------------*/
static void
slip_input_callback(void) {
	PRINTF("It works!\n");
	parse_hello_reply(&uip_buf[0]);
}


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
		/* Init slip connection */
		slip_arch_init(BAUD2UBR(115200));
		PRINTF("sec-arp: Start slip process\n");
		process_start(&slip_process, NULL);
		slip_set_input_callback(slip_input_callback);

		/* Send hello packet */
		create_hello();

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
 * format: | message_type(1) | operation(1) | device id(16) | device MAC (8) |
 */
/*-----------------------------------------------------------------------------------*/
static void
create_hello(void)
{
	hello_packet_t packet;
	uint8_t state, i;
	uint8_t buf[HELLO_PACKETSIZE];

	/* Construct packet */
	packet.type = HELLO_PACKET;
	packet.operation = SEC_ARP_REQUEST;

	buf[0] = packet.type;
	buf[1] = packet.operation;

	/* Get device-id */
	for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
		state = uip_ds6_if.addr_list[i].state;
		if(uip_ds6_if.addr_list[i].isused && state == ADDR_PREFERRED) {
			memcpy(&buf[2], &uip_ds6_if.addr_list[i].ipaddr.u8[0], 16);
		}
	}

	/* Get device MAC */
	memcpy(&buf[18], &rimeaddr_node_addr.u8[0], 8);

	/* Send buf over slip */
	slip_write(buf, HELLO_PACKETSIZE);

	PRINTF("sec-arp: create\n");
}

/*-----------------------------------------------------------------------------------*/
/**
 *	Parse the bootstrap packet from server
 *
 * format: | message_type(1) | operation(1) | link_nonce_cntr(1) | network key(16) | edge-router id(16) | sensor key(16) |
 */
/*-----------------------------------------------------------------------------------*/
static short
parse_hello_reply(uint8_t *buf)
{
	if(buf[0] != HELLO_ACK) {
		return 0;
	}

	if(buf[1] != SEC_ARP_REPLY) {
		return 0;
	}

#if DEBUG
	uint8_t i;
	PRINTF("sec-arp: buf ");
	for(i=0; i<(KEY_SIZE*3); i++) PRINTF("%02x ", buf[3+i]);
	PRINTF("\n");
#endif

	/* Write security data to Flash */
	xmem_erase(XMEM_ERASE_UNIT_SIZE, MAC_SECURITY_DATA);
	xmem_pwrite(&buf[3], (KEY_SIZE*3), MAC_SECURITY_DATA);

	PRINTF("sec-arp: parse OK\n");

	watchdog_reboot();

	return 1;
}

/*-----------------------------------------------------------------------------------*/
/**
 * Init security data
 */
/*-----------------------------------------------------------------------------------*/
static void
init_security_data(uint8_t *buf)
{
	PRINTF("sec-arp: setting security data\n");
	/* write network key to cc2420 reg */
	CC2420_WRITE_RAM_REV(&buf[0], CC2420RAM_KEY0, KEY_SIZE);

	/* Set sensor key and security app data */
	devices[0].msg_cntr = 0;
	memcpy(&devices[CENTRAL_ENTITY_INDEX].remote_device_id.u8[0], &buf[APPLAYER_OFFSET], 16);
	memcpy(&devices[SERVER_INDEX].remote_device_id.u8[0], &buf[APPLAYER_OFFSET], 16);
	memcpy(&devices[CENTRAL_ENTITY_INDEX].session_key[0], &buf[APPLAYER_OFFSET+16], KEY_SIZE);
}
#endif
