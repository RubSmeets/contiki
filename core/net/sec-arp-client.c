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
#include "dev/leds.h"
#include "contiki-conf.h"
#include "dev/watchdog.h"	/* include to soft restart µP */
#include "platform-conf.h"	/* include for xmem address */

#if ENABLE_CBC_LINK_SECURITY & SEC_CLIENT

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
		leds_on(LEDS_BLUE);
		PRINTF("No keys\n");
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

	PRINTF("sec-arp: create hello\n");
}

/*-----------------------------------------------------------------------------------*/
/**
 *	Parse the bootstrap packet from server
 *
 *	format: | encryption_nonce(3) | network key(16) | central_entity_id(16) | sensor key(16) | MIC(8)
 */
/*-----------------------------------------------------------------------------------*/
void __attribute__((__far__))
parse_hello_reply(uint8_t *data, uint16_t len)
{
	uint8_t state, i;
	uint8_t temp_data_len = len & 0xff;
	uint16_t msg_cntr = (uint16_t)data[1] << 8 | data[2];
	uint8_t address[16];

	/* Get own ip */
	for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
		state = uip_ds6_if.addr_list[i].state;
		if(uip_ds6_if.addr_list[i].isused && state == ADDR_PREFERRED) {
			memcpy(&address[0], &uip_ds6_if.addr_list[i].ipaddr.u8[0], 16);
		}
	}

	/* Set bootstrap key for decryption */
	CC2420_WRITE_RAM_REV(&devices[0].session_key[0], CC2420RAM_KEY1, KEY_SIZE);

	PRINTF("sec-arp: setup decryption - msg_cnt: %04x nonce_cnt: %02x ", msg_cntr, data[2]);
	PRINTF("address: "); for(i=0; i<16; i++) PRINTF("%02x",address[i]); PRINTF(" with key: ");
	for(i=0; i<16; i++) PRINTF("%02x", devices[0].session_key[i]); PRINTF("\n");

	PRINTF("sec-arp: encrypted data - data: "); for(i=0; i<temp_data_len; i++) PRINTF("%02x",data[i]); PRINTF("\n");

	/* Decrypt message */
	cc2420_decrypt_ccm(data, address, &msg_cntr, &data[0], &temp_data_len, NONCE_SIZE);

	PRINTF("sec-arp: dec_data "); for(i=0;i<temp_data_len;i++) PRINTF("%02x ", data[i]); PRINTF("\n");

	/* Check if authentication was successful */
	if(data[temp_data_len-1] == 0x00) {
		/* Write security data to Flash */
		xmem_erase(XMEM_ERASE_UNIT_SIZE, MAC_SECURITY_DATA);
		xmem_pwrite(&data[3], (SEC_KEY_SIZE*3), MAC_SECURITY_DATA);

		/* Reset nonce data in flash */
		xmem_erase(XMEM_ERASE_UNIT_SIZE, APP_NONCE_DATA);
		PRINTF("sec-arp: reboot()\n");
		watchdog_reboot();
	} else {
		PRINTF("sec-arp: auth. failed\n");
		return;
	}
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
