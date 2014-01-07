/*
 * sec-arp-server.c
 *
 *  Created on: Sep 24, 2013
 *      Author: crea
 */

#include "net/sec-arp-server.h"
#include "net/uip-ds6.h"
#include "dev/slip.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif

/*-----------------------------------------------------------------------------------*/
/**
 * Parse hello request packet
 *
 * format: | message_type(1) | operation(1) | device id(16)
 */
/*-----------------------------------------------------------------------------------*/
void
parse_hello_req(uint8_t *buf)
{
	PRINTF("sec-arp: parse\n");

	if(buf[0] != HELLO_PACKET) {
		return;
	}

	if(buf[1] != SEC_ARP_REQUEST) {
		return;
	}

	/* Extract src MAC-address */

	/* If we have a valid request packet send over slip (to check database) */
	slip_write(buf, HELLO_REQ_PACKETSIZE);
}
