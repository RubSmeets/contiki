/*
 * sec-arp-server.h
 *
 *  Created on: Sep 24, 2013
 *      Author: crea
 */

#ifndef SEC_ARP_SERVER_H_
#define SEC_ARP_SERVER_H_

#include <string.h>

/* Defines the reply packet length */
#define HELLO_REQ_PACKETSIZE 18

#define HELLO_PACKET 	1
#define HELLO_ACK	 	2
#define SEC_ARP_REQUEST	1
#define SEC_ARP_REPLY	2

void parse_hello_req(uint8_t *buf);

#endif /* SEC_ARP_SERVER_H_ */
