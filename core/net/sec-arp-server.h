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
#define HELLO_REPLY_PACKETSIZE 80

#define HELLO_PACKET 		'H'
#define HELLO_PACKET_REPLY 	'X'
#define HELLO_ACK	 	2
#define SEC_ARP_REQUEST	1
#define SEC_ARP_REPLY	2

void forward_hello_packet(uint8_t *data, uint16_t data_len, const uint8_t *mac_address);

#endif /* SEC_ARP_SERVER_H_ */
