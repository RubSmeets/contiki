/*
 * sec-arp.h
 *
 *  Created on: Aug 5, 2013
 *      Author: crea
 */

#ifndef SEC_ARP_H_
#define SEC_ARP_H_

#include <string.h>

typedef struct {
  uint8_t type;
  uint8_t operation;
} hello_packet_t;

/* Defines the reply packet length */
#define HELLO_REPLY_PACKETSIZE 51
#define HELLO_PACKETSIZE 26

#define HELLO_PACKET 	'H'
#define HELLO_ACK	 	'A'
#define SEC_ARP_REQUEST	'Q'
#define SEC_ARP_REPLY	'R'

void sec_arp_init(void);

#endif /* SEC_ARP_H_ */
