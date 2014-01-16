/*
 * sec-arp.h
 *
 *  Created on: Aug 5, 2013
 *      Author: crea
 */

#ifndef SEC_ARP_H_
#define SEC_ARP_H_

#include <string.h>

/* Defines the reply packet length */
#define HELLO_REPLY_PACKETSIZE 	51
#define HELLO_PACKETSIZE 		17

#define HELLO_PACKET 	'H'
#define HELLO_ACK	 	'A'
#define SEC_ARP_REQUEST	'Q'
#define SEC_ARP_REPLY	'R'

/* Global variable */
extern uint8_t hasKeys;

void __attribute__((__far__)) sec_arp_init(void);
void __attribute__((__far__)) create_hello(uint8_t *buf);

#endif /* SEC_ARP_H_ */
