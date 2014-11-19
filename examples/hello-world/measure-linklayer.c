/*
 * measure-cc2420.c
 *
 *  Created on: May 9, 2014
 *      Author: crea
 */
#include "contiki.h"
#include "dev/cc2420.h"
#include "dev/button-sensor.h"
#include "dev/hwconf.h"
#include "isr_compat.h"
#include "net/netstack.h"
#include "net/uip.h"
#include "net/uip-ds6.h"
#include "net/rime.h"

#include <string.h>

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define MAX_PAYLOAD			80

#define SEND_INTERVAL		(CLOCK_SECOND/128)
#define COUNT_INTERVAL		(CLOCK_SECOND)

static const char HELLO[] = "try this connection to get the full potential of the transceiver throughput with a counter for a  22";

static struct rime_sniffer *callback = NULL;
static rimeaddr_t dest;
static uint16_t payload_buf_aligned[MAX_PAYLOAD/2];
static uint8_t *payload_buf = (uint8_t *)payload_buf_aligned;
static uint16_t count;
static uint8_t allow_send;
static uint8_t send_amount;
static uint8_t * rime_ptr;

/*---------------------------------------------------------------------------*/
PROCESS(stream_process_1, "stream process 1");
PROCESS(stream_process_2, "stream process 2");
PROCESS(stream_process_3, "stream process 3");
PROCESS(stream_process_4, "stream process 4");
PROCESS(stream_process_5, "stream process 5");
/*---------------------------------------------------------------------------*/
PROCESS(measure_process, "measure_process for linklayer");
AUTOSTART_PROCESSES(&measure_process, &stream_process_1, &stream_process_2, &stream_process_3, &stream_process_4, &stream_process_5);
/*---------------------------------------------------------------------------*/

/*--------------------------------------------------------------------*/
/** \name Input/output functions common to all compression schemes
 * @{                                                                 */
/*--------------------------------------------------------------------*/
/**
 * Callback function for the MAC packet sent callback
 */
static void
packet_sent(void *ptr, int status, int transmissions)
{
  uip_ds6_link_neighbor_callback(status, transmissions);

  if(callback != NULL) {
    callback->output_callback(status);
  }
}
/*--------------------------------------------------------------------*/
/**
 * \brief This function is called by the 6lowpan code to send out a
 * packet.
 * \param dest the link layer destination address of the packet
 */
static void
send_packet(rimeaddr_t *dest) {

	packetbuf_clear();

	payload_buf[0] = (count >> 8) & 0xFF;
	payload_buf[1] = count & 0xFF;
	memcpy(rime_ptr, payload_buf, MAX_PAYLOAD);
	packetbuf_set_datalen(MAX_PAYLOAD);

	if(count < ((send_amount*5)+1)) {
		count++;
	} else {
		count = 1;
		allow_send = 0;
	}

	/* Set the link layer destination address for the packet as a
	* packetbuf attribute. The MAC layer can access the destination
	* address with the function packetbuf_addr(PACKETBUF_ADDR_RECEIVER).
	*/
	packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, dest);

	/* Provide a callback function to receive the result of
	 a packet transmission. */
	NETSTACK_MAC.send(&packet_sent, NULL);
}
/*--------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(measure_process, ev, data)
{
	static struct etimer periodic;
	static struct etimer second;

	PROCESS_BEGIN();

	PRINTF("Starting measure process for link-layer\n");

	SENSORS_ACTIVATE(button_sensor);

	/* reset rime buffer */
	packetbuf_clear();
	rime_ptr = (uint8_t *)packetbuf_dataptr();

	/* Set dummy payload buffer */
	memcpy(payload_buf, HELLO, MAX_PAYLOAD);
	count = 0x0001;

	/* Set MAC address of receiver */
	rimeaddr_copy(&dest, &rimeaddr_node_addr);
	dest.u8[7] = 0x06;

	/* Set amount of packets to be transmitted */
	send_amount = 0;
	allow_send = 1;

	etimer_set(&second, COUNT_INTERVAL);
	etimer_set(&periodic, SEND_INTERVAL);
	while(1) {
		PROCESS_YIELD();

		if (ev == sensors_event && data == &button_sensor) {
			send_amount += 5;
			PRINTF("Amount of packets to be send %d\n", (send_amount*5));
	    } else if(etimer_expired(&periodic)) {
			etimer_reset(&periodic);

			if((allow_send == 1) && (send_amount > 0)) {
				process_poll(&stream_process_1);
				process_poll(&stream_process_2);
				process_poll(&stream_process_3);
				process_poll(&stream_process_4);
//				process_poll(&stream_process_5);
			}
	    } else if(etimer_expired(&second)) {
			etimer_reset(&second);

	    	PRINTF("Expired!\n");
	    	allow_send = 1;
	    }

	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(stream_process_1, ev, data)
{

	PROCESS_BEGIN();
	PRINTF("stream process 1 started\n");

	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		send_packet(&dest);
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(stream_process_2, ev, data)
{

	PROCESS_BEGIN();
	PRINTF("stream process 2 started\n");

	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		send_packet(&dest);;
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(stream_process_3, ev, data)
{

	PROCESS_BEGIN();
	PRINTF("stream process 3 started\n");

	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		send_packet(&dest);
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(stream_process_4, ev, data)
{

	PROCESS_BEGIN();
	PRINTF("stream process 4 started\n");

	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		send_packet(&dest);
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(stream_process_5, ev, data)
{

	PROCESS_BEGIN();
	PRINTF("stream process 5 started\n");

	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		send_packet(&dest);
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
