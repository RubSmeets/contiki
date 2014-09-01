/*
 * measure-cc2420.c
 *
 *  Created on: May 9, 2014
 *      Author: crea
 */
#include "contiki.h"
#include "dev/cc2420.h"
#include "dev/button-sensor.h"
#include "dev/z1-phidgets.h"
#include "dev/z1-dac.h"
#include "dev/hwconf.h"
#include "isr_compat.h"
#include "net/netstack.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define SEND_INTERVAL		(CLOCK_SECOND/128)
#define COUNT_INTERVAL		(CLOCK_SECOND)

static const char HELLO[] = "try this connection to get the full potential of the transceiver throughput with a counter for a  22";

uint8_t toggle;

/*---------------------------------------------------------------------------*/
PROCESS(udp_stream_process_1, "UDP stream process 1");
PROCESS(udp_stream_process_2, "UDP stream process 2");
PROCESS(udp_stream_process_3, "UDP stream process 3");
PROCESS(udp_stream_process_4, "UDP stream process 4");
PROCESS(udp_stream_process_5, "UDP stream process 5");
/*---------------------------------------------------------------------------*/
PROCESS(measure_process, "measure_process for cc2420");
AUTOSTART_PROCESSES(&measure_process, &udp_stream_process_1, &udp_stream_process_2, &udp_stream_process_3, &udp_stream_process_4, &udp_stream_process_5);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(measure_process, ev, data)
{
	static struct etimer periodic;

	PROCESS_BEGIN();

	PRINTF("Starting measure process\n");

	SENSORS_ACTIVATE(button_sensor);

	toggle = 0;

	etimer_set(&periodic, SEND_INTERVAL);

	while(1) {
		PROCESS_YIELD();

		if (ev == sensors_event && data == &button_sensor) {
			PRINTF("button\n");
	    } else if(etimer_expired(&periodic)) {
			etimer_reset(&periodic);

			//NETSTACK_RADIO.send("testeddsdfdedssddfddss", 22);
			process_poll(&udp_stream_process_1);
			process_poll(&udp_stream_process_2);
			process_poll(&udp_stream_process_3);
			process_poll(&udp_stream_process_4);
			process_poll(&udp_stream_process_5);
	    }

	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_stream_process_1, ev, data)
{

	PROCESS_BEGIN();
	PRINTF("UDP stream process 1 started\n");

	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		PRINTF("1 ");
		NETSTACK_RADIO.send(HELLO, 100);
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_stream_process_2, ev, data)
{

	PROCESS_BEGIN();
	PRINTF("UDP stream process 2 started\n");

	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		PRINTF("2 ");
		NETSTACK_RADIO.send(HELLO, 100);
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_stream_process_3, ev, data)
{

	PROCESS_BEGIN();
	PRINTF("UDP stream process 3 started\n");

	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		PRINTF("3 ");
		NETSTACK_RADIO.send(HELLO, 100);
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_stream_process_4, ev, data)
{

	PROCESS_BEGIN();
	PRINTF("UDP stream process 4 started\n");

	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		PRINTF("4 ");
		NETSTACK_RADIO.send(HELLO, 100);
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_stream_process_5, ev, data)
{

	PROCESS_BEGIN();
	PRINTF("UDP stream process 5 started\n");

	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		PRINTF("5 ");
		NETSTACK_RADIO.send(HELLO, 100);
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
