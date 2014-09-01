/*
 * measure-cc2420-r.c
 *
 *  Created on: Jun 10, 2014
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
#include "net/mac/nullmac.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define SEND_INTERVAL		(CLOCK_SECOND)

uint16_t packet_count; /* defined in nullmac.h */

/*---------------------------------------------------------------------------*/
PROCESS(measure_process, "measure_process for cc2420");
AUTOSTART_PROCESSES(&measure_process);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(measure_process, ev, data)
{
	static struct etimer periodic;

	PROCESS_BEGIN();

	PRINTF("Starting measure process read\n");

	etimer_set(&periodic, SEND_INTERVAL);
	while(1) {
		PROCESS_YIELD();

		if(etimer_expired(&periodic)) {
		      etimer_reset(&periodic);

		      PRINTF("Count: %d", packet_count);
		      packet_count = 0;

		}
	}

	PROCESS_END();
}
