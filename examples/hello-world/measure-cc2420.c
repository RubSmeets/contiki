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

#define SEND_INTERVAL		(CLOCK_SECOND)

uint32_t counter;
uint8_t toggle;
uint16_t cnt;
uint8_t send_cnt, process_cnt;

/*---------------------------------------------------------------------------*/
PROCESS(udp_stream_process_1, "UDP stream process 1");
/*---------------------------------------------------------------------------*/
PROCESS(measure_process, "measure_process for cc2420");
AUTOSTART_PROCESSES(&measure_process, &udp_stream_process_1);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(measure_process, ev, data)
{
	static struct etimer periodic;

	PROCESS_BEGIN();

	PRINTF("Starting measure process\n");

	SENSORS_ACTIVATE(phidgets);
	//dac_init(Z1_DAC_0);

	toggle = 0;
	cnt = 0;
	send_cnt = 0;
	process_cnt = 0;

	PRINTF("Interrupt enable: %04x\n", ADC12IE);
	PRINTF("Control 1: %04x\n", ADC12CTL0);
	PRINTF("Control 2: %04x\n", ADC12CTL1);
	PRINTF("Phidget 5V 1:%d\n", phidgets.value(PHIDGET3V_2));
	PRINTF("FLAG register: %04x\n", ADC12IFG);
	PRINTF("Control dac: %04x\n", DAC12_0CTL);

	etimer_set(&periodic, SEND_INTERVAL);
	while(1) {
		PROCESS_YIELD();

		if(etimer_expired(&periodic)) {
		      etimer_reset(&periodic);
		      PRINTF("C: %ld\n", counter);
		      PRINTF("P: %d\n", process_cnt);
		      process_cnt = 0;
		      //dac_setValue(phidgets.value(PHIDGET3V_2), Z1_DAC_0);
		}
		counter = 0;

	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_stream_process_1, ev, data)
{

	PROCESS_BEGIN();
	PRINTF("UDP stream process 1 started\n");

	char buf[100];

	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		PRINTF("test ");
		sprintf(buf, "try this connection to get the full potential of the transceiver throughput with a counter for a  %02d", cnt);
		cnt++;
		process_cnt++;

		NETSTACK_RADIO.send(buf, 100);
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
HWCONF_ADC_IRQ(PHIDGET, ADC7);
ISR(ADC12, adc_service_routine)
{
	if(send_cnt == 200) {
		process_poll(&udp_stream_process_1);
		send_cnt = 0;
	} else {
		send_cnt++;
	}

	P6OUT = ((toggle & 0x01)<<2);
	counter++;
	toggle++;

	PHIDGET_CLEAR_IRQ_FLAG(); /* Clear Interrupt flag */
}
/*---------------------------------------------------------------------------*/
