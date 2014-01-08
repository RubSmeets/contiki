/*
 * hello-world2.c
 *
 *  Created on: Jan 7, 2014
 *      Author: crea
 */
#include "contiki.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include "net/uip.h"
#include "net/uip-ds6.h"
#include "sys/clock.h"

#include <string.h>

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define MAX_DATA 5

//struct data {
//  uip_ipaddr_t  	remote_device_id;
//  uint16_t			msg_cntr;
//  uint8_t			nonce_cntr;
//  uint16_t 			remote_msg_cntr;
//  uint8_t 	 		remote_nonce_cntr;
//  uint8_t			key_freshness;
//  uint8_t			session_key[16];
//  unsigned long		time_last_activity;
//};

//static struct data *list_data;

void __attribute__((__far__)) incrCounter(uint8_t* pointer);
void __attribute__((__far__)) passValueFromNear(uint8_t cnt);
short __attribute__((__far__)) decrCounter(uint8_t* pointer);

static uint8_t pcounter[10];
static uint8_t pcounter2;
const uint8_t __attribute__((__far__)) values[100] =
{0,1,2,3,4,5,6,7,8,9,
 0,1,2,3,4,5,6,7,8,9,
 0,1,2,3,4,5,6,7,8,9,
 0,1,2,3,4,5,6,7,8,9,
 0,1,2,3,4,5,6,7,8,9,
 0,1,2,3,4,5,6,7,8,9,
 0,1,2,3,4,5,6,7,8,9,
 0,1,2,3,4,5,6,7,8,9,
 0,1,2,3,4,5,6,7,8,9,
 0,1,2,3,4,5,6,7,8,9
};

static uint8_t count=0;

//-----------------------------------------------------------------
PROCESS(far_near_process, "Far and near process");
AUTOSTART_PROCESSES(&far_near_process);
//-----------------------------------------------------------------

PROCESS_THREAD(far_near_process, ev, data)
{
	PROCESS_BEGIN();

	/* Initialize stuff here. */
	PRINTF("\n++++++++++++++++++++++++++++++\n");
	PRINTF("+  Test hello world near far   +\n");
	PRINTF("++++++++++++++++++++++++++++++\n\n");

	SENSORS_ACTIVATE(button_sensor);

	static uint32_t seconds = 2;
	static struct etimer et; // Define the timer

	short status = 0;

	//list_data = (struct data*)calloc(MAX_DATA,sizeof(struct data));

	etimer_set(&et, CLOCK_SECOND*seconds);  // Set the timer

    while(1)
    {
		PROCESS_YIELD();

		if(etimer_expired(&et))
		{
			/* reset timer */
			etimer_reset(&et);
			incrCounter(pcounter);
			PRINTF("value %d!\n", pcounter[count]);
			PRINTF("value2 %d!\n", pcounter2);
			PRINTF("value3 %d!\n", values[count]);
		}

		if (ev == sensors_event && data == &button_sensor) {
			PRINTF("button works!\n");
			status = decrCounter(pcounter);
			PRINTF("status: %d\n", status);
			passValueFromNear(count);
			//PRINTF("%ld", (list_data+count)->time_last_activity);
			count++;
		}
	}
    //free(list_data);
	PROCESS_END();
}

void __attribute__((__far__)) incrCounter(uint8_t* pointer)
{
	pointer[0]++;
}

short __attribute__((__far__)) decrCounter(uint8_t* pointer)
{
	uint8_t value;

	value = pointer[2];
	if(value == 0) {
		PRINTF("ZERO\n");
		value = 100;
	}
	else {
		return 2;
	}
	pointer[2] = value;
	return 0;
}

void __attribute__((__far__)) passValueFromNear(uint8_t cnt)
{
	uint8_t state,i;
	uip_ipaddr_t curr_ip;

	cnt = cnt+10;
	pcounter2 = cnt+values[cnt];
	PRINTF("Pass count: %d\n", cnt);

	for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
		state = uip_ds6_if.addr_list[i].state;
		if(uip_ds6_if.addr_list[i].isused && state == ADDR_PREFERRED) {
			memcpy(&curr_ip.u8[0], &uip_ds6_if.addr_list[i].ipaddr.u8[0], 16);
		}
	}

	PRINTF("IP: "); for(i=0; i<15; i++) PRINTF("%02x", curr_ip.u8[i]); PRINTF("\n");

	//&(list_data+cnt)->time_last_activity = clock_seconds();
}
