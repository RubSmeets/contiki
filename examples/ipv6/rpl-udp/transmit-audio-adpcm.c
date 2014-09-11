/*
 * Transmit audio with compression (ADPCM)
 */

#include "contiki.h"
#include "isr_compat.h"

/* Communication includes */
#include "net/uip.h"
#include "net/uip-ds6.h"
#include "net/uip-udp-packet.h"

/* Include for peripheral */
#include "dev/leds.h"
#include "dev/cc2420.h"
#include "dev/button-sensor.h"
#include "dev/hwconf.h"

/* Audio includes */
#include "dev/z1-phidgets.h"
#include "ADPCM_codec.h"

#define DEBUG DEBUG_PRINT //DEBUG_PRINT
#include "net/uip-debug.h"

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

#define SEND_INTERVAL		(CLOCK_SECOND)
#define MAX_PAYLOAD_LEN		40

/* UDP connection data */
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;

/* Audio data */
static uint8_t mode;
static uint8_t tempSample;
static uint16_t seqnr;
static uint8_t audio_sample_buf[MAX_PAYLOAD_LEN*2];
static uint8_t ptr;

uint16_t counter;

/*---------------------------------------------------------------------------*/
PROCESS(measure_process, "measure_process for cc2420");
/*---------------------------------------------------------------------------*/
PROCESS(udp_stream_process_1, "UDP stream process 1");
/*---------------------------------------------------------------------------*/
PROCESS(udp_stream_process_2, "UDP stream process 2");
/*---------------------------------------------------------------------------*/
AUTOSTART_PROCESSES(&measure_process, &udp_stream_process_1, &udp_stream_process_2);
/*---------------------------------------------------------------------------*/


static void
tcpip_handler(void)
{

}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
      /* hack to make address "final" */
      if (state == ADDR_TENTATIVE) {
	uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

/* The choice of server address determines its 6LoPAN header compression.
 * (Our address will be compressed Mode 3 since it is derived from our link-local address)
 * Obviously the choice made here must also be selected in udp-server.c.
 *
 * For correct Wireshark decoding using a sniffer, add the /64 prefix to the 6LowPAN protocol preferences,
 * e.g. set Context 0 to aaaa::.  At present Wireshark copies Context/128 and then overwrites it.
 * (Setting Context 0 to aaaa::1111:2222:3333:4444 will report a 16 bit compressed address of aaaa::1111:22ff:fe33:xxxx)
 *
 * Note the IPCMV6 checksum verification depends on the correct uncompressed addresses.
 */

#if 0
/* Mode 1 - 64 bits inline */
   uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);
#elif 1
/* Mode 2 - 16 bits inline */
  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0, 0x00ff, 0xfe00, 1);
  //uip_ip6addr(&server_ipaddr, 0x20ff, 2, 0, 0, 0xc30c, 0, 0, 2);
  //uip_ip6addr(&server_ipaddr, 0x20ff, 1, 0, 0, 0, 0, 0, 1);
#else
/* Mode 3 - derived from server link-local (MAC) address */
  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0x0250, 0xc2ff, 0xfea8, 0xcd1a); //redbee-econotag
#endif
}
/*---------------------------------------------------------------------------*/
static void
start_transmit(void)
{
	char msg[5];
	leds_on(LEDS_RED);

	/* Prep. ADC variables */
	mode = 0x00;
	seqnr = 1;
	ptr = 0;

	/* Send start message */
	sprintf(&msg[0], "start");
	uip_udp_packet_sendto(client_conn, msg, 5, &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));

	/* Turn on ADC */
	SENSORS_ACTIVATE(phidgets);
}
/*---------------------------------------------------------------------------*/
static void
stop_transmit(void)
{
	char msg[4];
	SENSORS_DEACTIVATE(phidgets);	/* Deactivate ADC12 */

	/* Send start message */
	sprintf(&msg[0], "stop");
	uip_udp_packet_sendto(client_conn, msg, 4, &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));

	leds_off(LEDS_RED);

}
/*---------------------------------------------------------------------------*/
static void
init_timerB(void)
{
	/* Configure PIN OUT (TBCCR2 available as zolertia output)  */
	//P4DIR |= (1 << 2);
	//P4SEL |= (1 << 2);

	/* Turn off timer */
	TBCTL = 0x00;

	/* Set timer B - SMCLK (8 MHz?)	*/
	TBCTL = TBSSEL_2 | TBCLR;

	/* Set compare register 1 to toggle/reset */
	TBCCTL1 |= OUTMOD_3;

	/* Set clock period = 1000 (8000000/8000) */
	TBCCR0 = 0x03E2;	/* Values are adjusted to exact 8 kHz (994) */
	TBCCR1 = 0x01F1;	/* (497) */

	/* Start Timer_B in UP mode. */
	TBCTL |= MC_1;

}
/*---------------------------------------------------------------------------*/
__attribute__((__far__))
PROCESS_THREAD(measure_process, ev, data)
{
	static struct etimer periodic;
	static uint8_t toggle_record_play = 0;

	PROCESS_BEGIN();

	PRINTF("Starting audio transmit process\n");

	SENSORS_ACTIVATE(button_sensor);
	init_timerB();

	set_global_address();
	print_local_addresses();

	/* new connection with remote host */
	client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL);
	if(client_conn == NULL) {
		PRINTF("No UDP connection available, exiting the process!\n");
		PROCESS_EXIT();
	}
	udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT));

	PRINTF("Created a connection with the server ");
	PRINT6ADDR(&client_conn->ripaddr);
	PRINTF(" local/remote port %u/%u\n",
	UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));

	etimer_set(&periodic, SEND_INTERVAL);
	while(1) {
		PROCESS_YIELD();

		if(etimer_expired(&periodic)) {
		      etimer_reset(&periodic);
		      //PRINTF("c:%d", counter);
		      counter = 0;
		} else if(ev == sensors_event && data == &button_sensor) {
			if(toggle_record_play == 0x00) {
				/* Start record */
				start_transmit();
				toggle_record_play = 0x01;
			} else {
				/* Stop record */
				stop_transmit();
				toggle_record_play = 0x00;
			}
		}

	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
__attribute__((__far__))
PROCESS_THREAD(udp_stream_process_1, ev, data)
{

	PROCESS_BEGIN();
	PRINTF("UDP stream process 1 started\n");

	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		uip_udp_packet_sendto(client_conn, &audio_sample_buf[0], MAX_PAYLOAD_LEN, &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
__attribute__((__far__))
PROCESS_THREAD(udp_stream_process_2, ev, data)
{

	PROCESS_BEGIN();
	PRINTF("UDP stream process 2 started\n");

	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		uip_udp_packet_sendto(client_conn, &audio_sample_buf[MAX_PAYLOAD_LEN], MAX_PAYLOAD_LEN, &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
HWCONF_ADC_IRQ(PHIDGET, ADC7);
ISR(ADC12, adc_service_routine)
{
	switch(mode) {
		case 0x00:
			tempSample = ADPCM_Encoder(phidgets.value(PHIDGET3V_2))<<4; /* ADPCM code (bit 7-4) */
			mode=0x01;
			break;
		case 0x01:
			tempSample = tempSample + (ADPCM_Encoder(phidgets.value(PHIDGET3V_2)) & 0x0F); /* ADPCM code (bit 3-0) */
			audio_sample_buf[ptr] = tempSample;
			ptr++;
			mode=0x00;

			if(ptr == MAX_PAYLOAD_LEN)  /* packet filled? */
			{
				process_poll(&udp_stream_process_1);
			} else if(ptr == (MAX_PAYLOAD_LEN*2)) {
				ptr = 0;
				process_poll(&udp_stream_process_2);
			}
			break;
	}

	counter++;

	PHIDGET_CLEAR_IRQ_FLAG(); /* Clear Interrupt flag */
}
/*---------------------------------------------------------------------------*/
