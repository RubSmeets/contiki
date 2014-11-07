/*
 * Transmit audio with compression (ADPCM) and receive audio.
 */

#include "contiki.h"
#include "isr_compat.h"
#include <string.h>

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
#include "dev/z1-dac.h"
#include "dev/z1-phidgets.h"
#include "ADPCM_codec.h"

#define DEBUG DEBUG_PRINT //DEBUG_PRINT
#include "net/uip-debug.h"
#if DEBUG
#include <stdio.h>
#define PRINTFDEBUG(...) PRINTF(__VA_ARGS__)
#else
#define PRINTFDEBUG(...)
#endif /* DEBUG == 1*/

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

#define AUDIO_STREAM_TIMEOUT	(CLOCK_SECOND*2)	/* amount of seconds */
#define MAX_PAYLOAD_LEN			70
#define MAX_REC_PAYLOAD_LEN		80
#define AUDIO_HDR_SIZE			2

typedef struct {
	uint16_t	seqnr;
	uint8_t		*adpcm_data;
} udp_audio_packet_t;

/* UDP connection data */
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;

/* Audio data transmit */
static uint8_t transmit_mode;
static uint8_t enable_capture;
static udp_audio_packet_t packet;
static uint16_t audio_sample_buf_aligned[MAX_PAYLOAD_LEN];
static uint8_t *audio_sample_buf = (uint8_t *)audio_sample_buf_aligned;
static uint16_t udp_packet_buf_aligned[MAX_PAYLOAD_LEN+AUDIO_HDR_SIZE];
static uint8_t *udp_packet_buf = (uint8_t *)udp_packet_buf_aligned;
static uint8_t ptr;

/* Audio data receive */
static struct etimer periodic;
static uint8_t write_ptr;
static uint8_t read_ptr;
static uint8_t receive_mode;
static uint16_t audio_playback_aligned[(MAX_REC_PAYLOAD_LEN/2)*3];
static uint8_t *audio_playback = (uint8_t *)audio_playback_aligned;
static uint16_t decodedValue;

/* Functions */
static void stop_transmit(void);
static void start_transmit(void);

/*---------------------------------------------------------------------------*/
PROCESS(measure_process, "measure_process for cc2420");
/*---------------------------------------------------------------------------*/
PROCESS(udp_stream_process_1, "UDP stream process 1");
/*---------------------------------------------------------------------------*/
PROCESS(udp_stream_process_2, "UDP stream process 2");
/*---------------------------------------------------------------------------*/
AUTOSTART_PROCESSES(&measure_process, &udp_stream_process_1, &udp_stream_process_2);
/*---------------------------------------------------------------------------*/
HWCONF_PIN(AUDIO_AMP_ENABLE, 6, 4); /* Enable pin of audio amp is located on p6.4 */
HWCONF_TB_IRQ(TIMERB, 1);
HWCONF_ADC_IRQ(PHIDGET, ADC7);
/*---------------------------------------------------------------------------*/

static void
tcpip_handler(void)
{
	char *appdata;
	uint8_t i, len;

	if(uip_newdata()) {
		appdata = (char *)uip_appdata;
		len = uip_datalen();
		//PRINTFDEBUG("Len: %d", len);

		if(len == MAX_REC_PAYLOAD_LEN) {
			etimer_restart(&periodic);

			if(TIMERB_IRQ_ENABLED() == 0) {
				PRINTFDEBUG("DAC on\n");
				leds_on(LEDS_BLUE);

				/* Enable audio amp */
				AUDIO_AMP_ENABLE_SET();

				/* Prep. DAC variables */
				write_ptr = 0;
				read_ptr = 0;
				receive_mode = 0x01;
				decodedValue = 0;

				TIMERB_ENABLE_IRQ();
			}

			memcpy(&audio_playback[write_ptr*MAX_REC_PAYLOAD_LEN], appdata, len);

			if(write_ptr == 2) 	write_ptr = 0;
			else				write_ptr++;

		} else if(strncmp(appdata, "ready", 5) == 0) {
			/* Start audio transmission */
			PRINTFDEBUG("Capture on\n");
			leds_on(LEDS_RED);

			enable_capture = 1;
			/* TODO: Enable digital input interrupt:
			 * - input is high when sound pressure rises above predefined level
			 * - input is low when sound pressure is to low
			 */

			/* Turn on ADC */
			PHIDGET_ENABLE_IRQ();
			PHIDGET_START_CONVERSION();

		} else if(strncmp(appdata, "stop", 4) == 0) {
			/* Start audio transmission */
			PRINTFDEBUG("Capture off\n");
			leds_on(LEDS_RED);

			enable_capture = 0;

			/* Explicitly turn off adc transmission */
			stop_transmit();
		}
	}
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
  //uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0, 0x00ff, 0xfe00, 1);
  //uip_ip6addr(&server_ipaddr, 0x20ff, 2, 0, 0, 0xc30c, 0, 0, 2);
  //uip_ip6addr(&server_ipaddr, 0x20ff, 1, 0, 0, 0, 0, 0, 1);
   uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);
#else
/* Mode 3 - derived from server link-local (MAC) address */
  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0x0250, 0xc2ff, 0xfea8, 0xcd1a); //redbee-econotag
#endif
}
/*---------------------------------------------------------------------------*/
static void
start_transmit(void)
{
	char msg[4];

	/* Prep. ADC variables */
	transmit_mode = 0x00;
	packet.seqnr = 1;
	ptr = 0;

	PRINTFDEBUG("Starting Call\n");

	/* Send start message */
	sprintf(&msg[0], "call");
	uip_udp_packet_sendto(client_conn, msg, sizeof(msg), &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
}
/*---------------------------------------------------------------------------*/
static void
stop_transmit(void)
{
	/* Only disable IRQ not the REF voltage */
	PHIDGET_DISABLE_IRQ();
	PHIDGET_STOP_CONVERSION();

	/* Disable audio amp to save power */
	AUDIO_AMP_ENABLE_CLEAR();

	leds_off(LEDS_RED);
}
/*---------------------------------------------------------------------------*/
static void
init_timerB(void)
{
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

	PROCESS_BEGIN();

	PRINTF("Starting audio transmit process\n");

	SENSORS_ACTIVATE(button_sensor);
	/* Init the DAC here and enable external voltage ref. of ADC12
	   allowing enough time for the voltage to stabilize
	   Configure pin 6.4 to enable/disable audio amp.		*/
	dac_init(Z1_DAC_0);
	init_timerB();
	SENSORS_ACTIVATE(phidgets);
	AUDIO_AMP_ENABLE_MAKE_OUTPUT();


	/* Turn of ADC conversion and interrupt */
	PHIDGET_STOP_CONVERSION();
	PHIDGET_DISABLE_IRQ();

	set_global_address();
	print_local_addresses();

	/* The data sink runs with a 100% duty cycle in order to ensure high
	 packet reception rates. */
	NETSTACK_MAC.off(1);

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

	etimer_set(&periodic, AUDIO_STREAM_TIMEOUT);
	while(1) {
		PROCESS_YIELD();

		if(ev == sensors_event && data == &button_sensor) {
			start_transmit();
		} else if(ev == tcpip_event) {
			tcpip_handler();
		} else if(etimer_expired(&periodic)) {
			PRINTFDEBUG("time-out\n");

			/* Disable audio amp to save power */
			AUDIO_AMP_ENABLE_CLEAR();

			/* Turn off DAC */
			TIMERB_DISABLE_IRQ();
			leds_off(LEDS_BLUE);
		}
//		else if() {	/* read IO for microphone enable */
//			if(enable_capture) {
//				/* Turn on ADC */
//				PHIDGET_ENABLE_IRQ();
//				PHIDGET_START_CONVERSION();
//			}
//		}

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

		packet.adpcm_data = &audio_sample_buf[0];

		/* Create packet */
		udp_packet_buf[0] = packet.seqnr >> 8;
		udp_packet_buf[1] = packet.seqnr & 0xFF;
		memcpy(&udp_packet_buf[2], packet.adpcm_data, MAX_PAYLOAD_LEN);

		uip_udp_packet_sendto(client_conn, &udp_packet_buf[0], (MAX_PAYLOAD_LEN+AUDIO_HDR_SIZE), &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
		packet.seqnr++;
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

		packet.adpcm_data = &audio_sample_buf[MAX_PAYLOAD_LEN];

		/* Create packet */
		udp_packet_buf[0] = packet.seqnr >> 8;
		udp_packet_buf[1] = packet.seqnr & 0xFF;
		memcpy(&udp_packet_buf[2], packet.adpcm_data, MAX_PAYLOAD_LEN);

		uip_udp_packet_sendto(client_conn, &udp_packet_buf[0], (MAX_PAYLOAD_LEN+AUDIO_HDR_SIZE), &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
		packet.seqnr++;
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
uint8_t first = 0x01;
ISR(ADC12, adc_service_routine)
{
	uint8_t tempSample;
	int tempValue = phidgets.value(PHIDGET3V_2);

	if(first == 0x01) {
		if(ptr < MAX_PAYLOAD_LEN) {
			PRINTF("%04x", tempValue);
		} else {
			first = 0x00;
		}
	}


	switch(transmit_mode) {
		case 0x00:
			tempSample = ADPCM_Encoder(tempValue)<<4; /* ADPCM code (bit 7-4) */
			transmit_mode=0x01;
			break;
		case 0x01:
			tempSample = tempSample + (ADPCM_Encoder(tempValue) & 0x0F); /* ADPCM code (bit 3-0) */
			audio_sample_buf[ptr] = tempSample;
			ptr++;
			transmit_mode=0x00;

			if(ptr == MAX_PAYLOAD_LEN)  /* packet filled? */
			{
				process_poll(&udp_stream_process_1);
			} else if(ptr == (MAX_PAYLOAD_LEN*2)) {
				ptr = 0;
				process_poll(&udp_stream_process_2);
			}
			break;
	}

	PHIDGET_CLEAR_IRQ_FLAG(); /* Clear Interrupt flag */
}
/*---------------------------------------------------------------------------*/
ISR(TIMERB1, timerb1_service_routine)
{
	switch(receive_mode) {
		case 0x01: /* playback bit (7-4) */
			dac_setValue(decodedValue, Z1_DAC_0);

			decodedValue = ADPCM_Decoder((audio_playback[read_ptr]>>4) & 0x0F);
							 /* ADPCM_Decoder() execution time varies depending on the
								ADPCM code. This is the reason why the value is stored in
								a variable and during the next ISR call the DAC register
								is loaded. */
			receive_mode=0x02;
			break;
		default: /* playback bit (3-0) */
			dac_setValue(decodedValue, Z1_DAC_0);

			decodedValue = ADPCM_Decoder(audio_playback[read_ptr] & 0x0F);
							 /* ADPCM_Decoder() execution time varies depending on the
								ADPCM code. This is the reason why the value is stored in
								a variable and during the next ISR call the DAC register
								is loaded. */

			read_ptr++;
			receive_mode=0x01;

			if (read_ptr >= (MAX_REC_PAYLOAD_LEN*3))
			{
				read_ptr = 0;
			}
			break;
	}

	TIMERB_CLEAR_IRQ_FLAG(); /* Clear Interrupt flag */
}
/*---------------------------------------------------------------------------*/
