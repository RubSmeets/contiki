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
#define TONE_INTERVAL			(CLOCK_SECOND)
#define BACK_OFF_TIMEOUT		20  /* 20 clock seconds */
#define REPEAT_SINE				40  /* 40 * (125µs * 200 samples) = 1 second tone */
#define SINE_SAMPLE_COUNT		400
#define MAX_PAYLOAD_LEN			84
#define MAX_REC_PAYLOAD_LEN		84
#define AUDIO_HDR_SIZE			2
#define PRESSURE_IN				7	/* Port 2 pin 7 */
#define	BUTTON_EXT				6	/* Port 2 pin 6 */
#define AUDIO_HDR				4

/* 440 Hz sine wave 16-bit signed little endian Byte order */
static const uint8_t sine_440[] = {0x00, 0x00, 0xA9, 0x1E, 0xB1, 0x39, 0xE8, 0x4D, 0xE8, 0x58, 0x65, 0x59, 0x51, 0x4F, 0xDB, 0x3B, 0x52, 0x21, 0xD8, 0x02,
							0x08, 0xE4, 0x87, 0xC8, 0x94, 0xB3, 0xAB, 0xA7, 0x34, 0xA6, 0x5B, 0xAF, 0x0B, 0xC2, 0x0E, 0xDC, 0x51, 0xFA, 0x40, 0x19,
							0x33, 0x35, 0xDC, 0x4A, 0xAB, 0x57, 0x1C, 0x5A, 0xE5, 0x51, 0x00, 0x40, 0x8A, 0x26, 0x85, 0x08, 0x7E, 0xE9, 0x20, 0xCD,
							0xC7, 0xB6, 0x15, 0xA9, 0xAB, 0xA5, 0xEF, 0xAC, 0x05, 0xBE, 0xE9, 0xD6, 0xA8, 0xF4, 0xBF, 0x13, 0x7F, 0x30, 0x84, 0x47,
							0x15, 0x56, 0x77, 0x5A, 0x27, 0x54, 0xE4, 0x43, 0x9B, 0x2B, 0x29, 0x0E, 0x0A, 0xEF, 0xED, 0xD1, 0x43, 0xBA, 0xD7, 0xAA,
							0x7D, 0xA5, 0xD7, 0xAA, 0x43, 0xBA, 0xED, 0xD1, 0x0A, 0xEF, 0x29, 0x0E, 0x9B, 0x2B, 0xE4, 0x43, 0x27, 0x54, 0x77, 0x5A,
							0x15, 0x56, 0x84, 0x47, 0x7F, 0x30, 0xBF, 0x13, 0xA8, 0xF4, 0xE9, 0xD6, 0x05, 0xBE, 0xEF, 0xAC, 0xAB, 0xA5, 0x15, 0xA9,
							0xC7, 0xB6, 0x20, 0xCD, 0x7E, 0xE9, 0x85, 0x08, 0x8A, 0x26, 0x00, 0x40, 0xE5, 0x51, 0x1C, 0x5A, 0xAB, 0x57, 0xDC, 0x4A,
							0x33, 0x35, 0x40, 0x19, 0x51, 0xFA, 0x0E, 0xDC, 0x0B, 0xC2, 0x5B, 0xAF, 0x34, 0xA6, 0xAB, 0xA7, 0x94, 0xB3, 0x87, 0xC8,
							0x08, 0xE4, 0xD8, 0x02, 0x52, 0x21, 0xDB, 0x3B, 0x51, 0x4F, 0x65, 0x59, 0xE8, 0x58, 0xE8, 0x4D, 0xB1, 0x39, 0xA9, 0x1E,
							0x00, 0x00, 0x57, 0xE1, 0x4F, 0xC6, 0x18, 0xB2, 0x18, 0xA7, 0x9B, 0xA6, 0xAF, 0xB0, 0x25, 0xC4, 0xAE, 0xDE, 0x28, 0xFD,
							0xF8, 0x1B, 0x79, 0x37, 0x6C, 0x4C, 0x55, 0x58, 0xCC, 0x59, 0xA5, 0x50, 0xF5, 0x3D, 0xF2, 0x23, 0xAF, 0x05, 0xC0, 0xE6,
							0xCD, 0xCA, 0x24, 0xB5, 0x55, 0xA8, 0xE4, 0xA5, 0x1B, 0xAE, 0x00, 0xC0, 0x76, 0xD9, 0x7B, 0xF7, 0x82, 0x16, 0xE0, 0x32,
							0x39, 0x49, 0xEB, 0x56, 0x55, 0x5A, 0x11, 0x53, 0xFB, 0x41, 0x17, 0x29, 0x58, 0x0B, 0x41, 0xEC, 0x81, 0xCF, 0x7C, 0xB8,
							0xEB, 0xA9, 0x89, 0xA5, 0xD9, 0xAB, 0x1C, 0xBC, 0x65, 0xD4, 0xD7, 0xF1, 0xF6, 0x10, 0x13, 0x2E, 0xBD, 0x45, 0x29, 0x55,
							0x83, 0x5A, 0x29, 0x55, 0xBD, 0x45, 0x13, 0x2E, 0xF6, 0x10, 0xD7, 0xF1, 0x65, 0xD4, 0x1C, 0xBC, 0xD9, 0xAB, 0x89, 0xA5,
							0xEB, 0xA9, 0x7C, 0xB8, 0x81, 0xCF, 0x41, 0xEC, 0x58, 0x0B, 0x17, 0x29, 0xFB, 0x41, 0x11, 0x53, 0x55, 0x5A, 0xEB, 0x56,
							0x39, 0x49, 0xE0, 0x32, 0x82, 0x16, 0x7B, 0xF7, 0x76, 0xD9, 0x00, 0xC0, 0x1B, 0xAE, 0xE4, 0xA5, 0x55, 0xA8, 0x24, 0xB5,
							0xCD, 0xCA, 0xC0, 0xE6, 0xAF, 0x05, 0xF2, 0x23, 0xF5, 0x3D, 0xA5, 0x50, 0xCC, 0x59, 0x55, 0x58, 0x6C, 0x4C, 0x79, 0x37,
							0xF8, 0x1B, 0x28, 0xFD, 0xAE, 0xDE, 0x25, 0xC4, 0xAF, 0xB0, 0x9B, 0xA6, 0x18, 0xA7, 0x18, 0xB2, 0x4F, 0xC6, 0x57, 0xE1};

/* UDP connection data */
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;

/* Wait tone data */
static struct etimer calltone;
static uint8_t back_off_count;
static uint8_t repeat_sine_count;
static uint8_t play_tone;
static uint8_t tone_on_off;
static uint16_t sine_ptr;

/* Audio data transmit */
static const char PAUZE_STREAM[] = "pauzeS";
static const char START_STREAM[] = "startS";
static uint8_t transmit_mode;
static uint16_t audio_sample_buf_aligned[MAX_PAYLOAD_LEN];
static uint8_t *audio_sample_buf = (uint8_t *)audio_sample_buf_aligned;
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
turn_on_adc(void) {
	/* Prep. ADC variables */
	transmit_mode = 0x00;
	ptr = 0;

	/* Turn on ADC */
	PHIDGET_ENABLE_IRQ();
	PHIDGET_START_CONVERSION();
}
/*---------------------------------------------------------------------------*/
static void
turn_off_adc(void) {
	/* Only disable IRQ not the REF voltage */
	PHIDGET_DISABLE_IRQ();
	PHIDGET_STOP_CONVERSION();
}
/*---------------------------------------------------------------------------*/
static void
turn_on_dac(void) {
	/* Enable audio amp */
	AUDIO_AMP_ENABLE_SET();
	TIMERB_ENABLE_IRQ();
}
/*---------------------------------------------------------------------------*/
static void
turn_off_dac(void) {
	/* Explicitly turn off DAC */
	TIMERB_DISABLE_IRQ();
	AUDIO_AMP_ENABLE_CLEAR();
}
/*---------------------------------------------------------------------------*/
static void
turn_on_wait_tone(void) {
	sine_ptr = 0;
	repeat_sine_count = 0;

	/* Enable audio amp */
	turn_on_dac();
	tone_on_off = 0;
}
/*---------------------------------------------------------------------------*/
static void
turn_off_wait_tone(void) {
	turn_off_dac();
	tone_on_off = 1;
}
/*---------------------------------------------------------------------------*/
static void
turn_on_audio_playback(void) {
	/* Prep. DAC variables */
	write_ptr = 0;
	read_ptr = 0;
	receive_mode = 0x01;
	decodedValue = 0;

	turn_on_dac();
}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
	char *appdata;
	uint8_t len;

	if(uip_newdata()) {
		appdata = (char *)uip_appdata;
		len = uip_datalen();
		//PRINTFDEBUG("Len: %d", len);

		if(len == MAX_REC_PAYLOAD_LEN) {
			etimer_restart(&periodic);

			/* Disable wait tone */
			play_tone = 0x00;

			if(TIMERB_IRQ_ENABLED() == 0) {
				PRINTFDEBUG("DAC on\n");
				leds_on(LEDS_BLUE);

				/* Prepare audio playback */
				turn_on_audio_playback();
			}

			memcpy(&audio_playback[write_ptr*MAX_REC_PAYLOAD_LEN], appdata, len);

			if(write_ptr == 2) 	write_ptr = 0;
			else				write_ptr++;

		} else if(strncmp(appdata, "ready", 5) == 0) {
			/* Start audio transmission */
			PRINTFDEBUG("Capture on\n");
			leds_on(LEDS_RED);

			/* Disable wait tone */
			play_tone = 0x00;

			/* Allow sound to be recorded when sound pressure is high enough */
			enable_dio_irq();
			turn_on_adc();

		} else if(strncmp(appdata, "stop", 4) == 0) {
			/* Stop audio transmission */
			PRINTFDEBUG("Capture off\n");

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

  uip_ip6addr(&ipaddr, 0xbbbb, 0, 0, 0, 0, 0, 0, 0);
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
   uip_ip6addr(&server_ipaddr, 0xbbbb, 0, 0, 0, 0, 0, 0, 1);
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

	/* Play tone to indicate wait */
	PRINTF("Play tone\n");
	play_tone = 1;
	back_off_count = 0;
	etimer_restart(&calltone);

	PRINTFDEBUG("Starting Call\n");

	/* Send start message */
	sprintf(&msg[0], "call");
	uip_udp_packet_sendto(client_conn, msg, sizeof(msg), &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
}
/*---------------------------------------------------------------------------*/
static void
stop_transmit(void)
{
	/* Disable ADC */
	disable_dio_irq();
	turn_off_adc();

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
	   Configure pin 6.4 to enable/disable audio amp.
	   Configure digital I/O for audio pressure detection */
	init_dio(&measure_process);
	dac_init(Z1_DAC_0);
	init_timerB();
	SENSORS_ACTIVATE(phidgets);
	AUDIO_AMP_ENABLE_MAKE_OUTPUT();

	/* Turn off ADC conversion and interrupt */
	turn_off_adc();

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
	etimer_set(&calltone, TONE_INTERVAL);
	while(1) {
		PROCESS_YIELD();

		if((ev == sensors_event) && (data == &button_sensor)) { /* read IO for microphone enable */
			start_transmit();
			PRINTF("Button pressed\n");
		} else if(ev == PROCESS_EVENT_MSG) {	/* sound pressure high enough */
			if(*(uint8_t *)data < 32) {	/* Rising edge */
				/* Interrupt stream from remote */
				uip_udp_packet_sendto(client_conn, PAUZE_STREAM, 6, &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
				/* Turn on ADC */
				turn_on_adc();
			} else {	/* Falling edge */
				/* Enable stream from remote */
				uip_udp_packet_sendto(client_conn, START_STREAM, 6, &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
				/* Turn off ADC */
				turn_off_adc();
			}
		} else if(ev == tcpip_event) {
			tcpip_handler();
		} else if(etimer_expired(&periodic) && data == &periodic) {
			PRINTFDEBUG("time-out\n");
			/* Disable audio playback */
			turn_off_dac();
			leds_off(LEDS_BLUE);
		} else if(etimer_expired(&calltone) && data == &calltone) {
			if(play_tone) {
				etimer_restart(&calltone);

				/* Check how long the tone has been playing */
				if(back_off_count == BACK_OFF_TIMEOUT) {
					play_tone = 0x00;
				} else {
					back_off_count++;
				}

				/* Generate tone 1 second on - 1 second off */
				if(tone_on_off == 1) {
					turn_on_wait_tone();
				} else {
					turn_off_wait_tone();
				}
			} else {
				PRINTF("Stop tone\n");
				turn_off_wait_tone();
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
uint8_t first = 0x00;
ISR(ADC12, adc_service_routine)
{
	uint8_t tempSample;
	int tempValue = phidgets.value(PHIDGET3V_2);
	tempValue = tempValue - 2457;	/* remove offset (1,5V - ref 2,5V equals 60% of max value 4096) */

	if(first == 0x01) {
		if(ptr < MAX_PAYLOAD_LEN) {
			PRINTF("%04x", tempValue);
		} else {
			first = 0x00;
		}
	}

	if(ptr % MAX_PAYLOAD_LEN == 0) {
		signed int tempSample = ADPCM_getPrevSample();
		int tempStep = ADPCM_getPrevStepSize();
		audio_sample_buf[ptr] = tempSample >> 8;
		audio_sample_buf[ptr+1] = tempSample & 0xFF;
		audio_sample_buf[ptr+2] = tempStep >> 8;
		audio_sample_buf[ptr+3] = tempStep & 0xFF;
		ptr += 4;
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
	if(!play_tone) {
		if(read_ptr % MAX_REC_PAYLOAD_LEN == 0) {
			ADPCM_setPrevSample((signed int)((audio_playback[read_ptr]<<8) + (audio_playback[read_ptr+1] & 0xFF)));
			read_ptr += 2;
			ADPCM_setPrevStepSize((int)((audio_playback[read_ptr]<<8) + (audio_playback[read_ptr+1] & 0xFF)));
			read_ptr += 2;
		}

		switch(receive_mode) {
			case 0x01: /* playback bit (7-4) */
				dac_setValue(decodedValue, Z1_DAC_0, 0);

				decodedValue = ADPCM_Decoder((audio_playback[read_ptr]>>4) & 0x0F);
								 /* ADPCM_Decoder() execution time varies depending on the
									ADPCM code. This is the reason why the value is stored in
									a variable and during the next ISR call the DAC register
									is loaded. */
				receive_mode=0x02;
				break;
			default: /* playback bit (3-0) */
				dac_setValue(decodedValue, Z1_DAC_0, 0);

				decodedValue = ADPCM_Decoder(audio_playback[read_ptr] & 0x0F);
								 /* ADPCM_Decoder() execution time varies depending on the
									ADPCM code. This is the reason why the value is stored in
									a variable and during the next ISR call the DAC register
									is loaded. */

				read_ptr++;
				receive_mode=0x01;

				if (read_ptr >= (MAX_REC_PAYLOAD_LEN*3)) {
					read_ptr = 0;
				}
				break;
		}
	} else {
		if(repeat_sine_count < REPEAT_SINE) {
			decodedValue = (uint16_t) ((sine_440[sine_ptr+1] << 8) | (sine_440[sine_ptr] & 0xff));
			dac_setValue(decodedValue, Z1_DAC_0, 1);
			sine_ptr = sine_ptr + 2;

			if(sine_ptr == (SINE_SAMPLE_COUNT-2)) {
				repeat_sine_count++;
				sine_ptr = 0;
			}
		}
	}

	TIMERB_CLEAR_IRQ_FLAG(); /* Clear Interrupt flag */
}
/*---------------------------------------------------------------------------*/
