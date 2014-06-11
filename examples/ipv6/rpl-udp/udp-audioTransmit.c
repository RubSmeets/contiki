/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#include "net/uip.h"
#include "net/uip-ds6.h"
#include "net/uip-udp-packet.h"
#include "dev/button-sensor.h"
#include "sys/ctimer.h"
#include "cc2420-aes.h"
#include "dev/cc2420.h"
#include "symm-key-client-v1.h"
#include "dev/xmem.h"
#include "dev/z1-dac.h"

#include "dev/z1-phidgets.h"
#include "dev/hwconf.h"
#include "isr_compat.h"

//#include "contiki-conf.h"
#ifdef WITH_COMPOWER
#include "powertrace.h"
#endif
#include <stdio.h>
#include <string.h>

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

#define UDP_EXAMPLE_ID  190

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"

#ifndef PERIOD
#define PERIOD 1
#endif

#define START_INTERVAL		(15 * CLOCK_SECOND)
#define SEND_INTERVAL		(PERIOD * CLOCK_SECOND)
#define SEND_TIME		(random_rand() % (SEND_INTERVAL))
#define MAX_PAYLOAD_LEN		60

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;

static uint16_t audio_buf_aligned[256];
static uint8_t *audio_buf = (uint8_t *)audio_buf_aligned;

static uint16_t data_pointer = 1;
static uint16_t poll_cnt_1 = 0;
static uint16_t poll_cnt_2 = 0;
static uint16_t poll_cnt_3 = 0;
static uint16_t poll_cnt_4 = 0;
static uint16_t isr_cnt = 0;
static uint8_t record = 1;
static uint8_t select = 0;

/* Page size is 256 Bytes */
#define PAGE_SIZE	256
static uint8_t page_cnt = 0;

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
/*---------------------------------------------------------------------------*/
PROCESS(udp_stream_process_1, "UDP stream process 1");
/*---------------------------------------------------------------------------*/
PROCESS(udp_stream_process_2, "UDP stream process 2");
/*---------------------------------------------------------------------------*/
PROCESS(udp_stream_process_3, "UDP stream process 3");
/*---------------------------------------------------------------------------*/
PROCESS(udp_stream_process_4, "UDP stream process 4");
/*---------------------------------------------------------------------------*/
PROCESS(xmem_read_process, "xmem read process");
/*---------------------------------------------------------------------------*/
PROCESS(xmem_write_process, "xmem write process");
/*---------------------------------------------------------------------------*/
//AUTOSTART_PROCESSES(&udp_client_process, &udp_stream_process_1, &udp_stream_process_2, &udp_stream_process_3, &udp_stream_process_4, &xmem_read_process, &xmem_write_process);
AUTOSTART_PROCESSES(&udp_client_process, &xmem_write_process);
/*---------------------------------------------------------------------------*/

static void
tcpip_handler(void)
{

}
/*---------------------------------------------------------------------------*/
static void
send_packet(void *ptr)
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
__attribute__((__far__))
PROCESS_THREAD(udp_client_process, ev, data)
{
  static struct etimer periodic;
#if WITH_COMPOWER
  static int print = 0;
#endif

  PROCESS_BEGIN();

  PROCESS_PAUSE();

  SENSORS_ACTIVATE(button_sensor);

  set_global_address();
  
  PRINTF("UDP client process started\n");

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

#if WITH_COMPOWER
  powertrace_sniff(POWERTRACE_ON);
#endif

  //SENSORS_ACTIVATE(phidgets);

  /* Erase audio data segment */
  xmem_erase(XMEM_ERASE_UNIT_SIZE, AUDIO_DATA);
  dac_init(Z1_DAC_0);

  etimer_set(&periodic, SEND_INTERVAL);
  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
    else if (ev == sensors_event && data == &button_sensor) {
    	SENSORS_ACTIVATE(phidgets);
    }
    
    if(etimer_expired(&periodic)) {
      etimer_reset(&periodic);
      PRINTF("I: %d ", isr_cnt);
//      PRINTF("P1: %d ", poll_cnt_1);
//      PRINTF("P2: %d ", poll_cnt_2);
//      PRINTF("P3: %d ", poll_cnt_3);
//      PRINTF("P4: %d ", poll_cnt_4);
      PRINTF("PG: %d ", page_cnt);

      isr_cnt = 0;
      poll_cnt_1 = 0;
      poll_cnt_2 = 0;
      poll_cnt_3 = 0;
      poll_cnt_4 = 0;

      if(page_cnt > 230) {
    	  PRINTF("Playback\n");
    	  record = 0;
    	  page_cnt = 1;
    	  data_pointer = 0;
    	  xmem_pread(&audio_buf[0], PAGE_SIZE, (AUDIO_DATA+(page_cnt*PAGE_SIZE)));
      }

#if WITH_COMPOWER
      if (print == 0) {
	powertrace_print("#P");
      }
      if (++print == 3) {
	print = 0;
      }
#endif

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

		uip_udp_packet_sendto(client_conn, &audio_buf[MAX_PAYLOAD_LEN*3], MAX_PAYLOAD_LEN, &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
		poll_cnt_1++;
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

		uip_udp_packet_sendto(client_conn, &audio_buf[0], MAX_PAYLOAD_LEN, &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
		poll_cnt_2++;
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
__attribute__((__far__))
PROCESS_THREAD(udp_stream_process_3, ev, data)
{

	PROCESS_BEGIN();
	PRINTF("UDP stream process 3 started\n");

	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		uip_udp_packet_sendto(client_conn, &audio_buf[MAX_PAYLOAD_LEN], MAX_PAYLOAD_LEN, &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
		poll_cnt_3++;
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
__attribute__((__far__))
PROCESS_THREAD(udp_stream_process_4, ev, data)
{

	PROCESS_BEGIN();
	PRINTF("UDP stream process 4 started\n");

	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		uip_udp_packet_sendto(client_conn, &audio_buf[MAX_PAYLOAD_LEN*2], MAX_PAYLOAD_LEN, &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
		poll_cnt_4++;
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
__attribute__((__far__))
PROCESS_THREAD(xmem_read_process, ev, data)
{

	PROCESS_BEGIN();
	PRINTF("xmem read process started\n");

	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		if(select == 0) {
			xmem_pread(&audio_buf[PAGE_SIZE], PAGE_SIZE, (AUDIO_DATA+(page_cnt*PAGE_SIZE)));
		} else if(select == 1) {
			xmem_pread(&audio_buf[0], PAGE_SIZE, (AUDIO_DATA+(page_cnt*PAGE_SIZE)));
		}
		page_cnt++;
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
__attribute__((__far__))
PROCESS_THREAD(xmem_write_process, ev, data)
{

	PROCESS_BEGIN();
	PRINTF("xmem read process started\n");

	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		if(select == 2) {
			xmem_pwrite(&audio_buf[PAGE_SIZE], PAGE_SIZE , (AUDIO_DATA+(page_cnt*PAGE_SIZE)));
		} else if(select == 3) {
			xmem_pwrite(&audio_buf[0], PAGE_SIZE, (AUDIO_DATA+(page_cnt*PAGE_SIZE)));
		}
		page_cnt++;
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
//HWCONF_ADC_IRQ(PHIDGET, ADC7);
//ISR(ADC12, adc_service_routine)
//{
//	audio_buf[data_pointer % (MAX_PAYLOAD_LEN*4)] = phidgets.value(PHIDGET3V_2);
//
//	if(data_pointer == 0) {
//		/* if overflowed --> send last half of buffer */
//		process_poll(&udp_stream_process_1);
//	} else if(data_pointer == MAX_PAYLOAD_LEN) {
//		/* if data_pointer exceeds half of buffer size -> send first half */
//		process_poll(&udp_stream_process_2);
//	} else if(data_pointer == (MAX_PAYLOAD_LEN*2)) {
//		/* if data_pointer exceeds half of buffer size -> send first half */
//		process_poll(&udp_stream_process_3);
//	} else if(data_pointer == (MAX_PAYLOAD_LEN*3)) {
//		/* if data_pointer exceeds half of buffer size -> send first half */
//		process_poll(&udp_stream_process_4);
//	}
//
//	data_pointer++;
//	isr_cnt++;
//	PHIDGET_CLEAR_IRQ_FLAG(); /* Clear Interrupt flag */
//}
/*---------------------------------------------------------------------------*/
HWCONF_ADC_IRQ(PHIDGET, ADC7);
ISR(ADC12, adc_service_routine)
{
//	if(record == 1) {
//		audio_buf[data_pointer] = (uint8_t)(phidgets.value(PHIDGET3V_2) >> 4);
//		//dac_setValue((uint16_t)audio_buf[data_pointer], Z1_DAC_0);
//
//		if(data_pointer == 0) {
//			/* if overflowed --> send last half of buffer */
//			select = 2;
//			process_poll(&xmem_write_process);
//		} else if(data_pointer == PAGE_SIZE) {
//			/* if data_pointer exceeds half of buffer size -> send first half */
//			select = 3;
//			process_poll(&xmem_write_process);
//		}
//	} else {
//		dac_setValue((uint16_t)audio_buf[data_pointer], Z1_DAC_0);
//
//		if(data_pointer == 0) {
//			select = 0;
//			process_poll(&xmem_read_process);
//		} else if(data_pointer == PAGE_SIZE) {
//			select = 1;
//			process_poll(&xmem_read_process);
//		}
//	}

	data_pointer++;
	P6OUT = ((data_pointer & 0x0001)<<2);
	if(data_pointer == 512) data_pointer = 0;
	isr_cnt++;
	PHIDGET_CLEAR_IRQ_FLAG(); /* Clear Interrupt flag */
}
/*---------------------------------------------------------------------------*/
