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
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/rpl/rpl.h"
#include "isr_compat.h"

#include "net/netstack.h"
#include "dev/button-sensor.h"
#include "dev/hwconf.h"
#include "dev/leds.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "dev/z1-dac.h"
#include "dev/z1-phidgets.h"
#include "ADPCM_codec.h"

#define MAX_PAYLOAD_LEN		40

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_CLIENT_PORT	8765
#define UDP_SERVER_PORT	5678

static struct uip_udp_conn *server_conn;

static uint8_t write_ptr;
static uint8_t read_ptr;
static uint8_t mode;
static uint16_t audio_buf_aligned[(MAX_PAYLOAD_LEN/2)*3];
static uint8_t *audio_buf = (uint8_t *)audio_buf_aligned;
static uint16_t decodedValue;

PROCESS(udp_server_process, "UDP server process");
AUTOSTART_PROCESSES(&udp_server_process);

HWCONF_ADC_IRQ(PHIDGET, ADC7);
HWCONF_TB_IRQ(TIMERB, 1);

/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
	char *appdata;
	uint8_t i, len;

	if(uip_newdata()) {
		appdata = (char *)uip_appdata;
		len = uip_datalen();

		if(len == 5) {
			leds_on(LEDS_BLUE);

			/* Prep. ADC variables */
			write_ptr = 0;
			read_ptr = 0;
			mode = 0x01;
			decodedValue = 0;

			/* Enable Compare interrupt */
			//TIMERB_ENABLE_IRQ();

		} else if(len == 4) {
			/* Turn off ADC */
			TIMERB_DISABLE_IRQ();

			leds_off(LEDS_BLUE);
		} else {
			if(TIMERB_IRQ_ENABLED() == 0) {
				TIMERB_ENABLE_IRQ();
			}

			memcpy(&audio_buf[write_ptr*MAX_PAYLOAD_LEN], appdata, len);
			if(write_ptr == 2) 	write_ptr = 0;
			else				write_ptr++;
		}
	}
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Server IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(state == ADDR_TENTATIVE || state == ADDR_PREFERRED) {
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
PROCESS_THREAD(udp_server_process, ev, data)
{
  uip_ipaddr_t ipaddr;
  struct uip_ds6_addr *root_if;

  PROCESS_BEGIN();

  PROCESS_PAUSE();

  SENSORS_ACTIVATE(button_sensor);

  /* Initialise REF voltage (enough time to stabilize */
  dac_init(Z1_DAC_0);
  init_timerB();

  PRINTF("UDP server started\n");

#if UIP_CONF_ROUTER
/* The choice of server address determines its 6LoPAN header compression.
 * Obviously the choice made here must also be selected in udp-client.c.
 *
 * For correct Wireshark decoding using a sniffer, add the /64 prefix to the 6LowPAN protocol preferences,
 * e.g. set Context 0 to aaaa::.  At present Wireshark copies Context/128 and then overwrites it.
 * (Setting Context 0 to aaaa::1111:2222:3333:4444 will report a 16 bit compressed address of aaaa::1111:22ff:fe33:xxxx)
 * Note Wireshark's IPCMV6 checksum verification depends on the correct uncompressed addresses.
 */

#if 0
/* Mode 1 - 64 bits inline */
   uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);
#elif 1
/* Mode 2 - 16 bits inline */
  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0x00ff, 0xfe00, 1);
  //uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0xc30c, 0, 0, 4);
  //uip_ip6addr(&ipaddr, 0x20ff, 2, 0, 0, 0xc30c, 0, 0, 4);
#else
/* Mode 3 - derived from link local (MAC) address */
  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
#endif

  uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL);
  root_if = uip_ds6_addr_lookup(&ipaddr);
  if(root_if != NULL) {
    rpl_dag_t *dag;
    dag = rpl_set_root(RPL_DEFAULT_INSTANCE,(uip_ip6addr_t *)&ipaddr);
    uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
    rpl_set_prefix(dag, &ipaddr, 64);
    PRINTF("created a new RPL dag\n");
  } else {
    PRINTF("failed to create a new RPL DAG\n");
  }
#endif /* UIP_CONF_ROUTER */

  print_local_addresses();

  /* The data sink runs with a 100% duty cycle in order to ensure high
     packet reception rates. */
  NETSTACK_MAC.off(1);

  server_conn = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL);
  if(server_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));

  PRINTF("Created a server connection with remote address ");
  PRINT6ADDR(&server_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n", UIP_HTONS(server_conn->lport),
         UIP_HTONS(server_conn->rport));

  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    } else if (ev == sensors_event && data == &button_sensor) {
      PRINTF("Initiaing global repair\n");
      rpl_repair_root(RPL_DEFAULT_INSTANCE);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
ISR(TIMERB1, timerb1_service_routine)
{
	switch(mode) {
		case 0x01: /* playback bit (7-4) */
			dac_setValue(decodedValue, Z1_DAC_0);

			decodedValue = ADPCM_Decoder((audio_buf[read_ptr]>>4) & 0x0F);
							 /* ADPCM_Decoder() execution time varies depending on the
								ADPCM code. This is the reason why the value is stored in
								a variable and during the next ISR call the DAC register
								is loaded. */
			mode=0x02;
			break;
		default: /* playback bit (3-0) */
			dac_setValue(decodedValue, Z1_DAC_0);

			decodedValue = ADPCM_Decoder(audio_buf[read_ptr] & 0x0F);
							 /* ADPCM_Decoder() execution time varies depending on the
								ADPCM code. This is the reason why the value is stored in
								a variable and during the next ISR call the DAC register
								is loaded. */

			read_ptr++;
			mode=0x01;

			if (read_ptr >= (MAX_PAYLOAD_LEN*3))
			{
				read_ptr = 0;
			}
			break;
	}

	TIMERB_CLEAR_IRQ_FLAG(); /* Clear Interrupt flag */
}