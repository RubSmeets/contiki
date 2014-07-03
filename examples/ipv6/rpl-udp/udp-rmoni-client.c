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

/* ------------------------------------- */
/* Rmoni ASCII commands				     */
/* ------------------------------------- */
typedef enum {
	GET 			= 0x00,
	SET 			= 0x01,
	RES				= 0x02,
	RESET			= 0x03,
	DEFAULT			= 0x04,
	PING			= 0x05,
	PONG			= 0x06,
	GENERAL_INFO	= 0x07,
	NETWORK			= 0x08,
} rmoni_command_type_t;

/* ------------------------------------- */
/* Rmoni Measurement keys			     */
/* ------------------------------------- */
typedef enum {
	NTC_TEMP 			,
	HUMIDITY 			,
	CURRENT				,
	RESISTOR			,
	VOLTAGE				,
	CO2					,
	PRESSURE			,
	THERMOCOUPLE		,
	PARTICLE			,
	PRESSURE_kPA		,
	DISTANCE			,
	SOIL_MOISTURE		,
	LEAF_WETNESS		,
	PT100				,
	CONTACT				,
	ENERGY				,
	POWER				,
	UNITS				,
	UNITS_PER_HOUR		,
	VOLUME				,
	FLOW				,
	DUMMY				,
	RELAYS				,
	DIGITAL_IO			,
} rmoni_keys_type_t;

/* ------------------------------------- */
/* Rmoni Sensor pins				     */
/* ------------------------------------- */
typedef enum {
	PIN1	 			,
	PIN2	 			,
	PIN4				,
	PIN8				,
	PIN10				,
} rmoni_pin_type_t;

/* ------------------------------------- */
/* Rmoni sensor device					 */
/* ------------------------------------- */
struct rmoni_device {
	char		mac_address[16];
	char		version[17];
	uint8_t		initialised;
};

static const char * rmoni_commands[9] = {
	"RM^GET", "RM^SET", "RM^RES", "RM^RESET\r", "RM^DEFAULT\r", "RM^PING\r", "RM^PONG\r", "RM^N\r", "RM^NET"
};

static const char * rmoni_keys[24] = {
	"10", "11", "13", "14", "15", "16", "17", "19", "1C", "1D",
	"20", "21", "22", "23", "25", "26", "27", "28", "29", "2A",
	"2B", "33", "3F", "41"
};

static const char * rmoni_pins[10] = {
	"01", "02", "04", "08", "10", "20", "00", "00", "00", "00"
};

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
#define PERIOD 20
#endif

#define START_INTERVAL		(15 * CLOCK_SECOND)
#define SEND_INTERVAL		(PERIOD * CLOCK_SECOND)
#define SEND_TIME		(random_rand() % (SEND_INTERVAL))
#define MAX_PAYLOAD_LEN		40

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;
static struct rmoni_device rmoni_module;

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&udp_client_process);
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  char *str;
  uint8_t len;

  if(uip_newdata()) {
	  str = (char *)uip_appdata;
	  len = uip_datalen() & 0xff;
  }
}
/*---------------------------------------------------------------------------*/
static void
send_packet(void *ptr)
{
	uint8_t i;
	char buf[MAX_PAYLOAD_LEN];

	/* Request temperature */
	memcpy(buf, rmoni_commands[GET], 6);
	buf[6] = ',';
	memcpy(&buf[7], &rmoni_module.mac_address[0], 16);
	buf[23] = ',';
	memcpy(&buf[24], rmoni_keys[NTC_TEMP], 2);
	memcpy(&buf[26], rmoni_pins[PIN2], 2);
	buf[28] = '\r';

	for(i=0; i<29; i++) PRINTF("%c", buf[i]); PRINTF("\n");

    uip_udp_packet_sendto(client_conn, &buf, 29,
                            &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
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
PROCESS_THREAD(udp_client_process, ev, data)
{
  static struct etimer periodic;
  static struct ctimer backoff_timer;
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
  PRINTF(" local/remote port %u/%u\n", UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));

#if WITH_COMPOWER
  powertrace_sniff(POWERTRACE_ON);
#endif

  sprintf(&rmoni_module.mac_address[0], "524D4F0000A05F2D");

  etimer_set(&periodic, SEND_INTERVAL);
  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
    else if (ev == sensors_event && data == &button_sensor) {
      send_packet(NULL);
    }

    if(etimer_expired(&periodic)) {
      etimer_reset(&periodic);
      ctimer_set(&backoff_timer, SEND_TIME, send_packet, NULL);

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
