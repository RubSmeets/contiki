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
#include "net/uip.h"
#include "net/rpl/rpl.h"

#include "net/netstack.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "dev/uart1.h"
#include "dev/rmoni_serial_line.h"
#include "lib/ringbuf.h"

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"

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

#define ASCII_HEADER_SIZE	6
#define MAC_ADDRESS_SIZE	8
#define IPV6_ADDRESS_SIZE	16
#define SEPERATOR_CHAR(c) (c == 0x2C)

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_CLIENT_PORT	5678
#define UDP_SERVER_PORT	8765

#define UDP_EXAMPLE_ID  190

/* ------------------------------------- */
/* Rmoni sensor device					 */
/* ------------------------------------- */
struct rmoni_device {
	uint8_t		mac_address[MAC_ADDRESS_SIZE];
	char		version[17];
	uint8_t		initialised;
	uint8_t 	registered;
};

static struct uip_udp_conn *client_conn;
static struct rmoni_device rmoni_module;
static uip_ipaddr_t server_ipaddr;

static void rmoni_parse_message(char * msg);
static uint8_t* hex_decode(const char *in, size_t len, uint8_t *out);

PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&udp_client_process);
/*---------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* IMPORTANT: every command has to end with '\r' to avoid out-of-bound errors */
/*----------------------------------------------------------------------------*/
static void
rmoni_serial_output(const char *buf) {
	uint8_t i = 0;

	/* Start sending bytes until carriage return */
	while(buf[i] != '\r') {
		PRINTF("%c", buf[i]);
		uart1_writeb(buf[i]);
		i++;
	}
	/* Send carriage return to end command */
	uart1_writeb(buf[i]);
}
/*----------------------------------------------------------------------------*/
#define REGISTER_MSG_SIZE 25
/*----------------------------------------------------------------------------*/
static void
sendRegisterPacket(void) {
	uint8_t tempBuf[REGISTER_MSG_SIZE]; /* HDR(1) + Mac(8) + IP (16) */
	uip_ipaddr_t ipaddr;

	tempBuf[0] = 0x01;	/* Registering */
	memcpy(&tempBuf[1], &rmoni_module.mac_address[0], MAC_ADDRESS_SIZE);
	uip_ds6_select_src(&ipaddr, &server_ipaddr);
	memcpy(&tempBuf[9], &ipaddr.u8[0], IPV6_ADDRESS_SIZE);

	uip_udp_packet_sendto(client_conn, tempBuf, REGISTER_MSG_SIZE, &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));

	rmoni_module.registered = 1;
}
/*----------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  char *appdata;
  uint8_t i;

  if(uip_newdata()) {
	/* Store data */
    appdata = (char *)uip_appdata;
    appdata[uip_datalen()] = 0;
    PRINTF("Received: ");for(i=0; i<uip_datalen(); i++)PRINTF("%c",appdata[i]); PRINTF("\n");

    /* Store remote IP for reply */
    uip_ipaddr_copy(&client_conn->ripaddr, &UIP_IP_BUF->srcipaddr);

    if(!rmoni_module.registered) {
    	PRINTF("Device is registering\n");
    } else {
		/* Post command for Rmoni module (CHECK FOR VALID COMMAND!!!!!) */
		rmoni_serial_output(appdata);
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
set_global_address(void)
{
  uip_ipaddr_t ipaddr;

  //uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);	/* Default configuration */
  uip_ip6addr(&ipaddr, 0x20ff, 2, 0, 0, 0, 0, 0, 0);	/* Tunnel configuration */
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
  //uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL);

  /* Set server address (HACK to make server address final) */
  uip_ip6addr(&server_ipaddr, 0x20ff, 1, 0, 0, 0, 0, 0, 1);

}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  static struct etimer et;

  PROCESS_BEGIN();

  PROCESS_PAUSE();

  PRINTF("UDP rmoni relay node started\n");

  set_global_address();

  print_local_addresses();

  /* The data sink runs with a 100% duty cycle in order to ensure high
     packet reception rates. */
  NETSTACK_MAC.off(1);

  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL);
  if(client_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT));

  PRINTF("Created a client connection with remote address ");
  PRINT6ADDR(&client_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n", UIP_HTONS(client_conn->lport),
         UIP_HTONS(client_conn->rport));

  /* Initialize uart1 for rmoni module communication */
  uart1_init(9600);
  uart1_set_input(rmoni_serial_line_input);
  rmoni_serial_line_init();

  /* Set timer */
  etimer_set(&et, CLOCK_SECOND*4);

  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    } else if (ev == serial_line_rmoni_message) {
		/* Parse message and formulate reply */
		rmoni_parse_message((char *) data);
    } else if(etimer_expired(&et)) {
		/* Request general device information */
		if(!rmoni_module.initialised) {
			etimer_reset(&et);
			PRINTF("Request MAC\n");
			rmoni_serial_output(rmoni_commands[GENERAL_INFO]);
		} else if(!rmoni_module.registered) {
			etimer_set(&et, CLOCK_SECOND*60);
			PRINTF("Registering ...\n");
			sendRegisterPacket();
    	} else {
			etimer_reset(&et);
			PRINTF("I'm still alive\n");
			sendRegisterPacket();
		}
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#define INFO_MAC_OFFSET		11
#define INFO_VERSION_OFFSET 36
/*---------------------------------------------------------------------------*/
static void
rmoni_parse_message(char * msg) {
	uint8_t i = 0;

	PRINTF("Parsing message\n");

	/* Determine message size */
	while(msg[i] != '\0') {
		PRINTF("%c", msg[i]);
		i++;
	}
	PRINTF("\n");

	/* Parse packet */
	if(!rmoni_module.initialised) {
		if(memcmp(&msg[0], rmoni_commands[NETWORK], ASCII_HEADER_SIZE) == 0) {
			PRINTF("Init rmoni OK\n");
			/* Convert 2 ASCII bytes representing a byte in hex format to 1 byte hex */
			hex_decode(&msg[INFO_MAC_OFFSET], (MAC_ADDRESS_SIZE*2), &rmoni_module.mac_address[0]);
			memcpy(&rmoni_module.version[0], &msg[INFO_VERSION_OFFSET], 17);
			rmoni_module.initialised = 1;
		}
	} else {
		/* Reply with rmoni message */
		uip_udp_packet_send(client_conn, msg, (i-1));
		/* Clear remote ip address */
		uip_create_unspecified(&client_conn->ripaddr);
	}
}

/*---------------------------------------------------------------------------*/
/* --------------- UTILS ----------------*/
static uint8_t*
hex_decode(const char *in, size_t len, uint8_t *out)
{
        unsigned int i, t, hn, ln;

        for (t = 0,i = 0; i < len; i+=2,++t) {

                hn = in[i] > '9' ? in[i] - 'A' + 10 : in[i] - '0';
                ln = in[i+1] > '9' ? in[i+1] - 'A' + 10 : in[i+1] - '0';

                out[t] = (hn << 4 ) | ln;
        }

        return out;
}
/*---------------------------------------------------------------------------*/
