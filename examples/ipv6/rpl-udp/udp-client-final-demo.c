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
#include "net/uip.h"
#include "net/uip-ds6.h"
#include "net/uip-udp-packet.h"
#include "dev/button-sensor.h"
#include "cc2420-aes.h"
#include "dev/cc2420.h"
#include "symm-key-client-v1.h"
#include "dev/xmem.h"
#include "net/rime/rimeaddr.h"
#include "dev/adxl345.h"

#include <stdio.h>
#include <string.h>

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

#define DEBUG DEBUG_PRINT //DEBUG_PRINT
#include "net/uip-debug.h"

#ifndef PERIOD
#define PERIOD 20
#endif

#define ACCM_READ_INTERVAL  CLOCK_SECOND
#define SEND_INTERVAL		(PERIOD * CLOCK_SECOND)
#define MAX_PAYLOAD_LEN		40

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;
static uip_ipaddr_t ipaddr_udp_server_node;

uint8_t temp_sec_device_list[66];

static uint8_t push_cntr = 0;
static process_event_t sendMessage_event;

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
PROCESS(accel_process, "Test Accel process");
AUTOSTART_PROCESSES(&accel_process, &udp_client_process);
/*---------------------------------------------------------------------------*/
/* As several interrupts can be mapped to one interrupt pin, when interrupt
    strikes, the adxl345 interrupt source register is read. This function prints
    out which interrupts occurred. Note that this will include all interrupts,
    even those mapped to 'the other' pin, and those that will always signal even if
    not enabled (such as watermark). */

void __attribute__((__far__))
print_int(uint16_t reg){
#define ANNOYING_ALWAYS_THERE_ANYWAY_OUTPUT 0
#if ANNOYING_ALWAYS_THERE_ANYWAY_OUTPUT
  if(reg & ADXL345_INT_OVERRUN) {
    printf("Overrun ");
  }
  if(reg & ADXL345_INT_WATERMARK) {
    printf("Watermark ");
  }
  if(reg & ADXL345_INT_DATAREADY) {
    printf("DataReady ");
  }
#endif
  if(reg & ADXL345_INT_FREEFALL) {
	  PRINTF("Freefall ");
  }
  if(reg & ADXL345_INT_INACTIVITY) {
	  PRINTF("InActivity ");
  }
  if(reg & ADXL345_INT_ACTIVITY) {
	  PRINTF("Activity ");
  }
  if(reg & ADXL345_INT_DOUBLETAP) {
	  PRINTF("DoubleTap ");
  }
  if(reg & ADXL345_INT_TAP) {
	  PRINTF("Tap ");
  }
  PRINTF("\n");
}
/*---------------------------------------------------------------------------*/
/* accelerometer free fall detection callback */
void __attribute__((__far__))
accm_ff_cb(uint8_t reg){
  uint8_t data = 0x01;
  process_post(&udp_client_process, sendMessage_event, &data);
  print_int(reg);
}
/*---------------------------------------------------------------------------*/
/* accelerometer tap and double tap detection callback */

void __attribute__((__far__))
accm_tap_cb(uint8_t reg){
  uint8_t data = 0x02;
  process_post(&udp_client_process, sendMessage_event, &data);
  print_int(reg);
}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{

}
/*---------------------------------------------------------------------------*/
static void
send_packet(uint8_t event)
{

	char buf[40];
	uint8_t data_len = 23;
	uint8_t data_ptr;

	sprintf(&buf[0], "Pakje van %d met data: %d", rimeaddr_node_addr.u8[7], event);


#if ENABLE_CCM_APPLICATION & SEC_CLIENT
    data_ptr = keymanagement_send_encrypted_packet(client_conn, (uint8_t *)buf, &data_len, 0, &ipaddr_udp_server_node, UIP_HTONS(UDP_SERVER_PORT));
    PRINTF("result: %d\n", data_ptr);
#else
    uip_udp_packet_sendto(client_conn, &buf[data_ptr], 26,
                            &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
#endif

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
  uip_ip6addr(&server_ipaddr, 0x20ff, 1, 0, 0, 0, 0, 0, 1);
  uip_ip6addr(&ipaddr_udp_server_node, 0x20ff, 2, 0, 0, 0xc30c, 0, 0, 2); //udp server
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


  etimer_set(&periodic, SEND_INTERVAL);
  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
    else if (ev == sensors_event && data == &button_sensor) {
    	/* Increment push counter */
    	push_cntr++;
    	if(push_cntr == 10) {
    		PRINTF("Erase keys\n");
    		xmem_erase(XMEM_ERASE_UNIT_SIZE, MAC_SECURITY_DATA);
    	} else {
    		PRINTF("Push count: %d\n", push_cntr);
    	}
    } else if (ev == sendMessage_event) {
    	send_packet(*(uint8_t *)data);
    }

    if(etimer_expired(&periodic)) {
      etimer_reset(&periodic);

      /* Reset push counter */
      push_cntr = 0;

    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Main process, setups  */

static struct etimer et;

__attribute__((__far__))
PROCESS_THREAD(accel_process, ev, data) {
  PROCESS_BEGIN();
  {
    int16_t x, y, z;

    /* Register the event used for lighting up an LED when interrupt strikes. */
    sendMessage_event = process_alloc_event();

    /* Start and setup the accelerometer with default values, eg no interrupts enabled. */
    accm_init();

    /* Register the callback functions for each interrupt */
    ACCM_REGISTER_INT1_CB(accm_ff_cb);
    ACCM_REGISTER_INT2_CB(accm_tap_cb);

    /* Set what strikes the corresponding interrupts. Several interrupts per pin is
      possible. For the eight possible interrupts, see adxl345.h and adxl345 datasheet. */
    accm_set_irq(ADXL345_INT_FREEFALL, ADXL345_INT_TAP + ADXL345_INT_DOUBLETAP);

    while (1) {
	    x = accm_read_axis(X_AXIS);
	    y = accm_read_axis(Y_AXIS);
	    z = accm_read_axis(Z_AXIS);
	    //PRINTF("x: %d y: %d z: %d\n", x, y, z);

      etimer_set(&et, ACCM_READ_INTERVAL);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    }
  }
  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
