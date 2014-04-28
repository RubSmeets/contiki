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

#define MEASURE_ENERGY 0
#define MEASURE_TIME   0

#if MEASURE_ENERGY
#include "sys/energest.h"
#include "sys/rtimer.h"
#endif

//#include "contiki-conf.h"
#ifdef WITH_COMPOWER
#include "powertrace.h"
#endif
#include <stdio.h>
#include <string.h>

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

#define UDP_EXAMPLE_ID  190

#define DEBUG DEBUG_NONE //DEBUG_PRINT
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
static uip_ipaddr_t ipaddr_edge;

uint8_t temp_sec_device_list[66];

static uint8_t push_cntr = 0;

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&udp_client_process);
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  char *str;
  uint8_t len, result;

  if(uip_newdata()) {
	  str = (char *)uip_appdata;
	  len = uip_datalen() & 0xff;

//#if ENABLE_CCM_APPLICATION & SEC_CLIENT
//	  result = keymanagement_decrypt_packet(&UIP_IP_BUF->srcipaddr, (uint8_t *)str, &len, 0, 0);
//#endif
//
//	  if(str[3] == 'H') {
//		  P6OUT ^= 0x01;
//	  }
//
//	  PRINTF("DATA recv '"); for(result=3; result<(len-8); result++) {PRINTF("%c", str[result]);} PRINTF("'\n");
  }
}
/*---------------------------------------------------------------------------*/
static void
send_packet(void *ptr)
{
	uint16_t msg_cnt = 44;
	uint8_t nonce_cnt = 2;
	uint8_t data_len = 1;
	char buf[1];

	//sprintf(&buf[0], "Hello dit is een test bericht van ongeveer 100 bytes lang met associated data van 20 bytes lang dddd en nog eens 19 er ");
	sprintf(&buf[0], "H");

	//cc2420_encrypt_ccm((uint8_t *)buf, &server_ipaddr.u8[0], &msg_cnt, &nonce_cnt, &data_len, 50);
	//cc2420_decrypt_ccm((uint8_t *)buf, &server_ipaddr.u8[0], &msg_cnt, &nonce_cnt, &data_len, 0);

#if MEASURE_ENERGY
	rtimer_clock_t t1, t2;
	uint8_t i;
	rtimer_clock_t tbuf[50];
	uint8_t index = 0;
	uint8_t count[5] = {0, 0, 0, 0 ,0};
#define FINE_STEP 636 /* nano seconds -> 48 times write 2-byte to variable in 1/32768Hz interval, gives 1/(32768Hz*48) */
#define NORMAL_STEP 30518 /* nano seconds -> 1/32768Hz */

	uint32_t difference = 0;
	uint32_t normalTime = 0;
	uint32_t fineTime = 0;
	uint32_t totalTime = 0;

	/* Energy measurement variables*/
	struct energy_time {
		unsigned short source;
		long cpu;
		long lpm;
		long transmit;
		long listen;
	};

	static struct energy_time diff;
	static struct energy_time last;
	/*************************/

	/* update all counters */
	energest_flush();

	last.cpu = energest_type_time(ENERGEST_TYPE_CPU);
	last.lpm = energest_type_time(ENERGEST_TYPE_LPM);
	last.transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT);
	last.listen = energest_type_time(ENERGEST_TYPE_LISTEN);
	t1=RTIMER_NOW();

	/************** Start what we want to measure ********************/
	//radio->on();
	/* Encrypt message */
	keymanagement_decrypt_packet(&server_ipaddr, (uint8_t *) buf, &data_len, 47, 0);

	/************** Finish what we want to measure ********************/


	tbuf[0] = TAR;
	tbuf[1] = TAR;
	tbuf[2] = TAR;
	tbuf[3] = TAR;
	tbuf[4] = TAR;
	tbuf[5] = TAR;
	tbuf[6] = TAR;
	tbuf[7] = TAR;
	tbuf[8] = TAR;
	tbuf[9] = TAR;
	tbuf[10] = TAR;
	tbuf[11] = TAR;
	tbuf[12] = TAR;
	tbuf[13] = TAR;
	tbuf[14] = TAR;
	tbuf[15] = TAR;
	tbuf[16] = TAR;
	tbuf[17] = TAR;
	tbuf[18] = TAR;
	tbuf[19] = TAR;
	tbuf[20] = TAR;
	tbuf[21] = TAR;
	tbuf[22] = TAR;
	tbuf[23] = TAR;
	tbuf[24] = TAR;
	tbuf[25] = TAR;
	tbuf[26] = TAR;
	tbuf[27] = TAR;
	tbuf[28] = TAR;
	tbuf[29] = TAR;
	tbuf[30] = TAR;
	tbuf[31] = TAR;
	tbuf[32] = TAR;
	tbuf[33] = TAR;
	tbuf[34] = TAR;
	tbuf[35] = TAR;
	tbuf[36] = TAR;
	tbuf[37] = TAR;
	tbuf[38] = TAR;
	tbuf[39] = TAR;
	tbuf[40] = TAR;
	tbuf[41] = TAR;
	tbuf[42] = TAR;
	tbuf[43] = TAR;
	tbuf[44] = TAR;
	tbuf[45] = TAR;
	tbuf[46] = TAR;
	tbuf[47] = TAR;
	tbuf[48] = TAR;
	tbuf[49] = TAR;

	t2=RTIMER_NOW();

	diff.cpu = energest_type_time(ENERGEST_TYPE_CPU) - last.cpu;
	diff.lpm = energest_type_time(ENERGEST_TYPE_LPM) - last.lpm;
	diff.transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT) - last.transmit;
	diff.listen = energest_type_time(ENERGEST_TYPE_LISTEN) - last.listen;

	PRINTF("CPU=%lu, LPM=%lu, TRANSMIT=%lu, LISTEN=%lu, TICKS=%u\n", diff.cpu, diff.lpm, diff.transmit, diff.listen, t2-t1);


	PRINTF("t1 time: %u\n", t1);
	for(i=0; i<(50-1); i++) {
		if(tbuf[i] == tbuf[i+1]) {
			count[index]++;
		} else {
			index++;
		}
	}

	difference = (tbuf[0]-1) - t1;
	normalTime = difference*NORMAL_STEP;
	fineTime = FINE_STEP*(48 - count[0]);
	totalTime = normalTime + fineTime;

	PRINTF("Time in nano seconds: %lu\n", totalTime);

	PRINTF("counts: ");
	for(i=0; i<5; i++) {
		PRINTF("%d ", count[i]);
		count[i] = 0;
	}
	PRINTF("\n");

#endif


/*  static int seq_id;
  char buf[MAX_PAYLOAD_LEN];
  uint8_t data_ptr = 0;
  uint8_t data_len = 26;

  seq_id++;
  PRINTF("DATA send to %d 'Hello %d'\n",
         server_ipaddr.u8[sizeof(server_ipaddr.u8) - 1], seq_id);
  //sprintf(&buf[data_ptr], "Hello %d from the", seq_id);
  sprintf(&buf[data_ptr], "XXXXXXXXOOOOOOOOXXXXXXXXOOOOOOOO%d", seq_id);

  	uint8_t i;
    PRINTF("before: %02x\n", buf[0]);

    PRINTF("addr: ");
    for(i=0; i<16; i++) PRINTF("%.2x",server_ipaddr.u8[i]);
    PRINTF("\n");

#if ENABLE_CCM_APPLICATION & SEC_CLIENT
    data_ptr = keymanagement_send_encrypted_packet(client_conn, (uint8_t *)buf, &data_len, 0, &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
    //PRINTF("result: %d\n", data_ptr);
#else
    uip_udp_packet_sendto(client_conn, &buf[data_ptr], 26,
                            &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
#endif
*/
//
//  result = cc2420_encrypt_ccm((uint8_t *)buf, 24);
//  if(!result) PRINTF("Encryption failed: busy!\n");
//  else {
//	  //PRINTF("result: ");
//	  //for(i=0; i<35; i++) PRINTF("%.2x",(uint8_t)buf[i]);
//	  //PRINTF("\n");
//  }
//
//  uint32_t test = 0;
//  uint8_t test2 = 2;
//  result = cc2420_decrypt_ccm((uint8_t *)buf, &test, &test2, 34);
//  if(!result) PRINTF("Encryption failed: busy!\n");
//  else {
//	  //PRINTF("result: ");
//	  //for(i=0; i<35; i++) PRINTF("%.2x",(uint8_t)buf[i]);
//	  //PRINTF("\n");
//  }
//  uip_udp_packet_sendto(client_conn, &buf[data_ptr], 26,
//                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
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
//  client_conn = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL); //Server
  if(client_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT));
//  udp_bind(client_conn, UIP_HTONS(UDP_SERVER_PORT));	//Server

  PRINTF("Created a connection with the server ");
  PRINT6ADDR(&client_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
	UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));

#if WITH_COMPOWER
  powertrace_sniff(POWERTRACE_ON);
#endif

  /* Set output */
//  P6DIR |= 0x01; //Server

  etimer_set(&periodic, SEND_INTERVAL);
  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
    else if (ev == sensors_event && data == &button_sensor) {
    	uint8_t data_ptr = 0;
    	char buf[100];
    	uint8_t data_len = 17;

    	/* Increment push counter */
    	push_cntr++;
    	if(push_cntr == 10) {
    		PRINTF("Erase keys\n");
    		xmem_erase(XMEM_ERASE_UNIT_SIZE, MAC_SECURITY_DATA);
    	} else {
    		PRINTF("Push count: %d\n", push_cntr);
    	}

    	sprintf(&buf[data_ptr], "Hello from node 4");
    	uint8_t i;
    	PRINTF("Text:  "); for(i=0; i<data_len; i++) PRINTF("%c", buf[i]); PRINTF("\n");
    	PRINTF("Plain:  "); for(i=0; i<data_len; i++) PRINTF("%02x", buf[i]); PRINTF("\n");

#if ENABLE_CCM_APPLICATION & SEC_CLIENT
    	send_packet(NULL);
    	//data_ptr = keymanagement_send_encrypted_packet(client_conn, (uint8_t *)buf, &data_len, 0, &server_ipaddr, UIP_HTONS(UDP_CLIENT_PORT)); // Server
    	//data_ptr = keymanagement_send_encrypted_packet(client_conn, (uint8_t *)buf, &data_len, 0, &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
    	//uip_udp_packet_sendto(client_conn, &buf[data_ptr], 26, &ipaddr_edge, UIP_HTONS(5444));
#else
    	uip_udp_packet_sendto(client_conn, &buf[data_ptr], 26, &server_ipaddr, UIP_HTONS(UDP_CLIENT_PORT));
#endif
    }
    
    if(etimer_expired(&periodic)) {
      etimer_reset(&periodic);
      ctimer_set(&backoff_timer, SEND_TIME, send_packet, NULL);

      /* Reset push counter */
      push_cntr = 0;

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
