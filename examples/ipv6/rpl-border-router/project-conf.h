/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
 * All rights reserved.
 *
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
 */

#ifndef PROJECT_ROUTER_CONF_H_
#define PROJECT_ROUTER_CONF_H_

#ifndef UIP_FALLBACK_INTERFACE
#define UIP_FALLBACK_INTERFACE rpl_interface
#endif

/* Additional configuration for higher throughput */
#ifdef NETSTACK_CONF_RDC
#undef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC 				 nullrdc_driver	/* Always keep transceiver on without phase optimization or retransmit after collisions */
#else
#define NETSTACK_CONF_RDC				 nullrdc_driver
#endif

#ifdef NETSTACK_RDC_CHANNEL_CHECK_RATE
#undef NETSTACK_RDC_CHANNEL_CHECK_RATE
#define NETSTACK_RDC_CHANNEL_CHECK_RATE  64
#else
#define NETSTACK_RDC_CHANNEL_CHECK_RATE	 64
#endif

#ifdef UIP_CONF_BUFFER_SIZE
#undef UIP_CONF_BUFFER_SIZE
#define UIP_CONF_BUFFER_SIZE     		 200			/* Allow the border-router to send fragment bigger packets (up to 160 bytes needed) */
#else
#define UIP_CONF_BUFFER_SIZE	 	     200 //140
#endif

#ifdef QUEUEBUF_CONF_NUM
#undef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM          		 5	//4
#else
#define QUEUEBUF_CONF_NUM	 	     	 5
#endif

#ifndef UIP_CONF_RECEIVE_WINDOW
#define UIP_CONF_RECEIVE_WINDOW  60
#endif

#ifndef WEBSERVER_CONF_CFS_CONNS
#define WEBSERVER_CONF_CFS_CONNS 2
#endif

#endif /* PROJECT_ROUTER_CONF_H_ */
