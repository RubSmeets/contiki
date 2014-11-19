/*
 * intercom_adpcm-conf.h
 *
 *  Created on: Oct 5, 2014
 *      Author: crea
 */

#ifndef INTERCOM_ADPCM_CONF_H_
#define INTERCOM_ADPCM_CONF_H_

#ifdef NETSTACK_RDC_CHANNEL_CHECK_RATE
#undef NETSTACK_RDC_CHANNEL_CHECK_RATE
#define NETSTACK_RDC_CHANNEL_CHECK_RATE  64
#else
#define NETSTACK_RDC_CHANNEL_CHECK_RATE  64
#endif

#ifdef UIP_CONF_BUFFER_SIZE
#undef UIP_CONF_BUFFER_SIZE
#define UIP_CONF_BUFFER_SIZE			 240 /* Allow larger packets to be received from udp (160 bytes payload) */
#else
#define UIP_CONF_BUFFER_SIZE			 240
#endif

#ifdef QUEUEBUF_CONF_NUM
#undef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM                5
#else
#define QUEUEBUF_CONF_NUM                5
#endif

#ifdef NETSTACK_CONF_RDC
#undef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC nullrdc_driver	/* Always keep transceiver on without phase optimization or retransmit after collisions */
#else
#define NETSTACK_CONF_RDC nullrdc_driver
#endif

#ifdef DIO_CONF_SPECIAL
#undef DIO_CONF_SPECIAL
#define DIO_CONF_SPECIAL 				1
#else
#define DIO_CONF_SPECIAL 				1
#endif

#endif /* INTERCOM_ADPCM_CONF_H_ */
