/*
 * measure-linklayer-conf.h
 *
 *  Created on: Nov 12, 2014
 *      Author: crea
 */

#ifndef MEASURE_LINKLAYER_CONF_H_
#define MEASURE_LINKLAYER_CONF_H_

#ifdef NETSTACK_CONF_NETWORK
#undef NETSTACK_CONF_NETWORK
#define NETSTACK_CONF_NETWORK nullnetwork_driver
#endif

#ifdef NETSTACK_CONF_RDC
#undef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC nullrdc_driver	/* Always keep transceiver on without phase optimization or retransmit after collisions */
#endif

#ifdef NETSTACK_RDC_CHANNEL_CHECK_RATE
#undef NETSTACK_RDC_CHANNEL_CHECK_RATE
#define NETSTACK_RDC_CHANNEL_CHECK_RATE  64
#endif

#ifdef QUEUEBUF_CONF_NUM
#undef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM                5
#endif

#endif /* MEASURE_LINKLAYER_CONF_H_ */
