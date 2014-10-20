/*
 * transmit-audio-conf.h
 *
 *  Created on: Sep 9, 2014
 *      Author: crea
 */

#ifndef TRANSMIT_AUDIO_CONF_H_
#define TRANSMIT_AUDIO_CONF_H_

#define NETSTACK_RDC_CHANNEL_CHECK_RATE  16

#define QUEUEBUF_CONF_NUM                6

#define UIP_CONF_BUFFER_SIZE			 240 /* Allow larger packets to be received from udp (160 bytes payload) */

#define NETSTACK_CONF_RDC nullrdc_driver	/* Always keep transceiver on without phase optimization or retransmit after collisions */

#endif /* TRANSMIT_AUDIO_CONF_H_ */
