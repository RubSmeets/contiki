/*
 * intercom_adpcm-conf.h
 *
 *  Created on: Oct 5, 2014
 *      Author: crea
 */

#ifndef INTERCOM_ADPCM_CONF_H_
#define INTERCOM_ADPCM_CONF_H_

#define NETSTACK_RDC_CHANNEL_CHECK_RATE  16

#define QUEUEBUF_CONF_NUM                6

#define NETSTACK_CONF_RDC nullrdc_driver	/* Always keep transceiver on without phase optimization or retransmit after collisions */

#endif /* INTERCOM_ADPCM_CONF_H_ */
