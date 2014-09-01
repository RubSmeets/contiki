/*
 * measure-cc2420-conf.h
 *
 *  Created on: Aug 27, 2014
 *      Author: crea
 */

#ifndef MEASURE_CC2420_CONF_H_
#define MEASURE_CC2420_CONF_H_

#define DEBUG_SEC							 0 		/* Debug ON */

#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE 16		/* Max channel check rate */
#define CC2420_CONF_AUTOACK              	 0		/* Address recognition */

#define NETSTACK_CONF_RDC nullrdc_noframer_driver	/* Always keep transceiver on without phase optimization or retransmit after collisions */
#define NETSTACK_CONF_MAC nullmac_driver			/* no mac layer to handle retransmits */


#endif /* MEASURE_CC2420_CONF_H_ */
