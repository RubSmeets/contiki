/*
 * adpcm_test.c
 *
 *  Created on: Nov 3, 2014
 *      Author: crea
 */
#include "contiki.h"
#include "isr_compat.h"
#include <string.h>

/* Communication includes */
#include "net/uip.h"
#include "net/uip-ds6.h"
#include "net/uip-udp-packet.h"

/* Include for peripheral */
#include "dev/leds.h"
#include "dev/cc2420.h"
#include "dev/button-sensor.h"
#include "dev/hwconf.h"

/* Audio includes */
#include "dev/z1-dac.h"
#include "dev/z1-phidgets.h"
#include "ADPCM_codec.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define SEND_INTERVAL		(CLOCK_SECOND)

/*---------------------------------------------------------------------------*/
PROCESS(test_process, "Test ADPCM");
AUTOSTART_PROCESSES(&test_process);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(test_process, ev, data)
{
	uint8_t i, encoded;

	PROCESS_BEGIN();

	PRINTF("Starting test process\n");

	ADPCM_init();

	int data[20] = {0x0000, 0xffc8, 0xff88, 0xff0c, 0xfefc, 0xfebc, 0xfe74, 0xfe74, 0xfeec, 0xff2c,
						0xff88, 0xfff0, 0x0020, 0x0078, 0x00e4, 0x0124, 0x0144, 0x0104, 0x00f4, 0x0094};

	for(i = 0; i<20; i=i+2) {
		PRINTF("%04x, %d\n", data[i], i);
	  	encoded = (uint8_t)(ADPCM_Encoder(data[i])<<4);
	  	encoded |= (uint8_t)(ADPCM_Encoder(data[i+1]) & 0x0F);
	  	PRINTF("%04x\n", data[i+1]);
	  	PRINTF("C:%02x\n", encoded);
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
