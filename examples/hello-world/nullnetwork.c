/*
 * nullnetwork.c
 *
 *  Created on: Nov 12, 2014
 *      Author: crea
 */
#include "contiki.h"
#include "net/netstack.h"
#include "net/uip.h"
#include "net/rime.h"

#include <string.h>

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define MAX_PAYLOAD	80

static uint8_t* data_ptr;
static uint16_t length;
static uint16_t rec_count;

static void
input(void)
{
	uint16_t count;

	/* Get payload */
	data_ptr = (uint8_t *)packetbuf_dataptr();
	length = packetbuf_datalen();

	if(length == MAX_PAYLOAD) {
		rec_count++;
	}

	/* Get count number */
	count = (data_ptr[0] << 8) | (data_ptr[1] & 0xFF);
	if(count == 1) {
		PRINTF("Count was: %d\n", rec_count);
		rec_count = 1;
	}

}

void
sicslowpan_init(void)
{
	PRINTF("Init nullnetwork\n");
}

/*--------------------------------------------------------------------*/
const struct network_driver nullnetwork_driver = {
  "nullnetwork",
  sicslowpan_init,
  input
};
/*--------------------------------------------------------------------*/
