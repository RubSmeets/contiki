/*
 * aes128_z1_test.c
 *
 *  Created on: Nov 25, 2014
 *      Author: crea
 */

#include "contiki.h"
#include "dev/cc2420.h"
#include "dev/button-sensor.h"
#include "measureSpeed.h"

#include <string.h>

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...)
#define PRINTFDEBUG(...)
#else
#define PRINTF(...)
#define PRINTFDEBUG(...)
#endif

#define SEND_INTERVAL		(CLOCK_SECOND/6)
#define MAX_PAYLOAD_LEN		40
#define NUMB_MEASUREMENTS	80

#define NUMB_VECTORS		4

typedef struct {
	uint8_t address[16];
	uint8_t adata[30];
	uint8_t adata_len;
	uint8_t mdata[80];
	uint8_t mdata_len;
	uint16_t msg_cnt;
	uint8_t nonce_cnt;
} sec_data_t;

static const uint8_t keys128[][16] = {
	 { 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	 { 0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
	   0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf }
	 };

static const sec_data_t vectors[] = {
	{
		{0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
		 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01}, /* address */
		{0xff, 0xff, 0xff, 0xff}, /* adata */
		4, /* adata_len */
		{0x14, 0xaa, 0xbb, 0x00, 0x00, 0x01, 0x02, 0x03,
		 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
		 0x0c, 0x0d, 0x0e, 0x0f }, /* mdata */
		30, /* mdata_len */
		4, /* msg_cnt */
		1 /* nonce_cnt */
	}, {
		{0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
		 0x01, 0x01, 0x01, 0x01, 0xaa, 0xff, 0x04, 0x01}, /* address */
		{0xff, 0xff, 0xff}, /* adata */
		3, /* adata_len */
		{0x14, 0xaa, 0xbb, 0x00, 0x00, 0x01, 0x02, 0x03,
		 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
		 0x0c, 0x0d, 0x0e, 0x0f, 0x14, 0xaa, 0xbb, 0x00,
		 0x0c, 0x0d, 0x0e, 0x0f, 0x14, 0xaa, 0xbb, 0x00}, /* mdata */
		32, /* mdata_len */
		66, /* msg_cnt */
		15 /* nonce_cnt */
	}, {
		{0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
		 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01}, /* address */
		{0xff, 0xff}, /* adata */
		20, /* adata_len */
		{0x14, 0xaa, 0xbb, 0x00, 0x00, 0x01, 0x02, 0x03,
		 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
		 0x0c, 0x0d, 0x0e, 0x0f }, /* mdata */
		80, /* mdata_len */
		4, /* msg_cnt */
		1 /* nonce_cnt */
	}, {
		{0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
		 0x01, 0x01, 0x01, 0xaf, 0xf1, 0x0f, 0x01, 0x01}, /* address */
		{0xff, 0xff}, /* adata */
		10, /* adata_len */
		{0x14, 0xaa, 0xbb, 0x00, 0x00, 0x01, 0x02, 0x03,
		 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
		 0x0c, 0x0d, 0x0e, 0x0f }, /* mdata */
		100, /* mdata_len */
		46, /* msg_cnt */
		123 /* nonce_cnt */
	}
};

static uint16_t buf_aligned[64];
static uint8_t *buf = (uint8_t *)buf_aligned;

static uint32_t encrypt_results[NUMB_MEASUREMENTS];
static uint32_t decrypt_results[NUMB_MEASUREMENTS];
static uint8_t encrypt_index;
static uint8_t decrypt_index;
static uint8_t vector_index;

static uint8_t measure_cnt;
/*---------------------------------------------------------------------------*/
PROCESS(measure_process, "measure_process for cc2420");
AUTOSTART_PROCESSES(&measure_process);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
static void
aes_ccm_encrypt_packet(void) {
	uint8_t tot_len, i;

	PRINTFDEBUG("Encrypt: ");

	/* Assemble packet */
	memcpy(&buf[0], &vectors[vector_index].adata[0], vectors[vector_index].adata_len);
	memcpy(&buf[vectors[vector_index].adata_len], &vectors[vector_index].mdata[0], vectors[vector_index].mdata_len);

	tot_len = vectors[vector_index].adata_len + vectors[vector_index].mdata_len;
	PRINTFDEBUG("Encrypting packet of length: %d\n", tot_len);

	measurement_start();
	cc2420_encrypt_ccm(buf, vectors[vector_index].address, &vectors[vector_index].msg_cnt, &vectors[vector_index].nonce_cnt, &tot_len, vectors[vector_index].adata_len);
	encrypt_results[encrypt_index++] = measurement_stop();

	PRINTFDEBUG("enc_data: "); for(i=0; i<tot_len; i++) {PRINTFDEBUG("%02x", buf[i]);} PRINTFDEBUG("\n");

}
/*---------------------------------------------------------------------------*/
static void
aes_ccm_decrypt_packet(void) {
	uint8_t tot_len, i;

	PRINTFDEBUG("Decrypt: ");

	tot_len = vectors[vector_index].adata_len + vectors[vector_index].mdata_len + 8;

	PRINTFDEBUG("Decrypting packet of length: %d\n", tot_len);

	measurement_start();
	cc2420_decrypt_ccm(&buf[1], vectors[vector_index].address, &vectors[vector_index].msg_cnt, &vectors[vector_index].nonce_cnt, &tot_len, vectors[vector_index].adata_len);
	decrypt_results[decrypt_index++] = measurement_stop();

	PRINTFDEBUG("dec_data: "); for(i=0; i<tot_len; i++) {PRINTFDEBUG("%02x", buf[i+1]);} PRINTFDEBUG("\n");
	if(buf[tot_len+1] == 0x00) {
		PRINTFDEBUG(" OK\n");
	} else {
		PRINTFDEBUG(" NOT OK\n");
	}

}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(measure_process, ev, data)
{
	static struct etimer periodic;
	static uint8_t i;
	static uint64_t sum_encrypt;
	static uint64_t sum_decrypt;
	static uint64_t res_encrypt;
	static uint64_t res_decrypt;

	PROCESS_BEGIN();

	PRINTF("++++++++++++++++++++++++++++++\n");
	PRINTF("+    Measure AES128 speed    +\n");
	PRINTF("++++++++++++++++++++++++++++++\n");
	PRINTF("+     Test z1 hardware       +\n");
	PRINTF("++++++++++++++++++++++++++++++\n\n");

	measure_cnt = 0;
	encrypt_index = 0;
	decrypt_index = 0;
	sum_encrypt = 0;
	sum_decrypt = 0;
	vector_index = 0;

	etimer_set(&periodic, SEND_INTERVAL);
	while(1) {
		PROCESS_YIELD();

		if(etimer_expired(&periodic)) {
			if(vector_index < NUMB_VECTORS) {
				etimer_reset(&periodic);
				if(measure_cnt < NUMB_MEASUREMENTS) {
					aes_ccm_encrypt_packet();
					aes_ccm_decrypt_packet();
					measure_cnt++;
				} else {
					PRINTF("\nencrypt:\n");
					for(i=0; i<NUMB_MEASUREMENTS; i++) {
						/* Calculate average */
						PRINTF("%lu\n", encrypt_results[i]);
						sum_encrypt += encrypt_results[i];
					}
					PRINTF("\ndecrypt:\n");
					for(i=0; i<NUMB_MEASUREMENTS; i++) {
						/* Calculate average */
						PRINTF("%lu\n", decrypt_results[i]);
						sum_decrypt += decrypt_results[i];
					}

					res_encrypt = sum_encrypt / NUMB_MEASUREMENTS;
					res_decrypt = sum_decrypt / NUMB_MEASUREMENTS;

					PRINTF("Measurement results for vector %d avg. encrypt: %llu ns, avg. decrypt: %llu ns\n", vector_index, res_encrypt, res_decrypt);

					/* Reset variables and restart measurements for the next vector */
					measure_cnt = 0;
					encrypt_index = 0;
					decrypt_index = 0;
					sum_encrypt = 0;
					sum_decrypt = 0;
					vector_index++;
				}
			}
		}

	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
