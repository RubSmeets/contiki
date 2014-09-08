/*
 * measure-cc2420.c
 *
 *  Created on: May 9, 2014
 *      Author: crea
 */
#include "contiki.h"
#include "dev/cc2420.h"
#include "dev/button-sensor.h"
#include "dev/hwconf.h"
#include "isr_compat.h"

/* Audio includes */
#include "dev/z1-phidgets.h"
#include "dev/z1-dac.h"
#include "ADPCM_codec.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define SEND_INTERVAL		(CLOCK_SECOND)
#define MAX_PAYLOAD_LEN		10

/* Audio data */
static uint16_t audio_buf_aligned[(MAX_PAYLOAD_LEN)];
static uint8_t *audio_buf = (uint8_t *)audio_buf_aligned;
static uint16_t sample_buf_aligned[(MAX_PAYLOAD_LEN)];
static uint8_t *sample_buf = (uint8_t *)sample_buf_aligned;
static uint8_t tempSample = 0;

static uint8_t mode = 0;
static uint8_t mem_pointer = 0;
static uint8_t sample_pointer = 0;


uint16_t counter;

/*---------------------------------------------------------------------------*/
PROCESS(measure_process, "measure_process for cc2420");
AUTOSTART_PROCESSES(&measure_process);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(measure_process, ev, data)
{
	static struct etimer periodic;

	PROCESS_BEGIN();

	PRINTF("Starting measure process\n");

	SENSORS_ACTIVATE(phidgets);
	dac_init(Z1_DAC_0);
	P6DIR |= (1 << 2);


	etimer_set(&periodic, SEND_INTERVAL);
	while(1) {
		PROCESS_YIELD();

		if(etimer_expired(&periodic)) {
		      etimer_reset(&periodic);
		      PRINTF("c:%d", counter);
		      counter = 0;
		} else if(ev == PROCESS_EVENT_POLL) {
			//PRINTF("Poll\n");
		}

	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
HWCONF_ADC_IRQ(PHIDGET, ADC7);
ISR(ADC12, adc_service_routine)
{
	//uint8_t ptr = data_pointer % (MAX_PAYLOAD_LEN*2);
	sample_buf[sample_pointer] = phidgets.value(PHIDGET3V_2);

	if (Mode == 0x00) {
		tempSample = ADPCM_Encoder(sample_buf[sample_pointer])<<4; /* ADPCM code (bit 7-4) */
	    Mode=0x01;
	} else {
		tempSample = tempSample + ADPCM_Encoder(phidgets.value(PHIDGET3V_2)); /* ADPCM code (bit 3-0) */
	    audio_buf[mem_pointer] = tempSample;
	    mem_pointer++;
	    Mode=0x00;
	}

	//audio_buf[ptr] = (phidgets.value(PHIDGET3V_2) >> 4) & 0xFF;
	//dac_setValue(audio_buf[ptr], Z1_DAC_0);

	if(sample_pointer == 39) {	/* Buffer full */
		process_poll(&measure_process);
	}

	sample_pointer++;
	counter++;

	PHIDGET_CLEAR_IRQ_FLAG(); /* Clear Interrupt flag */
}
/*---------------------------------------------------------------------------*/
