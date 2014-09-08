/*
 * Measure audio with compression (ADPCM)
 */

#include "contiki.h"
#include "isr_compat.h"
#include "dev/flash.h"
#include "dev/watchdog.h"

/* Include for peripheral */
#include "dev/leds.h"
#include "dev/cc2420.h"
#include "dev/button-sensor.h"
#include "dev/hwconf.h"

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
#define MAX_PAYLOAD_LEN		20

/* Audio data */
#define AUDIO_MEM_START	0x18000L
#define AUDIO_MEM_STOP	0x19FFFL
static uint8_t mode;
static uint8_t tempSample;
uint32_t flash_addr;
uint8_t *pMemory = (uint8_t *)AUDIO_MEM_START;;
uint16_t decodedValue;

uint16_t counter;

/*---------------------------------------------------------------------------*/
PROCESS(measure_process, "measure_process for cc2420");
AUTOSTART_PROCESSES(&measure_process);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
static void
delay(void) {
	uint16_t i;
	for (i = 0; i < 0x9FFF; i++) nop();
}
/*---------------------------------------------------------------------------*/
static void
start_record(void)
{
	/* Turn on green led to indicate erase */
	leds_on(LEDS_GREEN);

	/* Erase internal flash segments */
	_DINT();
	watchdog_stop();
	/* DCO(SMCLK) is 2,4576MHz, /6 = 409600 Hz
	     select SMCLK for flash timing, divider 5+1 */
	FCTL2 = 0xA5C5;

	/* Run through 512 byte segments (0x200L) and clear */
	for(flash_addr = AUDIO_MEM_START; flash_addr < AUDIO_MEM_STOP; flash_addr += 0x200L) {
		flash_clear((unsigned short *)flash_addr);
	}

	_EINT();
	watchdog_start();

	/* Turn of green and turn on red to indicate record */
	leds_off(LEDS_GREEN);
	leds_on(LEDS_RED);

	/* xxxx CARE for interrupts that can cause system crash during recordings (only writing to flashing during ADC interrupt) */
	FCTL3 = FWKEY;                   /* Unlock */
	FCTL1 = FWKEY | WRT;             /* Enable writing to the flash */

	/* Turn on ADC */
	mode = 0x00;
	SENSORS_ACTIVATE(phidgets);
}
/*---------------------------------------------------------------------------*/
static void
playback(void)
{
	mode = 0x02;
	decodedValue = 0x00;

	leds_on(LEDS_BLUE);

	/* Configure DAC and ADC12 REF voltage */
	dac_init(Z1_DAC_0);

	/* Should wait before voltages are stabilized */
	delay();   /* wait until voltages have stab. */
	delay();   /* wait until voltages have stab. */

	/* Create our sample frequency (ADC12 module -> not reading samples) */
	SENSORS_ACTIVATE(phidgets);

}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(measure_process, ev, data)
{
	static struct etimer periodic;
	static uint8_t toggle_record_play = 0;

	PROCESS_BEGIN();

	PRINTF("Starting measure process\n");

	SENSORS_ACTIVATE(button_sensor);

	etimer_set(&periodic, SEND_INTERVAL);
	while(1) {
		PROCESS_YIELD();

		if(etimer_expired(&periodic)) {
		      etimer_reset(&periodic);
		      //PRINTF("c:%d", counter);
		      counter = 0;
		} else if(ev == sensors_event && data == &button_sensor) {
			if(toggle_record_play == 0x00) {
				/* Start record */
				start_record();
				toggle_record_play = 0x01;
			} else {
				/* Play audio samples */
				playback();
				toggle_record_play = 0x00;
			}
		} else if(ev == PROCESS_EVENT_POLL) {
			PRINTF("Poll\n");
			pMemory = (uint8_t *)AUDIO_MEM_START;

			/* Exiting record mode */
			if(mode == 0x00) {
				FCTL1 = FWKEY;           /* Disable writing and erasing */
				FCTL3 = FWKEY | LOCK;    /* Lock the flash */

				leds_off(LEDS_RED);
			}
			/* Exiting playback mode */
			else {
				dac_disable(Z1_DAC_0);

				leds_off(LEDS_BLUE);
			}
//			PRINTF("Sample: "); for(i=0; i<(MAX_PAYLOAD_LEN*2); i++) PRINTF("%03x ", sample_buf_aligned[i]); PRINTF("\n");
//			PRINTF("ADPCM: "); for(i=0; i<(MAX_PAYLOAD_LEN); i++) PRINTF("%03x ", audio_buf[i]); PRINTF("\n");
//
//			/* decode ADPCM */
//			mode = 0x02;
//			j = 0;
//			for(i=0; i<(MAX_PAYLOAD_LEN*2); i++) {
//				if(mode == 0x02) {
//					sample_buf_aligned[i] = ADPCM_Decoder((audio_buf[j]>>4) & 0x0F);
//					mode = 0x00;
//				} else {
//					sample_buf_aligned[i] = ADPCM_Decoder(audio_buf[j] & 0x0F);
//					j++;
//					mode = 0x02;
//				}
//			}
//
//			PRINTF("decode: "); for(i=0; i<(MAX_PAYLOAD_LEN*2); i++) PRINTF("%03x ", sample_buf_aligned[i]); PRINTF("\n");
		}

	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
HWCONF_ADC_IRQ(PHIDGET, ADC7);
ISR(ADC12, adc_service_routine)
{
	switch(mode) {
		case 0x00:
			tempSample = ADPCM_Encoder(phidgets.value(PHIDGET3V_2))<<4; /* ADPCM code (bit 7-4) */
			mode=0x01;
			break;
		case 0x01:
			tempSample = tempSample + (ADPCM_Encoder(phidgets.value(PHIDGET3V_2)) & 0x0F); /* ADPCM code (bit 3-0) */
			*pMemory = tempSample;
			pMemory++;
			mode=0x00;

			if((uint32_t)pMemory >= (uint32_t)AUDIO_MEM_STOP)  /* memory full? */
			{
				SENSORS_DEACTIVATE(phidgets);	/* Deactivate ADC12 */
				process_poll(&measure_process);
			}
			break;
		case 0x02: /* playback bit (7-4) */
			dac_setValue(decodedValue, Z1_DAC_0);

			decodedValue = ADPCM_Decoder((*pMemory>>4) & 0x0F);
			                 /* ADPCM_Decoder() execution time varies depending on the
			                    ADPCM code. This is the reaosn why the value is stored in
			                    a variable and during the next ISR call the DAC register
			                    is loaded. */
			mode=0x04;
			break;
		default: /* playback bit (3-0) */
			dac_setValue(decodedValue, Z1_DAC_0);

			decodedValue = ADPCM_Decoder(*pMemory & 0x0F);
							 /* ADPCM_Decoder() execution time varies depending on the
							    ADPCM code. This is the reaosn why the value is stored in
							    a variable and during the next ISR call the DAC register
							    is loaded. */

			pMemory++;
			mode=0x02;

			if ((uint32_t)pMemory >= (uint32_t)AUDIO_MEM_STOP)
			{
				SENSORS_DEACTIVATE(phidgets);	/* Deactivate ADC12 */
				process_poll(&measure_process);
			}
			break;
	}

	counter++;

	PHIDGET_CLEAR_IRQ_FLAG(); /* Clear Interrupt flag */
}
/*---------------------------------------------------------------------------*/
