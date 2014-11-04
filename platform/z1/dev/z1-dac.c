/*
 * z1-dac.c
 *
 *  Created on: May 16, 2014
 *      Author: crea
 */

#include "contiki.h"
#include "dev/z1-dac.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/**
 *
 */
void
dac_init(uint8_t type) {

	/* Port directions(P6DIR) and selection(P6SEL) has no influence on DAC functionality */
	P6DIR = PIN6;
	P6SEL = PIN6;

	/* Enable internal reference voltage (external references are not available on Z1) */
	ADC12CTL0 |= REF2_5V + REFON;

	switch(type) {
		case Z1_DAC_0:
			/* Configure Amplification settings before calibration */
			DAC12_0CTL = DAC12AMP_5;

			/* Start calibration DAC */
			DAC12_0CTL |= DAC12CALON;
			PRINTF("Calibrate DAC\n");
			while((DAC12_0CTL & DAC12CALON) != 0) {
				_NOP();
			}
			PRINTF("Calibrate DAC done\n");

			DAC12_0CTL |= DAC12SREF0 + DAC12IR + DAC12DF + DAC12ENC; /* Internal reference (2.5V), DAC output is 1x reference, Medium/Medium, 12bit resolution, 2s complement */
			PRINTF("Reg: %04x\n", DAC12_0CTL);
			break;
		case Z1_DAC_1:
			DAC12_1CTL = DAC12SREF1 + DAC12IR + DAC12AMP_5; /* Internal reference (2.5V), DAC output is 1x reference, Medium/Medium */
			break;
	}

}

/**
 *
 */
void
dac_setValue(uint16_t value, uint8_t type) {
	uint16_t temp = 0;
	/* value: [1]111 0001 0011 0101 -> temp: [1]001 0011 0101  */
	temp = ((value>>1) & 0x07FF) + ((value>>4) & 0x0800);

	switch(type) {
		case Z1_DAC_0:
			//DAC12_0DAT = (value>>4);
			DAC12_0DAT = temp;
			break;
		case Z1_DAC_1:
			DAC12_1DAT = value;
			break;
	}
}

/**
 *
 */
void
dac_disable(uint8_t type) {

	/* Do not disable Reference voltage generator (may be in use by ADC) */
	switch(type) {
		case Z1_DAC_0:
			DAC12_0CTL = 0x00;
			break;
		case Z1_DAC_1:
			DAC12_1CTL = 0x00;
			break;
	}
}
