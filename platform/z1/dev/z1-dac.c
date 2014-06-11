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
			DAC12_0CTL = DAC12SREF0 + DAC12RES + DAC12IR + DAC12AMP5 + DAC12ENC; /* Internal reference (2.5V), DAC output is 1x reference, Medium/Medium, 8bit resolution */
			break;
		case Z1_DAC_1:
			DAC12_1CTL = DAC12SREF1 + DAC12IR + DAC12AMP5; /* Internal reference (2.5V), DAC output is 1x reference, Medium/Medium */
			break;
	}

}

/**
 *
 */
void
dac_setValue(uint16_t value, uint8_t type) {
	switch(type) {
		case Z1_DAC_0:
			DAC12_0DAT = value;
			break;
		case Z1_DAC_1:
			DAC12_1DAT = value;
			break;
	}
}
