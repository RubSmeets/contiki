/*
 * z1-dac.h
 *
 *  Created on: May 16, 2014
 *      Author: crea
 */

#ifndef Z1_DAC_H_
#define Z1_DAC_H_

#define Z1_DAC_0 0
#define Z1_DAC_1 1

#define PIN6 	  (1<<6)
#define PIN4	  (1<<4)

void dac_init(uint8_t type);
void dac_setValue(uint16_t value, uint8_t type);
void dac_disable(uint8_t type);

#endif /* Z1_DAC_H_ */
