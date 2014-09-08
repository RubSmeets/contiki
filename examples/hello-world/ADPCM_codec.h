/*
 * ADPCM_codec.h
 *
 *  Created on: Sep 4, 2014
 *      Author: crea
 */

#ifndef ADPCM_CODEC_H_
#define ADPCM_CODEC_H_

void ADPCM_init(void);
char ADPCM_Encoder(int16_t Input);
signed int ADPCM_Decoder(char code);

#endif /* ADPCM_CODEC_H_ */
