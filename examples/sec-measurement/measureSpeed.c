/*
 * measureSpeed.c
 *
 *  Created on: Nov 25, 2014
 *      Author: crea
 */

#include "sys/energest.h"
#include "sys/rtimer.h"
#include <string.h>

#define DEBUG_SEC 1
#if DEBUG_SEC
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTFDEBUG(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#define PRINTFDEBUG(...)
#endif

#define FINE_STEP 636 		/* nano seconds -> 48 times write 2-byte to variable in 1/32768Hz interval, gives 1/(32768Hz*48) */
#define NORMAL_STEP 30518 	/* nano seconds -> 1/32768Hz */

/* Energy measurement variables*/
typedef struct {
	unsigned short source;
	long cpu;
	long lpm;
	long transmit;
	long listen;
} energy_time_t;

static rtimer_clock_t t1;
static rtimer_clock_t t2;
static rtimer_clock_t tbuf[50];
static uint8_t i;
static uint32_t difference = 0;
static uint32_t normalTime = 0;
static uint32_t fineTime = 0;
static uint32_t totalTime = 0;
static rtimer_clock_t tbuf[50];
static uint8_t index = 0;
static uint8_t count[5];

static energy_time_t diff;
static energy_time_t last;


inline void
measurement_start(void) {
	PRINTFDEBUG("Start speed measurement\n");

	/* update all counters */
	energest_flush();

	difference = 0;
	normalTime = 0;
	fineTime = 0;
	totalTime = 0;

	index = 0;

	memset(count, 0, sizeof(count));
	memset(tbuf, 0, sizeof(tbuf));

	last.cpu = energest_type_time(ENERGEST_TYPE_CPU);
	last.lpm = energest_type_time(ENERGEST_TYPE_LPM);
	last.transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT);
	last.listen = energest_type_time(ENERGEST_TYPE_LISTEN);

	/* Start measurement */
	t1=RTIMER_NOW();
}

inline uint32_t
measurement_stop(void) {
	tbuf[0] = TAR;
	tbuf[1] = TAR;
	tbuf[2] = TAR;
	tbuf[3] = TAR;
	tbuf[4] = TAR;
	tbuf[5] = TAR;
	tbuf[6] = TAR;
	tbuf[7] = TAR;
	tbuf[8] = TAR;
	tbuf[9] = TAR;
	tbuf[10] = TAR;
	tbuf[11] = TAR;
	tbuf[12] = TAR;
	tbuf[13] = TAR;
	tbuf[14] = TAR;
	tbuf[15] = TAR;
	tbuf[16] = TAR;
	tbuf[17] = TAR;
	tbuf[18] = TAR;
	tbuf[19] = TAR;
	tbuf[20] = TAR;
	tbuf[21] = TAR;
	tbuf[22] = TAR;
	tbuf[23] = TAR;
	tbuf[24] = TAR;
	tbuf[25] = TAR;
	tbuf[26] = TAR;
	tbuf[27] = TAR;
	tbuf[28] = TAR;
	tbuf[29] = TAR;
	tbuf[30] = TAR;
	tbuf[31] = TAR;
	tbuf[32] = TAR;
	tbuf[33] = TAR;
	tbuf[34] = TAR;
	tbuf[35] = TAR;
	tbuf[36] = TAR;
	tbuf[37] = TAR;
	tbuf[38] = TAR;
	tbuf[39] = TAR;
	tbuf[40] = TAR;
	tbuf[41] = TAR;
	tbuf[42] = TAR;
	tbuf[43] = TAR;
	tbuf[44] = TAR;
	tbuf[45] = TAR;
	tbuf[46] = TAR;
	tbuf[47] = TAR;
	tbuf[48] = TAR;
	tbuf[49] = TAR;

	t2=RTIMER_NOW();

	diff.cpu = energest_type_time(ENERGEST_TYPE_CPU) - last.cpu;
	diff.lpm = energest_type_time(ENERGEST_TYPE_LPM) - last.lpm;
	diff.transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT) - last.transmit;
	diff.listen = energest_type_time(ENERGEST_TYPE_LISTEN) - last.listen;

	PRINTFDEBUG("CPU=%lu, LPM=%lu, TRANSMIT=%lu, LISTEN=%lu, TICKS=%u\n", diff.cpu, diff.lpm, diff.transmit, diff.listen, t2-t1);

	for(i=0; i<(50); i++) {
		if(tbuf[i] == tbuf[i+1]) {
			count[index]++;
		} else {
			index++;
		}
	}

	difference = (tbuf[0]-1) - t1;
	normalTime = difference*NORMAL_STEP;
	fineTime = FINE_STEP*(48 - count[0]);
	totalTime = normalTime + fineTime;

	PRINTFDEBUG("%lu", totalTime); //PRINTF(" ns");

	PRINTFDEBUG("counts: ");
	for(i=0; i<5; i++) {
		PRINTFDEBUG("%d ", count[i]);
		count[i] = 0;
	}
	PRINTFDEBUG("\n");
	return totalTime;
}
