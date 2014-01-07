#include "contiki.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include <stdio.h> /* For printf() */
//#include <stdint.h>

//__asm(
//	.data
//	.char 0,0,0,0,0,0,0

//);

void __attribute__((__far__,__c20__)) SetBlueLED(uint8_t* __attribute__ ((__d20__)) pointer);
void __attribute__((__near__)) CallNear(uint8_t* __attribute__ ((__d20__)) pointer);
//-----------------------------------------------------------------
PROCESS(blink_timer_process, "blink with timer example");
AUTOSTART_PROCESSES(&blink_timer_process);
//-----------------------------------------------------------------
//static char  buffer[2000] __attribute__((__d20__,__far__))={"Dit is een test"};
//static char buffer2[2000] __attribute__((section(".fardata2")))={"Dit is een test"};
static uint8_t* __attribute__((__d20__))  pcounter;

PROCESS_THREAD(blink_timer_process, ev, data)
{

	//buffer[0]++;
	PROCESS_EXITHANDLER(goto exit);
	PROCESS_BEGIN();
	static uint8_t __attribute__ ((__d20__)) counter;
	pcounter= &counter;
	/* Initialize stuff here. */
	printf("\n++++++++++++++++++++++++++++++\n");
	printf("+    LESSON 1, EXERCISE 2    +\n");
	printf("++++++++++++++++++++++++++++++\n");
	printf("+     Blink app w/ timer     +\n");
	printf("++++++++++++++++++++++++++++++\n\n");
	SENSORS_ACTIVATE(button_sensor);
	leds_on(LEDS_ALL);
	printf("+       All leds are off     +\n\n");
	printf("Press the user button to start\n\n");

    while(1)
    {
		/* Do the rest of the stuff here. */
		static uint32_t seconds = 1;
		static struct etimer et; // Define the timer
		//leds_toggle(LEDS_BLUE);
		etimer_set(&et, CLOCK_SECOND*seconds);  // Set the timer

		PROCESS_WAIT_EVENT();  // Waiting for a event, don't care which
		//leds_toggle(LEDS_BLUE);
		//if(ev == sensors_event)
		//{  // If the event it's provoked by the user button, then...
			//if(data == &button_sensor)
			//{

				//printf("+       Timer started        +\n");
			//}
		//}
		SetBlueLED(&counter);
		counter++;
		if(etimer_expired(&et))
		{  // If the event it's provoked by the timer expiration, then...
			etimer_reset(&et);
		}
	}
	exit:
	leds_off(LEDS_ALL);
	PROCESS_END();
}

void __attribute__((__far__,__c20__)) SetBlueLED(uint8_t* __attribute__ ((__d20__)) pointer)
{
	uint8_t value = (*pointer);
	//buffer[0]=buffer[1];
	//buffer2[0]=buffer2[1];
	if (value>=10)
	{
		CallNear(pointer);
		//(*pointer)=0;
		//leds_off(LEDS_BLUE);
		//leds_off(LEDS_RED);
		//leds_toggle(LEDS_ALL);
	}
	if (value==5)
	{
		leds_toggle(LEDS_RED);
	}
	if (value==3)
	{
		leds_toggle(LEDS_BLUE);
	}

	//else leds_off(LEDS_BLUE);
	//leds_toggle(LEDS_BLUE);
}

void  __attribute__((__near__)) CallNear(uint8_t* __attribute__ ((__d20__)) pointer)
{
	//leds_toggle(LEDS_ALL);
	//unsigned char L = (unsigned char)(*LED);
	//leds_toggle(LEDS_ALL);
	if ((*pointer)==10)
	{
		leds_off(LEDS_RED);
		leds_off(LEDS_GREEN);
	}
	else if ((*pointer)==12)
	{
		//leds_on(LEDS_RED);
		leds_on(LEDS_GREEN);
		(*pointer)=0;
	}
}

