/*
 * Copyright (c) 2006, Swedish Institute of Computer Science
 * All rights reserved. 
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met: 
 * 1. Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 * 3. Neither the name of the Institute nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software 
 *    without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND 
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
 * SUCH DAMAGE. 
 *
 * @(#)$Id: DIO.c.v 1.2 2013/06/04 15:41:13 bg- Exp $
 */

/* Code adapted by Segers Laurent (June 2013) to fullfill all interuptable IO pins (Digital IO or DIO) */
#include "contiki.h"
#include "isr_compat.h"
#include "dio.h"

#define DIO_PORT 2

static uint8_t IO_PINS=0; 
static dio_msg_t dio_msg;
static struct process *selecting_proc;
//-------------------------------------------------------
inline void dio_init(struct process *proc)
{
	dio_msg.type = DIO_MSG_TYPE;
	P2DIR &= ~IO_PINS;
  	P2SEL &= ~IO_PINS;
	P2IES |= IO_PINS;	/* Falling edge detection */
	P2IFG &= ~IO_PINS;
	//P2IE |= IO_PINS;
	selecting_proc = proc;
  	if(proc != NULL)
    		P2IE |= IO_PINS;
  	else
    		P2IE &= ~IO_PINS;
}
/*------------------------------------------------------*/
inline void dio_setPins(uint8_t pin)
{
	IO_PINS|=dio_Pin(pin); //-- enable the pins by orings them
}
/*------------------------------------------------------*/
inline void dio_clearPins(uint8_t pin)
{
	IO_PINS&=~dio_Pin(pin); //-- disable the pin by "and" them to zero
}
/*------------------------------------------------------*/
ISR(PORT2, __dio_interrupt)
{
  	static struct timer debouncetimer; 	
	dio_msg.value = P2IFG&P2IE;
	//P2IES^=P2IFG; //-- make sure the interrupt will handle both transitions (low to high and high to low)
  	P2IFG &= ~IO_PINS;
  
  	if(timer_expired(&debouncetimer)) 
	{    		
    		timer_set(&debouncetimer, CLOCK_SECOND/4);
    		if(selecting_proc != NULL) 	process_post(selecting_proc, PROCESS_EVENT_MSG, &dio_msg);    		
    		LPM4_EXIT;
  	}
}
/*---------------------------------------------------------------------*/
inline uint8_t dio_pinstatus(uint8_t pin) //-- get status of requested pin (high or low)
{
	uint8_t io=dio_Pin(pin);
	return ((io&P2IN)>0?1:0);
}
/*---------------------------------------------------------------------*/
inline uint8_t dio_interruptedPin(uint8_t* pins)
{		
	if (*pins&PIN7) {(*pins)=(*pins)&(~PIN7); return 7;} //-- deselect the generated interrupt = done!
	if (*pins&PIN6) {(*pins)=(*pins)&(~PIN6); return 6;}
	if (*pins&PIN5) {(*pins)=(*pins)&(~PIN5); return 5;}
	if (*pins&PIN4) {(*pins)=(*pins)&(~PIN4); return 4;}
	if (*pins&PIN3) {(*pins)=(*pins)&(~PIN3); return 3;}
	if (*pins&PIN2) {(*pins)=(*pins)&(~PIN2); return 2;}
	if (*pins&PIN1) {(*pins)=(*pins)&(~PIN1); return 1;}
	if (*pins&PIN0) {(*pins)=(*pins)&(~PIN0); return 0;}	
	return 0xFF;
}
