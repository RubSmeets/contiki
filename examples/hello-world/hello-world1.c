/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
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
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"

#include <stdio.h> /* For printf() */

uint8_t test;
static uint8_t state;

uint8_t __attribute__((__far__)) test_far_function(void);

/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process, ev, data)
{
  uint8_t i = 0;
  uint8_t count = 0;

  PROCESS_BEGIN();

  printf("Hello, world\n");
  
  state = 0;

  while(1) {

	  if(test > 10) {
		  printf("ok\n");
		  count = test_far_function();
	  }

	  switch(state) {
	  	  case 0:
	  		  test = 0;
	  		  state = 1;
	  		  break;
	  	  case 1:
	  		test = 0;
	  		  state = 2;
	  		  break;
	  	  case 2:
	  		test = 0;
	  		state = 3;
	  		  break;
	  	  case 3:
	  		test = 0;
	  		state = 4;
	  		  break;
	  	  case 4:
	  		test = 0;
	  		state = 5;
	  		  break;
	  	  case 5:
	  		test = 0;
	  		state = 6;
	  		  break;
	  	  case 6:
	  		test = 0;
	  		state = 7;
	  		  break;
	  	  case 7:
	  		test = 0;
	  		state = 8;
	  		  break;
	  	  case 8:
	  		test = 0;
	  		state = 0;
	  		  break;
	  	  case 30:
	  		state = 0;
	  		break;
	  	  default:
	  		test = 0;
	  		state = 0;
	  		  break;
	  }

	  for(i=0; i<20; i++) {
		  test = i;
	  }

	  if(count == 30)	printf("far\n");

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
uint8_t __attribute__((__far__))
test_far_function(void)
{
	uint8_t count_temp=0;

	count_temp = state;

	if(count_temp == 4) {
		count_temp = 200;
	}

	if(count_temp > 100) {
		count_temp = 30;
	}

	return count_temp;
}
