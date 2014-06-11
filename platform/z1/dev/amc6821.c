/* code written by Segers Laurent */
/* Written in June 2013 */
#include <stdio.h>
#include "amc6821.h"
#include "contiki.h"
#include "i2cmaster.h"

uint8_t set_reg[3];
uint8_t invert=0;

/*
 * Initializes the PWM to zero (complete off) * 
 */
void
pwm_init() 
{
	i2c_disable();	
	i2c_enable ();
	pwm_inverted(1);

	set_reg[0] = CONFREG2_ADDR;
        set_reg[1] = ONLY_PWM_CONFREG2;	
	send_i2c(2,set_reg);	

	set_reg[0]=FANREG_ADDR;
	set_reg[1]=0x38;
	send_i2c(2,set_reg);
	pwm_set(0); /* turn PWM off */	
  	return;
}
/*--------------------------------------------------------------------------*/
void pwm_kill()
{
	set_reg[0] = CONFREG2_ADDR;
        set_reg[1] = 0x00;	
	send_i2c(2,set_reg);		
}
/*--------------------------------------------------------------------------
 * Inverts the output PWM. If value==1, the value is inverted (0% when value 255, 100% when value 0) 
 *--------------------------------------------------------------------------*/
void pwm_inverted(uint8_t value) 
{
	set_reg[0]=CONFREG1_ADDR;
	//set_reg[1]=(value==1?ONLY_PWM_CONFREG1:0x08);	
	set_reg[1]=ONLY_PWM_CONFREG1;		
	send_i2c(2,set_reg);
	invert=value;	
	return;
}
/*--------------------------------------------------------------------------*/
void pwm_set(uint8_t value)
{	
  	set_reg[0]=DCYREG_ADDR;
	set_reg[1]=(invert==1?0xFF-value:value);
	send_i2c(2,set_reg);	
	return;
}
/*--------------------------------------------------------------------------*/
/* sends data (AMC6821 commands) over I2C bus*/
void send_i2c(uint8_t length, uint8_t* message)
{	
	uint8_t wait=0;
	while (i2c_busy ());
	i2c_transmitinit (AMC6821_ADDRESS);
	while (i2c_busy ());
	i2c_transmit_n (length, message);
	while (i2c_busy ());
	while (wait<255){ wait++;}
	//i2c_disable();
	return;
}
/*--------------------------------------------------------------------------*/
