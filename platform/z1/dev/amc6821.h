#ifndef AMC6821_H
#define AMC6821_H

/*
 * By default, We'll use the Software-DCY mode.
 * The PWM freq/duty will be set by software
 *
 * When PWM_MODE is tied to GND, 1KHz - 40KHz,
 * instead (VDD or air) 10Hz - 94Hz.
 *
 * The slave address is set depending of A0/A1 pins
 * We'll tie A0 and A1 to GND, so address is 0x18
 *
 */
  
#define AMC6821_ADDRESS 0x18

#define CONFREG1_ADDR 0x00
#define CONFREG2_ADDR 0x01


/* 
 * After power-on, the IC doesn't perform any measure
 * until START bit of configuration reg 1 is set.  After
 * power-on the duty cycle is set to 33%.
 * Setting START to 0 will also disable de ADC, fan speed
 * measurement (tachiometer) and other operation modes except
 * DCY-software mode.
 *
 * The software-DCY mode is an open-loop mode, is determined by clearing both
 * bits FDRC1 and FDRC0.  
 *
 * +-----------+-------+-------+--------------+--------+-------+--------+-------+
 * | THERMOVIE | FDRC1 | FDRC0 | FAN-FAULT-EN | PWMINV | FANIE | INT-EN | START |
 * +-----------+-------+-------+--------------+--------+-------+--------+-------+
 * |    0      |   0   |   0   |       0      |    1   |   0   |   0    |   0   |
 * +-----------+-------+-------+--------------+--------+-------+--------+-------+ 
 */

#define ONLY_PWM_CONFREG1 0x08

/*
 * When PWM-EN (config-reg2) is clear, PWM is disabled, else
 * set is enabled.  When enabled, if PWMINV (config-reg1) is
 * set it pulls high PWM-OUT for 100% duty cycle, if clear then
 * PWM-OUT goes low for 100% duty cycle (default).
 *
 * When enabled, RESET bit does, well, reset.
 *
 * +-------+-------+-------+-------+-------+---------+-----------+--------+
 * | RESET | PSVIE | RTOIE | LTOIE | RTFIE | TACH-EN | TACH-MODE | PWM-EN |
 * +-------+-------+-------+-------+-------+---------+-----------+--------+
 * |  0    |   0   |   0   |  0    |   0   |    0    |     1     |   1    |
 * +-------+-------+-------+-------+-------+---------+-----------+--------+
 */

#define ONLY_PWM_CONFREG2 0x03
#define ONLY_PWM_CONFREG2_STOPWM 0x02

/* 
 * DCY register defines duty cycle with 8bit resolution 1/255 (0.392%)
 * Default power-on value for frequency is 30Hz/25KHz (PWM-MODE)
 *
 */

#define DCYREG_ADDR 0x22
#define DCYREG_25DC 0x40
#define DCYREG_50DC 0x80

/*
 * Spin-up start is disabled by setting FSPD bit (fan characteristics reg) to 1
 * Frequency can be set in the following manner:
 * 
 * PWM2 PWM 1 PWM0 Freq
 *  0    0     0   10Hz
 *  0    0     1   15Hz
 *  0    1     0   23Hz
 *  0    1     1   30Hz
 *  1    0     0   38Hz
 *  1    0     1   47Hz  
 *  1    1     0   65Hz
 *  1    1     1   94Hz
 *
 * PWM2 PWM 1 PWM0 Freq
 *  0    0     0   1KHz
 *  0    0     1   10KHz
 *  0    1     0   20KHz
 *  0    1     1   25KHz
 *  1    0     0   30KHz
 *  1    0     1   40KHz  
 *  1    1     0   40KHz
 *  1    1     1   40KHz 
 *
 * +-------+-------+-------+-------+-------+---------+-----------+--------+
 * | FSPD  | XXXX  | PWM2  | PWM1  | PWM0  | STIME2  |  STIME1   | STIME0 |
 * +-------+-------+-------+-------+-------+---------+-----------+--------+
 * |  1    |   0   |   0   |  0    |   0   |    0    |     0     |   0    |
 * +-------+-------+-------+-------+-------+---------+-----------+--------+
 */

#define FANREG_ADDR 0x20
#define PWM_ONLY_FANREG_1KHZ 0x88

enum {
  AMC_IDLE = 0,
  AMC_START,
  AMC_PWMSET,
  AMC_FANSET,
  AMC_PWMDUTY,
  AMC_STOP,
} CMD;

/* Initializes the PWM to zero (complete off) */
void pwm_init();
/*--------------------------------------------------------------------------*/
void pwm_kill();
/*--------------------------------------------------------------------------
 * Inverts the output PWM. If value==1, the value is inverted (0% when value 255, 100% when value 0) 
 *--------------------------------------------------------------------------*/
void pwm_inverted(uint8_t value);
void pwm_set(uint8_t value);
/* sends data (AMC6821 commands) over I2C bus*/
void send_i2c(uint8_t length, uint8_t* message);


#endif



