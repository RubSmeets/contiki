#ifndef __DIO_DETECT_H__
#define __DIO_DETECT_H__
/**
 * DIO - digital input output
 * Code based on Button-drivers from Contiki (Zolertia Z1) and adapted for Port 2 digital IO
 * This code handles the Port 2 settings (enabling reading IO status, and setting accoriding interrupts) * 
 * \author Segers Laurent
 * \company Erasmus University College of Brussels
 * \date 6 june 2013
 */

#define DIO_MSG_TYPE       1
#define __dio_interrupt vector_ffe6

enum PINS{PIN0=0x01,PIN1=0x02,PIN2=0x04,PIN3=0x08,PIN4=0x10,PIN5=0x20,PIN6=0x40,PIN7=0x80};

static uint8_t DIOPINS[] = {PIN0,PIN1,PIN2,PIN3,PIN4,PIN5,PIN6,PIN7};

/** 
 * Returns the hexadecimal value (PIN assigment for the regsiter of the MCU) of the corresponding pin 
 */
#define dio_Pin(pin) DIOPINS[pin]

struct Dio_msg {
  uint8_t type;
  uint8_t value;
};

/**
 * Initialises the IO status of the port -> should be used after setPins and clearPins has been used
 */
void dio_init(struct process *proc);
/**
 *  Sets a want pin to the interruptable pins. 
 */
void dio_setPins(uint8_t pin);
/**
 * Clear the undesired pins (not interruptable anymore)
 */
void dio_clearPins(uint8_t pin);
/**
 * Get the status of a specified pin
 */
uint8_t dio_pinstatus(uint8_t pin);
/**
 * Get the next pin that caused the interrupt (highest number first in case of multiple pin interrupt)
 */
uint8_t dio_interruptedPin(uint8_t* pins);


#endif /* __DIO_DETECT_H__ */

