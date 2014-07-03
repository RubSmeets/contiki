/*
 * rmoni_serial.c
 *
 *  Created on: Jul 1, 2014
 *      Author: crea
 */

#include "contiki.h"
#include "dev/uart1.h"
#include "dev/rmoni_serial_line.h"
#include "dev/button-sensor.h"

#include "lib/ringbuf.h"
#include <string.h>

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* ------------------------------------- */
/* Rmoni ASCII commands				     */
/* ------------------------------------- */
typedef enum {
	GET 			= 0x00,
	SET 			= 0x01,
	RES				= 0x02,
	RESET			= 0x03,
	DEFAULT			= 0x04,
	PING			= 0x05,
	PONG			= 0x06,
	GENERAL_INFO	= 0x07,
	NETWORK			= 0x08,
} rmoni_command_type_t;

/* ------------------------------------- */
/* Rmoni Measurement keys			     */
/* ------------------------------------- */
typedef enum {
	NTC_TEMP 			,
	HUMIDITY 			,
	CURRENT				,
	RESISTOR			,
	VOLTAGE				,
	CO2					,
	PRESSURE			,
	THERMOCOUPLE		,
	PARTICLE			,
	PRESSURE_kPA		,
	DISTANCE			,
	SOIL_MOISTURE		,
	LEAF_WETNESS		,
	PT100				,
	CONTACT				,
	ENERGY				,
	POWER				,
	UNITS				,
	UNITS_PER_HOUR		,
	VOLUME				,
	FLOW				,
	DUMMY				,
	RELAYS				,
	DIGITAL_IO			,
} rmoni_keys_type_t;

/* ------------------------------------- */
/* Rmoni Sensor pins				     */
/* ------------------------------------- */
typedef enum {
	PIN1	 			,
	PIN2	 			,
	PIN4				,
	PIN8				,
	PIN10				,
} rmoni_pin_type_t;

/* ------------------------------------- */
/* Rmoni sensor device					 */
/* ------------------------------------- */
struct rmoni_device {
	uint8_t		mac_address[16];
	char		version[17];
	uint8_t		initialised;
};

static const char * rmoni_commands[9] = {
	"RM^GET", "RM^SET", "RM^RES", "RM^RESET\r", "RM^DEFAULT\r", "RM^PING\r", "RM^PONG\r", "RM^N\r", "RM^NET"
};

static const char * rmoni_keys[24] = {
	"10", "11", "13", "14", "15", "16", "17", "19", "1C", "1D",
	"20", "21", "22", "23", "25", "26", "27", "28", "29", "2A",
	"2B", "33", "3F", "41"
};

static const char * rmoni_pins[10] = {
	"01", "02", "04", "08", "10", "20", "00", "00", "00", "00"
};

#define ASCII_HEADER_SIZE	6
#define SEPERATOR_CHAR(c) (c == 0x2C)

static struct rmoni_device rmoni_module;

static void rmoni_parse_message(char * msg);

/*----------------------------------------------------------------------------*/
PROCESS(hello_process, "hello process");
AUTOSTART_PROCESSES(&hello_process);
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* IMPORTANT: every command has to end with '\r' to avoid out-of-bound errors */
/*----------------------------------------------------------------------------*/
static void
rmoni_serial_output(const char *buf) {
	uint8_t i = 0;

	/* Start sending bytes until carriage return */
	while(buf[i] != '\r') {
		PRINTF("%c", buf[i]);
		uart1_writeb(buf[i]);
		i++;
	}
	/* Send carriage return to end command */
	uart1_writeb(buf[i]);
}
/*----------------------------------------------------------------------------*/
PROCESS_THREAD(hello_process, ev, data)
{
	static struct etimer et;
	static char message[60];
	PROCESS_BEGIN();

	SENSORS_ACTIVATE(button_sensor);

	PRINTF("Hello process\n");

	uart1_init(9600);
	uart1_set_input(rmoni_serial_line_input);
	rmoni_serial_line_init();

	/* Set timer */
	etimer_set(&et, CLOCK_SECOND*4);

	while(1) {
		PROCESS_YIELD();

		if(ev == serial_line_rmoni_message) {
			/* Parse message */
			rmoni_parse_message((char *) data);
		} else if(ev == sensors_event && data == &button_sensor) {
			/* Request temperature */
			memcpy(message, rmoni_commands[GET], 6);
			message[6] = ',';
			memcpy(&message[7], &rmoni_module.mac_address[0], 16);
			message[23] = ',';
			memcpy(&message[24], rmoni_keys[NTC_TEMP], 2);
			memcpy(&message[26], rmoni_pins[PIN2], 2);
			message[28] = '\r';
			rmoni_serial_output(message);
		} else if(etimer_expired(&et)) {
			/* Request general device information */
			if(!rmoni_module.initialised) {
				etimer_reset(&et);
				PRINTF("Request MAC\n");
				rmoni_serial_output(rmoni_commands[GENERAL_INFO]);
			} else {
				etimer_stop(&et);
			}
		}
	}

	PROCESS_END();
}
/*----------------------------------------------------------------------------*/
#define INFO_MAC_OFFSET		11
#define INFO_VERSION_OFFSET 36
/*----------------------------------------------------------------------------*/
static void
rmoni_parse_message(char * msg) {
	uint8_t i = 0;

	while(msg[i] != '\0') {
		PRINTF("%c", msg[i]);
		i++;
	}
	PRINTF("\n");

	/* Parse info packet */
	if(!rmoni_module.initialised) {
		if(memcmp(&msg[0], rmoni_commands[NETWORK], ASCII_HEADER_SIZE) == 0) {
			PRINTF("Init rmoni OK\n");
			memcpy(&rmoni_module.mac_address[0], &msg[INFO_MAC_OFFSET], 16);
			memcpy(&rmoni_module.version[0], &msg[INFO_VERSION_OFFSET], 17);
			rmoni_module.initialised = 1;
		}
	} else {

	}
}
