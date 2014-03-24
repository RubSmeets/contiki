/*
 * sec_data.c
 *
 *  Created on: Jan 6, 2014
 *      Author: crea
 */
#include "net/sec_data.h"
#include <string.h>

#define DEBUG_SEC 0
#if DEBUG_SEC
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTFDEBUG(...) printf(__VA_ARGS__)
#else
#define PRINTFDEBUG(...)
#define PRINTF(...)
#endif

/* Functions used in sec_data.c */
static void remove_sec_device(uint8_t index);
/*-----------------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------------*/
/* Can be moved to UTILS															 */
/*-----------------------------------------------------------------------------------*/
uint16_t __attribute__((__far__))
get16(uint8_t *buffer, int pos)
{
  return (uint16_t)buffer[pos] << 8 | buffer[pos + 1];
}
/*-----------------------------------------------------------------------------------*/
void __attribute__((__far__))
set16(uint8_t *buffer, int pos, uint16_t value)
{
  buffer[pos++] = value >> 8;
  buffer[pos++] = value & 0xff;
}
/*-----------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------*/



/*-----------------------------------------------------------------------------------*/
/**
 * Get the security data from flash for device at a given index (index)
 */
/*-----------------------------------------------------------------------------------*/
void __attribute__((__far__))
set_session_key_of_index(int index)
{
	uint8_t i;
	PRINTFDEBUG("key: "); for(i=0;i<16;i++) PRINTFDEBUG("%02x ", devices[index].session_key[i]); PRINTFDEBUG("\n");
	CC2420_WRITE_RAM_REV(&devices[index].session_key[0], CC2420RAM_KEY1, SEC_KEY_SIZE);
}

/*-----------------------------------------------------------------------------------*/
/**
 *	Only reset the device ID of the security data from device at position "index"
 *
 *	@param current device_index
 */
/*-----------------------------------------------------------------------------------*/
void __attribute__((__far__))
resetDeviceID_by_Index(uint8_t index)
{
	memset(&devices[index].remote_device_id.u8[0], 0, DEVICE_ID_SIZE);
}

/*-----------------------------------------------------------------------------------*/
/**
 *	Reset security data from device at position "index"
 *
 *	@param current device_index
 */
/*-----------------------------------------------------------------------------------*/
void __attribute__((__far__))
reset_sec_data(uint8_t index)
{
	devices[index].nonce_cntr = 1;
	devices[index].msg_cntr = 0;
	devices[index].remote_msg_cntr = 0;
	devices[index].remote_nonce_cntr = 0;
	devices[index].time_last_activity = 0;

	if(index >= NON_RESERVED_INDEXES) {
		/* Set as free spot */
		devices[index].key_freshness = FREE_SPOT;
		/* Reset device id */
		memset(&devices[index].remote_device_id.u8[0], 0, DEVICE_ID_SIZE);
	}
}

/*-----------------------------------------------------------------------------------*/
/**
 *	Copy the device id to the reserved spot for key-exchange
 *
 *	@param current device_index
 */
/*-----------------------------------------------------------------------------------*/
void __attribute__((__far__))
copy_id_to_reserved(uint8_t index)
{
	memcpy(&devices[RESERVED_INDEX].remote_device_id.u8[0], &devices[index].remote_device_id.u8[0], DEVICE_ID_SIZE);
}

/*-----------------------------------------------------------------------------------*/
/**
 * Search the given IP address
 */
/*-----------------------------------------------------------------------------------*/
int __attribute__((__far__))
search_device_id(uip_ipaddr_t *curr_device_id, uint8_t search_offset)
{
	int index = DEVICE_NOT_FOUND;
	uint8_t i;

	for(i = search_offset; i < MAX_DEVICES; i++) {
		if(memcmp(&curr_device_id->u8[0], &devices[i].remote_device_id.u8[0], DEVICE_ID_SIZE) == 0) {
			index = i;
			break;
		}
	}
	return index;
}

/*-----------------------------------------------------------------------------------*/
/**
 * Search for the index of device that wants to request a key or has
 * to update one.
 */
/*-----------------------------------------------------------------------------------*/
uint8_t __attribute__((__far__))
find_index_for_request(keyfreshness_flags_type_t search_option)
{
	uint8_t i;
	for(i=0; i<MAX_DEVICES; i++) {
		if(devices[i].key_freshness == search_option) {
			return i;
		}
	}

	if((search_option == UPDATE_NONCE) && (update_key_exchange_nonce == 1)) return MAX_DEVICES+1;
	return MAX_DEVICES;
}

/*-----------------------------------------------------------------------------------*/
/**
 *	Remove the security device that has been inactive the longest time
 */
/*-----------------------------------------------------------------------------------*/
int __attribute__((__far__))
remove_least_active_device(void)
{
	uint8_t i;
	int least_active_index = 3;

	/* Find the longest inactive security device */
	for(i=2; i<MAX_DEVICES; i++) {
		if(devices[least_active_index].time_last_activity > devices[i].time_last_activity) {
			least_active_index = i;
		}
	}

	/* Clear the longest inactive device */
	remove_sec_device(least_active_index);

	return least_active_index;
}

/*-----------------------------------------------------------------------------------*/
/**
 *	Removes a security device from stored devices
 *
 *	@param current device_index
 */
/*-----------------------------------------------------------------------------------*/
static void
remove_sec_device(uint8_t index)
{
	reset_sec_data(index);
	amount_of_known_devices--;
}
