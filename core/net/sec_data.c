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

	if(index != RESERVED_INDEX) {
		/* Set as free spot */
		devices[index].key_freshness = FREE_SPOT;
		/* Reset device id */
		memset(&devices[index].remote_device_id.u8[0], 0, DEVICE_ID_SIZE);
	}
}

/*-----------------------------------------------------------------------------------*/
/**
 * Update nonce writes the new nonce of devices[index] to flash memory
 *
 * @param index of device
 */
/*-----------------------------------------------------------------------------------*/
void __attribute__((__far__))
update_nonce(uint8_t index)
{
	devices[index].key_freshness = FRESH;
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
 *	Reset the failed key-exchanges to expired
 */
/*-----------------------------------------------------------------------------------*/
void __attribute__((__far__))
reset_failed_key_exchanges(void)
{
	uint8_t i;
	for(i=2; i<MAX_DEVICES; i++) {
		if(devices[i].key_freshness == FAILED) {
			devices[i].key_freshness = EXPIRED;
		}
	}
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
 * add the given device id to secured communication
 */
/*-----------------------------------------------------------------------------------*/
int __attribute__((__far__))
add_device_id(uip_ipaddr_t* curr_device_id)
{
	int index = DEVICE_NOT_FOUND;

	/* Make room for new device */
	if(amount_of_known_devices == MAX_DEVICES) {
		index = remove_least_active_device();
	}

	/* Add device to known devices */
	index = find_index_for_request(FREE_SPOT);
	memcpy(&devices[index].remote_device_id.u8[0], &curr_device_id->u8[0], DEVICE_ID_SIZE);
	amount_of_known_devices++;

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
	int least_active_index = 2;

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
void __attribute__((__far__))
remove_sec_device(uint8_t index)
{
	reset_sec_data(index);
	amount_of_known_devices--;
}

/*-----------------------------------------------------------------------------------*/
/**
 *	Store the temporary security data in a free spot if not found
 */
/*-----------------------------------------------------------------------------------*/
void __attribute__((__far__))
store_reserved_sec_data(void)
{
	int index;

	index = search_device_id(&devices[RESERVED_INDEX].remote_device_id,2);
	if(index < 0) {
		index = find_index_for_request(FREE_SPOT);
	}

	/* store security device data */
	devices[index] = devices[RESERVED_INDEX];
	devices[index].key_freshness = FRESH;

	/* Reset RESERVED id */
	memset(&devices[RESERVED_INDEX].remote_device_id.u8[0], 0, DEVICE_ID_SIZE);
}

///*-----------------------------------------------------------------------------------*/
///**
// *	Help function to parse the content of communication reply message.
// *
// *	@param pointer to data
// *	@param pointer to current device id
// *	@return failed/successful
// */
///*-----------------------------------------------------------------------------------*/
//#define ID_OFFSET				23
//#define SESSIONKEY_OFFSET		7
//#define REQUEST_NONCE_OFFSET	4
//
//uint8_t parse_comm_reply_message(uint8_t *data) {
//	uint8_t temp_request_nonce[3];
//	uip_ipaddr_t curr_ip;
//
//	/* Get own ip address */
//	uip_ds6_select_src(&curr_ip, &devices[RESERVED_INDEX].remote_device_id);
//
//	/* Assemble request nonce */
//	set16(temp_request_nonce, 0, request_nonce);
//	temp_request_nonce[2] = request_nonce_cntr;
//
//	/* Check request nonce */
//	if(memcmp(&data[REQUEST_NONCE_OFFSET], &temp_request_nonce[0], 3) != 0) {
//		/* Doesn't belong with current request - replay message */
//		//PRINTF("key: wrong req_nonce\n");
//		return 0;
//	}
//
//	/* Check device id */
//	if(memcmp(&data[ID_OFFSET], &curr_ip.u8[0], DEVICE_ID_SIZE) != 0) {
//		/* Wrong id */
//		//PRINTF("key: wrong id\n");
//		return 0;
//	}
//
//	/* Store security data */
//	reset_sec_data(RESERVED_INDEX);
//	memcpy(&devices[RESERVED_INDEX].session_key[0], &data[SESSIONKEY_OFFSET], SEC_KEY_SIZE);
//
//	/* Increment request nonce */
//	increment_request_nonce();
//
//	//PRINTF("key: Parse ok\n");
//
//	return 1;
//}
