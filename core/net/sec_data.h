/*
 * sec_data.h
 *
 *  Created on: Jan 6, 2014
 *      Author: crea
 */

#ifndef SEC_DATA_H_
#define SEC_DATA_H_

#include "net/uip.h"


/* ------------------------------------- */
/* Different protocol message sizes 	 */
/* ------------------------------------- */
#define INIT_REQUEST_MSG_SIZE	1	/* msg_type(1) */
#define INIT_REPLY_MSG_SIZE		4	/* msg_type(1) | req_nonce(3) */
#define COMM_REQUEST_MSG_SIZE	39	/* msg_type(1) | device_id(16) | remote_device_id(16) | req_nonce(3) | remote_req_nonce(3) */
#define COMM_REPLY_MSG_SIZE		47	/* encryption_nonce(3) | msg_type(1) | encrypted_req_nonce(3) | encrypted_sessionkey(16) | encrypted_remote_device_id(16) | MIC(8) */
#define VERIFY_REQUEST_MSG_SIZE	28	/* encryption_nonce(3) | msg_type(1) | encrypted_verify_nonce(3) | padding (12) | MIC(8) */
#define VERIFY_REPLY_MSG_SIZE	28	/* encryption_nonce(3) | msg_type(1) | encrypted_remote_verify_nonce(3) | padding (12) | MIC(8) */

/* ------------------------------------- */
/* Message variables 					 */
/* ------------------------------------- */
#define MAX_MESSAGE_COUNT 	0xFFFFFFFF
#define MAX_NONCE_COUNT		0xFF
#define MAX_MESSAGE_SIZE	50

/* ------------------------------------- */
/* Variable sizes 						 */
/* ------------------------------------- */
#define SEC_DATA_SIZE 			32
#define SEC_KEY_SIZE			16
#define KEY_NONCE_SIZE			4
#define NONCE_CNTR_SIZE			1
#define LENGTH_SIZE				1	/* To ensure that the data array stays inbounds */
#define ADATA_KEYEXCHANGE		1
#define DEVICE_ID_SIZE			16

/* ------------------------------------- */
/* General definitions					 */
/* ------------------------------------- */
#define DEVICE_NOT_FOUND 		-1
#define MAX_DEVICES 			5 /* Max amount of devices a node can communicate with securely */
#define EDGE_ROUTER_INDEX		0 /* reserved slot in the devices array for central entity */
#define RESERVED_INDEX			1 /* reserved slot in the devices array for key-negotiation */

/* ------------------------------------- */
/* Session-key status flags			     */
/* ------------------------------------- */
typedef enum {
	/**< The session key is still valid and fresh */
	FRESH 			= 0x03,
	/**< The session key has expired */
	EXPIRED 		= 0x01,
	/**< The nonce has to be updated in flash */
	UPDATE_NONCE	= 0x02,
	/**< Free spot in device list */
	FREE_SPOT		= 0x00,
	/**< Reserved spot in device list */
	RESERVED		= 0x04,
	/**< Key exchange failed before */
	FAILED			= 0x05,
} keyfreshness_flags_type_t;

/* ------------------------------------- */
/* Global variable for security devices  */
/* ------------------------------------- */
struct device_sec_data {
  uip_ipaddr_t  	remote_device_id;
  uint16_t			msg_cntr;
  uint8_t			nonce_cntr;
  uint16_t 			remote_msg_cntr;
  uint8_t 	 		remote_nonce_cntr;
  uint8_t			key_freshness;
  uint8_t			session_key[16];
  unsigned long		time_last_activity;
};

typedef uint8_t keyExNonce_type_t;

/* ------------------------------------- */
/* Functions used in key management      */
/* ------------------------------------- */
int  search_device_id(uip_ipaddr_t* curr_device_id, uint8_t search_offset);
int  add_device_id(uip_ipaddr_t* curr_device_id);
uint8_t find_index_for_request(keyfreshness_flags_type_t search_option);
int  remove_least_active_device(void);
void remove_sec_device(uint8_t index);
void resetDeviceID_by_Index(uint8_t index);
void reset_sec_data(uint8_t index);
void update_nonce(uint8_t index);
void copy_id_to_reserved(uint8_t index);
void reset_failed_key_exchanges(void);
void store_reserved_sec_data(void);
uint8_t parse_comm_reply_message(uint8_t *data);

/* ------------------------------------- */
/* Supporting functions				     */
/* ------------------------------------- */
//void increment_request_nonce(void);
//void increment_verify_nonce(void);
//void get_decrement_verify_nonce(uint8_t *temp_verify_nonce);
/* ------------------------------------- */
/* Can be moved to UTILS			     */
/* ------------------------------------- */
void set16(uint8_t *buffer, int pos, uint16_t value);
uint16_t get16(uint8_t *buffer, int pos);

extern  struct device_sec_data devices[MAX_DEVICES];

#endif /* SEC_DATA_H_ */
