/*
 * keymanagement-v1.c
 *
 *  Created on: Jul 26, 2013
 *      Author: crea
 */

#include "net/symm-key-client-v1.h"
#include "net/sec_data.h"
#include "net/packetbuf.h"
#include "sys/clock.h"
#include "net/sec-arp-client.h"

#include <string.h>

#if ENABLE_CCM_APPLICATION & SEC_CLIENT

#define DEBUG_SEC 0
#if DEBUG_SEC
#include <stdio.h>
static const char * printstate[10] = {
	"S_REQ", "S_REPLY", "S_COMM_REQ", "S_COMM_REPLY", "S_VERIFY_REQ", "S_VERIFY_REPLY", "S_SUCCES", "S_FAILED", "S_IDLE"
};
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTFBOOT(...)
#define PRINTFDEBUG(...) printf(__VA_ARGS__)
#else
#define PRINTFDEBUG(...)
#define PRINTFBOOT(...)
#define PRINTF(...)
#endif

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UDP_CLIENT_SEC_PORT 5446
#define UDP_SERVER_SEC_PORT 5444

#define AUTHENTICATION_SUCCES	0x00
#define NORMAL_MODE				0x00
#define SECURITY_MODE			0x01

/* Register offsets */
#define SEC_KEY_OFFSET			16
#define MSG_TYPE_OFFSET			3

/* Timing defines */
#define CHECK_INTERVAL		(CLOCK_SECOND)*5
#define MAX_WAIT_TIME_SEND		5
#define MAX_SEND_TRIES			3

/* Different states */
#define S_IDLE 			0
#define S_REQUEST_KEY	1
#define S_UPDATE_KEY	2
#define S_BOOTSTRAP_KEY	3

/* Different key exchange states */
#define S_INIT_REQUEST			0
#define S_INIT_REPLY			1
#define S_COMM_REQUEST			2
#define S_COMM_REPLY			3
#define S_VERIFY_REQUEST		4
#define S_VERIFY_REPLY			5
#define S_KEY_EXCHANGE_SUCCES	6
#define S_KEY_EXCHANGE_FAILED	7
#define S_KEY_EXCHANGE_IDLE 	8

/* Different key update states */
#define S_UPDATE_REQ			9
#define S_UPDATE_REPLY_OK		10
#define S_UPDATE_SUCCES			14
#define S_UPDATE_FAILED			15

/* Message types */
#define UPDATE_MSG				0xE0
#define UPDATE_REQ				0xD0
#define UPDATE_OK				0xF0
#define S_UPDATE_NETWORK		0x02
#define S_UPDATE_SENSOR			0x00
#define S_UPDATE_SESSION		0x01

/* General variables */
static short state;
static short key_exchange_state;
static uint8_t send_tries;
static struct uip_udp_conn *sec_conn;

/* Global variables */
uint8_t amount_of_known_devices;
uint8_t update_key_exchange_nonce;

/* Buffer variables */
static uint16_t keypacketbuf_aligned[(MAX_MESSAGE_SIZE) / 2 + 1];
static uint8_t *keypacketbuf = (uint8_t *)keypacketbuf_aligned;
static uint8_t tot_len;

/* Key exchange nonces */
static uint16_t request_nonce = 1;
static uint16_t verify_nonce = 1;
static keyExNonce_type_t request_nonce_cntr;
static keyExNonce_type_t verify_nonce_cntr;
static uint8_t remote_request_nonce[KEY_NONCE_SIZE+NONCE_CNTR_SIZE];
static uint8_t remote_verify_nonce[KEY_NONCE_SIZE+NONCE_CNTR_SIZE];
static uint8_t remote_server_nonce[KEY_NONCE_SIZE+NONCE_CNTR_SIZE];

/* Functions used in key management layer */
static uint8_t key_exchange_protocol(void);
static void send_key_exchange_packet(void);
static void init_reply_message(void);
static void comm_request_message(void);
void __attribute__((__far__)) verify_request_message(void);
void __attribute__((__far__)) verify_reply_message(uint8_t* remote_nonce);
static short receive_callback(uint8_t *data, uint16_t len);
static short receive_key_update(uint8_t *data, uint16_t len);
uint8_t __attribute__((__far__)) check_for_update_packet(uint8_t *data, uint8_t* len, uint8_t messageLength);
uint8_t __attribute__((__far__)) parse_comm_reply_message(uint8_t *data);
static uint8_t parse_update_message(uint8_t *data);
static uint8_t key_update_protocol(uint8_t req_device);
static void send_key_update_packet(uint8_t req_device);

static void store_reserved_sec_data(void);
static int  add_device_id(uip_ipaddr_t* curr_device_id);
static void reset_failed_key_exchanges(void);
static void update_nonce_flash(uint8_t index);
static void update_sensor_key_flash(void);

/*---------------------------------------------------------------------------*/
PROCESS(keymanagement_process, "key management");
/*---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------*/
/* Supporting functions																 */
/*-----------------------------------------------------------------------------------*/
static void
increment_request_nonce(void)
{
	if(request_nonce == 0xffff) {
		request_nonce_cntr++;
		request_nonce = 0;
		update_key_exchange_nonce = 1;
	}
	else {
		request_nonce++;
	}
}
/*-----------------------------------------------------------------------------------*/
static void
increment_verify_nonce(void)
{
	if(verify_nonce == 0xffff) {
		verify_nonce_cntr++;
		verify_nonce = 0;
		update_key_exchange_nonce = 1;
	}
	else {
		verify_nonce++;
	}
}
/*-----------------------------------------------------------------------------------*/
static void
get_decrement_verify_nonce(uint8_t *temp_verify_nonce)
{
	uint16_t temp_nonce = verify_nonce;

	if(temp_nonce == 0) {
		temp_verify_nonce[0] = verify_nonce_cntr-1;
		temp_nonce = 0xffff;
	} else {
		temp_nonce--;
		temp_verify_nonce[0] = verify_nonce_cntr;
	}

	set16(temp_verify_nonce, 1, temp_nonce);
}
/*-----------------------------------------------------------------------------------*/
void __attribute__((__far__))
incrementSendTries(void)
{
	if(send_tries > MAX_WAIT_TIME_SEND) {
		/* Back to idle state if we didn't get response */
		if(key_exchange_state < S_KEY_EXCHANGE_IDLE) key_exchange_state = S_KEY_EXCHANGE_FAILED;
		else key_exchange_state = S_UPDATE_FAILED;
		send_tries = 0;
	}
	else send_tries++;
}
/*-----------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------*/
/**
 * Initialization function
 */
/*-----------------------------------------------------------------------------------*/
void __attribute__((__far__))
keymanagement_init(void)
{
	//uint8_t bootstrapKey[16] = {0x42,0x96,0x75,0x58,0xa0,0x41,0x18,0x76,0x3b,0x01,0x56,0xbe,0xc9,0xb4,0x53,0x36}; //node 7
	//uint8_t bootstrapKey[16] = {0x30,0xcf,0xad,0xd4,0x1b,0x2a,0xb9,0x72,0x00,0xa7,0x3c,0x30,0x82,0x00,0xf8,0x12}; //node 2
	uint8_t bootstrapKey[16] = {0x46,0xce,0x52,0x9c,0xe7,0x43,0x9c,0xac,0xc3,0xdf,0xf4,0x17,0x33,0x52,0x1e,0x10}; // node 4
	//uint8_t bootstrapKey[16] = {0xd8,0xdb,0xa2,0x63,0xe1,0x96,0x86,0x81,0xd2,0x05,0x92,0x40,0xf4,0xe3,0xc8,0xc4}; // node 6

	uint8_t temp_sec_nonce_list[KEY_NONCE_SIZE + NONCE_CNTR_SIZE];
	short i = 0;

	/* Check if we have a network key */
	if(!hasKeys) {
		state = S_BOOTSTRAP_KEY;

		/* Set the bootstrap-key for getting key material */
		memcpy(devices[CENTRAL_ENTITY_INDEX].session_key, bootstrapKey, SEC_KEY_SIZE);

		PRINTFBOOT("Boot: key "); for(i=0; i<SEC_KEY_SIZE; i++) {PRINTFBOOT("%02x ", devices[CENTRAL_ENTITY_INDEX].session_key[i]);} PRINTFBOOT("\n");
	}
	else {
		/* State to idle */
		state = S_IDLE;
		key_exchange_state = S_KEY_EXCHANGE_IDLE;
	}

	/* Read the nonce data from flash and ensure freshness */
	xmem_pread(&temp_sec_nonce_list[0], (KEY_NONCE_SIZE + NONCE_CNTR_SIZE), APP_NONCE_DATA);
	PRINTFBOOT("Boot: Reading nonces: "); for(i=0; i<(KEY_NONCE_SIZE + NONCE_CNTR_SIZE); i++) {PRINTFBOOT("%02x ", temp_sec_nonce_list[i]);} PRINTFBOOT("\n");

	/* Set key-exchange nonce counters and increment them */
	temp_sec_nonce_list[0]++;
	temp_sec_nonce_list[1]++;
	request_nonce_cntr = temp_sec_nonce_list[0];
	verify_nonce_cntr = temp_sec_nonce_list[1];

	/* Security trade-off (If nonce value overflows, replay attacks are easy) */
	temp_sec_nonce_list[KEY_NONCE_SIZE]++;

	PRINTFBOOT("Boot: incrementing nonces: req - %x ver - %x nonce - %x\n", request_nonce_cntr, verify_nonce_cntr, temp_sec_nonce_list[KEY_NONCE_SIZE]);
	/* Set reserved spot for temporary security data */
	devices[RESERVED_INDEX].nonce_cntr = 1;
	devices[RESERVED_INDEX].key_freshness = RESERVED;

	/* Set central entity security data */
	devices[CENTRAL_ENTITY_INDEX].nonce_cntr = temp_sec_nonce_list[KEY_NONCE_SIZE];

	/* If nonce counters overflow we need new sensor key */
	if((temp_sec_nonce_list[0] == MAX_NONCE_COUNT) || (temp_sec_nonce_list[1] == MAX_NONCE_COUNT) || (temp_sec_nonce_list[KEY_NONCE_SIZE] == MAX_NONCE_COUNT)) {
		devices[CENTRAL_ENTITY_INDEX].key_freshness = UPDATE_KEY;
	} else {
		devices[CENTRAL_ENTITY_INDEX].key_freshness = UPDATE_NONCE;
	}

	/* Set Server security data */
	devices[SERVER_INDEX].key_freshness = UPDATE_KEY;

	/* Write nonce data back to flash */
	update_key_exchange_nonce = 1;

	/* Central Server, server and reserved */
	amount_of_known_devices = 3;

	PRINTFBOOT("Starting Key management process\n");
	/* Start process */
	process_start(&keymanagement_process, NULL);
}

/*-----------------------------------------------------------------------------------*/
/**
 * Output function for the application layer to create and send an encrypted packet
 * over a specified udp_connection.
 *
 * @param the connection
 * @param the data to be encrypted
 * @param the data length of packet
 * @param the associated data (not encrypted but authenticated)
 * @param the remote ip-address
 * @param the remote udp-port
 * @return encrypt-flags
 */
/*-----------------------------------------------------------------------------------*/
short __attribute__((__far__))
keymanagement_send_encrypted_packet(struct uip_udp_conn *c, uint8_t *data, uint8_t *data_len,
								unsigned short adata_len, uip_ipaddr_t *toaddr, uint16_t toport)
{
	uint8_t i, total_len;
	int dest_index;
	uip_ipaddr_t curr_ip;
	uint8_t tempbuf[*data_len+APP_MIC_LEN+NONCE_SIZE+LENGTH_SIZE];

	/* Check the destination IPv6-address */
	if(uip_is_addr_unspecified(toaddr)) return ENCRYPT_FAILED;
	dest_index = search_device_id(toaddr, 0);

	if(dest_index < 0) {
		/* try to add designated device */
		PRINTFDEBUG("Adding device: "); for(i=0; i<DEVICE_ID_SIZE; i++) {PRINTFDEBUG("%02x ", toaddr->u8[i]);} PRINTFDEBUG("\n");
		dest_index = add_device_id(toaddr);
		return KEY_REQUEST_TX;
	} else if((dest_index == RESERVED_INDEX) && (toport != UIP_HTONS(UDP_CLIENT_SEC_PORT))) {
		PRINTFDEBUG("Reserved device on wrong port\n");
		return KEY_REQUEST_TX;
	} else if((dest_index == CENTRAL_ENTITY_INDEX) && (toport != UIP_HTONS(UDP_SERVER_SEC_PORT))) {
		/* Use session key instead of sensor key */
		dest_index = SERVER_INDEX;
	}

	PRINTFDEBUG("Found device: "); for(i=0; i<DEVICE_ID_SIZE; i++) {PRINTFDEBUG("%02x ", toaddr->u8[i]);} PRINTFDEBUG(" at index: %d\n", dest_index);
	/* Check if the key is still valid */
	if(devices[dest_index].key_freshness == EXPIRED) return KEY_REQUEST_TX;

	/* Check nonce counter value first */
	if(devices[dest_index].nonce_cntr == MAX_NONCE_COUNT) {
		/* Request new key */
		PRINTFDEBUG("Max nonce count reached\n");
		if(dest_index < NON_RESERVED_INDEXES) {
			devices[dest_index].key_freshness = UPDATE_KEY;
		} else {
			devices[dest_index].key_freshness = EXPIRED;
		}
		return KEY_REQUEST_TX;
	}

	/* Check the message counter value */
	if(devices[dest_index].msg_cntr == MAX_MESSAGE_COUNT) {
		/*
		 * Increment the nonce counter, reset message counter
		 * and inform the state machine that the nonce has
		 * to be updated in flash.
		 */
		devices[dest_index].nonce_cntr++;
		devices[dest_index].msg_cntr = 0;
		if(dest_index == CENTRAL_ENTITY_INDEX) {
			devices[dest_index].key_freshness = UPDATE_NONCE;
		}
	}

	/* Get Session key from flash */
	set_session_key_of_index(dest_index);

	/* Get own ip address */
	uip_ds6_select_src(&curr_ip, toaddr);

	/* Extend data packet with nonce */
	for(i=1; i < (MSG_NONCE_SIZE+1); i++) tempbuf[i] = (devices[dest_index].msg_cntr >> ((MSG_NONCE_SIZE-i)*8)) & 0xff;
	tempbuf[0] = devices[dest_index].nonce_cntr;

	/* Set Associated data */
	adata_len = adata_len + NONCE_SIZE;

	/* Copy data to temp buf */
	memcpy(&tempbuf[NONCE_SIZE], data, *data_len);

	total_len = *data_len + NONCE_SIZE;

	PRINTFDEBUG("msg count: %d - nonce count: %d\n", devices[dest_index].msg_cntr, devices[dest_index].nonce_cntr);

	/* Encrypt message */
	if(!cc2420_encrypt_ccm(tempbuf, &curr_ip.u8[0], &devices[dest_index].msg_cntr, &devices[dest_index].nonce_cntr, &total_len, adata_len)) return ENCRYPT_FAILED;

	/* Send packet over udp connection (Increment pointer by 1 to ignore length byte) */
	uip_udp_packet_sendto(c, &tempbuf[1], (int)total_len, toaddr, toport);

	/* Increment message counter if transmission successful!!!!!!!*/
	devices[dest_index].msg_cntr++;

	/* Update the current activity time */
	devices[dest_index].time_last_activity = clock_seconds();

	PRINTFDEBUG("msg count: %d - nonce count: %d\n", devices[dest_index].msg_cntr, devices[dest_index].nonce_cntr);
	//PRINTFDEBUG("Cipher: "); for(i=0; i<(total_len+1); i++) {PRINTFDEBUG("%02x ", tempbuf[1+i]);} PRINTFDEBUG("\n");
	PRINTF("key: Encrypt and send OK\n");

	return ENCRYPT_OK;
}

/*-----------------------------------------------------------------------------------*/
/**
 * Input function of application layer to decrypt messages
 *
 * @param source ip-address
 * @param the encrypted data
 * @param the packet length
 * @param the associated data
 * @return decrypt-flags
 */
/*-----------------------------------------------------------------------------------*/
short __attribute__((__far__))
keymanagement_decrypt_packet(uip_ipaddr_t *remote_device_id, uint8_t *data, uint8_t *data_len, unsigned short adata_len, uint8_t sec_flag)
{
	uint8_t src_nonce_cntr;
	uint8_t i;
	uint16_t src_msg_cntr = 0;

	int src_index;

	/* Check if source address is known */
	src_index = search_device_id(remote_device_id, 0);

	if(src_index == RESERVED_INDEX) {
		PRINTFDEBUG("src_index RESERVED\n");
	} else if((src_index == CENTRAL_ENTITY_INDEX) && (sec_flag != SEC_FLAG)) {
		PRINTFDEBUG("src_index SERVER\n");
		src_index = SERVER_INDEX;
	} else {
		PRINTFDEBUG("src_index %d\n", src_index);
	}

	if(src_index < 0) return DEVICE_NOT_FOUND_RX;
	PRINTFDEBUG("Found device: "); for(i=0; i<DEVICE_ID_SIZE; i++) {PRINTFDEBUG("%02x ", remote_device_id->u8[i]);} PRINTFDEBUG("\n");

	/* Check if the key is fresh */
	if(devices[src_index].key_freshness == EXPIRED) return KEY_REQUEST_TX;

	/* Check nonce and message counter values */
	for(i=1; i < (MSG_NONCE_SIZE+1); i++) src_msg_cntr |= ((uint16_t)data[i] << ((MSG_NONCE_SIZE-i)*8));
	src_nonce_cntr = data[0];

	PRINTFDEBUG("dec_nonce %d dec_msgcntr %d\n", src_nonce_cntr, src_msg_cntr);

	if(((src_msg_cntr <= devices[src_index].remote_msg_cntr) && (src_nonce_cntr <= devices[src_index].remote_nonce_cntr)) ||
					(src_nonce_cntr < devices[src_index].remote_nonce_cntr)) {
		PRINTFDEBUG("Replay message storeM: %d, recM: %d\n", devices[src_index].remote_msg_cntr, src_msg_cntr);
		return REPLAY_MESSAGE;
	}

	/* Get key for decryption */
	set_session_key_of_index(src_index);

	/* Set Associated data */
	adata_len = adata_len + NONCE_SIZE;

	/* Decrypt message */
	if(!(cc2420_decrypt_ccm(data, &devices[src_index].remote_device_id.u8[0], &src_msg_cntr, &src_nonce_cntr, data_len, adata_len))) return DECRYPT_FAILED;

	PRINTFDEBUG("dec_data: "); for(i=0; i<*data_len; i++) {PRINTFDEBUG("%02x ", data[i]);} PRINTFDEBUG("\n");

	/* Check if authentication was successful */
	if(data[*data_len-1] != AUTHENTICATION_SUCCES) return AUTHENTICATION_FAILED;

	/* Store new values in security data */
	devices[src_index].remote_msg_cntr = src_msg_cntr;
	devices[src_index].remote_nonce_cntr = src_nonce_cntr;

	/* Update the current activity time */
	devices[src_index].time_last_activity = clock_seconds();

	PRINTF("key: Decrypt packet OK\n");
	return DECRYPT_OK;
}

/*-----------------------------------------------------------------------------------*/
/**
 * Key management process
 */
/*-----------------------------------------------------------------------------------*/
__attribute__((__far__))
PROCESS_THREAD(keymanagement_process, ev, data)
{
	static struct etimer periodic;
	uint8_t device_index;

	PROCESS_BEGIN();

	PRINTF("Key management process: started\n");

	/*
	 * new connection with remote host at port 0
	 * to allow multiple remote ports on the same
	 * connection
	 */
	sec_conn = udp_new(NULL, 0, NULL);
	if(sec_conn == NULL) {
		PRINTF("key: No UDP conn, exiting proc!\n");
		PROCESS_EXIT();
	}
	udp_bind(sec_conn, UIP_HTONS(UDP_CLIENT_SEC_PORT));

	/* Periodic checking of state -> is the time overhead big????? or Event based checking-???? */
	etimer_set(&periodic, CHECK_INTERVAL);
	while(1) {
		PROCESS_YIELD();

		if((etimer_expired(&periodic)) || (ev == tcpip_event)) {
			/* Check if there is data to be processed */
			if(uip_newdata()) {
				PRINTF("New data!\n");
				/* Check if we have the right connection */
				if(uip_udp_conn->lport == UIP_HTONS(UDP_CLIENT_SEC_PORT)) {
					switch(state) {
						case S_BOOTSTRAP_KEY:
							/* Drop packet */
							PRINTF("Drop packet\n");
							break;
						case S_UPDATE_KEY:
							/* key-update parser */
							state = receive_key_update((uint8_t *) uip_appdata, uip_datalen());
							break;
						default:
							/* Default parser for key exchange */
							state = receive_callback((uint8_t *) uip_appdata, uip_datalen());
							break;
					}
				}
			} else {
				etimer_reset(&periodic);
			}

			/* Search for changes of nonce data */
			device_index = find_index_for_request(UPDATE_NONCE);
			if(device_index != MAX_DEVICES) {
				update_nonce_flash(device_index);
			}

			/* Search for key update requests */
			device_index = find_index_for_request(UPDATE_KEY);

			/* Search for changes in the state */
			switch(state) {
				case S_IDLE:
					/* Check if there is a key-update */
					if(device_index != MAX_DEVICES) {
						state = S_UPDATE_KEY;
						key_exchange_state = S_UPDATE_REQ;
					} else {
						/* Search for expired session keys */
						device_index = find_index_for_request(EXPIRED);
						if(device_index != MAX_DEVICES) {
							state = S_REQUEST_KEY;
							key_exchange_state = S_INIT_REQUEST;
							copy_id_to_reserved(device_index);
						} else {
							reset_failed_key_exchanges();
						}
					}
					break;

				case S_REQUEST_KEY:
					/* Check if there is a key-update */
					if(device_index != MAX_DEVICES) {
						state = S_UPDATE_KEY;
						key_exchange_state = S_UPDATE_REQ;
					} else {
						/* Proceed with key exchange */
						state = key_exchange_protocol();
					}
					break;

				case S_UPDATE_KEY:
					state = key_update_protocol(device_index);
					break;

				case S_BOOTSTRAP_KEY:
					PRINTFBOOT("State Boot\n");
					state = S_BOOTSTRAP_KEY;
					break;

				default:
					state = S_IDLE;
					break;
			}
		}
	}

	PROCESS_END();
}

/*-----------------------------------------------------------------------------------*/
/**
 * key_exchange_protocol is the main callback (protocol) function that decides if the
 * protocol should continue or stop.
 *
 * @return stop/continue
 */
/*-----------------------------------------------------------------------------------*/
static uint8_t
key_exchange_protocol(void)
{
#if DEBUG_SEC
	PRINTFDEBUG("key: exchange state %s\n", printstate[key_exchange_state]);
#endif
	/* Is there anything to send? */
	if(key_exchange_state == S_KEY_EXCHANGE_SUCCES) {
		/* Key exchange is finished */
		PRINTF("key: Success\n");

		/* Store security data */
		store_reserved_sec_data();
		key_exchange_state = S_KEY_EXCHANGE_IDLE;
		return S_REQUEST_KEY;

	} else if(key_exchange_state == S_KEY_EXCHANGE_FAILED) {
		PRINTF("key: Failed\n");
		/* Increment fails of requested device */
		int device_index = search_device_id(&devices[RESERVED_INDEX].remote_device_id, NON_RESERVED_INDEXES);
		if(!(device_index < 0)) {
			devices[device_index].key_freshness = FAILED;
		}
		/* Key exchange failed */
		key_exchange_state = S_KEY_EXCHANGE_IDLE;
		return S_REQUEST_KEY;

	} else if(key_exchange_state == S_KEY_EXCHANGE_IDLE) {
		/* Reset RESERVED device */
		reset_sec_data(RESERVED_INDEX);

		/* Reset device id */
		memset(&devices[RESERVED_INDEX].remote_device_id.u8[0], 0, DEVICE_ID_SIZE);
		return S_IDLE;

	}

	/* Create and send protocol message */
	send_key_exchange_packet();

	/* Increment send tries */
	incrementSendTries();

	return S_REQUEST_KEY;
}

/*-----------------------------------------------------------------------------------*/
/**
 * key_update_protocol is the main callback (protocol) function that decides if the
 * protocol should continue or stop.
 *
 * @return stop/continue
 */
/*-----------------------------------------------------------------------------------*/
static uint8_t
key_update_protocol(uint8_t req_device)
{
	/* Is there anything to send? */
	if(key_exchange_state == S_UPDATE_SUCCES) {
		/* Key exchange is finished */
		PRINTF("key: Update Success\n");
		devices[req_device].key_freshness = UPDATE_NONCE;

		/* Store security data */
		key_exchange_state = S_KEY_EXCHANGE_IDLE;
		return S_IDLE;

	} else if(key_exchange_state == S_UPDATE_FAILED) {
		PRINTF("key: Update Failed\n");
		/* Time-out after too many fails */

		/* Key exchange failed */
		key_exchange_state = S_KEY_EXCHANGE_IDLE;
		return S_IDLE;
	}

	/* Create and send protocol message */
	send_key_update_packet(req_device);

	/* Increment send tries */
	incrementSendTries();

	return S_UPDATE_KEY;
}

/*-----------------------------------------------------------------------------------*/
/**
 * Key-exchange output function. Creates and sends a protocol message according
 * to the current state.
 */
/*-----------------------------------------------------------------------------------*/
static void
send_key_exchange_packet(void)
{
	short i = 0;
	keypacketbuf[0] = key_exchange_state;
	tot_len = 1;

	/* Check if still need to send */
	if(send_tries >= MAX_SEND_TRIES) return;

	switch(key_exchange_state) {
		case S_INIT_REQUEST:
			PRINTF("key: Sending request\n");
			/* Send packet to remote device */
			PRINTFDEBUG("key: plain "); for(i=0; i<tot_len; i++) {PRINTFDEBUG("%02x", keypacketbuf[i]);} PRINTFDEBUG("\n");
			uip_udp_packet_sendto(sec_conn, keypacketbuf, tot_len, &devices[RESERVED_INDEX].remote_device_id, UIP_HTONS(UDP_CLIENT_SEC_PORT));
			break;

		case S_INIT_REPLY:	/* | request_nonce(3) | */
			PRINTF("key: Sending reply to request\n");
			/* Create message */
			init_reply_message();

			PRINTFDEBUG("key: plain "); for(i=0; i<tot_len; i++) {PRINTFDEBUG("%02x", keypacketbuf[i]);} PRINTFDEBUG("\n");
			/* Send packet to remote device */
			uip_udp_packet_sendto(sec_conn, keypacketbuf, tot_len, &devices[RESERVED_INDEX].remote_device_id, UIP_HTONS(UDP_CLIENT_SEC_PORT));
			break;

		case S_COMM_REQUEST: /* | id curr(16) | id remote(16) | request_nonce(3) | remote request nonce(3) | */
			PRINTF("key: Sending communication request to server\n");
			/* Create message */
			comm_request_message();

			PRINTFDEBUG("key: plain "); for(i=0; i<tot_len; i++) {PRINTFDEBUG("%02x", keypacketbuf[i]);} PRINTFDEBUG("\n");
			/* Send packet to edge-router */
			uip_udp_packet_sendto(sec_conn, keypacketbuf, tot_len, &devices[CENTRAL_ENTITY_INDEX].remote_device_id, UIP_HTONS(UDP_SERVER_SEC_PORT));
			break;

		case S_VERIFY_REQUEST: /* | Ek{verify nonce} | */
			if(send_tries > 1) {
				PRINTF("key: Sending verify request\n");
				/* Create message */
				verify_request_message();

				PRINTFDEBUG("key: plain "); for(i=0; i<tot_len; i++) {PRINTFDEBUG("%02x", keypacketbuf[i]);} PRINTFDEBUG("\n");
				/* Send encrypted packet to remote device */
				keymanagement_send_encrypted_packet(sec_conn, keypacketbuf, &tot_len, ADATA_KEYEXCHANGE,
														&devices[RESERVED_INDEX].remote_device_id, UIP_HTONS(UDP_CLIENT_SEC_PORT));
			}
			break;

		case S_VERIFY_REPLY: /* | Ek{verify nonce-1} | */
			PRINTF("key: Sending verify reply\n");
			/* Create message */
			if(send_tries < 1) verify_reply_message(remote_verify_nonce);

			PRINTFDEBUG("key: plain "); for(i=0; i<tot_len; i++) {PRINTFDEBUG("%02x", keypacketbuf[i]);} PRINTFDEBUG("\n");
			/* Send encrypted packet to remote device */
			keymanagement_send_encrypted_packet(sec_conn, keypacketbuf, &tot_len, ADATA_KEYEXCHANGE,
													&devices[RESERVED_INDEX].remote_device_id, UIP_HTONS(UDP_CLIENT_SEC_PORT));

			/* Choose next state */
			if(send_tries == MAX_SEND_TRIES-1) {
				key_exchange_state = S_KEY_EXCHANGE_SUCCES;
			}
			break;

		default:
			break;
	}
}

/*-----------------------------------------------------------------------------------*/
/**
 *	Set keypacketbuf with init reply message
 */
/*-----------------------------------------------------------------------------------*/
static void
init_reply_message(void)
{
	set16(keypacketbuf, 2, request_nonce);
	keypacketbuf[1] = request_nonce_cntr;
	tot_len = INIT_REPLY_MSG_SIZE;
}

/*-----------------------------------------------------------------------------------*/
/**
 *	Set keypacketbuf with communication request message
 */
/*-----------------------------------------------------------------------------------*/
static void
comm_request_message(void)
{
	uip_ipaddr_t curr_ip;

	/* Get own ip address */
	uip_ds6_select_src(&curr_ip, &devices[CENTRAL_ENTITY_INDEX].remote_device_id);

	/* Copy own ID */
	memcpy(&keypacketbuf[1], &curr_ip.u8[0], DEVICE_ID_SIZE);
	/* Copy remote ID */
	memcpy(&keypacketbuf[17], &devices[RESERVED_INDEX].remote_device_id.u8[0], DEVICE_ID_SIZE);
	/* Copy request nonce */
	set16(keypacketbuf, 34, request_nonce);
	keypacketbuf[33] = request_nonce_cntr;
	/* Copy remote request nonce */
	memcpy(&keypacketbuf[36], remote_request_nonce, (KEY_NONCE_SIZE+NONCE_CNTR_SIZE));

	tot_len = 39;
}

/*-----------------------------------------------------------------------------------*/
/**
 *	Set keypacketbuf with verify request message
 */
/*-----------------------------------------------------------------------------------*/
void __attribute__((__far__))
verify_request_message(void)
{
	/* Copy verify nonce */
	set16(keypacketbuf, 2, verify_nonce);
	keypacketbuf[1] = verify_nonce_cntr;
	/* Pad buf with zero for min block size AES*/
	memset(&keypacketbuf[4], 0, 12);

	tot_len = 16;
}

/*-----------------------------------------------------------------------------------*/
/**
 *	Set keypacketbuf with verify reply message
 */
/*-----------------------------------------------------------------------------------*/
void __attribute__((__far__))
verify_reply_message(uint8_t* remote_nonce)
{
	uint16_t temp_rverify_nonce;
	temp_rverify_nonce = get16(remote_nonce, 1);

	/* Subtract 1 from the remote verify nonce */
	if(temp_rverify_nonce == 0) {
		temp_rverify_nonce = 0xffff;
		remote_nonce[0]--;
	} else {
		temp_rverify_nonce--;
	}
	set16(remote_nonce, 1, temp_rverify_nonce);

	/* Copy remote verify nonce */
	memcpy(&keypacketbuf[1], remote_nonce, (KEY_NONCE_SIZE+NONCE_CNTR_SIZE));
	/* Pad buf with zero for min block size AES*/
	memset(&keypacketbuf[4], 0, 12);

	tot_len = 16;
}

/*-----------------------------------------------------------------------------------*/
/**
 * Key-update output function. Creates and sends a protocol message according
 * to the current state.
 */
/*-----------------------------------------------------------------------------------*/
static void
send_key_update_packet(uint8_t req_device)
{
	/* Check if still need to send */
	if(send_tries >= MAX_SEND_TRIES) return;

	switch(key_exchange_state) {
		case S_UPDATE_REQ: /* | encrypted nonce(3) | msg_type(1) | request_nonce(3) | padding(12) | MAC(8) */
			PRINTF("key: Sending update request\n");
			/* Assemble req packet */
			/* MSG_type: 0 = SENSOR UPDATE, 1 = SESSION UPDATE */
			keypacketbuf[0] = UPDATE_REQ | req_device;
			verify_request_message();

			/* Send encrypted packet to remote server */
			keymanagement_send_encrypted_packet(sec_conn, keypacketbuf, &tot_len, ADATA_KEYEXCHANGE,
													&devices[CENTRAL_ENTITY_INDEX].remote_device_id, UIP_HTONS(UDP_SERVER_SEC_PORT));
			break;

		case S_UPDATE_REPLY_OK:	/* | encrypted nonce(3) | msg_type(1) | request_nonce(3) - 1 | padding(12) | MAC(8) */
			PRINTF("key: Sending OK reply\n");
			/* Create message */
			/* MSG_type: 0 = SENSOR UPDATE, 1 = SESSION UPDATE */
			keypacketbuf[0] = UPDATE_OK | req_device;
			if(send_tries < 1) verify_reply_message(remote_server_nonce);

			uint8_t i;
			PRINTFDEBUG("key: plain "); for(i=0; i<tot_len; i++) {PRINTFDEBUG("%02x", keypacketbuf[i]);} PRINTFDEBUG("\n");
			/* Send encrypted packet to remote server */
			keymanagement_send_encrypted_packet(sec_conn, keypacketbuf, &tot_len, ADATA_KEYEXCHANGE,
													&devices[CENTRAL_ENTITY_INDEX].remote_device_id, UIP_HTONS(UDP_SERVER_SEC_PORT));

			/* Choose next state */
			if(send_tries == MAX_SEND_TRIES-1) {
				key_exchange_state = S_UPDATE_SUCCES;
			}
			break;

		default:
			break;
	}
}

/*-----------------------------------------------------------------------------------*/
/**
 * The parse function dissects the incoming messages according to the
 * current state. It also serves as next-state function for the protocol.
 *
 * @param udp payload data
 * @param udp packet lenght
 * @return failed/successful
 *
 * After specific time every step has to return to key exchange idle!
 *
 */
/*-----------------------------------------------------------------------------------*/
static short
receive_callback(uint8_t *data, uint16_t len)
{
	uint8_t temp_data_len = len & 0xff;
	uint8_t temp_verify_nonce[3];
	int device_index;

	PRINTFDEBUG("len: %d\n", len);
	PRINTFDEBUG("msg_type: %02x\n", data[3]);

	/* --------------------------- */
	/* Look for key update packets */
	/* --------------------------- */
	if(check_for_update_packet(data, &temp_data_len, UPDATE_MSG_SIZE) == S_UPDATE_KEY) return S_UPDATE_KEY;

	/* ----------------------------- */
	/* Look for key exchange packets */
	/* ----------------------------- */
	switch(key_exchange_state) {
		case S_KEY_EXCHANGE_IDLE:
			if(data[0] == S_INIT_REQUEST && len == INIT_REQUEST_MSG_SIZE) {
				PRINTF("key: Got key-exchange request\n");
				/* Check if we know the source */
				device_index = search_device_id(&UIP_IP_BUF->srcipaddr, NON_RESERVED_INDEXES);
				if(device_index < 0) {
					memcpy(&devices[RESERVED_INDEX].remote_device_id.u8[0], &UIP_IP_BUF->srcipaddr.u8[0], DEVICE_ID_SIZE);
				} else {
					copy_id_to_reserved((uint8_t)device_index);
				}
				/* If there is a valid request we need to reply */
				key_exchange_state = S_INIT_REPLY;
				/* Send tries reset */
				send_tries = 0;
			}
			break;

		case S_INIT_REQUEST:
			if(data[0] == S_INIT_REPLY && len == INIT_REPLY_MSG_SIZE && send_tries > 0) {
				PRINTF("key: Got request reply\n");
				/* Check the remote device id */
				if(memcmp(&UIP_IP_BUF->srcipaddr.u8[0], &devices[RESERVED_INDEX].remote_device_id.u8[0], DEVICE_ID_SIZE) == 0) {
					/* Get the remote nonce */
					memcpy(&remote_request_nonce[0], &data[1], 3);

					key_exchange_state = S_COMM_REQUEST;
					/* Send tries reset */
					send_tries = 0;
				}
			}
			break;

		case S_INIT_REPLY:	   /* | request_nonce(3) | */
			if(data[MSG_TYPE_OFFSET] == S_COMM_REPLY && len == COMM_REPLY_MSG_SIZE) {
				if(keymanagement_decrypt_packet(&UIP_IP_BUF->srcipaddr, data, &temp_data_len, ADATA_KEYEXCHANGE, SEC_FLAG) == DECRYPT_OK) {
					PRINTF("key: Got reply from server\n");
					/* Parse packet */
					if(parse_comm_reply_message(data)) {
						/* Send verify message */
						key_exchange_state = S_COMM_REPLY;
						/* Send tries reset */
						send_tries = 0;
					}
				}
			}
			break;

		case S_COMM_REQUEST:   /* | remote_decryption_nonce(3) | msg_type(1) | request_nonce(3) | sessionkey(16) | id remote(16) | MIC(8) | */
			if(data[MSG_TYPE_OFFSET] == S_COMM_REPLY && len == COMM_REPLY_MSG_SIZE) {
				if(keymanagement_decrypt_packet(&UIP_IP_BUF->srcipaddr, data, &temp_data_len, ADATA_KEYEXCHANGE, SEC_FLAG) == DECRYPT_OK) {
					PRINTF("key: Got reply from server\n");
					/* Parse packet */
					if(parse_comm_reply_message(data)) {
						/* Wait for Verify message */
						key_exchange_state = S_VERIFY_REQUEST;
						/* Send tries reset */
						send_tries = 0;
					}
				}
			}
			break;

		case S_COMM_REPLY:
			if(data[MSG_TYPE_OFFSET] == S_VERIFY_REQUEST && len == VERIFY_REQUEST_MSG_SIZE && send_tries > 0) {
				if(keymanagement_decrypt_packet(&UIP_IP_BUF->srcipaddr, data, &temp_data_len, ADATA_KEYEXCHANGE, SEC_FLAG) == DECRYPT_OK) {
					PRINTF("key: Got verification request\n");
					/* Store verify nonce */
					memcpy(&remote_verify_nonce[0], &data[4], (KEY_NONCE_SIZE+NONCE_CNTR_SIZE));
					/* reply to verify message */
					key_exchange_state = S_VERIFY_REPLY;
					/* Send tries reset */
					send_tries = 0;
				}
			}
			break;

		case S_VERIFY_REQUEST: /* | Ek{verify nonce} | */
			if(data[MSG_TYPE_OFFSET] == S_VERIFY_REPLY && len == VERIFY_REPLY_MSG_SIZE) {
				if(keymanagement_decrypt_packet(&UIP_IP_BUF->srcipaddr, data, &temp_data_len, ADATA_KEYEXCHANGE, SEC_FLAG) == DECRYPT_OK) {
					/* Decrement verify request nonce */
					PRINTF("key: Got reply verification\n");
					get_decrement_verify_nonce(temp_verify_nonce);
					PRINTFDEBUG("temp: %02x %02x %02x\n", temp_verify_nonce[0], temp_verify_nonce[1], temp_verify_nonce[2]);
					PRINTFDEBUG("rece: %02x %02x %02x\n", data[4], data[5], data[6]);
					/* Compare verify reply nonce */
					if(memcmp(&temp_verify_nonce[0], &data[4], (KEY_NONCE_SIZE+NONCE_CNTR_SIZE)) == 0) {
						/* Increment verify nonce */
						increment_verify_nonce();
						/* Send tries reset */
						send_tries = 0;
						/* Choose next state */
						key_exchange_state = S_KEY_EXCHANGE_SUCCES;
					}
				}
			}
			break;

		default:
			break;
	}

	return S_REQUEST_KEY;
}

/*-----------------------------------------------------------------------------------*/
/**
 * The receive key update function dissects the incoming messages according to the
 * current update state. It also serves as next-state function for the protocol.
 *
 * @param udp payload data
 * @param udp packet lenght
 * @return failed/successful
 *
 * After specific time every step has to return to key exchange idle!
 *
 */
/*-----------------------------------------------------------------------------------*/
static short
receive_key_update(uint8_t *data, uint16_t len)
{
	uint8_t temp_data_len = len & 0xff;

	PRINTFDEBUG("len: %d\n", len);
	PRINTFDEBUG("msg_type: %02x\n", data[3]);

	/* ---------------------------------------- */
	/* Look for key update packets from server  */
	/* They have absolute priority				*/
	/* ---------------------------------------- */
	check_for_update_packet(data, &temp_data_len, UPDATE_MSG_SIZE);

	switch(key_exchange_state) {
		case S_UPDATE_REQ: /* encryption_nonce(3) | msg_type(1) | server_nonce(3) | encrypted_key(16) | encrypted_req_nonce(3) | MAC(8) */
			return check_for_update_packet(data, &temp_data_len, UPDATE_REPLY_MSG_SIZE);
			break;

		default:
			break;
	}

	return S_UPDATE_KEY;
}

/*-----------------------------------------------------------------------------------*/
/**
 *	Help function to check if we received update.
 *
 *	@param pointer to data
 *	@param length of packet
 *	@param length to check
 *	@return state
 */
/*-----------------------------------------------------------------------------------*/
uint8_t __attribute__((__far__))
check_for_update_packet(uint8_t *data, uint8_t* len, uint8_t messageLength)
{
	if(((data[MSG_TYPE_OFFSET] & 0xF0) == UPDATE_MSG) && (*len == messageLength)) {
		/* Check the destination information */
		if(memcmp(&UIP_IP_BUF->srcipaddr.u8[0], &devices[CENTRAL_ENTITY_INDEX].remote_device_id.u8[0], DEVICE_ID_SIZE) != 0) {
			/* Wrong ip address -> continue normal operation */
			return state;
		}
		/* Decrypt message */
		if(keymanagement_decrypt_packet(&UIP_IP_BUF->srcipaddr, data, len, ADATA_KEYEXCHANGE, SEC_FLAG) == DECRYPT_OK) {
			PRINTF("key: Got key update\n");
			/* Parse packet */
			if(!parse_update_message(data)) return state;

			key_exchange_state = S_UPDATE_REPLY_OK;
			/* Send tries reset */
			send_tries = 0;
			return S_UPDATE_KEY;
		}
	}

	return state;
}

/*-----------------------------------------------------------------------------------*/
/**
 *	Help function to parse the content of update reply message.
 *
 *	@param pointer to data
 *	@return failed/successful
 */
/*-----------------------------------------------------------------------------------*/
static uint8_t
parse_update_message(uint8_t *data) /* encryption_nonce(3) | msg_type(1) | server_nonce(3) | encrypted_key(16) | **encrypted_req_nonce(3) | MAC(8) */
{
	uint8_t msg_type = (data[MSG_TYPE_OFFSET] & 0x0F);
	uint8_t temp_verify_nonce[3];

	if(msg_type <= S_UPDATE_SESSION) {
		PRINTF("key: parse update\n");
		if(key_exchange_state == S_UPDATE_REQ) {
			/* If we started the request we need to check our request nonce */
			get_decrement_verify_nonce(temp_verify_nonce);

			/* Compare verify reply nonce */
			if(memcmp(&temp_verify_nonce[0], &data[4+(KEY_NONCE_SIZE+NONCE_CNTR_SIZE)+SEC_KEY_SIZE], (KEY_NONCE_SIZE+NONCE_CNTR_SIZE)) == 0) {
				/* Increment verify nonce */
				increment_verify_nonce();
			} else {
				return 0;
			}
		}

		/* Set new key */
		memcpy(&devices[msg_type].session_key[0], &data[4+(KEY_NONCE_SIZE+NONCE_CNTR_SIZE)], SEC_KEY_SIZE);
		reset_sec_data(msg_type);
		devices[msg_type].key_freshness = UPDATE_KEY;

		/* If sensor key -> update flash */
		if(msg_type == S_UPDATE_SENSOR) update_sensor_key_flash();

		/* Store Server nonce */
		memcpy(&remote_server_nonce[0], &data[4], (KEY_NONCE_SIZE+NONCE_CNTR_SIZE));

		PRINTF("key: parse update success\n");
	} else {
		return 1;
	}
	return 1;
}

/*-----------------------------------------------------------------------------------*/
/**
 *	Help function to parse the content of communication reply message.
 *
 *	@param pointer to data
 *	@return failed/successful
 */
/*-----------------------------------------------------------------------------------*/
#define ID_OFFSET				23
#define SESSIONKEY_OFFSET		7
#define REQUEST_NONCE_OFFSET	4

uint8_t __attribute__((__far__))
parse_comm_reply_message(uint8_t *data)
{
	uint8_t temp_request_nonce[3];

	/* Assemble request nonce */
	set16(temp_request_nonce, 0, request_nonce);
	temp_request_nonce[2] = request_nonce_cntr;

	/* Check request nonce */
	if(memcmp(&data[REQUEST_NONCE_OFFSET], &temp_request_nonce[0], (KEY_NONCE_SIZE+NONCE_CNTR_SIZE)) != 0) {
		/* Doesn't belong with current request - replay message */
		PRINTF("key: wrong req_nonce\n");
		return 0;
	}

	/* Check remote device id */
	if(memcmp(&data[ID_OFFSET], &devices[RESERVED_INDEX].remote_device_id.u8[0], DEVICE_ID_SIZE) != 0) {
		/* Wrong id */
		PRINTF("key: wrong id\n");
		return 0;
	}

	/* Store security data */
	reset_sec_data(RESERVED_INDEX);
	memcpy(&devices[RESERVED_INDEX].session_key[0], &data[SESSIONKEY_OFFSET], SEC_KEY_SIZE);

	/* Increment request nonce */
	increment_request_nonce();

	PRINTF("key: Parse ok\n");

	return 1;
}

/*-----------------------------------------------------------------------------------*/
/**
 *	Store the temporary security data in a free spot if not found
 */
/*-----------------------------------------------------------------------------------*/
static void
store_reserved_sec_data(void)
{
	int index;

	index = search_device_id(&devices[RESERVED_INDEX].remote_device_id, NON_RESERVED_INDEXES);
	if(index < 0) {
		/* Make room for new device */
		index = remove_least_active_device();
	}

	/* store security device data */
	devices[index] = devices[RESERVED_INDEX];
	devices[index].key_freshness = FRESH;

	/* Reset RESERVED id */
	memset(&devices[RESERVED_INDEX].remote_device_id.u8[0], 0, DEVICE_ID_SIZE);
}

/*-----------------------------------------------------------------------------------*/
/**
 * add the given device id to secured communication
 */
/*-----------------------------------------------------------------------------------*/
static int
add_device_id(uip_ipaddr_t* curr_device_id)
{
	int index = DEVICE_NOT_FOUND;

	/* Make room for new device */
	if(amount_of_known_devices == MAX_DEVICES) {
		index = remove_least_active_device();
		PRINTFDEBUG("Removed device index: %d\n", index);
	}

	/* Add device to known devices */
	index = find_index_for_request(FREE_SPOT);
	memcpy(&devices[index].remote_device_id.u8[0], &curr_device_id->u8[0], DEVICE_ID_SIZE);
	amount_of_known_devices++;

	/* Set key_freshness to expired to force request new key */
	devices[index].key_freshness = EXPIRED;

	return index;
}

/*-----------------------------------------------------------------------------------*/
/**
 *	Reset the failed key-exchanges to expired
 */
/*-----------------------------------------------------------------------------------*/
static void
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
 * Update nonce writes the new nonce of devices[index] to flash memory
 *
 * @param index of device
 */
/*-----------------------------------------------------------------------------------*/
static void
update_nonce_flash(uint8_t index)
{
	uint8_t temp_sec_nonce_list[KEY_NONCE_SIZE + NONCE_CNTR_SIZE];

	/* Read the security data from flash and populate nonce list */
	xmem_pread(&temp_sec_nonce_list[0], (KEY_NONCE_SIZE + NONCE_CNTR_SIZE), APP_NONCE_DATA);

	/* Update nonce */
	if(index == CENTRAL_ENTITY_INDEX) {
		temp_sec_nonce_list[KEY_NONCE_SIZE] = devices[index].nonce_cntr;
		/* Clear request */
		devices[index].key_freshness = FRESH;
		PRINTFDEBUG("Updating central server nonce %x\n", temp_sec_nonce_list[KEY_NONCE_SIZE]);
	} else if(index == (MAX_DEVICES+1)) {
		temp_sec_nonce_list[0] = request_nonce_cntr;
		temp_sec_nonce_list[1] = verify_nonce_cntr;
		/* Clear request */
		update_key_exchange_nonce = 0;
		PRINTFDEBUG("Updating key verify/request nonce req-%x ver-%x\n", temp_sec_nonce_list[0], temp_sec_nonce_list[1]);
	} else {
		devices[index].key_freshness = FRESH;
		return;
	}

	/* Write back to flash memory */
	xmem_erase(XMEM_ERASE_UNIT_SIZE, APP_NONCE_DATA);
	xmem_pwrite(&temp_sec_nonce_list[0], (KEY_NONCE_SIZE + NONCE_CNTR_SIZE), APP_NONCE_DATA);
}

/*-----------------------------------------------------------------------------------*/
/**
 * Update sensor key writes new key of devices[CENTRAL_ENTITY_INDEX] to flash memory
 */
/*-----------------------------------------------------------------------------------*/
static void
update_sensor_key_flash(void)
{
	uint8_t temp_sec_data[SEC_KEY_SIZE*3];

	/* Read security data from Flash mem */
	xmem_pread(temp_sec_data, (SEC_KEY_SIZE*3), MAC_SECURITY_DATA);

	/* Copy new sensor key */
	memcpy(&temp_sec_data[(SEC_KEY_SIZE*2)], &devices[CENTRAL_ENTITY_INDEX].session_key[0], SEC_KEY_SIZE);

	/* Write back to flash memory */
	xmem_erase(XMEM_ERASE_UNIT_SIZE, MAC_SECURITY_DATA);
	xmem_pwrite(&temp_sec_data[0], (SEC_KEY_SIZE*3), MAC_SECURITY_DATA);
}

#endif