/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
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
 *         Header file for the key management protocol version 1
 * \author
 *         Ruben Smeets
 */

#ifndef KEYMANAGEMENT_V1_H_
#define KEYMANAGEMENT_V1_H_

#include "contiki-net.h"

void keymanagement_init(void);
short keymanagement_send_encrypted_packet(struct uip_udp_conn *c, uint8_t *data, uint8_t *data_len, unsigned short adata_len, uip_ipaddr_t *toaddr, uint16_t toport);
short keymanagement_decrypt_packet(uip_ipaddr_t *remote_device_id, uint8_t *data, uint8_t *data_len, unsigned short adata_len);

//#define ENCRYPT_OK 			0
//#define ENCRYPT_FAILED 		1
//#define DECRYPT_OK 			2
//#define DECRYPT_FAILED 		3
//#define KEY_REQUEST_TX 		4
//#define NO_SPACE_FOR_DEVICE 5
//#define KEY_MANAGE_BUSY 	6

typedef enum {
  /**< The key management layer encryption was OK. */
  ENCRYPT_OK,

  /**< The key management layer encryption failed. */
  ENCRYPT_FAILED,

  /**< The key management layer decryption was OK. */
  DECRYPT_OK,

  /**< The key management layer decryption failed. */
  DECRYPT_FAILED,

  /**< The key management layer is requesting key. */
  KEY_REQUEST_TX,

  /**< The key management can't add device because there is no space. */
  NO_SPACE_FOR_DEVICE,

  /**< Key management is busy. */
  KEY_MANAGE_BUSY,

  /**< No security data found for device */
  DEVICE_NOT_FOUND_RX,

  /**< Replay message */
  REPLAY_MESSAGE,

  /**< Authentication failed */
  AUTHENTICATION_FAILED,

} keymanagement_flags_type_t;


#endif /* KEYMANAGEMENT_V1_H_ */
