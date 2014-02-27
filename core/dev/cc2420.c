/*
 * Copyright (c) 2007, Swedish Institute of Computer Science
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
/*
 * This code is almost device independent and should be easy to port.
 */

#include <string.h>

#include "contiki.h"

#if defined(__AVR__)
#include <avr/io.h>
#endif

#include "dev/leds.h"
#include "dev/spi.h"
#include "dev/cc2420.h"
#include "dev/cc2420_const.h"

#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/netstack.h"

#define WITH_SEND_CCA 1

#define FOOTER_LEN 2

#ifndef CC2420_CONF_CHECKSUM
#define CC2420_CONF_CHECKSUM 0
#endif /* CC2420_CONF_CHECKSUM */

#ifndef CC2420_CONF_CHANNEL
#define CC2420_CONF_CHANNEL 26
#endif /* CC2420_CONF_CHANNEL */

#ifndef CC2420_CONF_CCA_THRESH
#define CC2420_CONF_CCA_THRESH -45
#endif /* CC2420_CONF_CCA_THRESH */


#ifndef CC2420_CONF_AUTOACK
#define CC2420_CONF_AUTOACK 0
#endif /* CC2420_CONF_AUTOACK */

#if CC2420_CONF_CHECKSUM
#include "lib/crc16.h"
#define CHECKSUM_LEN 2
#else
#define CHECKSUM_LEN 0
#endif /* CC2420_CONF_CHECKSUM */

#define AUX_LEN (CHECKSUM_LEN + FOOTER_LEN)


#define FOOTER1_CRC_OK      0x80
#define FOOTER1_CORRELATION 0x7f

#define DEBUG_SEC 1
#if DEBUG_SEC
#include <stdio.h>
#define PRINTFSEC(...)
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTDEBUG(...) printf(__VA_ARGS__)
#else
#define PRINTFSEC(...) do {} while (0)
#define PRINTF(...)
#define PRINTDEBUG(...)
#endif

#define DEBUG_LEDS DEBUG
#undef LEDS_ON
#undef LEDS_OFF
#if DEBUG_LEDS
#define LEDS_ON(x) leds_on(x)
#define LEDS_OFF(x) leds_off(x)
#else
#define LEDS_ON(x)
#define LEDS_OFF(x)
#endif

void cc2420_arch_init(void);

/* XXX hack: these will be made as Chameleon packet attributes */
rtimer_clock_t cc2420_time_of_arrival, cc2420_time_of_departure;

int cc2420_authority_level_of_sender;

int cc2420_packets_seen, cc2420_packets_read;

static uint8_t volatile pending;

#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)

volatile uint8_t cc2420_sfd_counter;
volatile uint16_t cc2420_sfd_start_time;
volatile uint16_t cc2420_sfd_end_time;

static volatile uint16_t last_packet_timestamp;
/*---------------------------------------------------------------------------*/
PROCESS(cc2420_process, "CC2420 driver");
/*---------------------------------------------------------------------------*/


int cc2420_on(void);
int cc2420_off(void);

static int cc2420_read(void *buf, unsigned short bufsize);

static int cc2420_prepare(const void *data, unsigned short len);
static int cc2420_transmit(unsigned short len);
static int cc2420_send(const void *data, unsigned short len);

static int cc2420_receiving_packet(void);
static int pending_packet(void);
static int cc2420_cca(void);
/*static int detected_energy(void);*/

#if ENABLE_CBC_LINK_SECURITY
#include "net/sec-arp-client.h"
#define ACK_PACKET_SIZE 	3
static uint8_t mic_len;
inline void cc2420_initLinkLayerSec(void);
#endif

#if ENABLE_CBC_LINK_SECURITY & SEC_EDGE
uint8_t potentialHello;

#define MIN_SIGNAL_STRENGTH 0x10
#define HELLO_PACKET_LENGTH 32
#endif

#if ENABLE_CCM_APPLICATION
#define CC2420_SEC_TXKEYSEL_1 (1<<6)
#define CC2420_SEC_RXKEYSEL_1 (1<<5)
#define RX 1
#define TX 0

static void setAssociatedData(unsigned short RX_nTX, unsigned short hdrlen);
static void setNonce(unsigned short RX_nTX, uint8_t *p_address_nonce, uint32_t *msg_ctr, uint8_t *p_nonce_ctr);
#endif

signed char cc2420_last_rssi;
uint8_t cc2420_last_correlation;

const struct radio_driver cc2420_driver =
  {
    cc2420_init,
    cc2420_prepare,
    cc2420_transmit,
    cc2420_send,
    cc2420_read,
    /* cc2420_set_channel, */
    /* detected_energy, */
    cc2420_cca,
    cc2420_receiving_packet,
    pending_packet,
    cc2420_on,
    cc2420_off,
  };

static uint8_t receive_on;

static int channel;

/*---------------------------------------------------------------------------*/

static void
getrxdata(void *buf, int len)
{
  CC2420_READ_FIFO_BUF(buf, len);
}
static void
getrxbyte(uint8_t *byte)
{
  CC2420_READ_FIFO_BYTE(*byte);
}
static void
flushrx(void)
{
  uint8_t dummy;

  CC2420_READ_FIFO_BYTE(dummy);
  CC2420_STROBE(CC2420_SFLUSHRX);
  CC2420_STROBE(CC2420_SFLUSHRX);
}
/*---------------------------------------------------------------------------*/
static void
strobe(enum cc2420_register regname)
{
  CC2420_STROBE(regname);
}
/*---------------------------------------------------------------------------*/
static unsigned int
status(void)
{
  uint8_t status;
  CC2420_GET_STATUS(status);
  return status;
}
/*---------------------------------------------------------------------------*/
static uint8_t locked, lock_on, lock_off;

static void
on(void)
{
  CC2420_ENABLE_FIFOP_INT();
  strobe(CC2420_SRXON);

  BUSYWAIT_UNTIL(status() & (BV(CC2420_XOSC16M_STABLE)), RTIMER_SECOND / 100);

  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  receive_on = 1;
}
static void
off(void)
{
  /*  PRINTF("off\n");*/
  receive_on = 0;

  /* Wait for transmission to end before turning radio off. */
  BUSYWAIT_UNTIL(!(status() & BV(CC2420_TX_ACTIVE)), RTIMER_SECOND / 10);

  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
  strobe(CC2420_SRFOFF);
  CC2420_DISABLE_FIFOP_INT();

  if(!CC2420_FIFOP_IS_1) {
    flushrx();
  }
}
/*---------------------------------------------------------------------------*/
#define GET_LOCK() locked++
static void RELEASE_LOCK(void) {
  if(locked == 1) {
    if(lock_on) {
      on();
      lock_on = 0;
    }
    if(lock_off) {
      off();
      lock_off = 0;
    }
  }
  locked--;
}
/*---------------------------------------------------------------------------*/
static unsigned
getreg(enum cc2420_register regname)
{
  unsigned reg;
  CC2420_READ_REG(regname, reg);
  return reg;
}
/*---------------------------------------------------------------------------*/
static void
setreg(enum cc2420_register regname, unsigned value)
{
  CC2420_WRITE_REG(regname, value);
}
/*---------------------------------------------------------------------------*/
static void
set_txpower(uint8_t power)
{
  uint16_t reg;

  reg = getreg(CC2420_TXCTRL);
  reg = (reg & 0xffe0) | (power & 0x1f);
  setreg(CC2420_TXCTRL, reg);
}
/*---------------------------------------------------------------------------*/
#define AUTOACK (1 << 4)
#define ADR_DECODE (1 << 11)
#define RXFIFO_PROTECTION (1 << 9)
#define CORR_THR(n) (((n) & 0x1f) << 6)
#define FIFOP_THR(n) ((n) & 0x7f)
#define RXBPF_LOCUR (1 << 13);
/*---------------------------------------------------------------------------*/
int
cc2420_init(void)
{
  uint16_t reg;
  {
    int s = splhigh();
    cc2420_arch_init();		/* Initalize ports and SPI. */
    CC2420_DISABLE_FIFOP_INT();
    CC2420_FIFOP_INT_INIT();
    splx(s);
  }

  /* Turn on voltage regulator and reset. */
  SET_VREG_ACTIVE();
  clock_delay(250);
  SET_RESET_ACTIVE();
  clock_delay(127);
  SET_RESET_INACTIVE();
  clock_delay(125);


  /* Turn on the crystal oscillator. */
  strobe(CC2420_SXOSCON);

  /* Turn on/off automatic packet acknowledgment and address decoding. */
  reg = getreg(CC2420_MDMCTRL0);

#if CC2420_CONF_AUTOACK
  reg |= AUTOACK | ADR_DECODE;
#else
  reg &= ~(AUTOACK | ADR_DECODE);
#endif /* CC2420_CONF_AUTOACK */
  setreg(CC2420_MDMCTRL0, reg);

  /* Set transmission turnaround time to the lower setting (8 symbols
     = 0.128 ms) instead of the default (12 symbols = 0.192 ms). */
  /*  reg = getreg(CC2420_TXCTRL);
  reg &= ~(1 << 13);
  setreg(CC2420_TXCTRL, reg);*/

  
  /* Change default values as recomended in the data sheet, */
  /* correlation threshold = 20, RX bandpass filter = 1.3uA. */
  setreg(CC2420_MDMCTRL1, CORR_THR(20));
  reg = getreg(CC2420_RXCTRL1);
  reg |= RXBPF_LOCUR;
  setreg(CC2420_RXCTRL1, reg);

  /* Set the FIFOP threshold to maximum. */
  setreg(CC2420_IOCFG0, FIFOP_THR(127));

  /* Turn off "Security enable" (page 32). */
#if ENABLE_CBC_LINK_SECURITY
    cc2420_initLinkLayerSec();
#else
    reg = getreg(CC2420_SECCTRL0);
    reg &= ~RXFIFO_PROTECTION;
    setreg(CC2420_SECCTRL0, reg);
#endif

  cc2420_set_pan_addr(0xffff, 0x0000, NULL);
  cc2420_set_channel(CC2420_CONF_CHANNEL);
  cc2420_set_cca_threshold(CC2420_CONF_CCA_THRESH);

  flushrx();

  process_start(&cc2420_process, NULL);
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
cc2420_transmit(unsigned short payload_len)
{
  int i, txpower;
  uint8_t total_len;
#if CC2420_CONF_CHECKSUM
  uint16_t checksum;
#endif /* CC2420_CONF_CHECKSUM */

  GET_LOCK();

  txpower = 0;
  if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
    /* Remember the current transmission power */
    txpower = cc2420_get_txpower();
    /* Set the specified transmission power */
    set_txpower(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) - 1);
  }

  total_len = payload_len + AUX_LEN;
  
  /* The TX FIFO can only hold one packet. Make sure to not overrun
   * FIFO by waiting for transmission to start here and synchronizing
   * with the CC2420_TX_ACTIVE check in cc2420_send.
   *
   * Note that we may have to wait up to 320 us (20 symbols) before
   * transmission starts.
   */
#ifndef CC2420_CONF_SYMBOL_LOOP_COUNT
#error CC2420_CONF_SYMBOL_LOOP_COUNT needs to be set!!!
#else
#define LOOP_20_SYMBOLS CC2420_CONF_SYMBOL_LOOP_COUNT
#endif

#if WITH_SEND_CCA
  strobe(CC2420_SRXON);
  BUSYWAIT_UNTIL(status() & BV(CC2420_RSSI_VALID), RTIMER_SECOND / 10);
  strobe(CC2420_STXONCCA);
#if ENABLE_CBC_LINK_SECURITY
  /* Wait until encryption complete */
  BUSYWAIT_UNTIL(!(status() & BV(CC2420_ENC_BUSY)), RTIMER_SECOND / 10);
#endif

#else /* WITH_SEND_CCA */
  strobe(CC2420_STXON);
#endif /* WITH_SEND_CCA */
  for(i = LOOP_20_SYMBOLS; i > 0; i--) {
    if(CC2420_SFD_IS_1) {
      {
        rtimer_clock_t sfd_timestamp;
        sfd_timestamp = cc2420_sfd_start_time;
        if(packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE) ==
           PACKETBUF_ATTR_PACKET_TYPE_TIMESTAMP) {
          /* Write timestamp to last two bytes of packet in TXFIFO. */
          CC2420_WRITE_RAM(&sfd_timestamp, CC2420RAM_TXFIFO + payload_len - 1, 2);
        }
      }

      if(!(status() & BV(CC2420_TX_ACTIVE))) {
        /* SFD went high but we are not transmitting. This means that
           we just started receiving a packet, so we drop the
           transmission. */
        RELEASE_LOCK();
        return RADIO_TX_COLLISION;
      }
      if(receive_on) {
	ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
      }
      ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
      /* We wait until transmission has ended so that we get an
	 accurate measurement of the transmission time.*/
      BUSYWAIT_UNTIL(!(status() & BV(CC2420_TX_ACTIVE)), RTIMER_SECOND / 10);

#ifdef ENERGEST_CONF_LEVELDEVICE_LEVELS
      ENERGEST_OFF_LEVEL(ENERGEST_TYPE_TRANSMIT,cc2420_get_txpower());
#endif
      ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
      if(receive_on) {
	ENERGEST_ON(ENERGEST_TYPE_LISTEN);
      } else {
	/* We need to explicitly turn off the radio,
	 * since STXON[CCA] -> TX_ACTIVE -> RX_ACTIVE */
	off();
      }

      if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
        /* Restore the transmission power */
        set_txpower(txpower & 0xff);
      }

      RELEASE_LOCK();
      return RADIO_TX_OK;
    }
  }

  /* If we are using WITH_SEND_CCA, we get here if the packet wasn't
     transmitted because of other channel activity. */
  RIMESTATS_ADD(contentiondrop);
  PRINTF("cc2420: do_send() transmission never started\n");

  if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
    /* Restore the transmission power */
    set_txpower(txpower & 0xff);
  }

  RELEASE_LOCK();
  return RADIO_TX_COLLISION;
}
/*---------------------------------------------------------------------------*/
static int
cc2420_prepare(const void *payload, unsigned short payload_len)
{
  uint8_t total_len;
#if CC2420_CONF_CHECKSUM
  uint16_t checksum;
#endif /* CC2420_CONF_CHECKSUM */
  GET_LOCK();

  PRINTF("cc2420: sending %d bytes\n", payload_len);

  RIMESTATS_ADD(lltx);

  /* Wait for any previous transmission to finish. */
  /*  while(status() & BV(CC2420_TX_ACTIVE));*/

  /* Write packet to TX FIFO. */
  strobe(CC2420_SFLUSHTX);

#if CC2420_CONF_CHECKSUM
  checksum = crc16_data(payload, payload_len, 0);
#endif /* CC2420_CONF_CHECKSUM */

#if ENABLE_CBC_LINK_SECURITY
  /* Extend total length with MIC and increment framecounter */
  total_len = payload_len + mic_len + AUX_LEN; //8 Byte MIC
  PRINTFSEC("Pay_len: %d, tot_len: %d\n", payload_len, total_len);
#else
  total_len = payload_len + AUX_LEN;
#endif

  CC2420_WRITE_FIFO_BUF(&total_len, 1);
  CC2420_WRITE_FIFO_BUF(payload, payload_len);
#if CC2420_CONF_CHECKSUM
  CC2420_WRITE_FIFO_BUF(&checksum, CHECKSUM_LEN);
#endif /* CC2420_CONF_CHECKSUM */

  RELEASE_LOCK();
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
cc2420_send(const void *payload, unsigned short payload_len)
{
  cc2420_prepare(payload, payload_len);
  return cc2420_transmit(payload_len);
}
/*---------------------------------------------------------------------------*/
int
cc2420_off(void)
{
  /* Don't do anything if we are already turned off. */
  if(receive_on == 0) {
    return 1;
  }

  /* If we are called when the driver is locked, we indicate that the
     radio should be turned off when the lock is unlocked. */
  if(locked) {
    /*    printf("Off when locked (%d)\n", locked);*/
    lock_off = 1;
    return 1;
  }

  GET_LOCK();
  /* If we are currently receiving a packet (indicated by SFD == 1),
     we don't actually switch the radio off now, but signal that the
     driver should switch off the radio once the packet has been
     received and processed, by setting the 'lock_off' variable. */
  if(status() & BV(CC2420_TX_ACTIVE)) {
    lock_off = 1;
  } else {
    off();
  }
  RELEASE_LOCK();
  return 1;
}
/*---------------------------------------------------------------------------*/
int
cc2420_on(void)
{
  if(receive_on) {
    return 1;
  }
  if(locked) {
    lock_on = 1;
    return 1;
  }

  GET_LOCK();
  on();
  RELEASE_LOCK();
  return 1;
}
/*---------------------------------------------------------------------------*/
int
cc2420_get_channel(void)
{
  return channel;
}
/*---------------------------------------------------------------------------*/
int
cc2420_set_channel(int c)
{
  uint16_t f;

  GET_LOCK();
  /*
   * Subtract the base channel (11), multiply by 5, which is the
   * channel spacing. 357 is 2405-2048 and 0x4000 is LOCK_THR = 1.
   */
  channel = c;

  f = 5 * (c - 11) + 357 + 0x4000;
  /*
   * Writing RAM requires crystal oscillator to be stable.
   */
  BUSYWAIT_UNTIL((status() & (BV(CC2420_XOSC16M_STABLE))), RTIMER_SECOND / 10);

  /* Wait for any transmission to end. */
  BUSYWAIT_UNTIL(!(status() & BV(CC2420_TX_ACTIVE)), RTIMER_SECOND / 10);

  setreg(CC2420_FSCTRL, f);

  /* If we are in receive mode, we issue an SRXON command to ensure
     that the VCO is calibrated. */
  if(receive_on) {
    strobe(CC2420_SRXON);
  }

  RELEASE_LOCK();
  return 1;
}
/*---------------------------------------------------------------------------*/
void
cc2420_set_pan_addr(unsigned pan,
                    unsigned addr,
                    const uint8_t *ieee_addr)
{
  uint16_t f = 0;
  uint8_t tmp[2];

  GET_LOCK();
  
  /*
   * Writing RAM requires crystal oscillator to be stable.
   */
  BUSYWAIT_UNTIL(status() & (BV(CC2420_XOSC16M_STABLE)), RTIMER_SECOND / 10);

  tmp[0] = pan & 0xff;
  tmp[1] = pan >> 8;
  CC2420_WRITE_RAM(&tmp, CC2420RAM_PANID, 2);

  tmp[0] = addr & 0xff;
  tmp[1] = addr >> 8;
  CC2420_WRITE_RAM(&tmp, CC2420RAM_SHORTADDR, 2);
  if(ieee_addr != NULL) {
    uint8_t tmp_addr[8];
    /* LSB first, MSB last for 802.15.4 addresses in CC2420 */
    for (f = 0; f < 8; f++) {
      tmp_addr[7 - f] = ieee_addr[f];
    }
    CC2420_WRITE_RAM(tmp_addr, CC2420RAM_IEEEADDR, 8);
  }
  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
/*
 * Interrupt leaves frame intact in FIFO.
 */
int
cc2420_interrupt(void)
{
  CC2420_CLEAR_FIFOP_INT();
  process_poll(&cc2420_process);

  last_packet_timestamp = cc2420_sfd_start_time;
  pending++;
  cc2420_packets_seen++;
  return 1;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc2420_process, ev, data)
{
  int len;
  PROCESS_BEGIN();

  PRINTF("cc2420_process: started\n");

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    PRINTF("cc2420_process: calling receiver callback\n");

    packetbuf_clear();
    packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, last_packet_timestamp);
    len = cc2420_read(packetbuf_dataptr(), PACKETBUF_SIZE);
    PRINTFSEC("cc2420: len after read %d", len);
    
    packetbuf_set_datalen(len);
    
    NETSTACK_RDC.input();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static int
cc2420_read(void *buf, unsigned short bufsize)
{
  uint8_t footer[2];
  uint8_t len;
  uint8_t *pbuf;
#if CC2420_CONF_CHECKSUM
  uint16_t checksum;
#endif /* CC2420_CONF_CHECKSUM */

  if(!CC2420_FIFOP_IS_1) {
	PRINTFSEC("pin low\n");
    return 0;
  }
  /*  if(!pending) {
    return 0;
    }*/
  
  pending = 0;
  
  GET_LOCK();

  cc2420_packets_read++;

  getrxbyte(&len);
  PRINTFSEC("len: %d\n", len);

#if ENABLE_CBC_LINK_SECURITY & SEC_CLIENT
  /*
   * Check bufsize to know if we are waiting for ACK-packet
   * these packets aren't encrypted and give errors when performing
   * decryption.
   */
  if((len != (ACK_PACKET_SIZE + AUX_LEN)) || (hasKeys != 0)) {
	  strobe(CC2420_SRXDEC);
	  BUSYWAIT_UNTIL(!(status() & BV(CC2420_ENC_BUSY)), RTIMER_SECOND);
  }
#elif ENABLE_CBC_LINK_SECURITY & SEC_EDGE
  /*
   * Check bufsize to know if we are waiting for ACK-packet
   * these packets aren't encrypted and give errors when performing
   * decryption.
   */
  if(len != (ACK_PACKET_SIZE + AUX_LEN)) {
	  strobe(CC2420_SRXDEC);
	  BUSYWAIT_UNTIL(!(status() & BV(CC2420_ENC_BUSY)), RTIMER_SECOND);
  }
#endif

  if(len > CC2420_MAX_PACKET_LEN) {
    /* Oops, we must be out of sync. */
    flushrx();
    RIMESTATS_ADD(badsynch);
    RELEASE_LOCK();
    return 0;
  }

  if(len <= AUX_LEN) {
    flushrx();
    RIMESTATS_ADD(tooshort);
    RELEASE_LOCK();
    return 0;
  }

  if(len - AUX_LEN > bufsize) {
    flushrx();
    RIMESTATS_ADD(toolong);
    RELEASE_LOCK();
    return 0;
  }

#if ENABLE_CBC_LINK_SECURITY & SEC_CLIENT
  /*
   * Check if we are receiving an ACK-packet. They don't have
   * a MIC message appended.
   */

  getrxdata(buf, len - AUX_LEN);
  pbuf = buf;

#if 1
  uint8_t p;
  PRINTDEBUG("R: ");
  PRINTDEBUG("%.2X ", len);for(p = 0; p < len-AUX_LEN; p++)PRINTDEBUG("%.2x", pbuf[p]);PRINTDEBUG("\n");
#endif

  if((len != (ACK_PACKET_SIZE + AUX_LEN)) || (hasKeys != 0)) {
	  if(pbuf[len-(AUX_LEN+1)] != 0x00)
	  {
		  PRINTF("cc2420: FAILED TO AUTHENTICATE\n");
		  flushrx();
		  RELEASE_LOCK();
		  return 0;
	  }
	  PRINTF("cc2420: SUCCESS\n");
  }
#elif ENABLE_CBC_LINK_SECURITY & SEC_EDGE
  /*
   * Check if we are receiving an ACK-packet. They don't have
   * a MIC message appended.
   */
  potentialHello = 0;

  getrxdata(buf, len - AUX_LEN);
  pbuf = buf;

#if 1
  uint8_t p;
  PRINTDEBUG("R: ");
  PRINTDEBUG("%.2X ", len);for(p = 0; p < len-AUX_LEN; p++)PRINTDEBUG("%.2x", pbuf[p]);PRINTDEBUG("\n");
#endif

  if(len != (ACK_PACKET_SIZE + AUX_LEN)) {
	  if(pbuf[len-(AUX_LEN+1)] != 0x00) {
		  if((len - AUX_LEN - mic_len) == HELLO_PACKET_LENGTH) {
			  PRINTF("cc2420: Potential hello packet\n");
			  potentialHello = 1;
		  } else {
			  PRINTF("cc2420: FAILED TO AUTHENTICATE\n");
			  flushrx();
			  RELEASE_LOCK();
			  return 0;
		  }
	  }
	  PRINTF("cc2420: SUCCESS\n");
  }
#else
  getrxdata(buf, len - AUX_LEN);
#endif

#if CC2420_CONF_CHECKSUM
  getrxdata(&checksum, CHECKSUM_LEN);
#endif /* CC2420_CONF_CHECKSUM */
  getrxdata(footer, FOOTER_LEN);

#if CC2420_CONF_CHECKSUM
  if(checksum != crc16_data(buf, len - AUX_LEN, 0)) {
    PRINTF("checksum failed 0x%04x != 0x%04x\n",
	   checksum, crc16_data(buf, len - AUX_LEN, 0));
  }

  if(footer[1] & FOOTER1_CRC_OK &&
     checksum == crc16_data(buf, len - AUX_LEN, 0)) {
#else
  if(footer[1] & FOOTER1_CRC_OK) {
#endif /* CC2420_CONF_CHECKSUM */
    cc2420_last_rssi = footer[0];
    cc2420_last_correlation = footer[1] & FOOTER1_CORRELATION;

#if ENABLE_CBC_LINK_SECURITY & SEC_EDGE
    PRINTDEBUG("cc2420: rssi %02x\n", footer[0]);
    PRINTDEBUG("cc2420: correlation %02x\n", cc2420_last_correlation);

    /* Check the signal strength */
    if(potentialHello == 1) {
		if((!((footer[0] & 0x80)>0)) && ((footer[0] & 0x7F) > MIN_SIGNAL_STRENGTH)) {
			PRINTDEBUG("cc2420: close enough\n");
		} else {
			PRINTDEBUG("cc2420: NOT close enough\n");
			len = 0;
		}
    }
#endif

    packetbuf_set_attr(PACKETBUF_ATTR_RSSI, cc2420_last_rssi);
    packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, cc2420_last_correlation);

    RIMESTATS_ADD(llrx);

  } else {
    RIMESTATS_ADD(badcrc);
    len = AUX_LEN;
  }

  if(CC2420_FIFOP_IS_1) {
    if(!CC2420_FIFO_IS_1) {
      /* Clean up in case of FIFO overflow!  This happens for every
       * full length frame and is signaled by FIFOP = 1 and FIFO =
       * 0. */
      PRINTFSEC("2\n");
      flushrx();
    } else {
      PRINTF("Poll\n");
      /* Another packet has been received and needs attention. */
      process_poll(&cc2420_process);
    }
  }

  RELEASE_LOCK();

  if(len < AUX_LEN) {
	PRINTFSEC("3\n");
    return 0;
  }

#if ENABLE_CBC_LINK_SECURITY
  /*
   * ACK-packet doens't have MIC message appended. Therefore
   * we don't need to subtract the mic-length from the total len.
   */
  if(len != (ACK_PACKET_SIZE + AUX_LEN))  {
	  return len - AUX_LEN - mic_len;
  } else {
	  return len - AUX_LEN;
  }
#else
  return len - AUX_LEN;
#endif
}
/*---------------------------------------------------------------------------*/
void
cc2420_set_txpower(uint8_t power)
{
  GET_LOCK();
  set_txpower(power);
  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
int
cc2420_get_txpower(void)
{
  int power;
  GET_LOCK();
  power = (int)(getreg(CC2420_TXCTRL) & 0x001f);
  RELEASE_LOCK();
  return power;
}
/*---------------------------------------------------------------------------*/
int
cc2420_rssi(void)
{
  int rssi;
  int radio_was_off = 0;

  if(locked) {
    return 0;
  }
  
  GET_LOCK();

  if(!receive_on) {
    radio_was_off = 1;
    cc2420_on();
  }
  BUSYWAIT_UNTIL(status() & BV(CC2420_RSSI_VALID), RTIMER_SECOND / 100);

  rssi = (int)((signed char)getreg(CC2420_RSSI));

  if(radio_was_off) {
    cc2420_off();
  }
  RELEASE_LOCK();
  return rssi;
}
/*---------------------------------------------------------------------------*/
/*
static int
detected_energy(void)
{
  return cc2420_rssi();
}
*/
/*---------------------------------------------------------------------------*/
int
cc2420_cca_valid(void)
{
  int valid;
  if(locked) {
    return 1;
  }
  GET_LOCK();
  valid = !!(status() & BV(CC2420_RSSI_VALID));
  RELEASE_LOCK();
  return valid;
}
/*---------------------------------------------------------------------------*/
static int
cc2420_cca(void)
{
  int cca;
  int radio_was_off = 0;

  /* If the radio is locked by an underlying thread (because we are
     being invoked through an interrupt), we preted that the coast is
     clear (i.e., no packet is currently being transmitted by a
     neighbor). */
  if(locked) {
    return 1;
  }

  GET_LOCK();
  if(!receive_on) {
    radio_was_off = 1;
    cc2420_on();
  }

  /* Make sure that the radio really got turned on. */
  if(!receive_on) {
    RELEASE_LOCK();
    if(radio_was_off) {
      cc2420_off();
    }
    return 1;
  }

  BUSYWAIT_UNTIL(status() & BV(CC2420_RSSI_VALID), RTIMER_SECOND / 100);

  cca = CC2420_CCA_IS_1;

  if(radio_was_off) {
    cc2420_off();
  }
  RELEASE_LOCK();
  return cca;
}
/*---------------------------------------------------------------------------*/
int
cc2420_receiving_packet(void)
{
  return CC2420_SFD_IS_1;
}
/*---------------------------------------------------------------------------*/
static int
pending_packet(void)
{
  return CC2420_FIFOP_IS_1;
}
/*---------------------------------------------------------------------------*/
void
cc2420_set_cca_threshold(int value)
{
  uint16_t shifted = value << 8;
  GET_LOCK();
  setreg(CC2420_RSSI, shifted);
  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
#if ENABLE_CBC_LINK_SECURITY
inline void
cc2420_initLinkLayerSec(void)
{
	uint16_t reg;

	/* Enable key material */
	mic_len = MIC_LEN;

#if ENABLE_CBC_LINK_SECURITY & SEC_EDGE
	potentialHello = 0;
#endif

	/* Set security control register 0 */
	reg = getreg(CC2420_SECCTRL0);
	/* Stand alone key 1, Use 8 bytes MIC, Use RX fifo protection */
	reg |= CC2420_SECCTRL0_SAKEYSEL1 | (CC2420_SECCTRL0_SEC_M_IDX << 2) | CC2420_SECCTRL0_RXFIFO_PROTECTION;
	/* Set TX keyselect to '0' */
	reg &= ~(1<<6);
	/* Set CBC-MAC Mode */
	reg |= CC2420_SECCTRL0_CBC_MAC;

	setreg(CC2420_SECCTRL0, reg);
	PRINTFSEC("cc2420: SEC0 reg: %.2X\n", reg);

	PRINTF("cc2420: Init CBC MAC complete\n");
}
#endif
/*---------------------------------------------------------------------------*/
#if ENABLE_CCM_APPLICATION
/*---------------------------------------------------------------------------*/
static void
setAssociatedData(unsigned short RX_nTX, unsigned short hdrlen)
{
	/* SECCTRL1 must be set correctly to size of MAC HDR (21 + 5) for IEEE802.15.4-2003 */
	uint16_t reg;
	if(RX_nTX) 	reg = (((uint16_t)(hdrlen)) & 0x00ff);
	else 		reg = ((((uint16_t)(hdrlen))<<8) & 0xff00);
	setreg(CC2420_SECCTRL1, reg);
	PRINTFSEC("cc2420: RX_nTX: %d, Associated data: %d, SEC1 reg: %.2X\n, ", RX_nTX, hdrlen, reg);
}
/*---------------------------------------------------------------------------*/
static void
setNonce(unsigned short RX_nTX, uint8_t *p_address_nonce, uint32_t *p_msg_ctr, uint8_t *p_nonce_ctr)
{
	uint8_t nonce[16];
	//uint8_t ieee_addr_temp[8];

	/* Set flags:
	 *		CTR flag (0 0) -> reserved for future expansion
	 *		CBC flag (0 1) -> 7-bit reserved, 6-bit Adata is 1 in my case (not everything is encrypted)
	 *		L (1) 		   -> n+q=15 (n=13)(q=2) l=[q-1]3 | n is the nonce length (see standard)
	 */

	/* NOG MAKEN DAT JE OOK KAN ENCRYPTEREN ZONDER ADATA!!!!!!*/

	nonce[0] =  0x00 | 0x01 | 0x08;
	memcpy(&nonce[1], &p_address_nonce[8], 8);	/* Setting source address using identifier*/
	nonce[9] = 0xFF & (*p_msg_ctr>>24);			/* Setting frame counter */
	nonce[10] = 0xFF & (*p_msg_ctr>>16);
	nonce[11] = 0xFF & (*p_msg_ctr>>8);
	nonce[12] = 0xFF & (*p_msg_ctr);
	nonce[13] = *p_nonce_ctr;			/* Setting key sequence counter (incremented with every new key) */
	nonce[14] = 0x00;					/* Setting MSB of Block counter to 0x00 to be complained with IEEE802.15.4 */
	nonce[15] = 0x01;					/* (only set on initiation) */

	uint8_t i;
	PRINTFSEC("READ NONCE: "); for(i=0; i<16; i++) PRINTFSEC("%.2X ",nonce[i]); PRINTFSEC("\n");

	/* Write Tx Nonce */
	if(RX_nTX) 	CC2420_WRITE_RAM_REV(nonce, CC2420RAM_RXNONCE, 16);
	else		CC2420_WRITE_RAM_REV(nonce, CC2420RAM_TXNONCE, 16);
}
/*---------------------------------------------------------------------------*/
int __attribute__((__far__))
cc2420_decrypt_ccm(uint8_t *data, uint8_t *address_nonce, uint16_t *src_msg_cntr, uint8_t *src_nonce_cntr,
				uint8_t *data_len, unsigned short adata_len)
{
	unsigned int stats;
	uint8_t  tot_len;
	uint16_t reg_old, reg;
	uint32_t tmp_src_msg_cntr = 0;

	/* Check if we are encrypting */
	stats = status();
	if(stats & BV(CC2420_ENC_BUSY)) return 0;

	/* Set security control reg 0 */
	reg_old = getreg(CC2420_SECCTRL0);
	reg = (CC2420_SECCTRL0_SEC_M_IDX << 2) | CC2420_SECCTRL0_RXKEYSEL1 | CC2420_SECCTRL0_RXFIFO_PROTECTION | CC2420_SECCTRL0_CCM;
	PRINTFSEC("cc2420: Reg 0: %.2x\n",reg);

	/* Set associated data RX to 5 */
	setAssociatedData(RX, adata_len);

	/* Set Nonce Rx */
	tmp_src_msg_cntr = (uint32_t)*src_msg_cntr;
	setNonce(RX, address_nonce, &tmp_src_msg_cntr, src_nonce_cntr);

	/* Flush the RXFIFO */
	flushrx();

	/* Set RXFIFO */
	tot_len = *data_len + AUX_LEN;
	CC2420_WRITE_RXFIFO_BUF(&tot_len, 1);
	CC2420_WRITE_RXFIFO_BUF(data, *data_len);

	setreg(CC2420_SECCTRL0, reg);

	/* Decrypt FIFO buffer */
	strobe(CC2420_SRXDEC);
	BUSYWAIT_UNTIL(!(status() & BV(CC2420_ENC_BUSY)), RTIMER_SECOND);

	/* Read RXFIFO buffer */
	getrxbyte(&tot_len);
	getrxdata(data, *data_len);

	/* Restore security control reg 0 */
	setreg(CC2420_SECCTRL0, reg_old);
	PRINTFSEC("cc2420: Reg 0 restore: %.2x\n",reg_old);

	/* Restore security control reg 1 */
	setAssociatedData(RX, 0);

	/* Flush the RXFIFO */
	flushrx();

	return 1;
}
/*---------------------------------------------------------------------------*/
int __attribute__((__far__))
cc2420_encrypt_ccm(uint8_t *data, uint8_t *address_nonce, uint16_t *msg_cntr, uint8_t *nonce_cntr,
				uint8_t *data_len, unsigned short adata_len)
{
	unsigned int stats;
	uint8_t  tot_len;
	uint16_t reg_old, reg;
	uint32_t tmp_msg_cntr = 0;

	/* Check if we are transmitting or encrypting */
	stats = status();
	if((stats & BV(CC2420_ENC_BUSY)) || (stats & BV(CC2420_TX_ACTIVE))) return 0;

	/* Set security control reg 0 */
	reg_old = getreg(CC2420_SECCTRL0);
	reg = (CC2420_SECCTRL0_SEC_M_IDX << 2) | CC2420_SECCTRL0_RXFIFO_PROTECTION | CC2420_SECCTRL0_TXKEYSEL1 | CC2420_SECCTRL0_CCM;
	setreg(CC2420_SECCTRL0, reg);
	PRINTFSEC("cc2420: Reg 0: %.2x\n",reg);

	/* Set associated data TX to 5 */
	setAssociatedData(TX, adata_len);

	/* Set Nonce tx */
	tmp_msg_cntr = (uint32_t)*msg_cntr;
	setNonce(TX, address_nonce, &tmp_msg_cntr, nonce_cntr);

	/* Flush the TXFIFO */
	strobe(CC2420_SFLUSHTX);

	/* Set TXFIFO */
	tot_len = *data_len + APP_MIC_LEN + AUX_LEN;
	CC2420_WRITE_FIFO_BUF(&tot_len, 1);
	CC2420_WRITE_FIFO_BUF(data, *data_len);

	/* Encrypt FIFO buffer */
	strobe(CC2420_STXENC);
	BUSYWAIT_UNTIL(!(status() & BV(CC2420_ENC_BUSY)), RTIMER_SECOND);

	/* Read TXFIFO buffer */
	CC2420_READ_RAM(data, CC2420RAM_TXFIFO, tot_len-1);

	/* Restore security control reg 0 */
	setreg(CC2420_SECCTRL0, reg_old);
	PRINTFSEC("cc2420: Reg 0 restore: %.2x\n",reg_old);

	/* Restore security control reg 1 */
	setAssociatedData(TX, 0);

	/* Update data_len with current value */
	*data_len = tot_len-AUX_LEN;

	return 1;
}
/*---------------------------------------------------------------------------*/
#endif
