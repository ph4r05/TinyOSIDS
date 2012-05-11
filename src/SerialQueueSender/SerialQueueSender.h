/*
 * "Copyright (c) 2006 Washington University in St. Louis.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 *
 * IN NO EVENT SHALL WASHINGTON UNIVERSITY IN ST. LOUIS BE LIABLE TO ANY PARTY
 * FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING
 * OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF WASHINGTON
 * UNIVERSITY IN ST. LOUIS HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * WASHINGTON UNIVERSITY IN ST. LOUIS SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND WASHINGTON UNIVERSITY IN ST. LOUIS HAS NO
 * OBLIGATION TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR
 * MODIFICATIONS."
 */

/**
 *
 * @author Kevin Klues (klueska@cs.wustl.edu)
 * @version $Revision: 1.16 $
 * @date $Date: 2010/02/22 05:29:57 $
 */

#ifndef SERIAL_QUEUE_SENDER_H
#define SERIAL_QUEUE_SENDER_H

#ifndef SERIALSENDER_BUFFER_SIZE
#define SERIALSENDER_BUFFER_SIZE 32 
#endif

#if SERIALSENDER_BUFFER_SIZE > 255
  #define SerialSenderQueueC	BigQueueC
  #define SerialSenderQueue	BigQueue
#else
  #define SerialSenderQueueC	QueueC
  #define SerialSenderQueue	Queue
#endif

#ifdef _H_msp430hardware_h
  #include <stdio.h>
#else
#ifdef _H_atmega128hardware_H
  #include "avr_stdio.h"
#else
#ifdef __M16C62PHARDWARE_H__ 
  #include "m16c62p_printf.h"
#else
  #include "generic_printf.h"
#endif
#endif
#endif
#include "message.h"
//int printfflush();

// carry serial report message
typedef nx_struct SerialSendReportMsg {
	nx_am_id_t type;
  	nx_uint8_t payload[0];
} SerialSendReportMsg;

typedef nx_struct SerialSendHeader {
	nx_am_id_t type;
} SerialSendHeader;

/**
 * Metadata for sending 
 */
typedef struct senderMetadata {
	uint8_t len;
	uint8_t retries;
	am_addr_t addr;
	void * payload;
} senderMetadata_t;

/**
 * Global report structure
 */
 typedef nx_struct globalReportMsg {
 	// type determining contents of message
 	nx_uint8_t type;
 	// another multiplex - subtype
 	nx_uint8_t subtype;
 	// counter, prefer having it
 	nx_uint16_t counter;
	// message content 	
 	nx_union {
 		nx_struct {
	 		nx_uint16_t a;
	 		nx_uint16_t b;
	 		nx_uint16_t c;
	 		nx_uint16_t d;
	 		nx_uint16_t e;
	 		nx_uint16_t f;
	 		nx_uint16_t g;
	 		nx_uint16_t h;
	 	} single16;
	 	
	 	nx_struct {
	 		nx_uint8_t a[16];
	 	} arr8;
	 	
	 	nx_struct {
	 		nx_uint16_t a[8];
	 	} arr16;
	 	
	 	nx_struct {
	 		nx_uint32_t a[4];
	 	} arr32;
	 	
	 	nx_struct {
	 		nx_uint64_t a[2];
	 	} arr64;
 	} data;
} globalReportMsg_t;


/**
 * variable length message
 * Length of sub-payload is determined as packet length - 4B (type, subtype, counter) 
 */
typedef nx_struct globalSizedReportMsg {
 	// type determining contents of message
 	nx_uint8_t type;
 	// another multiplex - subtype
 	nx_uint8_t subtype;
 	// counter, prefer having it
 	nx_uint16_t counter;
	// message content, prefer 16bit single unit - 	
	nx_uint16_t data[0];
} globalSizedReportMsg_t;


enum {
  AM_GLOBALREPORT_MSG = 0xdd,
  AM_GLOBALSIZEDREPORT_MSG = 0xde,
};

#endif //PRINTF_H

