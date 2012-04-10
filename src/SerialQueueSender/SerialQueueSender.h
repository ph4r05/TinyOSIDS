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

enum {
  AM_PRINTF_MSG = 128,
};

#endif //PRINTF_H

