/*
 * Copyright (c) 2008 Dimas Abreu Dutra
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the Stanford University nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL DIMAS ABREU
 * DUTRA OR HIS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author Dimas Abreu Dutra
 */

#ifndef RSSIMAPAPP_H__
#define RSSIMAPAPP_H__

// disable debug in "production"
//#define DEBUGPRINTF

#ifndef TOSH_DATA_LENGTH
#define TOSH_DATA_LENGTH 112
#endif

/**
 * Warning!
 * If you want to generate Java Messages by MIG (genJavaMsgs.sh) you need to comment line
 * define MIG.
 * 
 * MIG has trouble to include some needed header files, so they are included in MUGhlp.h.
 */

#ifdef DEBUGPRINTF
#ifdef MIG
#include "../printf.h"
#else
#include "printf.h"
#endif
#endif

#include "../commands.h"

#ifndef NULL
#define NULL ((void*)0)
#endif

// basic message types
enum {
  AM_RSSIMSG = 10,
  AM_PINGMSG = 11,

  // new messages
  AM_MULTIPINGMSG = 12,
  AM_MULTIPINGRESPONSEMSG = 13,
  
  AM_MULTIPINGRESPONSEREPORTMSG = 16,
  AM_MULTIPINGRESPONSETINYREPORTMSG = 17,
  
  AM_NOISEFLOORREADINGMSG = 18,
};

// ping response
// RssiMeassured
typedef nx_struct RssiMsg{
  nx_uint8_t nodeid;

  // destination node id
  // is this still needed??
  nx_uint8_t dstnodeid;

  // SEQ number
  nx_uint16_t counter;

  // RSSI from mobile node (intercept base will fill this in)
  // todo:
  //    optimize this, define new message, basestation will wrap this to new message
  //    time overhead???
  nx_int16_t rssi;

  // RSSI of ping message from base station
  nx_int16_t rssiFromBase;
} RssiMsg;

// standard ping-response protocol
// ping request
typedef nx_struct PingMsg{
	nx_uint8_t nodeid;
	nx_uint16_t counter;
	nx_uint8_t txpower;
	nx_uint8_t channel;
} PingMsg;

// message to request multiple packets from destination
// 1:N packets
typedef nx_struct MultiPingMsg {
	// where to send ping message? single node or broadcast
	nx_uint16_t destination;
	
	// SEQ number ot this request
	nx_uint16_t counter;

	// tx power of destination
	nx_uint8_t txpower;

	// channel at which to send
	nx_uint8_t channel;

	// number of packets to send
	nx_uint16_t packets;

	// timer delay between message send in ms
	nx_uint16_t delay;
	
	// desired packet size in bytes
	nx_uint8_t size;
	
	// target = packets. CurPacket is incremented when:
	// TRUE => only on succ sent packet => sendDone()==SUCC
	// FALSE => on every Send()==SUCC
	nx_bool counterStrategySuccess;
	
	// if true then timer is started periodically and at each timer tick
	// message is sent 
	// if false new mesage is sent after previous message was successfully sent in 
	// sendDone()
	nx_bool timerStrategyPeriodic;
} MultiPingMsg;

// packet intended to be sent over serial line to indicate meassured noise floor
typedef nx_struct NoiseFloorReadingMsg {
	// SEQ number
	nx_uint16_t counter;

	// noise floor reading from node
	nx_uint16_t noise;
} NoiseFloorReadingMsg;

typedef nx_struct MultiPingResponseMsg {
	// SEQ number
	nx_uint16_t counter;
	// requets number
	nx_uint16_t request;

	// data field to support variable sized messages
	nx_uint8_t data[0];
} MultiPingResponseMsg;

#ifndef RSSI_QUEUE_LEN
#define RSSI_QUEUE_LEN 32
#endif

typedef nx_struct MultiPingResponseReportStruct {
	nx_uint16_t nodeid;
	nx_uint16_t request;
	nx_uint16_t nodecounter;
	nx_int16_t rssi;
	nx_uint8_t len;
} MultiPingResponseReportStruct_t;

// maximum size of report queue buffer
#define MAX_REPORT_QUEUE_SIZE 100

// max data packet size
// need to take into account TOSH_DATA_LENGTH
// packet structure size must be smaller than TOSH_DATA_LENGTH
// otherwise no packet will be send
#define MULTIPINGRESPONSEREPORT_MAXDATA 3

// boot up timer request
#define BOOTUPTIMER_FIRST 500
#define BOOTUPTIMER_NEXT 5000

// used by static nodes to report RSSI values of mobile node
typedef nx_struct MultiPingResponseReportMsg {
	// SEQ number
	nx_uint16_t counter;

	// number of correct data
	nx_uint8_t datanum;

	// unifiing structure
	// mig has problem to convert this message to java class
	// mig cannot work wih structures inside messages
	//MultiPingResponseReportStruct buff[8];
	nx_uint16_t nodeid[MULTIPINGRESPONSEREPORT_MAXDATA];
	nx_uint16_t request[MULTIPINGRESPONSEREPORT_MAXDATA];
	nx_uint16_t nodecounter[MULTIPINGRESPONSEREPORT_MAXDATA];
	nx_int16_t rssi[MULTIPINGRESPONSEREPORT_MAXDATA];
	nx_int8_t len[MULTIPINGRESPONSEREPORT_MAXDATA];
} MultiPingResponseReportMsg;

// massReportPacket
typedef nx_struct MassReportMsg {
	// SEQ number
	nx_uint16_t counter;

	// number of correct data
	nx_uint8_t datanum;

	// unifiing structure
	// mig has problem to convert this message to java class
	// mig cannot work wih structures inside messages
	//MultiPingResponseReportStruct buff[8];
	nx_uint8_t nodeid[MULTIPINGRESPONSEREPORT_MAXDATA];
	nx_uint16_t nodecounter[MULTIPINGRESPONSEREPORT_MAXDATA];
	nx_int16_t rssi[MULTIPINGRESPONSEREPORT_MAXDATA];
} MassReportMsg;

// used by static nodes to report RSSI values of mobile node
// lightweight report message
typedef nx_struct MultiPingResponseTinyReportMsg {
	// SEQ number
	nx_uint16_t counter;

	// unifiing structure
	// mig has problem to convert this message to java class
	// mig cannot work wih structures inside messages
	//MultiPingResponseReportStruct buff[8];
	nx_uint16_t nodeid;
	nx_uint16_t nodecounter;
	nx_int16_t rssi;
} MultiPingResponseTinyReportMsg;

typedef struct serialqueue_element{
  nx_struct message_t * msg;
  // payload pointer
  void * payload;
  // address to send message to
  uint16_t addr;
  // length of message to send - parameter to AMSend.send = length of payload
  uint8_t len;
  // AM message type
  uint8_t id;
  // if 1=> radio packet, otherwise serial
  bool isRadioMsg;
} serialqueue_element_t;

typedef struct queueSenderQueue_element{
  // payload pointer
  void * payload;
  // address to send message to
  uint16_t addr;
  // length of message to send - parameter to AMSend.send = length of payload
  uint8_t len;
} queueSenderQueue_element_t;

#endif //RSSIMAPAPP_H__
