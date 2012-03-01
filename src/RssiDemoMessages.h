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

#ifndef RSSIDEMOMESSAGES_H__
#define RSSIDEMOMESSAGES_H__

// redefine TOSH_DATA_LENGTH ??
//#define TOSH_DATA_LENGTH 32

// basic message types
enum {
  AM_RSSIMSG = 10,
  AM_PINGMSG = 11,

  // new messages
  AM_MULTIPINGMSG = 12,
  AM_MULTIPINGRESPONSEMSG = 13,
  AM_COMMANDMSG = 14,
  
  AM_MULTIPINGRESPONSEREPORTMSG = 16,
  AM_MULTIPINGRESPONSETINYREPORTMSG = 17
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
typedef nx_struct MultiPingMsg{
//        // nodeid:)
//	nx_uint8_t nodeid;

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
} MultiPingMsg;

typedef nx_struct MultiPingResponseMsg{
    // SEQ number
    nx_uint16_t counter;

    // RSSI from mobile node (intercept base will fill this in)
    // todo:
    //    optimize this, define new message, basestation will wrap this to new message
    //    time overhead???
    nx_int16_t rssi;
} MultiPingResponseMsg;

typedef nx_struct MultiPingResponseReportStruct {
    nx_uint8_t nodeid;
    nx_uint8_t nodecounter;
    nx_int16_t rssi;
} MultiPingResponseReportStruct;

// maximum size of report queue buffer
#define MAX_REPORT_QUEUE_SIZE 100

// max data packet size
// need to take into account TOSH_DATA_LENGTH
// packet structure size must be smaller than TOSH_DATA_LENGTH
// otherwise no packet will be send
#define MULTIPINGRESPONSEREPORT_MAXDATA 4

// boot up timer request
#define BOOTUPTIMER_FIRST 500
#define BOOTUPTIMER_NEXT 5000

// used by static nodes to report RSSI values of mobile node
typedef nx_struct MultiPingResponseReportMsg{
    // SEQ number
    nx_uint16_t counter;

    // number of correct data
    nx_uint8_t datanum;

    // unifiing structure
    // mig has problem to convert this message to java class
    // mig cannot work wih structures inside messages
    //MultiPingResponseReportStruct buff[8];
    nx_uint16_t nodeid[MULTIPINGRESPONSEREPORT_MAXDATA];
    nx_uint16_t nodecounter[MULTIPINGRESPONSEREPORT_MAXDATA];
    nx_int16_t rssi[MULTIPINGRESPONSEREPORT_MAXDATA];
} MultiPingResponseReportMsg;


// massReportPacket
typedef nx_struct MassReportMsg{
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
typedef nx_struct MultiPingResponseTinyReportMsg{
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

/**
 * Command message
 * BaseStation manages its network using this simple protocol. More tasks can be
 * handled using it. For more complex settings its better to use another protocol.
 *
 * Available commands:
 *  - abort
 *      abort current operation
 *      Applicable on mobile nodes or on static nodes.
 *  - identify
 *      Nodes should reply its identification (static/mobile) when receives this packet
 *      BaseStation would know wether nodes lives and which one does.
 *  - myid
 *      Nodes returns its identification
 *  - reset
 *      Reset all state variables. It should have same effect as pushing reset button on node
 *  - ACK
 *      ACKnowledges previous packet. May relate to another command sent by BS
 *  - NACK
 *      Negative ACKnowledge
 *  - setBS
 *      tells static nodes to set/change current base station
 *  - lock
 *      changes locking for static nodes
 *  - getReportingStatus
 *      gets actual reporting status of given node on MultiPingResponses
 *  - setReportingStatus
 *      tells node wether or not to send reports on MultiPingResponses
 *      usefull when analyzing environment by pinging static nodes. Sometimes
 *      may be wanted static nodes not to report anything
 *  - setOperationMode
 *      nodes may be various types, static, dynamic, dead, base station
 *  - setReportProtocol
 *      reportprotocol to be used when in "static" mode
 *
 * Another possible commands, not yet implemented. Not needed now.
 *  - setTx
 *  - setChannel
 *  - setDelay
 *  - setPacketCount
 *  - setRandomizedThreshold
 *  - etc...
 */
typedef nx_struct CommandMsg{
    // command
    nx_uint8_t command_code;

    // command version. Support for protocol versioning. Some nodes may use older
    // firmware. May be used as packet subtype field
    nx_uint8_t command_version;

    // unique command identifier
    // nodes would be able to ACK or NACK commands.
    // It poses more reliable communication.
    // @not-implemented-yet
    nx_uint16_t command_id;

    // in case of ACK command, it is used as answer on specific command
    // only sugar, this info could be stored in command_data_next
    nx_uint8_t reply_on_command;
    nx_uint16_t reply_on_command_id;

    // some data associated with command (parameters for example)s
    nx_uint16_t command_data;

    // for future use
    // may contain another parameters while command_data would tell subtype of protocol
    nx_uint16_t command_data_next[4];
} CommandMsg;

/**
 * Defining available commands for nodes
 */
enum {
    COMMAND_NONE=0,
    COMMAND_ABORT=1,
    COMMAND_IDENTIFY=2,
    COMMAND_RESET=3,
    COMMAND_SETTX=4,
    COMMAND_SETCHANNEL=5,
    COMMAND_ACK=6,
    COMMAND_NACK=7,
    COMMAND_SETBS=8,
    COMMAND_LOCK=9,

    COMMAND_GETREPORTINGSTATUS=10,

    COMMAND_SETREPORTINGSTATUS=11,
    COMMAND_SETDORANDOMIZEDTHRESHOLDING=12,
    COMMAND_SETQUEUEFLUSHTHRESHOLD=13,
    COMMAND_SETTINYREPORTS=14,
    COMMAND_SETOPERATIONMODE=15,
    COMMAND_SETREPORTPROTOCOL=16,
    COMMAND_FLUSHREPORTQUEUE=17,
    COMMAND_SETNOISEFLOORREADING=18,
    COMMAND_SETSAMPLESENSORREADING=24,

    COMMAND_SETREPORTGAP=19,

    // sensor readings
    COMMAND_GETSENSORREADING=20,
    COMMAND_SENSORREADING=21,
            
    // pinning
    COMMAND_SETPIN=22,
    COMMAND_GETPIN=23,
            
    // settings
    // Fetching is request sent to base station after booting node up. 
    // Base station will then re-send node settings from node register to 
    // booted node (can be after reset already)
    COMMAND_FETCHSETTINGS=25
};

// node ID boudnary for mobile nodes
// nodes with ID>=MOBILE_NODE_ID_BOUDNARY are considered as mobile by default.
// this can be changed via commands
#define MOBILE_NODE_ID_BOUDNARY 200

/**
 * Identifications for COMMAND_IDENTIFY
 */
enum {
    NODE_REPORTING=1,
    NODE_TALKING=2,
    NODE_BS=3,
    NODE_DEAD=4
};


/**
 * reporting protocols
 */
enum {
    REPORTING_MEDIUM=1,
    REPORTING_TINY=2,
    REPORTING_MASS=3
};

/**
 * sensor readings
 */
enum {
    /*******************************************************************************\
	* ConditionTypeValide =	000x xxxx xxxx xxxx 					*
	* 	Last line =	000x 0000 0000 0000 => (A & 1000) / 1000		*
	* 	typeId =	0000 xxxx 0000 0000 => (A & 0F00) / 0100		*
	* 	verbId =	0000 0000 xxxx 0000 => (A & 00F0) / 0010		*
	* 	msg or logic =	0000 0000 0000 x000 => (A & 0008) / 0008 		*
	* 	msg or logic =	0000 0000 0000 0xxx => (A & 0007) / 0001 		*
	\*******************************************************************************/
	SCALE_LAST_LINE = 0x1000, //4096;
	MASK_LAST_LINE = 0x1000, //4096;
	SCALE_TYPE = 0x0100, //256;
	MASK_TYPE = 0x0F00, //3840;
	SCALE_VERB = 0x0010, //16;
	MASK_VERB = 0x00F0, //240;
	SCALE_SELECT_MSG = 0x0008, //8;
	MASK_SELECT_MSG = 0x0008, //8;
	SCALE_MSG_LOGIC = 0x0001, //1;
	MASK_MSG_LOGIC = 0x0007, //7;

	/*******************************************************************************\
	* readingLong =		0000 xxxx xxxx xxxx 					*
	* 	type reading =	0000 xx00 0000 0000 => (A & 0C00) / 0400		*
	* 	number alert =	0000 00xx xxxx xxxx => (A & 03FF) / 0001		*
	\*******************************************************************************/
	SCALE_TYPE_READ = 0x0400, //1024
	MASK_TYPE_READ = 0x1C00, //3072
	SCALE_NUM_ALERT = 0x0001, //1
	MASK_NUM_ALERT = 0x03FF //1023
};
#endif //RSSIDEMOMESSAGES_H__
