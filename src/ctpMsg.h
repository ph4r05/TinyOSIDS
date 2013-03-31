#ifndef CTPMSG_H__
#define CTPMSG_H__ 1

/**
 * Warning!
 * If you want to generate Java Messages by MIG (genJavaMsgs.sh) you need to
 * define MIG - local definition will be used
 * 
 * MIG has trouble to include some needed header files, so they are included in MUGhlp.h.
 */
#ifndef MIG 
#include <Ctp.h>
#else
#include "MIGhlp.h"
#endif

// basic message types
enum {
  AM_CTPINFOMSG = 0xec,
  AM_CTPSENDREQUESTMSG = 0xee,
  AM_CTPRESPONSEMSG = 0xef,
  AM_CTPREPORTDATAMSG = 0xed,
  AM_COLLECTIONDEBUGMSG = 0x72
};

typedef nx_struct CtpResponseMsg {
    nx_uint16_t origin;
    nx_uint16_t seqno;
    nx_uint16_t parent;
    nx_uint16_t metric;
    nx_uint8_t dataType;
    nx_uint16_t data;
} CtpResponseMsg;

// message to request multiple packets from destination, CTP protocol
// 1:N packets
enum {
	CTP_SEND_REQUEST_COUNTER_STRATEGY_SUCCESS = 0x1,
	CTP_SEND_REQUEST_TIMER_STRATEGY_PERIODIC = 0x2,
	CTP_SEND_REQUEST_PACKETS_UNLIMITED = 0x4,
};

typedef nx_struct CtpSendRequestMsg {
	// SEQ number ot this request, identifier
	nx_uint16_t counter;

	// number of packets to send
	nx_uint16_t packets;

	// timer delay between message send in ms
	nx_uint16_t delay;
	
	// percentage of delay +- variability, if 0 no variability is used
	nx_uint16_t delayVariability;
	
	// desired packet size in bytes
	nx_uint8_t size;
	
	// datasource of CtpMessage - can be random/sensor reading
	nx_uint8_t dataSource;
	
	// flags field
	// 0x1 		counterStrategySuccess
	//				CurPacket is incremented when:
	// 				TRUE => only on succ sent packet => sendDone()==SUCC
	// 				FALSE => on every Send()==SUCC
	//
	// 0x2 		timerStrategyPeriodic
	//				 if true then timer is started periodically and at each timer tick
	// 				 message is sent 
	// 				 if false new mesage is sent after previous message was successfully sent in 
	// 				 sendDone()
	//
	// 0x4		unlimited packet count
	
	nx_uint16_t flags;
} CtpSendRequestMsg;

// ctp spoof report message, all collected information available
typedef nx_struct CtpReportDataMsg {
	nx_struct CtpResponseMsg response;
	
	// local time when this message was created
	nx_uint32_t localTime32khz;
	
	// source of message
	nx_am_addr_t amSource;
	
	// packet timestamp
	nx_uint32_t timestamp32khz;
	
	// RSSI of received packet
	nx_union{
		nx_struct {
	       ctp_data_header_t ctpDataHeader;
	       nx_int16_t rssi;
	    } recv;
	    
	    nx_struct {
	    	nx_uint32_t ccaWaitTime;
	    	nx_uint16_t ccaWaitRounds;
	    	
	    	nx_uint8_t fwdRetryCount; 
	    } sent;
	} data;
	
	// LSB
	// 1. bit 0x1 = spoofed boolean
	// 2. bit 0x2 = normal CTP reception == TRUE, otherwise was tapped
	// 3. bit 0x4 = message was sent from node - reporting of sending by same node
	//			in this case rssi, ctpDataHeader has no meaning, thus must be nulled
	// 4. bit 0x8 = sent message, detected from sendDone snooper in ForwardingEngine.
	nx_uint8_t flags;
} CtpReportDataMsg;


typedef nx_struct CtpInfoMsg {
    nx_uint8_t type;
    nx_uint32_t localTime32khz;
    nx_union {        
        nx_struct {
        	nx_uint16_t data[6];
       	} data;
        
        nx_struct {
        	nx_uint16_t parent;
        	nx_uint16_t etx;
        	nx_uint8_t neighbors;
        	nx_uint8_t serialQueueSize;
        	nx_uint16_t ctpSeqNo;
        	nx_uint8_t ctpBusyCount;
        	nx_uint16_t flags;
        } status;
        
        nx_struct {
        	nx_uint8_t num;
        	nx_uint16_t addr;
        	nx_uint16_t linkQuality;
        	nx_uint16_t routeQuality;		
        	nx_uint16_t flags;
        } neighInfo;

    } data;
} CtpInfoMsg;


#endif // CTPMSG_H__