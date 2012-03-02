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

#include "ApplicationDefinitions.h"
#include "../RssiDemoMessages.h"
//Defining the preprocessor variable CC2420_NO_ACKNOWLEDGEMENTS will disable all forms of acknowledgments at compile time.
//Defining the preprocessor variable CC2420_HW_ACKNOWLEDGEMENTS will enable hardware acknowledgments and disable software acknowledgments.
#define CC2420_NO_ACKNOWLEDGEMENTS 1
	
#ifdef __CC2420_H__

#elif defined(TDA5250_MESSAGE_H)
      
#else
  
#endif 

module RssiBaseC {
	uses {
		interface Intercept as RssiMsgIntercept;
  		interface Intercept as SimpleRssiMsgIntercept;
  		interface Intercept as CommandMsgIntercept;
  		interface Intercept as SerialCommandIntercept;
  		interface AMSend as UartAMSend;
  		interface AMSend as UartCmdAMSend;
	}

/*
  uses interface Intercept as Report;
*/
  uses interface Timer<TMilli> as AliveTimer;
  uses interface Timer<TMilli> as SendTimer;
  uses interface AMSend as PingMsgSend;
  uses interface SplitControl as RadioControl;
  uses interface SplitControl as SerialControl;
  uses interface Packet as UartPacket;
  uses interface AMPacket as UartAMPacket;
  uses interface Packet as RadioPacket;
  uses interface AMPacket as RadioAMPacket;  
  uses interface Packet;
  uses interface AMPacket;
  uses interface Leds;
  uses interface Boot;

#ifdef __CC2420_H__
  uses interface CC2420Packet;
#elif  defined(PLATFORM_IRIS)  
  uses interface PacketField<uint8_t> as PacketRSSI;
#elif defined(TDA5250_MESSAGE_H)
  uses interface Tda5250Packet;
#endif 

} implementation {
  message_t pkt;
  message_t cmdPkt;
  
  bool busy = TRUE;
  bool serialBusy = TRUE;
  
  uint16_t counter = 0;
  uint8_t blinkCnSend = 0;
  
  // buffer for last rssi value
  uint16_t lastRssiValue=0;
  am_addr_t lastNodeId;
  bool doReportSend=FALSE;

  // forward declarations
  void sendBlink();
  uint16_t getRssi(message_t *msg);
  
  // reporting
  void task sendReport();
  void task sendAlive();

  // decision function, should be message cathed on radio forwarded to serial on BS?
  // we can change some fields in given message
  // todo: report against given message type ID
	event bool RssiMsgIntercept.forward(message_t * msg, void * payload, uint8_t len) {
		// store RSSI to local buffer
		lastRssiValue = getRssi(msg);
		lastNodeId = call RadioAMPacket.source(msg);
		doReportSend = TRUE;
		
		// initiate task sending
		
		
		// get message payload
		//MultiPingResponseMsg * rssiMsg = (MultiPingResponseMsg * ) payload;

		// fill rssi field in
		// RSSI of packet arrived
		//rssiMsg->rssi = 0;//getRssi(msg); 

		sendBlink();

		// TRUE -> Yes, forward this message to serial port
		return TRUE;
	}

	event bool SimpleRssiMsgIntercept.forward(message_t * msg, void * payload, uint8_t len) {
		// get message payload
		RssiMsg * rssiMsg = (RssiMsg * ) payload;

		// fill rssi field in
		// RSSI of packet arrived
		rssiMsg->rssi = getRssi(msg);

		sendBlink();

		// TRUE -> Yes, forward this message to serial port
		return TRUE;
	}
  
	event bool CommandMsgIntercept.forward(message_t *msg, void *payload, uint8_t len){
		// OK we can forward command messages to application, no problem ;-)
		return TRUE;
	}

	// Controls forwarding of messages received on serial interface.
	// If is packet destined for me only
	event bool SerialCommandIntercept.forward(message_t *msg, void *payload, uint8_t len){
		// if message is destined for me, process it, get destination
		am_addr_t destination = call UartAMPacket.destination(msg);
		bool forMe = call UartAMPacket.isForMe(msg);
		
		if (forMe){
			// process command here...
			sendBlink();
			
			// do not forward private communication:)
			return (AM_BROADCAST_ADDR==destination);
		}
		
		return TRUE;
	}
	
	/************************ SIGNALIZATION ************************/
  void sendBlink() {
    // no blinking here, overhead
      return;
      
      
      if (TRUE || blinkCnSend==0){
        call Leds.led1Toggle();
      }

      //blinkCnSend = (blinkCnSend+1) % 10;
      blinkCnSend=0;
  }  
	
	/************************ INITIALIZATION ************************/
	event void Boot.booted() {
		// nothing to say, you are goood ;-)
	}
  
  	event void RadioControl.startDone(error_t error){
		busy=FALSE;
	}

	event void RadioControl.stopDone(error_t error){
		busy=TRUE;
	}

	event void SerialControl.stopDone(error_t error){
		serialBusy=TRUE;
		call AliveTimer.stop();
	}

	event void SerialControl.startDone(error_t error){
		serialBusy=FALSE;
		call AliveTimer.startPeriodic(3000);
	}
  
    /************************ SENDING ************************/
	event void SendTimer.fired() {
		;
	}

	event void PingMsgSend.sendDone(message_t * m, error_t error) {
		busy = FALSE;
	}
  
  	// send RSSI report to application
	void task sendReport() {
		if(doReportSend == FALSE) {
			return;
		}

		if( ! serialBusy) {
			MultiPingResponseReportMsg * btrpkt = (MultiPingResponseReportMsg* ) (call UartAMSend.getPayload(&pkt, sizeof(MultiPingResponseReportMsg)));
			// only one report here, yet
			btrpkt->datanum=1;
			btrpkt->counter=counter++;
			btrpkt->nodecounter[0] = 0;
			btrpkt->nodeid[0] = lastNodeId;
			btrpkt->rssi[0] = lastRssiValue;
			
			//PingMsg * btrpkt = (PingMsg * )(call Packet.getPayload(&pkt, 0));
			//btrpkt->nodeid = TOS_NODE_ID;
			//btrpkt->counter = counter;
			if(call UartAMSend.send(AM_BROADCAST_ADDR, &pkt, sizeof(MultiPingResponseReportMsg)) == SUCCESS) {
				serialBusy = TRUE;
				sendBlink();
			}
			else {
				post sendReport();
			}
		}
		else {
			post sendReport();
		}
	}
  
  	event void UartAMSend.sendDone(message_t *msg, error_t error){
  		serialBusy = FALSE;
		if(error == SUCCESS) {
			doReportSend = FALSE;
		}
		else {
			post sendReport();
		}
	}
	
	// alive counter fired -> signalize that I am alive to application
	event void AliveTimer.fired(){
		if (serialBusy){
			return;
		}
		
		post sendAlive();
	}
	
	// sends alive packet to application to know that node is OK
	void task sendAlive(){
		if (serialBusy) {
			dbg("Cannot send indentify message");
			post sendAlive();
		}
		
		CommandMsg * btrpkt = (CommandMsg* ) (call UartCmdAMSend.getPayload(&cmdPkt, sizeof(CommandMsg)));
		// only one report here, yet
		btrpkt->command_id = counter;
		btrpkt->reply_on_command = COMMAND_IDENTIFY;
		btrpkt->command_code = COMMAND_ACK;
		btrpkt->command_version = 1;
		btrpkt->command_data = NODE_BS;
		// fill radio chip here
#ifdef __CC2420_H__
        btrpkt->command_data_next[0]=1;
#elif defined(PLATFORM_IRIS)
        btrpkt->command_data_next[0]=2;
#elif defined(TDA5250_MESSAGE_H)
        btrpkt->command_data_next[0]=3;
#else
        btrpkt->command_data_next[0]=4;
#endif
        // fill node ID
        btrpkt->command_data_next[1] = TOS_NODE_ID;
		if(call UartCmdAMSend.send(AM_BROADCAST_ADDR, &cmdPkt, sizeof(CommandMsg)) == SUCCESS) {
			serialBusy = TRUE;
			sendBlink();
		}
		else {
			dbg("Cannot send identify message");	
		}
	}
	
	event void UartCmdAMSend.sendDone(message_t *msg, error_t error){
		serialBusy=FALSE;
	}
  
/*
  void task sendEchoReply(){	      
    if (!busy) {
    	PingMsg* btrpkt = (PingMsg*)(call Packet.getPayload(&pkt, 0));
    	btrpkt->nodeid  = TOS_NODE_ID;
    	btrpkt->counter = counter;
    	if (call PingMsgSend.send(AM_BROADCAST_ADDR, &pkt, sizeof(PingMsg)) == SUCCESS) {
      	    busy = TRUE;
      	    sendBlink();
    	}
    	else
    	{
    		post sendEchoReply();
    	}
    }
    else
    {
        post sendEchoReply();
    }
  }
*/

/************************ READING ************************/
// radio dependent code, RSSI reading
// RSSI extraction from packet metadata
#ifdef __CC2420_H__
  uint16_t getRssi(message_t *msg){
    return (uint16_t) call CC2420Packet.getRssi(msg);
  }
#elif defined(CC1K_RADIO_MSG_H)
    uint16_t getRssi(message_t *msg){
    cc1000_metadata_t *md =(cc1000_metadata_t*) msg->metadata;
    return md->strength_or_preamble;
  }
#elif defined(PLATFORM_IRIS)
  uint16_t getRssi(message_t *msg){
    if(call PacketRSSI.isSet(msg))
      return (uint16_t) call PacketRSSI.get(msg);
    else
      return 0xFFFF;
  }
#elif defined(TDA5250_MESSAGE_H)
   uint16_t getRssi(message_t *msg){
       return call Tda5250Packet.getSnr(msg);
   }
#else
   uint16_t getRssi(message_t *msg){
       return -1;
   }
#error Radio chip not supported! This demo currently works only \
         for motes with CC1000, CC2420, RF230 or TDA5250 radios.
#endif
}