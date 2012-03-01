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
  uses interface Intercept as RssiMsgIntercept;
  uses interface Intercept as SimpleRssiMsgIntercept;
/*
  uses interface Intercept as Report;
*/
  
  uses interface Timer<TMilli> as SendTimer;
  uses interface AMSend as PingMsgSend;
  uses interface SplitControl as RadioControl;
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
  bool busy = FALSE;
  uint16_t counter = 0;

  uint8_t blinkCnSend = 0;

  // forward declarations
  void sendBlink();
  uint16_t getRssi(message_t *msg);
/*
  void task sendEchoReply();
*/

  // decision function, should be message cathed on radio forwarded to serial on BS?
  // we can change some fields in given message
  // todo: report against given message type ID
  event bool RssiMsgIntercept.forward(message_t *msg, void *payload, uint8_t len) {
    // get message payload
    MultiPingResponseMsg *rssiMsg = (MultiPingResponseMsg*) payload;

    // fill rssi field in
    // RSSI of packet arrived
    rssiMsg->rssi = 0;//getRssi(msg);

    sendBlink();

    // TRUE -> Yes, forward this message to serial port
    return TRUE;
  }

  event bool SimpleRssiMsgIntercept.forward(message_t *msg, void *payload, uint8_t len) {
    // get message payload
    RssiMsg *rssiMsg = (RssiMsg*) payload;

    // fill rssi field in
    // RSSI of packet arrived
    rssiMsg->rssi = getRssi(msg);

    sendBlink();

    // TRUE -> Yes, forward this message to serial port
    return TRUE;
  }

/*
  event bool Report.forward(message_t *msg, void *payload, uint8_t len){
      return TRUE;
  }
*/
  void sendBlink() {
    // no blinking here, overhead
      return;
      
      
      if (blinkCnSend==0){
        call Leds.led1Toggle();
      }

      //blinkCnSend = (blinkCnSend+1) % 10;
      blinkCnSend=0;
  }  

  // system bood event handler
  event void Boot.booted(){
  	busy = TRUE;
        call RadioControl.start();
  }
  
  event void SendTimer.fired(){
/*
        post sendEchoReply();
*/
  }
  
 /**
  * Radio Control Start Done
  */
  event void RadioControl.startDone(error_t result){
    //do not send now
    if (result == SUCCESS) {
      ;//call SendTimer.startPeriodic(500);
    }
    else {
      call RadioControl.start();
    }
    
    busy = FALSE;
  }

  // stop done, empty event handler, nothing to do here
  event void RadioControl.stopDone(error_t result){
  }
  
  event void PingMsgSend.sendDone(message_t *m, error_t error){
  	busy = FALSE;
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
