/*
 * "Copyright (c) 2000-2005 The Regents of the University  of California.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF
 * CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 *
 * Copyright (c) 2002-2003 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA,
 * 94704.  Attention:  Intel License Inquiry.
 */

#include "Timer.h"
#include "RadioCountToLeds.h"
#include "printf.h"

#include "CC2420.h"
#include "AM.h"
#include "Ieee154.h"

/**
 * Implementation of the RadioCountToLeds application. RadioCountToLeds
 * maintains a 4Hz counter, broadcasting its value in an AM packet
 * every time it gets updated. A RadioCountToLeds node that hears a counter
 * displays the bottom three bits on its LEDs. This application is a useful
 * test to show that basic AM communication and timers work.
 *
 * @author Philip Levis
 * @date   June 6 2005
 */

module RadioCountToLedsC {
  uses {
    interface Leds;
    interface Boot;
    interface Receive;
    interface AMSend;
    interface Timer<TMilli> as MilliTimer;
    interface SplitControl as AMControl;
    interface Packet;
    interface CC2420SecurityMode as CC2420Security;
    interface CC2420Keys;

    interface PacketLink;
    
    interface AMSend as CmdSend;
    interface CC2420Packet;
    interface CC2420PacketBody;
  }
}
implementation {

  message_t packet;
  uint8_t key[16] = {0x98,0x67,0x7F,0xAF,0xD6,0xAD,0xB7,0x0C,0x59,0xE8,0xD9,0x47,0xC9,0x71,0x15,0x0F};
  uint8_t keyReady = 0; // should be set to 1 when key setting is done

  bool locked;
  uint16_t counter = 0;
  
  // commands
  bool cmdBusy=FALSE;
  uint16_t cmdCounter=0;
  message_t cmdPkt;
  CommandMsg cmdPayload;

  event void Boot.booted()
  {
    call AMControl.start();
  }

	event void AMControl.startDone(error_t err) {
		if(err == SUCCESS) {
			call CC2420Keys.setKey(0, key);
			if(TOS_NODE_ID == 5) 
				call MilliTimer.startPeriodic(512);
		}
		else {
			call AMControl.start();
		}
	}

  event void AMControl.stopDone(error_t err)
  {
  }

  event void CC2420Keys.setKeyDone(uint8_t keyNo, uint8_t* skey)
  {
    keyReady = 1;
  }

  int8_t getRssi(message_t *msg){
    return call CC2420Packet.getRssi(msg);
  }
  
  uint8_t getLqi(message_t *msg){
  	return call CC2420Packet.getLqi(msg);
  }

  event void MilliTimer.fired()
  {
    counter++;
    dbg("RadioCountToLedsC", "RadioCountToLedsC: timer fired, counter is %hu.\n", counter);
    if (locked) {
      return;
    }
    else if(keyReady == 1) {

      radio_count_msg_t* rcm = (radio_count_msg_t*)call Packet.getPayload(&packet, sizeof(radio_count_msg_t));

      if (rcm == NULL) {
	return;
      }

      rcm->counter = counter;
      //call CC2420Security.setCtr(&packet, 0, 0);
      call CC2420Security.setCbcMac(&packet, 0, 0, 4);
      //call CC2420Security.setCcm(&packet, 1, 0, 16);
      //call PacketLink.setRetries(&packet, 3);
      if (call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(radio_count_msg_t)) == SUCCESS) {
	dbg("RadioCountToLedsC", "RadioCountToLedsC: packet sent.\n", counter);
	locked = TRUE;
      }
    }
  }

  event message_t* Receive.receive(message_t* bufPtr,
				   void* payload, uint8_t len)
  {
    dbg("RadioCountToLedsC", "Received packet of length %hhu.\n", len);
    if (len != sizeof(radio_count_msg_t)) {return bufPtr;}
    else {
      radio_count_msg_t* rcm = (radio_count_msg_t*)payload;
      cc2420_metadata_t * meta = call CC2420PacketBody.getMetadata(bufPtr);
      uint8_t lqi = getLqi(bufPtr);
      int8_t rssi = getRssi(bufPtr); 
      
      printf("counter: %d len: %d lqi: %d rssi: %d crc %d\n",
      	rcm->counter, len, lqi, rssi, meta->crc);
      printfflush();
      
      // received on radio! send report command
      cmdPayload.command_id = rcm->counter;
      cmdPayload.command_data = lqi;
      cmdPayload.command_data_next[0] = meta->crc;
      cmdPayload.command_data_next[1] = rssi;
      cmdPayload.command_data_next[2] = len;
      
//      if (rcm->counter & 0x1) {
//	call Leds.led0On();
//      }
//      else {
//	call Leds.led0Off();
//      }
//      if (rcm->counter & 0x2) {
//	call Leds.led1On();
//      }
//      else {
//	call Leds.led1Off();
//      }
//      if (rcm->counter & 0x4) {
//	call Leds.led2On();
//      }
//      else {
//	call Leds.led2Off();
//      }
      return bufPtr;
    }
  }

  event void AMSend.sendDone(message_t* msg, error_t error)
  {
    if (&packet == msg) {
      locked = FALSE;
    }
  }


	event void CmdSend.sendDone(message_t *msg, error_t error){
		// TODO Auto-generated method stub
	}
}
