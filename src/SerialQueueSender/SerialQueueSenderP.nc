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
 
 /*
 * Copyright (c) 2007 Stanford University.
 * All rights reserved.
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
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL STANFORD
 * UNIVERSITY OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * This is the PrintfP component.  It provides the printf service for printing
 * data over the serial interface using the standard c-style printf command.  
 * Data printed using printf are buffered and only sent over the serial line after
 * the buffer is half full or an explicit call to printfflush() is made.  This 
 * buffer has a maximum size of 250 bytes at present.  This component is wired
 * to a shadowed MainC component so that printf statements can be made anywhere 
 * throughout your code, so long as you include the "printf.h" header file in 
 * every file you wish to use it.  Take a look at the printf tutorial (lesson 15)
 * for more details.
 *
 * The printf service is currently only available for msp430 based motes 
 * (i.e. telos, eyes) and atmega128x based motes (i.e. mica2, micaz, iris).  On the
 * atmega platforms, avr-libc version 1.4 or above must be used.
 */
 
/**
 * @author Kevin Klues <klueska@cs.stanford.edu>
 * @date September 18, 2007
 */

#include "SerialQueueSender.h"

generic module SerialQueueSenderP(typedef queue_t, uint8_t QUEUE_SIZE, am_id_t amtype) @safe() {
  provides {
    interface QueueSender;
  }
  uses {
    interface AMSend;
    interface Packet;
    interface AMPacket;
  }
}
implementation {
  
  enum {
    S_STOPPED,
    S_STARTED,
    S_FLUSHING,
  };

   // serial queue & management
   // uartQueue = queue of received radio packets 
   // waiting to be send over UART.
   // Real messages are stored in bufs. Pointers to this
   // queue is given as return value of Packet.receive 
   // -> where to store next message.
   queue_t uartQueueBufs[SERIALSENDER_BUFFER_SIZE];
   queueSenderQueue_element_t uartQueueMeta[SERIALSENDER_BUFFER_SIZE];
    // array of pointers to messages in queue. Messages are really stored 
    // in buffs or in 1 message queue in receive buffer.
//    message_t * uartQueue[SERIALSENDER_BUFFER_SIZE];
    // queue for external packets. If not null, then packet on specified position is
    // from external source
    //  a) keeps message original pointer for messageSent notif
    //  b) signalization purposes for special handling with external messages
//    message_t * uartQueueExternal[SERIALSENDER_BUFFER_SIZE];
    uint8_t uartIn, uartOut;
    bool uartBusy, uartFull;
//    // signalizes whether packet is enqueued from queue
//    uint8_t uartPacketExternal[SERIALSENDER_BUFFER_SIZE/8+1];
    
  
  
  message_t tmpMsg;
  uint8_t state = S_STOPPED;


//  task void retrySend() {
//    if(call AMSend.send(AM_BROADCAST_ADDR, &printfMsg, sizeof(printf_msg_t)) != SUCCESS)
//      post retrySend();
//  }
//  
//  void sendNext() {
//    int i;
//    printf_msg_t* m = (printf_msg_t*)call Packet.getPayload(&printfMsg, sizeof(printf_msg_t));
//    uint16_t length_to_send = (call Queue.size() < sizeof(printf_msg_t)) ? call Queue.size() : sizeof(printf_msg_t);
//    memset(m->buffer, 0, sizeof(printf_msg_t));
//    for(i=0; i<length_to_send; i++)
//      m->buffer[i] = call Queue.dequeue();
//    if(call AMSend.send(AM_BROADCAST_ADDR, &printfMsg, sizeof(printf_msg_t)) != SUCCESS)
//      post retrySend();  
//  }
//  
//  event void AMSend.sendDone(message_t* msg, error_t error) {    
//    if(error == SUCCESS) {
//      if(call Queue.size() > 0)
//        sendNext();
//      else state = S_STARTED;
//    }
//    else post retrySend();
//  }
//  


	command queueSenderQueue_element_t * QueueSender.element(uint8_t idx){
		// TODO Auto-generated method stub
		
		return NULL;
	}

	command error_t QueueSender.enqueue(queueSenderQueue_element_t *newVal){
		// TODO Auto-generated method stub
		
		return FAIL;
	}
	
//	command error_t QueueSender.enqueueRaw(am_id_t id, am_addr_t addr, message_t *msg, void * payload, uint8_t len, bool radioPacket){
//		return FAIL;
//	}

	command queueSenderQueue_element_t * QueueSender.head(){
		queueSenderQueue_element_t elem;
		message_t * curElem;
		if (call QueueSender.empty()){
			return NULL;
		}
		
		curElem = &(uartQueueBufs[uartIn-1]);
		call Packet.getPayload(curElem, 1);
		//elem.payload = call ;
//  // address to send message to
//  uint16_t addr;
//  // length of message to send - parameter to AMSend.send = length of payload
//  uint8_t len;
//  // AM message type
//  uint8_t id;
		
		
		
		return NULL;
		//return call Queue.head();
	}

	command uint8_t QueueSender.size(){
		return (uartOut > uartIn) ? SERIALSENDER_BUFFER_SIZE - uartOut + uartIn : uartIn - uartOut;
	}

	command uint8_t QueueSender.maxSize(){
		return SERIALSENDER_BUFFER_SIZE;
	}

	command bool QueueSender.empty(){
		return ((uartFull==FALSE) && (uartIn==uartOut));
	}

	command bool QueueSender.full(){
		return uartFull;
	}
	
	
//	command error_t SerialQueue.enqueue(serialqueue_element_t * newVal){
//		message_t * ret; 
//
//		// if queue is full, cannot add new
//		if (uartFull) {
//                // serial queue full       
//                failBlink();
//                dropBlink();
//                ++uartFailCounter;
//                timedUartSendTask();
//                return ENOMEM;
//        }
//		
//		// queue manipulation has to be atomic to preserve pointers/counters consistency
//        atomic
//        {
//			// check full again
//			if (uartFull) {
//				timedUartSendTask();
//				return ENOMEM;
//			}
//			// now is guaranted that nothing happened to queue from last check
//			
//	    	// pointer to free memory block. copy here message passed for sending
//	        ret = uartQueue[uartIn];
//
//	        // ser proper packet-type flag
//	        if (newVal->isRadioMsg){
//	        	uartPacketSerial[uartIn/8] &= ~(1<<(uartIn%8));
//	        } else {
//	        	uartPacketSerial[uartIn/8] |= (1<<(uartIn%8));
//	        }
//	        
//	        // set always to 1 here
//	        uartPacketExternal[uartIn/8] |= (1<<uartIn%8);
//	        
//	        //memcpy(ret, newVal->msg, sizeof(message_t));
//	        
//	        // set fields as radio packet - BaseStation assumes that radio 
//	        // packets are in this queue -> no problem, next calls will
//	        // consider message as radio message and puts correct fields
//	        if (newVal->isRadioMsg){
//	        	// copy whole message to defines address, preserve my addresses for 
//	        	// future - N+1 cycling queue for receive
//		        memcpy(call RadioPacket.getPayload(ret, newVal->len), 
//		        	newVal->payload, newVal->len);
//		        
//	        	
//	        	call RadioAMPacket.setDestination(ret, newVal->addr);
//	        	call RadioAMPacket.setType(ret, newVal->id);
//	        	call RadioAMPacket.setSource(ret, TOS_NODE_ID);
//	        	call RadioPacket.setPayloadLength(ret, newVal->len);
//	        } else {
//	        	// copy whole message to defines address, preserve my addresses for 
//		        // future - N+1 cycling queue for receive
//	        	memcpy(call UartPacket.getPayload(ret, newVal->len), 
//	        		newVal->payload, newVal->len);
//	        	
//	        	call UartAMPacket.setDestination(ret, newVal->addr);
//	        	call UartAMPacket.setType(ret, newVal->id);
//	        	call UartAMPacket.setSource(ret, TOS_NODE_ID);
//	        	call UartPacket.setPayloadLength(ret, newVal->len);
//	        }
//	        
//	        // store external message pointer - when enqueueing new messages then do not
//	        // signal sendDone, massive sending. 
//	        uartQueueExternal[uartIn] = NULL;
//	        // to current free place in queue is stored actualy received 
//	        // message from radio.
//	        uartQueue[uartIn] = ret;
//			// next slot in queue - cyclic buffer, move
//	        uartIn = (uartIn + 1) % UART_QUEUE_LEN;
//			// cyclic queue is full - 1bit signalization
//	        if (uartIn == uartOut){
//	            uartFull = TRUE;
//	        }
//	    }
//	    
//	    // timer replaced
//        // start timed message sending - better performance in event driven OS
//        timedUartSendTask();
//
//		return SUCCESS;
//	}
	

	event void AMSend.sendDone(message_t *msg, error_t error){
		// TODO Auto-generated method stub
	}
}
