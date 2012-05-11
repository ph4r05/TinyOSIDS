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

#ifdef SQDEBUG
#include "printf.h"
#endif

generic module SerialQueueSenderP(typedef queue_t, uint8_t QUEUE_SIZE, am_id_t amtype) @safe() {
  provides {
    interface QueueSender;
  }
  uses {
    interface Boot;
    interface Queue<senderMetadata_t*> as SendQueue;
    interface Pool<queue_t> as MessagePool;
    interface Pool<senderMetadata_t> as MessageMetaPool;
    
    interface Timer<TMilli> as RetxmitTimer;
    interface AMSend;
    interface Packet;
    interface AMPacket;
    
    interface Random;
  }
}
implementation {
  
  enum {
    S_STOPPED,
    S_STARTED,
    S_FLUSHING,
    
    MAX_RETRIES=30,
    
    SENDFAIL_WINDOW=3,
    SENDFAIL_OFFSET=2,
    
    SENDDONEFAIL_WINDOW=3,
    SENDDONEFAIL_OFFSET=2,
  };
  
    message_t uartPacket;
    bool sending;
    uint8_t blen;
    uint16_t statLogReceived = 0;
    uint16_t statEnqueueFail = 0;
    uint16_t statSendFail = 0;
    uint16_t statSendDoneFail = 0;
    uint16_t statSendDoneOk = 0;
    uint16_t statSendDoneBug = 0;
	bool enabled=FALSE;
	uint8_t state = S_STOPPED;
	
	// Forward declarations
	task void sendTask();
	void messageDequeue(senderMetadata_t * metaPtr);
	static void startRetxmitTimer(uint16_t window, uint16_t offset);
	
	event void Boot.booted(){
		sending = FALSE;
        blen = (uint8_t) sizeof(queue_t);
        statSendFail = 0;
        statLogReceived = 0;
        statEnqueueFail = 0;
        statSendDoneOk = 0;
        statSendDoneFail = 0;
        statSendDoneBug = 0;
	}

	command senderMetadata_t * QueueSender.element(uint8_t idx){
		return call SendQueue.element(idx);
	}

	command senderMetadata_t * QueueSender.head(){
		return call SendQueue.head();
	}

	command uint8_t QueueSender.size(){
		return call SendQueue.size();
	}

	command uint8_t QueueSender.maxSize(){
		return call SendQueue.maxSize();
	}

	command bool QueueSender.empty(){
		return call SendQueue.empty();
	}

	command bool QueueSender.full(){
		return ((call SendQueue.size()) == (call SendQueue.maxSize()));
	}

	event void AMSend.sendDone(message_t *msg, error_t error){
		senderMetadata_t * metaPtr = call SendQueue.head();
        if (metaPtr == NULL || (&uartPacket) != msg) {
            //bad mojo - sendqueue contains empty elem, sent message is not right one
            statSendDoneBug++;
#ifdef SQDEBUG            
			printf("badMojo\n");
#endif            
        } else {
        	if (error == SUCCESS){
        		// send successfully, can dequeue entry, return data to pool, start sending again
        		statSendDoneOk++;
				messageDequeue(metaPtr);
				
            	post sendTask(); 
#ifdef SQDEBUG            	
				printf("sentDoneSucc\n");
#endif				            	       		
        	} else {
        		// error occurred - decrement retrycount and react upon 
        		// decrement retry counter
        		statSendDoneFail+=1;
            	metaPtr->retries -= 1;
            	if (metaPtr->retries==0){
            		// message expired, move ahead
            		statSendDoneFail += 1;
            		messageDequeue(metaPtr);
            	}
#ifdef SQDEBUG            	
				printf("sentDoneReTX\n");
#endif				                
                // start retxmit timer - kind of backoff
                startRetxmitTimer(SENDDONEFAIL_WINDOW, SENDFAIL_OFFSET);
        	}
        	
        	// not sending - sentDone
        	sending=FALSE;
        }
	}

	/**
	 * Handles message expired event - retrycount reached 0
	 */
	void messageDequeue(senderMetadata_t * metaPtr){
		atomic {
        	call SendQueue.dequeue();
        	call MessagePool.put((queue_t *)metaPtr->payload);
        	call MessageMetaPool.put(metaPtr);
        }
	}
	
	/**
	 * Starts retransmit timer. Window - size of interval to wait.
	 */
	static void startRetxmitTimer(uint16_t window, uint16_t offset) {
		// if already running - leave alone.
		// can be problem otherwise - every new start request cancel previous started
		// => timer could never fire provided there are subsequent start requests 
		if (call RetxmitTimer.isRunning()){
			return;
		} else {
	    	uint16_t r = call Random.rand16();
	    	r %= window;
	    	r += offset;
	    	call RetxmitTimer.startOneShot(r);
	    	dbg("SerialQueueSender", "Rexmit timer will fire in %hu ms\n", r);
	    }
	}
	
	event void RetxmitTimer.fired(){
#ifdef SQDEBUG
		printf("retxmit\n");
#endif			
		post sendTask();
	}
	
	task void sendTask() {
        if (sending) {
        	// already sending something, do not interfere, exit
            return;
        } else if (call SendQueue.empty()) {
        	// nothing to send, send queue empty, exit
            return;
        } else {
            senderMetadata_t * metaPtr = call SendQueue.head();
            queue_t * smsg = (queue_t *) metaPtr->payload;
            queue_t * msgPayload = (queue_t *) call AMSend.getPayload(&uartPacket, metaPtr->len);
            error_t eval = SUCCESS;
            // copy data to payload
            memcpy((void*)msgPayload, (void*)smsg, metaPtr->len);
            // try so send message
            eval = call AMSend.send(AM_BROADCAST_ADDR, &uartPacket, metaPtr->len);
            if (eval == SUCCESS) {
                sending = TRUE;
#ifdef SQDEBUG                
				printf("sentOK\n");
#endif				                
                return;
            } else {
            	// decrement retry counter
#ifdef SQDEBUG            	
				printf("sentErr: %d; Rtr: %d\n", eval, metaPtr->retries);
#endif				            	
            	metaPtr->retries -= 1;
            	if (metaPtr->retries==0){
            		// message expired, move ahead
            		statSendFail+=1;
            		messageDequeue(metaPtr);
            	}
                
                // try to send next packet
                post sendTask();
            }
        }
    }

	command error_t QueueSender.enqueueData(void *payload, uint8_t len){
		if (enabled==FALSE){
#ifdef SQDEBUG			
			printf("OFF\n");
#endif			
			return EOFF;
		} 
    	
        statLogReceived++;
        if (call MessagePool.empty()) {
        	// no space in message pool, cannot store new data
#ifdef SQDEBUG
			printf("esize1\n");
#endif			        	
            return ESIZE;
        } else if (len > blen) {
        	// cannot accept payload longer that defined
#ifdef SQDEBUG        	
			printf("esize2: yl: %d ; bl: %d\n", len, blen);
#endif			        	
        	return ESIZE;
        } else {
        	queue_t * payloadTyped = (queue_t *)NULL;
            queue_t * msg = (queue_t *)NULL;
            senderMetadata_t * metaPtr = NULL;
        
        	// pools need to be consistent
        	atomic {
	        	// re-type payload as queue_t
	        	payloadTyped = (queue_t *) payload;
	        	// obtain space from pool
	            msg = call MessagePool.get();
	            metaPtr = call MessageMetaPool.get();
            }
            
		    if (msg==NULL || metaPtr==NULL) {
		    	// cannot obtain space for data
		        return FAIL;
		    }
		    
		    // copy obtained payload to obtained one from pool 
	    	memcpy((void*)msg, (void*)payload, len);
	    	// init meta
	    	metaPtr->addr = AM_BROADCAST_ADDR;
			metaPtr->len = len;
			metaPtr->retries = MAX_RETRIES;
			metaPtr->payload = (void*) msg;
			
			// enqueue new data to send queue
            if (call SendQueue.enqueue(metaPtr) == SUCCESS) {
#ifdef SQDEBUG            	
				printf("enqueueOK\n");
#endif					            	
                post sendTask();
                return SUCCESS;
            } else {
#ifdef SQDEBUG            	
				printf("enqueueFAIL\n");
#endif				            	
                statEnqueueFail++;
                atomic {
                	call MessagePool.put(msg);
                	call MessageMetaPool.put(metaPtr);
                }
                return FAIL;
            }
        }
		
		return SUCCESS;
	}

	command error_t QueueSender.sendState(bool start){
		enabled = start;
		return SUCCESS;
	}
}
