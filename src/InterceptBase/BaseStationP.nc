/*
 * Copyright (c) 2000-2005 The Regents of the University  of California.  
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the University of California nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright (c) 2002-2005 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE     
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA, 
 * 94704.  Attention:  Intel License Inquiry.
 */

/*
 * @author Phil Buonadonna
 * @author Gilman Tolle
 * @author David Gay
 * @author Dimas Abreu Dutra
 */

/* 
 * BaseStationP bridges packets between a serial channel and the radio.
 * Messages moving from serial to radio will be tagged with the group
 * ID compiled into the TOSBase, and messages moving from radio to
 * serial will be filtered by that same group id.
 */

//Defining the preprocessor variable CC2420_NO_ACKNOWLEDGEMENTS will disable all forms of acknowledgments at compile time.
//Defining the preprocessor variable CC2420_HW_ACKNOWLEDGEMENTS will enable hardware acknowledgments and disable software acknowledgments.
#define CC2420_NO_ACKNOWLEDGEMENTS 1
#include "AM.h"
#include "Serial.h"
#include "../RssiDemoMessages.h"

module BaseStationP @safe() {
    uses
    {
        interface Boot;
        interface SplitControl as SerialControl;
        interface SplitControl as RadioControl;

        interface AMSend as UartSend[am_id_t id];
        interface Receive as UartReceive[am_id_t id];
        interface Packet as UartPacket;
        interface AMPacket as UartAMPacket;

        interface AMSend as RadioSend[am_id_t id];
        interface Receive as RadioReceive[am_id_t id];
        interface Receive as RadioSnoop[am_id_t id];
        interface Packet as RadioPacket;
        interface AMPacket as RadioAMPacket;
        
        interface Timer<TMilli> as ResetTimer;
        interface Timer<TMilli> as UartTimer;
        interface Timer<TMilli> as RadioTimer;
        
        interface Reset as Reset;
        interface Leds;
    }
	
	provides {
		// interface for intercepting radio traffic (control forwarding)
    	interface Intercept as RadioIntercept[am_id_t amid];
    	// interface for intercepting serial traffic (control forwarding)
    	interface Intercept as SerialIntercept[am_id_t amid];
    	
    	// msg sending - reporting
    	interface AMSend as SerialSend[am_id_t id];
    	
    	// provide queue on serial for others by queue interface
    	// only enqueue is permited, deque will fail...
    	// sending is handled by internal mechanism, no sendDone is performed 
    	interface Queue<serialqueue_element_t *> as SerialQueue;
    	
    	// just for notification on radio start/stop
    	interface SplitControl as BSRadioControl;
    	interface SplitControl as BSSerialControl;
    	
    	// receive interface for serial is not needed here, messages can be processed in SerialIntercept
    	// if needed... Receiving is not problem with serial interface started...
    	// interface Receive as SerialReceive[am_id_t id]; 
    	
    	// more complex BS configuration
    	interface InterceptBaseConfig;
    	
    	interface AMSend[am_id_t id];
    	interface Receive[am_id_t id];
    	interface Receive as Snoop[am_id_t id];
    	
    	interface AMTap;
    }
}

implementation
{  
    enum {
        UART_QUEUE_LEN = 32,
        RADIO_QUEUE_LEN = 16,
        TIME_TO_RESET=10000,
        UART_TIME=3,
        RADIO_TIME=3,
        RESET_TIME=400,
        
        UART_RESET_THRESHOLD=30,
    };

    // serial queue & management
    // uartQueue = queue of received radio packets 
    // waiting to be send over UART.
    // Real messages are stored in bufs. Pointers to this
    // queue is given as return value of Packet.receive 
    // -> where to store next message.
    message_t uartQueueBufs[UART_QUEUE_LEN];
    // array of pointers to messages in queue. Messages are really stored 
    // in buffs or in 1 message queue in receive buffer.
    message_t * uartQueue[UART_QUEUE_LEN];
    // queue for external packets. If not null, then packet on specified position is
    // from external source
    //  a) keeps message original pointer for messageSent notif
    //  b) signalization purposes for special handling with external messages
    message_t * uartQueueExternal[UART_QUEUE_LEN];
    uint8_t uartIn, uartOut;
    bool uartBusy, uartFull;
    // indicator - if 1 then packet in uart queue 
    // is already serial packet - important for determining 
    // source, destination...
    uint8_t uartPacketSerial[UART_QUEUE_LEN/8+1];
    // signalizes whether packet is enqueued from queue
    uint8_t uartPacketExternal[UART_QUEUE_LEN/8+1];

    // radio queue & management
    // radioQueue = queue of received uart packets 
    // waiting to be send over radio    
    message_t radioQueueBufs[RADIO_QUEUE_LEN];
    message_t * radioQueue[RADIO_QUEUE_LEN];
    // queue for external packets. If not null, then packet on specified position is
    // from external source
    //  a) keeps message original pointer for messageSent notif
    //  b) signalization purposes for special handling with external messages
    message_t * radioQueueExternal[RADIO_QUEUE_LEN];
    uint8_t radioIn, radioOut;
    bool radioBusy, radioFull;
    // source, destination...
    uint8_t radioPacketSerial[RADIO_QUEUE_LEN/8+1];
    // signalizes whether packet is enqueued from queue
    uint8_t radioPacketExternal[RADIO_QUEUE_LEN/8+1];

	// is TRUE then forward by default
	bool globalRadioForward=TRUE;
	bool globalSerialForward=TRUE;
	bool defaultRadioForward=FALSE;
	bool defaultSerialForward=FALSE;
	bool addressRecognition=TRUE;
	bool radioSnooping=TRUE;

    //uint8_t count = 0;
    uint8_t tmpLen;
    
    uint8_t uartFailCounter = 0;
    bool inReset=FALSE;
    int8_t resetPhase=0;

    // forward declarations
    task void uartSendTask();
    task void radioSendTask();
    void timedUartSendTask();
    message_t * receive(message_t *msg, void *payload, uint8_t len, am_id_t id, bool snoop);

    void sucBlink() {
        // no blibking on sucess
        // - overhead
        
        call Leds.led0Toggle();
    }

    void sucRadioBlink() {
        // no blibking on sucess
        // - overhead
        
        call Leds.led0Toggle();
    }

    // queue full signalization
    void dropBlink() {
        call Leds.led1Toggle();
    }

    // send failed signalization
    void failBlink() {
        call Leds.led2Toggle();
    }
    
    // event timer for UART - send next packet
	event void UartTimer.fired() {
		if(inReset == TRUE) 
			return;

		// if over, perform HW restart with watchdog
		if(uartFailCounter > UART_RESET_THRESHOLD) {
			uartFailCounter = 0;
			inReset = TRUE;

			dropBlink();
			//call Reset.reset();

			// initiate restart. STOP radio, STOP serial, START serial, START radio;
			//call RadioControl.stop();
		}
		else {
			// thresholds are OK, just send message
			post uartSendTask();
		}
    }
    
    // event timer for radio - send next packet
    event void RadioTimer.fired(){
        post radioSendTask();
    }
    
    // timer controls radio reset - if something went wrong
    // in radio stack, it may become unresponsible and restart is needed
    // reset works in 4 phases: radio stop, serial stop, serial start, radio start 
    // each resetTimer tick should change phase.
    // on phase -1 is performed HW reset (watchdog)
    event void ResetTimer.fired(){
        error_t curError;
        
        // if we are not in reset, stop execution
        if (inReset==FALSE){
            call ResetTimer.stop();
            return;
        }
        
        atomic{
            if (resetPhase==-1){
                // hard software reset of node
                resetPhase=4;
                failBlink();
                call Reset.reset();
                
                // this code should not be reached
                // otherwise hardware reset does not work...
                dropBlink();
                sucBlink();
            }
            
            // depending on current reset phase changes are made to devices
            // there are 4 phases at all 
            if (resetPhase==0){
                // phase 0 = stop radio
                curError = call RadioControl.stop();
                
                // if curError EALREADY => already stoped, move to the next phase
                if (curError == EALREADY){
                    // already stopped, move next
                    ++resetPhase;
                } else if (curError == EBUSY){
                    //EBUSY if the component is in the middle of powering up i.e. a start() command has been called, and a startDone() event is pending
                    //resetPhase=4;
                    failBlink();
                    call ResetTimer.startOneShot(RESET_TIME);
                    return;
                } else if (curError == SUCCESS){
                    // it is OK here, wait for event; do nothing
                    dropBlink();
                    return;
                } else {
                    // fail here, try again to power off with delay                
                    failBlink();
                    call ResetTimer.startOneShot(RESET_TIME);
                    return;
                }
            }
                
            if (resetPhase==1){
                // phase 1 = stop serial
                curError = call SerialControl.stop();
                
                // if curError EALREADY => already stoped, move to the next phase
                if (curError == EALREADY){
                    ++resetPhase;
                } else if (curError == EBUSY){
                    //EBUSY if the component is in the middle of powering up i.e. a start() command has been called, and a startDone() event is pending
                    //resetPhase=3;
                    failBlink();
                    call ResetTimer.startOneShot(RESET_TIME);
                    return;
                } else if (curError == SUCCESS){
                    // it is OK here, wait for event; do nothing
                    dropBlink();
                    return;
                } else {
                    // fail here, try again to power off with delay
                    failBlink();
                    call ResetTimer.startOneShot(RESET_TIME);
                    return;
                }
            }
                
            if (resetPhase==2){
                // phase 2 = reset queue state + start serial
                uartIn = uartOut = 0;
                uartBusy = FALSE;
                uartFull = TRUE;

                // radio queue init
                radioIn = radioOut = 0;
                radioBusy = FALSE;
                radioFull = TRUE;
        
                curError = call SerialControl.start();
                
                // if curError EALREADY => already stoped, move to the next phase
                if (curError == EALREADY){
                    ++resetPhase;
                    call ResetTimer.startOneShot(RESET_TIME);
                    return;
                } else if (curError == EBUSY){
                    //EBUSY if the component is in the middle of powering down i.e. a stop() command has been called, and a stopDone() event is pending
                    failBlink();
                    
                    resetPhase=2;
                    call ResetTimer.startOneShot(RESET_TIME);
                    return;
                } else if (curError == SUCCESS){
                    // it is OK here, wait for event; do nothing
                    dropBlink();
                    return;
                } else {
                    // fail here, try again to power off with delay
                    failBlink();
                    call ResetTimer.startOneShot(RESET_TIME);
                    return;
                }
            }
                
            if (resetPhase==3){
                // phase 3 = start radio
                curError = call RadioControl.start();
                
                // if curError EALREADY => already stoped, move to the next phase
                if (curError == EALREADY){
                    ++resetPhase;
                    call ResetTimer.startOneShot(RESET_TIME);
                    return;
                } else if (curError == EBUSY){
                    //EBUSY if the component is in the middle of powering down i.e. a stop() command has been called, and a stopDone() event is pending
                    resetPhase=3;
                    failBlink();
                    call ResetTimer.startOneShot(RESET_TIME);
                    return;
                } else if (curError == SUCCESS){
                    // it is OK here, wait for event; do nothing
                    dropBlink();
                    return;
                } else {
                    // fail here, try again to power off with delay
                    failBlink();
                    call ResetTimer.startOneShot(RESET_TIME);
                    return;
                }
            }
             
            if (resetPhase==4){
                // reset complete, stop reset timer
                call ResetTimer.stop();
                inReset=FALSE;
                resetPhase=0;
                
                sucBlink();
                dropBlink();
                failBlink();
           }
        }
    }

    // event handler, on system boot
    // perform init tasks
    // prepare queues, starts interfaces
    event void Boot.booted() {
        uint8_t i;

        // serial queue init
        for (i = 0; i < UART_QUEUE_LEN; i++) {
            uartQueue[i] = &uartQueueBufs[i];
            uartQueueExternal[i] = NULL;
        }
        uartIn = uartOut = 0;
        uartBusy = FALSE;
        uartFull = TRUE;

        // radio queue init
        for (i = 0; i < RADIO_QUEUE_LEN; i++) {
            radioQueue[i] = &radioQueueBufs[i];
        }
        radioIn = radioOut = 0;
        radioBusy = FALSE;
        radioFull = TRUE;        
        // prepare interfaces by special way of reset

        inReset=TRUE;
        resetPhase=2;
        call ResetTimer.startOneShot(RESET_TIME);
 

		failBlink();
		
//		// start radio & serial
//        call RadioControl.start();
//        call SerialControl.start();
    }

    // event handler, radio start done
    event void RadioControl.startDone(error_t error) {
        uint8_t i;
        
        // sucessfull starting
        if (error == SUCCESS || error == EALREADY) {
            sucBlink();
            atomic {
                for (i = 0; i < RADIO_QUEUE_LEN; i++) {
                    radioQueue[i] = &radioQueueBufs[i];
                }
                radioIn = radioOut = 0;
                radioBusy = FALSE;
                radioFull = FALSE;
                
                // sucessfull reset, move to next phase
                ++resetPhase;
                call ResetTimer.startOneShot(RESET_TIME);
            }
            
            // signalize 
            signal BSRadioControl.startDone(error);
        } else {
            failBlink();
            dropBlink();
            
            // stagnation, try again
            call ResetTimer.startOneShot(RESET_TIME);
        }
    }

    // event handler, serial start done
    event void SerialControl.startDone(error_t error) {
        uint8_t i;
        if (error == SUCCESS || error == EALREADY) {
            sucBlink();
            atomic {
                // serial queue init
                for (i = 0; i < UART_QUEUE_LEN; i++) {
                    uartQueue[i] = &uartQueueBufs[i];
                    uartQueueExternal[i] = NULL;
                }
                uartIn = uartOut = 0;
                uartBusy = FALSE;
                uartFull = FALSE;
                
                // sucessfull reset, move to next phase
                ++resetPhase;
                call ResetTimer.startOneShot(RESET_TIME);
            }
            
            // signalize 
            signal BSSerialControl.startDone(error);
        } else {
            failBlink();
            dropBlink();
            
            // stagnation, try again
            call ResetTimer.startOneShot(RESET_TIME);
        }
    }

	// event handler - serial interface was stopped
    event void SerialControl.stopDone(error_t error) {
        if (error == SUCCESS) {
            sucBlink();
            
            // sucessfull reset, move to next phase
            resetPhase++;
            call ResetTimer.startOneShot(RESET_TIME);
            
            // signalize 
            signal BSSerialControl.stopDone(error);
        } else {
            failBlink();
            dropBlink();
            
            // stagnation, try again
            call ResetTimer.startOneShot(RESET_TIME);
        }
    }

    // event handler - radio interface was stopped
    event void RadioControl.stopDone(error_t error) {
        if (error == SUCCESS) {
            sucBlink();
            
            // sucessfull reset, move to next phase
            resetPhase++;
            call ResetTimer.startOneShot(RESET_TIME);
            
            // signalize 
            signal BSRadioControl.stopDone(error);
        } else {
            call RadioControl.stop();
            failBlink();
            dropBlink();
            
            // stagnation, try again
            call ResetTimer.startOneShot(RESET_TIME);
        }
    }

	// event handler - message snooped on radio interface, passing to general function receive
    event message_t * RadioSnoop.receive[am_id_t id](message_t *msg, void *payload, uint8_t len) {
    	if (radioSnooping==FALSE){
    		return msg;	
    	}
    	
    	// tapping interface
    	signal AMTap.snoop(id, msg, payload, len);
    	
    	// normal snoop interface
    	signal Snoop.receive[id](msg, payload, len);
    	
    	// forward to default receive
        return receive(msg, payload, len, id, TRUE);
    }

	// event handler - message snooped on radio interface, passing to general function receive
    event message_t * RadioReceive.receive[am_id_t id](message_t *msg, void *payload, uint8_t len) {
    	
    	// tapping interface
    	signal AMTap.receive(id, msg, payload, len);
    	
    	// normal receive interface
    	signal Receive.receive[id](msg, payload, len);
    	
    	// forward to default receive
        return receive(msg, payload, len, id, FALSE);
    }

    // message received from radio here
    // decide whether to forward it to serial port
    message_t * receive(message_t *msg, void *payload, uint8_t len, am_id_t id, bool snoop) {
        message_t *ret = msg;
        
        // if in reset do nothing for now
        if (inReset==TRUE) return ret;
        
        // global radio forward is disabled->no received packet from radio to serial
        // will be re-sended
        if (globalRadioForward==FALSE){
        	return ret;
        }
        
        atomic
        {
	        // decision point if message should be forwarded or ignored
	        // in this signal processing function can be packet processed
	        if (!(signal RadioIntercept.forward[id](msg, payload, len))){
	        	// packet is not interesting for me, skip
	            return ret;
	        }
	                
            // if serial queue is not full, we can put message to it
            // as consequence message received from radio is forwarded to
            // UART queue to be sent over UART to application
            if (!uartFull) {
            	// pointer to free memory block where to store new message
            	// is stored to ret. Here will be stored new message received
            	// in next event.
                ret = uartQueue[uartIn];
                // to current free place in queue is stored actualy received 
                // message from radio.
                uartQueue[uartIn] = msg;
                // signalize that this message is internal
                uartQueueExternal[uartIn] = NULL;
                // packet is from radio
				uartPacketSerial[uartIn/8] &= ~(1<<(uartIn%8));
				// next slot in queue - cyclic buffer, move
                uartIn = (uartIn + 1) % UART_QUEUE_LEN;
				// cyclic queue is full - 1bit signalization
                if (uartIn == uartOut){
                    uartFull = TRUE;
                }

                // timer replaced
                // start timed message sending - better performance in event driven OS
                timedUartSendTask();
            } else {
                // serial queue full         
                failBlink();
                dropBlink();

                ++uartFailCounter;
                timedUartSendTask();
                // reset with restarting
                //inReset=TRUE;
                //resetPhase=0; 
                //call Reset.reset();
                //call ResetTimer.startOneShot(RESET_TIME);
            }
        }

        return ret;
    }

	// starts timer for UART message sending if applicable
    void timedUartSendTask(){
        // do nothing if in middle of reset process
        if (inReset==TRUE) return;
        
        //post uartSendTask();
        //return;
        
        // timer replaced
        //post uartSendTask();
        if (call UartTimer.isRunning()==FALSE){
            call UartTimer.startOneShot(UART_TIME);
        }
    }

    // task, send data to serial from queue
    // manage queue
    task void uartSendTask() {
        uint8_t len;
        am_id_t id;
        am_addr_t addr, src;
        error_t sendError;
        message_t* msg;
        
        // busy uart? already sending something?
        if (uartBusy){
        	post uartSendTask();
        	return;
        }
        
        // inPointer==outPointer and queue is not full => queue is empty
        // nothing to do. return
        atomic {
	        if (uartIn == uartOut && !uartFull) {
	            uartBusy = FALSE;
	            return;
	        }
	
	        msg = uartQueue[uartOut];
	        
	        // depending on packet type
	        if ((uartPacketSerial[uartOut/8] & (1<<(uartOut%8)))>0) {
	        	// packet is serial - pushed by amsend or enqueued
	        	tmpLen = len = call UartPacket.payloadLength(msg);
	        	id = call UartAMPacket.type(msg);
	        	addr = call UartAMPacket.destination(msg);
	        	src = call UartAMPacket.source(msg);
	        } else {
	        	// otherwise packet is radio type and came from radio interface
	        	tmpLen = len = call RadioPacket.payloadLength(msg);
	        	id = call RadioAMPacket.type(msg);
	        	addr = call RadioAMPacket.destination(msg);
	        	src = call RadioAMPacket.source(msg);
	        }
	        
	        // set packet source, ok
	        call UartAMPacket.setSource(msg, src);
	
	        // try to send with SerialActiveMessageC component
	        sendError = call UartSend.send[id](addr, uartQueue[uartOut], len);
	        switch(sendError){
	            case SUCCESS:
	                //sucBlink();
	                uartBusy=TRUE;
	                uartFailCounter=0;
	                break;
	                
	            // was size of packet correct?
	            case ESIZE:
	                // size is problem, remove packet from queue to send
	                //++uartFailCounter;
	                failBlink();
	                
	                // remove here not to waste space in queue
	                if (++uartOut >= UART_QUEUE_LEN)
	                    uartOut = 0;
	                if (uartFull)
	                    uartFull = FALSE;
	                
	                break;
	                
	            case EBUSY:
	                // link is busy, try again later
	                
	                break;
	                
	            case FAIL:
	                // general fail
	                //++uartFailCounter;
	                failBlink();
	                break;
	        }
        }
            
        // try to send everything again
        // (before this was only as reaction on FAIL, if causes problem, remove
        // it from global case and add only as reaction on FAIL
        if (sendError!=SUCCESS){
        	timedUartSendTask();
        }
    }

    // event handler, serial send done
    event void UartSend.sendDone[am_id_t id](message_t* msg, error_t error) {            
        if (error != SUCCESS){
            failBlink();
            //++uartFailCounter;
        } else {
        	// signalize exernal message sent?
        	bool signalizeExternal=FALSE;
        	// pointer
            uint8_t uartOutPrev = uartOut;
        	
            uartFailCounter=0;
            sucBlink();
            
            atomic {
            	// if there is any specific parametrized sender, this could be its message
	            if (msg == uartQueue[uartOut]) {
	            	uartBusy = FALSE;
	            	
	            	// can signalize external out
					signalizeExternal=uartQueueExternal[uartOut]!=NULL;
	            	
	            	// out pointer is incremented, equivalent of (uartOut+1 `mod` UART_QUEUE_LEN)
	                if (++uartOut >= UART_QUEUE_LEN)
	                    uartOut = 0;
	                // queue will definitelly cannot be still full after 1 element removal.
	                // equivalent: uartFull = FALSE;
	                if (uartFull)
	                    uartFull = FALSE;
	            }
            }
            
            // signalize external, outside of atomic block
            if (signalizeExternal){
            	// signalize with original message address pointer - for reciever to be able 
            	// to pair this event with action
            	signal SerialSend.sendDone[id](uartQueueExternal[uartOutPrev], SUCCESS);
            }
        }
        
        // timer replaced
        //post uartSendTask();
        timedUartSendTask();
    }

    // event handler, receive message on serial
    event message_t * UartReceive.receive[am_id_t id](message_t *msg, void *payload, uint8_t len) {
        message_t *ret = msg;
        bool reflectToken = FALSE;
        
        // global serial forwarding disabled
		if (globalSerialForward==FALSE){
			return ret;
		}
        
        // should I forward this message catched on serial to radio?
        if (!signal SerialIntercept.forward[id](msg, payload, len)){
            return ret;
        }

        atomic {
	        if (!radioFull) {
	            reflectToken = TRUE;
	            ret = radioQueue[radioIn];
	            radioQueue[radioIn] = msg;
	            
	            // signalize that this message is internal
                radioQueueExternal[radioIn] = NULL;
                // packet is from serial
				radioPacketSerial[radioIn/8] |= (1<<(radioIn%8));
	            
	            if (++radioIn >= RADIO_QUEUE_LEN)
	                radioIn = 0;
	            if (radioIn == radioOut)
	                radioFull = TRUE;
	
	            if (!radioBusy) {
	                // timer replaced
	                post radioSendTask();
//	                call RadioTimer.startOneShot(RADIO_TIME);
//	                radioBusy = TRUE;
	            }
	            
	        } else {
	        	post radioSendTask();
	            // dropBlink();
	        }
        }

        if (reflectToken) {
            //call UartTokenReceive.ReflectToken(Token);
        }

        return ret;
    }

    // task, send data over radio
    // we are sending data from radio queue
    task void radioSendTask() {
        uint8_t len;
        am_id_t id;
        am_addr_t addr, src;
        message_t* msg;

		if (radioBusy){
			post radioSendTask();
		}

        atomic
        if (radioIn == radioOut && !radioFull) {
            radioBusy = FALSE;
            return;
        }

        msg = radioQueue[radioOut];
        
        // depending on packet type
        if ((radioPacketSerial[radioOut/8] & (1<<(radioOut%8)))>0) {
        	// packet is serial - pushed by amsend or enqueued
        	tmpLen = len = call UartPacket.payloadLength(msg);
        	id = call UartAMPacket.type(msg);
        	addr = call UartAMPacket.destination(msg);
        	src = call UartAMPacket.source(msg);
        } else {
        	// otherwise packet is radio type and came from radio interface
        	tmpLen = len = call RadioPacket.payloadLength(msg);
        	id = call RadioAMPacket.type(msg);
        	addr = call RadioAMPacket.destination(msg);
        	src = call RadioAMPacket.source(msg);
        }
        
        // set packet source, ok
	    call RadioAMPacket.setSource(msg, src);
        
        if (call RadioSend.send[id](addr, msg, len) == SUCCESS){
        	radioBusy=TRUE;
            //sucRadioBlink();
        }
        else {
            failBlink();
            
            // timer replaced
            //post radioSendTask();
            call RadioTimer.startOneShot(RADIO_TIME);
        }
    }

    // event handler, radio send done, remove from queue if successfull
    event void RadioSend.sendDone[am_id_t id](message_t* msg, error_t error) {
        if (error != SUCCESS){
            failBlink();
        }
        else {
        	// signalize exernal message sent?
        	bool signalizeExternal=FALSE;
        	// pointer
            uint8_t radioOutPrev = radioOut;
            
            // blink on success transmission
            sucRadioBlink();
            
            atomic{
            	// here we could receive message from different parametrized sender
	            if (msg == radioQueue[radioOut]) {
	            	radioBusy=FALSE;
	            	
	            	// can signalize external out
					signalizeExternal=uartQueueExternal[uartOut]!=NULL;
						
	                if (++radioOut >= RADIO_QUEUE_LEN)
	                    radioOut = 0;
	                if (radioFull)
	                    radioFull = FALSE;
	            }
	        }
            
             // signalize external, outside of atomic block
            if (signalizeExternal){
            	// signalize with original message address pointer - for reciever to be able 
            	// to pair this event with action
            	signal AMSend.sendDone[id](uartQueueExternal[radioOutPrev], SUCCESS);
            }
        }

        // timer replaced
        post radioSendTask();
        //call RadioTimer.startOneShot(RADIO_TIME);
    }

    // decision function, should be current message catched on radio forwarded to serial?
    // is usually overriden in component using this interface
    default event bool RadioIntercept.forward[am_id_t amid](message_t* msg, void* payload, uint8_t len){
    	// by default do not forward messages that are directly for me
    	if ((call RadioAMPacket.destination(msg)) == TOS_NODE_ID){
    		return FALSE;
    	} 
    	
    	// next action depends on default settings - can be set from outside
		return defaultRadioForward;
    }

    // shold be message cathed on serial forwarded to radio?
    default event bool SerialIntercept.forward[am_id_t amid](message_t* msg, void* payload, uint8_t len){
    	// by default do not forward messages that are directly for me
    	if ((call UartAMPacket.destination(msg)) == TOS_NODE_ID){
    		return FALSE;
    	}
    	
    	// next action depends on default settings - can be set from outside
		return defaultSerialForward;
    }

	/***************** AMSend Commands ****************/
	/**
	 * SerialSend interface - add message to send to message queue
	 */
	default event void SerialSend.sendDone[uint8_t id](message_t* msg, error_t err) {
		return;
	}

	// kind of schyzophrenic interface - returning radio's get payload 
	// there is assumption that in queue are radio packets - hold it
	command void * SerialSend.getPayload[am_id_t id](message_t *msg, uint8_t len){
		return call UartSend.getPayload[id](msg, len);
	}

	command uint8_t SerialSend.maxPayloadLength[am_id_t id](){
		return call UartSend.maxPayloadLength[id]();
	}

	command error_t SerialSend.cancel[am_id_t id](message_t *msg){
		// does not support cacneling already queued packet - sorry
		// @TODO: may be fixed by another indicator - set NULL, such messages will be skipped
		return FAIL;
	}

	// similar to received() function
	command error_t SerialSend.send[am_id_t id](am_addr_t addr, message_t *msg, uint8_t len){
		message_t * ret;        
        // if in reset do nothing for now
        if (inReset==TRUE){
        	// cannot send anything
        	return EBUSY;
        } 
	
		// queue manipulation has to be atomic to preserve pointers/counters consistency
        atomic
        {
            // if serial queue is not full, we can put message to it
            // as consequence message received from radio is forwarded to
            // UART queue to be sent over UART to application
            if (!uartFull) {
            	// pointer to free memory block. copy here message passed for sending
                ret = uartQueue[uartIn];
                // copy whole message to defines address, preserve my addresses for 
                // future - N+1 cycling queue for receive
                //memcpy(ret, msg, sizeof(message_t));
                memcpy(call UartPacket.getPayload(ret, len), 
		        	   call UartPacket.getPayload(msg, len), 
		        	   len);
                // assumes that packet is serial
                uartPacketSerial[uartIn/8] |= (1<<(uartIn%8));
                // set fields as radio packet - BaseStation assumes that radio 
                // packets are in this queue -> no problem, next calls will
                // consider message as radio message and puts correct fields
                call UartAMPacket.setDestination(ret, addr);
                call UartAMPacket.setType(ret, id);
                call UartAMPacket.setSource(ret, TOS_NODE_ID);
                call UartPacket.setPayloadLength(ret, len);
                
                // store external message pointer - indicates externality, provides binding
                // for messageSent event
                uartQueueExternal[uartIn] = msg;
                
                // to current free place in queue is stored actualy received 
                // message from radio.
                uartQueue[uartIn] = ret;
				// next slot in queue - cyclic buffer, move
                uartIn = (uartIn + 1) % UART_QUEUE_LEN;

				// cyclic queue is full - 1bit signalization
                if (uartIn == uartOut){
                    uartFull = TRUE;
                }

                // timer replaced
                // start timed message sending - better performance in event driven OS
                timedUartSendTask();
            } else {
                // serial queue full       
                failBlink();
                dropBlink();
                ++uartFailCounter;
                
                timedUartSendTask();
                
                return ENOMEM;
            }
        }

        return SUCCESS;
	}

	/************************* SPLIT CONTROL *********************************/
	command error_t BSRadioControl.stop(){
		// not supported, just observer
		return FAIL;
	}

	command error_t BSRadioControl.start(){
		// not supported, just observer
		return FAIL;
	}

	command error_t BSSerialControl.start(){
		// not supported, just observer
		return FAIL;
	}

	command error_t BSSerialControl.stop(){
		// not supported, just observer
		return FAIL;
	}
	
	default event void BSRadioControl.startDone(error_t error) {
		return;
	}
	
	default event void BSRadioControl.stopDone(error_t error) {
		return;
	}
	
	default event void BSSerialControl.startDone(error_t error) {
		return;
	}	
	
	default event void BSSerialControl.stopDone(error_t error) {
		return;
	}
	
	
	//uartIn, uartOut;
    //bool uartBusy, uartFull;
	/************************* SERIAL QUEUE *********************************/
	command bool SerialQueue.empty() {
		return uartFull==FALSE && uartIn==uartOut;
	}

	command uint8_t SerialQueue.size() {
		return (uartOut > uartIn) ? UART_QUEUE_LEN - uartOut + uartIn : uartIn - uartOut;
	}

	command uint8_t SerialQueue.maxSize() {
		return UART_QUEUE_LEN;
	}

	command error_t SerialQueue.enqueue(serialqueue_element_t * newVal){
		message_t * ret; 

		// if queue is full, cannot add new
		if (uartFull) {
                // serial queue full       
                failBlink();
                dropBlink();
                ++uartFailCounter;
                timedUartSendTask();
                return ENOMEM;
        }
		
		// queue manipulation has to be atomic to preserve pointers/counters consistency
        atomic
        {
			// check full again
			if (uartFull) {
				timedUartSendTask();
				return ENOMEM;
			}
			// now is guaranted that nothing happened to queue from last check
			
	    	// pointer to free memory block. copy here message passed for sending
	        ret = uartQueue[uartIn];

	        // ser proper packet-type flag
	        if (newVal->isRadioMsg){
	        	uartPacketSerial[uartIn/8] &= ~(1<<(uartIn%8));
	        } else {
	        	uartPacketSerial[uartIn/8] |= (1<<(uartIn%8));
	        }
	        
	        // set always to 1 here
	        uartPacketExternal[uartIn/8] |= (1<<uartIn%8);
	        
	        //memcpy(ret, newVal->msg, sizeof(message_t));
	        
	        // set fields as radio packet - BaseStation assumes that radio 
	        // packets are in this queue -> no problem, next calls will
	        // consider message as radio message and puts correct fields
	        if (newVal->isRadioMsg){
	        	// copy whole message to defines address, preserve my addresses for 
	        	// future - N+1 cycling queue for receive
		        memcpy(call RadioPacket.getPayload(ret, newVal->len), 
		        	newVal->payload, newVal->len);
		        
	        	
	        	call RadioAMPacket.setDestination(ret, newVal->addr);
	        	call RadioAMPacket.setType(ret, newVal->id);
	        	call RadioAMPacket.setSource(ret, TOS_NODE_ID);
	        	call RadioPacket.setPayloadLength(ret, newVal->len);
	        } else {
	        	// copy whole message to defines address, preserve my addresses for 
		        // future - N+1 cycling queue for receive
	        	memcpy(call UartPacket.getPayload(ret, newVal->len), 
	        		newVal->payload, newVal->len);
	        	
	        	call UartAMPacket.setDestination(ret, newVal->addr);
	        	call UartAMPacket.setType(ret, newVal->id);
	        	call UartAMPacket.setSource(ret, TOS_NODE_ID);
	        	call UartPacket.setPayloadLength(ret, newVal->len);
	        }
	        
	        // store external message pointer - when enqueueing new messages then do not
	        // signal sendDone, massive sending. 
	        uartQueueExternal[uartIn] = NULL;
	        // to current free place in queue is stored actualy received 
	        // message from radio.
	        uartQueue[uartIn] = ret;
			// next slot in queue - cyclic buffer, move
	        uartIn = (uartIn + 1) % UART_QUEUE_LEN;
			// cyclic queue is full - 1bit signalization
	        if (uartIn == uartOut){
	            uartFull = TRUE;
	        }
	    }
	    
	    // timer replaced
        // start timed message sending - better performance in event driven OS
        timedUartSendTask();

		return SUCCESS;
	}

	command serialqueue_element_t * SerialQueue.head(){
		return NULL;
	}

	command serialqueue_element_t * SerialQueue.dequeue(){
		return NULL;
	}

	command serialqueue_element_t * SerialQueue.element(uint8_t idx){
		return NULL;
	}



/**
 * InterceptBaseConfig
 */
	command void InterceptBaseConfig.setRadioSnoopEnabled(bool enabled){
		radioSnooping=enabled;
	}

	command bool InterceptBaseConfig.getDefaultSerialFilteringEnabled(){
		return radioSnooping;
	}

	command void InterceptBaseConfig.setDefaultRadioFilteringEnabled(bool enabled){
		defaultRadioForward=!enabled;
	}

	command bool InterceptBaseConfig.getDefaultRadioFilteringEnabled(){
		return !defaultRadioForward;
	}

	command void InterceptBaseConfig.setGlobalSerialFilteringEnabled(bool enabled){
		globalSerialForward=!enabled;
	}

	async command bool InterceptBaseConfig.getRadioSnoopEnabled(){
		return !globalSerialForward;
	}

	command void InterceptBaseConfig.setAddressRecognitionEnabled(bool enabled){
		//TODO: write functionality here, need to call CC2420 config...
		
		
		
		addressRecognition=enabled;
	}

	async command bool InterceptBaseConfig.getAddressRecognitionEnabled(){
		return addressRecognition;
	}

	command void InterceptBaseConfig.setDefaultSerialFilteringEnabled(bool enabled){
		defaultSerialForward=!enabled;
	}

	command void InterceptBaseConfig.setGlobalRadioFilteringEnabled(bool enabled){
		 globalRadioForward=!enabled;
	}

	command bool InterceptBaseConfig.getGlobalSerialFilteringEnabled(){
		return !globalSerialForward;
	}

	command error_t InterceptBaseConfig.sync(){
		return SUCCESS;
	}

	command bool InterceptBaseConfig.getGlobalRadioFilteringEnabled(){
		return !globalRadioForward;
	}

	command uint8_t InterceptBaseConfig.getRadioQueueFree(){
		return RADIO_QUEUE_LEN - ((radioOut > radioIn) ? RADIO_QUEUE_LEN - radioOut + radioIn : radioIn - radioOut);
	}

	command uint8_t InterceptBaseConfig.getSerialQueueFree(){
		return UART_QUEUE_LEN - ((uartOut > uartIn) ? UART_QUEUE_LEN - uartOut + uartIn : uartIn - uartOut);
	}
	
	command uint8_t InterceptBaseConfig.getSerialFailed(){
		return uartFailCounter;
	}		

    default event message_t* Receive.receive[am_id_t amid](message_t* msg, void* payload, uint8_t len){
    	return msg;
    }
    
    default event message_t* Snoop.receive[am_id_t amid](message_t* msg, void* payload, uint8_t len){
    	return msg;
    }    

	command void * AMSend.getPayload[am_id_t id](message_t *msg, uint8_t len){
		return call RadioPacket.getPayload(msg, len);
	}

	command uint8_t AMSend.maxPayloadLength[am_id_t id](){
		return call RadioPacket.maxPayloadLength();
	}

	command error_t AMSend.cancel[am_id_t id](message_t *msg){
		// cancelmap here
		return FAIL;
	}

	command error_t AMSend.send[am_id_t id](am_addr_t addr, message_t *msg, uint8_t len){
		message_t * ret;        
        // if in reset do nothing for now
        if (inReset==TRUE){
        	// cannot send anything
        	return EBUSY;
        } 
	
		// queue manipulation has to be atomic to preserve pointers/counters consistency
        atomic
        {
            // if serial queue is not full, we can put message to it
            // as consequence message received from radio is forwarded to
            // radio queue to be sent over radio to application
            if (!radioFull) {
            	// pointer to free memory block. copy here message passed for sending
                ret = radioQueue[radioIn];
                // copy whole message to defines address, preserve my addresses for 
                // future - N+1 cycling queue for receive
                //memcpy(ret, msg, sizeof(message_t));
                memcpy(call RadioPacket.getPayload(ret, len), 
		        	   call RadioPacket.getPayload(msg, len), 
		        	   len);
                // assumes that packet is from radio
                radioPacketSerial[radioIn/8] &= ~(1<<(radioIn%8));
                // set fields as radio packet - BaseStation assumes that radio 
                // packets are in this queue -> no problem, next calls will
                // consider message as radio message and puts correct fields
                call RadioAMPacket.setDestination(ret, addr);
                call RadioAMPacket.setType(ret, id);
                call RadioAMPacket.setSource(ret, TOS_NODE_ID);
                call RadioPacket.setPayloadLength(ret, len);
                
                // store external message pointer - indicates externality, provides binding
                // for messageSent event
                radioQueueExternal[radioIn] = msg;
                
                // to current free place in queue is stored actualy received 
                // message from radio.
                radioQueue[radioIn] = ret;
				// next slot in queue - cyclic buffer, move
                radioIn = (radioIn + 1) % RADIO_QUEUE_LEN;

				// cyclic queue is full - 1bit signalization
                if (radioIn == radioOut){
                    radioFull = TRUE;
                }

                // timer replaced
                // start timed message sending - better performance in event driven OS
                post radioSendTask();
            } else {
                // serial queue full       
                failBlink();
                dropBlink();
                
                post radioSendTask();             
                return ENOMEM;
            }
        }

        return SUCCESS;
	}
	
	default event void AMSend.sendDone[uint8_t id](message_t* msg, error_t err) {
		return;
	}
	
	/* Packet receive */
  	default event message_t* AMTap.receive(uint8_t type, message_t* msg, void *payload, uint8_t len){
  		return msg;
  	}
  
  	/* Snoop */
  	default event message_t* AMTap.snoop(uint8_t type, message_t* msg, void *payload, uint8_t len){
 		return msg;		
  	}
 
  	/* Send */
  	default event message_t* AMTap.send(uint8_t type, message_t* msg, uint8_t len){
  		return msg;
  	}  	
}
