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

//#include "Reset.h"
#include "AM.h"
#include "Serial.h"

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

    provides interface Intercept as RadioIntercept[am_id_t amid];
    provides interface Intercept as SerialIntercept[am_id_t amid];
}

implementation
{  
    enum {
        UART_QUEUE_LEN = 32,
        RADIO_QUEUE_LEN = 32,
        TIME_TO_RESET=10000,
        UART_TIME=10,
        RADIO_TIME=5,
        RESET_TIME=300,
        
        UART_RESET_THRESHOLD=20,
    };

    // serial queue & management
    message_t uartQueueBufs[UART_QUEUE_LEN];
    message_t * uartQueue[UART_QUEUE_LEN];
    uint8_t uartIn, uartOut;
    bool uartBusy, uartFull;

    // radio queue & management
    message_t radioQueueBufs[RADIO_QUEUE_LEN];
    message_t * radioQueue[RADIO_QUEUE_LEN];
    uint8_t radioIn, radioOut;
    bool radioBusy, radioFull;

    //uint8_t count = 0;
    uint8_t tmpLen;
    
    uint8_t uartFailCounter = 0;
    bool inReset=FALSE;
    uint8_t resetPhase=0;

    // forward declarations
    task void uartSendTask();
    task void radioSendTask();
    void timedUartSendTask();
    message_t * receive(message_t* msg, void* payload, uint8_t len, am_id_t id);

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
    
    event void UartTimer.fired(){
        if (inReset==TRUE) return;
        
        // if over, perform HW restart with watchdog
        if (uartFailCounter > UART_RESET_THRESHOLD){
            uartFailCounter=0;
            inReset=TRUE;
            
            dropBlink();
            call Reset.reset();
            
            // initiate restart. STOP radio, STOP serial, START serial, START radio;
            //call RadioControl.stop();
        } else {
                post uartSendTask();
        }
    }
    
    event void RadioTimer.fired(){
        post radioSendTask();
    }
    
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
                
                dropBlink();
                sucBlink();
            }
            
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
                    
                    resetPhase=1;
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
                    resetPhase=0;
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

    // eventhandler, on system boot
    // perform init tasks
    // prepare queues, start interfaces
    event void Boot.booted() {
        uint8_t i;

        // serial queue init
        for (i = 0; i < UART_QUEUE_LEN; i++) {
            uartQueue[i] = &uartQueueBufs[i];
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

        // start radio & serial
        //call RadioControl.start();
        //call SerialControl.start();
        
        // prepare interfaces by special way of reset
/*
        inReset=TRUE;
        resetPhase=2;
        call ResetTimer.startOneShot(RESET_TIME);
 * 
*/
        dropBlink();
        call RadioControl.start();
        call SerialControl.start();
    }

    // eventhandler, radio start done
    event void RadioControl.startDone(error_t error) {
        uint8_t i;
        
        // sucessfull starting
        if (error == SUCCESS) {
            sucBlink();
            atomic {
                for (i = 0; i < RADIO_QUEUE_LEN; i++) {
                    radioQueue[i] = &radioQueueBufs[i];
                }
                radioIn = radioOut = 0;
                radioBusy = FALSE;
                radioFull = FALSE;
                
                // sucessfull reset, move to next phase
                resetPhase++;
                call ResetTimer.startOneShot(RESET_TIME);
            }
        } else {
            failBlink();
            dropBlink();
            
            // stagnation, try again
            call ResetTimer.startOneShot(RESET_TIME);
        }
    }

    // eventhandler, serial start done
    event void SerialControl.startDone(error_t error) {
        uint8_t i;
        if (error == SUCCESS) {
            sucBlink();
            atomic {
                // serial queue init
                for (i = 0; i < UART_QUEUE_LEN; i++) {
                    uartQueue[i] = &uartQueueBufs[i];
                }
                uartIn = uartOut = 0;
                uartBusy = FALSE;
                uartFull = FALSE;
                
                // sucessfull reset, move to next phase
                resetPhase++;
                call ResetTimer.startOneShot(RESET_TIME);
            }
        } else {
            failBlink();
            dropBlink();
            
            // stagnation, try again
            call ResetTimer.startOneShot(RESET_TIME);
        }
    }

    event void SerialControl.stopDone(error_t error) {
        if (error == SUCCESS) {
            sucBlink();
            
            // sucessfull reset, move to next phase
            resetPhase++;
            call ResetTimer.startOneShot(RESET_TIME);
        } else {
            failBlink();
            dropBlink();
            
            // stagnation, try again
            call ResetTimer.startOneShot(RESET_TIME);
        }
    }

    event void RadioControl.stopDone(error_t error) {
        if (error == SUCCESS) {
            sucBlink();
            
            // sucessfull reset, move to next phase
            resetPhase++;
            call ResetTimer.startOneShot(RESET_TIME);
        } else {
            call RadioControl.stop();
            failBlink();
            dropBlink();
            
            // stagnation, try again
            call ResetTimer.startOneShot(RESET_TIME);
        }
    }

    event message_t * RadioSnoop.receive[am_id_t id](message_t *msg, void *payload, uint8_t len) {
        return receive(msg, payload, len, id);
    }

    event message_t * RadioReceive.receive[am_id_t id](message_t *msg, void *payload, uint8_t len) {
        return receive(msg, payload, len, id);
    }

    // message received from radio here
    // decide whether to forward it to serial port
    message_t * receive(message_t *msg, void *payload, uint8_t len, am_id_t id) {
        message_t *ret = msg;
        
        // if in reset do nothing for now
        if (inReset==TRUE) return ret;
        
        if (!signal RadioIntercept.forward[id](msg, payload, len))
            return ret;

        atomic
        {
            // if serial queue is not full, we can put message to it
            if (!uartFull) {
                ret = uartQueue[uartIn];
                uartQueue[uartIn] = msg;

                uartIn = (uartIn + 1) % UART_QUEUE_LEN;

                if (uartIn == uartOut)
                    uartFull = TRUE;

                if (!uartBusy) {
                    // timer replaced
                    //post uartSendTask();
                    timedUartSendTask();
            
                    uartBusy = TRUE;
                } 
                else {
                    //dropBlink();
                }
            } else {
                // serial queue full
/*
                uartIn = uartOut = 0;
                uartFull = FALSE;
*/              
                failBlink();
                dropBlink();
                
                
                uartFull=TRUE;
                radioFull=TRUE;
                ++uartFailCounter;
                call UartTimer.stop();
                
                // reset with restarting
                //inReset=TRUE;
                //resetPhase=0; 
                call Reset.reset();
                //call ResetTimer.startOneShot(RESET_TIME);
            }
        }

        return ret;
    }

    void timedUartSendTask(){
        // do nothing if in middle of reset process
        if (inReset==TRUE) return;
        
        // timer replaced
        //post uartSendTask();
        if (call UartTimer.isRunning()==FALSE){
            call UartTimer.startOneShot(UART_TIME);
        }
    }

    // task, send data to serial
    // manage queue
    task void uartSendTask() {
        uint8_t len;
        am_id_t id;
        am_addr_t addr, src;
        error_t sendError;
        message_t* msg;
        
        //sucBlink();
        
        atomic
        if (uartIn == uartOut && !uartFull) {
            uartBusy = FALSE;
            return;
        }

        msg = uartQueue[uartOut];
        tmpLen = len = call RadioPacket.payloadLength(msg);
        id = call RadioAMPacket.type(msg);
        addr = call RadioAMPacket.destination(msg);
        src = call RadioAMPacket.source(msg);
        call UartAMPacket.setSource(msg, src);

        // try to send with SerialActiveMessageC component
        sendError = call UartSend.send[id](addr, uartQueue[uartOut], len);
        switch(sendError){
            case SUCCESS:
                //sucBlink();
                uartFailCounter=0;
                break;
                
            // was size of packet correct?
            case ESIZE:
                // size is problem, remove packet from queue to send
                ++uartFailCounter;
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
                ++uartFailCounter;
                failBlink();
                break;
        }
            
        // try to send everything again
        // (before this was only as reaction on FAIL, if causes problem, remove
        // it from global case and add only as reaction on FAIL
        timedUartSendTask();
    }

    // eventhandler, serial send done
    event void UartSend.sendDone[am_id_t id](message_t* msg, error_t error) {
        if (error != SUCCESS){
            failBlink();
            ++uartFailCounter;
        } else {
            uartFailCounter=0;
            sucBlink();
            
            // ph4r05 edit, set uartBusy to false
            uartBusy = FALSE;
            
            // CO AK TOTO NIE JE SPRAVA KTORA SA ODOSLALA? NEZAPLNI FRONTU?
            // 
            atomic
            if (msg == uartQueue[uartOut]) {
                if (++uartOut >= UART_QUEUE_LEN)
                    uartOut = 0;
                if (uartFull)
                    uartFull = FALSE;
            }
        }
        
        // timer replaced
        //post uartSendTask();
        timedUartSendTask();
    }

    // eventhandler, receive message on serial
    event message_t * UartReceive.receive[am_id_t id](message_t *msg, void *payload, uint8_t len) {
        message_t *ret = msg;
        bool reflectToken = FALSE;

        //sucBlink();
        
        // should I forward this message catched on serial to radio?
        if (!signal SerialIntercept.forward[id](msg, payload, len))
            return ret;

        atomic
        if (!radioFull) {
            reflectToken = TRUE;
            ret = radioQueue[radioIn];
            radioQueue[radioIn] = msg;
            if (++radioIn >= RADIO_QUEUE_LEN)
                radioIn = 0;
            if (radioIn == radioOut)
                radioFull = TRUE;

            if (!radioBusy) {
                // timer replaced
                //post radioSendTask();
                call RadioTimer.startOneShot(RADIO_TIME);
                radioBusy = TRUE;
            }
            
        } else {
            // dropBlink();
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
        am_addr_t addr;
        message_t* msg;

        atomic
        if (radioIn == radioOut && !radioFull) {
            radioBusy = FALSE;
            return;
        }

        msg = radioQueue[radioOut];
        len = call UartPacket.payloadLength(msg);
        addr = call UartAMPacket.destination(msg);
        id = call UartAMPacket.type(msg);
        
        if (call RadioSend.send[id](addr, msg, len) == SUCCESS){
            //sucRadioBlink();
        }
        else {
            failBlink();
            
            // timer replaced
            //post radioSendTask();
            call RadioTimer.startOneShot(RADIO_TIME);
        }
    }

    // ebenthandler, radio send done
    event void RadioSend.sendDone[am_id_t id](message_t* msg, error_t error) {
        if (error != SUCCESS){
            failBlink();
        }
        else {
            // blink on success transmission
            sucRadioBlink();
            
            atomic
            if (msg == radioQueue[radioOut]) {
                if (++radioOut >= RADIO_QUEUE_LEN)
                    radioOut = 0;
                if (radioFull)
                    radioFull = FALSE;

/*
                if (uartFull || radioFull){
                    call ResetTimer.startOneShot(TIME_TO_RESET);
                } else {
                    call ResetTimer.stop(); 
                }
*/

            }
        }

        // timer replaced
        //post radioSendTask();
        call RadioTimer.startOneShot(RADIO_TIME);
    }

    // decision function, should be current message catched on radio forwarded to serial?
    // is usually overriden in component using this interface
    default event bool RadioIntercept.forward[am_id_t amid](message_t* msg, void* payload, uint8_t len){
        return TRUE;
    }

    // shold be message cathed on serial forwarded to radio?
    default event bool SerialIntercept.forward[am_id_t amid](message_t* msg, void* payload, uint8_t len){
        return TRUE;
    }
}
