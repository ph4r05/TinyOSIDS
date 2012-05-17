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

#include "application.h"
#include <message.h>
#include <AM.h>

//Defining the preprocessor variable CC2420_NO_ACKNOWLEDGEMENTS will disable all forms of acknowledgments at compile time.
//Defining the preprocessor variable CC2420_HW_ACKNOWLEDGEMENTS will enable hardware acknowledgments and disable software acknowledgments.
//#define CC2420_NO_ACKNOWLEDGEMENTS 1

module RssiBaseC @safe() {
	uses { 		
  		interface AMSend as UartAMSend;
  		interface AMSend as UartNoiseAMSend;
  		interface Reset as Reset;
  		
	  /****************** COMMAND PROTOCOL ***************************/
	  interface Receive as CommandMsgReceiver;
	  interface AMSend as CommandSender;
	  interface Receive as UartCommandMsgReceiver;
	  interface AMSend as UartCommandSender;
	  
	  /****************** RSSI SAMPLING *****************************/
	  interface Receive as RSSIMsgReceiver;
	  interface Receive as MultiPingResponseMsgReceiver;
	  interface Receive as MultiPingMsgReceiver;
	  interface Receive as UartMultiPingMsgReceiver;
	  interface QueueSender as UartMultiPingResponseSender;
	
	}
  
  uses interface Timer<TMilli> as InitTimer;
  uses interface Timer<TMilli> as AliveTimer;
  uses interface Timer<TMilli> as SendTimer;
  uses interface Timer<TMilli> as PingTimer;
  
  uses interface AMSend as PingMsgSend;
  uses interface PacketAcknowledgements as Acks;
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

  // store RSSI measurements
  uses interface Queue<MultiPingResponseReportStruct_t> as RSSIQueue;

	/**************** RADIO DEPENDENT INTERFACES ****************/
#ifdef __CC2420_H__
  uses interface CC2420Packet;
  // uses interface CC2420PacketBody;
  // set channel
  uses interface CC2420Config;
#elif  defined(PLATFORM_IRIS)  
  uses interface PacketField<uint8_t> as PacketRSSI;
  uses interface PacketField<uint8_t> as PacketTransmitPower;
#elif defined(TDA5250_MESSAGE_H)
  uses interface Tda5250Packet;
#endif

	/**************** NOISE FLOOR READING ****************/
  	uses interface Timer<TMilli> as NoiseFloorTimer;
#ifdef __CC2420_H__
	// noise floor rssi reading
	uses interface Read<uint16_t> as ReadRssi;
#endif

} implementation {
	message_t pkt;
	
	/**************** COMMANDS ****************/
	message_t cmdPkt;
	message_t cmdPktResponse;
	uint16_t commandCounter=0;
	am_addr_t commandDest=0;
	bool commandRadio=FALSE;
	// base station address
  	am_addr_t baseid = 1;
 
 	bool cmdRadioBusy=FALSE;
	/**************** NOISE FLOOR READING ****************/
  	message_t noisePkt;
  	uint16_t noiseData=0;
  	uint16_t noiseCounter=0;
  	// if true then data in noiseData, noiseCounter can be sent to application
  	bool noiseFresh=FALSE;
  	// in the middle of sending?
  	bool noiseBusy=FALSE;
  	// how often perform noise floor reading? in ms
  	// if 0 then reading is disabled
  	uint16_t noiseTimerInterval;
  	
  	/**************** PINGS ****************/
  	nx_struct MultiPingMsg multiPingRequest;
  	uint16_t multiPingCurPackets=0;
  	bool multiPingWorking=FALSE;
  	bool multiPingBusy=FALSE;
  	message_t pingPkt;
  	uint8_t cur_channel;
  	
  	// basic node operation mode
  	// since some protocol are stateful we need to know what are we supposed to do
  	uint8_t operationMode=1;
  	
	/**************** GENERIC ****************/
	 bool radioOn=FALSE;  
     bool radioRealOn=FALSE;  
     uint16_t radioCn=0;
     uint16_t radioInitCn=0;
  
	bool busy = TRUE;
	bool serialBusy = TRUE;
  
	uint16_t counter = 0;
	uint16_t aliveCounter = 0;
	uint8_t blinkCnSend = 0;

	// forward declarations
	void sendBlink();
	uint16_t getRssi(message_t *msg);
  
    // reporting
	void task sendReport();
	void task sendAlive();
	
	/********************** FORWARD DECLARATIONS ********************/
	void setPower(message_t *, uint8_t);
	uint8_t getPower(message_t *);
  	void setChannel(uint8_t);
  	uint8_t getChannel();
  	error_t readNoiseFloor();
  	void task readNoideTask();
	void setAutoAck(bool enableAutoAck, bool hwAutoAck);
	void setAck(message_t *msg, bool status);
	void CommandReceived(message_t * msg, void * payload, uint8_t len);
	void task sendCommandACK();
	void task sendCommandRadio();
	void setNoiseInterval(uint16_t interval);
	void task sendMultipleEcho();
	void setAddressRecognitionEnabled(bool enabled);
	
	void RssiMsgReceived(message_t ONE * msg, void ONE * payload, uint8_t len);
	void SimpleRssiMsgReceived(message_t ONE * msg, void ONE * payload, uint8_t len);
	/********************** PING RESPONSE RECEIVED ********************/

	event message_t* MultiPingResponseMsgReceiver.receive(message_t* msg, void* payload, uint8_t len){
		RssiMsgReceived(msg, payload, len);

		// do not waste bandwidth with this useless message
		// important information is extracted here, for PC app
		// it has no meaning
		return msg;
	}
	
	void RssiMsgReceived(message_t ONE * msg, void ONE * payload, uint8_t len) {
		// store RSSI to local buffer
		MultiPingResponseReportStruct_t struct2save;
		MultiPingResponseMsg * btrpkt = (MultiPingResponseMsg * ) payload;
		
		// if full?
		if ((call RSSIQueue.maxSize() - call RSSIQueue.size())==0){
			// queue full -> forward it and exit...
			post sendReport();
			// do not waste bandwidth with this useless message
			// important information is extracted here, for PC app
			// it has no meaning
			return;
		}
		
		atomic {
			// send report
			struct2save.nodeid = call RadioAMPacket.source(msg);   
			struct2save.rssi = getRssi(msg);
			struct2save.nodecounter = btrpkt->counter;
			struct2save.len = len;
			struct2save.request = btrpkt->request;
			
			call RSSIQueue.enqueue(struct2save);
		}
		
		sendBlink();
		post sendReport();
	}

	event message_t* RSSIMsgReceiver.receive(message_t* msg, void* payload, uint8_t len) {
		SimpleRssiMsgReceived(msg, payload, len);
		
		// do not waste bandwidth with this useless message
		// important information is extracted here, for PC app
		// it has no meaning
		return msg;
	}
	
	// DEPRECATED
	void SimpleRssiMsgReceived(message_t ONE * msg, void ONE * payload, uint8_t len) {
		// store RSSI to local buffer
		MultiPingResponseReportStruct_t struct2save;
		RssiMsg * btrpkt = (RssiMsg * ) payload;
		
		// if full?
		if ((call RSSIQueue.maxSize() - call RSSIQueue.size())==0){
			// queue full -> forward it and exit...
			post sendReport();
			// do not waste bandwidth with this useless message
			// important information is extracted here, for PC app
			// it has no meaning			
			return;
		}
		
		atomic {
			// send report
			struct2save.nodeid = call RadioAMPacket.source(msg);   
			struct2save.rssi = getRssi(msg);
			struct2save.nodecounter = btrpkt->counter;
			struct2save.len = len;
			struct2save.request = 0;
			
			call RSSIQueue.enqueue(struct2save);
		}
		
		sendBlink();
		post sendReport();
	}
  
    /********************* COMMANDS RECEIVE  ************************/
      
	event message_t* CommandMsgReceiver.receive(message_t* msg, void* payload, uint8_t len){
		sendBlink();
		// process
		CommandReceived(msg, payload, len);
		return msg;
	}

	// Controls forwarding of messages received on serial interface.
	// If is packet destined for me only
	event message_t* UartCommandMsgReceiver.receive(message_t* msg, void* payload, uint8_t len){
		// process command here...
		sendBlink();
		
		// process
		CommandReceived(msg, payload, len);
		
		return msg;
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
		// prepare radio init 500ms after boot
		call InitTimer.startOneShot(500);
		
		// start serial comm
		call SerialControl.start();
	}
  
	void task startRadio() {
		call RadioControl.start();
	}

	void task stopRadio() {
		call RadioControl.stop();
	}

	event void InitTimer.fired() {
		radioOn = ! radioOn;
		radioInitCn += 1;

		if(radioOn) {
			post startRadio();
		}
		else {
			post stopRadio();
		}
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
		call UartMultiPingResponseSender.sendState(TRUE);
		call AliveTimer.startPeriodic(500);
	}
  
    /************************ SENDING ************************/
	event void SendTimer.fired() {
		post sendReport();
	}
  
  	// send RSSI report to application by serial / uart
  	// if there is any data in RSSI Queue, 1-3 of them are sent over serial
  	// to application - 
	void task sendReport() {
		if(call RSSIQueue.empty()) {
			// empty queue, nothing to do... OK
			return;
		}

		{
			// queue is full?
			if(call UartMultiPingResponseSender.full()==FALSE) {
				// dequeue from RSSI QUEUE, Build message, add to serial queue
				uint8_t i=0;
				uint8_t toSend=call RSSIQueue.size();
				MultiPingResponseReportMsg btrpktBuff;
				MultiPingResponseReportMsg * btrpkt = &btrpktBuff;
//				serialqueue_element_t tmpElement;
				
				toSend = toSend > MULTIPINGRESPONSEREPORT_MAXDATA ? MULTIPINGRESPONSEREPORT_MAXDATA : toSend;
				btrpkt->datanum = toSend;
					
				atomic for(i=0; i<toSend; i++){
					// get data, but leave in queue, removed is only if operation was succ
					MultiPingResponseReportStruct_t tmpStruct = call RSSIQueue.element(i);
					btrpkt->counter=counter++;
					btrpkt->nodecounter[i] = tmpStruct.nodecounter;
					btrpkt->nodeid[i] = tmpStruct.nodeid;
					btrpkt->rssi[i] = tmpStruct.rssi;
					btrpkt->len[i] = tmpStruct.len;
					btrpkt->request[i] = tmpStruct.request;
				}
	
				if (call UartMultiPingResponseSender.enqueueData(btrpkt, sizeof(MultiPingResponseReportMsg))==SUCCESS){
//				// use queue here to add messages
//				// build queue message
//				tmpElement.addr = AM_BROADCAST_ADDR;
//				tmpElement.isRadioMsg = FALSE;
//				tmpElement.msg = &pkt;
//				tmpElement.len = sizeof(MultiPingResponseReportMsg) + multiPingRequest.size;
//				tmpElement.id = AM_MULTIPINGRESPONSEREPORTMSG;
//				tmpElement.payload = btrpkt;
//				if (call UartQueue.enqueue(&tmpElement)==SUCCESS){
					// now really delete element from queue
					atomic for(i=0; i<toSend; i++){
						// remove elements from queue - added to send queue OK
						call RSSIQueue.dequeue();
					}
					
					post sendReport();
					return;
				} else {
					// message add failed, try again later
					post sendReport();
					return;
				}
			}
			else {
				// send queue is full, try again later
				post sendReport();
			}
		}
	}
  
  	event void UartAMSend.sendDone(message_t *msg, error_t error){
  		serialBusy = FALSE;
  		post sendReport();
	}
	
	// alive counter fired -> signalize that I am alive to application
	event void AliveTimer.fired(){
		// first 10 messages are sent quickly
		if (aliveCounter>10){
			call AliveTimer.startPeriodic(2000);
		}
		
		post sendAlive();
	}
	
	// sends alive packet to application to know that node is OK
	void task sendAlive(){
		CommandMsg * btrpkt = NULL;
		if (serialBusy) {
			dbg("Cannot send indentify message");
			post sendAlive();
			return;
		}
		
		atomic {
			btrpkt = (CommandMsg* ) (call UartCommandSender.getPayload(&cmdPkt, sizeof(CommandMsg)));
			
			// need to set source manualy here
			call UartAMPacket.setSource(&cmdPkt, TOS_NODE_ID);
			
			// only one report here, yet
			btrpkt->command_id = aliveCounter;
			btrpkt->reply_on_command = COMMAND_IDENTIFY;
			btrpkt->command_code = COMMAND_ACK;
			btrpkt->command_version = 1;
			btrpkt->command_data = TOS_NODE_ID;
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
	        
	        // congestion information
	        // first 8 bites = free slots in radio queue
	        // next 8 bites = free slots in serial queue
	        btrpkt->command_data_next[2] = call UartMultiPingResponseSender.size();
//	        btrpkt->command_data_next[2] |= (call UartNoiseFloorMsgSender.size() << 8);
	        btrpkt->command_data_next[3] = call RSSIQueue.size();
	        btrpkt->command_data_next[3] |= 0;
	        
			if(call UartCommandSender.send(TOS_NODE_ID, &cmdPkt, sizeof(CommandMsg)) == SUCCESS) {
				aliveCounter+=1;
				serialBusy = TRUE;
				sendBlink();
			}
			else {
				post sendAlive();
				dbg("Cannot send identify message");
			}
		}
	}
	
	event void UartCommandSender.sendDone(message_t *msg, error_t error){
		serialBusy=FALSE;
		atomic {
			// send alive messages, if false - increment problem counter
			if (msg==&cmdPkt && error!=SUCCESS){
					post sendAlive();
			}		
		}
	}

    /************************ COMMAND RECEIVED ************************/
	void CommandReceived(message_t * msg, void * payload, uint8_t len) {
		CommandMsg * btrpkt = NULL;
		CommandMsg * btrpktresponse = NULL;
		if(len != sizeof(CommandMsg)) {
			// invalid length - cannot process
			return;
		}		
		
		// get received message
		btrpkt = (CommandMsg * ) payload;

		// get local message, prepare it
		btrpktresponse = (CommandMsg * )(call UartCommandSender.getPayload(
				&cmdPktResponse, sizeof(CommandMsg)));

		// set reply ID by default
		btrpktresponse->reply_on_command = btrpkt->command_code;
		btrpktresponse->reply_on_command_id = btrpkt->command_id;
		btrpktresponse->command_code = COMMAND_ACK;
		commandDest = baseid;

		// decision based on type of command
		switch(btrpkt->command_code) {
			case COMMAND_IDENTIFY : // send my identification. Perform as task
				call AliveTimer.startOneShot(10);
			break;

			
			case COMMAND_RESET : // perform hard HW reset with watchdog to be sure that node is clean
				btrpktresponse->command_code = COMMAND_ACK;
				// should trigger HW restart - by watchdog freeze
				call AliveTimer.stop();
				call PingTimer.stop();
				
				// reset is too fast, application tries to send it 
				call Reset.reset();
				// code execution should not reach this statement
				post sendCommandACK();
			break;

			case COMMAND_ABORT : 
				//abort();
				btrpktresponse->command_code = COMMAND_ACK;
				//signalize(2);
				//post sendCommandACK();
			break;

			case COMMAND_SETTX : 
				//base_tx_power = btrpkt->command_data;
				btrpktresponse->command_code = COMMAND_ACK;
				//post sendCommandACK();
				//signalize(2);
			break;

			case COMMAND_SETBS : 
				//baseid = btrpkt->command_data;
				btrpktresponse->command_code = COMMAND_ACK;
				//post sendCommandACK();
				//signalize(2);
			break;


			// setting noise floor reading
			case COMMAND_SETNOISEFLOORREADING : 
				setNoiseInterval((uint16_t) btrpkt->command_data);
				call NoiseFloorTimer.startOneShot(noiseTimerInterval);
				btrpktresponse->command_code = COMMAND_ACK;
				//post sendCommandACK();
				//signalize(2);
			break;

			// pin settings
			// sets digital value on output pin
			case COMMAND_SETPIN : 
			break;

			// set request for sensor reading
			// request driven mode
			// can be specified to use timer-driven mode and its parameters
			// (delay)
			case COMMAND_GETSENSORREADING : // readMode = which sensor to read in this request
			break;

			// changing variable doSensorReadingSampling
			// If true then sensor readings from another nodes will be sampled for RSSI signal
			case COMMAND_SETSAMPLESENSORREADING : 
			break;

			// another sensor reading packet.
			// if is sampling enabled, do rssi sample of this packet and add to queue   
			case COMMAND_SENSORREADING : 
			break;
			
			// address sniffing?
			case COMMAND_RADIO_ADDRESS_RECOGNITION_ENABLED:
				{
				bool enabledRecognition = (btrpkt->command_data>0);
				setAddressRecognitionEnabled(enabledRecognition);
				
				btrpktresponse->command_code = COMMAND_ACK;
				post sendCommandACK();
				}
			break;
			
			// send response as fast as possible
			case COMMAND_PING:
				post sendCommandACK();
			break;
			
			// timesync request, send time global right now to serial
			case COMMAND_TIMESYNC_GETGLOBAL:
					
			break;
			
			// send timesync request to broadcast radio
			case COMMAND_TIMESYNC_GETGLOBAL_BCAST:
				commandDest = AM_BROADCAST_ADDR;
				btrpktresponse->command_code = COMMAND_TIMESYNC_GETGLOBAL;
				post sendCommandRadio();
			break;

			default: 
				;
		}

		return;
	}

	/**
	 * Send defined command to radio
	 */
	void task sendCommandRadio(){
		CommandMsg* btrpkt = NULL;
	  	 if (cmdRadioBusy){
	  	 	post sendCommandRadio();
	  	 	return;
	  	 }

		btrpkt=(CommandMsg*)(call CommandSender.getPayload(&cmdPktResponse, 0));

    		// setup message with data
		btrpkt->command_id = counter;

		// send to base directly
		// sometimes node refuses to send too large packet. it will always end with fail
		// depends of buffers size.
		if (call CommandSender.send(commandDest, &cmdPktResponse, sizeof(CommandMsg)) == SUCCESS) {
	 	    cmdRadioBusy=TRUE;
		}
		else {
		        dbg("Cannot send message");
			post sendCommandRadio();
		}	
	}
	
	event void CommandSender.sendDone(message_t *msg, error_t error){
		if (&cmdPktResponse==msg){
			cmdRadioBusy=FALSE;
			if (error!=SUCCESS){
				post sendCommandRadio();
			}
		}
	}

	/**
   	 * Send ACK command
     * packet is prepared before calling this
   	 */
  void task sendCommandACK(){
  	CommandMsg* btrpkt = NULL;
  	  if (serialBusy){
  	  	post sendCommandACK();
  	  	return;
  	  }
  	
    btrpkt=(CommandMsg*)(call UartCommandSender.getPayload(&cmdPktResponse, 0));

    // setup message with data
    btrpkt->command_id = counter;
    
    // need to set source
    call UartAMPacket.setSource(&cmdPktResponse, TOS_NODE_ID);

    // deprecated, allow to send any command
    // used for queueFlush command too
    //
    // ACK as reply, if not set already
    //if (btrpkt->command_code != COMMAND_ACK && btrpkt->command_code != COMMAND_NACK)
    //    btrpkt->command_code = COMMAND_ACK;

    // send to base directly
    // sometimes node refuses to send too large packet. it will always end with fail
    // depends of buffers size.
	if (call UartCommandSender.send(commandDest, &cmdPktResponse, sizeof(CommandMsg)) == SUCCESS) {
  	    busy = TRUE;
	}
	else {
        dbg("Cannot send message");
		post sendCommandACK();
	}
   
  }

/************************ NOISE FLOOR ************************/
	void task readNoideTask(){
		readNoiseFloor();
	}
	
	event void NoiseFloorTimer.fired(){
		// just trigger reading, answer will be returned in async event
		readNoiseFloor();
	}
	
	// sends prepared noise floor reading data
	void task sendNoiseReading(){
		NoiseFloorReadingMsg * btrpkt = NULL;
		// ise noise floor data valid?
		if (noiseFresh==FALSE){
			// nothing to do, noise data is not valid
			return;	
		}
		
		// if is busy try another time
		if (noiseBusy==TRUE){
			post sendNoiseReading();
			return;
		}
		
		// need to set source
    	call UartAMPacket.setSource(&noisePkt, TOS_NODE_ID);
		
		// construct new command packet with answer
		btrpkt = (NoiseFloorReadingMsg* ) (call UartNoiseAMSend.getPayload(&noisePkt, sizeof(NoiseFloorReadingMsg)));
		// only one report here, yet
		btrpkt->counter = noiseCounter;
		btrpkt->noise = noiseData;
		
		if(call UartNoiseAMSend.send(TOS_NODE_ID, &noisePkt, sizeof(NoiseFloorReadingMsg)) == SUCCESS) {
			noiseBusy=TRUE;
			sendBlink();
		}
		else {
			dbg("Cannot send identify message");
			post sendNoiseReading();	
		}
      }
	
	// sets noise reading interval
	// if 0 => disable noise floor reading timer
	void setNoiseInterval(uint16_t interval){
		noiseTimerInterval = interval;
		if (interval==0 && (call NoiseFloorTimer.isRunning())){
			// stop running timer
			call NoiseFloorTimer.stop();
		}
	}
	
	// noise report was sent, process answer
	event void UartNoiseAMSend.sendDone(message_t *msg, error_t error){
		noiseBusy=FALSE;
		if (error!=SUCCESS){
			// try to re-send
			post sendNoiseReading();
			return;
		}
		
		// send OK
		noiseFresh=FALSE;
		noiseCounter+=1;
		// start again if applicable
		if (noiseTimerInterval>0){
			call NoiseFloorTimer.startOneShot(noiseTimerInterval);
		}
	}	
  
/****************************** PINGING *******************************/ 
	event void PingTimer.fired(){
		// if packets = 0 send unlimited number of packets
		// sending can be stopped by receiving another request with non-null number
		// of requested packets to send
		if(multiPingRequest.packets == 0) {
			post sendMultipleEcho();
			return;
		}
		else {
			if(multiPingRequest.packets > multiPingCurPackets) {
				// still has sent less packets than expected - send
				post sendMultipleEcho();
			}
			else {
				// turn timer off iff every packet was already sent
				call PingTimer.stop();

				// reset coutner
				multiPingCurPackets = 0;
				// freeze next attempts
				multiPingRequest.delay = 0;
			}
		}
	}
	
	// ping send done
	event void PingMsgSend.sendDone(message_t * m, error_t error) {
		multiPingBusy = FALSE;
		
		// packet counter increment based on strategy chosen
		if (multiPingRequest.counterStrategySuccess){
			// increment only on succ
			if (error==SUCCESS){
				multiPingCurPackets+=1;
			}
		} else {
			// increment everytime
			multiPingCurPackets+=1;
		}
		
		// timer starting based on strategy chosen
		if (multiPingRequest.timerStrategyPeriodic==FALSE && multiPingRequest.delay>0){
			// here start only one shot timer, if this strategy is prefered
			call PingTimer.startOneShot(multiPingRequest.delay);
		}
		
	}
	
	/**
	 * Sends echo - timer triggered
	 */
	void task sendMultipleEcho(){
		MultiPingResponseMsg * btrpkt = NULL;
		if (multiPingBusy){
			post sendMultipleEcho();
			return;
		}
		
		// generate payload with specified size
    	btrpkt = (MultiPingResponseMsg*)(call Packet.getPayload(&pingPkt, sizeof(MultiPingResponseMsg) + multiPingRequest.size));

    	// set transmit power for each packet
    	setPower(&pingPkt, multiPingRequest.txpower);
    	
    	// disable ACK by default
    	setAck(&pingPkt, FALSE);

        // ping coutner
        btrpkt->counter = multiPingCurPackets;
        btrpkt->request = multiPingRequest.counter;

    	if (call PingMsgSend.send(multiPingRequest.destination, &pingPkt, sizeof(MultiPingResponseMsg) + multiPingRequest.size) == SUCCESS) {
      	    multiPingBusy = TRUE;
    	}
    	else {
    		// send failed, post to try again
    		post sendMultipleEcho();
    	}
	}
	
	// multi ping request received (radio or serial, does not matter)
	void MultiPingRequestReceived(message_t *msg, void *payload, uint8_t len){
		MultiPingMsg * btrpkt = NULL;
		if(len != sizeof(MultiPingMsg)) {
			// invalid length - cannot process
			return;
		}		
				
		// get received message
		btrpkt = (MultiPingMsg * ) payload;
		// size is constrained to TOSH_DATA_LENGTH
		if ((btrpkt->size + sizeof(MultiPingResponseMsg))> call RadioPacket.maxPayloadLength()){
			return;
		}
		
		// copy current request from packet to local var
		memcpy((void*)&multiPingRequest, payload, sizeof(MultiPingMsg));
		// set curr running to zero, stop timer 
		call PingTimer.stop();
		multiPingCurPackets = 0;
		// if delay=0 => stop ping send
		if(btrpkt->delay==0){
			call PingTimer.stop();
			return;
		}
	    
		// depending on timer strategy start timer...
		if (btrpkt->timerStrategyPeriodic){
			// periodic timer with defined delay
			call PingTimer.startPeriodic(btrpkt->delay);
		} else {
			// one-shot timer only
			call PingTimer.startOneShot(btrpkt->delay);
		}
	}
	
	// multi ping request from radio
	event message_t * UartMultiPingMsgReceiver.receive(message_t *msg, void *payload, uint8_t len){
		// process command here...
		sendBlink();
		
		// process
		MultiPingRequestReceived(msg, payload, len);
		return msg;
	}

	// multi ping request from serial
	event message_t * MultiPingMsgReceiver.receive(message_t *msg, void *payload, uint8_t len){
		// process command here...
		sendBlink();
		
		// process
		MultiPingRequestReceived(msg, payload, len);
		
		return msg;
	}
 
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

/************************ Channel/TX power settings ************************/
#ifdef __CC2420_H__
  void setPower(message_t *msg, uint8_t power){
  	if (power >= 1 && power <=31){
			call CC2420Packet.setPower(msg, power);
    }
  }
  
  // set channel
  event void CC2420Config.syncDone( error_t error ) {                                                                 
  }
  
  /**
   * Change the channel of the radio, between 11 and 26
   */
  void setChannel( uint8_t new_channel ){
  	if (cur_channel==0){
  		cur_channel = getChannel();
  	}
  	
  	if (cur_channel!=new_channel){
  		call CC2420Config.setChannel(new_channel);
        call CC2420Config.sync();
  		cur_channel = new_channel;
  	}	  
  }
  
  uint8_t getChannel(){
  	return call CC2420Config.getChannel();
  }

  void setAutoAck(bool enableAutoAck, bool hwAutoAck){
    call CC2420Config.setAutoAck(enableAutoAck, hwAutoAck);
    call CC2420Config.sync();
  }

  void setAddressRecognitionEnabled(bool enabled){
	call CC2420Config.setAddressRecognition(enabled, enabled);
	call CC2420Config.sync();
  }

  void setAck(message_t *msg, bool status){
  	if (status){
      	call Acks.requestAck(msg);
    } else {
    	call Acks.noAck(msg);
    }
  }

  /**
   * Split-phase command to read noise floor
   * Can meassure only when radio is not busy and there is some space on queue
   */
  error_t readNoiseFloor(){
      return call ReadRssi.read();
      
      //failBlink();
      //return FAIL;
  }

  // noise floor results
  event void ReadRssi.readDone(error_t error, uint16_t data) {
      if (error==SUCCESS){
      	noiseFresh = TRUE;
      	noiseData = data;
      	post sendNoiseReading();
	  } else {
	  	// noise floor reading failed, start again
	  	post readNoideTask();
	  }
  }
#elif defined(PLATFORM_IRIS)
  /**
   * Set transmit power
   */
  void setPower(message_t *msg, uint8_t power){
  	 cur_tx_power = power;
         call PacketTransmitPower.set(msg, power);
  }

  /**
   * Get transmit power
   */
  uint8_t getPower(message_t *msg){
      return call PacketTransmitPower.get(msg);
  }

  /**
   * Change the channel of the radio, between 11 and 26
   */
  void setChannel(uint8_t new_channel){
  	;
  }

  uint8_t getChannel(){
  	;
  }

  void setAutoAck(bool enableAutoAck, bool hwAutoAck){
       ;
  }

  void setAck(message_t *msg, bool status){
      ;
  }

  /**
   * Split-phase command to read noise floor
   * Can meassure only when radio is not busy and there is some space on queue
   */
  error_t readNoiseFloor(){
      if (busy==FALSE && radioFull==FALSE){
          //busy=TRUE;
          //return call ReadRssi.read();
      }

      failBlink();
      return FAIL;
  }

#else
#error Radio chip not supported! This demo currently works only \
         for motes with CC1000, CC2420, RF230 or TDA5250 radios.  
   	void setPower(message_t *, uint8_t){
   		
   	}
   	
	uint8_t getPower(message_t *){
		return 0;
	}
	
  	void setChannel(uint8_t){
  		
  	}
  	
  	uint8_t getChannel(){
  		return 0;
  	}
  	
  	error_t readNoiseFloor(){
  		return FAIL;
  	}
  	
	void setAutoAck(bool enableAutoAck, bool hwAutoAck){
		
	}
	
	void setAck(message_t *msg, bool status){
		
	}
	
	void setAddressRecognitionEnabled(bool enabled){
				
	}
#endif	
}
