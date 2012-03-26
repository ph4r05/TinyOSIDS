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
#include "Reset.h"
#include <message.h>
#include <AM.h>

//Defining the preprocessor variable CC2420_NO_ACKNOWLEDGEMENTS will disable all forms of acknowledgments at compile time.
//Defining the preprocessor variable CC2420_HW_ACKNOWLEDGEMENTS will enable hardware acknowledgments and disable software acknowledgments.
#define CC2420_NO_ACKNOWLEDGEMENTS 1
	
#ifdef __CC2420_H__

#elif defined(TDA5250_MESSAGE_H)
      
#else
  
#endif 

module RssiBaseC @safe() {
	uses {
		interface Intercept as RssiMsgIntercept;
  		interface Intercept as SimpleRssiMsgIntercept;
  		interface Intercept as CommandMsgIntercept;
  		interface Intercept as SerialCommandIntercept;
  		interface Intercept as MultiPingRadioIntercept;
  		interface Intercept as MultiPingSerialIntercept;
  		
  		interface AMSend as UartAMSend;
  		interface AMSend as UartCmdAMSend;
  		interface AMSend as UartNoiseAMSend;
  		interface Queue<serialqueue_element_t *> as UartQueue;
  		
  		interface InterceptBaseConfig;
  		
  		interface Reset as Reset;
  		
  		/*************** CTP ****************/
  		interface Random;
  		interface StdControl as RoutingControl;
        
        interface Send as CtpSend;
        interface Receive as CtpReceive;
        interface CollectionPacket;
        interface RootControl; 
        interface CtpInfo;
        interface CollectionDebug;
        interface Timer<TMilli> as CtpTimer;
        
        interface Intercept as CtpRequestSerialIntercept;
        
        // send report one by one
        interface AMSend as UartCtpReportDataAMSend;
        
        interface AMTap;
        interface AMTap as AMTapForg;
	
		// do not intercept CTP communication
		interface Intercept as CtpRoutingIntercept;
		interface Intercept as CtpDataIntercept;
		interface Intercept as CtpDebugIntercept;
	}

/*
  uses interface Intercept as Report;
*/
  uses interface Timer<TMilli> as AliveTimer;
  uses interface Timer<TMilli> as SendTimer;
  uses interface Timer<TMilli> as PingTimer;
  
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

  // store RSSI measurements
  uses interface Queue<MultiPingResponseReportStruct_t> as RSSIQueue;

	/**************** RADIO DEPENDENT INTERFACES ****************/
#ifdef __CC2420_H__
  uses interface CC2420Packet;
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
  	uint16_t multiPingCurPackets;
  	bool multiPingWorking=FALSE;
  	bool multiPingBusy=FALSE;
  	message_t pingPkt;
  	uint8_t cur_channel;
  	
  	// basic node operation mode
  	// since some protocol are stateful we need to know what are we supposed to do
  	uint8_t operationMode=NODE_REPORTING;
  	
  	/**************** CTP ****************/
  	nx_struct CtpSendRequestMsg ctpSendRequest;
  	uint16_t ctpCurPackets;
  	bool ctpBusy=FALSE;
  	bool ctpWorking=FALSE;
  	message_t ctpPkt;
  	message_t ctpReportPkt;
  	
  	// tx power for CTP data and route messages - CTP tree scaling
  	// default - set to maximum tx power
  	uint8_t ctpTxData=31;
  	uint8_t ctpTxRoute=31; 
  	
  	void task sendCtpMsg();
  	
	/**************** GENERIC ****************/
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
	void setNoiseInterval(uint16_t interval);
	void task sendMultipleEcho();
	void setAddressRecognitionEnabled(bool enabled);
	
	/********************** INTERCEPT HANDLERS ********************/
	// decision function, should be message cathed on radio forwarded to serial on BS?
	// we can change some fields in given message
	// todo: report against given message type ID
	event bool RssiMsgIntercept.forward(message_t * msg, void * payload, uint8_t len) {
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
			return FALSE;
		}
		
		atomic {
			// send report
			struct2save.nodeid = call RadioAMPacket.source(msg);   
			struct2save.rssi = getRssi(msg);
			struct2save.nodecounter = btrpkt->counter;
			struct2save.len = len;
			
			call RSSIQueue.enqueue(struct2save);
		}
		
		sendBlink();
		post sendReport();

		// do not waste bandwidth with this useless message
		// important information is extracted here, for PC app
		// it has no meaning
		return FALSE;
	}

	event bool SimpleRssiMsgIntercept.forward(message_t * msg, void * payload, uint8_t len) {
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
			return FALSE;
		}
		
		atomic {
			// send report
			struct2save.nodeid = call RadioAMPacket.source(msg);   
			struct2save.rssi = getRssi(msg);
			struct2save.nodecounter = btrpkt->counter;
			struct2save.len = len;
			
			call RSSIQueue.enqueue(struct2save);
		}
		
		sendBlink();
		post sendReport();

		// do not waste bandwidth with this useless message
		// important information is extracted here, for PC app
		// it has no meaning
		return FALSE;
	}
  
	event bool CommandMsgIntercept.forward(message_t *msg, void *payload, uint8_t len){
		// OK we can forward command messages to application, no problem ;-)
		am_addr_t destination = call UartAMPacket.destination(msg);
		bool forMe = call UartAMPacket.isForMe(msg);
		
		if (forMe){
			// process command here...
			sendBlink();
			
			// process
			CommandReceived(msg, payload, len);
			
			// do not forward private communication:)
			return (AM_BROADCAST_ADDR==destination);
		}
		
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
			
			// process
			CommandReceived(msg, payload, len);
			
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
		
		// start CTP's routing controll
		call RoutingControl.start();
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

		atomic {
			// queue is full?
			if(call UartQueue.maxSize() > call UartQueue.size()) {
				// dequeue from RSSI QUEUE, Build message, add to serial queue
				uint8_t i=0;
				uint8_t toSend=call RSSIQueue.size();
				MultiPingResponseReportMsg * btrpkt = (MultiPingResponseReportMsg* ) (call UartAMSend.getPayload(&pkt, sizeof(MultiPingResponseReportMsg) + multiPingRequest.size));
				serialqueue_element_t tmpElement;
				
				toSend = toSend > MULTIPINGRESPONSEREPORT_MAXDATA ? MULTIPINGRESPONSEREPORT_MAXDATA : toSend;
				btrpkt->datanum = toSend;
					
				for(i=0; i<toSend; i++){
					// get data, but leave in queue, removed is only if operation was succ
					MultiPingResponseReportStruct_t tmpStruct = call RSSIQueue.element(i);
					btrpkt->counter=counter++;
					btrpkt->nodecounter[i] = tmpStruct.nodecounter;
					btrpkt->nodeid[i] = tmpStruct.nodeid;
					btrpkt->rssi[i] = tmpStruct.rssi;
					btrpkt->len[i] = tmpStruct.len;
				}
	
				// use queue here to add messages
				// build queue message
				tmpElement.addr = AM_BROADCAST_ADDR;
				tmpElement.isRadioMsg = FALSE;
				tmpElement.msg = &pkt;
				tmpElement.len = sizeof(MultiPingResponseReportMsg) + multiPingRequest.size;
				tmpElement.id = AM_MULTIPINGRESPONSEREPORTMSG;
				tmpElement.payload = btrpkt;
				if (call UartQueue.enqueue(&tmpElement)==SUCCESS){
					// now really delete element from queue
					for(i=0; i<toSend; i++){
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
			btrpkt = (CommandMsg* ) (call UartCmdAMSend.getPayload(&cmdPkt, sizeof(CommandMsg)));
			// only one report here, yet
			btrpkt->command_id = aliveCounter;
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
	        
	        // congestion information
	        // first 8 bites = free slots in radio queue
	        // next 8 bites = free slots in serial queue
	        btrpkt->command_data_next[2] = call InterceptBaseConfig.getRadioQueueFree();
	        btrpkt->command_data_next[2] |= (call InterceptBaseConfig.getSerialQueueFree() << 8);
	        btrpkt->command_data_next[3] = call RSSIQueue.size();
	        btrpkt->command_data_next[3] |= (call InterceptBaseConfig.getSerialFailed()<<8);
	        
			if(call UartCmdAMSend.send(TOS_NODE_ID, &cmdPkt, sizeof(CommandMsg)) == SUCCESS) {
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
	
	event void UartCmdAMSend.sendDone(message_t *msg, error_t error){
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
		btrpktresponse = (CommandMsg * )(call UartCmdAMSend.getPayload(
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

			case COMMAND_GETREPORTINGSTATUS : 
				btrpktresponse->command_code = COMMAND_ACK;
				//btrpktresponse->command_data = (nx_uint16_t) doReporting;
				//post sendCommandACK();
				//signalize(2);
			break;

			case COMMAND_SETREPORTINGSTATUS : 
				//doReporting = (bool) btrpkt->command_data;
				//btrpktresponse->command_code = COMMAND_ACK;
				//post sendCommandACK();
				//signalize(2);
			break;

			case COMMAND_SETOPERATIONMODE : 
				//operationMode = btrpkt->command_data;
				btrpktresponse->command_code = COMMAND_ACK;
				//post sendCommandACK();
				//signalize(2);
			break;

			case COMMAND_SETREPORTPROTOCOL : 
				//reportProtocol = btrpkt->command_data;
				btrpktresponse->command_code = COMMAND_ACK;
				//post sendCommandACK();
				//signalize(2);
			break;

			// flush report queue command
			case COMMAND_FLUSHREPORTQUEUE : // if we are dead node, do not listed
				//
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
//				if(setGIO((uint8_t) btrpkt->command_data_next[0],
//					btrpkt->command_data == 1 ? TRUE : FALSE)) {
//				btrpktresponse->command_code = COMMAND_ACK;
//				post sendCommandACK();
//				signalize(2);
//			}
			break;

			// set request for sensor reading
			// request driven mode
			// can be specified to use timer-driven mode and its parameters
			// (delay)
			case COMMAND_GETSENSORREADING : // readMode = which sensor to read in this request
//				readMode = (uint8_t) btrpkt->command_data;
//
//				// timer-driven mode request?
//				// if in additional data[0] is 1 then modify current settings
//				// for timer-driven sensor reading
//				// else do not touch that timer.
//				if(btrpkt->command_data_next[0] == 1) {
//					signalize(3);
//					// timeout is defined in command_data_next[1]
//					// if timeout is 0 then stop current timer and do not start new
//					if(btrpkt->command_data_next[1] == 0) {
//						// stop only if running
//						if(call SensorReadingTimer.isRunning()) {
//							call SensorReadingTimer.stop();
//						}
//					}
//					else {
//						// start periodic timer with defined timeout (in ms)
//						call SensorReadingTimer.startPeriodic(btrpkt->command_data_next[1]);
//					}	
//
//					// flags here
//					// sensor reading destination is broadcast?
//					if((btrpkt->command_data_next[2]&1) > 0) {
//						// broadcast destination (probably rssi-sampled on anchor nodes)
//						sensorReadingDest = 65535U;
//					}
//					else {
//						// no broadcast destination, answer -> base station
//						sensorReadingDest = command_dest;
//					}
//				}
//	
//				// read appropriate sensors
//				readSensors();
//	
//				// signalize command received as ussual
//				//signalize(2);
			break;

			// changing variable doSensorReadingSampling
			// If true then sensor readings from another nodes will be sampled for RSSI signal
			case COMMAND_SETSAMPLESENSORREADING : 
//				doSensorReadingSampling = btrpkt->command_data > 0 ? TRUE : FALSE;
//				btrpktresponse->command_code = COMMAND_ACK;
//				post sendCommandACK();
//				signalize(2);
			break;

			// another sensor reading packet.
			// if is sampling enabled, do rssi sample of this packet and add to queue   
			case COMMAND_SENSORREADING : 
//				if(doSensorReadingSampling) {
//					// do sample this message for rssi, is it possible now?
//					message2sampleReceived(getRssi(msg), btrpkt->command_id, call AMPacket.source(msg));
//				}
//				else {
//					// else ignore this packet, not interested in reading sensors 
//					// of foreign nodes
//					;
//				}
			break;
			
			// disable radio forward
			case COMMAND_FORWARDING_RADIO_ENABLED:
				if (btrpkt->command_data>0){
					call InterceptBaseConfig.setGlobalRadioFilteringEnabled(FALSE);
				} else {
					call InterceptBaseConfig.setGlobalRadioFilteringEnabled(TRUE);
				}
				
				btrpktresponse->command_code = COMMAND_ACK;
				post sendCommandACK();
			break;
			
			// disable serial forward
			case COMMAND_FORWARDING_SERIAL_ENABLED: 
				if (btrpkt->command_data>0){
					call InterceptBaseConfig.setGlobalSerialFilteringEnabled(FALSE);
				} else {
					call InterceptBaseConfig.setGlobalSerialFilteringEnabled(TRUE);
				}
				
				btrpktresponse->command_code = COMMAND_ACK;
				post sendCommandACK();
			break;
			
			// disable radio forward
			case COMMAND_DEFAULT_FORWARDING_RADIO_ENABLED:
				if (btrpkt->command_data>0){
					call InterceptBaseConfig.setDefaultRadioFilteringEnabled(FALSE);
				} else {
					call InterceptBaseConfig.setDefaultRadioFilteringEnabled(TRUE);
				}
				
				btrpktresponse->command_code = COMMAND_ACK;
				post sendCommandACK();
			break;
			
			// disable serial forward
			case COMMAND_DEFAULT_FORWARDING_SERIAL_ENABLED: 
				if (btrpkt->command_data>0){
					call InterceptBaseConfig.setDefaultSerialFilteringEnabled(FALSE);
				} else {
					call InterceptBaseConfig.setDefaultSerialFilteringEnabled(TRUE);
				}
				
				btrpktresponse->command_code = COMMAND_ACK;
				post sendCommandACK();
			break;
			
			// forward snooped messages on radio?
			case COMMAND_RADIO_SNOOPING_ENABLED: 
				if (btrpkt->command_data>0){
					call InterceptBaseConfig.setRadioSnoopEnabled(TRUE);
				} else {
					call InterceptBaseConfig.setRadioSnoopEnabled(FALSE);
				}
				
				btrpktresponse->command_code = COMMAND_ACK;
				post sendCommandACK();
			break;
			
			// address sniffing?
			case COMMAND_RADIO_ADDRESS_RECOGNITION_ENABLED:
				{
				bool enabledRecognition = (btrpkt->command_data>0);
				call InterceptBaseConfig.setAddressRecognitionEnabled(enabledRecognition);
				setAddressRecognitionEnabled(enabledRecognition);
				
				btrpktresponse->command_code = COMMAND_ACK;
				post sendCommandACK();
				}
			break;
			
			// set CTP is root?
			case COMMAND_SET_CTP_ROOT:
				if (btrpkt->command_data>0){
					btrpktresponse->command_data = call RootControl.setRoot();
				} else if (call RootControl.isRoot()){
					btrpktresponse->command_data = call RootControl.unsetRoot();
				}

				btrpktresponse->command_code = COMMAND_ACK;
				post sendCommandACK();
			break;
			
			// call CTP route recomputing command - depending on data, CtpInfo interface is used
			// data=1 -> CtpInfo.triggerRouteUpdate()
			// data=2 -> CtpInfo.triggerImmediateRouteUpdate()
			// data=3 -> CtpInfo.recomputeRoutes()
			case COMMAND_CTP_ROUTE_UPDATE:
				if (btrpkt->command_data==1){
					call CtpInfo.triggerRouteUpdate();
					btrpktresponse->command_data = 1;
				} else if (btrpkt->command_data==2){
					call CtpInfo.triggerImmediateRouteUpdate();
					btrpktresponse->command_data = 2;
				} else if (btrpkt->command_data==3){
					call CtpInfo.recomputeRoutes();
					btrpktresponse->command_data = 3;
				}
		
				btrpktresponse->command_code = COMMAND_ACK;
				post sendCommandACK();
			break;
			
			// gets basic CTP info from CtpInfo interface
			// data=0 -> returns parent, etx, neighbors count in data[0], data[1], data[2]
			// data=1 -> info about neighbor specified in data[0]. Returned addr, link quality, route
    		//				quality, congested bit
			case COMMAND_CTP_GETINFO:
				if(btrpkt->command_data==0){
					// provide basic information about my CTP perspective
					am_addr_t parent=0;
					uint16_t etx=0;
					
					call CtpInfo.getParent(&parent);
					call CtpInfo.getEtx(&etx);
					
					btrpktresponse->command_data_next[0] = parent;
					btrpktresponse->command_data_next[1] = etx;
					btrpktresponse->command_data_next[2] = call CtpInfo.numNeighbors();
					btrpktresponse->command_code = COMMAND_ACK;
					post sendCommandACK();
				} else if (btrpkt->command_data==1){
					// provide information about particular neighbor
					uint8_t neighNum = (uint8_t) btrpkt->command_data_next[0];
					
					btrpktresponse->command_data_next[0] = call CtpInfo.getNeighborAddr(neighNum);
					btrpktresponse->command_data_next[1] = call CtpInfo.getNeighborLinkQuality(neighNum);
					btrpktresponse->command_data_next[2] = call CtpInfo.getNeighborRouteQuality(neighNum);
					btrpktresponse->command_data_next[3] = call CtpInfo.isNeighborCongested(btrpktresponse->command_data_next[0]);
					btrpktresponse->command_code = COMMAND_ACK;
					post sendCommandACK();
				}
			break;
			
			// other CTP controling, can set TX power for packets
			// data=0 -> set tx power for OUTPUT messages for CTP protocol.
			// 				if data[0] == 1	-> set TXpower for ROUTE messages on data[1] level
			//			 	if data[0] == 2 -> set TXpower for DATA messages on data[1] level
			//				if data[0] == 3 -> set TXpower for both ROUTE, DATA messages on data[1] level
			case COMMAND_CTP_CONTROL:
				if (btrpkt->command_data==0){
					// set tx power for ROUTE messages
					if (btrpkt->command_data_next[0] & 1){
						ctpTxRoute = (uint8_t) btrpkt->command_data_next[1];
					}
					
					// set tx power for DATA messages
					if (btrpkt->command_data_next[0] & 2){
						ctpTxData = (uint8_t) btrpkt->command_data_next[1];
					}
					
					btrpktresponse->command_code = COMMAND_ACK;
					post sendCommandACK();
				}
			break;

			default: 
				;
		}

		return;
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
  	
    btrpkt=(CommandMsg*)(call UartCmdAMSend.getPayload(&cmdPktResponse, 0));

    // setup message with data
    btrpkt->command_id = counter;

    // deprecated, allow to send any command
    // used for queueFlush command too
    //
    // ACK as reply, if not set already
    //if (btrpkt->command_code != COMMAND_ACK && btrpkt->command_code != COMMAND_NACK)
    //    btrpkt->command_code = COMMAND_ACK;

    // send to base directly
    // sometimes node refuses to send too large packet. it will always end with fail
    // depends of buffers size.
	if (call UartCmdAMSend.send(commandDest, &cmdPktResponse, sizeof(CommandMsg)) == SUCCESS) {
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

        // ping coutner
        btrpkt->counter = multiPingCurPackets;

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
		if ((btrpkt->size + sizeof(MultiPingResponseMsg))>TOSH_DATA_LENGTH){
			return;
		}
		
		// copy current request from packet to local var
		memcpy(&multiPingRequest, payload, sizeof(MultiPingMsg));
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
	event bool MultiPingRadioIntercept.forward(message_t *msg, void *payload, uint8_t len){
		am_addr_t destination = call RadioAMPacket.destination(msg);
		bool forMe = call RadioAMPacket.isForMe(msg);
		
		if (forMe){
			// process command here...
			sendBlink();
			
			// process
			MultiPingRequestReceived(msg, payload, len);
			
			// do not forward private communication:)
			return (AM_BROADCAST_ADDR==destination);
		}
		
		return TRUE;
	}

	// multi ping request from serial
	event bool MultiPingSerialIntercept.forward(message_t *msg, void *payload, uint8_t len){
		am_addr_t destination = call RadioAMPacket.destination(msg);
		bool forMe = call RadioAMPacket.isForMe(msg);
		
		if (forMe){
			// process command here...
			sendBlink();
			
			// process
			MultiPingRequestReceived(msg, payload, len);
			
			// do not forward private communication:)
			return (AM_BROADCAST_ADDR==destination);
		}
		
		return TRUE;
	}
  
  
  	event void InterceptBaseConfig.syncDone(error_t error){
		// TODO Auto-generated method stub
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
  		multiPingRequest.txpower = power;
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
      ;
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

/****************************** CTP ***********************************/
	/**
	 * Task for sending CTP message, triggered by timer, send random data
	 */
	void task sendCtpMsg(){
		CtpResponseMsg * btrpkt = NULL;
		uint16_t metric;
        am_addr_t parent;
		

		if (ctpBusy){
			post sendCtpMsg();
			return;
		}
		
		btrpkt = (CtpResponseMsg*)(call CtpSend.getPayload(&ctpPkt, sizeof(CtpResponseMsg)));

        call CtpInfo.getParent(&parent);
        call CtpInfo.getEtx(&metric);

        btrpkt->origin = TOS_NODE_ID;
        btrpkt->seqno = ctpCurPackets;
        btrpkt->data = call Random.rand16();
        btrpkt->parent = parent;
        btrpkt->hopcount = 0;
        btrpkt->metric = metric;
        
        if (call CtpSend.send(&ctpPkt, sizeof(CtpResponseMsg)) == SUCCESS) {
            ctpBusy=TRUE;
            dbg("IDS-app","App: sent packet with seqno %d to parent %d", msg->seqno, msg->parent);
        } else {
        	post sendCtpMsg();
        }
	}

	/**
	 * CTP message was sent. Start timer again according to properties set
	 */
	event void CtpSend.sendDone(message_t *msg, error_t error){
		if (&ctpPkt == msg) {
            ctpBusy = FALSE;
            // packet counter increment based on strategy chosen
			if (ctpSendRequest.counterStrategySuccess){
				// increment only on succ
				if (error==SUCCESS){
					ctpCurPackets+=1;
				}
			} else {
				// increment everytime
				ctpCurPackets+=1;
			}
			
			// timer starting based on strategy chosen
			if (ctpSendRequest.timerStrategyPeriodic==FALSE && ctpSendRequest.delay>0){
				// here start only one shot timer, if this strategy is prefered
				call CtpTimer.startOneShot(ctpSendRequest.delay);
			}
        }
	}

	/**
	 * CTP timer event, when fired, need to send new CtpReading 
	 */
	event void CtpTimer.fired(){
		// if packets = 0 send unlimited number of packets
		// sending can be stopped by receiving another request with non-null number
		// of requested packets to send
		if(ctpSendRequest.packets == 0) {
			post sendCtpMsg();
			return;
		}
		else {
			if(ctpSendRequest.packets > ctpCurPackets) {
				// still has sent less packets than expected - send
				post sendCtpMsg();
			}
			else {
				// turn timer off iff every packet was already sent
				call CtpTimer.stop();

				// reset coutner
				ctpCurPackets = 0;
				// freeze next attempts
				ctpSendRequest.delay = 0;
			}
		}
	}

	/**
	 * if CtpRequest to send is intercepted on serial, then new sending is started
	 */
	event bool CtpRequestSerialIntercept.forward(message_t *msg, void *payload, uint8_t len){
		// if message is destined for me, process it, get destination
		am_addr_t destination = call UartAMPacket.destination(msg);
		bool forMe = call UartAMPacket.isForMe(msg);
		
		if (forMe){
			// message is for me, thus need to start sending CtpMessages
			CtpSendRequestMsg * btrpkt = NULL;
			if(len != sizeof(CtpSendRequestMsg)) {
				// invalid length - cannot process
				return FALSE;
			}		
			
			// get received message
			btrpkt = (CtpSendRequestMsg * ) payload;			
			// copy current request from packet to local var
			memcpy(&ctpSendRequest, payload, sizeof(CtpSendRequestMsg));
			// set curr running to zero, stop timer 
			call CtpTimer.stop();
			ctpCurPackets = 0;
			// if delay=0 => stop ping send
			if(btrpkt->delay==0){
				call CtpTimer.stop();
				return FALSE;
			}
			
			// depending on timer strategy start timer...
			if (btrpkt->timerStrategyPeriodic){
				// periodic timer with defined delay
				call CtpTimer.startPeriodic(btrpkt->delay);
			} else {
				// one-shot timer only
				call CtpTimer.startOneShot(btrpkt->delay);
			}
		}
			
		return (AM_BROADCAST_ADDR==destination);
	}

	event void UartCtpReportDataAMSend.sendDone(message_t *msg, error_t error){
		// TODO Auto-generated method stub
	}
	
	/**
	 * CTP message received, dump to serial...
	 */
 	event message_t * CtpReceive.receive(message_t *msg, void *payload, uint8_t len){
		// size incorrect
		if (sizeof(CtpResponseMsg)!=len){
			return msg;
		}
		
		atomic {
			// queue is full?
			if(call UartQueue.maxSize() > call UartQueue.size()) {
				// dequeue from RSSI QUEUE, Build message, add to serial queue
				serialqueue_element_t tmpElement;
				CtpReportDataMsg * btrpkt = (CtpReportDataMsg* ) (call UartAMSend.getPayload(&ctpReportPkt, sizeof(CtpReportDataMsg)));
				
				// spoof = false, ctp original=1
				btrpkt->flags = 0x2;
				btrpkt->amSource = call AMPacket.source(msg);
				memcpy(&(btrpkt->response), payload, len);
				memset(&(btrpkt->ctpDataHeader), 0, sizeof(btrpkt->ctpDataHeader));						
	
				// use queue here to add messages
				// build queue message
				tmpElement.addr = TOS_NODE_ID;
				tmpElement.isRadioMsg = FALSE;
				tmpElement.msg = &ctpReportPkt;
				tmpElement.len = sizeof(CtpReportDataMsg);
				tmpElement.id = AM_CTPREPORTDATAMSG;
				tmpElement.payload = btrpkt;
				if (call UartQueue.enqueue(&tmpElement)==SUCCESS){
					return msg;
				} else {
					// message add failed, try again later
					return msg;
				}
			}
		}
		
		return msg;
	} 
	
	/**
	 * CTP packet received in RAW form from AMTap interface
	 */
	void ctpReceived(uint8_t type, message_t *msg, void *payload, uint8_t len, bool spoof){
		//CTP spoof is interesting for me
		CtpResponseMsg * response = NULL;
		ctp_data_header_t  * ctpDataHeader = NULL;
		
		// interested only in my collection id
		ctpDataHeader = (ctp_data_header_t  *) payload;
		if (ctpDataHeader->type!=AM_CTPRESPONSEMSG){
			return;
		}
		
		// get correct payload
		response = (CtpResponseMsg *) (payload + sizeof(ctp_data_header_t));
		
		atomic {
			// queue is full?
			if(call UartQueue.maxSize() > call UartQueue.size()) {
				// dequeue from RSSI QUEUE, Build message, add to serial queue
				CtpReportDataMsg * btrpkt = (CtpReportDataMsg* ) (call UartAMSend.getPayload(&ctpReportPkt, sizeof(CtpReportDataMsg)));
				serialqueue_element_t tmpElement;		
				
				btrpkt->flags = 0x0;
				btrpkt->flags |= spoof ? 0x1:0x0;
				btrpkt->amSource = call AMPacket.source(msg);
				memcpy(&(btrpkt->ctpDataHeader), ctpDataHeader, sizeof(ctp_data_header_t));
				memcpy(&(btrpkt->response), response, sizeof(CtpResponseMsg));		
	
				// use queue here to add messages
				// build queue message
				tmpElement.addr = TOS_NODE_ID;
				tmpElement.isRadioMsg = FALSE;
				tmpElement.msg = &ctpReportPkt;
				tmpElement.len = sizeof(CtpReportDataMsg);
				tmpElement.id = AM_CTPREPORTDATAMSG;
				tmpElement.payload = btrpkt;
				if (call UartQueue.enqueue(&tmpElement)==SUCCESS){
					return;
				} else {
					// message add failed, try again later
					return;
				}
			}
		}
	}

	/**
	 * Raw MSG Snooping, interested in CTP data messages, advantage = has complete ctp data
	 */
	event message_t * AMTap.snoop(uint8_t type, message_t *msg, void *payload, uint8_t len){
		//CTP spoof is interesting for me
		if (type!=AM_CTP_DATA){
			return msg;
		}
		
		ctpReceived(type, msg, payload, len, TRUE);
		return msg;
	}

	/**
	 * Raw MSG reception, interested in CTP data messages, advantage = has complete ctp data
	 */
	event message_t * AMTap.receive(uint8_t type, message_t *msg, void *payload, uint8_t len){
		//CTP spoof is interesting for me
		if (type!=AM_CTP_DATA){
			return msg;
		}
		
		ctpReceived(type, msg, payload, len, FALSE);
		return msg;
	}

	/**
	 * Raw message sending - disable for now, nothing interesting yet
	 * TODO: write basestation implementation for this correctly, now 
	 * basestation does not support this send interface
	 */
	event message_t * AMTap.send(uint8_t type, message_t *msg, uint8_t len){
		
		return msg;
	}

	/**
	 * Do not forward CTP raw data, avoid UART congestion
	 */
	event bool CtpRoutingIntercept.forward(message_t *msg, void *payload, uint8_t len){
		return FALSE;
	}

	/**
	 * Do not forward CTP raw data, avoid UART congestion
	 */
	event bool CtpDebugIntercept.forward(message_t *msg, void *payload, uint8_t len){
		return FALSE;
	}

	/**
	 * Do not forward CTP raw data, avoid UART congestion
	 */
	event bool CtpDataIntercept.forward(message_t *msg, void *payload, uint8_t len){
		return FALSE;
	}

	/**
	 * Nothing to do here, tapping is performed at basestation tap interface, not here
	 * Due to basestation nature (direct wiring to ActiveMessageC) ForgedMessage cannot 
	 * perform wiring to this as well in order to avoid fan-out warnings and problems
	 */
	event message_t * AMTapForg.receive(uint8_t type, message_t *msg, void *payload, uint8_t len){
		return msg;
	}
	
	/**
	 * Nothing to do here, tapping is performed at baasestation tap interface, not here
	 * Due to basestation nature (direct wiring to ActiveMessageC) ForgedMessage cannot 
	 * perform wiring to this as well in order to avoid fan-out warnings and problems
	 */
	event message_t * AMTapForg.snoop(uint8_t type, message_t *msg, void *payload, uint8_t len){
		return msg;
	}

	/**
	 * ForgedMessage component now provides only this send - here we can directly regulate output
	 * TXpower for some messages, especially for CTP messages.
	 * Data CTP message could be set before sending to CTP, but ROUTE messages are
	 * generated inside CTP module unreachable from outside, thus it is unable to modify TXpower for
	 * ROUTE messages directly. By setting txpower for both ROUTE and DATA CTP tree should scale 
	 */
	event message_t * AMTapForg.send(uint8_t type, message_t *msg, uint8_t len){
		//CTP messages are interesting for me
		if (type!=AM_CTP_DATA){
			return msg;
		}
		
		// CTP ROUTE message sent here, set wanted tx power
		// maximum power is default, thus ignore maximum power level
		if (type==AM_CTP_DATA && ctpTxData<31){
			setPower(msg, ctpTxData);
		}
		
		// CTP ROUTE message sent here, set wanted tx power
		// maximum power is default, thus ignore maximum power level
		if (type==AM_CTP_ROUTING && ctpTxData<31){
			setPower(msg, ctpTxRoute);
		}
		
		return msg; 
	}


}