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
#include "Reset.h"
#include <message.h>
#include <AM.h>

//#define REPORT_NEW_DELAY

//Defining the preprocessor variable CC2420_NO_ACKNOWLEDGEMENTS will disable all forms of acknowledgments at compile time.
//Defining the preprocessor variable CC2420_HW_ACKNOWLEDGEMENTS will enable hardware acknowledgments and disable software acknowledgments.
//#define CC2420_NO_ACKNOWLEDGEMENTS 1

module IDSCollectP @safe() {
	uses {  		
  		/****************** COMMAND PROTOCOL ***************************/
	    interface Receive as CommandMsgReceiver;
	    interface AMSend as CommandSender;
	    interface Receive as UartCommandMsgReceiver;
	    interface AMSend as UartCommandSender;
  		
  		/****************** RESET *************************************/
  		
  		interface Reset as Reset;
  		
  		/*************** CTP ****************/
  		interface Random;
  		interface StdControl as ForwardingControl;
  		interface StdControl as CtpLoggerControl;
  		interface Init as RoutingInit;
  		interface Init as ForwardingInit;
  		interface Init as LinkEstimatorInit;
        
        interface Send as CtpSend;
        interface Receive as CtpReceive;
        interface CollectionPacket;
        interface RootControl; 
        interface CtpInfo;
        interface CollectionDebug;
        interface Timer<TMilli> as CtpTimer;
        interface Timer<TMilli> as TreeTimer;
        
        interface Receive as UartCtpSendRequestReceiver;
        
        interface AMTap as AMTapForg;
		
		interface CollectionDebug as CtpLogger;
		interface ForwardControl;
		
		interface QueueSender as UartCtpReportDataSender;
		interface QueueSender as UartCtpInfoMsgSender;
		
	}

  uses interface Timer<TMilli> as InitTimer;
  uses interface Timer<TMilli> as AliveTimer;
  
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
  
  uses interface LocalTime<T32khz> as LocalTime32khz;
  uses interface LocalTime<TMilli> as LocalTimeMilli;

	/**************** RADIO DEPENDENT INTERFACES ****************/

	  
	 
#ifdef __CC2420_H__
  uses interface CC2420Packet;
  uses interface CC2420PacketBody;
  uses interface PacketTimeStamp<T32khz, uint32_t> as PacketTimeStamp32khz;
  uses interface PacketTimeStamp<TMilli, uint32_t> as PacketTimeStampMilli;

  // set channel
  uses interface CC2420Config;
#ifdef CC2420_HW_SECURITY 
	uses interface CC2420SecurityMode as CC2420Security;
	uses interface CC2420Keys;
#endif

#elif  defined(PLATFORM_IRIS)  
  uses interface PacketField<uint8_t> as PacketRSSI;
  uses interface PacketField<uint8_t> as PacketTransmitPower;
#elif defined(TDA5250_MESSAGE_H)
  uses interface Tda5250Packet;
#endif
} implementation {
	message_t pkt;
	uint8_t cur_channel;
	
	/**************** COMMANDS ****************/
	message_t cmdPkt;
	message_t cmdPktResponse;
	uint16_t commandCounter=0;
	am_addr_t commandDest=0;
	bool commandRadio=FALSE;
	// base station address
  	am_addr_t baseid = 1;
 
 	bool cmdRadioBusy=FALSE;  	
  	
  	/**************** CTP ****************/
  	nx_struct CtpSendRequestMsg ctpSendRequest;
  	uint16_t ctpCurPackets=0;
  	uint16_t ctpBusyCount=0;
  	bool ctpBusy=FALSE;
  	bool ctpWorking=FALSE;
  	message_t ctpPkt;
  	message_t ctpReportPkt;
  	message_t ctpInfoPkt;
  	
  	// tx power for CTP data and route messages - CTP tree scaling
  	// default - set to maximum tx power
  	uint8_t ctpTxData=31;
  	uint8_t ctpTxRoute=31; 
  	
  	enum {
  		CTP_RETX_TIME=10,
  		DBG_CTP_SEND_FAIL=0x60,
  		DBG_CTP_SEND_RETRY_FAIL=0x61,
  		DBG_CTP_FINISHED=0x62,
  		DBG_CTP_NEWDELAY=0x63,
  	};
  	
  	// Config structure
  	// This configuration structure can be stored to flash memory
  	// in order to be configurable at runtime and to survive node
  	// restart/power-cycle. 
  	// Now it is hardwired in boot, but can be easily modiffied
  	// to be configurable at runtime with config commands. 
  	// Each config command should change this config structure,
  	// it would be then saved to flash memory like in http://docs.tinyos.net/tinywiki/index.php/Storage
  	typedef struct config_t {
  		uint8_t ctpTxData;
  		uint8_t ctpTxRoute;
  		// send request wired to configuration structure
  		nx_struct CtpSendRequestMsg ctpSendRequest;
  		// if YES then after boot is launched CTP send according to ctpSendRequest
  		bool sendingCTP;
  		// static CTP root address, only 1 node can be root here
  		uint16_t rootAddress;
  		// if tree dumping is enabled
  		bool treeDumping;
  		// tree dumping interval
  		uint16_t treeDumpingInterval;
	} config_t;
  	config_t bconf;
  	
  	uint16_t ctpGetNewDelay();
  	void task sendCtpMsg();
  	void ctpMessageSend(message_t *msg, void *payload);
  	void sendCtpInfoMsg(uint8_t type, uint8_t arg);
  	
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
	void task sendAlive();
	
	/********************** FORWARD DECLARATIONS ********************/
	void setPower(message_t *, uint8_t);
	uint8_t getPower(message_t *);
  	void setChannel(uint8_t);
  	uint8_t getChannel();
	void setAutoAck(bool enableAutoAck, bool hwAutoAck);
	void setAck(message_t *msg, bool status);
	void CommandReceived(message_t * msg, void * payload, uint8_t len);
	void task sendCommandACK();
	void task sendCommandRadio();
	void setAddressRecognitionEnabled(bool enabled);
	
	/************************** COMMANDS ***************************/
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
		// hard wired configuration
		bconf.ctpTxData=7;
		bconf.ctpTxRoute=7;
		bconf.sendingCTP=TRUE;
		bconf.rootAddress=50;
		
		bconf.treeDumping=TRUE;
		bconf.treeDumpingInterval=2000;
		
		bconf.ctpSendRequest.packets=100;
		bconf.ctpSendRequest.delay=10000;
		bconf.ctpSendRequest.delayVariability=5000;
		bconf.ctpSendRequest.flags |= CTP_SEND_REQUEST_COUNTER_STRATEGY_SUCCESS;
		bconf.ctpSendRequest.flags |= CTP_SEND_REQUEST_PACKETS_UNLIMITED;
		bconf.ctpSendRequest.flags &= ~(CTP_SEND_REQUEST_TIMER_STRATEGY_PERIODIC);
		// apply config		
		ctpTxData=bconf.ctpTxData;
		ctpTxRoute=bconf.ctpTxRoute;
		//==========================================================
		
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

	// dump tree structure
	event void TreeTimer.fired(){
		sendCtpInfoMsg(0, 0);
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
			
		// start forwarding
		call ForwardingControl.start();
		
		// tree dumping?
		if (bconf.treeDumping){
			call TreeTimer.startPeriodic(bconf.treeDumpingInterval);
		}
		
		// apply loaded config - prevents suddenly node reset and
		// destroying txpower-down-scaled tree with full power transmission
		if (bconf.sendingCTP){
			// start sending each X seconds
			memcpy((uint8_t*) &ctpSendRequest, (uint8_t*) &(bconf.ctpSendRequest), sizeof(nx_struct CtpSendRequestMsg));
			
			// set correct root if this node ID is root
			if (TOS_NODE_ID==bconf.rootAddress){
				call RootControl.setRoot();
			}

			// one-shot timer only, add some time for CTP tree stabilization at boot
			call CtpTimer.startOneShot(ctpGetNewDelay()+3000);
			
			// send report about sending
			sendCtpInfoMsg(0, 0);
		}
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
		call UartCtpReportDataSender.sendState(TRUE);
		call UartCtpInfoMsgSender.sendState(TRUE);
		call AliveTimer.startPeriodic(500);
	}
  
    /************************ SENDING ************************/
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
			// only one report here, yet
			btrpkt->command_id = aliveCounter;
			btrpkt->reply_on_command = COMMAND_IDENTIFY;
			btrpkt->command_code = COMMAND_ACK;
			btrpkt->command_version = 1;
			btrpkt->command_data = 1;
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
	        btrpkt->command_data_next[2] = 0;
	        btrpkt->command_data_next[2] |= (0 << 8);
	        btrpkt->command_data_next[3] = 0;
	        btrpkt->command_data_next[3] |= (0<<8);
	        // need to set source
    		call UartAMPacket.setSource(&cmdPkt, TOS_NODE_ID);
    		
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
			// data=4 -> complete routing reinit
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
				} else if (btrpkt->command_data==4){
					call CtpTimer.stop();
					ctpBusy=FALSE;
					ctpBusyCount=0;
					
					call ForwardingControl.stop();
					call LinkEstimatorInit.init();
					call RoutingInit.init();
					call ForwardingInit.init();
					call ForwardingControl.start();
					
					btrpktresponse->command_data = 4;
				} else if (btrpkt->command_data==5){
					call ForwardingInit.init();
				} else if (btrpkt->command_data==6){
					call ForwardControl.postTask();
				} else if (btrpkt->command_data==7){
					call ForwardControl.flushQueue();
				} else {
					btrpktresponse->command_data = 0xffff;
				}
		
				btrpktresponse->command_code = COMMAND_ACK;
				post sendCommandACK();
			break;
			
			// gets basic CTP info from CtpInfo interface
			// data=0 -> returns parent, etx, neighbors count in data[0], data[1], data[2]
			// data=1 -> info about neighbor specified in data[0]. Returned addr, link quality, route
    		//				quality, congested bit
			case COMMAND_CTP_GETINFO:
				sendCtpInfoMsg(btrpkt->command_data, btrpkt->command_data_next[0]);
			break;
			
			// other CTP controling, can set TX power for packets
			// data=0 -> set tx power for OUTPUT messages for CTP protocol.
			// 				if data[0] == 1	-> set TXpower for ROUTE messages on data[1] level
			//			 	if data[0] == 2 -> set TXpower for DATA messages on data[1] level
			//				if data[0] == 3 -> set TXpower for both ROUTE, DATA messages on data[1] level
			//
			// data=1 -> enable/disable CTP debug 
			//
			case COMMAND_CTP_CONTROL:
				if (btrpkt->command_data==0){
					// output CTP TX power
					btrpktresponse->command_data_next[0] = 0;
					btrpktresponse->command_data_next[1] = 0;
						
					// set tx power for ROUTE messages
					if ((btrpkt->command_data_next[0] & 1)>0){
						ctpTxRoute = (uint8_t) btrpkt->command_data_next[1];
						btrpktresponse->command_data_next[0] = ctpTxRoute;
					}
					
					// set tx power for DATA messages
					if ((btrpkt->command_data_next[0] & 2)>0){
						ctpTxData = (uint8_t) btrpkt->command_data_next[1];
						btrpktresponse->command_data_next[1] = ctpTxData; 
					}
					
					btrpktresponse->command_code = COMMAND_ACK;
					post sendCommandACK();
					
				} else if (btrpkt->command_data==1) {
					// CTP debug
					if (btrpkt->command_data_next[0]>0){
						call CtpLoggerControl.start();
					} else {
						call CtpLoggerControl.stop();
					}
					
					btrpktresponse->command_data_next[0] = btrpkt->command_data_next[0];
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

    // deprecated, allow to send any command
    // used for queueFlush command too
    //
    // ACK as reply, if not set already
    //if (btrpkt->command_code != COMMAND_ACK && btrpkt->command_code != COMMAND_NACK)
    //    btrpkt->command_code = COMMAND_ACK;

	// need to set source
    call UartAMPacket.setSource(&cmdPktResponse, TOS_NODE_ID);

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
        error_t sendResult=SUCCESS;
		
		// CTP didn't returned a response
		if (ctpBusy){
			ctpBusyCount+=1;
			
			// logg fail
        	call CtpLogger.logEventSimple(DBG_CTP_SEND_RETRY_FAIL, ctpBusyCount);
        	
			//if ((ctpBusyCount % 25) == 0){
				// send status report message
				sendCtpInfoMsg(0,0);
			//}
			
			// start re-tx timer, if aperiodic timer is choosen
			if ((ctpSendRequest.flags & CTP_SEND_REQUEST_TIMER_STRATEGY_PERIODIC)==0){
				call CtpTimer.startOneShot(CTP_RETX_TIME + (ctpBusyCount % 50));
			} else {
				post sendCtpMsg();
			}
			return;
		}
		
		btrpkt = (CtpResponseMsg*)(call CtpSend.getPayload(&ctpPkt, sizeof(CtpResponseMsg)));

        call CtpInfo.getParent(&parent);
        call CtpInfo.getEtx(&metric);

        btrpkt->origin = TOS_NODE_ID;
        btrpkt->seqno = ctpCurPackets;
        btrpkt->data = call Random.rand16();
        btrpkt->parent = parent;
        btrpkt->metric = metric;
        
        sendResult = call CtpSend.send(&ctpPkt, sizeof(CtpResponseMsg));
        if (sendResult == SUCCESS) {
            ctpBusy=TRUE;
            dbg("IDS-app","App: sent packet with seqno %d to parent %d", msg->seqno, msg->parent);
            
            // send status report message
			sendCtpInfoMsg(0,0);
            
            // report sending 
            ctpMessageSend(&ctpPkt, btrpkt);
        } else {
        	// log fail
        	call CtpLogger.logEventSimple(DBG_CTP_SEND_FAIL, sendResult);
        	
        	// start re-tx timer, if aperiodic timer is chosen
			if ((ctpSendRequest.flags & CTP_SEND_REQUEST_TIMER_STRATEGY_PERIODIC)==0){
				call CtpTimer.startOneShot(CTP_RETX_TIME);
			} else {
				post sendCtpMsg();
			}
        }
	}

	/**
	 * CTP message was sent. Start timer again according to properties set
	 */
	event void CtpSend.sendDone(message_t *msg, error_t error){
		//if (&ctpPkt == msg) {
			ctpBusyCount=0;
            ctpBusy = FALSE;
            // packet counter increment based on strategy chosen
			if ((ctpSendRequest.flags & CTP_SEND_REQUEST_COUNTER_STRATEGY_SUCCESS) > 0){
				// increment only on succ
				if (error==SUCCESS){
					ctpCurPackets+=1;
				}
			} else {
				// increment everytime
				ctpCurPackets+=1;
			}
			
			// timer starting based on strategy chosen
			if ((ctpSendRequest.flags & CTP_SEND_REQUEST_TIMER_STRATEGY_PERIODIC)==0 && ctpSendRequest.delay>0){
				// here start only one shot timer, if this strategy is prefered
				// generate new delay based on variability
				call CtpTimer.startOneShot(ctpGetNewDelay());
			}
        //}
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
			if(ctpSendRequest.packets > ctpCurPackets || (ctpSendRequest.flags & CTP_SEND_REQUEST_PACKETS_UNLIMITED) > 0) {
				// still has sent less packets than expected - send
				post sendCtpMsg();
			}
			else if((ctpSendRequest.flags & CTP_SEND_REQUEST_PACKETS_UNLIMITED) == 0) {
				// Turn timer off iff every packet was already sent
				// But if PACKETS_UNLIMITED flag is set, do not stop this process			
				call CtpTimer.stop();

				// reset coutner
				ctpCurPackets = 0;
				// freeze next attempts
				ctpSendRequest.delay = 0;
				
				// log finish
        		call CtpLogger.logEventSimple(DBG_CTP_FINISHED, 0);
			}
		}
	}
	
	/**
	 * Generates new delay for CTP timer, using variability
	 */
	uint16_t ctpGetNewDelay(){
		uint16_t newDelay = ctpSendRequest.delay;
		if (ctpSendRequest.delayVariability>0){
			uint16_t r = call Random.rand16();
	    	r %= (2 * ctpSendRequest.delayVariability);
	    	r -= ctpSendRequest.delayVariability;
	    	newDelay = ctpSendRequest.delay + r;
#ifdef REPORT_NEW_DELAY	    	
	    	call CtpLogger.logEventDbg(DBG_CTP_NEWDELAY, newDelay, r, 1);
#endif	    	
		} else {
#ifdef REPORT_NEW_DELAY			
			call CtpLogger.logEventSimple(DBG_CTP_NEWDELAY, newDelay);
#endif			
		}
		
		return newDelay;
	}

	/**
	 * if CtpRequest to send is intercepted on serial, then new sending is started
	 */
	 
	event message_t * UartCtpSendRequestReceiver.receive(message_t *msg, void *payload, uint8_t len){
		// message is for me, thus need to start sending CtpMessages
		CtpSendRequestMsg * btrpkt = NULL;
		if(len != sizeof(CtpSendRequestMsg)) {
			// invalid length - cannot process
			return msg;
		}	
		
		// set curr running to zero, stop timer 
		call CtpTimer.stop();
		ctpCurPackets = 0;
		ctpBusyCount=0;
        ctpBusy = FALSE;	
		
		// get received message
		btrpkt = (CtpSendRequestMsg * ) payload;			
		// copy current request from packet to local var
		memcpy((void*) (&ctpSendRequest), payload, sizeof(CtpSendRequestMsg));
        
		// if delay=0 => stop ping send
		if(btrpkt->delay==0){
			return msg;
		}
		
		// depending on timer strategy start timer...
		if ((btrpkt->flags & CTP_SEND_REQUEST_TIMER_STRATEGY_PERIODIC) > 0){
			// periodic timer with defined delay
			call CtpTimer.startPeriodic(ctpGetNewDelay());
		} else {
			// one-shot timer only
			call CtpTimer.startOneShot(ctpGetNewDelay());
		}
		
		// send report about sending
		sendCtpInfoMsg(0, 0);
		return msg;
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
			if(call UartCtpReportDataSender.full() == FALSE) {
				// dequeue from RSSI QUEUE, Build message, add to serial queue
				CtpReportDataMsg dataBuff;
				CtpReportDataMsg * btrpkt = &dataBuff;
				
				// spoof = false, ctp original=1
				btrpkt->flags = 0x2;
				btrpkt->amSource = call AMPacket.source(msg);
				btrpkt->data.recv.rssi = getRssi(msg);
				btrpkt->timestamp32khz = ((call PacketTimeStamp32khz.isValid(msg))==FALSE) ? 0 : call PacketTimeStamp32khz.timestamp(msg);
				btrpkt->localTime32khz = call LocalTime32khz.get();
				memcpy((void*)(&(btrpkt->response)), payload, len);
				memset((void*)(&(btrpkt->data.recv.ctpDataHeader)), 0, sizeof(btrpkt->data.recv.ctpDataHeader));						

				if (call UartCtpReportDataSender.enqueueData(btrpkt, sizeof(CtpReportDataMsg))==SUCCESS){
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
	 * Called when CTP message was sent - report it to base station
	 */
	void ctpMessageSend(message_t *msg, void *payload){
		//CTP spoof is interesting for me
		CtpResponseMsg * response = payload;
				
		atomic {
			// queue is full?
			if(call UartCtpReportDataSender.full() == FALSE) {
				// dequeue from RSSI QUEUE, Build message, add to serial queue
				CtpReportDataMsg dataBuff;
				CtpReportDataMsg * btrpkt = &dataBuff;	
				
				btrpkt->flags = 0x4;
				btrpkt->amSource = TOS_NODE_ID;
                btrpkt->timestamp32khz = ((call PacketTimeStamp32khz.isValid(msg))==FALSE) ? 0 : call PacketTimeStamp32khz.timestamp(msg);
                btrpkt->localTime32khz = call LocalTime32khz.get();
                memcpy((void*)&(btrpkt->response), (void*)response, sizeof(CtpResponseMsg));
				
				if (call UartCtpReportDataSender.enqueueData(btrpkt, sizeof(CtpReportDataMsg))){
					return;
				} else {
					// message add failed, try again later
					return;
				}
			}
		}
	}
	
	/**
	 * Send CTP info message - global status / particular neigh info
	 */
	void sendCtpInfoMsg(uint8_t type, uint8_t arg){
		CtpInfoMsg infoBuff;
		CtpInfoMsg * btrpkt = &infoBuff;	
	
		{
			// queue is full?
			if(call UartCtpInfoMsgSender.full()==FALSE) {
				// dequeue from RSSI QUEUE, Build message, add to serial queue
				btrpkt->type = type;
				btrpkt->localTime32khz = call LocalTime32khz.get();
				if(type==0){
					am_addr_t parent = 0;
					uint16_t etx=0;
					
					// provide basic information about my CTP perspective					
					call CtpInfo.getParent(&parent);
					call CtpInfo.getEtx(&etx);
					(btrpkt->data).status.parent = parent;
					btrpkt->data.status.etx = etx;
					btrpkt->data.status.neighbors = call CtpInfo.numNeighbors();
					btrpkt->data.status.serialQueueSize = call UartCtpReportDataSender.size();
					btrpkt->data.status.ctpSeqNo = ctpCurPackets;
					btrpkt->data.status.ctpBusyCount = ctpBusyCount;
					btrpkt->data.status.flags = 0;
					btrpkt->data.status.flags |= ctpBusy ? 1 : 0;
				} else if (type==1){
					// provide information about particular neighbor
					btrpkt->data.neighInfo.num = arg;
					btrpkt->data.neighInfo.addr = call CtpInfo.getNeighborAddr(arg);
					btrpkt->data.neighInfo.linkQuality = call CtpInfo.getNeighborLinkQuality(arg);
					btrpkt->data.neighInfo.routeQuality = call CtpInfo.getNeighborRouteQuality(arg);
					btrpkt->data.neighInfo.flags = 0;
					btrpkt->data.neighInfo.flags |= call CtpInfo.isNeighborCongested(btrpkt->data.neighInfo.addr) ? 1 : 0;
				} else {
					return;
				}
				
				atomic {
					if (call UartCtpInfoMsgSender.full()==FALSE && call UartCtpInfoMsgSender.enqueueData(btrpkt, sizeof(CtpInfoMsg))==SUCCESS){
						return;
					} else {
						// message add failed, try again later
						return;
					}
				}
			}
		}
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
			if(call UartCtpReportDataSender.full() == FALSE) {
				// dequeue from RSSI QUEUE, Build message, add to serial queue
				CtpReportDataMsg dataBuff;
				CtpReportDataMsg * btrpkt = &dataBuff;		
				
				btrpkt->flags = 0x0;
				btrpkt->flags |= spoof ? 0x1:0x0;
				btrpkt->amSource = call AMPacket.source(msg);
				btrpkt->data.recv.rssi = getRssi(msg);
                btrpkt->timestamp32khz = ((call PacketTimeStamp32khz.isValid(msg))==FALSE) ? 0 : call PacketTimeStamp32khz.timestamp(msg);
                btrpkt->localTime32khz = call LocalTime32khz.get();
				memcpy((void*)&(btrpkt->data.recv.ctpDataHeader), (void*)ctpDataHeader, sizeof(ctp_data_header_t));
				memcpy((void*)&(btrpkt->response), (void*)response, sizeof(CtpResponseMsg));		
				
				// use queue here to add messages
				// build queue message
				if (call UartCtpReportDataSender.enqueueData(btrpkt, sizeof(CtpReportDataMsg))==SUCCESS){
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
	event message_t * AMTapForg.snoop(uint8_t type, message_t *msg, void *payload, uint8_t len){
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
	event message_t * AMTapForg.receive(uint8_t type, message_t *msg, void *payload, uint8_t len){
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
	event message_t * AMTapForg.send(uint8_t type, message_t *msg, uint8_t len){
		// CTP ROUTE message sent here, set wanted tx power
		// maximum power is default, thus ignore maximum power level
		if (type==AM_CTP_DATA && ctpTxData<=31){
			setPower(msg, ctpTxData);
		}
		
		// CTP ROUTE message sent here, set wanted tx power
		// maximum power is default, thus ignore maximum power level
		if (type==AM_CTP_ROUTING && ctpTxRoute<=31){
			setPower(msg, ctpTxRoute);
		}
		
		return msg; 
	}
}
