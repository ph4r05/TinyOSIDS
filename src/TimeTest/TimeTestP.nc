/**
 * Application to test that the TinyOS java toolchain can communicate
 * with motes over the serial port. 
 *
 *  @author Gilman Tolle
 *  @author Philip Levis
 *  
 *  @date   Aug 12 2005
 *
 **/

#include "Timer.h"
#include "TestSerial.h"

module TimeTestP {
  uses {
    interface SplitControl as Control;
    interface SplitControl as ControlRadio;
    interface Leds;
    interface Boot;
//    interface Receive;
    interface AMSend;
    interface Timer<TMilli> as MilliTimer;
    interface Packet;

// my extension

    interface Receive as RadioCmdRecv;
    interface Receive as UartCmdRecv;

    interface AMSend as UartCmdAMSend;
    interface AMSend as RadioCmdAMSend;

    interface Reset as Reset;

    interface Timer<TMilli> as AliveTimer;
  }
}
implementation {

  message_t packet;

  bool locked = FALSE;
  uint16_t counter = 0;

  /**************** COMMANDS ****************/
  message_t cmdPkt;
  message_t cmdPktResponse;
  uint16_t commandCounter=0;
  am_addr_t commandDest=0;
  bool commandRadio=FALSE;
  // base station address
  am_addr_t baseid = 1;
  bool cmdRadioBusy=FALSE;
  bool cmdUartBusy=FALSE;

  // message 2 send
  CommandMsg cmdMsgPayload;

  uint16_t cmdReceived=0;

  uint16_t aliveCounter=0;
  bool serialBusy=FALSE;
  /********** Forward declarations **********/
  void CommandReceived(message_t * msg, void * payload, uint8_t len);
  void task sendCommandRadio();
  void task sendCommandACK();
  void task sendAlive();



  /************** MAIN CODE BELOW ***********/
  event void Boot.booted() {
    call Control.start();
  }
  
  event void MilliTimer.fired() {
    counter++;
    if (locked) {
      return;
    }
    else {
      test_serial_msg_t* rcm = (test_serial_msg_t*)call Packet.getPayload(&packet, sizeof(test_serial_msg_t));
      if (rcm == NULL) {return;}
      if (call Packet.maxPayloadLength() < sizeof(test_serial_msg_t)) {
	return;
      }

      rcm->counter = counter;
      if (call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(test_serial_msg_t)) == SUCCESS) {
	locked = TRUE;
      }
    }
  }

 event void AMSend.sendDone(message_t* bufPtr, error_t error) {
    if (&packet == bufPtr) {
      locked = FALSE;
    }
  }

////////////////////////////////////////////////////////////////
////////////  MY extensions ////////////////////////////////////
////////////////////////////////////////////////////////////////
    /************************ COMMAND RECEIVED ************************/
    void CommandReceived(message_t * msg, void * payload, uint8_t len) {
		CommandMsg * btrpkt = NULL;
		CommandMsg * btrpktresponse = NULL;
		//if(len != sizeof(CommandMsg)) {
		//	// invalid length - cannot process
		//	return;
		//}		
	
		cmdReceived+=1;

		// get received message
		btrpkt = (CommandMsg * ) payload;

		// get local message, prepare it
		//btrpktresponse = (CommandMsg * )(call UartCmdAMSend.getPayload(&cmdPktResponse, sizeof(CommandMsg)));
		btrpktresponse = (CommandMsg * )(&cmdMsgPayload);

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
				aliveCounter=0;
				//call AliveTimer.stop();
				
				// reset is too fast, application tries to send it 
				//call Reset.reset();
				// code execution should not reach this statement
				post sendCommandACK();
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
				break;
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

	btrpkt=(CommandMsg*)(call RadioCmdAMSend.getPayload(&cmdPktResponse, 0));

	// copy data from command msg payload stored
	memcpy((void *)btrpkt, (void *)&cmdMsgPayload, sizeof(CommandMsg));

	// setup message with data
	btrpkt->command_id = counter;

	// send to base directly
	// sometimes node refuses to send too large packet. it will always end with fail
	// depends of buffers size.
	if (call RadioCmdAMSend.send(commandDest, &cmdPktResponse, sizeof(CommandMsg)) == SUCCESS) {
	    cmdRadioBusy=TRUE;
	}
	else {
		dbg("Cannot send message");
		post sendCommandRadio();
	}	
  }


  /**
   * Send ACK command
   * packet is prepared before calling this
   */
   void task sendCommandACK(){
	CommandMsg* btrpkt = NULL;
	if (cmdUartBusy){
		post sendCommandACK();
		return;
	}

    btrpkt=(CommandMsg*)(call UartCmdAMSend.getPayload(&cmdPktResponse, 0));

    // copy data from command msg payload stored
    memcpy((void *)btrpkt, (void *)&cmdMsgPayload, sizeof(CommandMsg));

    // setup message with data
    btrpkt->command_id = counter;

    // send to base directly
    // sometimes node refuses to send too large packet. it will always end with fail
    // depends of buffers size.
    if (call UartCmdAMSend.send(commandDest, &cmdPktResponse, sizeof(CommandMsg)) == SUCCESS) {
	cmdUartBusy = TRUE;
    }
    else {
	dbg("Cannot send message");
	post sendCommandACK();
    }
  }

  /**
   * Command received on uart
   */
  event message_t* UartCmdRecv.receive(message_t* bufPtr, void* payload, uint8_t len) {
	CommandReceived(bufPtr, payload, len);

	return bufPtr;
  }

  /**
   * Command received on radio
   */ 
  event message_t* RadioCmdRecv.receive(message_t* bufPtr, void* payload, uint8_t len) {
	CommandReceived(bufPtr, payload, len);

	return bufPtr;
  }
  
  /**
   * Radio command send done
   */
  event void RadioCmdAMSend.sendDone(message_t *msg, error_t error){
	if (&cmdPktResponse==msg || &cmdPkt == msg){
		cmdRadioBusy=FALSE;
		if (error!=SUCCESS){
			post sendCommandRadio();
		}
	}
  }

  /**
   * Uart command send done
   */
  event void UartCmdAMSend.sendDone(message_t* bufPtr, error_t error) {
	if (&cmdPktResponse==bufPtr || &cmdPkt==bufPtr){
		cmdUartBusy=FALSE;
		if (error!=SUCCESS){
			post sendCommandACK();
		}
	}
  }

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
	if (cmdUartBusy) {
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
	btrpkt->command_data_next[2] = cmdReceived;
	btrpkt->command_data_next[3] = 0;
	
		if(call UartCmdAMSend.send(TOS_NODE_ID, &cmdPkt, sizeof(CommandMsg)) == SUCCESS) {
			aliveCounter+=1;
			cmdUartBusy = TRUE;
		}
		else {
			post sendAlive();
			dbg("Cannot send identify message");
		}
	}
  }

//////////////////////////////////////////////////////
// Radio & serial initialization
//////////////////////////////////////////////////////

  /** 
   * Serial initialized event
   */ 
  event void Control.startDone(error_t err) {
    if (err == SUCCESS) {
//      call MilliTimer.startPeriodic(1000);

	// initialize radio communication now
	call AliveTimer.startPeriodic(500);

	// initialize radio communication now
//	call ControlRadio.start();
    }
  }
  event void Control.stopDone(error_t err) {}

  /**
   * Radio initialized event
   */
  event void ControlRadio.startDone(error_t err) {
    if (err == SUCCESS) {
        // radio initialized  
//	call AliveTimer.startPeriodic(500);
    }
  }
  event void ControlRadio.stopDone(error_t err) {}

}




