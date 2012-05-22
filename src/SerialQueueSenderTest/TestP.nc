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
#include "application.h"
#include "printf.h"

module TestP {
  uses {
    interface SplitControl as Control;
    interface Leds;
    interface Boot;
    interface Receive;
    interface QueueSender as SerialSender;
    interface Timer<TMilli> as MilliTimer;
    interface Packet;
  }
}
implementation {
  enum {
	RADIO_CYCLE=1
  };


  message_t packet;

  bool locked = FALSE;
  uint16_t counter = 0;
  uint16_t recv = 0;
  bool sendIt=FALSE;		


  /************** MAIN CODE BELOW ***********/

  event void Boot.booted() {
  	// init serial sender
  	call SerialSender.sendState(TRUE);
    
    // initialize serial communication right now
    call Control.start();
  }
 
  // status sending 
  // should be fired every second
  event void MilliTimer.fired() {
		test_serial_msg_t * rcm = (test_serial_msg_t * ) call Packet.getPayload(&packet, sizeof(test_serial_msg_t));
		counter++;
		
		if(rcm == NULL) {
			return;
		}
		if(call Packet.maxPayloadLength() < sizeof(test_serial_msg_t)) {
			return;
		}

		rcm->counter = counter;
		rcm->received = recv;
		rcm->radioOn = 0;
		rcm->radioCn = 0;
		rcm->radioSent = 0;
		rcm->radioRecv = 0;
		rcm->radioErr = 0;
		rcm->radioErrCn = 0;

		printf("SMEnq, cn: %d\n", counter);
		call SerialSender.enqueueData(rcm, sizeof(test_serial_msg_t));

  }

  event message_t* Receive.receive(message_t* bufPtr, void* payload, uint8_t len) {
	recv+=1;
	return bufPtr;
   }
   
  /** 
   * Serial initialized event
   */ 
  event void Control.startDone(error_t err) {
    if (err == SUCCESS) {
      // serial init successful, start sending status reports 
      call MilliTimer.startPeriodic(500);
    }
  }
  event void Control.stopDone(error_t err) {}

}




