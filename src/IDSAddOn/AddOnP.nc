#include "addon.h"
module AddOnP @safe() {
    uses
    {
        interface Boot;
        interface Reset as Reset;
        interface Leds;
        
        // tapping interface to listen all communication to/from this node
        interface AMTap;
        
        // message authenticity
        interface MACAuth;
        
        // receive messages with wrong CRC
        // - out of normal message path due to message history, lpl and another modules
        // involved during message reception. Receiving such invalid packet could 
        // mislead these modules.
		interface ReceiveBadCRC;     
		
		// configuring CC2420 modes of operation
		interface CC2420Config;   
		
		// Receive indicator from transmit module - determined channel
		// occupancy by CCA sampling
		interface ReceiveIndicator as CCAIndicator;
		
		// hook every sendDone event - inspect CCA wait times
		interface CC2420Transmit;
		
		// to be able to directly access packet fields (metadata, headers)
		interface CC2420Packet;
	    interface CC2420PacketBody;
    }
	
	provides {
		interface AddOn;
    }
}

implementation
{  
    // event handler, on system boot
    // perform init tasks
    // prepare queues, starts interfaces
    event void Boot.booted() {
        
    }
    
    command void AddOn.test(){
    	
    }
	
	event message_t * AMTap.send(uint8_t type, message_t *msg, uint8_t len){
		// TODO Auto-generated method stub
		return msg;
	}

	event message_t * AMTap.snoop(uint8_t type, message_t *msg, void *payload, uint8_t len){
		// TODO Auto-generated method stub
		return msg;
	}

	event message_t * AMTap.receive(uint8_t type, message_t *msg, void *payload, uint8_t len){
		// TODO Auto-generated method stub
		return msg;
	}

	event message_t * ReceiveBadCRC.receive(message_t *msg, void *payload, uint8_t len){
		// TODO Auto-generated method stub
		return msg;
	}

	event void CC2420Config.syncDone(error_t error){
		// TODO Auto-generated method stub
	}

	async event void CC2420Transmit.sendDone(message_t *p_msg, error_t error){
		// TODO Auto-generated method stub
	}
}
