/**
 * This component just forwards messages between forged components 
 * (AMSender, AMReceiver, AMSnoop, AMSnoopingReceiver) and ActiveMessageC
 * 
 * Moreover it allows to tap all messages by AMTap interface
 */

module ForgedActiveMessageP{
  provides{
		interface AMSend[uint8_t id];
		interface Receive[uint8_t id];
		interface Receive as Snoop[uint8_t id];
		interface AMTap;
		interface SendDoneTap;
		
  }
  uses {
    interface AMSend as ExtAMSend[uint8_t];
    interface Receive as ExtReceive[uint8_t];
    interface Receive as ExtSnoop[uint8_t];
    interface AMPacket;

    
  }
}

implementation {

	/********** AMSend ************************/
	command error_t AMSend.send[am_id_t id](am_addr_t addr, message_t * msg, uint8_t len) {
		//uint8_t packet_type = call AMPacket.type(msg);
		msg = signal AMTap.send(id, msg, len);
		return call ExtAMSend.send[id](addr, msg, len);
	}

	command error_t AMSend.cancel[uint8_t id](message_t * msg) {
		return call ExtAMSend.cancel[id](msg);
	}

	command uint8_t AMSend.maxPayloadLength[uint8_t id]() {
		return call ExtAMSend.maxPayloadLength[id]();
	}

	command void * AMSend.getPayload[uint8_t id](message_t * msg, uint8_t len) {
		return call AMSend.getPayload[id](msg, len);
		//return call AMSend.getPayload[call AMPacket.type(msg)](msg,len);
	}

	event void ExtAMSend.sendDone[uint8_t id](message_t * msg, error_t error) {
		signal SendDoneTap.sendDone(id, msg, error);
		signal AMSend.sendDone[id](msg, error);
		//signal AMSend.sendDone[call AMPacket.type(msg)](msg, error);
	}

	default event void AMSend.sendDone[am_id_t id](message_t * msg,	error_t error) {
	}

	/********* AM RECEIVE ********************/
	event message_t * ExtReceive.receive[am_id_t id](message_t * msg, void * payload, uint8_t len) {
		am_id_t type = call AMPacket.type(msg);
		msg = signal AMTap.receive(type, msg, payload, len);
		return signal Receive.receive[type](msg, payload, len);
	}

	default event message_t * Receive.receive[am_id_t id](message_t * msg, void * payload, uint8_t len) {
		return msg;
	}

	/********* AM SNOOP **********************/
	event message_t * ExtSnoop.receive[am_id_t id](message_t * msg, void * payload, uint8_t len) {
		uint8_t type = call AMPacket.type(msg);
		msg = signal AMTap.snoop(type, msg, payload, len);
		return signal Snoop.receive[type](msg, payload, len);
	}

	default event message_t * Snoop.receive[am_id_t id](message_t * msg, void * payload, uint8_t len) {
		return msg;
	}

	/******************************************/
	default event message_t * AMTap.receive(uint8_t type, message_t * msg, void * payload, uint8_t len) {
		return msg;
	}

	default event message_t * AMTap.snoop(uint8_t type, message_t * msg, void * payload, uint8_t len) {
		return msg;
	}

	default event message_t * AMTap.send(uint8_t type, message_t * msg,	uint8_t len) {
		return msg;
	}
	
	default event void SendDoneTap.sendDone(uint8_t type, message_t* msg, error_t error){
        ;
    }		
}
