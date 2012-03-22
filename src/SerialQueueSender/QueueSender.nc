#include "../RssiDemoMessages.h"
#include "SerialQueueSender.h"

interface QueueSender {
	// on serialStarted event call sendState(true)
	command error_t sendState(bool start);
	
	command bool empty();
	command bool full();
	command uint8_t size();
	command uint8_t maxSize();
	command error_t enqueueRaw(am_id_t id, am_addr_t addr, message_t *msg, void * payload, uint8_t len, bool radioPacket);
	command error_t enqueue(queueSenderQueue_element_t * newVal);
	command queueSenderQueue_element_t * head();
	command queueSenderQueue_element_t * element(uint8_t idx);
}