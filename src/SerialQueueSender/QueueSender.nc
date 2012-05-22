#include "SerialQueueSender.h"

interface QueueSender {
	// on serialStarted event call sendState(true)
	command error_t sendState(bool start);
	
	command bool empty();
	command bool full();
	command uint8_t size();
	command uint8_t maxSize();
	command error_t enqueueData(void * payload, uint8_t len);
	command senderMetadata_t * head();
	command senderMetadata_t * element(uint8_t idx);
}