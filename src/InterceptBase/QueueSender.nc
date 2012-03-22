#include "../RssiDemoMessages.h"
interface QueueSender {
	command bool empty();
	command bool full();
	command uint8_t size();
	command uint8_t maxSize();
	command error_t enqueue(queueSenderQueue_element_t * newVal);
	command queueSenderQueue_element_t * head();
	command queueSenderQueue_element_t * element(uint8_t idx);
}