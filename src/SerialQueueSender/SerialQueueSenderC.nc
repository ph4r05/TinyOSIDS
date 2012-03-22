#include "SerialQueueSender.h"

generic configuration SerialQueueSenderC(typedef queue_t, uint8_t QUEUE_SIZE, am_id_t amtype) {
  provides {
    interface QueueSender;
  }
}
implementation {
  components new SerialAMSenderC(amtype);
  components new SerialQueueSenderP(queueSenderQueue_element_t, QUEUE_SIZE, amtype) as SQ;
  
  QueueSender = SQ.QueueSender; 
  SQ.AMSend -> SerialAMSenderC;
  SQ.Packet -> SerialAMSenderC;
  SQ.AMPacket -> SerialAMSenderC;
}