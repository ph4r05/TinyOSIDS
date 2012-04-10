#include "SerialQueueSender.h"

generic configuration SerialQueueSenderC(typedef queue_t, uint8_t QUEUE_SIZE, am_id_t amtype) {
  provides {
    interface QueueSender;
  }
}
implementation {
  components new SerialQueueSenderP(queue_t, QUEUE_SIZE, amtype) as SQ;
  
  // components needed for serial queue sender P
  components new SerialAMSenderC(amtype);
  components new PoolC(queue_t, QUEUE_SIZE) as MessagePool;
  components new QueueC(senderMetadata_t*, QUEUE_SIZE) as SendQueue;
  components new PoolC(senderMetadata_t, QUEUE_SIZE) as MessageMetaPool;
  components RandomC;
  components new TimerMilliC() as RetxmitTimer;
  
  QueueSender = SQ.QueueSender; 
  SQ.AMSend -> SerialAMSenderC;
  SQ.Packet -> SerialAMSenderC;
  SQ.AMPacket -> SerialAMSenderC;
  SQ.MessagePool -> MessagePool;
  SQ.MessageMetaPool -> MessageMetaPool;
  SQ.SendQueue -> SendQueue;
  SQ.Random -> RandomC;
  SQ.RetxmitTimer-> RetxmitTimer;
}