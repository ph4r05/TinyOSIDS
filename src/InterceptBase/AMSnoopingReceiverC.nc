/**
 * The virtualized abstraction to hearing all packets of a given AM type,
 * whether destined for this node or not.
 *
 * @author Philip Levis
 * @date   Jan 16 2006
 * @see    TEP 116: Packet Protocols
 */ 
 
 
 /** 
  * FORGED COMPONENT, used for tapping communication from radio stack
  * Lumir Honus
  */


#include "AM.h"

generic configuration AMSnoopingReceiverC(am_id_t AMId) {
  provides {
    interface Receive;
    interface Packet;
    interface AMPacket;
  }
}

implementation {
  components BaseStationC;

  // fan-in call
  Receive = BaseStationC.Snoop[AMId];
  Receive = BaseStationC.Receive[AMId];
  
  Packet = BaseStationC;
  AMPacket = BaseStationC;
}