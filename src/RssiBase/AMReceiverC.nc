/**
 * The virtualized AM reception abstraction.
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

generic configuration AMReceiverC(am_id_t amId) {
  provides {
    interface Receive;
    interface Packet;
    interface AMPacket;
  }
}

implementation {
  components BaseStationC;

  Receive = BaseStationC.Receive[amId];
  Packet = BaseStationC;
  AMPacket = BaseStationC;
}