/**
 * The virtualization of snooping on overheard packets that are not
 * destined to this node. 
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

generic configuration AMSnooperC(am_id_t AMId) {
  provides {
    interface Receive;
    interface Packet;
    interface AMPacket;
  }
}

implementation {
  components BaseStationC;

  Receive = BaseStationC.Snoop[AMId];
  Packet = BaseStationC;
  AMPacket = BaseStationC;
}
