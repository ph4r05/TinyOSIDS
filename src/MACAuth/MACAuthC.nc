/**
 * @author ph4r05
 * @modified Apr 16, 2012 
 */

configuration MACAuthC {
	provides interface MACAuth;
} implementation {
  // reset timers
  components CC2420PacketC as Packet;
  components MACAuthP;
  
  MACAuthP.CC2420Packet -> Packet;
  MACAuthP.CC2420PacketBody -> Packet;
  
  MACAuth = MACAuthP.MACAuth;
}