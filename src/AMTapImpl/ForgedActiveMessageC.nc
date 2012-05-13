/**
 * This component serves to tapping the communication from 
 * radio stack.  
 */

configuration ForgedActiveMessageC
{
	provides
	{
		interface SplitControl;

	    interface AMSend[am_id_t id];
	    interface Receive[am_id_t id];
	    interface Receive as Snoop[am_id_t id];
	
	    interface Packet;
	    interface AMPacket;
	    interface PacketAcknowledgements;
	    interface PacketTimeStamp<T32khz, uint32_t> as PacketTimeStamp32khz;
	    interface PacketTimeStamp<TMilli, uint32_t> as PacketTimeStampMilli;
	    interface LowPowerListening;
		
  		interface AMTap;
	}

}

implementation
{
	components ActiveMessageC as AM;
  	components ForgedActiveMessageP;

    AMTap = ForgedActiveMessageP.AMTap;    
	
  	//forged components
  	AMSend       = ForgedActiveMessageP.AMSend;
  	Receive      = ForgedActiveMessageP.Receive;
	Snoop        = ForgedActiveMessageP.Snoop;
	
  	//defaults 
	SplitControl = AM;
 	Packet       = AM;
	AMPacket     = AM;
	PacketAcknowledgements	= AM;
	LowPowerListening = AM;

	ForgedActiveMessageP.ExtAMSend -> AM.AMSend;
  	ForgedActiveMessageP.ExtReceive -> AM.Receive;
  	ForgedActiveMessageP.ExtSnoop -> AM.Snoop;
  	ForgedActiveMessageP.AMPacket -> AM.AMPacket;
  	
	  components CC2420PacketC;
	  PacketTimeStamp32khz = CC2420PacketC.PacketTimeStamp32khz;
	  PacketTimeStampMilli = CC2420PacketC.PacketTimeStampMilli;
}
