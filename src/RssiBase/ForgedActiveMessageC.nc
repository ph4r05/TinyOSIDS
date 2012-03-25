/**
 * This component serves to tapping the communication from 
 * radio stack.  
 */

configuration ForgedActiveMessageC
{
	provides
	{
		interface SplitControl;
		interface AMSend[uint8_t id];
//		interface Receive[uint8_t id];
//		interface Receive as Snoop[uint8_t id];
		interface Packet;
		interface AMPacket;
		interface PacketAcknowledgements;


  		interface AMTap;
	}

}

implementation
{
	components ActiveMessageC;
  	components ForgedActiveMessageP;

    AMTap = ForgedActiveMessageP.AMTap;    
	
  	//forged components
  	AMSend       = ForgedActiveMessageP.AMSend;
//  	Receive      = ForgedActiveMessageP.Receive;
//	Snoop        = ForgedActiveMessageP.Snoop;
	
  	//defaults 
	SplitControl = ActiveMessageC;
 	Packet       = ActiveMessageC;
	AMPacket     = ActiveMessageC;
	PacketAcknowledgements	= ActiveMessageC;

	ForgedActiveMessageP.ExtAMSend -> ActiveMessageC.AMSend;
//  	ForgedActiveMessageP.ExtReceive -> ActiveMessageC.Receive;
//  	ForgedActiveMessageP.ExtSnoop -> ActiveMessageC.Snoop;
  	ForgedActiveMessageP.AMPacket -> ActiveMessageC.AMPacket;
}
