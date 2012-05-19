
configuration AddOnC {
	provides {
		interface AddOn;
	}
}
implementation {
  // main components for reset, bootup, leds	
  components MainC, LedsC;
  
  // active message components
  components ActiveMessageC as Radio, SerialActiveMessageC as Serial;
  
  // main addon component
  components AddOnP;
  
  // HW reset
  components ResetC;
  AddOnP.Reset -> ResetC;
  
  // tapping interface from forged message
  // Since BaseStation does not support radioSending correctly now, we need
  // to hook send() calls by this way. Needed to set TX power for some messages
  components ForgedActiveMessageC as FAM;
  AddOnP.AMTap -> FAM.AMTap;
  
  // MACAuth component for verifying authenticity of received messages (TOS has no means to determine this)
  components MACAuthC;
  AddOnP.MACAuth -> MACAuthC;
  
  // component for listening to invalid packets
  components BadCRCReceiverC;
  AddOnP.ReceiveBadCRC -> BadCRCReceiverC;
  
  // CC2420 configuration
  components CC2420ControlC;
  AddOnP.CC2420Config -> CC2420ControlC;
  
  components CC2420TransmitC;
  AddOnP.CCAIndicator -> CC2420TransmitC.EnergyIndicator;
  AddOnP.CC2420Transmit -> CC2420TransmitC.CC2420Transmit;
  
  // to be able to access CC2420 packet metadata
  components CC2420PacketC;
  AddOnP.CC2420PacketBody -> CC2420PacketC;
  AddOnP.CC2420Packet -> CC2420PacketC;
  
  // provides wiring
  AddOn = AddOnP.AddOn;
}
