
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
  // With this interface addOn can listen to every message sent/received
  components ForgedActiveMessageC as FAM;
  AddOnP.AMTap -> FAM.AMTap;
  
  // MACAuth component for verifying authenticity of received messages (TOS has no means to determine this)
  components MACAuthC;
  AddOnP.MACAuth -> MACAuthC;
  
  // component for listening to invalid packets (CRC check failed)
  components BadCRCReceiverC;
  AddOnP.ReceiveBadCRC -> BadCRCReceiverC;
  
  // CC2420 configuration - set behaviour for CRC failed packets
  components CC2420ControlC;
  AddOnP.CC2420Config -> CC2420ControlC;
  
  // ability to sample CCA to determine radio channel occupancy
  components CC2420TransmitC;
  AddOnP.CCAIndicator -> CC2420TransmitC.EnergyIndicator;
  // ability to hook every sendDone event to inspect number of round 
  // waited for free radio channel
  AddOnP.CC2420Transmit -> CC2420TransmitC.CC2420Transmit;
  
  // to be able to access CC2420 packet metadata - inspect waiting for free channel
  components CC2420PacketC;
  AddOnP.CC2420PacketBody -> CC2420PacketC;
  AddOnP.CC2420Packet -> CC2420PacketC;
  
  // statistics manager is wired to this addon 
  components StatManC;
  AddOnP.StatMan -> StatManC.StatMan;
  
  // provides wiring
  AddOn = AddOnP.AddOn;
}
