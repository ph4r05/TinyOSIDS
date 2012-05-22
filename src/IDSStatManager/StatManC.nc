
configuration StatManC {
	provides {
		interface StatMan;
	}
}
implementation {
  // main components for reset, bootup, leds	
  components MainC, LedsC;
  
  // active message components
  components ActiveMessageC as Radio, SerialActiveMessageC as Serial;
  
  // main addon component
  components AddOnC;
  
  // statman 
  components StatManP;
  
  // HW reset
  components ResetC;
  StatManP.Reset -> ResetC;
  
  // provides wiring
  StatMan = StatManP.StatMan;
}
