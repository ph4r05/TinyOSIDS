#include "statman.h"
module StatManP @safe() {
    uses
    {
        interface Boot;
        interface Reset as Reset;
        interface Leds;
    }
	
	provides {
		interface StatMan;
    }
}

implementation
{  
    // event handler, on system boot
    // perform init tasks
    // prepare queues, starts interfaces
    event void Boot.booted() {
        
    }
    
    command void StatMan.test(){
    	
    }
}
