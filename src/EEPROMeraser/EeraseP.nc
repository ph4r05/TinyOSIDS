/**
 * Application to test that the TinyOS java toolchain can communicate
 * with motes over the serial port. 
 *
 *  @author Gilman Tolle
 *  @author Philip Levis
 *  
 *  @date   Aug 12 2005
 *
 **/

#ifdef TESTDEBUG 
#include "printf.h"
#endif
module EeraseP {
  uses {
    interface Leds;
    interface Boot;
    
    interface BlockRead;
    interface BlockWrite; 
  }
}
implementation {
  enum {
	TEST=1
  };

  event void Boot.booted() {
    if (call BlockWrite.erase() != SUCCESS){
        call Leds.led2On();
    } else {
    	call Leds.led1On();
    }
  }
  
  event void BlockRead.readDone(storage_addr_t addr, void *buf, storage_len_t len, error_t error){
  	
    }

    event void BlockRead.computeCrcDone(storage_addr_t addr, storage_len_t len, uint16_t crc, error_t error){
            // TODO Auto-generated method stub
    }

    event void BlockWrite.writeDone(storage_addr_t addr, void *buf, storage_len_t len, error_t error){
            // TODO Auto-generated method stub
    }

    event void BlockWrite.eraseDone(error_t error){
            if (error==SUCCESS){
            	call Leds.led1On();
            } else {
            	call Leds.led2On();
            }
    }

    event void BlockWrite.syncDone(error_t error){
            // TODO Auto-generated method stub
    }
  
}
