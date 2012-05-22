/**
 * The basic addon interface.
 *
 * @author Ph4r05
 */ 

#include <TinyError.h>
#include <message.h>

interface StatMan {
   command void test();
//  event message_t* receive(message_t* msg, void* payload, uint8_t len);

   /**
    * Here should be event listeners according to IDS specification:
    *  - when message is sent, notify stat manager about CCA waiting time for free radio channel
    *  - notify when packet with invalid CRC is received 
    * 		global - count only invalid CRC packets
    * 		local  - count invalid CRC packets for particular sending node id (this info can be invalid too, but we accept that risk)
    *  - notify when packet with bad authentication result is received
    *  - notify about every snooped message - watchdog monitoring
    *  - collect CCA state info periodically
    *   
    * 
    */
}

