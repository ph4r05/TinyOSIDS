/**
 * The basic addon interface.
 *
 * @author Ph4r05
 */ 

#include <TinyError.h>
#include <message.h>

interface AddOn {
   command void test();
//  event message_t* receive(message_t* msg, void* payload, uint8_t len);
  
}

