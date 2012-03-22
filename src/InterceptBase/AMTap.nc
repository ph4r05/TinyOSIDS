/** 
 * AMTap interface. 
 * 
 * This interface is provided by ForgedActiveMessageP and used by Statistics Manager.
 * Using this interface the Statistics Manager hears all processed messages 
 */

interface AMTap {
  
  /* Packet receive */
  event message_t* receive(uint8_t type, message_t* msg, void *payload, uint8_t len);
  
  /* Snoop */
  event message_t* snoop(uint8_t type, message_t* msg, void *payload, uint8_t len);
 
  /* Send */
  event message_t* send(uint8_t type, message_t* msg, uint8_t len);

}