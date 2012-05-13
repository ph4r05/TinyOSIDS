/** 
 * AMTap interface. 
 * 
 * This interface is provided by ForgedActiveMessageP and used by Statistics Manager.
 * Using this interface the Statistics Manager hears all processed messages 
 */

interface AMTap {
  
  /**
   * Packet received event
   * Should be called BEFORE any other functions reacting on message received event,
   * just after message received - to get raw data to receive engine
   * 
   * If NULL returned, packet SHOULD be ignored as if no packet was received
   * 
   * @param type am_type of message received
   * @param msg pointer to message received
   * @param payload pointer to payload data set by receiving layer
   * @param len length of payload block in bytes
   */
  event message_t* receive(uint8_t type, message_t* msg, void *payload, uint8_t len);
  
  /**
   * Packet snoop event
   * Should be called BEFORE any other functions reacting on message received event,
   * just after message received - to get raw data to receive engine
   * 
   * If NULL returned, packet SHOULD be ignored as if no packet was received
   * 
   * @param type am_type of message received
   * @param msg pointer to message received
   * @param payload pointer to payload data set by receiving layer
   * @param len length of payload block in bytes
   */
  event message_t* snoop(uint8_t type, message_t* msg, void *payload, uint8_t len);
 
  /**
   * Event just before packet sending
   * 
   * If NULL is returned, packet SHOULD be ignored - pretend SENDING but throw away
   * 
   * @param type am_type of message
   * @param msg pointer to message
   * @param len length of payload block in bytes
   */
  event message_t* send(uint8_t type, message_t* msg, uint8_t len);

}