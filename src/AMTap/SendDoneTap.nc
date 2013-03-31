/** 
 * SendDoneTap interface. 
 * 
 * This interface is provided by ForgedActiveMessageP and used by Statistics Manager.
 * Using this interface the Statistics Manager hears all processed messages
 * 
 * This is used to capture SendDone() calls of lower layer to obtain carrier sensing times
 * of sent or failed messages. 
 */

interface SendDoneTap {
  
  /**
   * SendDone event
   *  
   * @param type am_type of message received
   * @param msg pointer to message received
   * @param error error status
   */
  event void sendDone(uint8_t type, message_t* msg, error_t error);
}