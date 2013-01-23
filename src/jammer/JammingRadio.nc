/**
 *
 */
interface JammingRadio {

  /**
   * Enables/disabled radio jamming
   *
   * @param enabled enabled
   * @return void
   */
  command void setJamming(bool enabled);
  
  /**
   * Enables/disabled radio jamming
   *
   * @param enabled enabled
   * @param tx_power transmit power to use
   * @return void
   */
  command void setJammingTX(bool enabled, uint8_t tx_power);
  
  /**
   * Enables/disabled radio jamming
   *
   * @param tx_power transmit power to use
   * @param msg message to send
   * @param dst message destination
   * @return void
   */
  command void setJammingMsgTX(uint8_t tx_power, message_t * msg, am_addr_t dst);

  /**
   * Timeout after message was sent
   */
  command void setJammingTimeout(uint16_t timeout);
}