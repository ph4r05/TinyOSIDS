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
   * @return void
   */
  command void setJammingTX(bool enabled, uint8_t tx_power);

  /**
   * Timeout after message was sent
   */
  command void setJammingTimeout(uint16_t timeout);
}