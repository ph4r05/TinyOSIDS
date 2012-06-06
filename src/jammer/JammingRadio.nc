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
   * Timeout after message was sent
   */
  command void setJammingTimeout(uint16_t timeout);
}