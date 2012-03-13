interface InterceptBaseConfig {

  /**
   * Sync configuration changes with the radio hardware. This only
   * applies to set commands below.
   *
   * @return SUCCESS if the request was accepted, FAIL otherwise.
   */
  command error_t sync();
  event void syncDone( error_t error );

  /**
   * Global radio filtering
   */
  command bool getGlobalRadioFilteringEnabled();
  command void setGlobalRadioFilteringEnabled( bool enabled );
  
  /**
   * Global serial filtering
   */
  command bool getGlobalSerialFilteringEnabled();
  command void setGlobalSerialFilteringEnabled( bool enabled );  

  /**
   * Default radio filtering
   */
  command bool getDefaultRadioFilteringEnabled();
  command void setDefaultRadioFilteringEnabled( bool enabled );
  
  /**
   * Default serial filtering
   */
  command bool getDefaultSerialFilteringEnabled();
  command void setDefaultSerialFilteringEnabled( bool enabled ); 

  /**
   * Address recognition - helps to snoop/sniff foreign packets... 
   */
  async command bool getAddressRecognitionEnabled();
  command void setAddressRecognitionEnabled( bool enabled ); 

  /**
   * Radio snooping - forward packets from snoop interface to queue
   */
  async command bool getRadioSnoopEnabled();
  command void setRadioSnoopEnabled( bool enabled );
  
  command uint8_t getRadioQueueFree();
  command uint8_t getSerialQueueFree();
  command uint8_t getSerialFailed(); 

  /**
   * Change the short address of the radio.                                                                                                                                               
   * 
  async command uint16_t getShortAddr();
  command void setShortAddr( uint16_t address );
  */
}