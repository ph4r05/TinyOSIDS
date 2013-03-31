/**
 * Implements attacks on forward layer of CTP.
 *  - Packet delay attack
 *      - flat packet delay attack - set number of milliseconds that ALL
 *      packets to be forwarded will be delayed
 *      - callback packet delay attack, decision about delay could be made
 *      based packet content - event is signalized.
 *  - Selective forwarder attack
 *      - flat packet dropping. User sets probability that packet will be dropped.
 *      - callback packet dropping can be easily implemented by wiring to Intercept 
 *          interface of CtpForwardingEngine. 
 *      based on packet content
 */
interface CtpAttacker {
	/**
	 * Enables flat packet delaying - each packet will be delayed by specified
	 * amount of time.
	 * 
	 * @param type 
	 *         0 = disabled
	 *         1 = flat (for all)
	 *         2 = callback (with signalizing event)
	 * @param milli - number of milliseconds that each packet to be forwarded
	 *     should be delayed
	 */
   command error_t enablePacketDelay(uint8_t type, uint16_t milli);
   
  /**
   * Signals that a message has been received, which is supposed to be
   * forwarded to another destination. 
   *
   * @param 'message_t* ONE msg' The complete message received.
   *
   * @param 'void* COUNT(len) payload' The payload portion of the packet for this
   * protocol layer.
   *
   * @param len The length of the payload buffer.
   *
   * @return TRUE indicates the packet should be forwarded, FALSE
   * indicates that it should not be forwarded.
   *
   */
  event bool attackPacketDelayCallback(message_t* msg, void* payload, uint8_t len, am_id_t type);
   
   /**
    * Disables flat packet delaying. Packets will not be subject to delay anymore.
    */
   command error_t disablePacketDelay();
   
   /**
    * Determines current state of packet delaying attack.
    */
   command uint8_t getAttackPacketDelayType(); 

  /**
   * Signals that a message has been received, which is supposed to be
   * forwarded to another destination. User logic should decide whether this packet
   * should be subject to probabilistic dropping or not.
   *
   * @param 'message_t* ONE msg' The complete message received.
   *
   * @param 'void* COUNT(len) payload' The payload portion of the packet for this
   * protocol layer.
   *
   * @param len The length of the payload buffer.
   *
   * @return TRUE indicates the packet should be considered for dropping, 
   *    FALSE means that this packet should be forwarder normally.
   *
   */
    event bool attackPacketDropCallback(message_t* msg, void* payload, uint8_t len, am_id_t type);

    /**
     * Enabled packet dropping attack. 
     * Each incoming packet will be dropped with probability p.
     * @param p
     */
   command error_t enablePacketDropping(uint8_t type, float p);
   
   /**
    * Disables flat packet dropping attack.
    * If was flat dropping attack enabled, after this call packets won't
    * be subject to packet dropping with (specified probability) anymore.
    */
   command error_t disablePacketDropping();
   
   /**
    * Determines state of flat packet dropping attack.
    */
   command uint8_t getPacketDroppingType();    
}
