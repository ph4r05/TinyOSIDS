/**
 * Implements attacks on forward layer of CTP.
 *  - Packet delay attack
 *      - flat packet delay attack - set number of milliseconds that ALL
 *      packets to be forwarded will be delayed
 *      - TODO: callback packet delay attack, decision about delay could be made
 *      based packet content
 *  - Selective forwarder attack
 *      - flat packet dropping. User sets probability that packet will be dropped.
 *      - TODO: callback packet dropping, decision about packet drop could be made
 *      based on packet content
 */
interface ForwarderAttacker {
	/**
	 * Enables flat packet delaying - each packet will be delayed by specified
	 * amount of time.
	 * 
	 * @param milli - number of milliseconds that each packet to be forwarded
	 *     should be delayed
	 */
   command error_t enableFlatPacketDelay(uint16_t milli);
   
   /**
    * Disables flat packet delaying. Packets will not be subject to delay anymore.
    */
   command error_t disableFlatPacketDelay();
   
   /**
    * Determines current state of packet delaying attack.
    */
   command bool isFlatPacketDelayEnabled();


    /**
     * Enabled packet dropping attack. 
     * Each incoming packet will be dropped with probability p.
     * @param p
     */
   command error_t enableFlatPacketDropping(float p);
   
   /**
    * Disables flat packet dropping attack.
    * If was flat dropping attack enabled, after this call packets won't
    * be subject to packet dropping with (specified probability) anymore.
    */
   command error_t disableFlatPacketDropping();
   
   /**
    * Determines state of flat packet dropping attack.
    */
   command bool isFlatPacketDroppingEnabled();    
}
