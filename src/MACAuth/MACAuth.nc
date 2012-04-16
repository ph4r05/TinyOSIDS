/** 
 * CBCMAC interface 
 * able to detect auth failure with this interface.
 * 
 * Normaly when CBC-MAC is choosen, no component checks authenticity of given packet
 * According to CC2420 datasheet and 802.15.4 standard def. page 177 MIC(=message integrity code)
 * is appended to payload data. Length of payload has to contain MIC length. CC2420ReceiveP.nc
 * strips MIC and does not take care about any authenticity of message.
 * 
 * According to CC2420 received message requiring auth test (FCF field, secHeader),
 * last byte of MIC is 0x0 <=> Auth test passed, 0xff otherwise
 */

interface MACAuth {
  
  /**
   * Returns TRUE <=> message is using MAC (CBCMAC, CCM).
   * CC2420_HW_SECURITY has to be defined, otherwise FALSE is returned;
   * 
   * Decision is made according to secMode in secHeader.
   * 
   * @param msg NON-NULL pointer to message
   */
  command bool isUsingMac(message_t * ONE msg);
  
  /**
   * Returns length of MIC code length - derived from  secMode in secHeader.
   * Returns 0 if security is disabled or usingMac==false
   * 
   * @param msg NON-NULL pointer to message
   */
  command uint8_t getMICLength(message_t * ONE msg);
  
  /**
   * Returns TRUE if message is using MAC and there is any space  
   * 
   * @param msg NON-NULL pointer to message
   * @param payload pointer to payload
   * @param length 	payload length as returned from receive() function
   */
  command bool isMICPresent(message_t * ONE msg, void * ONE payload, uint8_t length);
  
  /**
   * Returns FALSE IF: 
   * message is using MAC, MIC is not present OR Auth failed according to CC2420 datasheet
   * 
   * @param msg NON-NULL pointer to message
   * @param payload pointer to payload
   * @param length 	payload length as returned from receive() function
   */
  command bool isAuthentic(message_t * ONE msg, void * ONE payload, uint8_t length);
  
  /**
   * Returns FALSE IF: 
   * message is using MAC, MIC is not present OR Auth failed according to CC2420 datasheet.
   * WORKS only if cc2420_metadata_t is changed to contain new field:
   * bool authenticated
   * 
   * Is better since another layer down the hierarchy could wrap our payload inside another thus
   * there is not inevitably MIC right after payload (may be another footer of down layer)
   * 
   * @param msg NON-NULL pointer to message
   */
  command bool isAuthenticUsingFlag(message_t * ONE msg, void * ONE_NOK payload, uint8_t length);

}
