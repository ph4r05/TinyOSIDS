/**
 * @author Dusan (ph4r05) Klinec
 */

module MACAuthP
{
	provides interface MACAuth;
	uses interface CC2420Packet;
	uses interface CC2420PacketBody;
}
implementation
{
	bool isUsingMac(message_t * ONE msg){
#ifndef CC2420_HW_SECURITY
		return FALSE;
#else
	// obtain security header
	security_header_t secHdr = (call CC2420PacketBody.getHeader(msg))->secHdr;
	
	if (secHdr.secLevel == CBC_MAC_4 
		|| secHdr.secLevel == CBC_MAC_8
		|| secHdr.secLevel == CBC_MAC_16
		|| secHdr.secLevel == CCM_4
		|| secHdr.secLevel == CCM_8
		|| secHdr.secLevel == CCM_16){
			return TRUE;
		} else {
			return FALSE;
		}
#endif
	}
	

	

	/**
	 * Real function providing this info
	 */
	uint8_t getMICLength(message_t * ONE msg){
#ifndef CC2420_HW_SECURITY
		return 0;
#else
	// obtain security header
	security_header_t secHdr = (call CC2420PacketBody.getHeader(msg))->secHdr;
	
	if (secHdr.secLevel == NO_SEC){
	  return 0;
	}else if (secHdr.secLevel == CBC_MAC_4){
	  return 4;
	}else if (secHdr.secLevel == CBC_MAC_8){
	  return 8;
	}else if (secHdr.secLevel == CBC_MAC_16){
	  return 16;
	}else if (secHdr.secLevel == CTR){
	  return 0;
	}else if (secHdr.secLevel == CCM_4){
	  return 4;
	}else if (secHdr.secLevel == CCM_8){
	  return 8;
	}else if (secHdr.secLevel == CCM_16){
	  return 16;
	} else {
	  return 0;
	}
#endif
	}

	command bool MACAuth.isUsingMac(message_t * ONE msg){
		return isUsingMac(msg);
	}

	command uint8_t MACAuth.getMICLength(message_t *msg){
		return getMICLength(msg);
	}

	command bool MACAuth.isMICPresent(message_t *msg, void *payload, uint8_t length){
#ifndef CC2420_HW_SECURITY
		return FALSE;
#else
	// obtain security header
	security_header_t secHdr = (call CC2420PacketBody.getHeader(msg))->secHdr;
	uint8_t micLength = getMICLength(msg);
	cc2420_metadata_t * metaPtr = NULL;
	
	if (micLength==0){
		return FALSE;
	}
	
	// get address of metadata
	metaPtr = call CC2420PacketBody.getMetadata(msg);
	
	// this assertion should hold -> MIC is between payload and metadata
	if (payload+length+micLength <= ((void*)metaPtr)){
		return TRUE;
	} else {
		return FALSE;
	}
#endif
	}

	command bool MACAuth.isAuthentic(message_t *msg, void *payload, uint8_t length){
		// TODO Auto-generated method stub
	}
}
