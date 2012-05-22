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

	void * ONE_NOK getLastMICByte(message_t * ONE msg, void * ONE payload, uint8_t length){
#ifndef CC2420_HW_SECURITY
		return 0;
#else
	// obtain security header
	security_header_t secHdr = (call CC2420PacketBody.getHeader(msg))->secHdr;
	uint8_t micLength = getMICLength(msg);
	cc2420_metadata_t * metaPtr = NULL;
	
	if (micLength==0){
		return 0;
	}
	
	// get address of metadata
	metaPtr = call CC2420PacketBody.getMetadata(msg);
	
	// this assertion should hold -> MIC is between payload and metadata
	if (payload+length+micLength <= ((void*)metaPtr)){
		return (payload+length+micLength-1);
	} else {
		return 0;
	}
#endif
	}

	command bool MACAuth.isUsingMac(message_t * ONE msg){
		return isUsingMac(msg);
	}

	command uint8_t MACAuth.getMICLength(message_t * ONE msg){
		return getMICLength(msg);
	}

	command bool MACAuth.isMICPresent(message_t * ONE msg, void * ONE payload, uint8_t length){
#ifndef CC2420_HW_SECURITY
		return FALSE;
#else
		void * micByte = getLastMICByte(msg, payload, length);
		return micByte != 0;
#endif
	}

	command bool MACAuth.isAuthentic(message_t * ONE msg, void * ONE payload, uint8_t length){
#ifndef CC2420_HW_SECURITY
		return TRUE;
#else
		if (isUsingMac(msg) && getMICLength(msg)>0){
			void * micByte = getLastMICByte(msg, payload, length);
			return (*((uint8_t * )micByte)) == 0x0;
		} else {
			return TRUE;
		}
#endif
	}

	command bool MACAuth.isAuthenticUsingFlag(message_t * ONE msg, void * ONE_NOK payload, uint8_t length){
#ifndef CC2420_HW_SECURITY
		return TRUE;
#elif defined(CC2420_METADATA_EXTENDED) && defined(CC2420_HW_SECURITY) 
		// we have our extra "authentic" boolean defined here
		// get metadata and return flag directly
		return (call CC2420PacketBody.getMetadata(msg))->authentic;
#elif defined(CC2420_HW_SECURITY) && !defined(CC2420_METADATA_EXTENDED)
		if(payload!=NULL){
			return call MACAuth.isAuthentic(msg, payload, length);
		} else {
		 	return FALSE;
		}
#endif		
	}
}
