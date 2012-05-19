COMPONENT=AddOnC

INCLUDES= -I../Reset		 		\
		  -I../AMTap				\
 		  -I../ObjQueue		 		\
          -I../SerialQueueSender	\
          -I../AMTapImpl			\
          -I../MACAuth				\
          -I../TOSCTP				\
          -I../TOSph4IDS		 				
#		  -I%T/lib/printf			\          

CFLAGS += $(INCLUDES)

# disable the hardware address recognition - needed for snooping...
CFLAGS+=-DCC2420_NO_ADDRESS_RECOGNITION

ifneq ($(filter iris,$(MAKECMDGOALS)),) 
	CFLAGS += -DRF230_RSSI_ENERGY
endif

# increase tosh data length
CFLAGS += -DTOSH_DATA_LENGTH=112

include $(MAKERULES)
