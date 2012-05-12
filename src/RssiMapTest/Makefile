COMPONENT=RssiBaseAppC

INCLUDES= -I..               		\
          -I../Reset		 		\
          -I../SerialQueueSender	

CFLAGS += $(INCLUDES)

# disable the hardware address recognition - needed for snooping...
CFLAGS+=-DCC2420_NO_ADDRESS_RECOGNITION

ifneq ($(filter iris,$(MAKECMDGOALS)),) 
	CFLAGS += -DRF230_RSSI_ENERGY
endif

# increase tosh data length
CFLAGS += -DTOSH_DATA_LENGTH=34

include $(MAKERULES)
