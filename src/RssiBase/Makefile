COMPONENT=RssiBaseAppC

INCLUDES= -I..               \
          -I../InterceptBase \
          -I../Reset

CFLAGS += $(INCLUDES)

# disable the hardware address recognition - needed for snooping...
#CFLAGS+=-DCC2420_NO_ADDRESS_RECOGNITION

ifneq ($(filter iris,$(MAKECMDGOALS)),) 
	CFLAGS += -DRF230_RSSI_ENERGY
endif

include $(MAKERULES)
