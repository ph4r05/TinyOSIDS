COMPONENT=TimeTestC

##INCLUDES= -I..               		\

INCLUDES= -I../Reset		 		\
          -I$(TOSDIR)/lib/net 		\
          -I$(TOSDIR)/lib/net/le	\
          -I$(TOSDIR)/lib/net/ctp	
#          \
#          -I$(TOSDIR)/lib/net/lqi

CFLAGS += $(INCLUDES)

# disable the hardware address recognition - needed for snooping...
#CFLAGS+=-DCC2420_NO_ADDRESS_RECOGNITION
CFLAGS+=-DCC2420_CHANNEL=17

#ifneq ($(filter iris,$(MAKECMDGOALS)),) 
#	CFLAGS += -DRF230_RSSI_ENERGY
#endif

# increase tosh data length
#CFLAGS += -DTOSH_DATA_LENGTH=34

include $(MAKERULES)
