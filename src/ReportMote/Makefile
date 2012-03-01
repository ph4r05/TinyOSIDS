COMPONENT=SendingMoteAppC
CFLAGS += -DCC2420_NO_ACKNOWLEDGEMENTS
CFLAGS += -DCC2420_NO_ADDRESS_RECOGNITION

INCLUDES= -I..

CFLAGS += $(INCLUDES)

# RSSI_ENERGY for iris motes
ifneq ($(filter iris,$(MAKECMDGOALS)),) 
#	CFLAGS += -DRF230_RSSI_ENERGY
endif

# include own platform.h with initialization redefinition
# for IRIS nodes to perform correct restart
-include platform.h

include $(MAKERULES)
