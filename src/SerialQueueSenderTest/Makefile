COMPONENT=TestC

INCLUDES= -I../SerialQueueSender	\
          -I%T/lib/printf

CFLAGS += $(INCLUDES)

# increase tosh data length
CFLAGS += -DTOSH_DATA_LENGTH=64

include $(MAKERULES)
