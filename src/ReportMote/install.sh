#!/bin/bash
if [ ! "$#" -eq 2 ]; then
	echo "Usage: $0 TOS_NODE_ID usbNodeID"
	exit
fi

	     sudo chmod 666 "/dev/ttyUSB$2" && \
	     make telosb "install,$1" "bsl,/dev/ttyUSB$2"

