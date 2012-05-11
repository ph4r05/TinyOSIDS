#!/bin/bash
sudo /etc/init.d/gpsd stop && \
	     sudo chmod 666 "/dev/ttyUSB$2" && \
	     make telosb "install,$1" "bsl,/dev/ttyUSB$2"

