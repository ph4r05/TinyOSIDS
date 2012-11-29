#!/bin/bash
mig java -target=telosb -java-classname=TestSerialMsg Messages4mig.h test_serial_msg -o TestSerialMsg.java
mig java -target=telosb -java-classname=TimeSyncReportMsg Messages4mig.h timeSyncReport -o TimeSyncReportMsg.java
