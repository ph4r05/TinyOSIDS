#!/bin/bash
mig java -target=telosb -java-classname=CommandMsg Messages4mig.h CommandMsg -o CommandMsg.java
mig java -target=telosb -java-classname=MultiPingMsg Messages4mig.h MultiPingMsg -o MultiPingMsg.java
mig java -target=telosb -java-classname=MultiPingResponseMsg Messages4mig.h MultiPingResponseMsg -o MultiPingResponseMsg.java
mig java -target=telosb -java-classname=MultiPingResponseReportMsg Messages4mig.h MultiPingResponseReportMsg -o MultiPingResponseReportMsg.java
mig java -target=telosb -java-classname=NoiseFloorReadingMsg Messages4mig.h NoiseFloorReadingMsg -o NoiseFloorReadingMsg.java
mig java -target=telosb -java-classname=PingMsg Messages4mig.h PingMsg -o PingMsg.java
mig java -target=telosb -java-classname=RssiMsg Messages4mig.h RssiMsg -o RssiMsg.java
mig java -target=telosb -java-classname=IdentifyMsg Messages4mig.h IdentifyMsg -o IdentifyMsg.java
mig java -target=telosb -java-classname=CtpSendRequestMsg Messages4mig.h CtpSendRequestMsg -o CtpSendRequestMsg.java
mig java -target=telosb -java-classname=CtpResponseMsg Messages4mig.h CtpResponseMsg -o CtpResponseMsg.java
mig java -target=telosb -java-classname=CtpReportDataMsg Messages4mig.h CtpReportDataMsg -o CtpReportDataMsg.java
mig java -target=telosb -java-classname=CollectionDebugMsg Messages4mig.h CollectionDebugMsg -o CollectionDebugMsg.java
mig java -target=telosb -java-classname=CtpInfoMsg Messages4mig.h CtpInfoMsg -o CtpInfoMsg.java
mig java -target=telosb -java-classname=PrintfMsg Messages4mig.h printf_msg -o PrintfMsg.java
mig java -target=telosb -java-classname=LowlvlTimeSyncMsg32 Messages4mig.h LowlvlTimeSyncMsg32 -o LowlvlTimeSyncMsg32.java
mig java -target=telosb -java-classname=LowlvlTimeSyncMsg64 Messages4mig.h LowlvlTimeSyncMsg64 -o LowlvlTimeSyncMsg64.java
