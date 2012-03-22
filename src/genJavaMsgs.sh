#!/bin/bash
mig java -target=telosb -java-classname=CommandMsg RssiDemoMessages.h CommandMsg -o CommandMsg.java
mig java -target=telosb -java-classname=MultiPingMsg RssiDemoMessages.h MultiPingMsg -o MultiPingMsg.java
mig java -target=telosb -java-classname=MultiPingResponseMsg RssiDemoMessages.h MultiPingResponseMsg -o MultiPingResponseMsg.java
mig java -target=telosb -java-classname=MultiPingResponseReportMsg RssiDemoMessages.h MultiPingResponseReportMsg -o MultiPingResponseReportMsg.java
mig java -target=telosb -java-classname=NoiseFloorReadingMsg RssiDemoMessages.h NoiseFloorReadingMsg -o NoiseFloorReadingMsg.java
mig java -target=telosb -java-classname=PingMsg RssiDemoMessages.h PingMsg -o PingMsg.java
mig java -target=telosb -java-classname=RssiMsg RssiDemoMessages.h RssiMsg -o RssiMsg.java
mig java -target=telosb -java-classname=IdentifyMsg RssiDemoMessages.h IdentifyMsg -o IdentifyMsg.java
mig java -target=telosb -java-classname=CtpSendRequestMsg RssiDemoMessages.h CtpSendRequestMsg -o CtpSendRequestMsg.java
mig java -target=telosb -java-classname=CtpResponseMsg RssiDemoMessages.h CtpResponseMsg -o CtpResponseMsg.java
mig java -target=telosb -java-classname=CtpReportDataMsg RssiDemoMessages.h CtpReportDataMsg -o CtpReportDataMsg.java
