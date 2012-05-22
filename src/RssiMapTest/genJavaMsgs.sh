#!/bin/bash
mig java -target=telosb -java-classname=CommandMsg application_mig.h CommandMsg -o java/CommandMsg.java
mig java -target=telosb -java-classname=MultiPingMsg application_mig.h MultiPingMsg -o java/MultiPingMsg.java
mig java -target=telosb -java-classname=MultiPingResponseMsg application_mig.h MultiPingResponseMsg -o java/MultiPingResponseMsg.java
mig java -target=telosb -java-classname=MultiPingResponseReportMsg application_mig.h MultiPingResponseReportMsg -o java/MultiPingResponseReportMsg.java
mig java -target=telosb -java-classname=NoiseFloorReadingMsg application_mig.h NoiseFloorReadingMsg -o java/NoiseFloorReadingMsg.java
mig java -target=telosb -java-classname=PingMsg application_mig.h PingMsg -o java/PingMsg.java
mig java -target=telosb -java-classname=RssiMsg application_mig.h RssiMsg -o java/RssiMsg.java
mig java -target=telosb -java-classname=IdentifyMsg application_mig.h IdentifyMsg -o java/IdentifyMsg.java
mig java -target=telosb -java-classname=PrintfMsg application_mig.h printf_msg -o java/PrintfMsg.java
