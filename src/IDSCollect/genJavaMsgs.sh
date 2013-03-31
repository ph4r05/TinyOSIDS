#!/bin/bash
mig java -target=telosb -java-classname=CommandMsg application_mig.h CommandMsg -o java/CommandMsg.java
mig java -target=telosb -java-classname=IdentifyMsg application_mig.h IdentifyMsg -o java/IdentifyMsg.java
mig java -target=telosb -java-classname=CtpSendRequestMsg application_mig.h CtpSendRequestMsg -o java/CtpSendRequestMsg.java
mig java -target=telosb -java-classname=CtpResponseMsg application_mig.h CtpResponseMsg -o java/CtpResponseMsg.java
mig java -target=telosb -java-classname=CtpReportDataMsg application_mig.h CtpReportDataMsg -o java/CtpReportDataMsg.java
mig java -target=telosb -java-classname=CollectionDebugMsg application_mig.h CollectionDebugMsg -o java/CollectionDebugMsg.java
mig java -target=telosb -java-classname=CtpInfoMsg application_mig.h CtpInfoMsg -o java/CtpInfoMsg.java
#mig java -target=telosb -java-classname=TimeSyncMsg application_mig.h TimeSyncMsg -o java/TimeSyncMsg.java
