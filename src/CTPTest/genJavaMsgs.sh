#!/bin/bash
mig java -target=telosb -java-classname=CommandMsg application_mig.h CommandMsg -o java/CommandMsg.java
mig java -target=telosb -java-classname=IdentifyMsg application_mig.h IdentifyMsg -o java/CIdentifyMsg.java
mig java -target=telosb -java-classname=CtpSendRequestMsg application_mig.h CtpSendRequestMsg -o java/CCtpSendRequestMsg.java
mig java -target=telosb -java-classname=CtpResponseMsg application_mig.h CtpResponseMsg -o java/CCtpResponseMsg.java
mig java -target=telosb -java-classname=CtpReportDataMsg application_mig.h CtpReportDataMsg -o java/CCtpReportDataMsg.java
mig java -target=telosb -java-classname=CollectionDebugMsg application_mig.h CollectionDebugMsg -o java/CCollectionDebugMsg.java
mig java -target=telosb -java-classname=CtpInfoMsg application_mig.h CtpInfoMsg -o java/CCtpInfoMsg.java
#mig java -target=telosb -java-classname=TimeSyncMsg application_mig.h TimeSyncMsg -o java/CTimeSyncMsg.java
