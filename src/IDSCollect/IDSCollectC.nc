/*
 * Copyright (c) 2012 
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the Stanford University nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL DIMAS ABREU
 * DUTRA OR HIS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author Dusan Klinec (ph4r05)
 */

#include "application.h"
#include "message.h"

configuration IDSCollectC {
} implementation {
  components IDSCollectP as App;
  
  // need to have this component to be able to work with commands send via serial
  components SerialActiveMessageC as Serial;
  components ActiveMessageC, MainC, LedsC;

  // init timer (radio init)
  components new TimerMilliC() as InitTimer;

  // keep alive send timer
  components new TimerMilliC() as AliveTimer;
  
  // dumps actual CTP structure to application
  components new TimerMilliC() as TreeTimer;
  
  // CCA sampling timer
  components new TimerMilliC() as CCATimer;
  
  components ResetC;
  
  /**************** NOISE FLOOR READING ****************/

#ifdef __CC2420_H__
  components CC2420ActiveMessageC;
  App -> CC2420ActiveMessageC.CC2420Packet; 
  
  components CC2420RadioC;
  components CC2420ControlC;
  components CC2420ControlP;
  components CC2420PacketC;
  components CC2420TransmitC;
  
  App.CC2420PacketBody -> CC2420PacketC.CC2420PacketBody;
  App.PacketTimeStamp32khz -> CC2420PacketC.PacketTimeStamp32khz;
  App.PacketTimeStampMilli -> CC2420PacketC.PacketTimeStampMilli;
  App -> CC2420ControlC.CC2420Config;
  
  App.EnergyIndicator -> CC2420TransmitC.EnergyIndicator;
  App.ByteIndicator -> CC2420TransmitC.ByteIndicator;  
#elif  defined(PLATFORM_IRIS)
  components  RF230ActiveMessageC, PacketField;
  App -> RF230ActiveMessageC.PacketRSSI;
#elif defined(TDA5250_MESSAGE_H)
  components Tda5250ActiveMessageC;
  App -> Tda5250ActiveMessageC.Tda5250Packet;
#endif
	
	
  /****************** COMMAND PROTOCOL ***************************/
  components new AMReceiverC(AM_COMMANDMSG) as CommandMsgReceiver;
  App.CommandMsgReceiver -> CommandMsgReceiver;
  
  components new AMSenderC(AM_COMMANDMSG) as CommandMsgSender;
  App.CommandSender -> CommandMsgSender;
  
  components new SerialAMReceiverC(AM_COMMANDMSG) as UartCommandMsgReceiver;
  App.UartCommandMsgReceiver -> UartCommandMsgReceiver;
  
  components new SerialAMSenderC(AM_COMMANDMSG) as UartCommandMsgSender;
  App.UartCommandSender -> UartCommandMsgSender;
	 
 
  // split controll notifiers
  App.RadioControl -> ActiveMessageC;
  App.SerialControl -> Serial;
  
  App.Boot -> MainC;
  App.InitTimer -> InitTimer;
  App.AliveTimer -> AliveTimer;
  App.TreeTimer -> TreeTimer;
  App.CCATimer -> CCATimer;
  
  //App.RadioControl -> ActiveMessageC;
  App.Leds -> LedsC;
  App.Packet -> ActiveMessageC;
  App.AMPacket -> ActiveMessageC;
  App.Acks -> ActiveMessageC;
  
  App.RadioPacket -> ActiveMessageC;
  App.RadioAMPacket -> ActiveMessageC;
  
  App.UartPacket -> Serial;
  App.UartAMPacket -> Serial;
  
  App.Reset -> ResetC;
  
  /** Serial sender for sending reports */
  components new SerialQueueSenderC(CtpReportDataMsg, 12, AM_CTPREPORTDATAMSG) as UartCtpReportDataSender;
  App.UartCtpReportDataSender -> UartCtpReportDataSender;
  
  components new SerialQueueSenderC(CtpInfoMsg, 6, AM_CTPINFOMSG) as UartCtpInfoMsgSender;
  App.UartCtpInfoMsgSender -> UartCtpInfoMsgSender;
  
  /**************** Collector ****************/   
    components CollectionC as Collector, new CollectionSenderC(AM_CTPRESPONSEMSG);
    
    App.ForwardingControl -> Collector.StdControl;
    App.RoutingInit -> Collector.RoutingInit;
    App.ForwardingInit -> Collector.ForwardingInit;
    App.LinkEstimatorInit -> Collector.LinkEstimatorInit;
    
    App.CtpSend -> CollectionSenderC;
    App.RootControl -> Collector;
  	App.CtpInfo -> Collector;
//    App.CtpCongestion -> Collector;
    App.CollectionPacket -> Collector;
    App.CtpReceive -> Collector.Receive[AM_CTPRESPONSEMSG];
    App.FixedTopology -> Collector.FixedTopology;
    App.CtpAttacker -> Collector.CtpAttacker;

    components RandomC;
    App.Random -> RandomC;
    
    // send timer for repeated sending of CTP messages
  	components new TimerMilliC() as CtpTimer;
  	App.CtpTimer -> CtpTimer;
  	
  	// intercept requests for CTP sending
  	components new SerialAMReceiverC(AM_CTPSENDREQUESTMSG) as UartCtpSendRequestReceiver;
  	App.UartCtpSendRequestReceiver -> UartCtpSendRequestReceiver;
  	
  	// tapping interface from forged message
  	// Since BaseStation does not support radioSending correctly now, we need
  	// to hook send() calls by this way. Needed to set TX power for some messages
  	components ForgedActiveMessageC as FAM;
  	App.AMTapForg -> FAM.AMTap;
  	
  	// LOGGER DISABLED TEMPORARILY
  	// Logger produces intensive data streams, if everybody is set to listen to 
  	// everything (snooping, addressDetection=false), it can cause UART queues to 
  	// overflow very quickly, since debug message can be send plenty times for 
  	// only 1 CTP message from 1 node (approx. each routing decision) 
  	App.ForwardControl -> Collector.ForwardControl;
  	
  	// implement debug to see real CTP behavior and routing decisions
  components UARTDebugSenderP as LoggerC;
  components new SerialAMSenderC(AM_CTP_DEBUG) as UartCtpDebugSender;
	LoggerC.UARTSend -> UartCtpDebugSender;
	LoggerC.Boot -> MainC;
	
	components new PoolC(message_t, 5) as CTPDbgPool;
	components new QueueC(message_t*, 5) as CTPDbgQueue;
	LoggerC.SendQueue -> CTPDbgQueue;
	LoggerC.MessagePool -> CTPDbgPool;  	
    Collector.CollectionDebug -> LoggerC;
    App.CtpLoggerControl -> LoggerC.StdControl;
    App.CtpLogger -> LoggerC.CollectionDebug;
    
    //
    // Local Timers
    //
    components Counter32khz32C, new CounterToLocalTimeC(T32khz);
    CounterToLocalTimeC.Counter -> Counter32khz32C;
    App.LocalTime32khz -> CounterToLocalTimeC;                                                                                                                                                        
    
    //DummyTimer is introduced to compile apps that use no timers
    components HilTimerMilliC, new TimerMilliC() as DummyTimer;
    App.LocalTimeMilli -> HilTimerMilliC;
    
}
