/*
 * Copyright (c) 2008 Dimas Abreu Dutra
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
 * @author Dimas Abreu Dutra
 */

#include "../RssiDemoMessages.h"
#include "message.h"
#include "AM.h"

#if defined(PLATFORM_TELOSB)
#include "CC2420.h"

#elif defined(PLATFORM_IRIS)
#include "RF230Radio.h"

//#else
//#error "Platform unknown"
#endif


//Defining the preprocessor variable CC2420_NO_ACKNOWLEDGEMENTS will disable all forms of acknowledgments at compile time.
//Defining the preprocessor variable CC2420_HW_ACKNOWLEDGEMENTS will enable hardware acknowledgments and disable software acknowledgments.
#define CC2420_NO_ACKNOWLEDGEMENTS 1

configuration SendingMoteAppC {
} implementation {
// basic components
  components ActiveMessageC, MainC, LedsC;

  // own application
  components SendingMoteC as App;
  
// timers
  // window flush timer
  components new TimerMilliC() as FlushTimer;

  // send timer for repeated sending
  components new TimerMilliC() as SendTimer;

  // window flush gap timer
  components new TimerMilliC() as GapTimer;

  // signaling timer
  components new TimerMilliC() as SignalTimer;

  // noise floor timer
  components new TimerMilliC() as NoiseFloorTimer;

  // sensor reading timer
  components new TimerMilliC() as SensorReadingTimer;
  
  // sensor reading timer
  components new TimerMilliC() as BootupTimer;


// sender
  // response on ping.
  components new AMSenderC(AM_RSSIMSG) as RssiMsgSender;

  // response on multiping
  components new AMSenderC(AM_MULTIPINGRESPONSEMSG) as MultiPingResponseSender;

  // reporting message
  components new AMSenderC(AM_MULTIPINGRESPONSEREPORTMSG) as ReportMsgSender;

  // tiny report message
  components new AMSenderC(AM_MULTIPINGRESPONSETINYREPORTMSG) as TinyReportMsgSender;

  // command message, node can reply
  components new AMSenderC(AM_COMMANDMSG) as CommandMsgSender;
  
// receiver
  // very simple protocol for debugging
  components new AMReceiverC(AM_PINGMSG) as PingMsgReader;

  // ping from some node. ReportingNodes should has this enabled in further version
  // since BS may need to ping static nodes for some reason (network init, env probe)
  components new AMReceiverC(AM_MULTIPINGMSG) as MultiPingMsgReader;

  // response on which should reportNode react
  components new AMReceiverC(AM_MULTIPINGRESPONSEMSG) as ResponseMsgReader;

  // simple command protocol
  components new AMReceiverC(AM_COMMANDMSG) as CommandMsgReader;

// random
  // random generator for framing
  // collision avoidance on mass reporting.
  // If n reporting nodes has same window it would cause large peaks in bandwidth
  // during reporting
  components RandomC;

 // big queue for packet reports
 // components new BigQueueC(MultiPingResponseReportStruct, 2) as BigQueueA;

//
// RSSI readings wiring
//
#ifdef __CC2420_H__
  components CC2420ActiveMessageC;  
  App.CC2420Packet -> CC2420ActiveMessageC.CC2420Packet;
  
  // setting channel
  components CC2420RadioC;
  components CC2420ControlC;
  components CC2420ControlP;
  App -> CC2420ControlC.CC2420Config;
  App.ReadRssi -> CC2420ControlP;
#elif  defined(PLATFORM_IRIS)
  components RF230ActiveMessageC;
  App.PacketRSSI -> RF230ActiveMessageC.PacketRSSI;
  App.PacketTransmitPower -> RF230ActiveMessageC.PacketTransmitPower;
#elif defined(TDA5250_MESSAGE_H)
  components Tda5250ActiveMessageC;
  App -> Tda5250ActiveMessageC.Tda5250Packet;
#endif

//
// platform specific sensors
//
#if defined(PLATFORM_TELOSB)
	components new SensirionSht11C(), new HamamatsuS10871TsrC();
	App.Temperature -> SensirionSht11C.Temperature;
	App.Humidity -> SensirionSht11C.Humidity;
	App.Light -> HamamatsuS10871TsrC;
        
        // General IO controll
        components HplMsp430GeneralIOC;
        App.Pin0 ->HplMsp430GeneralIOC.Port60;
        App.Pin1 ->HplMsp430GeneralIOC.Port61;
#elif defined(PLATFORM_TYNDALL25)
	components new PhotoC() as Light;
	components new TempC() as Temperature;
	components new MicrophoneC() as Humidity;

	App.Temperature -> Temperature;
	App.Humidity -> Humidity;
	App.Light -> Light;
#elif defined(PLATFORM_IRIS)
        // no sensors for IRIS platform
#else
        //#error "No SENSORS defined for this platform"
#endif


// Generic wiring
  App.Boot -> MainC;
  App.Leds -> LedsC;
  App.RadioControl -> ActiveMessageC;

// randomC component wiring
  App.Random -> RandomC;

// timers wiring
  App.SendTimer -> SendTimer;
  App.GapTimer -> GapTimer;
  App.FlushTimer -> FlushTimer;
  App.SignalTimer -> SignalTimer;
  App.NoiseFloorTimer -> NoiseFloorTimer;
  App.SensorReadingTimer -> SensorReadingTimer;
  App.BootupTimer -> BootupTimer;

// sender wiring
  App.MultiPingResponseSend -> MultiPingResponseSender;
  App.RssiMsgSend -> RssiMsgSender;
  App.ReportMsgSend -> ReportMsgSender;
  App.TinyReportMsgSend -> TinyReportMsgSender;
  App.CommandMsgSend -> CommandMsgSender;
  
// reveicer wiring
  App.SimplePingReceive -> PingMsgReader;
  App.PingReceive -> MultiPingMsgReader;
  App.ResponseReceive -> ResponseMsgReader;
  App.CommandReceive -> CommandMsgReader;

// report packet
    App.Packet -> ReportMsgSender;
    App.AMPacket -> ReportMsgSender;

    //App.Reset -> ResetC;
// big queue for packet reports
//    App.ReportQueue -> BigQueueA;

    
// DEPRECATED below this line
///////////////////////////////////////////////////////////////////////////////

    /*
  App.RadioSend -> ActiveMessageC.AMSend;
*/
  //App.RadioReceive -> MsgReader;



//  App.Packet -> RssiMsgSender;
//  App.AMPacket -> RssiMsgSender;
//App.RadioReceive -> ActiveMessageC.Receive;
}
