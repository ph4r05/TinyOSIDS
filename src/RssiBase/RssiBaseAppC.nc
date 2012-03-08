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
#include "Reset.h"

configuration RssiBaseAppC {
} implementation {
  components BaseStationC;
  components RssiBaseC as App;
  
  // need to have this component to be able to work with commands send via serial
  components SerialActiveMessageC as Serial;
  
  components ActiveMessageC, MainC, LedsC;  
  components new AMSenderC(AM_MULTIPINGRESPONSEMSG) as PingMsgSender;
  //components new AMSenderC(AM_PINGMSG) as PingMsgSender;
  // RSSI report send timer
  components new TimerMilliC() as SendTimer;
  // keep alive send timer
  components new TimerMilliC() as AliveTimer;
  // send timer for repeated sending of pings
  components new TimerMilliC() as PingTimer;
  
  // RSSI reading queue
  components new QueueC(MultiPingResponseReportStruct_t, RSSI_QUEUE_LEN) as RSSIQueue;
  
  components ResetC;
  
  /**************** NOISE FLOOR READING ****************/
  // repeatedly noise floor reading timer
  components new TimerMilliC() as NoiseFloorTimer;

#ifdef __CC2420_H__
  components CC2420ActiveMessageC;
  App -> CC2420ActiveMessageC.CC2420Packet;
  
   // setting channel
  components CC2420RadioC;
  components CC2420ControlC;
  components CC2420ControlP;
  App -> CC2420ControlC.CC2420Config;
  App.ReadRssi -> CC2420ControlP;
#elif  defined(PLATFORM_IRIS)
  components  RF230ActiveMessageC, PacketField;
  App -> RF230ActiveMessageC.PacketRSSI;
#elif defined(TDA5250_MESSAGE_H)
  components Tda5250ActiveMessageC;
  App -> Tda5250ActiveMessageC.Tda5250Packet;
#endif
	
// interceptors for radio messages - may be processed
  App.SimpleRssiMsgIntercept -> BaseStationC.RadioIntercept[AM_RSSIMSG];
  App.CommandMsgIntercept -> BaseStationC.RadioIntercept[AM_COMMANDMSG];
  App.RssiMsgIntercept->BaseStationC.RadioIntercept[AM_MULTIPINGRESPONSEMSG];
  
// MultiPingRequest intercept from radio
  App.MultiPingRadioIntercept -> BaseStationC.RadioIntercept[AM_MULTIPINGMSG];
// MultiPingRequest intercept from serial
  App.MultiPingSerialIntercept -> BaseStationC.SerialIntercept[AM_MULTIPINGMSG];
  
// serial interceptors - do not forward messages for myself
  App.SerialCommandIntercept->BaseStationC.SerialIntercept[AM_COMMANDMSG];
// sending reports to UART via queue
  App.UartAMSend -> BaseStationC.SerialSend[AM_MULTIPINGRESPONSEREPORTMSG];
// sending commands and alive reports  
 App.UartCmdAMSend -> BaseStationC.SerialSend[AM_COMMANDMSG];
// sending noise floor readings 
 App.UartNoiseAMSend -> BaseStationC.SerialSend[AM_NOISEFLOORREADINGMSG];
// Quick serial sending by pushing to queue
 App.UartQueue -> BaseStationC.SerialQueue;
 App.RSSIQueue -> RSSIQueue;
//MultiPingResponseReportStruct_t 
 
 // split controll notifiers
  App.RadioControl -> BaseStationC.BSRadioControl;
  App.SerialControl -> BaseStationC.BSSerialControl;
  
  App.Boot -> MainC;
  App.SendTimer -> SendTimer;
  App.AliveTimer -> AliveTimer;
  App.PingTimer -> PingTimer;
  
  App.PingMsgSend -> PingMsgSender;
  //App.RadioControl -> ActiveMessageC;
  App.Leds -> LedsC;
  App.Packet -> PingMsgSender;
  App.AMPacket -> PingMsgSender;
  
  App.RadioPacket -> ActiveMessageC;
  App.RadioAMPacket -> ActiveMessageC;
  
  App.UartPacket -> Serial;
  App.UartAMPacket -> Serial;
  
  App.BSControl -> BaseStationC.BSControl;
  
  App.Reset -> ResetC;
  
  /**************** NOISE FLOOR READING ****************/
  App.NoiseFloorTimer -> NoiseFloorTimer;
}
