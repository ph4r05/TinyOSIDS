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

//#include <stdlib.h>

#include "ApplicationDefinitions.h"
#include "../RssiDemoMessages.h"
#include "AM.h"
#include "Reset.h"

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

module SendingMoteC {
  uses interface Boot;
  uses interface Timer<TMilli> as SendTimer;
  uses interface Timer<TMilli> as FlushTimer;
  uses interface Timer<TMilli> as GapTimer;
  uses interface Timer<TMilli> as SignalTimer;
  uses interface Timer<TMilli> as NoiseFloorTimer;
  uses interface Timer<TMilli> as SensorReadingTimer;
  uses interface Timer<TMilli> as BootupTimer;

// sender
  uses interface AMSend as RssiMsgSend;
  uses interface AMSend as ReportMsgSend;
  uses interface AMSend as TinyReportMsgSend;
  uses interface AMSend as CommandMsgSend;
  uses interface AMSend as MultiPingResponseSend;
/*
  uses interface AMSend as RadioSend;
*/

  uses interface SplitControl as RadioControl;
  
// receiver
  uses interface Receive as SimplePingReceive;
  uses interface Receive as PingReceive;
  uses interface Receive as ResponseReceive;
  uses interface Receive as CommandReceive;
  //uses interface Receive as RadioReceive[am_id_t id];
  	
  uses interface Leds;
  uses interface Packet;
  uses interface AMPacket;

  // random generator for framing
  uses interface Random;
  
  // reset node
  //interface Reset as Reset;

  // platform specific sensors
#if defined(PLATFORM_TELOSB)
	//SENSORS
        uses interface Read<uint16_t> as Temperature; // sensor
        uses interface Read<uint16_t> as Humidity; // sensor
        uses interface Read<uint16_t> as Light;
        
        // general pins
        uses interface HplMsp430GeneralIO as Pin0;
        uses interface HplMsp430GeneralIO as Pin1;
#elif defined(PLATFORM_TYNDALL25)
        //SENSORS
        uses interface Read<uint16_t> as Temperature; // sensor
        uses interface Read<uint16_t> as Humidity; // sensor
        uses interface Read<uint16_t> as Light;
#elif defined(PLATFORM_IRIS)
        // no sensors for IRIS platform
#else
        //#error "No SENSORS defined for this platform"
#endif
  //queue packet
  //uses interface BigQueue<MultiPingResponseReportStruct> as ReportQueue;
  
/**
 * RSSI readings
 */
#ifdef __CC2420_H__
  uses interface CC2420Packet;
  
  // set channel
  uses interface CC2420Config;

  // noise floor rssi reading
  uses interface Read<uint16_t> as ReadRssi;
#elif defined(PLATFORM_IRIS)
  uses interface PacketField<uint8_t> as PacketRSSI;
  uses interface PacketField<uint8_t> as PacketTransmitPower;
#elif defined(TDA5250_MESSAGE_H)
  uses interface Tda5250Packet;    
#else
  uses interface PacketField<uint8_t> as PacketRSSI;
#endif  
} implementation {

    // by default use 8times larger queue than send packet
    enum {
        // length of queue defined by constant in common config file
        // if is too big, tmote wont boot (no blue signalization at start)
        RADIO_QUEUE_LEN = MAX_REPORT_QUEUE_SIZE,

        // timer is started when packet is received
        // it will fire after QUEUE_TIMEOUT ms what invokes queue flushing
        QUEUE_TIMEOUT = 1000,

        // gap in [ms] between reports
        REPORT_SEND_GAP=15,

        // gap in [ms] when to start sending reports after mass queue size threshold reached
        REPORT_SEND_THRESH=15,

        // how many noise floor readings has to be performed
        NOISEFLOOR_READINGS=10,

        // gap in [ms] when sampling noisefloor
        NOISEFLOOR_DELAY=100
    };

    // radio queue & management
    MultiPingResponseReportStruct radioBuff[MAX_REPORT_QUEUE_SIZE];
    uint8_t radioIn, radioOut;
    bool radioBusy, radioFull;
    uint8_t queueFlushThreshold = MULTIPINGRESPONSEREPORT_MAXDATA;    

    // timeout timer fired, invoke queue flushing
    bool queueTimeout = FALSE;

  // message to send
  message_t pkt;

  // another message for privileged communication
  // for. ex. commands
  message_t pkt_priv;

  // response message
  message_t pkt_response;

  // where I should send next command?
  // node_id, destination
  uint16_t command_dest=1;
  
  // where to send sensor reading data?
  // can be broadcast when are rssi-sampled sensor reading data
  // or base station or asking station
  uint16_t sensorReadingDest=1;

  // radio busy flag
  // main variable controlling radio access
  bool busy = FALSE;

  // message counter
  uint16_t counter = 0;
  uint16_t counter_ping=0;

  // tx power switching
  uint8_t cur_tx_power;
  uint8_t tx_power = 31;
  uint8_t base_tx_power = 31;

  // channel switching
  uint8_t cur_channel = 0;
  uint8_t channel = 0;

  // blink counters
  uint8_t blinkCnSend = 0;
  uint8_t blinkCnRecv = 0;
  uint8_t blinkCnFail = 0;

  // timer delay
  uint16_t delay = 100;
  uint16_t packets2send = 0;

  // base station address
  uint16_t baseid = 1;

  // reporting status flag
  // if FALSE do not report anything
  // causes node to not act as mirror
  bool doReporting=TRUE;

  // if TRUE queue will be flushed on random basis
  // otherwise queue will be flushed when reaches queueFlushThreshold
  bool doRandomizedThresholding=TRUE;

  // for signal timer to visual communication
  uint16_t signalCounter=0;

  // basic node operation mode
  // since some protocol are stateful we need to know what are we supposed to do
  uint8_t operationMode=NODE_REPORTING;

  // flag determining whether do rssi-sampling of sensor reading data
  bool doSensorReadingSampling = FALSE;
          
  // which reporting protocol should be used
  // - randomized medium reports
  // - tiny reports
  // - mass reports with mobile radio silence during reporting phase
  uint8_t reportProtocol=REPORTING_MEDIUM;

  // report trigered by 3rd party or by us indicator
  // when TRUE, flushing queue to basestation, another reception of responses
  // is blocked/incoming packets are ignored
  bool reportMassTrigered=FALSE;

  // report mass queue threshold
  // if queue reaches this threshold && reportProtocol==REPORTING_MASS, responseQueue
  // will start to flush, setting reportMassTrigered=TRUE and sending command on broadcast
  // COMMAND_FLUSHQUEUE...
  uint16_t massReportQueueThreshold=0;

  // do noise floor reading?
  bool doNoiseFloorReading=TRUE;

  // number of noiseFloor readings measured at the moment
  // (for 1 mass report cycle)
  uint8_t noiseFloorMeasured=0;

  // time gap between reports
  uint8_t reportingGap=REPORT_SEND_GAP;
  
  // is this mote freshly booted?
  // if TRUE, sendIdentification will request settings from base station.
  // this option is automatically set to FALSE after first command from base station received
  bool tabulaRasa=TRUE;
  
  // number of sent identification messages
  uint8_t bootupCounter=0;

  // node sensor reading mode
  // 1 = temperature
  // 2 = light
  // 3 = humidity
  uint8_t readMode = 1;

    uint16_t temperature;
    uint16_t humidity;
    uint16_t light;
    uint16_t oldSensorValue=0;
    uint16_t tmpReading = 0;
    uint16_t msgReadingLong = 0;
    uint16_t numConditionValide = 0;

/**
 * Forward declarations
 */
  uint16_t getRssi(message_t *msg);
  message_t* receive(message_t* msg, void* payload, uint8_t len, am_id_t id);
  void task sendMultipleEcho();
  void task sendReport();
  void setAutoAck(bool enableAutoAck, bool hwAutoAck);
  uint8_t getChannel();
  void reset_node();
  void abort();
  void sendQueueFlushCommand();
  void startReportFlush();
  void task sendIdentification();

  // platform specific
  void setPower(message_t *msg, uint8_t power);
  void setChannel(uint8_t new_channel);
  error_t readNoiseFloor();
  void task sendReading();
  void readSensors();
  bool setGIO(uint8_t pin, bool enabled);
  bool message2sampleReceived(uint16_t rssi, uint16_t cn, uint16_t source);
  
  /**
   * Queue management
   */

  /**
   * Returns number of elements in queue
   *
   * @return number of elements in queue
   */
  uint16_t queueSize(){
      return (radioOut > radioIn) ? RADIO_QUEUE_LEN - radioOut + radioIn : radioIn - radioOut;
  }

/**
 * Leds signals
 */
  void rcvBlink() {
      // blink each 10th message
      if (blinkCnRecv==0){
        call Leds.led2Toggle();
      }

      blinkCnRecv = (blinkCnRecv+1) % 20;
  }

  void sendBlink() {
      // blink each 10th message
      if (blinkCnSend==0){
        call Leds.led1Toggle();
      }

      blinkCnSend = (blinkCnSend+1) % 5;
  }  

  // fail signalization - blink on each second fail
  void failBlink() {
      if (blinkCnFail==0){
        call Leds.led0Toggle();
      }

      blinkCnFail = (blinkCnFail+1) % 1;
  }

  /**
   * Visual signalization by 2nd/blue LED
   * Used to ACKnowledge some command (on boot, after command reception)
   *
   * @param num - how many times blink with led
   */
  void signalize(uint8_t num){
      signalCounter = 2*num;
      
      // disable signalization led
      call Leds.led0Off();
      call Leds.led1Off();
      call Leds.led2Off();

      // start periodic timer to toggle 
      call SignalTimer.startPeriodic(500);
  }

  // eventhandler, system booted
  // perform radio init
  event void Boot.booted(){
  	uint8_t i=0;
        busy = TRUE;
        
        // radio queue init
        for (i = 0; i < RADIO_QUEUE_LEN; i++) {
            radioBuff[i].nodeid = 0;
            radioBuff[i].nodecounter = 0;
            radioBuff[i].rssi = 0;
        }
        radioIn = radioOut = 0;
        radioBusy = FALSE;
        radioFull = TRUE;

        call RadioControl.start();

        // use reset function
        reset_node();

        // signalize success boot procedure
        // sometimes very usefull (when used ram region is too big, mote won't boot)
        signalize(2);
  }


  /**
   * Radio Control Start Stop
   */
  event void RadioControl.startDone(error_t result){
    // send identification when radio started
    if (result == SUCCESS) {
        busy = FALSE;

        // send identification to base station
        // when booted up
        call BootupTimer.startOneShot(BOOTUPTIMER_FIRST);
    }
    else {
      call RadioControl.start();
    }
  }

  event void RadioControl.stopDone(error_t result){
  }

  /**
   * Timer fired right after bootup to setup nodes correctly
   */
  event void BootupTimer.fired(){
      post sendIdentification();
  }
  
  /**
   * Timeout timer event handler
   * should be oneShot timer firing on queue timeout
   * set flag on flush and invoke sending function
   */
  event void FlushTimer.fired(){
      queueTimeout=TRUE;

      // if we use mass report protocol, flush whole queue
      if (reportProtocol==REPORTING_MASS){          
          // send report command only if I am first who is reporting this
          if (reportMassTrigered==FALSE){
                sendQueueFlushCommand();
          }
          
          reportMassTrigered=TRUE;
      }

      // call now report send
      post sendReport();
  }

  // led ACK signalization
  event void SignalTimer.fired(){
      if (signalCounter<=0) {
          call SignalTimer.stop();
          return;
      }

      call Leds.led1Toggle();
      call Leds.led2Toggle();
      signalCounter--;
  }

  /**
   * SendTimer eventhandler
   * Used in talking mode to send responses on echo request
   *
   * if packets2send==0 mote will send unlimited number of packets
   * otherwise mote will send packets2send packets
   */
  event void SendTimer.fired(){
      // if packets = 0 send unlimited number of packets
      // sending can be stopped by receiving another request with non-null number
      // of requested packets to send
      if (packets2send == 0){
          post sendMultipleEcho();
      }
      else
      {
          if (packets2send >= counter_ping){
            post sendMultipleEcho();
          }
          else{
            // turn timer off iff every packet was already sent
            call SendTimer.stop();

            // reset coutner
            counter_ping = 0;
            failBlink();
          }
      }
  }

  /**
   * Make gaps between packet sends.
   * This one is intended to make gaps between reports when flushing queue to base station.
   * Solves collisions and bad sends sometimes
   */
  event void GapTimer.fired(){
      // call now report send
      post sendReport();
  }

  /**
   * When fired perform noise floor reading from radio.
   */
  event void NoiseFloorTimer.fired(){
      readNoiseFloor();
  }

  /**
   * for timer-driven sensor reading
   */
  event void SensorReadingTimer.fired(){
        readSensors();
  }

  /**
   * Send answer on multiple ping request
   * According to requested parameters from request send as many packets as wanted
   * Used in localization scheme when trying to sample RSSI from packet test set
   */
  void task sendMultipleEcho(){
     if (!busy) {
    	MultiPingResponseMsg* btrpkt = (MultiPingResponseMsg*)(call Packet.getPayload(&pkt_response, 0));

    	// set transmit power for each packet
    	setPower(&pkt_response, tx_power);

        // ping coutner
        btrpkt->counter = counter_ping;

    	if (call MultiPingResponseSend.send(AM_BROADCAST_ADDR, &pkt_response, sizeof(MultiPingResponseMsg)) == SUCCESS) {
      	    busy = TRUE;
    	}
    	else {
    		failBlink();
    		post sendMultipleEcho();
    	}
    } else {
            failBlink();
            post sendMultipleEcho();
    }
  }

  /**
   *  Decision function, should send part of queue ?
   */
  bool doQueueSend() {
      uint16_t qsize=0;

      // is this node set to be dead?
      if (operationMode == NODE_DEAD) return FALSE;

      // no-reporting?
      if (doReporting==FALSE) return FALSE;

      // if medium report queue is full
      if (radioFull) return TRUE;

      // timeout timer fired?
      if (queueTimeout) return TRUE;

      // if size of queue is bigger than threshold
      // threshold shoul be randomized to minimize collisions
      // assume we have 12 static nodes, it would consume bandwidth when
      // it tries to send reports at same time.
      // From same reason it is usefull to keep backoff random timer between two
      // report pakets

      // compute queue size. we now hat radioFull = false
      qsize = queueSize();

      // tiny report, 1 in queue is enough
      if (reportProtocol==REPORTING_TINY && qsize>=1) return TRUE;

      // threshold, randomize decision here from interval <4,8>
      if (reportProtocol==REPORTING_MEDIUM && qsize >= queueFlushThreshold) return TRUE;

      // mass report, if triggered by another node, or size is too big
      if (reportProtocol==REPORTING_MASS && ( reportMassTrigered || qsize >= massReportQueueThreshold )) return TRUE;

      // otherwise return false;
      return FALSE;
  }

  /**
   * Action performed when report queue flush is needed
   * IF is noise floor reading enabled, perform it at first, then flush queue
   * otherwise do flush queue right now
   */
  void startReportFlush(){
      // if we are not using MASS report protocol, do not perform noisereading
      if (reportProtocol!=REPORTING_MASS){ 
          post sendReport();
          return;
      }

      // now holds reportProtocol==REPORTING_MASS
      if (doNoiseFloorReading==TRUE){
          // perform noise floor reading
          call NoiseFloorTimer.startOneShot(NOISEFLOOR_DELAY);
      }
      else {
          // call queue flush right now
          call GapTimer.startOneShot(REPORT_SEND_THRESH);
      }
  }

  /**
   * send report on multiple ping
   */
  void task sendReport(){
      uint8_t tmpOut=0;
      uint8_t tmpI=0;
      
     if (!busy) {
         MultiPingResponseReportMsg* btrpkt=NULL;
         MultiPingResponseTinyReportMsg* btrpkt_tiny=NULL;
                 
         // do queue send? if conditions are not met
         if (!doQueueSend()) return;

        // radio empty queue test, on empty queue return false
        //atomic
        if (radioIn == radioOut && !radioFull) {
            radioBusy = FALSE;
            return;
        }

         // reset rssi noise floor reading counter
         noiseFloorMeasured=0;

         // code for tiny reports
         if (reportProtocol==REPORTING_TINY){
            btrpkt_tiny = (MultiPingResponseTinyReportMsg*)(call Packet.getPayload(&pkt, 0));

            // setup message with data
            btrpkt_tiny->counter = counter;

            //atomic
            btrpkt_tiny->nodeid = radioBuff[tmpOut].nodeid;
            btrpkt_tiny->nodecounter = radioBuff[tmpOut].nodecounter;
            btrpkt_tiny->rssi = radioBuff[tmpOut].rssi;

            // set transmit power for each packet according to basestation instrutions
            setPower(&pkt, base_tx_power);

            // send to base directly
            // sometimes node refuses to send too large packet. it will always end with fail
            // depends of buffers size.
            if (call TinyReportMsgSend.send(baseid, &pkt, sizeof(MultiPingResponseTinyReportMsg)) == SUCCESS) {
                busy = TRUE;
                counter++;
            }
            else
            {
                    failBlink();
                    dbg("Cannot send message");
                    
                    // try re-send in 5ms
                    call GapTimer.startOneShot(reportingGap + reportingGap/5 - (call Random.rand16() % (reportingGap/3)));
            }
         }
         
         // code for REPORTING_MEDIUM, REPORTING_MASS
         else {
            btrpkt = (MultiPingResponseReportMsg*)(call Packet.getPayload(&pkt, 0));

            // setup message with data
            btrpkt->counter = counter;

            //atomic
            for(tmpOut=radioOut, tmpI=0; tmpOut!=radioIn && tmpI < queueFlushThreshold ; tmpI++, tmpOut = (tmpOut+1) % RADIO_QUEUE_LEN){
                btrpkt->nodeid[tmpI] = radioBuff[tmpOut].nodeid;
                btrpkt->nodecounter[tmpI] = radioBuff[tmpOut].nodecounter;
                btrpkt->rssi[tmpI] = radioBuff[tmpOut].rssi;

                if (tmpI>queueFlushThreshold) break;
            }

            // store num of real result to datanum field
            btrpkt->datanum = tmpI;

            // set transmit power for each packet according to basestation instrutions
            setPower(&pkt, base_tx_power);

            // send to base directly
            // sometimes node refuses to send too large packet. it will always end with fail
            // depends of buffers size.
            if (call ReportMsgSend.send(baseid, &pkt, sizeof(MultiPingResponseReportMsg)) == SUCCESS) {
                busy = TRUE;
                counter++;
            }
            else
            {
                    failBlink();
                    dbg("Cannot send message");

                    // try re-send in 5ms
                    call GapTimer.startOneShot(reportingGap + reportingGap/5 - (call Random.rand16() % (reportingGap/3)));
            }
         }
    }
    else
    {
            failBlink();
            //post sendReport();

            // try re-send in 5ms
            call GapTimer.startOneShot(reportingGap + reportingGap/5 - (call Random.rand16() % (reportingGap/3)));
    }
  }

  /**
   * Send ACK command
   * packet si prepared before calling this
   */
  void task sendCommandACK(){
      if (!busy) {
         CommandMsg* btrpkt=(CommandMsg*)(call Packet.getPayload(&pkt_priv, 0));

        // setup message with data
        btrpkt->command_id = counter;

        // deprecated, allow to send any command
        // used for queueFlush command too
        //
        // ACK as reply, if not set already
        //if (btrpkt->command_code != COMMAND_ACK && btrpkt->command_code != COMMAND_NACK)
        //    btrpkt->command_code = COMMAND_ACK;

        // send to base directly
        // sometimes node refuses to send too large packet. it will always end with fail
        // depends of buffers size.
    	if (call CommandMsgSend.send(command_dest, &pkt_priv, sizeof(CommandMsg)) == SUCCESS) {
      	    busy = TRUE;
    	}
    	else
    	{
    		failBlink();
                dbg("Cannot send message");
    		post sendCommandACK();
    	}
    }
    else
    {
            failBlink();
            post sendCommandACK();
    }
  }
  
  /**
   * Set my identification to base station
   */
  void task sendIdentification(){
      if (!busy) {
        CommandMsg* btrpkt=(CommandMsg*)(call Packet.getPayload(&pkt_priv, 0));

        // setup message with data
        btrpkt->command_id = counter;

        // reply on COMMAND_IDENTIFY
        btrpkt->reply_on_command = COMMAND_IDENTIFY;

        // ACK as reply
        btrpkt->command_code = COMMAND_ACK;

        // i am static node
        btrpkt->command_data = operationMode;

        // fill radio chip here
#ifdef __CC2420_H__
        btrpkt->command_data_next[0]=1;
#elif defined(PLATFORM_IRIS)
        btrpkt->command_data_next[0]=2;
#elif defined(TDA5250_MESSAGE_H)
        btrpkt->command_data_next[0]=3;
#else
        btrpkt->command_data_next[0]=4;
#endif

        // fill node ID
        btrpkt->command_data_next[1] = TOS_NODE_ID;
        
        // tabula rasa?
        if (tabulaRasa){
            btrpkt->command_data_next[2] = 1;
            
            // send message again until there is no tabulaRasa settings (until is 
            // some command sent by BS)
            call BootupTimer.startOneShot(bootupCounter < 1 ? BOOTUPTIMER_FIRST : BOOTUPTIMER_NEXT);
            bootupCounter+=1;
        } else {
            btrpkt->command_data_next[2] = 0;
        }

        // send to base directly
        // sometimes node refuses to send too large packet. it will always end with fail
        // depends of buffers size.
    	if (call CommandMsgSend.send(baseid, &pkt_priv, sizeof(CommandMsg)) == SUCCESS) {
      	    busy = TRUE;
    	}
    	else {
    		failBlink();
                dbg("Cannot send message");
    		post sendIdentification();
    	}
    }
    else {
            failBlink();
            post sendIdentification();
    }
  }

  // clear bigqueue
  void clearQueue(){
/*
      uint16_t size = call ReportQueue.size();
      uint16_t c = 0;

      for(c=0; c<size; c++){
          call ReportQueue.dequeue();
      }
*/
  }


  /**
   * stop doing everything now
   */
  void abort(){
      // stop timers
      call SendTimer.stop();
      call GapTimer.stop();
      call FlushTimer.stop();
      call SignalTimer.stop();
      call SensorReadingTimer.stop();
      call BootupTimer.stop();
      
      packets2send = 0;
      signalCounter=0;
      noiseFloorMeasured=0;

      // radio queue
      radioIn = radioOut = 0;
      radioBusy = FALSE;
      radioFull = TRUE;

      // delete all elements from report queue
      clearQueue();

      // disable leds
      call Leds.led0Off();
      call Leds.led1Off();
      call Leds.led2Off();
  }

  /**
   * Performs hard reset of node - node will be restarted HW via watchdog freezing
   */
  void reset_hard(){
      // try to perform reall reset
      failBlink();
      //call Reset.reset();
      resetMote();
  }
  
  /**
   * reset node to initial state.
   */
  void reset_node(){
      // abort all running processes
      abort();

      queueFlushThreshold = MULTIPINGRESPONSEREPORT_MAXDATA;

      // timeout timer fired, invoke queue flushing
      queueTimeout = FALSE;

      // radiu busy flag
      busy = FALSE;

      // message counter
      counter = 0;

      // tx power switching
      tx_power = 31;
      base_tx_power = 31;

      // channel switching
      cur_channel = 0;
      channel = 0;

      // blink counters
      blinkCnSend = 0;
      blinkCnRecv = 0;
      blinkCnFail = 0;

      // timer delay
      delay = 100;
      packets2send = 0;
      baseid = 1;
      doReporting=TRUE;
      doRandomizedThresholding=TRUE;

      signalCounter=0;

      // disable leds
      call Leds.led0Off();
      call Leds.led1Off();
      call Leds.led2Off();

      // radio queue
      radioIn = radioOut = 0;
      radioBusy = FALSE;
      radioFull = TRUE;

      // delete all elements from report queue
      clearQueue();

      // default operation mode
      if (TOS_NODE_ID >= MOBILE_NODE_ID_BOUDNARY){
          operationMode = NODE_TALKING;
      }
      else {
          operationMode = NODE_REPORTING;
          doSensorReadingSampling = TRUE;
      }

      // default report type is medium
      reportProtocol=REPORTING_MEDIUM;

      // disable all trigers
      reportMassTrigered=FALSE;

      // calculate new threshold for mass report messages
      massReportQueueThreshold = MAX_REPORT_QUEUE_SIZE * 0.8;

      // set default destination for commands as BS
      command_dest = baseid;

      doNoiseFloorReading=TRUE;
      noiseFloorMeasured=0;
      reportingGap=REPORT_SEND_GAP;
      readMode = 1;
      
      doSensorReadingSampling = FALSE;
      
      tabulaRasa = TRUE;
      bootupCounter = 0;
  }

  /** 
   * Tiny msg send report
   */
  event void ReportMsgSend.sendDone(message_t *m, error_t error){
      uint8_t tmpI = 0;
      busy = FALSE;

  	if (error == SUCCESS){
            // send was successfull
            sendBlink();

            // turn timer off
            call FlushTimer.stop();

            // manage queue, move queue pointer
            atomic
            if (TRUE) {
                // change pointers, move end of queue
                for(tmpI=0; radioOut!=radioIn && tmpI<queueFlushThreshold; tmpI++){
                    radioOut = (radioOut + 1) % RADIO_QUEUE_LEN;
                }

                // radio full flag
                radioFull = FALSE;

                // if we change threshold randomly, compute new one
                if (doRandomizedThresholding){
                    // generate new threshold for new round
                    // here is safe place to do it (end of the round)
                    queueFlushThreshold = 1 + (call Random.rand16() % MULTIPINGRESPONSEREPORT_MAXDATA);
                }
            }
        }
        else {
            // failed -> make fail blink signalization
            failBlink();

            // try re-send in 5ms
            call GapTimer.startOneShot(reportingGap + reportingGap/5 - (call Random.rand16() % (reportingGap/3)));
        }

      // on mass report do massive report send
      // call sendRecord by GapTimer again, if there is still something to send
      // from queue
      if (reportProtocol == REPORTING_MASS && reportMassTrigered==TRUE){
          // check if report is over = flush queue is empty
          uint16_t qsize = queueSize();

          if (qsize==0){
              reportMassTrigered=FALSE;

              // radio queue
              radioIn = radioOut = 0;
              radioBusy = FALSE;
              radioFull = TRUE;

              // delete all elements from report queue
              clearQueue();
          }
          else {
            // triger gapTimer
            call GapTimer.startOneShot(reportingGap + reportingGap/5 - (call Random.rand16() % (reportingGap/3)));
          }
      }
  }

  /**
   * Report sent event handler
   * Manage queue, compute new threshold and stop timers
   */
  event void TinyReportMsgSend.sendDone(message_t *m, error_t error){
        busy = FALSE;
  	if (error == SUCCESS){
            // send was successfull
            sendBlink();

            // turn timer off
            call FlushTimer.stop();

            // manage queue, move queue pointer
            atomic
            if (TRUE) {
                // change pointers, move end of queue
                radioOut = (radioOut + 1) % RADIO_QUEUE_LEN;
                
                // radio full flag
                radioFull = FALSE;
            }
        }
        else {
            failBlink();
        }
  }

  event void RssiMsgSend.sendDone(message_t *m, error_t error){
    busy = FALSE;

    if (error == SUCCESS){
        sendBlink();
    }
    else {
        failBlink();
    }
  }

  event void MultiPingResponseSend.sendDone(message_t *m, error_t error){
        busy = FALSE;

        if (error == SUCCESS){
                sendBlink();

                // increment counter only on succ sended
                counter_ping++;
        }
        else {
            failBlink();
        }
  }

  /**
   * Command message sent event.
   * Can react on it according to error level
   */
  event void CommandMsgSend.sendDone(message_t *m, error_t error){
    // CommandMsg* btrpktresponse = (CommandMsg *) (call Packet.getPayload(m, 0));      
      
    // busy = false, we sent packet, now it is free
    busy = FALSE;
  }
  
/**
 * Radio Receive
 */
/*
  event message_t *RadioReceive.receive[am_id_t id](message_t *msg,void *payload, uint8_t len) {
        return receive(msg, payload, len, id);
  }
*/

  //
  //
  // COMMAND RECEIVED
  //
  //
  event message_t *CommandReceive.receive(message_t *msg, void *payload,  uint8_t len) {
      if (len == sizeof(CommandMsg)){
          // get received message
          CommandMsg* btrpkt = (CommandMsg*) payload;
          
          // get local message, prepare it
          CommandMsg* btrpktresponse = (CommandMsg *) (call Packet.getPayload(&pkt_priv, 0));

          // set reply ID by default
          btrpktresponse->reply_on_command = btrpkt->command_code;
          btrpktresponse->reply_on_command_id = btrpkt->command_id;
          btrpktresponse->command_code = COMMAND_ACK;
          command_dest=baseid;
          
          // this is probably new setting from base station -> tabulaRasa option 
          // to false
          tabulaRasa = FALSE;
          
          // decision based on type of command
          switch(btrpkt->command_code){
              case COMMAND_IDENTIFY:
                  // send my identification. Perform as task
                  post sendIdentification();
                  signalize(2);

                  break;

              case COMMAND_RESET:
                  // perform hard HW reset with watchdog to be sure that node is clean
                  // after boot-up
                  reset_hard();
                  
                  // if here something went wrong, node should have been reseted here.
                  // Thus perform SW reset - set node state to initial conditions
                  reset_node();

                  btrpktresponse->command_code = COMMAND_ACK;
                  
                  // signalize 1 = HW reset was not successful. To bea able to distinguish
                  // between HW and SW reset.
                  signalize(1);
                  //post sendCommandACK();
                  break;

              case COMMAND_ABORT:
                  abort();
                  
                  btrpktresponse->command_code = COMMAND_ACK;
                  signalize(2);
                  post sendCommandACK();
                  break;

              case COMMAND_SETTX:
                  base_tx_power = btrpkt->command_data;

                  btrpktresponse->command_code = COMMAND_ACK;
                  post sendCommandACK();
                  signalize(2);
                  break;

              case COMMAND_SETBS:
                  baseid = btrpkt->command_data;

                  btrpktresponse->command_code = COMMAND_ACK;
                  post sendCommandACK();
                  signalize(2);
                  break;

              case COMMAND_GETREPORTINGSTATUS:
                  btrpktresponse->command_code = COMMAND_ACK;
                  btrpktresponse->command_data = (nx_uint16_t) doReporting;
                  post sendCommandACK();
                  signalize(2);
                  break;

              case COMMAND_SETREPORTINGSTATUS:
                  doReporting = (bool) btrpkt->command_data;

                  btrpktresponse->command_code = COMMAND_ACK;
                  post sendCommandACK();
                  signalize(2);
                  break;

              // controll random thresholding
              case COMMAND_SETDORANDOMIZEDTHRESHOLDING:
                  doRandomizedThresholding = (bool) btrpkt->command_data;

                  btrpktresponse->command_code = COMMAND_ACK;
                  post sendCommandACK();
                  signalize(2);
                  break;

              // set queue threshold
              case COMMAND_SETQUEUEFLUSHTHRESHOLD:
                  if (reportProtocol == REPORTING_MEDIUM)
                    queueFlushThreshold = btrpkt->command_data;
                  else if (reportProtocol == REPORTING_MASS)
                    massReportQueueThreshold = btrpkt->command_data;
                  else return msg;

                  btrpktresponse->command_code = COMMAND_ACK;
                  post sendCommandACK();
                  signalize(2);
                  break;

              // use tiny reports instead of sum reports
              case COMMAND_SETTINYREPORTS:
                  reportProtocol = REPORTING_TINY;

                  // seth threshold to 1
                  queueFlushThreshold = 1;

                  btrpktresponse->command_code = COMMAND_ACK;
                  post sendCommandACK();
                  signalize(2);
                  break;

              case COMMAND_SETOPERATIONMODE:
                  operationMode = btrpkt->command_data;

                  btrpktresponse->command_code = COMMAND_ACK;
                  post sendCommandACK();
                  signalize(2);
                  break;

              case COMMAND_SETREPORTPROTOCOL:
                  reportProtocol = btrpkt->command_data;

                  btrpktresponse->command_code = COMMAND_ACK;
                  post sendCommandACK();
                  signalize(2);
                  break;

              // flush report queue command
              case COMMAND_FLUSHREPORTQUEUE:
                  // if we are dead node, do not listed
                  if (operationMode==NODE_DEAD) return msg;

                  // if we are in talking mode, stop listening (pause maybe)
                  else if (operationMode==NODE_TALKING){
                        // now stop send
                        abort();

                        // send signal to user
                        signalize(1);
                  }

                  // in reporting mode obey command and do queue flush until is empty
                  // only when using MASS report protocol
                  // this kind of synchronization aplies only on marr report protocol
                  else if (operationMode==NODE_REPORTING && reportProtocol==REPORTING_MASS && reportMassTrigered==FALSE){
                      // perform noise floor reading at first, then flush queue,
                      // do not send my own command
                      reportMassTrigered = TRUE;

                      // send signal to user
                      signalize(1);

                      // send reports
                      post sendReport();
                  }
                  break;

              // setting noise floor reading
              case COMMAND_SETNOISEFLOORREADING:
                  doNoiseFloorReading = (bool) btrpkt->command_data;

                  btrpktresponse->command_code = COMMAND_ACK;
                  post sendCommandACK();
                  signalize(2);
                  break;

              // setting reporting gap timeout
              case COMMAND_SETREPORTGAP:
                  reportingGap = btrpkt->command_data;

                  btrpktresponse->command_code = COMMAND_ACK;
                  post sendCommandACK();
                  signalize(2);
                  break;

                  
              // pin settings
              // sets digital value on output pin
              case COMMAND_SETPIN:                  
                  if (setGIO((uint8_t) btrpkt->command_data_next[0], btrpkt->command_data==1 ? TRUE:FALSE)){
                      btrpktresponse->command_code = COMMAND_ACK;
                      post sendCommandACK();
                      signalize(2);
                  }
                  break;
                  
              // set request for sensor reading
              // request driven mode
              // can be specified to use timer-driven mode and its parameters
              // (delay)
              case COMMAND_GETSENSORREADING:
                  // readMode = which sensor to read in this request
                  readMode = (uint8_t) btrpkt->command_data;

                  // timer-driven mode request?
                  // if in additional data[0] is 1 then modify current settings
                  // for timer-driven sensor reading
                  // else do not touch that timer.
                  if (btrpkt->command_data_next[0]==1){
                      signalize(3);
                      // timeout is defined in command_data_next[1]
                      // if timeout is 0 then stop current timer and do not start new
                      if (btrpkt->command_data_next[1]==0){
                          // stop only if running
                          if (call SensorReadingTimer.isRunning()){
                              call SensorReadingTimer.stop();
                          }
                      } else {
                          // start periodic timer with defined timeout (in ms)
                          call SensorReadingTimer.startPeriodic(btrpkt->command_data_next[1]);
                      }

                      // flags here
                      // sensor reading destination is broadcast?
                      if ((btrpkt->command_data_next[2]&1)>0){
                          // broadcast destination (probably rssi-sampled on anchor nodes)
                          sensorReadingDest = 65535U;
                      } else {
                          // no broadcast destination, answer -> base station
                          sensorReadingDest = command_dest;
                      }
                  }

                  // read appropriate sensors
                  readSensors();

                  // signalize command received as ussual
                  //signalize(2);
                  break;
                  
                  
              // changing variable doSensorReadingSampling
              // If true then sensor readings from another nodes will be sampled for RSSI signal
              case COMMAND_SETSAMPLESENSORREADING:
                  doSensorReadingSampling = btrpkt->command_data > 0 ? TRUE : FALSE;

                  btrpktresponse->command_code = COMMAND_ACK;
                  post sendCommandACK();
                  signalize(2);
                  break;
                  
              // another sensor reading packet.
              // if is sampling enabled, do rssi sample of this packet and add to queue   
              case COMMAND_SENSORREADING:
                  if (doSensorReadingSampling){
                      // do sample this message for rssi, is it possible now?
                      message2sampleReceived(getRssi(msg), btrpkt->command_id, call AMPacket.source(msg));
                  } else {
                      // else ignore this packet, not interested in reading sensors 
                      // of foreign nodes
                      ;
                  }
                  break;

              default:
                  failBlink();
                  signalize(1);
          }
      }
      else {
          signalize(1);
          failBlink();
      }
      
      return msg;
 }

  event message_t *PingReceive.receive(message_t *msg, void *payload,  uint8_t len) {
        // ping is allowed everytime
        // but when is in reporting mode, do not resume after massQueueFlush command
        return receive(msg, payload, len, AM_MULTIPINGMSG);
  }

  /**
   * Send reply on request with given TX power
   */
  void task sendEchoReply() {
    if (!busy) {
        RssiMsg* btrpkt = (RssiMsg*)(call Packet.getPayload(&pkt, 0));
        btrpkt->nodeid  = TOS_NODE_ID;

        // set transmit power for each packet
        setPower(&pkt, tx_power);

        // set channel
        //setChannel(channel);

        if (call RssiMsgSend.send(AM_BROADCAST_ADDR, &pkt, sizeof(RssiMsg)) == SUCCESS) {
            busy = TRUE;
        }
        else
        {
                failBlink();
                post sendEchoReply();
        }
    }
    else
    {
            failBlink();
            post sendEchoReply();
    }
  }

  event message_t *SimplePingReceive.receive(message_t *msg, void *payload,  uint8_t len) {
        // ping is allowed everytime
        // but when is in reporting mode, do not resume after massQueueFlush command
        if (len == sizeof(PingMsg)) {
            PingMsg* btrpkt = (PingMsg*)payload;

            if (!busy)
            {
              // get local tmp
              RssiMsg* btrpktresponse = (RssiMsg *)(call Packet.getPayload(&pkt, 0));

              // do some logic with counter
              rcvBlink();
              counter = btrpkt->counter;

                // prepare message to send
              btrpktresponse->nodeid  = TOS_NODE_ID;
              btrpktresponse->dstnodeid  = btrpkt->nodeid;
              btrpktresponse->counter = btrpkt->counter;
              btrpktresponse->rssiFromBase = getRssi(msg);

              // set my tx_power and channel as base station wants
              tx_power = btrpkt->txpower;
              channel = btrpkt->channel;

              post sendEchoReply();
            }
          }
          else
          {
            failBlink();
          }
          return msg;
  }

  /**
   * Report receive message handler
   * Ignore incoming messages when:
   *    - node is in DEAD status
   *    - node is not in REPORTING status
   *    - node is in REPORTING status, reportProtocol==REPORTING_MASS and we are now sending
   *        mass reports to BaseStation. Not to store data when another nodes are sending prorably
   *        It would cause badly measured RSSI values probably
   */
  event message_t *ResponseReceive.receive(message_t *msg, void *payload,  uint8_t len) {
        // reporting is inactive if:
        //  - node is in DEAD status
        //  - node is not in reporting status
        if (operationMode == NODE_DEAD) return msg;
        if (operationMode != NODE_REPORTING) return msg;

        // if we are in mass reporting phase, do not store new reports, ignore it
        if (reportProtocol==REPORTING_MASS && reportMassTrigered==TRUE) return msg;

	return receive(msg, payload, len, AM_MULTIPINGRESPONSEMSG);
  }

  // global message receivings
  message_t* receive(message_t* msg, void* payload, uint8_t len, am_id_t id){
      // message AM_MULTIPINGMSG received
      // node is expected to answer on ping request
       if (id == AM_MULTIPINGMSG && len == sizeof(MultiPingMsg)){
            // multiping message
            MultiPingMsg* btrpkt = (MultiPingMsg*) payload;

            busy=FALSE;
            if (!busy) {
                // get local tmp
                MultiPingResponseMsg* btrpktresponse = (MultiPingResponseMsg *) (call Packet.getPayload(&pkt_response, 0));

                // stop timer right now
                call SendTimer.stop();

                // do some logic with counter
                rcvBlink();

                // prepare message to send
                btrpktresponse->counter = btrpkt->counter;

                // set my tx_power and channel as base station wants
                tx_power = btrpkt->txpower;

                // copy delay and packet count
                delay = btrpkt->delay;
                packets2send = btrpkt->packets;

                // counter -> 0
                counter_ping = 0;

                // disable autoACK
                setAutoAck(FALSE, FALSE);

                // start counter firing periodically with requested delay
                call SendTimer.startPeriodic(delay);

                // signalize TXpower
                //signalize(tx_power);
            }
       }

       // response from another node arrived
       // if we are in reporting mode, report this
       else if (id == AM_MULTIPINGRESPONSEMSG && len == sizeof(MultiPingResponseMsg)) {
            // multiping message
            // reportMote needs queue for incomming MultiPingResponse messages!!!
            
            MultiPingResponseMsg* btrpkt = (MultiPingResponseMsg*) payload;
            
            if (!busy) {
                bool added2queue = FALSE;

                // do some logic with counter
                rcvBlink();
                
                // disable autoACK
                setAutoAck(FALSE, FALSE);

                // process message
                added2queue = message2sampleReceived(getRssi(msg), btrpkt->counter, call AMPacket.source(msg));
            }
        }

       // another messages
        else {
            // don't know such message
            failBlink();
            dbg("Dont know such message!");
        }
        return msg;
  }
  
  /**
   * Message to sample received.
   * Add given data to queue and process queue if needed.
   */
  bool message2sampleReceived(uint16_t rssi, uint16_t cn, uint16_t source){
      bool returnValue=FALSE;
      
      // if reporting is disabled do nothing
      if (doReporting==FALSE){
          return FALSE;
      }
      
    atomic
    if (!radioFull) {
        uint16_t qsize=queueSize();
        
        // init storage structure, fill meassured data in
        radioBuff[radioIn].nodeid = source;
        radioBuff[radioIn].nodecounter = cn;
        radioBuff[radioIn].rssi = rssi;

        // sending queue counter management
        if (++radioIn >= RADIO_QUEUE_LEN)
            radioIn = 0;
        if (radioIn == radioOut)
            radioFull = TRUE;

        // turn timeout timer flag off. New packet received, timer has to be restarted
        queueTimeout = FALSE;

        // set timeout timer
        // if is waiting window too long, flush sending queue
        // according to http://www.tinyos.net/dist-2.0.0/tinyos-2.0.0beta1/doc/html/tep102.html
        // start will cancel any running timer and start over again, we want oneShot timer
        call FlushTimer.startOneShot(QUEUE_TIMEOUT);

        // if is radio free try to send messages from queue
        if (!radioBusy) {
            post sendReport();
            radioBusy = TRUE;
        }

        // manage mass queue here
        // if is queue filled to the threshold, perform queue flush
        qsize=queueSize();
        if (reportProtocol == REPORTING_MASS && qsize >= massReportQueueThreshold){
                // get local command message, prepare it to send queueFlush
                CommandMsg* btrpktresponse = (CommandMsg *) (call Packet.getPayload(&pkt_priv, 0));

                // set mass trigering to true
                reportMassTrigered=TRUE;

                // send flush queue command
                btrpktresponse->reply_on_command = 0;
                btrpktresponse->reply_on_command_id = 0;
                btrpktresponse->command_code = COMMAND_FLUSHREPORTQUEUE;
                command_dest=65535U;

                // send it on broadcast address
                post sendCommandACK();

                // trigger flushing, delayed
                call GapTimer.startOneShot(REPORT_SEND_THRESH);
        }
        
        returnValue = TRUE;
    } else {
        failBlink();
        dbg("RadioQueue full!");
        
        returnValue = FALSE;
    }

    // now simply post packet
    post sendReport();
    
    return returnValue;
  }

  /**
   * send queue flush command
   */
  void sendQueueFlushCommand(){
        // send report command only if I am first who is reporting this
        if (reportMassTrigered==FALSE){
            // get local command message, prepare it to send queueFlush
            CommandMsg* btrpktresponse = (CommandMsg *) (call Packet.getPayload(&pkt_priv, 0));

            // set mass trigering to true
            reportMassTrigered=TRUE;

            // send flush queue command
            btrpktresponse->reply_on_command = 0;
            btrpktresponse->reply_on_command_id = 0;
            btrpktresponse->command_code = COMMAND_FLUSHREPORTQUEUE;
            command_dest=65535U;

            // send it on broadcast address
            post sendCommandACK();
        }
  }

  /**
   * Read appropriate sensors according to readMode setting
   * Callling READ interface. Results are returnet as corresponding  .readDone event
   */
    void readSensors(){
        
        #if defined(PLATFORM_TELOSB) || defined(PLATFORM_TYNDALL25)
        if ((readMode & 15) == 1 || (readMode & 15) == 4)
            call Temperature.read();
        if ((readMode & 15) == 2)
            call Light.read();
        if ((readMode & 15) == 3)
            call Humidity.read();
        #else
        ;
        #endif
    }

#if defined(PLATFORM_TELOSB) || defined(PLATFORM_TYNDALL25)
    event void Temperature.readDone(error_t ok, uint16_t val) {
            if (ok == SUCCESS ) {
                    temperature = val;
                    if ((readMode & 15) == 1 || (readMode & 15) == 4){//temperature
                            // tmpReading is now 10 * realValue in degrees of celsius
                            // conversion formula is: (3)  temperature = -39.60 + 0.01*SOt
                            // source: http://docs.tinyos.net/index.php/Boomerang_ADC_Example
                            tmpReading = 10*(-39.60 + 0.01*val);
                            msgReadingLong = numConditionValide + 1*SCALE_TYPE_READ; // read temperature
                    }
            }

            // readMode == 4 ~ temperature compensated relative humidity
            if ((readMode & 15) == 4){
                 call Humidity.read();
                return;
            } else {
                // send sensor reading as command message
                post sendReading();
            }
    }

    event void Light.readDone(error_t ok, uint16_t val) {
            if (ok == SUCCESS ) {
                    light = val;
                    if ((readMode & 15) == 2){//Light
                            tmpReading = val;
                            msgReadingLong = numConditionValide + 2*SCALE_TYPE_READ; // read Light
                    }
            }

            // send sensor reading as command message
            post sendReading();
    }

    event void Humidity.readDone(error_t ok, uint16_t val) {
            if (ok == SUCCESS ) {
                    humidity = val;
                    if ((readMode & 15) == 3){//Humidity
                            // -4 + 0.0405*SOrh + (-2.8 * 10^-6)*(SOrh^2)
                            tmpReading = -4 + 0.0405*val + (-2.8 * 1e-6)*(val*val) ;
                            msgReadingLong = numConditionValide + 3*SCALE_TYPE_READ; //read Humidity
                    } else if ((readMode & 15) == 4){
                        // temperature compensated humidity
                        uint8_t temperatureTmp = tmpReading/10;
                        tmpReading = (temperatureTmp - 25) * (0.01 + 0.00008*val) + (-4 + 0.0405*val + (-2.8 * 1e-6)*(val*val)) ;
                        msgReadingLong = numConditionValide + 3*SCALE_TYPE_READ; //read Humidity
                    }
            }

            // send sensor reading as command message
            post sendReading();
    }
#endif
    
    void task sendReading(){
         if (!busy) {
             CommandMsg* btrpkt=(CommandMsg*)(call Packet.getPayload(&pkt_priv, 0));

            // setup message with data
            btrpkt->command_id = 1;
            btrpkt->command_data = tmpReading;
            btrpkt->command_code = COMMAND_SENSORREADING;
            btrpkt->command_data_next[0] = readMode;
            
            // set transmit power for each packet
            setPower(&pkt_priv, tx_power);

            // send to base directly
            // sometimes node refuses to send too large packet. it will always end with fail
            // depends of buffers size.
            if (call CommandMsgSend.send(sensorReadingDest, &pkt_priv, sizeof(CommandMsg)) == SUCCESS) {
                busy = TRUE;
            } else {
                failBlink();
                dbg("Cannot send message");
                post sendReading();
            }
        }
        else
        {
            failBlink();
            post sendReading();
        }
    }


    /**
     * 
     * Pin setting
     * Set digital output on specified pin as specified
     * 
     */
    bool setGIO(uint8_t pin, bool enabled){
        #if defined(PLATFORM_MICA) || defined(PLATFORM_MICA2) || defined(PLATFORM_MICA2DOT) || defined(PLATFORM_MICAZ)
                return FALSE;
        #elif defined(PLATFORM_TELOS) || defined(PLATFORM_TELOSB) || defined(PLATFORM_EPIC)
                // msp430
                if (pin == 0 && enabled==TRUE){
                    call Pin0.set();
                } else if (pin==0 && enabled==FALSE){
                    call Pin0.clr();
                } else if (pin==1 && enabled==TRUE){
                    call Pin1.set();
                } else if (pin==1 && enabled==FALSE){
                    call Pin1.clr();
                } else {
                    return FALSE;
                }
                
                return TRUE;
        #elif defined(PLATFORM_IRIS)
                  return FALSE;
        #else
                  return FALSE;
        #endif
        return FALSE;
    }
    
    
/**
 * GetRssi multiplatform implementation
 * determines RSSI from incoming message
 */
#ifdef __CC2420_H__  
  uint16_t getRssi(message_t *msg){
    return (uint16_t) call CC2420Packet.getRssi(msg);
  }
#elif defined(CC1K_RADIO_MSG_H)
    uint16_t getRssi(message_t *msg){
    cc1000_metadata_t *md =(cc1000_metadata_t*) msg->metadata;
    return md->strength_or_preamble;
  }
#elif defined(PLATFORM_IRIS)
  uint16_t getRssi(message_t *msg){
    if(call PacketRSSI.isSet(msg))
      return (uint16_t) call PacketRSSI.get(msg);
    else
      return 0xFFFF;
  }
#elif defined(TDA5250_MESSAGE_H)
   uint16_t getRssi(message_t *msg){
       return call Tda5250Packet.getSnr(msg);
   }
#else
  #error Radio chip not supported! This demo currently works only \
         for motes with CC1000, CC2420, RF230 or TDA5250 radios.  
#endif

/**
 * Set transmit power for node
 */
#ifdef __CC2420_H__
  // set power  
  void setPower(message_t *msg, uint8_t power){
  	if (power >= 1 && power <=31)
  	{
  		cur_tx_power = power;
                call CC2420Packet.setPower(msg, power);
        }
  }
  
  // set channel
  event void CC2420Config.syncDone( error_t error ) {                                                                 
  }
  
  /**
   * Change the channel of the radio, between 11 and 26
   */
  void setChannel( uint8_t new_channel ){
  	if (cur_channel==0){
  		cur_channel = getChannel();
  	}
  	
  	if (cur_channel!=new_channel){
  		call CC2420Config.setChannel(new_channel);
                call CC2420Config.sync();
  		cur_channel = new_channel;
  	}	  
  }
  
  uint8_t getChannel(){
  	return call CC2420Config.getChannel();
  }

  void setAutoAck(bool enableAutoAck, bool hwAutoAck){
    call CC2420Config.setAutoAck(enableAutoAck, hwAutoAck);
    call CC2420Config.sync();
  }

  void setAck(message_t *msg, bool status){
      ;
  }

  /**
   * Split-phase command to read noise floor
   * Can meassure only when radio is not busy and there is some space on queue
   */
  error_t readNoiseFloor(){
      if (busy==FALSE && radioFull==FALSE){
          busy=TRUE;
          return call ReadRssi.read();
      }

      failBlink();
      return FAIL;
  }

  // noise floor results
  event void ReadRssi.readDone(error_t error, uint16_t data) {
      busy=FALSE;
      // enqueue this data
      if (error==SUCCESS){
          call Leds.led2Toggle();

          // store noise floor rssi reading on queue
          // i am source
          radioBuff[radioIn].nodeid = TOS_NODE_ID;
          radioBuff[radioIn].nodecounter = noiseFloorMeasured;
          radioBuff[radioIn].rssi = data;

          // sending queue counter management
          if (++radioIn >= RADIO_QUEUE_LEN)
              radioIn = 0;
          if (radioIn == radioOut)
              radioFull = TRUE;

          // increment reading ID
          noiseFloorMeasured++;

          // start one shot timer
          call NoiseFloorTimer.startOneShot(NOISEFLOOR_DELAY);
      }
      else {
          failBlink();
         // start one shot timer
          call NoiseFloorTimer.startOneShot(NOISEFLOOR_DELAY);
      }

      // are we finished?
      if (noiseFloorMeasured >= NOISEFLOOR_READINGS){
            // stop timer if yes
            call NoiseFloorTimer.stop();

            // now flush queue
            call GapTimer.startOneShot(REPORT_SEND_GAP);
      }
  }

#elif defined(PLATFORM_IRIS)
  // IRIS
  // set power

  /**
   * Set transmit power
   */
  void setPower(message_t *msg, uint8_t power){
  	 cur_tx_power = power;
         call PacketTransmitPower.set(msg, power);
  }

  /**
   * Get transmit power
   */
  uint8_t getPower(message_t *msg){
      return call PacketTransmitPower.get(msg);
  }

  /**
   * Change the channel of the radio, between 11 and 26
   */
  void setChannel(uint8_t new_channel){
/*
  	if (cur_channel==0){
  		cur_channel = getChannel();
  	}

  	if (cur_channel!=new_channel){
  		call CC2420Config.setChannel(new_channel);
                call CC2420Config.sync();
  		cur_channel = new_channel;
  	}
*/
  }

  uint8_t getChannel(){
  	//return call CC2420Config.getChannel();
  }

  // dummy implementation, no such method here
  void setAutoAck(bool enableAutoAck, bool hwAutoAck){
       ;
  }

  void setAck(message_t *msg, bool status){
      ;
  }

  /**
   * Split-phase command to read noise floor
   * Can meassure only when radio is not busy and there is some space on queue
   */
  error_t readNoiseFloor(){
      if (busy==FALSE && radioFull==FALSE){
          //busy=TRUE;
          //return call ReadRssi.read();
      }

      failBlink();
      return FAIL;
  }

#else
  #error Radio chip not supported! This demo currently works only \
         for motes with CC1000, CC2420, RF230 or TDA5250 radios.  
#endif
}
