/*
 * Copyright (c) 2002, Vanderbilt University
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 *
 * IN NO EVENT SHALL THE VANDERBILT UNIVERSITY BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE VANDERBILT
 * UNIVERSITY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * THE VANDERBILT UNIVERSITY SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE VANDERBILT UNIVERSITY HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 *
 * @author: Miklos Maroti, Brano Kusy (kusy@isis.vanderbilt.edu), Janos Sallai
 * Ported to T2: 3/17/08 by Brano Kusy (branislav.kusy@gmail.com)
 * Modified to UARTSync version: 17/11/2012 Dusan (Ph4r05) Klinec (ph4r05@gmail.com)
 */
#include "../UARTtimeSync.h"
#ifdef TUARTSYNC
#include "printf.h"
#endif
generic module TimeUARTSyncP(typedef precision_tag)
{
    provides
    {
        interface Init;
        interface StdControl;
        interface GlobalUARTTime<precision_tag>;

        //interfaces for extra functionality: need not to be wired
        interface TimeUARTSyncInfo;
        interface TimeSyncMode;
        interface TimeSyncNotify;
    }
    uses
    {
        interface Boot;
        interface Receive;
        interface Packet;
        interface Leds;
        interface LocalTime<precision_tag> as LocalTime;
    }
}
implementation
{
#ifndef TIMESYNC_RATE
#define TIMESYNC_RATE   10
#endif

    enum {
        MAX_ENTRIES           = 6,              // number of entries in the table
        BEACON_RATE           = TIMESYNC_RATE,  // how often send the beacon msg (in seconds)
        ROOT_TIMEOUT          = 5,              //time to declare itself the root if no msg was received (in sync periods)
        IGNORE_ROOT_MSG       = 4,              // after becoming the root ignore other roots messages (in send period)
        ENTRY_VALID_LIMIT     = 4,              // number of entries to become synchronized
        ENTRY_SEND_LIMIT      = 3,              // number of entries to send sync messages
        ENTRY_THROWOUT_LIMIT  = 100,            // if time sync error is bigger than this clear the table
    };

    typedef struct TableItem
    {
        uint8_t     state;
        uint64_t    localTime;
        int64_t     timeOffset; // GlobalTime - localTime
    } TableItem;

    enum {
        ENTRY_EMPTY = 0,
        ENTRY_FULL = 1,
    };

    TableItem   table[MAX_ENTRIES];
    uint8_t tableEntries;

    enum {
        STATE_IDLE = 0x00,
        STATE_PROCESSING = 0x01,
        STATE_SENDING = 0x02,
        STATE_INIT = 0x04,
    };

    uint8_t state, mode;

/*
    We do linear regression from localTime to timeOffset (GlobalUARTTime - localTime).
    This way we can keep the slope close to zero (ideally) and represent it
    as a float with high precision.

        timeOffset - offsetAverage = skew * (localTime - localAverage)
        timeOffset = offsetAverage + skew * (localTime - localAverage)
        GlobalTime = localTime + offsetAverage + skew * (localTime - localAverage)
*/

    float       skew;
    uint64_t    localAverage;
    int64_t     offsetAverage;
    uint8_t     numEntries; // the number of full entries in the table

    message_t processedMsgBuffer;
    message_t* processedMsg;
        
    uint64_t localTimeProcessedMsg=0; // local time from moment TimeSyncMessage arrived
    uint64_t highLocalTime; // high 32bit part of localtime.

    uint8_t heartBeats=0; // the number of successfully sent messages
                          // since adding a new entry with lower beacon id than ours

    async command uint64_t GlobalUARTTime.getLocalTime()
    {
        return ((call LocalTime.get()) | (((uint64_t)highLocalTime)<<32));
    }

    async command error_t GlobalUARTTime.getGlobalTime(uint64_t *time)
    {
        *time = call GlobalUARTTime.getLocalTime();
        return call GlobalUARTTime.local2Global(time);
    }

    error_t is_synced()
    {
      if (numEntries>=ENTRY_VALID_LIMIT)
        return SUCCESS;
      else
        return FAIL;
    }


    async command error_t GlobalUARTTime.local2Global(uint64_t *time)
    {
        *time += offsetAverage + (int64_t)(skew * (int64_t)(*time - localAverage));
        return is_synced();
    }

    async command error_t GlobalUARTTime.global2Local(uint64_t *time)
    {
        uint64_t approxLocalTime = *time - offsetAverage;
        *time = approxLocalTime - (int64_t)(skew * (int64_t)(approxLocalTime - localAverage));
        return is_synced();
    }

	/**
	 * Computes conversion parameters from local_time to global_time using stored values in table
	 */
    void calculateConversion()
    {
        float newSkew = skew;
        uint64_t newLocalAverage;
        int64_t newOffsetAverage;

        int64_t localSum;
        int64_t offsetSum;

        int8_t i;

        // find first meaningful entry
        for(i = 0; i < MAX_ENTRIES && table[i].state != ENTRY_FULL; ++i)
            ;

        if( i >= MAX_ENTRIES )  // table is empty
            return;
        
		/**
		 * We use a rough approximation first to avoid time overflow errors. The idea
		 * is that all times in the table should be relatively close to each other.
		 * 
		 * This way is real average computed in the end. Time overflow error:
		 * if we would sum all times in table just like in normal average algorithm,
		 * total sum could be too big and it could overflow. This way avoids overflow
		 * because it is initialized with first element, then is added only
		 * difference between times, thus (time_i - time_0)/N;
		 * 
		 * Normal average = 1/N SUM(time_i) 
		 *  -time_0 -> 1/N (SUM(time_i-time_0))
		 *  +time_0 -> 1/N (SUM(time_i-time_0)) + time_0 = SUM(time_i - time_0)/N
         */
        newLocalAverage = table[i].localTime;
        newOffsetAverage = table[i].timeOffset;

        localSum = 0;
        offsetSum = 0;

        while( ++i < MAX_ENTRIES )
            if( table[i].state == ENTRY_FULL ) {
                localSum += (int64_t)(table[i].localTime - newLocalAverage) / tableEntries;
                offsetSum += (int64_t)(table[i].timeOffset - newOffsetAverage) / tableEntries;
            }

        newLocalAverage += localSum;
        newOffsetAverage += offsetSum;
        
        /**
         * Average is computed here. We have localtime and offset average from stored entries in table.
         */
        localSum = offsetSum = 0;
        for(i = 0; i < MAX_ENTRIES; ++i)
            if( table[i].state == ENTRY_FULL ) {
                int64_t a = table[i].localTime - newLocalAverage;
                int64_t b = table[i].timeOffset - newOffsetAverage;

                localSum += (int64_t)a * a;
                offsetSum += (int64_t)a * b;
            }

        /**
         * after this:
         * localSum  = SUM_{i=0}^{N} ((table[i].localTime - newLocalAverage)^2)
         * offsetSum = SUM_{i=0}^{N} ((table[i].localTime - newLocalAverage) * (table[i].timeOffset - newOffsetAverage)) 
         * 
         * No overflow will occur provided entries in table are relatively close to each other (time)
         */

        if( localSum != 0 )
            newSkew = (float)offsetSum / (float)localSum;

        atomic
        {
            skew = newSkew;
            offsetAverage = newOffsetAverage;
            localAverage = newLocalAverage;
            numEntries = tableEntries;
        }
    }

    void clearTable()
    {
        int8_t i;
        for(i = 0; i < MAX_ENTRIES; ++i)
            table[i].state = ENTRY_EMPTY;

        atomic numEntries = 0;
    }

    uint8_t numErrors=0;
    void addNewEntry(TimeSyncMsg *msg)
    {
        int8_t i, freeItem = -1, oldestItem = 0;
        uint64_t age, oldestTime = 0, msgGlobalTime=0;
        int64_t timeError=0;

        tableEntries = 0;

        // assemble local time from received message
        msgGlobalTime=msg->low | (((uint64_t)(msg->high))<<32);

        // clear table if the received entry's been inconsistent for some time
        timeError = localTimeProcessedMsg;
        call GlobalUARTTime.local2Global((uint64_t*)(&timeError));
        timeError -= msgGlobalTime;
        
        // time error logic changed - root is server, thus no time error, except zero time is provided
        if (msg->high==0 && msg->low==0){
        	timeError=ENTRY_THROWOUT_LIMIT+1;
        }
        
        if( (is_synced() == SUCCESS) &&
            (timeError > ENTRY_THROWOUT_LIMIT || timeError < -ENTRY_THROWOUT_LIMIT))
        {
#ifdef TUARTSYNC
        printf("MsgIgn\n");
#endif
            if (++numErrors>3){
#ifdef TUARTSYNC
                printf("ClearTbl\n");
#endif
                clearTable();
            }
        }
        else
            numErrors = 0;
        
        // find oldest entry item - LRU
        for(i = 0; i < MAX_ENTRIES; ++i) {
            age = localTimeProcessedMsg - table[i].localTime;

            //logical time error compensation
            if( age >= 0x7FFFFFFFL )
                table[i].state = ENTRY_EMPTY;

            if( table[i].state == ENTRY_EMPTY )
                freeItem = i;
            else
                ++tableEntries;

            if( age >= oldestTime ) {
                oldestTime = age;
                oldestItem = i;
            }
        }

        if( freeItem < 0 )
            freeItem = oldestItem;
        else
            ++tableEntries;
        
        // add entry to table
        table[freeItem].state = ENTRY_FULL;
        table[freeItem].localTime = localTimeProcessedMsg;
        table[freeItem].timeOffset = msgGlobalTime - localTimeProcessedMsg;
        
        // increase heartbeats
        atomic{
            heartBeats+=1;
        }
        
#ifdef TUARTSYNC
        printf("TimeAdded:%d; loc: %ld; glob: %ld\n", freeItem, localTimeProcessedMsg, msgGlobalTime);
#endif
    }

    void task processMsg()
    {
        TimeSyncMsg* msg = (TimeSyncMsg*)(call Packet.getPayload(processedMsg, sizeof(TimeSyncMsg)));
        call Leds.led0Toggle();
#ifdef TUARTSYNC
        printf("SRecv\n");
#endif
        addNewEntry(msg);
        calculateConversion();
        signal TimeSyncNotify.msg_received();
        state &= ~STATE_PROCESSING;
    }

    event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len)
    {
        if( (state & STATE_PROCESSING) == 0 ) {
            message_t* old = processedMsg;
            uint32_t curTime = call LocalTime.get();
            
            // local time overflow! Warning, 32bit counter will last for 49 days, 
            // need to receive single message in this interval, otherwise will be late
            // workaround: set millisecond timer with very large timeout (e.g. 2^20) to watch for localTime 
            // overflows...
            if ((localTimeProcessedMsg & 0xFFFFFFFF) > curTime){
            	atomic{
            	   highLocalTime+=1;
            	}
            }
            
            processedMsg = msg;
            localTimeProcessedMsg = curTime | (((uint64_t)highLocalTime)<<32);

            state |= STATE_PROCESSING;
            post processMsg();

            return old;
        }

        return msg;
    }

    command error_t TimeSyncMode.setMode(uint8_t mode_){
        if (mode == mode_)
            return SUCCESS;

        mode = mode_;
        return SUCCESS;
    }

    command uint8_t TimeSyncMode.getMode(){
        return mode;
    }

    command error_t TimeSyncMode.send(){
        if (mode == TS_USER_MODE){
            ;
            return SUCCESS;
        }
        return FAIL;
    }

    command error_t Init.init()
    {
        atomic{
            skew = 0.0;
            localAverage = 0;
            offsetAverage = 0;
            heartBeats = 0;
        };

        clearTable();

        processedMsg = &processedMsgBuffer;
        state = STATE_INIT;

        return SUCCESS;
    }

    event void Boot.booted()
    {
      
    }

    command error_t StdControl.start()
    {
        mode = TS_TIMER_MODE;
        heartBeats = 0;

        return SUCCESS;
    }

    command error_t StdControl.stop()
    {
        return SUCCESS;
    }

    async command float     TimeUARTSyncInfo.getSkew() { return skew; }
    async command uint64_t  TimeUARTSyncInfo.getOffset() { return offsetAverage; }
    async command uint64_t  TimeUARTSyncInfo.getSyncPoint() { return localAverage; }
    async command uint16_t  TimeUARTSyncInfo.getRootID() { return 0; }
    async command uint8_t   TimeUARTSyncInfo.getSeqNum() { return 0; }
    async command uint8_t   TimeUARTSyncInfo.getNumEntries() { return numEntries; }
    async command uint8_t   TimeUARTSyncInfo.getHeartBeats() { return heartBeats; }

    default event void TimeSyncNotify.msg_received(){}
    default event void TimeSyncNotify.msg_sent(){}
}
