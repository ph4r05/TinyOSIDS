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
 * Author: Miklos Maroti, Brano Kusy, Janos Sallai
 * Date last modified: 3/17/03
 * Ported to T2: 3/17/08 by Brano Kusy (branislav.kusy@gmail.com)
 */

#include "../UARTtimeSync.h"
configuration TimeUARTSyncC
{
  uses interface Boot;
  provides interface Init;
  provides interface StdControl;
  provides interface GlobalUARTTime<TMilli>;

  //interfaces for extra functionality: need not to be wired
  provides interface TimeUARTSyncInfo;
  provides interface TimeSyncMode;
  provides interface TimeSyncNotify;
}

implementation
{
  components new TimeUARTSyncP(TMilli);

  GlobalUARTTime   =   TimeUARTSyncP;
  StdControl       =   TimeUARTSyncP;
  Init             =   TimeUARTSyncP;
  Boot             =   TimeUARTSyncP;
  TimeUARTSyncInfo =   TimeUARTSyncP;
  TimeSyncMode     =   TimeUARTSyncP;
  TimeSyncNotify   =   TimeUARTSyncP;

  components new SerialAMReceiverC(AM_LOWLVLTIMESYNCMSG) as SerialTestRecv;
  TimeUARTSyncP.Receive         ->  SerialTestRecv;

  components SerialActiveMessageC as SerialAM;
  TimeUARTSyncP.Packet -> SerialAM;

  components HilTimerMilliC;
  TimeUARTSyncP.LocalTime       ->  HilTimerMilliC;

#if defined(TIMESYNC_LEDS)
  components LedsC;
#else
  components NoLedsC as LedsC;
#endif
  TimeUARTSyncP.Leds  ->  LedsC;

}
