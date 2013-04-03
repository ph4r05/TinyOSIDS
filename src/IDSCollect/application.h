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

#ifndef CTPTEST_APP_H__
#define CTPTEST_APP_H__

#ifndef TOSH_DATA_LENGTH
#define TOSH_DATA_LENGTH 34
#endif

#include "../TOSph4IDS/cc2420_ids.h"
#include "printf.h"

/**
 * Warning!
 * If you want to generate Java Messages by MIG (genJavaMsgs.sh) you need to comment line
 * define MIG.
 * 
 * MIG has trouble to include some needed header files, so they are included in MUGhlp.h.
 */
#include "../ctpMsg.h"
#include "../commands.h"

/**
 * Include CTP definitions for application, not needed when generating MIG data
 */
#ifndef MIG
#include "Ctp.h"
#endif

#ifndef NULL
#define NULL ((void*)0)
#endif

#define CCA_SAMPLE_BUFFER_SIZE 4
#define CCA_SAMPLE_BUFFER_ELEM_SIZE 16
#define CCA_SAMPLE_TIMER_MILLI 50
#define CTPINFO_TYPE_CCA 2


#define CTP_FORWARD_ATTACKER_DROPPING 1
#define CTP_FORWARD_ATTACKER_DELAY 1

#define CTP_ROUTING_TABLE_SIZE 60
typedef struct staticRoute_t_ {
	am_addr_t nodeId;
	am_addr_t parentId;
} staticRoute_t;

// Config structure
// This configuration structure can be stored to flash memory
// in order to be configurable at runtime and to survive node
// restart/power-cycle. 
// Now it is hardwired in boot, but can be easily modiffied
// to be configurable at runtime with config commands. 
// Each config command should change this config structure,
// it would be then saved to flash memory like in http://docs.tinyos.net/tinywiki/index.php/Storage
typedef struct config_t {
    uint8_t ctpTxData;
    uint8_t ctpTxRoute;
    // send request wired to configuration structure
    nx_struct CtpSendRequestMsg ctpSendRequest;
    // if YES then after boot is launched CTP send according to ctpSendRequest
    bool sendingCTP;
    // static CTP root address, only 1 node can be root here
    uint16_t rootAddress;
    // if tree dumping is enabled
    bool treeDumping;
    // if CCA sampling is enabled
    bool CCASampling;
    // tree dumping interval
    uint16_t treeDumpingInterval;
    // static routing table
    bool useStaticRoute;
    staticRoute_t rtable[CTP_ROUTING_TABLE_SIZE];
} config_t;

#endif // CTPTEST_APP_H__