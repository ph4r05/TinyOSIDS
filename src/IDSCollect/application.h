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

#define CTP_FORWARD_ATTACKER_DROPPING 1
#define CTP_FORWARD_ATTACKER_DELAY 1

#endif // CTPTEST_APP_H__