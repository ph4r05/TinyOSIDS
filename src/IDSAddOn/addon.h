#ifndef ADDON_H__
#define ADDON_H__

// override default cc2420 header file - modified metadata files
#include "cc2420_ids.h"

// command protocol
#include "../commands.h"

// define null if is not already defined
#ifndef NULL
#define NULL ((void*)0)
#endif

/**
 * Warning!
 * If you want to generate Java Messages by MIG (genJavaMsgs.sh) you need to comment line
 * define MIG.
 * 
 * MIG has trouble to include some needed header files, so they are included in MUGhlp.h.
 */
#ifdef DEBUGPRINTF
#ifdef MIG
#include "../printf.h"
#else
#include "printf.h"
#endif
#endif

/**
 * Include CTP definitions for application, not needed when generating MIG data
 */
#ifndef MIG
//#include "Ctp.h"
#include "AM.h"
#include "Serial.h"
#else 

#endif



#endif