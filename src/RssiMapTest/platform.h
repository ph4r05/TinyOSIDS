/* 
 * File:   platform.h
 * Author: ph4r05
 *
 * Created on May 27, 2011, 11:07 AM
 */

//#ifndef PLATFORM_H
//#define	PLATFORM_H

// if iris mote, redefine bootstrap to support WatchDog based restart
#if defined(PLATFORM_IRIS)
    #ifndef platform_bootstrap
    #define platform_bootstrap() \
        MCUSR = 0; \
        wdt_disable();
    #endif
#endif

//#endif	/* PLATFORM_H */

