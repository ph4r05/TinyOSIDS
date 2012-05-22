#if defined(PLATFORM_IRIS)
   #ifndef platform_bootstrap
   #define platform_bootstrap() \
       MCUSR = 0; \
        wdt_disable();
    #endif
#endif

