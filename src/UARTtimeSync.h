#ifndef TIMESYNC_H__
#define TIMESYNC_H__ 1

// use this to define long timestamps - 64 bit
// #define TSTAMP64

#ifdef TSTAMP64
// we have 64bit timestamps
typedef nx_uint64_t nx_timestamp_t;
typedef uint64_t timestamp_t;
typedef int64_t  timestamp_diff_t;
#else
typedef nx_uint32_t nx_timestamp_t;
typedef uint32_t timestamp_t;
typedef int32_t  timestamp_diff_t;
#endif


// serial timesync
// timesync messages are sent over serial to synchronize global time according to 
// application
typedef nx_struct LowlvlTimeSyncMsg{
	nx_uint8_t counter;
	nx_uint8_t flags;
	nx_uint16_t offset;
	
#ifdef TSTAMP64
    nx_uint64_t globalTime;
#else
	nx_uint32_t high;
	nx_uint32_t low;
#endif
}  LowlvlTimeSyncMsg;

// AMId
enum {
  AM_LOWLVLTIMESYNCMSG = 0xEA,
  TIMESYNCMSG_LEN = sizeof(LowlvlTimeSyncMsg),
  TS_TIMER_MODE = 0,      // see TimeSyncMode interface
  TS_USER_MODE = 1,       // see TimeSyncMode interface
};

#endif // TIMESYNC_H__
