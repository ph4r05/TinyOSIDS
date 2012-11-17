#ifndef TIMESYNC_H__
#define TIMESYNC_H__ 1

// serial timesync
// timesync messages are sent over serial to synchronize global time according to 
// application
typedef nx_struct TimeSyncMsg{
	nx_uint8_t counter;
	nx_uint16_t offset;
	nx_uint32_t high;
	nx_uint32_t low;
	nx_uint8_t flags;
}  TimeSyncMsg;

// AMId
enum {
  AM_TIMESYNCMSG = 0xea,
  TIMESYNCMSG_LEN = sizeof(TimeSyncMsg) - sizeof(nx_uint32_t),
    TS_TIMER_MODE = 0,      // see TimeSyncMode interface
    TS_USER_MODE = 1,       // see TimeSyncMode interface
};

#endif // TIMESYNC_H__