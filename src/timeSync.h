#ifndef TIMESYNC_H__
#define TIMESYNC_H__ 1

// AMId
enum {
  AM_TIMESYNCMSG = 0xea,
};

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

#endif // TIMESYNC_H__