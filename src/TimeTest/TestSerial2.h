
#ifndef TEST_SERIAL_H
#define TEST_SERIAL_H

#define CC2420_CHANNEL 17


typedef nx_struct test_serial_msg {
  nx_uint16_t counter;
  nx_uint16_t received;
  nx_uint8_t radioCn;
  nx_uint8_t radioOn;

  nx_uint8_t radioErr;
  nx_uint8_t radioErrCn;
  nx_uint16_t radioRecv;
  nx_uint16_t radioSent;
  
} test_serial_msg_t;

typedef nx_struct timeSyncReport {
  nx_uint64_t localTime;
  nx_uint64_t globalTime;
  
  nx_uint64_t lastSync;
  nx_uint64_t offset;
  nx_float skew;
  
  nx_uint8_t hbeats;
  nx_uint8_t entries;
	
} timeSyncReport;

typedef nx_struct CommandMsg {
	nx_uint8_t command_code;
	nx_uint8_t command_version;
	nx_uint16_t command_id;
	nx_uint8_t reply_on_command;
	nx_uint16_t reply_on_command_id;
	nx_uint16_t command_data;
	nx_uint16_t command_data_next[4];
} CommandMsg;


enum {
  AM_TEST_SERIAL_MSG = 0x89,
  AM_COMMANDMSG = 14,
  AM_TIME_SYNC_REPORT = 0x68,
};

enum {
	COMMAND_IDENTIFY = 2,
	COMMAND_RESET = 3,
	COMMAND_ACK = 6,

	// invokes request on global time for every node which heard this request
	COMMAND_TIMESYNC_GETGLOBAL=36, 
	
	// request response protocol to meassure RTT of channel, should be as fast as possible
	COMMAND_PING=37,
	
	// send timesync get_global command to radio broadcast
	COMMAND_TIMESYNC_GETGLOBAL_BCAST=38, 
};

#endif
