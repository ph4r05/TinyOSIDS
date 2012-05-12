#ifndef COMMANDS_H__
#define COMMANDS_H__ 1

/**
 * Contains command & identify messages
 * Common for majority of applications
 */


// message id
enum {
  AM_COMMANDMSG = 14,
  AM_IDENTIFYMSG = 40
};

/**
 * Command message
 * BaseStation manages its network using this simple protocol. More tasks can be
 * handled using it. For more complex settings its better to use another protocol.
 *
 * Available commands:
 *  - abort
 *      abort current operation
 *      Applicable on mobile nodes or on static nodes.
 *  - identify
 *      Nodes should reply its identification (static/mobile) when receives this packet
 *      BaseStation would know wether nodes lives and which one does.
 *  - myid
 *      Nodes returns its identification
 *  - reset
 *      Reset all state variables. It should have same effect as pushing reset button on node
 *  - ACK
 *      ACKnowledges previous packet. May relate to another command sent by BS
 *  - NACK
 *      Negative ACKnowledge
 *  - setBS
 *      tells static nodes to set/change current base station
 *  - lock
 *      changes locking for static nodes
 *  - getReportingStatus
 *      gets actual reporting status of given node on MultiPingResponses
 *  - setReportingStatus
 *      tells node wether or not to send reports on MultiPingResponses
 *      usefull when analyzing environment by pinging static nodes. Sometimes
 *      may be wanted static nodes not to report anything
 *  - setOperationMode
 *      nodes may be various types, static, dynamic, dead, base station
 *  - setReportProtocol
 *      reportprotocol to be used when in "static" mode
 *
 * Another possible commands, not yet implemented. Not needed now.
 *  - setTx
 *  - setChannel
 *  - setDelay
 *  - setPacketCount
 *  - setRandomizedThreshold
 *  - etc...
 */
typedef nx_struct CommandMsg {
	// command
	nx_uint8_t command_code;

	// command version. Support for protocol versioning. Some nodes may use older
	// firmware. May be used as packet subtype field
	nx_uint8_t command_version;

	// unique command identifier
	// nodes would be able to ACK or NACK commands.
	// It poses more reliable communication.
	// @not-implemented-yet
	nx_uint16_t command_id;

	// in case of ACK command, it is used as answer on specific command
	// only sugar, this info could be stored in command_data_next
	nx_uint8_t reply_on_command;
	nx_uint16_t reply_on_command_id;

	// some data associated with command (parameters for example)s
	nx_uint16_t command_data;

	// for future use
	// may contain another parameters while command_data would tell subtype of protocol
	nx_uint16_t command_data_next[4];
} CommandMsg;


typedef nx_struct IdentifyMsg {
	// command
	nx_uint16_t counter;
	// my node id - announcement
	nx_uint16_t nodeId;
	// reply on some command?
	nx_uint8_t replyOn;
	// platform identification
	nx_uint8_t platformId;
	// number of identify messages sent after boot yet. If overflow
	// stops on maximum number.
	nx_uint8_t identifyAfterBoot;
	
	nx_uint8_t radioQueueLen;
	nx_uint8_t serialQueueLen;
	nx_uint8_t rssiQueueLen;
	nx_uint8_t failCount;

	// for future use
	// may contain another parameters while command_data would tell subtype of protocol
	nx_uint16_t command_data_next[4];
} IdentifyMsg;

/**
 * Defining available commands for nodes
 */
enum {
	COMMAND_NONE = 0,
	COMMAND_ABORT = 1,
	COMMAND_IDENTIFY = 2,
	COMMAND_RESET = 3,
	COMMAND_SETTX = 4,
	COMMAND_SETCHANNEL = 5,
	COMMAND_ACK = 6,
	COMMAND_NACK = 7,
	COMMAND_SETBS = 8,
	COMMAND_LOCK = 9,

	COMMAND_GETREPORTINGSTATUS = 10,

	COMMAND_SETREPORTINGSTATUS = 11,
	COMMAND_SETDORANDOMIZEDTHRESHOLDING = 12,
	COMMAND_SETQUEUEFLUSHTHRESHOLD = 13,
	COMMAND_SETTINYREPORTS = 14,
	COMMAND_SETOPERATIONMODE = 15,
	COMMAND_SETREPORTPROTOCOL = 16,
	COMMAND_FLUSHREPORTQUEUE = 17,
	COMMAND_SETNOISEFLOORREADING = 18,
	COMMAND_SETSAMPLESENSORREADING = 24,

	COMMAND_SETREPORTGAP = 19,

	// sensor readings
	COMMAND_GETSENSORREADING = 20,
	COMMAND_SENSORREADING = 21,

	// pinning
	COMMAND_SETPIN = 22,
	COMMAND_GETPIN = 23,

	// settings
	// Fetching is request sent to base station after booting node up. 
	// Base station will then re-send node settings from node register to 
	// booted node (can be after reset already)
	COMMAND_FETCHSETTINGS = 25,
	
	// base station settings - forwarding from radio to serial?
	COMMAND_FORWARDING_RADIO_ENABLED = 26,
	// base station settings - forwarding from serial to radio?
	COMMAND_FORWARDING_SERIAL_ENABLED = 27,
	
	// base station settings - forwarding from radio to serial? default=without specific wiring
	COMMAND_DEFAULT_FORWARDING_RADIO_ENABLED = 28,
	// base station settings - forwarding from serial to radio? setRadioSnoopEnabled
	COMMAND_DEFAULT_FORWARDING_SERIAL_ENABLED = 29,
	
	// base station settings - whether forward messages received on snoop interface?
	COMMAND_RADIO_SNOOPING_ENABLED = 30,
	// base station settings - address recognition? if false then mote will sniff foreign messages
	COMMAND_RADIO_ADDRESS_RECOGNITION_ENABLED = 31,
	
	// set node as CTP root
	COMMAND_SET_CTP_ROOT=32,
	
	// call CTP route recomputing command - depending on data, CtpInfo interface is used
	// data=1 -> CtpInfo.triggerRouteUpdate()
	// data=2 -> CtpInfo.triggerImmediateRouteUpdate()
	// data=3 -> CtpInfo.recomputeRoutes()
	// data=4 -> RouterReinit re-init neighbour table
	// data=5 -> LinkEstimator re-init neighbour table
	COMMAND_CTP_ROUTE_UPDATE=33,
	
	// gets basic CTP info from CtpInfo interface
	// data=0 -> returns parent, etx, neighbors count in data[0], data[1], data[2]
	// data=1 -> info about neighbor specified in data[0]. Returned addr, link quality, route
    //				quality, congested bit
	COMMAND_CTP_GETINFO=34,
	
	// other CTP controling, can set TX power for packets
	// data=0 -> set tx power for OUTPUT messages for CTP protocol.
	// 				if data[0] == 1	-> set TXpower for ROUTE messages on data[1] level
	//			 	if data[0] == 2 -> set TXpower for DATA messages on data[1] level
	//				if data[0] == 3 -> set TXpower for both ROUTE, DATA messages on data[1] level
	COMMAND_CTP_CONTROL=35,

	// invokes request on global time for every node which heard this request
	COMMAND_TIMESYNC_GETGLOBAL=36, 
	
	// request response protocol to meassure RTT of channel, should be as fast as possible
	COMMAND_PING=37,
	
	// send timesync get_global command to radio broadcast
	COMMAND_TIMESYNC_GETGLOBAL_BCAST=38, 
};

#endif //COMMANDS_H__