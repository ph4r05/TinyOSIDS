// mig helper header file - sometimes cannot include all neccessary TOS header files
// includes AM.h, Collection.h, Ctp.h

#ifndef MIGHLP_H__
#define MIGHLP_H__ 


#ifndef AM_H
#define AM_H

// These are the right types, but ncc currently does not 
// like parameters being network types
typedef nx_uint8_t nx_am_id_t;
typedef nx_uint8_t nx_am_group_t;
typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;
typedef uint8_t am_group_t;
typedef uint16_t am_addr_t;

enum {
  AM_BROADCAST_ADDR = 0xffff,
};

#ifndef DEFINED_TOS_AM_GROUP
#define DEFINED_TOS_AM_GROUP 0x22
#endif

#ifndef DEFINED_TOS_AM_ADDRESS
#define DEFINED_TOS_AM_ADDRESS 1
#endif

enum {
  TOS_AM_GROUP = DEFINED_TOS_AM_GROUP,
  TOS_AM_ADDRESS = DEFINED_TOS_AM_ADDRESS
};

#define UQ_AMQUEUE_SEND "amqueue.send"

#endif





/*
 *  @author Rodrigo Fonseca
 *  @date   $Date: 2006/12/12 18:23:29 $
 */
#ifndef COLLECTION_H
#define COLLECTION_H

enum {
    AM_COLLECTION_DATA = 20,
    AM_COLLECTION_CONTROL = 21,
    AM_COLLECTION_DEBUG = 22,
};

typedef uint8_t collection_id_t;
typedef nx_uint8_t nx_collection_id_t;

#endif





#ifndef CTP_H
#define CTP_H

#define UQ_CTP_CLIENT "CtpSenderC.CollectId"

enum {
    // AM types:
    AM_CTP_ROUTING = 0x70,
    AM_CTP_DATA    = 0x71,
    AM_CTP_DEBUG   = 0x72,

    // CTP Options:
    CTP_OPT_PULL      = 0x80, // TEP 123: P field
    CTP_OPT_ECN       = 0x40, // TEP 123: C field
    CTP_OPT_ALL       = 0xff
};

typedef nx_uint8_t nx_ctp_options_t;
typedef uint8_t ctp_options_t;

typedef nx_struct ctp_data_header{
  nx_ctp_options_t    options;
  nx_uint8_t          thl;
  nx_uint16_t         etx;
  nx_am_addr_t        origin;
  nx_uint8_t          originSeqNo;
  nx_collection_id_t  type;
  nx_uint8_t (COUNT(0) data)[0]; // Deputy place-holder, field will probably be removed when we Deputize Ctp
} ctp_data_header_t;

typedef nx_struct {
  nx_ctp_options_t    options;
  nx_am_addr_t        parent;
  nx_uint16_t         etx;
  nx_uint8_t (COUNT(0) data)[0]; // Deputy place-holder, field will probably be removed when we Deputize Ctp
} ctp_routing_header_t;

#endif







#ifndef _COLLECTION_UART_MSG
#define _COLLECTION_UART_MSG

//Comment format ->   :meaning:args
enum {
    NET_C_DEBUG_STARTED = 0xDE,

    NET_C_FE_MSG_POOL_EMPTY = 0x10,    //::no args
    NET_C_FE_SEND_QUEUE_FULL = 0x11,   //::no args
    NET_C_FE_NO_ROUTE = 0x12,          //::no args
    NET_C_FE_SUBSEND_OFF = 0x13,
    NET_C_FE_SUBSEND_BUSY = 0x14,
    NET_C_FE_BAD_SENDDONE = 0x15,
    NET_C_FE_QENTRY_POOL_EMPTY = 0x16,
    NET_C_FE_SUBSEND_SIZE = 0x17,
    NET_C_FE_LOOP_DETECTED = 0x18,
    NET_C_FE_SEND_BUSY = 0x19,

    NET_C_FE_SENDQUEUE_EMPTY = 0x50,
    NET_C_FE_PUT_MSGPOOL_ERR = 0x51,
    NET_C_FE_PUT_QEPOOL_ERR = 0x52,
    NET_C_FE_GET_MSGPOOL_ERR = 0x53,
    NET_C_FE_GET_QEPOOL_ERR = 0x54,

    NET_C_FE_SENT_MSG = 0x20,  //:app. send       :msg uid, origin, next_hop
    NET_C_FE_RCV_MSG =  0x21,  //:next hop receive:msg uid, origin, last_hop
    NET_C_FE_FWD_MSG =  0x22,  //:fwd msg         :msg uid, origin, next_hop
    NET_C_FE_DST_MSG =  0x23,  //:base app. recv  :msg_uid, origin, last_hop
    NET_C_FE_SENDDONE_FAIL = 0x24,
    NET_C_FE_SENDDONE_WAITACK = 0x25,
    NET_C_FE_SENDDONE_FAIL_ACK_SEND = 0x26,
    NET_C_FE_SENDDONE_FAIL_ACK_FWD  = 0x27,
    NET_C_FE_DUPLICATE_CACHE = 0x28,  //dropped duplicate packet seen in cache
    NET_C_FE_DUPLICATE_QUEUE = 0x29,  //dropped duplicate packet seen in queue
    NET_C_FE_DUPLICATE_CACHE_AT_SEND = 0x2A,  //dropped duplicate packet seen in cache


    NET_C_TREE_NO_ROUTE   = 0x30,   //:        :no args
    NET_C_TREE_NEW_PARENT = 0x31,   //:        :parent_id, hopcount, metric
    NET_C_TREE_ROUTE_INFO = 0x32,   //:periodic:parent_id, hopcount, metric
    NET_C_TREE_SENT_BEACON = 0x33,
    NET_C_TREE_RCV_BEACON = 0x34,

    NET_C_DBG_1 = 0x40,             //:any     :uint16_t a
    NET_C_DBG_2 = 0x41,             //:any     :uint16_t a, b, c
    NET_C_DBG_3 = 0x42,             //:any     :uint16_t a, b, c
};

typedef nx_struct CollectionDebugMsg {
    nx_uint8_t type;
    nx_union {
        nx_uint16_t arg;
        nx_struct {
            nx_uint16_t msg_uid;   
            nx_am_addr_t origin;
            nx_am_addr_t other_node;
        } msg;
        nx_struct {
            nx_am_addr_t parent;
            nx_uint8_t hopcount;
            nx_uint16_t metric;
        } route_info;
        nx_struct {
            nx_uint16_t a;
            nx_uint16_t b;
            nx_uint16_t c;
        } dbg;
    } data;
    nx_uint16_t seqno;
} CollectionDebugMsg;

#endif







#endif



