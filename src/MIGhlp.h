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



#endif
