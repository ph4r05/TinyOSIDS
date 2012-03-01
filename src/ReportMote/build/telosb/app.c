#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 151 "/opt/msp430/lib/gcc-lib/msp430/3.2.3/include/stddef.h" 3
typedef int ptrdiff_t;
#line 213
typedef unsigned int size_t;
#line 325
typedef int wchar_t;
# 8 "/usr/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
}  ;
#line 14
struct __nesc_attr_one_nok {
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
}  ;
# 38 "/opt/msp430/msp430/include/sys/inttypes.h" 3
typedef signed char int8_t;
typedef unsigned char uint8_t;

typedef int int16_t;
typedef unsigned int uint16_t;

typedef long int32_t;
typedef unsigned long uint32_t;

typedef long long int64_t;
typedef unsigned long long uint64_t;




typedef int16_t intptr_t;
typedef uint16_t uintptr_t;
# 235 "/usr/lib/ncc/nesc_nx.h"
static __inline uint8_t __nesc_ntoh_uint8(const void * source)  ;




static __inline uint8_t __nesc_hton_uint8(void * target, uint8_t value)  ;





static __inline uint8_t __nesc_ntoh_leuint8(const void * source)  ;




static __inline uint8_t __nesc_hton_leuint8(void * target, uint8_t value)  ;





static __inline int8_t __nesc_ntoh_int8(const void * source)  ;
#line 257
static __inline int8_t __nesc_hton_int8(void * target, int8_t value)  ;






static __inline uint16_t __nesc_ntoh_uint16(const void * source)  ;




static __inline uint16_t __nesc_hton_uint16(void * target, uint16_t value)  ;






static __inline uint16_t __nesc_ntoh_leuint16(const void * source)  ;




static __inline uint16_t __nesc_hton_leuint16(void * target, uint16_t value)  ;






static __inline int16_t __nesc_ntoh_int16(const void * source)  ;
#line 288
static __inline int16_t __nesc_hton_int16(void * target, int16_t value)  ;





static __inline uint32_t __nesc_ntoh_uint32(const void * source)  ;






static __inline uint32_t __nesc_hton_uint32(void * target, uint32_t value)  ;
#line 385
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 41 "/opt/msp430/msp430/include/sys/types.h" 3
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef u_int64_t u_quad_t;
typedef int64_t quad_t;
typedef quad_t *qaddr_t;

typedef char *caddr_t;
typedef const char *c_caddr_t;
typedef volatile char *v_caddr_t;
typedef u_int32_t fixpt_t;
typedef u_int32_t gid_t;
typedef u_int32_t in_addr_t;
typedef u_int16_t in_port_t;
typedef u_int32_t ino_t;
typedef long key_t;
typedef u_int16_t mode_t;
typedef u_int16_t nlink_t;
typedef quad_t rlim_t;
typedef int32_t segsz_t;
typedef int32_t swblk_t;
typedef int32_t ufs_daddr_t;
typedef int32_t ufs_time_t;
typedef u_int32_t uid_t;
# 40 "/opt/msp430/msp430/include/string.h" 3
extern void *memset(void *arg_0xf76fc220, int arg_0xf76fc378, size_t arg_0xf76fc510);
#line 61
extern void *memset(void *arg_0xf76ef118, int arg_0xf76ef270, size_t arg_0xf76ef408);
# 59 "/opt/msp430/msp430/include/stdlib.h" 3
#line 55
typedef struct __nesc_unnamed4242 {

  int quot;
  int rem;
} div_t;







#line 63
typedef struct __nesc_unnamed4243 {

  long quot;
  long rem;
} ldiv_t;
# 122 "/opt/msp430/msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/opt/msp430/msp430/include/sys/_types.h" 3
typedef long _off_t;
typedef long _ssize_t;
# 28 "/opt/msp430/msp430/include/sys/reent.h" 3
typedef __uint32_t __ULong;


struct _glue {

  struct _glue *_next;
  int _niobs;
  struct __sFILE *_iobs;
};

struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _atexit {
  struct _atexit *_next;
  int _ind;
  void (*_fns[32])(void );
};








struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 116
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;


  void *_cookie;

  int (*_read)(void *_cookie, char *_buf, int _n);
  int (*_write)(void *_cookie, const char *_buf, int _n);

  _fpos_t (*_seek)(void *_cookie, _fpos_t _offset, int _whence);
  int (*_close)(void *_cookie);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;

  struct _reent *_data;
};
#line 174
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};









struct _reent {


  int _errno;




  struct __sFILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *arg_0xf76d0510);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0xf76ceb88);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/opt/msp430/msp430/include/math.h" 3
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
#line 208
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 261
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 23 "/opt/tinyos-2.x/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 24
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;
uint16_t TOS_NODE_ID = 1;






struct __nesc_attr_atmostonce {
};
#line 35
struct __nesc_attr_atleastonce {
};
#line 36
struct __nesc_attr_exactlyonce {
};
# 40 "/opt/tinyos-2.x/tos/types/TinyError.h"
enum __nesc_unnamed4248 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9, 
  ENOMEM = 10, 
  ENOACK = 11, 
  ELAST = 11
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2)  ;
# 39 "/opt/msp430/msp430/include/msp430/iostructures.h" 3
#line 27
typedef union port {
  volatile unsigned char reg_p;
  volatile struct __nesc_unnamed4249 {
    unsigned char __p0 : 1, 
    __p1 : 1, 
    __p2 : 1, 
    __p3 : 1, 
    __p4 : 1, 
    __p5 : 1, 
    __p6 : 1, 
    __p7 : 1;
  } __pin;
} __attribute((packed))  ioregister_t;
#line 108
struct port_full_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t ifg;
  ioregister_t ies;
  ioregister_t ie;
  ioregister_t sel;
};









struct port_simple_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t sel;
};




struct port_full_t;



struct port_full_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;
# 116 "/opt/msp430/msp430/include/msp430/gpio.h" 3
volatile unsigned char P1OUT __asm ("0x0021");

volatile unsigned char P1DIR __asm ("0x0022");

volatile unsigned char P1IFG __asm ("0x0023");

volatile unsigned char P1IES __asm ("0x0024");

volatile unsigned char P1IE __asm ("0x0025");

volatile unsigned char P1SEL __asm ("0x0026");










volatile unsigned char P2OUT __asm ("0x0029");

volatile unsigned char P2DIR __asm ("0x002A");

volatile unsigned char P2IFG __asm ("0x002B");



volatile unsigned char P2IE __asm ("0x002D");

volatile unsigned char P2SEL __asm ("0x002E");










volatile unsigned char P3OUT __asm ("0x0019");

volatile unsigned char P3DIR __asm ("0x001A");

volatile unsigned char P3SEL __asm ("0x001B");










volatile unsigned char P4OUT __asm ("0x001D");

volatile unsigned char P4DIR __asm ("0x001E");

volatile unsigned char P4SEL __asm ("0x001F");










volatile unsigned char P5OUT __asm ("0x0031");

volatile unsigned char P5DIR __asm ("0x0032");

volatile unsigned char P5SEL __asm ("0x0033");










volatile unsigned char P6OUT __asm ("0x0035");

volatile unsigned char P6DIR __asm ("0x0036");

volatile unsigned char P6SEL __asm ("0x0037");
# 97 "/opt/msp430/msp430/include/msp430/usart.h" 3
volatile unsigned char U0CTL __asm ("0x0070");

volatile unsigned char U0TCTL __asm ("0x0071");



volatile unsigned char U0MCTL __asm ("0x0073");

volatile unsigned char U0BR0 __asm ("0x0074");

volatile unsigned char U0BR1 __asm ("0x0075");

volatile unsigned char U0RXBUF __asm ("0x0076");
#line 283
volatile unsigned char U1TCTL __asm ("0x0079");
# 27 "/opt/msp430/msp430/include/msp430/timera.h" 3
volatile unsigned int TA0CTL __asm ("0x0160");

volatile unsigned int TA0R __asm ("0x0170");


volatile unsigned int TA0CCTL0 __asm ("0x0162");

volatile unsigned int TA0CCTL1 __asm ("0x0164");
#line 70
volatile unsigned int TA0CCTL2 __asm ("0x0166");
#line 127
#line 118
typedef struct __nesc_unnamed4250 {
  volatile unsigned 
  taifg : 1, 
  taie : 1, 
  taclr : 1, 
  dummy : 1, 
  tamc : 2, 
  taid : 2, 
  tassel : 2;
} __attribute((packed))  tactl_t;
#line 143
#line 129
typedef struct __nesc_unnamed4251 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  dummy : 1, 
  scci : 1, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tacctl_t;


struct timera_t {
  tactl_t ctl;
  tacctl_t cctl0;
  tacctl_t cctl1;
  tacctl_t cctl2;
  volatile unsigned dummy[4];
  volatile unsigned tar;
  volatile unsigned taccr0;
  volatile unsigned taccr1;
  volatile unsigned taccr2;
};



struct timera_t;
# 26 "/opt/msp430/msp430/include/msp430/timerb.h" 3
volatile unsigned int TBR __asm ("0x0190");


volatile unsigned int TBCCTL0 __asm ("0x0182");





volatile unsigned int TBCCR0 __asm ("0x0192");
#line 76
#line 64
typedef struct __nesc_unnamed4252 {
  volatile unsigned 
  tbifg : 1, 
  tbie : 1, 
  tbclr : 1, 
  dummy1 : 1, 
  tbmc : 2, 
  tbid : 2, 
  tbssel : 2, 
  dummy2 : 1, 
  tbcntl : 2, 
  tbclgrp : 2;
} __attribute((packed))  tbctl_t;
#line 91
#line 78
typedef struct __nesc_unnamed4253 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  clld : 2, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tbcctl_t;


struct timerb_t {
  tbctl_t ctl;
  tbcctl_t cctl0;
  tbcctl_t cctl1;
  tbcctl_t cctl2;

  tbcctl_t cctl3;
  tbcctl_t cctl4;
  tbcctl_t cctl5;
  tbcctl_t cctl6;



  volatile unsigned tbr;
  volatile unsigned tbccr0;
  volatile unsigned tbccr1;
  volatile unsigned tbccr2;

  volatile unsigned tbccr3;
  volatile unsigned tbccr4;
  volatile unsigned tbccr5;
  volatile unsigned tbccr6;
};





struct timerb_t;
# 20 "/opt/msp430/msp430/include/msp430/basic_clock.h" 3
volatile unsigned char DCOCTL __asm ("0x0056");

volatile unsigned char BCSCTL1 __asm ("0x0057");

volatile unsigned char BCSCTL2 __asm ("0x0058");
# 18 "/opt/msp430/msp430/include/msp430/adc12.h" 3
volatile unsigned int ADC12CTL0 __asm ("0x01A0");

volatile unsigned int ADC12CTL1 __asm ("0x01A2");
#line 42
#line 30
typedef struct __nesc_unnamed4254 {
  volatile unsigned 
  adc12sc : 1, 
  enc : 1, 
  adc12tovie : 1, 
  adc12ovie : 1, 
  adc12on : 1, 
  refon : 1, 
  r2_5v : 1, 
  msc : 1, 
  sht0 : 4, 
  sht1 : 4;
} __attribute((packed))  adc12ctl0_t;
#line 54
#line 44
typedef struct __nesc_unnamed4255 {
  volatile unsigned 
  adc12busy : 1, 
  conseq : 2, 
  adc12ssel : 2, 
  adc12div : 3, 
  issh : 1, 
  shp : 1, 
  shs : 2, 
  cstartadd : 4;
} __attribute((packed))  adc12ctl1_t;
#line 74
#line 56
typedef struct __nesc_unnamed4256 {
  volatile unsigned 
  bit0 : 1, 
  bit1 : 1, 
  bit2 : 1, 
  bit3 : 1, 
  bit4 : 1, 
  bit5 : 1, 
  bit6 : 1, 
  bit7 : 1, 
  bit8 : 1, 
  bit9 : 1, 
  bit10 : 1, 
  bit11 : 1, 
  bit12 : 1, 
  bit13 : 1, 
  bit14 : 1, 
  bit15 : 1;
} __attribute((packed))  adc12xflg_t;


struct adc12_t {
  adc12ctl0_t ctl0;
  adc12ctl1_t ctl1;
  adc12xflg_t ifg;
  adc12xflg_t ie;
  adc12xflg_t iv;
};




struct adc12_t;
# 71 "/opt/msp430/msp430/include/msp430/common.h" 3
volatile unsigned int WDTCTL __asm ("0x0120");
# 83 "/opt/msp430/msp430/include/msp430x16x.h" 3
volatile unsigned char ME1 __asm ("0x0004");





volatile unsigned char ME2 __asm ("0x0005");
# 158 "/opt/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
static volatile uint8_t U0CTLnr __asm ("0x0070");
static volatile uint8_t I2CTCTLnr __asm ("0x0071");
static volatile uint8_t I2CDCTLnr __asm ("0x0072");
#line 193
typedef uint8_t mcu_power_t  ;
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)  ;


enum __nesc_unnamed4257 {
  MSP430_POWER_ACTIVE = 0, 
  MSP430_POWER_LPM0 = 1, 
  MSP430_POWER_LPM1 = 2, 
  MSP430_POWER_LPM2 = 3, 
  MSP430_POWER_LPM3 = 4, 
  MSP430_POWER_LPM4 = 5
};

static inline void __nesc_disable_interrupt(void )  ;





static inline void __nesc_enable_interrupt(void )  ;




typedef bool __nesc_atomic_t;
__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);






__nesc_atomic_t __nesc_atomic_start(void )   ;







void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)   ;
#line 248
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_float;typedef float __nesc_nxbase_nx_float  ;
# 8 "/opt/tinyos-2.x/tos/platforms/telosb/hardware.h"
enum __nesc_unnamed4258 {
  TOS_SLEEP_NONE = MSP430_POWER_ACTIVE
};
#line 36
static inline void TOSH_SET_SIMO0_PIN()  ;
#line 36
static inline void TOSH_CLR_SIMO0_PIN()  ;
#line 36
static inline void TOSH_MAKE_SIMO0_OUTPUT()  ;
static inline void TOSH_SET_UCLK0_PIN()  ;
#line 37
static inline void TOSH_CLR_UCLK0_PIN()  ;
#line 37
static inline void TOSH_MAKE_UCLK0_OUTPUT()  ;
#line 79
enum __nesc_unnamed4259 {

  TOSH_HUMIDITY_ADDR = 5, 
  TOSH_HUMIDTEMP_ADDR = 3, 
  TOSH_HUMIDITY_RESET = 0x1E
};



static inline void TOSH_SET_FLASH_CS_PIN()  ;
#line 88
static inline void TOSH_CLR_FLASH_CS_PIN()  ;
#line 88
static inline void TOSH_MAKE_FLASH_CS_OUTPUT()  ;
static inline void TOSH_SET_FLASH_HOLD_PIN()  ;
#line 89
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT()  ;
# 43 "../RssiDemoMessages.h"
enum __nesc_unnamed4260 {
  AM_RSSIMSG = 10, 
  AM_PINGMSG = 11, 


  AM_MULTIPINGMSG = 12, 
  AM_MULTIPINGRESPONSEMSG = 13, 
  AM_COMMANDMSG = 14, 

  AM_MULTIPINGRESPONSEREPORTMSG = 16, 
  AM_MULTIPINGRESPONSETINYREPORTMSG = 17
};
#line 76
#line 58
typedef nx_struct RssiMsg {
  nx_uint8_t nodeid;



  nx_uint8_t dstnodeid;


  nx_uint16_t counter;





  nx_int16_t rssi;


  nx_int16_t rssiFromBase;
} __attribute__((packed)) RssiMsg;








#line 80
typedef nx_struct PingMsg {
  nx_uint8_t nodeid;
  nx_uint16_t counter;
  nx_uint8_t txpower;
  nx_uint8_t channel;
} __attribute__((packed)) PingMsg;
#line 107
#line 89
typedef nx_struct MultiPingMsg {




  nx_uint16_t counter;


  nx_uint8_t txpower;


  nx_uint8_t channel;


  nx_uint16_t packets;


  nx_uint16_t delay;
} __attribute__((packed)) MultiPingMsg;










#line 109
typedef nx_struct MultiPingResponseMsg {

  nx_uint16_t counter;





  nx_int16_t rssi;
} __attribute__((packed)) MultiPingResponseMsg;





#line 120
typedef nx_struct MultiPingResponseReportStruct {
  nx_uint8_t nodeid;
  nx_uint8_t nodecounter;
  nx_int16_t rssi;
} __attribute__((packed)) MultiPingResponseReportStruct;
#line 154
#line 140
typedef nx_struct MultiPingResponseReportMsg {

  nx_uint16_t counter;


  nx_uint8_t datanum;





  nx_uint16_t nodeid[4];
  nx_uint16_t nodecounter[4];
  nx_int16_t rssi[4];
} __attribute__((packed)) MultiPingResponseReportMsg;
#line 172
#line 158
typedef nx_struct MassReportMsg {

  nx_uint16_t counter;


  nx_uint8_t datanum;





  nx_uint8_t nodeid[4];
  nx_uint16_t nodecounter[4];
  nx_int16_t rssi[4];
} __attribute__((packed)) MassReportMsg;
#line 187
#line 176
typedef nx_struct MultiPingResponseTinyReportMsg {

  nx_uint16_t counter;





  nx_uint16_t nodeid;
  nx_uint16_t nodecounter;
  nx_int16_t rssi;
} __attribute__((packed)) MultiPingResponseTinyReportMsg;
#line 257
#line 232
typedef nx_struct CommandMsg {

  nx_uint8_t command_code;



  nx_uint8_t command_version;





  nx_uint16_t command_id;



  nx_uint8_t reply_on_command;
  nx_uint16_t reply_on_command_id;


  nx_uint16_t command_data;



  nx_uint16_t command_data_next[4];
} __attribute__((packed)) CommandMsg;




enum __nesc_unnamed4261 {
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


  COMMAND_GETSENSORREADING = 20, 
  COMMAND_SENSORREADING = 21, 


  COMMAND_SETPIN = 22, 
  COMMAND_GETPIN = 23, 





  COMMAND_FETCHSETTINGS = 25
};









enum __nesc_unnamed4262 {
  NODE_REPORTING = 1, 
  NODE_TALKING = 2, 
  NODE_BS = 3, 
  NODE_DEAD = 4
};





enum __nesc_unnamed4263 {
  REPORTING_MEDIUM = 1, 
  REPORTING_TINY = 2, 
  REPORTING_MASS = 3
};




enum __nesc_unnamed4264 {








  SCALE_LAST_LINE = 0x1000, 
  MASK_LAST_LINE = 0x1000, 
  SCALE_TYPE = 0x0100, 
  MASK_TYPE = 0x0F00, 
  SCALE_VERB = 0x0010, 
  MASK_VERB = 0x00F0, 
  SCALE_SELECT_MSG = 0x0008, 
  MASK_SELECT_MSG = 0x0008, 
  SCALE_MSG_LOGIC = 0x0001, 
  MASK_MSG_LOGIC = 0x0007, 






  SCALE_TYPE_READ = 0x0400, 
  MASK_TYPE_READ = 0x1C00, 
  SCALE_NUM_ALERT = 0x0001, 
  MASK_NUM_ALERT = 0x03FF
};
# 39 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420.h"
typedef uint8_t cc2420_status_t;
#line 93
#line 87
typedef nx_struct security_header_t {
  unsigned char __nesc_filler0[1];


  nx_uint32_t frameCounter;
  nx_uint8_t keyID[1];
} __attribute__((packed)) security_header_t;
#line 113
#line 95
typedef nx_struct cc2420_header_t {
  nxle_uint8_t length;
  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;







  nxle_uint8_t network;


  nxle_uint8_t type;
} __attribute__((packed)) cc2420_header_t;





#line 118
typedef nx_struct cc2420_footer_t {
} __attribute__((packed)) cc2420_footer_t;
#line 143
#line 128
typedef nx_struct cc2420_metadata_t {
  nx_uint8_t rssi;
  nx_uint8_t lqi;
  nx_uint8_t tx_power;
  nx_bool crc;
  nx_bool ack;
  nx_bool timesync;
  nx_uint32_t timestamp;
  nx_uint16_t rxInterval;
} __attribute__((packed)) 





cc2420_metadata_t;





#line 146
typedef nx_struct cc2420_packet_t {
  cc2420_header_t packet;
  nx_uint8_t data[];
} __attribute__((packed)) cc2420_packet_t;
#line 179
enum __nesc_unnamed4265 {

  MAC_HEADER_SIZE = sizeof(cc2420_header_t ) - 1, 

  MAC_FOOTER_SIZE = sizeof(uint16_t ), 

  MAC_PACKET_SIZE = MAC_HEADER_SIZE + 28 + MAC_FOOTER_SIZE, 

  CC2420_SIZE = MAC_HEADER_SIZE + MAC_FOOTER_SIZE, 

  AM_OVERHEAD = 2
};

enum cc2420_enums {
  CC2420_TIME_ACK_TURNAROUND = 7, 
  CC2420_TIME_VREN = 20, 
  CC2420_TIME_SYMBOL = 2, 
  CC2420_BACKOFF_PERIOD = 20 / CC2420_TIME_SYMBOL, 
  CC2420_MIN_BACKOFF = 20 / CC2420_TIME_SYMBOL, 
  CC2420_ACK_WAIT_DELAY = 256
};

enum cc2420_status_enums {
  CC2420_STATUS_RSSI_VALID = 1 << 1, 
  CC2420_STATUS_LOCK = 1 << 2, 
  CC2420_STATUS_TX_ACTIVE = 1 << 3, 
  CC2420_STATUS_ENC_BUSY = 1 << 4, 
  CC2420_STATUS_TX_UNDERFLOW = 1 << 5, 
  CC2420_STATUS_XOSC16M_STABLE = 1 << 6
};

enum cc2420_config_reg_enums {
  CC2420_SNOP = 0x00, 
  CC2420_SXOSCON = 0x01, 
  CC2420_STXCAL = 0x02, 
  CC2420_SRXON = 0x03, 
  CC2420_STXON = 0x04, 
  CC2420_STXONCCA = 0x05, 
  CC2420_SRFOFF = 0x06, 
  CC2420_SXOSCOFF = 0x07, 
  CC2420_SFLUSHRX = 0x08, 
  CC2420_SFLUSHTX = 0x09, 
  CC2420_SACK = 0x0a, 
  CC2420_SACKPEND = 0x0b, 
  CC2420_SRXDEC = 0x0c, 
  CC2420_STXENC = 0x0d, 
  CC2420_SAES = 0x0e, 
  CC2420_MAIN = 0x10, 
  CC2420_MDMCTRL0 = 0x11, 
  CC2420_MDMCTRL1 = 0x12, 
  CC2420_RSSI = 0x13, 
  CC2420_SYNCWORD = 0x14, 
  CC2420_TXCTRL = 0x15, 
  CC2420_RXCTRL0 = 0x16, 
  CC2420_RXCTRL1 = 0x17, 
  CC2420_FSCTRL = 0x18, 
  CC2420_SECCTRL0 = 0x19, 
  CC2420_SECCTRL1 = 0x1a, 
  CC2420_BATTMON = 0x1b, 
  CC2420_IOCFG0 = 0x1c, 
  CC2420_IOCFG1 = 0x1d, 
  CC2420_MANFIDL = 0x1e, 
  CC2420_MANFIDH = 0x1f, 
  CC2420_FSMTC = 0x20, 
  CC2420_MANAND = 0x21, 
  CC2420_MANOR = 0x22, 
  CC2420_AGCCTRL = 0x23, 
  CC2420_AGCTST0 = 0x24, 
  CC2420_AGCTST1 = 0x25, 
  CC2420_AGCTST2 = 0x26, 
  CC2420_FSTST0 = 0x27, 
  CC2420_FSTST1 = 0x28, 
  CC2420_FSTST2 = 0x29, 
  CC2420_FSTST3 = 0x2a, 
  CC2420_RXBPFTST = 0x2b, 
  CC2420_FMSTATE = 0x2c, 
  CC2420_ADCTST = 0x2d, 
  CC2420_DACTST = 0x2e, 
  CC2420_TOPTST = 0x2f, 
  CC2420_TXFIFO = 0x3e, 
  CC2420_RXFIFO = 0x3f
};

enum cc2420_ram_addr_enums {
  CC2420_RAM_TXFIFO = 0x000, 
  CC2420_RAM_RXFIFO = 0x080, 
  CC2420_RAM_KEY0 = 0x100, 
  CC2420_RAM_RXNONCE = 0x110, 
  CC2420_RAM_SABUF = 0x120, 
  CC2420_RAM_KEY1 = 0x130, 
  CC2420_RAM_TXNONCE = 0x140, 
  CC2420_RAM_CBCSTATE = 0x150, 
  CC2420_RAM_IEEEADR = 0x160, 
  CC2420_RAM_PANID = 0x168, 
  CC2420_RAM_SHORTADR = 0x16a
};

enum cc2420_nonce_enums {
  CC2420_NONCE_BLOCK_COUNTER = 0, 
  CC2420_NONCE_KEY_SEQ_COUNTER = 2, 
  CC2420_NONCE_FRAME_COUNTER = 3, 
  CC2420_NONCE_SOURCE_ADDRESS = 7, 
  CC2420_NONCE_FLAGS = 15
};

enum cc2420_main_enums {
  CC2420_MAIN_RESETn = 15, 
  CC2420_MAIN_ENC_RESETn = 14, 
  CC2420_MAIN_DEMOD_RESETn = 13, 
  CC2420_MAIN_MOD_RESETn = 12, 
  CC2420_MAIN_FS_RESETn = 11, 
  CC2420_MAIN_XOSC16M_BYPASS = 0
};

enum cc2420_mdmctrl0_enums {
  CC2420_MDMCTRL0_RESERVED_FRAME_MODE = 13, 
  CC2420_MDMCTRL0_PAN_COORDINATOR = 12, 
  CC2420_MDMCTRL0_ADR_DECODE = 11, 
  CC2420_MDMCTRL0_CCA_HYST = 8, 
  CC2420_MDMCTRL0_CCA_MOD = 6, 
  CC2420_MDMCTRL0_AUTOCRC = 5, 
  CC2420_MDMCTRL0_AUTOACK = 4, 
  CC2420_MDMCTRL0_PREAMBLE_LENGTH = 0
};

enum cc2420_mdmctrl1_enums {
  CC2420_MDMCTRL1_CORR_THR = 6, 
  CC2420_MDMCTRL1_DEMOD_AVG_MODE = 5, 
  CC2420_MDMCTRL1_MODULATION_MODE = 4, 
  CC2420_MDMCTRL1_TX_MODE = 2, 
  CC2420_MDMCTRL1_RX_MODE = 0
};

enum cc2420_rssi_enums {
  CC2420_RSSI_CCA_THR = 8, 
  CC2420_RSSI_RSSI_VAL = 0
};

enum cc2420_syncword_enums {
  CC2420_SYNCWORD_SYNCWORD = 0
};

enum cc2420_txctrl_enums {
  CC2420_TXCTRL_TXMIXBUF_CUR = 14, 
  CC2420_TXCTRL_TX_TURNAROUND = 13, 
  CC2420_TXCTRL_TXMIX_CAP_ARRAY = 11, 
  CC2420_TXCTRL_TXMIX_CURRENT = 9, 
  CC2420_TXCTRL_PA_CURRENT = 6, 
  CC2420_TXCTRL_RESERVED = 5, 
  CC2420_TXCTRL_PA_LEVEL = 0
};

enum cc2420_rxctrl0_enums {
  CC2420_RXCTRL0_RXMIXBUF_CUR = 12, 
  CC2420_RXCTRL0_HIGH_LNA_GAIN = 10, 
  CC2420_RXCTRL0_MED_LNA_GAIN = 8, 
  CC2420_RXCTRL0_LOW_LNA_GAIN = 6, 
  CC2420_RXCTRL0_HIGH_LNA_CURRENT = 4, 
  CC2420_RXCTRL0_MED_LNA_CURRENT = 2, 
  CC2420_RXCTRL0_LOW_LNA_CURRENT = 0
};

enum cc2420_rxctrl1_enums {
  CC2420_RXCTRL1_RXBPF_LOCUR = 13, 
  CC2420_RXCTRL1_RXBPF_MIDCUR = 12, 
  CC2420_RXCTRL1_LOW_LOWGAIN = 11, 
  CC2420_RXCTRL1_MED_LOWGAIN = 10, 
  CC2420_RXCTRL1_HIGH_HGM = 9, 
  CC2420_RXCTRL1_MED_HGM = 8, 
  CC2420_RXCTRL1_LNA_CAP_ARRAY = 6, 
  CC2420_RXCTRL1_RXMIX_TAIL = 4, 
  CC2420_RXCTRL1_RXMIX_VCM = 2, 
  CC2420_RXCTRL1_RXMIX_CURRENT = 0
};

enum cc2420_rsctrl_enums {
  CC2420_FSCTRL_LOCK_THR = 14, 
  CC2420_FSCTRL_CAL_DONE = 13, 
  CC2420_FSCTRL_CAL_RUNNING = 12, 
  CC2420_FSCTRL_LOCK_LENGTH = 11, 
  CC2420_FSCTRL_LOCK_STATUS = 10, 
  CC2420_FSCTRL_FREQ = 0
};

enum cc2420_secctrl0_enums {
  CC2420_SECCTRL0_RXFIFO_PROTECTION = 9, 
  CC2420_SECCTRL0_SEC_CBC_HEAD = 8, 
  CC2420_SECCTRL0_SEC_SAKEYSEL = 7, 
  CC2420_SECCTRL0_SEC_TXKEYSEL = 6, 
  CC2420_SECCTRL0_SEC_RXKEYSEL = 5, 
  CC2420_SECCTRL0_SEC_M = 2, 
  CC2420_SECCTRL0_SEC_MODE = 0
};

enum cc2420_secctrl1_enums {
  CC2420_SECCTRL1_SEC_TXL = 8, 
  CC2420_SECCTRL1_SEC_RXL = 0
};

enum cc2420_battmon_enums {
  CC2420_BATTMON_BATT_OK = 6, 
  CC2420_BATTMON_BATTMON_EN = 5, 
  CC2420_BATTMON_BATTMON_VOLTAGE = 0
};

enum cc2420_iocfg0_enums {
  CC2420_IOCFG0_BCN_ACCEPT = 11, 
  CC2420_IOCFG0_FIFO_POLARITY = 10, 
  CC2420_IOCFG0_FIFOP_POLARITY = 9, 
  CC2420_IOCFG0_SFD_POLARITY = 8, 
  CC2420_IOCFG0_CCA_POLARITY = 7, 
  CC2420_IOCFG0_FIFOP_THR = 0
};

enum cc2420_iocfg1_enums {
  CC2420_IOCFG1_HSSD_SRC = 10, 
  CC2420_IOCFG1_SFDMUX = 5, 
  CC2420_IOCFG1_CCAMUX = 0
};

enum cc2420_manfidl_enums {
  CC2420_MANFIDL_PARTNUM = 12, 
  CC2420_MANFIDL_MANFID = 0
};

enum cc2420_manfidh_enums {
  CC2420_MANFIDH_VERSION = 12, 
  CC2420_MANFIDH_PARTNUM = 0
};

enum cc2420_fsmtc_enums {
  CC2420_FSMTC_TC_RXCHAIN2RX = 13, 
  CC2420_FSMTC_TC_SWITCH2TX = 10, 
  CC2420_FSMTC_TC_PAON2TX = 6, 
  CC2420_FSMTC_TC_TXEND2SWITCH = 3, 
  CC2420_FSMTC_TC_TXEND2PAOFF = 0
};

enum cc2420_sfdmux_enums {
  CC2420_SFDMUX_SFD = 0, 
  CC2420_SFDMUX_XOSC16M_STABLE = 24
};

enum cc2420_security_enums {
  CC2420_NO_SEC = 0, 
  CC2420_CBC_MAC = 1, 
  CC2420_CTR = 2, 
  CC2420_CCM = 3, 
  NO_SEC = 0, 
  CBC_MAC_4 = 1, 
  CBC_MAC_8 = 2, 
  CBC_MAC_16 = 3, 
  CTR = 4, 
  CCM_4 = 5, 
  CCM_8 = 6, 
  CCM_16 = 7
};


enum __nesc_unnamed4266 {

  CC2420_INVALID_TIMESTAMP = 0x80000000L
};
# 6 "/opt/tinyos-2.x/tos/types/AM.h"
typedef nx_uint8_t nx_am_id_t;
typedef nx_uint8_t nx_am_group_t;
typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;
typedef uint8_t am_group_t;
typedef uint16_t am_addr_t;

enum __nesc_unnamed4267 {
  AM_BROADCAST_ADDR = 0xffff
};









enum __nesc_unnamed4268 {
  TOS_AM_GROUP = 0x22, 
  TOS_AM_ADDRESS = 1
};
# 72 "/opt/tinyos-2.x/tos/lib/serial/Serial.h"
typedef uint8_t uart_id_t;



enum __nesc_unnamed4269 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4270 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4271 {
  SERIAL_PROTO_ACK = 67, 
  SERIAL_PROTO_PACKET_ACK = 68, 
  SERIAL_PROTO_PACKET_NOACK = 69, 
  SERIAL_PROTO_PACKET_UNKNOWN = 255
};
#line 110
#line 98
typedef struct radio_stats {
  uint8_t version;
  uint8_t flags;
  uint8_t reserved;
  uint8_t platform;
  uint16_t MTU;
  uint16_t radio_crc_fail;
  uint16_t radio_queue_drops;
  uint16_t serial_crc_fail;
  uint16_t serial_tx_fail;
  uint16_t serial_short_packets;
  uint16_t serial_proto_drops;
} radio_stats_t;







#line 112
typedef nx_struct serial_header {
  nx_am_addr_t dest;
  nx_am_addr_t src;
  nx_uint8_t length;
  nx_am_group_t group;
  nx_am_id_t type;
} __attribute__((packed)) serial_header_t;




#line 120
typedef nx_struct serial_packet {
  serial_header_t header;
  nx_uint8_t data[];
} __attribute__((packed)) serial_packet_t;



#line 125
typedef nx_struct serial_metadata {
  nx_uint8_t ack;
} __attribute__((packed)) serial_metadata_t;
# 48 "/opt/tinyos-2.x/tos/platforms/telosa/platform_message.h"
#line 45
typedef union message_header {
  cc2420_header_t cc2420;
  serial_header_t serial;
} message_header_t;



#line 50
typedef union TOSRadioFooter {
  cc2420_footer_t cc2420;
} message_footer_t;




#line 54
typedef union TOSRadioMetadata {
  cc2420_metadata_t cc2420;
  serial_metadata_t serial;
} message_metadata_t;
# 19 "/opt/tinyos-2.x/tos/types/message.h"
#line 14
typedef nx_struct message_t {
  nx_uint8_t header[sizeof(message_header_t )];
  nx_uint8_t data[28];
  nx_uint8_t footer[sizeof(message_footer_t )];
  nx_uint8_t metadata[sizeof(message_metadata_t )];
} __attribute__((packed)) message_t;
# 29 "/opt/tinyos-2.x/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4272 {
#line 29
  int notUsed;
} 
#line 29
TMilli;
typedef struct __nesc_unnamed4273 {
#line 30
  int notUsed;
} 
#line 30
T32khz;
typedef struct __nesc_unnamed4274 {
#line 31
  int notUsed;
} 
#line 31
TMicro;
# 33 "/opt/tinyos-2.x/tos/types/Ieee154.h"
typedef uint16_t ieee154_panid_t;
typedef uint16_t ieee154_saddr_t;

enum __nesc_unnamed4275 {
  IEEE154_BROADCAST_ADDR = 0xffff
};
# 38 "/opt/tinyos-2.x/tos/chips/cc2420/IEEE802154.h"
enum ieee154_fcf_enums {
  IEEE154_FCF_FRAME_TYPE = 0, 
  IEEE154_FCF_SECURITY_ENABLED = 3, 
  IEEE154_FCF_FRAME_PENDING = 4, 
  IEEE154_FCF_ACK_REQ = 5, 
  IEEE154_FCF_INTRAPAN = 6, 
  IEEE154_FCF_DEST_ADDR_MODE = 10, 
  IEEE154_FCF_SRC_ADDR_MODE = 14
};

enum ieee154_fcf_type_enums {
  IEEE154_TYPE_BEACON = 0, 
  IEEE154_TYPE_DATA = 1, 
  IEEE154_TYPE_ACK = 2, 
  IEEE154_TYPE_MAC_CMD = 3
};

enum iee154_fcf_addr_mode_enums {
  IEEE154_ADDR_NONE = 0, 
  IEEE154_ADDR_SHORT = 2, 
  IEEE154_ADDR_EXT = 3
};
# 32 "/opt/tinyos-2.x/tos/types/Leds.h"
enum __nesc_unnamed4276 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.h"
enum __nesc_unnamed4277 {
  MSP430TIMER_CM_NONE = 0, 
  MSP430TIMER_CM_RISING = 1, 
  MSP430TIMER_CM_FALLING = 2, 
  MSP430TIMER_CM_BOTH = 3, 

  MSP430TIMER_STOP_MODE = 0, 
  MSP430TIMER_UP_MODE = 1, 
  MSP430TIMER_CONTINUOUS_MODE = 2, 
  MSP430TIMER_UPDOWN_MODE = 3, 

  MSP430TIMER_TACLK = 0, 
  MSP430TIMER_TBCLK = 0, 
  MSP430TIMER_ACLK = 1, 
  MSP430TIMER_SMCLK = 2, 
  MSP430TIMER_INCLK = 3, 

  MSP430TIMER_CLOCKDIV_1 = 0, 
  MSP430TIMER_CLOCKDIV_2 = 1, 
  MSP430TIMER_CLOCKDIV_4 = 2, 
  MSP430TIMER_CLOCKDIV_8 = 3
};
#line 64
#line 51
typedef struct __nesc_unnamed4278 {

  int ccifg : 1;
  int cov : 1;
  int out : 1;
  int cci : 1;
  int ccie : 1;
  int outmod : 3;
  int cap : 1;
  int clld : 2;
  int scs : 1;
  int ccis : 2;
  int cm : 2;
} msp430_compare_control_t;
#line 76
#line 66
typedef struct __nesc_unnamed4279 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} msp430_timer_a_control_t;
#line 91
#line 78
typedef struct __nesc_unnamed4280 {

  int tbifg : 1;
  int tbie : 1;
  int tbclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tbssel : 2;
  int _unused1 : 1;
  int cntl : 2;
  int tbclgrp : 2;
  int _unused2 : 1;
} msp430_timer_b_control_t;
# 56 "/opt/tinyos-2.x/tos/chips/msp430/usart/msp430usart.h"
#line 48
typedef enum __nesc_unnamed4281 {

  USART_NONE = 0, 
  USART_UART = 1, 
  USART_UART_TX = 2, 
  USART_UART_RX = 3, 
  USART_SPI = 4, 
  USART_I2C = 5
} msp430_usartmode_t;










#line 58
typedef struct __nesc_unnamed4282 {
  unsigned int swrst : 1;
  unsigned int mm : 1;
  unsigned int sync : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
} __attribute((packed))  msp430_uctl_t;









#line 69
typedef struct __nesc_unnamed4283 {
  unsigned int txept : 1;
  unsigned int stc : 1;
  unsigned int txwake : 1;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
} __attribute((packed))  msp430_utctl_t;










#line 79
typedef struct __nesc_unnamed4284 {
  unsigned int rxerr : 1;
  unsigned int rxwake : 1;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int brk : 1;
  unsigned int oe : 1;
  unsigned int pe : 1;
  unsigned int fe : 1;
} __attribute((packed))  msp430_urctl_t;
#line 116
#line 99
typedef struct __nesc_unnamed4285 {
  unsigned int ubr : 16;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int  : 3;

  unsigned int  : 1;
  unsigned int stc : 1;
  unsigned int  : 2;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
  unsigned int  : 0;
} msp430_spi_config_t;





#line 118
typedef struct __nesc_unnamed4286 {
  uint16_t ubr;
  uint8_t uctl;
  uint8_t utctl;
} msp430_spi_registers_t;




#line 124
typedef union __nesc_unnamed4287 {
  msp430_spi_config_t spiConfig;
  msp430_spi_registers_t spiRegisters;
} msp430_spi_union_config_t;

msp430_spi_union_config_t msp430_spi_default_config = { 
{ 
.ubr = 0x0002, 
.ssel = 0x02, 
.clen = 1, 
.listen = 0, 
.mm = 1, 
.ckph = 1, 
.ckpl = 0, 
.stc = 1 } };
#line 169
#line 150
typedef enum __nesc_unnamed4288 {

  UBR_32KHZ_1200 = 0x001B, UMCTL_32KHZ_1200 = 0x94, 
  UBR_32KHZ_1800 = 0x0012, UMCTL_32KHZ_1800 = 0x84, 
  UBR_32KHZ_2400 = 0x000D, UMCTL_32KHZ_2400 = 0x6D, 
  UBR_32KHZ_4800 = 0x0006, UMCTL_32KHZ_4800 = 0x77, 
  UBR_32KHZ_9600 = 0x0003, UMCTL_32KHZ_9600 = 0x29, 

  UBR_1MHZ_1200 = 0x0369, UMCTL_1MHZ_1200 = 0x7B, 
  UBR_1MHZ_1800 = 0x0246, UMCTL_1MHZ_1800 = 0x55, 
  UBR_1MHZ_2400 = 0x01B4, UMCTL_1MHZ_2400 = 0xDF, 
  UBR_1MHZ_4800 = 0x00DA, UMCTL_1MHZ_4800 = 0xAA, 
  UBR_1MHZ_9600 = 0x006D, UMCTL_1MHZ_9600 = 0x44, 
  UBR_1MHZ_19200 = 0x0036, UMCTL_1MHZ_19200 = 0xB5, 
  UBR_1MHZ_38400 = 0x001B, UMCTL_1MHZ_38400 = 0x94, 
  UBR_1MHZ_57600 = 0x0012, UMCTL_1MHZ_57600 = 0x84, 
  UBR_1MHZ_76800 = 0x000D, UMCTL_1MHZ_76800 = 0x6D, 
  UBR_1MHZ_115200 = 0x0009, UMCTL_1MHZ_115200 = 0x10, 
  UBR_1MHZ_230400 = 0x0004, UMCTL_1MHZ_230400 = 0x55
} msp430_uart_rate_t;
#line 200
#line 171
typedef struct __nesc_unnamed4289 {
  unsigned int ubr : 16;

  unsigned int umctl : 8;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
  unsigned int  : 0;

  unsigned int  : 3;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int  : 1;

  unsigned int  : 2;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int  : 4;
  unsigned int  : 0;

  unsigned int utxe : 1;
  unsigned int urxe : 1;
} msp430_uart_config_t;








#line 202
typedef struct __nesc_unnamed4290 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl;
  uint8_t utctl;
  uint8_t urctl;
  uint8_t ume;
} msp430_uart_registers_t;




#line 211
typedef union __nesc_unnamed4291 {
  msp430_uart_config_t uartConfig;
  msp430_uart_registers_t uartRegisters;
} msp430_uart_union_config_t;
#line 248
#line 240
typedef struct __nesc_unnamed4292 {
  unsigned int i2cstt : 1;
  unsigned int i2cstp : 1;
  unsigned int i2cstb : 1;
  unsigned int i2cctrx : 1;
  unsigned int i2cssel : 2;
  unsigned int i2ccrm : 1;
  unsigned int i2cword : 1;
} __attribute((packed))  msp430_i2ctctl_t;
#line 276
#line 253
typedef struct __nesc_unnamed4293 {
  unsigned int  : 1;
  unsigned int mst : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int xa : 1;
  unsigned int  : 1;
  unsigned int txdmaen : 1;
  unsigned int rxdmaen : 1;

  unsigned int  : 4;
  unsigned int i2cssel : 2;
  unsigned int i2crm : 1;
  unsigned int i2cword : 1;

  unsigned int i2cpsc : 8;

  unsigned int i2csclh : 8;

  unsigned int i2cscll : 8;

  unsigned int i2coa : 10;
  unsigned int  : 6;
} msp430_i2c_config_t;








#line 278
typedef struct __nesc_unnamed4294 {
  uint8_t uctl;
  uint8_t i2ctctl;
  uint8_t i2cpsc;
  uint8_t i2csclh;
  uint8_t i2cscll;
  uint16_t i2coa;
} msp430_i2c_registers_t;




#line 287
typedef union __nesc_unnamed4295 {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;
#line 309
typedef uint8_t uart_speed_t;
typedef uint8_t uart_parity_t;
typedef uint8_t uart_duplex_t;

enum __nesc_unnamed4296 {
  TOS_UART_1200 = 0, 
  TOS_UART_1800 = 1, 
  TOS_UART_2400 = 2, 
  TOS_UART_4800 = 3, 
  TOS_UART_9600 = 4, 
  TOS_UART_19200 = 5, 
  TOS_UART_38400 = 6, 
  TOS_UART_57600 = 7, 
  TOS_UART_76800 = 8, 
  TOS_UART_115200 = 9, 
  TOS_UART_230400 = 10
};

enum __nesc_unnamed4297 {
  TOS_UART_OFF, 
  TOS_UART_RONLY, 
  TOS_UART_TONLY, 
  TOS_UART_DUPLEX
};

enum __nesc_unnamed4298 {
  TOS_UART_PARITY_NONE, 
  TOS_UART_PARITY_EVEN, 
  TOS_UART_PARITY_ODD
};
# 33 "/opt/tinyos-2.x/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
# 32 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420TimeSyncMessage.h"
typedef nx_uint32_t timesync_radio_t;





#line 34
typedef struct timesync_footer_t {

  nx_am_id_t type;
  timesync_radio_t timestamp;
} timesync_footer_t;
# 54 "Reset.h"
static inline void resetMote();
# 37 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.h"
enum __nesc_unnamed4299 {
  SHT11_TEMPERATURE_BITS = 14, 
  SHT11_HUMIDITY_BITS = 12
};

enum __nesc_unnamed4300 {
  SHT11_STATUS_LOW_RES_BIT = 1 << 0, 
  SHT11_STATUS_NO_RELOAD_BIT = 1 << 1, 
  SHT11_STATUS_HEATER_ON_BIT = 1 << 2, 
  SHT11_STATUS_LOW_BATTERY_BIT = 1 << 6
};
# 59 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12.h"
#line 48
typedef struct __nesc_unnamed4301 {

  unsigned int inch : 4;
  unsigned int sref : 3;
  unsigned int ref2_5v : 1;
  unsigned int adc12ssel : 2;
  unsigned int adc12div : 3;
  unsigned int sht : 4;
  unsigned int sampcon_ssel : 2;
  unsigned int sampcon_id : 2;
  unsigned int  : 0;
} msp430adc12_channel_config_t;








#line 61
typedef struct __nesc_unnamed4302 {


  volatile unsigned 
  inch : 4, 
  sref : 3, 
  eos : 1;
} __attribute((packed))  adc12memctl_t;

enum inch_enum {


  INPUT_CHANNEL_A0 = 0, 
  INPUT_CHANNEL_A1 = 1, 
  INPUT_CHANNEL_A2 = 2, 
  INPUT_CHANNEL_A3 = 3, 
  INPUT_CHANNEL_A4 = 4, 
  INPUT_CHANNEL_A5 = 5, 
  INPUT_CHANNEL_A6 = 6, 
  INPUT_CHANNEL_A7 = 7, 
  EXTERNAL_REF_VOLTAGE_CHANNEL = 8, 
  REF_VOLTAGE_NEG_TERMINAL_CHANNEL = 9, 
  TEMPERATURE_DIODE_CHANNEL = 10, 
  SUPPLY_VOLTAGE_HALF_CHANNEL = 11, 
  INPUT_CHANNEL_NONE = 12
};

enum sref_enum {

  REFERENCE_AVcc_AVss = 0, 
  REFERENCE_VREFplus_AVss = 1, 
  REFERENCE_VeREFplus_AVss = 2, 
  REFERENCE_AVcc_VREFnegterm = 4, 
  REFERENCE_VREFplus_VREFnegterm = 5, 
  REFERENCE_VeREFplus_VREFnegterm = 6
};

enum ref2_5v_enum {

  REFVOLT_LEVEL_1_5 = 0, 
  REFVOLT_LEVEL_2_5 = 1, 
  REFVOLT_LEVEL_NONE = 0
};

enum adc12ssel_enum {

  SHT_SOURCE_ADC12OSC = 0, 
  SHT_SOURCE_ACLK = 1, 
  SHT_SOURCE_MCLK = 2, 
  SHT_SOURCE_SMCLK = 3
};

enum adc12div_enum {

  SHT_CLOCK_DIV_1 = 0, 
  SHT_CLOCK_DIV_2 = 1, 
  SHT_CLOCK_DIV_3 = 2, 
  SHT_CLOCK_DIV_4 = 3, 
  SHT_CLOCK_DIV_5 = 4, 
  SHT_CLOCK_DIV_6 = 5, 
  SHT_CLOCK_DIV_7 = 6, 
  SHT_CLOCK_DIV_8 = 7
};

enum sht_enum {

  SAMPLE_HOLD_4_CYCLES = 0, 
  SAMPLE_HOLD_8_CYCLES = 1, 
  SAMPLE_HOLD_16_CYCLES = 2, 
  SAMPLE_HOLD_32_CYCLES = 3, 
  SAMPLE_HOLD_64_CYCLES = 4, 
  SAMPLE_HOLD_96_CYCLES = 5, 
  SAMPLE_HOLD_123_CYCLES = 6, 
  SAMPLE_HOLD_192_CYCLES = 7, 
  SAMPLE_HOLD_256_CYCLES = 8, 
  SAMPLE_HOLD_384_CYCLES = 9, 
  SAMPLE_HOLD_512_CYCLES = 10, 
  SAMPLE_HOLD_768_CYCLES = 11, 
  SAMPLE_HOLD_1024_CYCLES = 12
};

enum sampcon_ssel_enum {

  SAMPCON_SOURCE_TACLK = 0, 
  SAMPCON_SOURCE_ACLK = 1, 
  SAMPCON_SOURCE_SMCLK = 2, 
  SAMPCON_SOURCE_INCLK = 3
};

enum sampcon_id_enum {

  SAMPCON_CLOCK_DIV_1 = 0, 
  SAMPCON_CLOCK_DIV_2 = 1, 
  SAMPCON_CLOCK_DIV_3 = 2, 
  SAMPCON_CLOCK_DIV_4 = 3
};
enum CC2420ActiveMessageC____nesc_unnamed4303 {
  CC2420ActiveMessageC__CC2420_AM_SEND_ID = 0U
};
typedef T32khz CC2420ControlP__StartupTimer__precision_tag;
typedef uint32_t CC2420ControlP__StartupTimer__size_type;
typedef uint16_t CC2420ControlP__ReadRssi__val_t;
enum /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Timer*/Msp430Timer32khzC__0____nesc_unnamed4304 {
  Msp430Timer32khzC__0__ALARM_ID = 0U
};
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__frequency_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__frequency_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__precision_tag;
typedef uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC__0__to_precision_tag;
typedef uint32_t /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC__0__from_precision_tag;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC__0__upper_count_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC__0__from_precision_tag /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__size_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC__0__to_precision_tag /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__size_type;
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_precision_tag;
typedef uint32_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type;
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_precision_tag;
typedef uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__size_type;
enum /*CC2420ControlC.Spi*/CC2420SpiC__0____nesc_unnamed4305 {
  CC2420SpiC__0__CLIENT_ID = 0U
};
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0____nesc_unnamed4306 {
  Msp430Spi0C__0__CLIENT_ID = 0U
};
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0____nesc_unnamed4307 {
  Msp430Usart0C__0__CLIENT_ID = 0U
};
enum /*CC2420ControlC.SyncSpiC*/CC2420SpiC__1____nesc_unnamed4308 {
  CC2420SpiC__1__CLIENT_ID = 1U
};
enum /*CC2420ControlC.RssiResource*/CC2420SpiC__2____nesc_unnamed4309 {
  CC2420SpiC__2__CLIENT_ID = 2U
};
typedef T32khz CC2420TransmitP__PacketTimeStamp__precision_tag;
typedef uint32_t CC2420TransmitP__PacketTimeStamp__size_type;
typedef T32khz CC2420TransmitP__BackoffTimer__precision_tag;
typedef uint32_t CC2420TransmitP__BackoffTimer__size_type;
enum /*CC2420TransmitC.Spi*/CC2420SpiC__3____nesc_unnamed4310 {
  CC2420SpiC__3__CLIENT_ID = 3U
};
typedef T32khz CC2420ReceiveP__PacketTimeStamp__precision_tag;
typedef uint32_t CC2420ReceiveP__PacketTimeStamp__size_type;
typedef T32khz CC2420PacketP__PacketTimeStamp32khz__precision_tag;
typedef uint32_t CC2420PacketP__PacketTimeStamp32khz__size_type;
typedef T32khz CC2420PacketP__LocalTime32khz__precision_tag;
typedef TMilli CC2420PacketP__LocalTimeMilli__precision_tag;
typedef TMilli CC2420PacketP__PacketTimeStampMilli__precision_tag;
typedef uint32_t CC2420PacketP__PacketTimeStampMilli__size_type;
typedef T32khz /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag;
typedef /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__precision_tag;
typedef /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__precision_tag;
typedef uint32_t /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__size_type;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__1____nesc_unnamed4311 {
  Msp430Timer32khzC__1__ALARM_ID = 1U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type;
typedef TMilli /*CounterMilli32C.Transform*/TransformCounterC__1__to_precision_tag;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type;
typedef T32khz /*CounterMilli32C.Transform*/TransformCounterC__1__from_precision_tag;
typedef uint16_t /*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__1__upper_count_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__1__from_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__size_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__1__to_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type;
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__precision_tag;
typedef uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__size_type;
enum /*CC2420ReceiveC.Spi*/CC2420SpiC__4____nesc_unnamed4312 {
  CC2420SpiC__4__CLIENT_ID = 4U
};
typedef uint16_t RandomMlcgC__SeedInit__parameter;
enum CC2420TinyosNetworkC____nesc_unnamed4313 {
  CC2420TinyosNetworkC__TINYOS_N_NETWORKS = 1U
};
typedef TMilli SendingMoteC__SendTimer__precision_tag;
typedef TMilli SendingMoteC__BootupTimer__precision_tag;
typedef TMilli SendingMoteC__NoiseFloorTimer__precision_tag;
typedef uint16_t SendingMoteC__Light__val_t;
typedef uint16_t SendingMoteC__ReadRssi__val_t;
typedef TMilli SendingMoteC__SensorReadingTimer__precision_tag;
typedef uint16_t SendingMoteC__Humidity__val_t;
typedef TMilli SendingMoteC__SignalTimer__precision_tag;
typedef uint16_t SendingMoteC__Temperature__val_t;
typedef TMilli SendingMoteC__FlushTimer__precision_tag;
typedef TMilli SendingMoteC__GapTimer__precision_tag;
enum AMQueueP____nesc_unnamed4314 {
  AMQueueP__NUM_CLIENTS = 5U
};
typedef uint16_t /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__val_t;
typedef uint16_t /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__val_t;
typedef TMilli /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__precision_tag;
typedef TMilli HplSensirionSht11P__Timer__precision_tag;
enum /*SendingMoteAppC.SensirionSht11C*/SensirionSht11C__0____nesc_unnamed4315 {
  SensirionSht11C__0__TEMP_KEY = 0U
};
enum /*SendingMoteAppC.SensirionSht11C*/SensirionSht11C__0____nesc_unnamed4316 {
  SensirionSht11C__0__HUM_KEY = 1U
};
typedef uint16_t AdcP__Read__val_t;
typedef uint16_t AdcP__ReadNow__val_t;
typedef const msp430adc12_channel_config_t *AdcP__Config__adc_config_t;
typedef TMilli Msp430RefVoltGeneratorP__SwitchOffTimer__precision_tag;
typedef TMilli Msp430RefVoltGeneratorP__SwitchOnTimer__precision_tag;
typedef const msp430adc12_channel_config_t *Msp430RefVoltArbiterImplP__Config__adc_config_t;
enum /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0____nesc_unnamed4317 {
  Msp430Adc12ClientAutoRVGC__0__ID = 0U
};
typedef const msp430adc12_channel_config_t */*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfSub__adc_config_t;
typedef const msp430adc12_channel_config_t */*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfUp__adc_config_t;
enum /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC*/AdcReadClientC__0____nesc_unnamed4318 {
  AdcReadClientC__0__CLIENT = 0U
};
typedef TMilli AdcStreamP__Alarm__precision_tag;
typedef uint32_t AdcStreamP__Alarm__size_type;
typedef const msp430adc12_channel_config_t *AdcStreamP__AdcConfigure__adc_config_t;
typedef uint16_t AdcStreamP__ReadStream__val_t;
enum /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__2____nesc_unnamed4319 {
  Msp430Timer32khzC__2__ALARM_ID = 2U
};
typedef T32khz /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__frequency_tag;
typedef /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__frequency_tag /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__precision_tag;
typedef uint16_t /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__size_type;
typedef TMilli /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__to_precision_tag;
typedef uint32_t /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__to_size_type;
typedef T32khz /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__from_precision_tag;
typedef uint16_t /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__from_size_type;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__to_precision_tag /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Alarm__precision_tag;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Alarm__size_type;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__from_precision_tag /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__AlarmFrom__precision_tag;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__from_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__AlarmFrom__size_type;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__to_precision_tag /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Counter__precision_tag;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Counter__size_type;
typedef uint16_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t;
typedef /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__val_t;
typedef /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__val_t;
enum /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1____nesc_unnamed4320 {
  Msp430Adc12ClientAutoRVGC__1__ID = 1U
};
typedef const msp430adc12_channel_config_t */*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfSub__adc_config_t;
typedef const msp430adc12_channel_config_t */*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfUp__adc_config_t;
enum /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC*/AdcReadStreamClientC__0____nesc_unnamed4321 {
  AdcReadStreamClientC__0__RSCLIENT = 0U
};
typedef const msp430adc12_channel_config_t *HamamatsuS10871TsrP__AdcConfigure__adc_config_t;
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static error_t CC2420CsmaP__SplitControl__start(void );
# 81 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420CsmaP__SubBackoff__requestInitialBackoff(message_t * msg);






static void CC2420CsmaP__SubBackoff__requestCongestionBackoff(message_t * msg);
# 73 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static void CC2420CsmaP__CC2420Transmit__sendDone(message_t * p_msg, error_t error);
# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static error_t CC2420CsmaP__Send__send(
#line 56
message_t * msg, 







uint8_t len);
#line 101
static uint8_t CC2420CsmaP__Send__maxPayloadLength(void );
# 76 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
static void CC2420CsmaP__CC2420Power__startOscillatorDone(void );
#line 56
static void CC2420CsmaP__CC2420Power__startVRegDone(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void CC2420CsmaP__Resource__granted(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void CC2420CsmaP__sendDone_task__runTask(void );
#line 64
static void CC2420CsmaP__stopDone_task__runTask(void );
#line 64
static void CC2420CsmaP__startDone_task__runTask(void );
# 99 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
static void CC2420ControlP__CC2420Config__setAutoAck(bool enableAutoAck, bool hwAutoAck);
#line 86
static bool CC2420ControlP__CC2420Config__isAddressRecognitionEnabled(void );
#line 110
static bool CC2420ControlP__CC2420Config__isAutoAckEnabled(void );
#line 105
static bool CC2420ControlP__CC2420Config__isHwAutoAckDefault(void );
#line 64
static uint16_t CC2420ControlP__CC2420Config__getShortAddr(void );
#line 52
static error_t CC2420ControlP__CC2420Config__sync(void );
#line 70
static uint16_t CC2420ControlP__CC2420Config__getPanAddr(void );
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void CC2420ControlP__StartupTimer__fired(void );
# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static error_t CC2420ControlP__ReadRssi__read(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void CC2420ControlP__syncDone__runTask(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t CC2420ControlP__Init__init(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void CC2420ControlP__SpiResource__granted(void );
#line 92
static void CC2420ControlP__SyncResource__granted(void );
# 71 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
static error_t CC2420ControlP__CC2420Power__startOscillator(void );
#line 90
static error_t CC2420ControlP__CC2420Power__rxOn(void );
#line 51
static error_t CC2420ControlP__CC2420Power__startVReg(void );
#line 63
static error_t CC2420ControlP__CC2420Power__stopVReg(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void CC2420ControlP__sync__runTask(void );
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__Resource__release(void );
#line 78
static error_t CC2420ControlP__Resource__request(void );
# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void CC2420ControlP__InterruptCCA__fired(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void CC2420ControlP__RssiResource__granted(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
#line 51
static error_t MotePlatformC__Init__init(void );
# 35 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 32
static void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );



static void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 31
static void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );





static void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 34
static void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 29
static void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__default__initClocks(void );
# 54 "/opt/tinyos-2.x/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t Msp430ClockP__Init__init(void );
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0xf721cb28);
# 41 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__clear(void );


static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setClockSource(uint16_t clockSource);
#line 43
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__disableEvents(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setMode(int mode);





static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setInputDivider(uint16_t inputDivider);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0xf721cb28);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
static bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );



static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControl(msp430_compare_control_t control);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(uint16_t time);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );



static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__setControl(msp430_compare_control_t control);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__setEvent(uint16_t time);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );
#line 36
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t delta);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void );
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm);
#line 31
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void );
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void );
#line 36
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void );
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t delta);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__clearPendingInterrupt(void );
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEventFromNow(uint16_t delta);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 59 "/opt/tinyos-2.x/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
# 45 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0xf74014a0);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 45 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0xf74014a0);
# 46 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP__Scheduler__init(void );
#line 61
static void SchedulerBasicP__Scheduler__taskLoop(void );
#line 54
static bool SchedulerBasicP__Scheduler__runNextTask(void );
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 92 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type dt);
#line 62
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );










static void /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
#line 53
static /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__size_type /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get(void );
# 98 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 92
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);
#line 55
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);






static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 59 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void );






static bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void );
#line 64
static void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void );
#line 59
static bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void );
#line 64
static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__makeInput(void );






static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__makeOutput(void );
#line 59
static bool /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__getRaw(void );
#line 34
static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__set(void );




static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__clr(void );
#line 64
static void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__makeInput(void );






static void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__set(void );




static void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__clr(void );
#line 71
static void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__set(void );




static void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__clr(void );
#line 85
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 85
static void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void );
#line 59
static bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void );
#line 85
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void );
#line 78
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void );
#line 71
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void );




static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void );
#line 71
static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void );




static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr(void );
#line 71
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void );




static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void );




static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle(void );
#line 71
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );









static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__toggle(void );
#line 71
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );









static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle(void );
#line 71
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );
#line 64
static void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectIOFunc(void );
#line 34
static void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__set(void );
#line 78
static void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectModuleFunc(void );
#line 39
static void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__clr(void );
#line 64
static void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectIOFunc(void );
#line 34
static void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__set(void );
#line 78
static void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectModuleFunc(void );
#line 39
static void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__clr(void );
#line 64
static void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectModuleFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectModuleFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectModuleFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectModuleFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectModuleFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectModuleFunc(void );
# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__0__GeneralIO__makeInput(void );
#line 32
static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__0__GeneralIO__get(void );


static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__GeneralIO__makeOutput(void );
#line 29
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__GeneralIO__set(void );
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__GeneralIO__clr(void );

static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__2__GeneralIO__get(void );
#line 32
static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__3__GeneralIO__get(void );


static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__4__GeneralIO__makeOutput(void );
#line 29
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__4__GeneralIO__set(void );
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__4__GeneralIO__clr(void );


static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__5__GeneralIO__makeInput(void );
#line 32
static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__5__GeneralIO__get(void );


static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__6__GeneralIO__makeOutput(void );
#line 29
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__6__GeneralIO__set(void );
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__6__GeneralIO__clr(void );
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time);
# 43 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge(void );
#line 55
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable(void );
#line 42
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge(void );
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void HplMsp430InterruptP__Port14__clear(void );
#line 36
static void HplMsp430InterruptP__Port14__disable(void );
#line 56
static void HplMsp430InterruptP__Port14__edge(bool low_to_high);
#line 31
static void HplMsp430InterruptP__Port14__enable(void );









static void HplMsp430InterruptP__Port26__clear(void );
#line 61
static void HplMsp430InterruptP__Port26__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port17__clear(void );
#line 61
static void HplMsp430InterruptP__Port17__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port21__clear(void );
#line 61
static void HplMsp430InterruptP__Port21__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port12__clear(void );
#line 61
static void HplMsp430InterruptP__Port12__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port24__clear(void );
#line 61
static void HplMsp430InterruptP__Port24__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port15__clear(void );
#line 36
static void HplMsp430InterruptP__Port15__disable(void );
#line 56
static void HplMsp430InterruptP__Port15__edge(bool low_to_high);
#line 31
static void HplMsp430InterruptP__Port15__enable(void );









static void HplMsp430InterruptP__Port27__clear(void );
#line 61
static void HplMsp430InterruptP__Port27__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port10__clear(void );
#line 36
static void HplMsp430InterruptP__Port10__disable(void );
#line 56
static void HplMsp430InterruptP__Port10__edge(bool low_to_high);
#line 31
static void HplMsp430InterruptP__Port10__enable(void );









static void HplMsp430InterruptP__Port22__clear(void );
#line 61
static void HplMsp430InterruptP__Port22__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port13__clear(void );
#line 61
static void HplMsp430InterruptP__Port13__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port25__clear(void );
#line 61
static void HplMsp430InterruptP__Port25__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port16__clear(void );
#line 61
static void HplMsp430InterruptP__Port16__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port20__clear(void );
#line 61
static void HplMsp430InterruptP__Port20__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port11__clear(void );
#line 61
static void HplMsp430InterruptP__Port11__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port23__clear(void );
#line 61
static void HplMsp430InterruptP__Port23__default__fired(void );
#line 61
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 50 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable(void );
#line 42
static error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void );
# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired(void );
# 50 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable(void );
#line 43
static error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge(void );
# 71 "/opt/tinyos-2.x/tos/interfaces/SpiPacket.nc"
static void CC2420SpiP__SpiPacket__sendDone(
#line 64
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 62 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static error_t CC2420SpiP__Fifo__continueRead(
# 46 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0xf6ef53f8, 
# 62 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 91
static void CC2420SpiP__Fifo__default__writeDone(
# 46 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0xf6ef53f8, 
# 91 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
#line 82
static cc2420_status_t CC2420SpiP__Fifo__write(
# 46 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0xf6ef53f8, 
# 82 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 51
static cc2420_status_t CC2420SpiP__Fifo__beginRead(
# 46 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0xf6ef53f8, 
# 51 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 71
static void CC2420SpiP__Fifo__default__readDone(
# 46 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0xf6ef53f8, 
# 71 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
# 31 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420SpiP__ChipSpiResource__abortRelease(void );







static error_t CC2420SpiP__ChipSpiResource__attemptRelease(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void CC2420SpiP__SpiResource__granted(void );
# 63 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420SpiP__Ram__write(
# 47 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint16_t arg_0xf6ef5e20, 
# 63 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc"
uint8_t offset, uint8_t * data, uint8_t length);
# 47 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420SpiP__Reg__read(
# 48 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0xf6ef45e8, 
# 47 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
uint16_t *data);







static cc2420_status_t CC2420SpiP__Reg__write(
# 48 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0xf6ef45e8, 
# 55 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
uint16_t data);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__release(
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0xf6ef6910);
# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__immediateRequest(
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0xf6ef6910);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__request(
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0xf6ef6910);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void CC2420SpiP__Resource__default__granted(
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0xf6ef6910);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static bool CC2420SpiP__Resource__isOwner(
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0xf6ef6910);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void CC2420SpiP__grant__runTask(void );
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420SpiP__Strobe__strobe(
# 49 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0xf6ef4da0);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t StateImplP__Init__init(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/State.nc"
static void StateImplP__State__toIdle(
# 67 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
uint8_t arg_0xf6eaf378);
# 66 "/opt/tinyos-2.x/tos/interfaces/State.nc"
static bool StateImplP__State__isState(
# 67 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
uint8_t arg_0xf6eaf378, 
# 66 "/opt/tinyos-2.x/tos/interfaces/State.nc"
uint8_t myState);
#line 61
static bool StateImplP__State__isIdle(
# 67 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
uint8_t arg_0xf6eaf378);
# 45 "/opt/tinyos-2.x/tos/interfaces/State.nc"
static error_t StateImplP__State__requestState(
# 67 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
uint8_t arg_0xf6eaf378, 
# 45 "/opt/tinyos-2.x/tos/interfaces/State.nc"
uint8_t reqState);





static void StateImplP__State__forceState(
# 67 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
uint8_t arg_0xf6eaf378, 
# 51 "/opt/tinyos-2.x/tos/interfaces/State.nc"
uint8_t reqState);
# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(
# 42 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5da78);
# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(
# 42 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5da78);
# 59 "/opt/tinyos-2.x/tos/interfaces/SpiPacket.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(
# 44 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5c598, 
# 48 "/opt/tinyos-2.x/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
#line 71
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(
# 44 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5c598, 
# 64 "/opt/tinyos-2.x/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(
# 47 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5b7f8);
# 34 "/opt/tinyos-2.x/tos/interfaces/SpiByte.nc"
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(uint8_t tx);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(
# 46 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5cd90);
# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(
# 46 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5cd90);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(
# 46 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5cd90);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(
# 46 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5cd90);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(
# 46 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5cd90);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5d030);
# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5d030);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5d030);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5d030);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5d030);
# 54 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void );
# 180 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430Usart0P__Usart__enableRxIntr(void );
#line 197
static void HplMsp430Usart0P__Usart__clrRxIntr(void );
#line 97
static void HplMsp430Usart0P__Usart__resetUsart(bool reset);
#line 179
static void HplMsp430Usart0P__Usart__disableIntr(void );
#line 90
static void HplMsp430Usart0P__Usart__setUmctl(uint8_t umctl);
#line 177
static void HplMsp430Usart0P__Usart__disableRxIntr(void );
#line 207
static void HplMsp430Usart0P__Usart__clrIntr(void );
#line 80
static void HplMsp430Usart0P__Usart__setUbr(uint16_t ubr);
#line 224
static void HplMsp430Usart0P__Usart__tx(uint8_t data);
#line 128
static void HplMsp430Usart0P__Usart__disableUart(void );
#line 153
static void HplMsp430Usart0P__Usart__enableSpi(void );
#line 168
static void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 231
static uint8_t HplMsp430Usart0P__Usart__rx(void );
#line 192
static bool HplMsp430Usart0P__Usart__isRxIntrPending(void );
#line 158
static void HplMsp430Usart0P__Usart__disableSpi(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
# 50 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
static void LedsP__Leds__led0Off(void );





static void LedsP__Leds__led0Toggle(void );
#line 72
static void LedsP__Leds__led1Toggle(void );
#line 89
static void LedsP__Leds__led2Toggle(void );
#line 66
static void LedsP__Leds__led1Off(void );
#line 83
static void LedsP__Leds__led2Off(void );
# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__GeneralIO__toggle(void );



static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__GeneralIO__makeOutput(void );
#line 29
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__GeneralIO__set(void );

static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__GeneralIO__toggle(void );



static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__GeneralIO__makeOutput(void );
#line 29
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__GeneralIO__set(void );

static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__GeneralIO__toggle(void );



static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__GeneralIO__makeOutput(void );
#line 29
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__GeneralIO__set(void );
# 54 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(
# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0xf6d72758, 
# 54 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(
# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0xf6d72758);
# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired(void );
#line 39
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0xf6d71010);
# 54 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void );
# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id);
#line 43
static bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void );








static bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void );
# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(
# 55 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4c770);
# 51 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(
# 55 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4c770);
# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(
# 60 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4bb38);
# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(
# 60 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4bb38);
# 56 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 73
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested(void );
#line 46
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void );
#line 81
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested(void );
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(
# 54 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4ece8);
# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(
# 54 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4ece8);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(
# 54 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4ece8);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(
# 54 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4ece8);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(
# 54 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4ece8);
# 80 "/opt/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
# 7 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C.nc"
static void HplMsp430I2C0P__HplI2C__clearModeI2C(void );
#line 6
static bool HplMsp430I2C0P__HplI2C__isI2C(void );
# 44 "/opt/tinyos-2.x/tos/system/ActiveMessageAddressC.nc"
static am_addr_t ActiveMessageAddressC__amAddress(void );
# 50 "/opt/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );




static am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void );
# 66 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420TransmitP__RadioBackoff__setCongestionBackoff(uint16_t backoffTime);
#line 60
static void CC2420TransmitP__RadioBackoff__setInitialBackoff(uint16_t backoffTime);
# 50 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
static void CC2420TransmitP__CaptureSFD__captured(uint16_t time);
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void CC2420TransmitP__BackoffTimer__fired(void );
# 63 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420TransmitP__CC2420Receive__receive(uint8_t type, message_t * message);
# 51 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static error_t CC2420TransmitP__Send__send(message_t * p_msg, bool useCca);
# 24 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420TransmitP__ChipSpiResource__releasing(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t CC2420TransmitP__Init__init(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void CC2420TransmitP__SpiResource__granted(void );
# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t CC2420TransmitP__StdControl__start(void );









static error_t CC2420TransmitP__StdControl__stop(void );
# 91 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420TransmitP__TXFIFO__writeDone(uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420TransmitP__TXFIFO__readDone(uint8_t * data, uint8_t length, error_t error);
# 53 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
static void CC2420ReceiveP__CC2420Config__syncDone(error_t error);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void CC2420ReceiveP__receiveDone_task__runTask(void );
# 55 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420ReceiveP__CC2420Receive__sfd_dropped(void );
#line 49
static void CC2420ReceiveP__CC2420Receive__sfd(uint32_t time);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t CC2420ReceiveP__Init__init(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void CC2420ReceiveP__SpiResource__granted(void );
# 91 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420ReceiveP__RXFIFO__writeDone(uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420ReceiveP__RXFIFO__readDone(uint8_t * data, uint8_t length, error_t error);
# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void CC2420ReceiveP__InterruptFIFOP__fired(void );
# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t CC2420ReceiveP__StdControl__start(void );









static error_t CC2420ReceiveP__StdControl__stop(void );
# 56 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Packet.nc"
static void CC2420PacketP__CC2420Packet__setPower(message_t *p_msg, uint8_t power);
#line 77
static void CC2420PacketP__CC2420Packet__setNetwork(message_t *p_msg, uint8_t networkId);
#line 64
static int8_t CC2420PacketP__CC2420Packet__getRssi(message_t *p_msg);










static uint8_t CC2420PacketP__CC2420Packet__getNetwork(message_t *p_msg);
# 59 "/opt/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
static void CC2420PacketP__PacketTimeStamp32khz__clear(
#line 55
message_t * msg);
#line 67
static void CC2420PacketP__PacketTimeStamp32khz__set(
#line 62
message_t * msg, 




CC2420PacketP__PacketTimeStamp32khz__size_type value);
# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420PacketP__CC2420PacketBody__getHeader(message_t * msg);




static cc2420_metadata_t * CC2420PacketP__CC2420PacketBody__getMetadata(message_t * msg);
# 47 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
static uint8_t CC2420PacketP__PacketTimeSyncOffset__get(
#line 42
message_t * msg);
#line 39
static bool CC2420PacketP__PacketTimeSyncOffset__isSet(
#line 35
message_t * msg);
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 92 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Init__init(void );
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void );
#line 53
static /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__size_type /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__get(void );
# 98 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__size_type dt);
#line 105
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__fired(void );
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__overflow(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
# 125 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
#line 118
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );
#line 72
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(
# 37 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0xf6b05030);
# 81 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(
# 37 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0xf6b05030);
# 53 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(
# 37 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0xf6b05030, 
# 53 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
uint32_t dt);








static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(
# 37 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0xf6b05030, 
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
uint32_t dt);




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(
# 37 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0xf6b05030);
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void );
# 41 "/opt/tinyos-2.x/tos/interfaces/Random.nc"
static uint16_t RandomMlcgC__Random__rand16(void );
#line 35
static uint32_t RandomMlcgC__Random__rand32(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t RandomMlcgC__Init__init(void );
# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static void UniqueSendP__SubSend__sendDone(
#line 85
message_t * msg, 



error_t error);
#line 64
static error_t UniqueSendP__Send__send(
#line 56
message_t * msg, 







uint8_t len);
#line 101
static uint8_t UniqueSendP__Send__maxPayloadLength(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t UniqueSendP__Init__init(void );
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



UniqueReceiveP__SubReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t UniqueReceiveP__Init__init(void );
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



UniqueReceiveP__DuplicateReceive__default__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__SubSend__sendDone(
#line 85
message_t * msg, 



error_t error);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420TinyosNetworkP__SubReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void CC2420TinyosNetworkP__grantTask__runTask(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static error_t CC2420TinyosNetworkP__ActiveSend__send(
#line 56
message_t * msg, 







uint8_t len);
#line 114
static 
#line 112
void * 

CC2420TinyosNetworkP__ActiveSend__getPayload(
#line 111
message_t * msg, 


uint8_t len);
#line 101
static uint8_t CC2420TinyosNetworkP__ActiveSend__maxPayloadLength(void );
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420TinyosNetworkP__BareReceive__default__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420TinyosNetworkP__Resource__release(
# 46 "/opt/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0xf6a68628);
# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420TinyosNetworkP__Resource__immediateRequest(
# 46 "/opt/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0xf6a68628);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420TinyosNetworkP__Resource__request(
# 46 "/opt/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0xf6a68628);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void CC2420TinyosNetworkP__Resource__default__granted(
# 46 "/opt/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0xf6a68628);
# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__BareSend__default__sendDone(
#line 85
message_t * msg, 



error_t error);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init(void );
# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
#line 43
static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );








static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420ActiveMessageP__SubReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static void CC2420ActiveMessageP__SubSend__sendDone(
#line 85
message_t * msg, 



error_t error);
# 53 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
static void CC2420ActiveMessageP__CC2420Config__syncDone(error_t error);
# 95 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420ActiveMessageP__RadioBackoff__default__requestCca(
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0xf6a1d4b8, 
# 95 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);
#line 81
static void CC2420ActiveMessageP__RadioBackoff__default__requestInitialBackoff(
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0xf6a1d4b8, 
# 81 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);






static void CC2420ActiveMessageP__RadioBackoff__default__requestCongestionBackoff(
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0xf6a1d4b8, 
# 88 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);
# 59 "/opt/tinyos-2.x/tos/interfaces/SendNotifier.nc"
static void CC2420ActiveMessageP__SendNotifier__default__aboutToSend(
# 44 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0xf6a1ee20, 
# 59 "/opt/tinyos-2.x/tos/interfaces/SendNotifier.nc"
am_addr_t dest, 
#line 57
message_t * msg);
# 95 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420ActiveMessageP__SubBackoff__requestCca(message_t * msg);
#line 81
static void CC2420ActiveMessageP__SubBackoff__requestInitialBackoff(message_t * msg);






static void CC2420ActiveMessageP__SubBackoff__requestCongestionBackoff(message_t * msg);
# 67 "/opt/tinyos-2.x/tos/interfaces/Packet.nc"
static uint8_t CC2420ActiveMessageP__Packet__payloadLength(
#line 63
message_t * msg);
#line 115
static 
#line 112
void * 


CC2420ActiveMessageP__Packet__getPayload(
#line 110
message_t * msg, 




uint8_t len);
#line 95
static uint8_t CC2420ActiveMessageP__Packet__maxPayloadLength(void );
#line 83
static void CC2420ActiveMessageP__Packet__setPayloadLength(
#line 79
message_t * msg, 



uint8_t len);
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static error_t CC2420ActiveMessageP__AMSend__send(
# 39 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0xf6a213f8, 
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420ActiveMessageP__Snoop__default__receive(
# 41 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0xf6a1f490, 
# 60 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
#line 67
static 
#line 63
message_t * 



CC2420ActiveMessageP__Receive__default__receive(
# 40 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0xf6a21db8, 
# 60 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 77 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
static am_addr_t CC2420ActiveMessageP__AMPacket__source(
#line 73
message_t * amsg);
#line 57
static am_addr_t CC2420ActiveMessageP__AMPacket__address(void );









static am_addr_t CC2420ActiveMessageP__AMPacket__destination(
#line 63
message_t * amsg);
#line 92
static void CC2420ActiveMessageP__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 136
static am_id_t CC2420ActiveMessageP__AMPacket__type(
#line 132
message_t * amsg);
#line 151
static void CC2420ActiveMessageP__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
#line 125
static bool CC2420ActiveMessageP__AMPacket__isForMe(
#line 122
message_t * amsg);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void CC2420ActiveMessageP__RadioResource__granted(void );
# 53 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
static void SendingMoteC__CC2420Config__syncDone(error_t error);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void SendingMoteC__SendTimer__fired(void );
#line 72
static void SendingMoteC__BootupTimer__fired(void );
#line 72
static void SendingMoteC__NoiseFloorTimer__fired(void );
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



SendingMoteC__CommandReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SendingMoteC__sendIdentification__runTask(void );
# 49 "/opt/tinyos-2.x/tos/interfaces/Boot.nc"
static void SendingMoteC__Boot__booted(void );
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static void SendingMoteC__Light__readDone(error_t result, SendingMoteC__Light__val_t val);
#line 63
static void SendingMoteC__ReadRssi__readDone(error_t result, SendingMoteC__ReadRssi__val_t val);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void SendingMoteC__SensorReadingTimer__fired(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static void SendingMoteC__RadioControl__startDone(error_t error);
#line 117
static void SendingMoteC__RadioControl__stopDone(error_t error);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SendingMoteC__sendReport__runTask(void );
#line 64
static void SendingMoteC__sendEchoReply__runTask(void );
#line 64
static void SendingMoteC__sendCommandACK__runTask(void );
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static void SendingMoteC__Humidity__readDone(error_t result, SendingMoteC__Humidity__val_t val);
# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static void SendingMoteC__TinyReportMsgSend__sendDone(
#line 92
message_t * msg, 






error_t error);
#line 99
static void SendingMoteC__ReportMsgSend__sendDone(
#line 92
message_t * msg, 






error_t error);
#line 99
static void SendingMoteC__CommandMsgSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void SendingMoteC__SignalTimer__fired(void );
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



SendingMoteC__PingReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SendingMoteC__sendReading__runTask(void );
# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static void SendingMoteC__RssiMsgSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



SendingMoteC__ResponseReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static void SendingMoteC__MultiPingResponseSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static void SendingMoteC__Temperature__readDone(error_t result, SendingMoteC__Temperature__val_t val);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



SendingMoteC__SimplePingReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void SendingMoteC__FlushTimer__fired(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SendingMoteC__sendMultipleEcho__runTask(void );
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void SendingMoteC__GapTimer__fired(void );
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static error_t /*SendingMoteAppC.RssiMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static void /*SendingMoteAppC.RssiMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(
# 40 "/opt/tinyos-2.x/tos/system/AMQueueImplP.nc"
am_id_t arg_0xf68a3bc8, 
# 92 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(
# 38 "/opt/tinyos-2.x/tos/system/AMQueueImplP.nc"
uint8_t arg_0xf68a31e0, 
# 56 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 89
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(
# 38 "/opt/tinyos-2.x/tos/system/AMQueueImplP.nc"
uint8_t arg_0xf68a31e0, 
# 85 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void );
#line 64
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void );
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static error_t /*SendingMoteAppC.MultiPingResponseSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static void /*SendingMoteAppC.MultiPingResponseSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static error_t /*SendingMoteAppC.ReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static void /*SendingMoteAppC.ReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__2__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static error_t /*SendingMoteAppC.TinyReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static void /*SendingMoteAppC.TinyReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__3__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static error_t /*SendingMoteAppC.CommandMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__4__AMSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static void /*SendingMoteAppC.CommandMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__4__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 84 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureHumidityDone(error_t result, uint16_t val);
#line 116
static void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__writeStatusRegDone(error_t result);
#line 100
static void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__readStatusRegDone(error_t result, uint8_t val);
#line 54
static void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__resetDone(error_t result);
#line 69
static void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureTemperatureDone(error_t result, uint16_t val);
# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static error_t /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__read(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__granted(void );
# 84 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureHumidityDone(error_t result, uint16_t val);
#line 116
static void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__writeStatusRegDone(error_t result);
#line 100
static void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__readStatusRegDone(error_t result, uint8_t val);
#line 54
static void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__resetDone(error_t result);
#line 69
static void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureTemperatureDone(error_t result, uint16_t val);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__HumResource__granted(void );
# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static error_t /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__read(void );
# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__fired(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readSensor__runTask(void );
#line 64
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone__runTask(void );
# 84 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__measureHumidityDone(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0xf6823938, 
# 84 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t result, uint16_t val);
#line 76
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureHumidity(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0xf6823938);
# 61 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureTemperature(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0xf6823938);
# 116 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__writeStatusRegDone(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0xf6823938, 
# 116 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t result);
#line 100
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__readStatusRegDone(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0xf6823938, 
# 100 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t result, uint8_t val);
#line 54
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__resetDone(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0xf6823938, 
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t result);
#line 69
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__measureTemperatureDone(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0xf6823938, 
# 69 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t result, uint16_t val);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__fired(void );
# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__makeInput(void );
#line 32
static bool /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__get(void );


static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__makeOutput(void );
#line 29
static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__set(void );
static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__clr(void );


static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__makeInput(void );

static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__makeOutput(void );
#line 29
static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__set(void );
static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__clr(void );




static void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__makeOutput(void );
#line 29
static void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__set(void );
static void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__clr(void );
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static error_t HplSensirionSht11P__SplitControl__start(void );
#line 109
static error_t HplSensirionSht11P__SplitControl__stop(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void HplSensirionSht11P__stopTask__runTask(void );
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void HplSensirionSht11P__Timer__fired(void );
# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__fired(void );
# 50 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__disable(void );
#line 43
static error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__enableFallingEdge(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__Init__init(void );
# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static error_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(resource_client_id_t id);
#line 43
static bool /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEmpty(void );








static bool /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__dequeue(void );
# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(
# 55 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4c770);
# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(
# 60 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4bb38);
# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(
# 60 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4bb38);
# 56 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void );








static bool /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__isOwner(void );
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__release(
# 54 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4ece8);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__request(
# 54 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4ece8);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__default__granted(
# 54 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4ece8);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask__runTask(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__startDone(error_t error);
#line 117
static void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__stopDone(error_t error);
# 52 "/opt/tinyos-2.x/tos/lib/power/PowerDownCleanup.nc"
static void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__PowerDownCleanup__default__cleanup(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopTask__runTask(void );
# 73 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
static void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__requested(void );
#line 46
static void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__granted(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__startTask__runTask(void );
# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__default__start(void );









static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__default__stop(void );
# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static error_t AdcP__Read__read(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0xf676c668);
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static void AdcP__Read__default__readDone(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0xf676c668, 
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
error_t result, AdcP__Read__val_t val);
# 66 "/opt/tinyos-2.x/tos/interfaces/ReadNow.nc"
static void AdcP__ReadNow__default__readDone(
# 39 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0xf6768b08, 
# 66 "/opt/tinyos-2.x/tos/interfaces/ReadNow.nc"
error_t result, AdcP__ReadNow__val_t val);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static AdcP__Config__adc_config_t AdcP__Config__default__getConfiguration(
# 48 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0xf6766df0);
# 189 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcP__SingleChannel__default__getData(
# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0xf6758e40);
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcP__SingleChannel__default__configureSingle(
# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0xf6758e40, 
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config);
#line 227
static uint16_t * AdcP__SingleChannel__multipleDataReady(
# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0xf6758e40, 
# 227 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t * buffer, uint16_t numSamples);
#line 206
static error_t AdcP__SingleChannel__singleDataReady(
# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0xf6758e40, 
# 206 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t data);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t AdcP__ResourceRead__default__release(
# 44 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0xf6769670);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t AdcP__ResourceRead__default__request(
# 44 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0xf6769670);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void AdcP__ResourceRead__granted(
# 44 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0xf6769670);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void AdcP__readDone__runTask(void );
# 107 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
static void Msp430Adc12ImplP__MultiChannel__default__dataReady(
# 42 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0xf671d688, 
# 107 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
uint16_t *buffer, uint16_t numSamples);
# 112 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
static void Msp430Adc12ImplP__HplAdc12__conversionDone(uint16_t iv);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void Msp430Adc12ImplP__CompareA1__fired(void );
# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static void Msp430Adc12ImplP__Overflow__default__memOverflow(
# 43 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0xf671c010);
# 54 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static void Msp430Adc12ImplP__Overflow__default__conversionTimeOverflow(
# 43 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0xf671c010);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t Msp430Adc12ImplP__Init__init(void );
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void Msp430Adc12ImplP__TimerA__overflow(void );
# 189 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t Msp430Adc12ImplP__SingleChannel__getData(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0xf671e908);
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t Msp430Adc12ImplP__SingleChannel__configureSingle(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0xf671e908, 
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config);
#line 227
static uint16_t * Msp430Adc12ImplP__SingleChannel__default__multipleDataReady(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0xf671e908, 
# 227 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t * buffer, uint16_t numSamples);
#line 138
static error_t Msp430Adc12ImplP__SingleChannel__configureMultiple(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0xf671e908, 
# 138 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config, uint16_t * buffer, uint16_t numSamples, uint16_t jiffies);
#line 206
static error_t Msp430Adc12ImplP__SingleChannel__default__singleDataReady(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0xf671e908, 
# 206 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t data);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void Msp430Adc12ImplP__CompareA0__fired(void );
# 63 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
static adc12ctl0_t HplAdc12P__HplAdc12__getCtl0(void );
#line 82
static adc12memctl_t HplAdc12P__HplAdc12__getMCtl(uint8_t idx);
#line 106
static void HplAdc12P__HplAdc12__resetIFGs(void );
#line 118
static bool HplAdc12P__HplAdc12__isBusy(void );
#line 75
static void HplAdc12P__HplAdc12__setMCtl(uint8_t idx, adc12memctl_t memControl);
#line 128
static void HplAdc12P__HplAdc12__startConversion(void );
#line 51
static void HplAdc12P__HplAdc12__setCtl0(adc12ctl0_t control0);
#line 89
static uint16_t HplAdc12P__HplAdc12__getMem(uint8_t idx);





static void HplAdc12P__HplAdc12__setIEFlags(uint16_t mask);
#line 123
static void HplAdc12P__HplAdc12__stopConversion(void );
#line 57
static void HplAdc12P__HplAdc12__setCtl1(adc12ctl1_t control1);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__Init__init(void );
# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__enqueue(resource_client_id_t id);
#line 43
static bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEmpty(void );








static bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__dequeue(void );
# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(
# 52 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf6667be0);
# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__unconfigure(
# 56 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf6666898);
# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(
# 56 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf6666898);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__release(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf66671b8);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__request(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf66671b8);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__default__granted(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf66671b8);
# 88 "/opt/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
static uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ArbiterInfo__userId(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__runTask(void );
# 112 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
static void Msp430RefVoltGeneratorP__HplAdc12__conversionDone(uint16_t iv);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void Msp430RefVoltGeneratorP__SwitchOffTimer__fired(void );
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static error_t Msp430RefVoltGeneratorP__RefVolt_2_5V__start(void );
#line 83
static error_t Msp430RefVoltGeneratorP__RefVolt_1_5V__start(void );
#line 109
static error_t Msp430RefVoltGeneratorP__RefVolt_1_5V__stop(void );
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void Msp430RefVoltGeneratorP__SwitchOnTimer__fired(void );
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static Msp430RefVoltArbiterImplP__Config__adc_config_t Msp430RefVoltArbiterImplP__Config__default__getConfiguration(
# 43 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0xf6606df0);
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static void Msp430RefVoltArbiterImplP__RefVolt_2_5V__startDone(error_t error);
#line 117
static void Msp430RefVoltArbiterImplP__RefVolt_2_5V__stopDone(error_t error);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP__AdcResource__default__release(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0xf66073a0);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP__AdcResource__default__request(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0xf66073a0);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void Msp430RefVoltArbiterImplP__AdcResource__granted(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0xf66073a0);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP__ClientResource__release(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0xf66388c0);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP__ClientResource__request(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0xf66388c0);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void Msp430RefVoltArbiterImplP__ClientResource__default__granted(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0xf66388c0);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void Msp430RefVoltArbiterImplP__switchOff__runTask(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static void Msp430RefVoltArbiterImplP__RefVolt_1_5V__startDone(error_t error);
#line 117
static void Msp430RefVoltArbiterImplP__RefVolt_1_5V__stopDone(error_t error);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfSub__adc_config_t /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfSub__getConfiguration(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void AdcStreamP__bufferDone__runTask(void );
#line 64
static void AdcStreamP__readStreamDone__runTask(void );
#line 64
static void AdcStreamP__readStreamFail__runTask(void );
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void AdcStreamP__Alarm__fired(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t AdcStreamP__Init__init(void );
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static AdcStreamP__AdcConfigure__adc_config_t AdcStreamP__AdcConfigure__default__getConfiguration(
# 53 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0xf65bf010);
# 189 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcStreamP__SingleChannel__default__getData(
# 52 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0xf65c1170);
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcStreamP__SingleChannel__default__configureSingle(
# 52 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0xf65c1170, 
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config);
#line 227
static uint16_t * AdcStreamP__SingleChannel__multipleDataReady(
# 52 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0xf65c1170, 
# 227 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t * buffer, uint16_t numSamples);
#line 138
static error_t AdcStreamP__SingleChannel__default__configureMultiple(
# 52 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0xf65c1170, 
# 138 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config, uint16_t * buffer, uint16_t numSamples, uint16_t jiffies);
#line 206
static error_t AdcStreamP__SingleChannel__singleDataReady(
# 52 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0xf65c1170, 
# 206 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t data);
# 78 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
static error_t AdcStreamP__ReadStream__read(
# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0xf65c3690, 
# 78 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
uint32_t usPeriod);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired(void );
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow(void );
# 92 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__startAt(/*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__size_type t0, /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__size_type dt);





static /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Alarm__size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Alarm__getNow(void );
#line 92
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Alarm__startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Alarm__size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Alarm__size_type dt);
#line 67
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__AlarmFrom__fired(void );
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Counter__overflow(void );
# 89 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__bufferDone(
# 26 "/opt/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0xf657ed00, 
# 89 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
error_t result, 
#line 86
/*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__val_t * buf, 



uint16_t count);
#line 102
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__readDone(
# 26 "/opt/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0xf657ed00, 
# 102 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
error_t result, uint32_t usActualPeriod);
#line 89
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__bufferDone(
# 24 "/opt/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0xf657f258, 
# 89 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
error_t result, 
#line 86
/*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__val_t * buf, 



uint16_t count);
#line 102
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__readDone(
# 24 "/opt/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0xf657f258, 
# 102 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
error_t result, uint32_t usActualPeriod);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__default__release(
# 27 "/opt/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0xf657b678);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__granted(
# 27 "/opt/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0xf657b678);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfSub__adc_config_t /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfSub__getConfiguration(void );
#line 58
static HamamatsuS10871TsrP__AdcConfigure__adc_config_t HamamatsuS10871TsrP__AdcConfigure__getConfiguration(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static void CC2420CsmaP__SplitControl__startDone(error_t error);
#line 117
static void CC2420CsmaP__SplitControl__stopDone(error_t error);
# 95 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420CsmaP__RadioBackoff__requestCca(message_t * msg);
#line 81
static void CC2420CsmaP__RadioBackoff__requestInitialBackoff(message_t * msg);






static void CC2420CsmaP__RadioBackoff__requestCongestionBackoff(message_t * msg);
#line 66
static void CC2420CsmaP__SubBackoff__setCongestionBackoff(uint16_t backoffTime);
#line 60
static void CC2420CsmaP__SubBackoff__setInitialBackoff(uint16_t backoffTime);
# 51 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static error_t CC2420CsmaP__CC2420Transmit__send(message_t * p_msg, bool useCca);
# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static void CC2420CsmaP__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 41 "/opt/tinyos-2.x/tos/interfaces/Random.nc"
static uint16_t CC2420CsmaP__Random__rand16(void );
# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t CC2420CsmaP__SubControl__start(void );









static error_t CC2420CsmaP__SubControl__stop(void );
# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420CsmaP__CC2420PacketBody__getHeader(message_t * msg);




static cc2420_metadata_t * CC2420CsmaP__CC2420PacketBody__getMetadata(message_t * msg);
# 71 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
static error_t CC2420CsmaP__CC2420Power__startOscillator(void );
#line 90
static error_t CC2420CsmaP__CC2420Power__rxOn(void );
#line 51
static error_t CC2420CsmaP__CC2420Power__startVReg(void );
#line 63
static error_t CC2420CsmaP__CC2420Power__stopVReg(void );
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420CsmaP__Resource__release(void );
#line 78
static error_t CC2420CsmaP__Resource__request(void );
# 66 "/opt/tinyos-2.x/tos/interfaces/State.nc"
static bool CC2420CsmaP__SplitControlState__isState(uint8_t myState);
#line 45
static error_t CC2420CsmaP__SplitControlState__requestState(uint8_t reqState);





static void CC2420CsmaP__SplitControlState__forceState(uint8_t reqState);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t CC2420CsmaP__sendDone_task__postTask(void );
#line 56
static error_t CC2420CsmaP__stopDone_task__postTask(void );
#line 56
static error_t CC2420CsmaP__startDone_task__postTask(void );
# 74 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
enum CC2420CsmaP____nesc_unnamed4322 {
#line 74
  CC2420CsmaP__startDone_task = 0U
};
#line 74
typedef int CC2420CsmaP____nesc_sillytask_startDone_task[CC2420CsmaP__startDone_task];
enum CC2420CsmaP____nesc_unnamed4323 {
#line 75
  CC2420CsmaP__stopDone_task = 1U
};
#line 75
typedef int CC2420CsmaP____nesc_sillytask_stopDone_task[CC2420CsmaP__stopDone_task];
enum CC2420CsmaP____nesc_unnamed4324 {
#line 76
  CC2420CsmaP__sendDone_task = 2U
};
#line 76
typedef int CC2420CsmaP____nesc_sillytask_sendDone_task[CC2420CsmaP__sendDone_task];
#line 58
enum CC2420CsmaP____nesc_unnamed4325 {
  CC2420CsmaP__S_STOPPED, 
  CC2420CsmaP__S_STARTING, 
  CC2420CsmaP__S_STARTED, 
  CC2420CsmaP__S_STOPPING, 
  CC2420CsmaP__S_TRANSMITTING
};

message_t * CC2420CsmaP__m_msg;

error_t CC2420CsmaP__sendErr = SUCCESS;


bool CC2420CsmaP__ccaOn;






static inline void CC2420CsmaP__shutdown(void );


static error_t CC2420CsmaP__SplitControl__start(void );
#line 122
static inline error_t CC2420CsmaP__Send__send(message_t *p_msg, uint8_t len);
#line 170
static inline uint8_t CC2420CsmaP__Send__maxPayloadLength(void );
#line 202
static inline void CC2420CsmaP__CC2420Transmit__sendDone(message_t *p_msg, error_t err);




static inline void CC2420CsmaP__CC2420Power__startVRegDone(void );



static inline void CC2420CsmaP__Resource__granted(void );



static inline void CC2420CsmaP__CC2420Power__startOscillatorDone(void );




static inline void CC2420CsmaP__SubBackoff__requestInitialBackoff(message_t *msg);






static inline void CC2420CsmaP__SubBackoff__requestCongestionBackoff(message_t *msg);
#line 241
static inline void CC2420CsmaP__sendDone_task__runTask(void );
#line 254
static inline void CC2420CsmaP__startDone_task__runTask(void );







static inline void CC2420CsmaP__stopDone_task__runTask(void );









static inline void CC2420CsmaP__shutdown(void );
# 53 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
static void CC2420ControlP__CC2420Config__syncDone(error_t error);
# 55 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__RXCTRL1__write(uint16_t data);
# 55 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void CC2420ControlP__StartupTimer__start(CC2420ControlP__StartupTimer__size_type dt);
# 55 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__MDMCTRL0__write(uint16_t data);
# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void CC2420ControlP__RSTN__makeOutput(void );
#line 29
static void CC2420ControlP__RSTN__set(void );
static void CC2420ControlP__RSTN__clr(void );
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static void CC2420ControlP__ReadRssi__readDone(error_t result, CC2420ControlP__ReadRssi__val_t val);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t CC2420ControlP__syncDone__postTask(void );
# 47 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__RSSI__read(uint16_t *data);







static cc2420_status_t CC2420ControlP__IOCFG0__write(uint16_t data);
# 50 "/opt/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t CC2420ControlP__ActiveMessageAddress__amAddress(void );




static am_group_t CC2420ControlP__ActiveMessageAddress__amGroup(void );
# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void CC2420ControlP__CSN__makeOutput(void );
#line 29
static void CC2420ControlP__CSN__set(void );
static void CC2420ControlP__CSN__clr(void );




static void CC2420ControlP__VREN__makeOutput(void );
#line 29
static void CC2420ControlP__VREN__set(void );
static void CC2420ControlP__VREN__clr(void );
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SXOSCON__strobe(void );
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__SpiResource__release(void );
#line 78
static error_t CC2420ControlP__SpiResource__request(void );
#line 110
static error_t CC2420ControlP__SyncResource__release(void );
#line 78
static error_t CC2420ControlP__SyncResource__request(void );
# 76 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
static void CC2420ControlP__CC2420Power__startOscillatorDone(void );
#line 56
static void CC2420ControlP__CC2420Power__startVRegDone(void );
# 55 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__IOCFG1__write(uint16_t data);
#line 55
static cc2420_status_t CC2420ControlP__FSCTRL__write(uint16_t data);
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SRXON__strobe(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void CC2420ControlP__Resource__granted(void );
# 63 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420ControlP__PANID__write(uint8_t offset, uint8_t * data, uint8_t length);
# 50 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static error_t CC2420ControlP__InterruptCCA__disable(void );
#line 42
static error_t CC2420ControlP__InterruptCCA__enableRisingEdge(void );
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__RssiResource__release(void );
#line 78
static error_t CC2420ControlP__RssiResource__request(void );
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SRFOFF__strobe(void );
# 117 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
enum CC2420ControlP____nesc_unnamed4326 {
#line 117
  CC2420ControlP__sync = 3U
};
#line 117
typedef int CC2420ControlP____nesc_sillytask_sync[CC2420ControlP__sync];
enum CC2420ControlP____nesc_unnamed4327 {
#line 118
  CC2420ControlP__syncDone = 4U
};
#line 118
typedef int CC2420ControlP____nesc_sillytask_syncDone[CC2420ControlP__syncDone];
#line 85
#line 79
typedef enum CC2420ControlP____nesc_unnamed4328 {
  CC2420ControlP__S_VREG_STOPPED, 
  CC2420ControlP__S_VREG_STARTING, 
  CC2420ControlP__S_VREG_STARTED, 
  CC2420ControlP__S_XOSC_STARTING, 
  CC2420ControlP__S_XOSC_STARTED
} CC2420ControlP__cc2420_control_state_t;

uint8_t CC2420ControlP__m_channel;

uint8_t CC2420ControlP__m_tx_power;

uint16_t CC2420ControlP__m_pan;

uint16_t CC2420ControlP__m_short_addr;

bool CC2420ControlP__m_sync_busy;


bool CC2420ControlP__autoAckEnabled;


bool CC2420ControlP__hwAutoAckDefault;


bool CC2420ControlP__addressRecognition;


bool CC2420ControlP__hwAddressRecognition;

CC2420ControlP__cc2420_control_state_t CC2420ControlP__m_state = CC2420ControlP__S_VREG_STOPPED;



static void CC2420ControlP__writeFsctrl(void );
static void CC2420ControlP__writeMdmctrl0(void );
static void CC2420ControlP__writeId(void );





static inline error_t CC2420ControlP__Init__init(void );
#line 171
static inline error_t CC2420ControlP__Resource__request(void );







static inline error_t CC2420ControlP__Resource__release(void );







static inline error_t CC2420ControlP__CC2420Power__startVReg(void );
#line 199
static inline error_t CC2420ControlP__CC2420Power__stopVReg(void );







static inline error_t CC2420ControlP__CC2420Power__startOscillator(void );
#line 249
static inline error_t CC2420ControlP__CC2420Power__rxOn(void );
#line 279
static uint16_t CC2420ControlP__CC2420Config__getShortAddr(void );







static inline uint16_t CC2420ControlP__CC2420Config__getPanAddr(void );
#line 300
static error_t CC2420ControlP__CC2420Config__sync(void );
#line 332
static inline bool CC2420ControlP__CC2420Config__isAddressRecognitionEnabled(void );
#line 350
static void CC2420ControlP__CC2420Config__setAutoAck(bool enableAutoAck, bool hwAutoAck);








static inline bool CC2420ControlP__CC2420Config__isHwAutoAckDefault(void );






static inline bool CC2420ControlP__CC2420Config__isAutoAckEnabled(void );




static inline error_t CC2420ControlP__ReadRssi__read(void );




static inline void CC2420ControlP__SyncResource__granted(void );
#line 390
static inline void CC2420ControlP__SpiResource__granted(void );




static inline void CC2420ControlP__RssiResource__granted(void );
#line 408
static inline void CC2420ControlP__StartupTimer__fired(void );









static inline void CC2420ControlP__InterruptCCA__fired(void );
#line 442
static inline void CC2420ControlP__sync__runTask(void );



static inline void CC2420ControlP__syncDone__runTask(void );









static void CC2420ControlP__writeFsctrl(void );
#line 473
static void CC2420ControlP__writeMdmctrl0(void );
#line 492
static void CC2420ControlP__writeId(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t PlatformP__MoteInit__init(void );
#line 51
static error_t PlatformP__MoteClockInit__init(void );
#line 51
static error_t PlatformP__LedsInit__init(void );
# 10 "/opt/tinyos-2.x/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void );
# 6 "/opt/tinyos-2.x/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__uwait(uint16_t u);




static __inline void MotePlatformC__TOSH_wait(void );




static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set);










static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void );
#line 56
static inline error_t MotePlatformC__Init__init(void );
# 32 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__initTimerB(void );
#line 31
static void Msp430ClockP__Msp430ClockInit__initTimerA(void );
#line 29
static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__initClocks(void );
# 40 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static volatile uint8_t Msp430ClockP__IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP__TA0CTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP__TA0IV __asm ("0x012E");
static volatile uint16_t Msp430ClockP__TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP__TBIV __asm ("0x011E");

enum Msp430ClockP____nesc_unnamed4329 {

  Msp430ClockP__ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP__TARGET_DCO_DELTA = 4096 / 32 * Msp430ClockP__ACLK_CALIB_PERIOD
};

static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );



static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 68
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 89
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 104
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 119
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );





static inline void Msp430ClockP__startTimerA(void );
#line 152
static inline void Msp430ClockP__startTimerB(void );
#line 164
static void Msp430ClockP__set_dco_calib(int calib);





static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib);
#line 193
static inline void Msp430ClockP__busyCalibrateDco(void );
#line 218
static inline error_t Msp430ClockP__Init__init(void );
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0xf721cb28);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void );
# 80 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setMode(int mode);









static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__clear(void );









static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__disableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setClockSource(uint16_t clockSource);




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setInputDivider(uint16_t inputDivider);




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0xf721cb28);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void );
# 51 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
#line 70
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
#line 115
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );








static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void );
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__CC2int(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
#line 89
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControl(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t x);
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(uint16_t x);
#line 169
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void );
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__CC2int(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
#line 89
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__setControl(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t x);
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__setEvent(uint16_t x);
#line 169
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void );
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t;


static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void );
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void );
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void );
#line 74
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );
#line 119
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x);
#line 169
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void );
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)  ;
#line 61
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__captureControl(uint8_t l_cm);
#line 74
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void );
#line 99
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm);
#line 119
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 164
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void );




static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
#line 181
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void );
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get(void );
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__CC2int(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__compareControl(void );
#line 74
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__setControlAsCompare(void );
#line 119
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t x);
#line 169
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void );
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__get(void );
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t;


static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__clearPendingInterrupt(void );
#line 119
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEventFromNow(uint16_t x);
#line 169
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void );
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t;


static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void );
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t;


static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void );
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t;


static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP__VectorTimerB1__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerA0__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerA1__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerB0__fired(void );
# 11 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(12)))  ;
void sig_TIMERA1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(10)))  ;
void sig_TIMERB0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(26)))  ;
void sig_TIMERB1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(24)))  ;
# 54 "/opt/tinyos-2.x/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void );
# 51 "/opt/tinyos-2.x/tos/chips/msp430/McuSleepC.nc"
bool McuSleepC__dirty = TRUE;
mcu_power_t McuSleepC__powerState = MSP430_POWER_ACTIVE;




const uint16_t McuSleepC__msp430PowerBits[MSP430_POWER_LPM4 + 1] = { 
0, 
0x0010, 
0x0040 + 0x0010, 
0x0080 + 0x0010, 
0x0080 + 0x0040 + 0x0010, 
0x0080 + 0x0040 + 0x0020 + 0x0010 };


static inline mcu_power_t McuSleepC__getPowerState(void );
#line 104
static inline void McuSleepC__computePowerState(void );




static inline void McuSleepC__McuSleep__sleep(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t RealMainP__SoftwareInit__init(void );
# 49 "/opt/tinyos-2.x/tos/interfaces/Boot.nc"
static void RealMainP__Boot__booted(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t RealMainP__PlatformInit__init(void );
# 46 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
static void RealMainP__Scheduler__init(void );
#line 61
static void RealMainP__Scheduler__taskLoop(void );
#line 54
static bool RealMainP__Scheduler__runNextTask(void );
# 52 "/opt/tinyos-2.x/tos/system/RealMainP.nc"
int main(void )   ;
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(
# 45 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0xf74014a0);
# 59 "/opt/tinyos-2.x/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
# 50 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4330 {

  SchedulerBasicP__NUM_TASKS = 32U, 
  SchedulerBasicP__NO_TASK = 255
};

uint8_t SchedulerBasicP__m_head;
uint8_t SchedulerBasicP__m_tail;
uint8_t SchedulerBasicP__m_next[SchedulerBasicP__NUM_TASKS];








static __inline uint8_t SchedulerBasicP__popTask(void );
#line 86
static inline bool SchedulerBasicP__isWaiting(uint8_t id);




static inline bool SchedulerBasicP__pushTask(uint8_t id);
#line 113
static inline void SchedulerBasicP__Scheduler__init(void );









static bool SchedulerBasicP__Scheduler__runNextTask(void );
#line 138
static inline void SchedulerBasicP__Scheduler__taskLoop(void );
#line 159
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id);




static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id);
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time);

static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void );
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void );
# 46 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void );
#line 36
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void );










static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void );
#line 33
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 42 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
#line 54
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );




static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );










static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void );
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void );
# 38 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );









static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__get(void );






static bool /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void );










static void /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__overflow(void );
# 56 "/opt/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
/*Counter32khz32C.Transform*/TransformCounterC__0__upper_count_type /*Counter32khz32C.Transform*/TransformCounterC__0__m_upper;

enum /*Counter32khz32C.Transform*/TransformCounterC__0____nesc_unnamed4331 {

  TransformCounterC__0__LOW_SHIFT_RIGHT = 0, 
  TransformCounterC__0__HIGH_SHIFT_LEFT = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type ) - /*Counter32khz32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT, 
  TransformCounterC__0__NUM_UPPER_BITS = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type ) - 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type ) + 0, 



  TransformCounterC__0__OVERFLOW_MASK = /*Counter32khz32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS ? ((/*Counter32khz32C.Transform*/TransformCounterC__0__upper_count_type )2 << (/*Counter32khz32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get(void );
#line 122
static inline void /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__fired(void );
#line 92
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt);
#line 62
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void );
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__get(void );
# 66 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0;
/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt;

enum /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0____nesc_unnamed4332 {

  TransformAlarmC__0__MAX_DELAY_LOG2 = 8 * sizeof(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type ) - 1 - 0, 
  TransformAlarmC__0__MAX_DELAY = (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type )1 << /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY_LOG2
};

static inline /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 91
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__set_alarm(void );
#line 136
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type dt);









static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type dt);




static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
#line 166
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 48 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void );
#line 48
static inline uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void );
#line 48
static inline uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void );
#line 45
static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__set(void );
static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__clr(void );

static inline uint8_t /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__get(void );
static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__makeInput(void );

static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__makeOutput(void );
#line 45
static void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__set(void );
static void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__clr(void );



static inline void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__makeInput(void );

static inline void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__makeOutput(void );

static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void );
#line 54
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void );
#line 54
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
#line 48
static inline uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void );
#line 45
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void );
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void );
#line 45
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void );
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void );
#line 45
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );

static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle(void );




static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 45
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );

static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__toggle(void );




static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 45
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );

static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle(void );




static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__clr(void );



static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectIOFunc(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__clr(void );



static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectIOFunc(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectIOFunc(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectIOFunc(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectIOFunc(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectIOFunc(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectIOFunc(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectIOFunc(void );
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__0__HplGeneralIO__makeInput(void );
#line 59
static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__0__HplGeneralIO__get(void );
# 40 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__0__GeneralIO__get(void );
static inline void /*HplCC2420PinsC.CCAM*/Msp430GpioC__0__GeneralIO__makeInput(void );
# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__HplGeneralIO__makeOutput(void );
#line 34
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__HplGeneralIO__clr(void );
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__GeneralIO__makeOutput(void );
# 59 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__2__HplGeneralIO__get(void );
# 40 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__2__GeneralIO__get(void );
# 59 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__3__HplGeneralIO__get(void );
# 40 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__3__GeneralIO__get(void );
# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__4__HplGeneralIO__makeOutput(void );
#line 34
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__4__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__4__HplGeneralIO__clr(void );
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__4__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__4__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__4__GeneralIO__makeOutput(void );
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__5__HplGeneralIO__makeInput(void );
#line 59
static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__5__HplGeneralIO__get(void );
# 40 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__5__GeneralIO__get(void );
static inline void /*HplCC2420PinsC.SFDM*/Msp430GpioC__5__GeneralIO__makeInput(void );
# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__6__HplGeneralIO__makeOutput(void );
#line 34
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__6__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__6__HplGeneralIO__clr(void );
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__6__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__6__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__6__GeneralIO__makeOutput(void );
# 57 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow(void );
# 50 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captured(uint16_t time);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(uint8_t cm);

static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__enableEvents(void );
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents(void );
#line 33
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectIOFunc(void );
#line 78
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectModuleFunc(void );
# 38 "/opt/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(uint8_t mode);
#line 50
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge(void );



static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge(void );



static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable(void );






static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time);
# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void HplMsp430InterruptP__Port14__fired(void );
#line 61
static void HplMsp430InterruptP__Port26__fired(void );
#line 61
static void HplMsp430InterruptP__Port17__fired(void );
#line 61
static void HplMsp430InterruptP__Port21__fired(void );
#line 61
static void HplMsp430InterruptP__Port12__fired(void );
#line 61
static void HplMsp430InterruptP__Port24__fired(void );
#line 61
static void HplMsp430InterruptP__Port15__fired(void );
#line 61
static void HplMsp430InterruptP__Port27__fired(void );
#line 61
static void HplMsp430InterruptP__Port10__fired(void );
#line 61
static void HplMsp430InterruptP__Port22__fired(void );
#line 61
static void HplMsp430InterruptP__Port13__fired(void );
#line 61
static void HplMsp430InterruptP__Port25__fired(void );
#line 61
static void HplMsp430InterruptP__Port16__fired(void );
#line 61
static void HplMsp430InterruptP__Port20__fired(void );
#line 61
static void HplMsp430InterruptP__Port11__fired(void );
#line 61
static void HplMsp430InterruptP__Port23__fired(void );
# 53 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
void sig_PORT1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(8)))  ;
#line 68
static inline void HplMsp430InterruptP__Port11__default__fired(void );
static inline void HplMsp430InterruptP__Port12__default__fired(void );
static inline void HplMsp430InterruptP__Port13__default__fired(void );


static inline void HplMsp430InterruptP__Port16__default__fired(void );
static inline void HplMsp430InterruptP__Port17__default__fired(void );
static inline void HplMsp430InterruptP__Port10__enable(void );



static inline void HplMsp430InterruptP__Port14__enable(void );
static inline void HplMsp430InterruptP__Port15__enable(void );


static inline void HplMsp430InterruptP__Port10__disable(void );



static inline void HplMsp430InterruptP__Port14__disable(void );
static inline void HplMsp430InterruptP__Port15__disable(void );


static inline void HplMsp430InterruptP__Port10__clear(void );
static inline void HplMsp430InterruptP__Port11__clear(void );
static inline void HplMsp430InterruptP__Port12__clear(void );
static inline void HplMsp430InterruptP__Port13__clear(void );
static inline void HplMsp430InterruptP__Port14__clear(void );
static inline void HplMsp430InterruptP__Port15__clear(void );
static inline void HplMsp430InterruptP__Port16__clear(void );
static inline void HplMsp430InterruptP__Port17__clear(void );








static inline void HplMsp430InterruptP__Port10__edge(bool l2h);
#line 131
static inline void HplMsp430InterruptP__Port14__edge(bool l2h);





static inline void HplMsp430InterruptP__Port15__edge(bool l2h);
#line 158
void sig_PORT2_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(2)))  ;
#line 171
static inline void HplMsp430InterruptP__Port20__default__fired(void );
static inline void HplMsp430InterruptP__Port21__default__fired(void );
static inline void HplMsp430InterruptP__Port22__default__fired(void );
static inline void HplMsp430InterruptP__Port23__default__fired(void );
static inline void HplMsp430InterruptP__Port24__default__fired(void );
static inline void HplMsp430InterruptP__Port25__default__fired(void );
static inline void HplMsp430InterruptP__Port26__default__fired(void );
static inline void HplMsp430InterruptP__Port27__default__fired(void );
#line 195
static inline void HplMsp430InterruptP__Port20__clear(void );
static inline void HplMsp430InterruptP__Port21__clear(void );
static inline void HplMsp430InterruptP__Port22__clear(void );
static inline void HplMsp430InterruptP__Port23__clear(void );
static inline void HplMsp430InterruptP__Port24__clear(void );
static inline void HplMsp430InterruptP__Port25__clear(void );
static inline void HplMsp430InterruptP__Port26__clear(void );
static inline void HplMsp430InterruptP__Port27__clear(void );
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear(void );
#line 36
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__disable(void );
#line 56
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__edge(bool low_to_high);
#line 31
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__enable(void );
# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__fired(void );
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__enable(bool rising);








static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void );







static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable(void );







static inline void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear(void );
#line 36
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__disable(void );
#line 56
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__edge(bool low_to_high);
#line 31
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__enable(void );
# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__fired(void );
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__enable(bool rising);
#line 54
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge(void );



static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable(void );







static inline void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired(void );
# 59 "/opt/tinyos-2.x/tos/interfaces/SpiPacket.nc"
static error_t CC2420SpiP__SpiPacket__send(
#line 48
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
# 91 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420SpiP__Fifo__writeDone(
# 46 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0xf6ef53f8, 
# 91 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420SpiP__Fifo__readDone(
# 46 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0xf6ef53f8, 
# 71 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
# 24 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420SpiP__ChipSpiResource__releasing(void );
# 34 "/opt/tinyos-2.x/tos/interfaces/SpiByte.nc"
static uint8_t CC2420SpiP__SpiByte__write(uint8_t tx);
# 56 "/opt/tinyos-2.x/tos/interfaces/State.nc"
static void CC2420SpiP__WorkingState__toIdle(void );




static bool CC2420SpiP__WorkingState__isIdle(void );
#line 45
static error_t CC2420SpiP__WorkingState__requestState(uint8_t reqState);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__SpiResource__release(void );
#line 87
static error_t CC2420SpiP__SpiResource__immediateRequest(void );
#line 78
static error_t CC2420SpiP__SpiResource__request(void );
#line 118
static bool CC2420SpiP__SpiResource__isOwner(void );
#line 92
static void CC2420SpiP__Resource__granted(
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0xf6ef6910);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t CC2420SpiP__grant__postTask(void );
# 88 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
enum CC2420SpiP____nesc_unnamed4333 {
#line 88
  CC2420SpiP__grant = 5U
};
#line 88
typedef int CC2420SpiP____nesc_sillytask_grant[CC2420SpiP__grant];
#line 63
enum CC2420SpiP____nesc_unnamed4334 {
  CC2420SpiP__RESOURCE_COUNT = 5U, 
  CC2420SpiP__NO_HOLDER = 0xFF
};


enum CC2420SpiP____nesc_unnamed4335 {
  CC2420SpiP__S_IDLE, 
  CC2420SpiP__S_BUSY
};


uint16_t CC2420SpiP__m_addr;


uint8_t CC2420SpiP__m_requests = 0;


uint8_t CC2420SpiP__m_holder = CC2420SpiP__NO_HOLDER;


bool CC2420SpiP__release;


static error_t CC2420SpiP__attemptRelease(void );







static inline void CC2420SpiP__ChipSpiResource__abortRelease(void );






static inline error_t CC2420SpiP__ChipSpiResource__attemptRelease(void );




static error_t CC2420SpiP__Resource__request(uint8_t id);
#line 126
static error_t CC2420SpiP__Resource__immediateRequest(uint8_t id);
#line 149
static error_t CC2420SpiP__Resource__release(uint8_t id);
#line 178
static inline uint8_t CC2420SpiP__Resource__isOwner(uint8_t id);





static inline void CC2420SpiP__SpiResource__granted(void );




static cc2420_status_t CC2420SpiP__Fifo__beginRead(uint8_t addr, uint8_t *data, 
uint8_t len);
#line 209
static inline error_t CC2420SpiP__Fifo__continueRead(uint8_t addr, uint8_t *data, 
uint8_t len);



static inline cc2420_status_t CC2420SpiP__Fifo__write(uint8_t addr, uint8_t *data, 
uint8_t len);
#line 260
static cc2420_status_t CC2420SpiP__Ram__write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len);
#line 287
static inline cc2420_status_t CC2420SpiP__Reg__read(uint8_t addr, uint16_t *data);
#line 305
static cc2420_status_t CC2420SpiP__Reg__write(uint8_t addr, uint16_t data);
#line 318
static cc2420_status_t CC2420SpiP__Strobe__strobe(uint8_t addr);










static void CC2420SpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error);








static error_t CC2420SpiP__attemptRelease(void );
#line 358
static inline void CC2420SpiP__grant__runTask(void );








static inline void CC2420SpiP__Resource__default__granted(uint8_t id);


static inline void CC2420SpiP__Fifo__default__readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error);


static inline void CC2420SpiP__Fifo__default__writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error);
# 74 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
uint8_t StateImplP__state[4U];

enum StateImplP____nesc_unnamed4336 {
  StateImplP__S_IDLE = 0
};


static inline error_t StateImplP__Init__init(void );
#line 96
static error_t StateImplP__State__requestState(uint8_t id, uint8_t reqState);
#line 111
static inline void StateImplP__State__forceState(uint8_t id, uint8_t reqState);






static inline void StateImplP__State__toIdle(uint8_t id);







static inline bool StateImplP__State__isIdle(uint8_t id);






static bool StateImplP__State__isState(uint8_t id, uint8_t myState);
# 71 "/opt/tinyos-2.x/tos/interfaces/SpiPacket.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(
# 44 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5c598, 
# 64 "/opt/tinyos-2.x/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(
# 47 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5b7f8);
# 180 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__enableRxIntr(void );
#line 197
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__clrRxIntr(void );
#line 97
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(bool reset);
#line 177
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr(void );
#line 224
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(uint8_t data);
#line 168
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 231
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx(void );
#line 192
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending(void );
#line 158
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi(void );
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(
# 46 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5cd90);
# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__immediateRequest(
# 46 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5cd90);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(
# 46 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5cd90);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__isOwner(
# 46 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5cd90);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0xf6e5d030);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__postTask(void );
# 67 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
enum /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0____nesc_unnamed4337 {
#line 67
  Msp430SpiNoDmaP__0__signalDone_task = 6U
};
#line 67
typedef int /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0____nesc_sillytask_signalDone_task[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task];
#line 56
enum /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0____nesc_unnamed4338 {
  Msp430SpiNoDmaP__0__SPI_ATOMIC_SIZE = 2
};

uint16_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len;
uint8_t * /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf;
uint8_t * /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf;
uint16_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos;
uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_client;

static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void );


static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(uint8_t id);



static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(uint8_t id);



static inline uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(uint8_t id);



static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(uint8_t id);





static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(uint8_t id);



static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(uint8_t tx);
#line 111
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(uint8_t id);
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(uint8_t id);
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(uint8_t id);
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(uint8_t id);
static inline msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(uint8_t id);

static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp(void );
#line 144
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len);
#line 166
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void );



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data);
#line 183
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void );




static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void );

static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error);
# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__UCLK__selectIOFunc(void );
#line 78
static void HplMsp430Usart0P__UCLK__selectModuleFunc(void );
# 54 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void HplMsp430Usart0P__Interrupts__rxDone(uint8_t data);
#line 49
static void HplMsp430Usart0P__Interrupts__txDone(void );
# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__URXD__selectIOFunc(void );
#line 85
static void HplMsp430Usart0P__UTXD__selectIOFunc(void );
# 7 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C.nc"
static void HplMsp430Usart0P__HplI2C__clearModeI2C(void );
#line 6
static bool HplMsp430Usart0P__HplI2C__isI2C(void );
# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__SOMI__selectIOFunc(void );
#line 78
static void HplMsp430Usart0P__SOMI__selectModuleFunc(void );
# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void HplMsp430Usart0P__I2CInterrupts__fired(void );
# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__SIMO__selectIOFunc(void );
#line 78
static void HplMsp430Usart0P__SIMO__selectModuleFunc(void );
# 89 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static volatile uint8_t HplMsp430Usart0P__IE1 __asm ("0x0000");
static volatile uint8_t HplMsp430Usart0P__ME1 __asm ("0x0004");
static volatile uint8_t HplMsp430Usart0P__IFG1 __asm ("0x0002");
static volatile uint8_t HplMsp430Usart0P__U0TCTL __asm ("0x0071");

static volatile uint8_t HplMsp430Usart0P__U0TXBUF __asm ("0x0077");

void sig_UART0RX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(18)))  ;




void sig_UART0TX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(16)))  ;
#line 132
static inline void HplMsp430Usart0P__Usart__setUbr(uint16_t control);










static inline void HplMsp430Usart0P__Usart__setUmctl(uint8_t control);







static inline void HplMsp430Usart0P__Usart__resetUsart(bool reset);
#line 207
static inline void HplMsp430Usart0P__Usart__disableUart(void );
#line 238
static inline void HplMsp430Usart0P__Usart__enableSpi(void );








static void HplMsp430Usart0P__Usart__disableSpi(void );








static inline void HplMsp430Usart0P__configSpi(msp430_spi_union_config_t *config);








static void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 330
static inline bool HplMsp430Usart0P__Usart__isRxIntrPending(void );










static inline void HplMsp430Usart0P__Usart__clrRxIntr(void );



static inline void HplMsp430Usart0P__Usart__clrIntr(void );



static inline void HplMsp430Usart0P__Usart__disableRxIntr(void );







static inline void HplMsp430Usart0P__Usart__disableIntr(void );



static inline void HplMsp430Usart0P__Usart__enableRxIntr(void );
#line 382
static inline void HplMsp430Usart0P__Usart__tx(uint8_t data);



static uint8_t HplMsp430Usart0P__Usart__rx(void );
# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void LedsP__Led0__toggle(void );



static void LedsP__Led0__makeOutput(void );
#line 29
static void LedsP__Led0__set(void );

static void LedsP__Led1__toggle(void );



static void LedsP__Led1__makeOutput(void );
#line 29
static void LedsP__Led1__set(void );

static void LedsP__Led2__toggle(void );



static void LedsP__Led2__makeOutput(void );
#line 29
static void LedsP__Led2__set(void );
# 45 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void );
#line 68
static inline void LedsP__Leds__led0Off(void );




static inline void LedsP__Leds__led0Toggle(void );









static inline void LedsP__Leds__led1Off(void );




static inline void LedsP__Leds__led1Toggle(void );









static inline void LedsP__Leds__led2Off(void );




static inline void LedsP__Leds__led2Toggle(void );
# 44 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__HplGeneralIO__toggle(void );
#line 71
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__HplGeneralIO__makeOutput(void );
#line 34
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__HplGeneralIO__set(void );
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__GeneralIO__set(void );

static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__GeneralIO__toggle(void );



static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__GeneralIO__makeOutput(void );
# 44 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__HplGeneralIO__toggle(void );
#line 71
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__HplGeneralIO__makeOutput(void );
#line 34
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__HplGeneralIO__set(void );
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__GeneralIO__set(void );

static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__GeneralIO__toggle(void );



static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__GeneralIO__makeOutput(void );
# 44 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__HplGeneralIO__toggle(void );
#line 71
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__HplGeneralIO__makeOutput(void );
#line 34
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__HplGeneralIO__set(void );
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__GeneralIO__set(void );

static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__GeneralIO__toggle(void );



static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__GeneralIO__makeOutput(void );
# 80 "/opt/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void );
# 54 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(
# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0xf6d72758, 
# 54 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(
# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0xf6d72758);
# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__fired(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0xf6d71010);








static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(uint8_t id);
# 39 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1____nesc_unnamed4339 {
#line 39
  FcfsResourceQueueC__1__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[1U];
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void );




static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void );



static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void );
#line 72
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id);
# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(
# 55 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4c770);
# 51 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(
# 55 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4c770);
# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(
# 60 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4bb38);
# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(
# 60 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4bb38);
# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(resource_client_id_t id);
#line 43
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void );
#line 60
static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void );
# 73 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested(void );
#line 46
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(
# 54 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4ece8);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void );
# 75 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4340 {
#line 75
  ArbiterP__0__grantedTask = 7U
};
#line 75
typedef int /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_sillytask_grantedTask[/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask];
#line 67
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4341 {
#line 67
  ArbiterP__0__RES_CONTROLLED, ArbiterP__0__RES_GRANTING, ArbiterP__0__RES_IMM_GRANTING, ArbiterP__0__RES_BUSY
};
#line 68
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4342 {
#line 68
  ArbiterP__0__default_owner_id = 1U
};
#line 69
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4343 {
#line 69
  ArbiterP__0__NO_RES = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;



static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(uint8_t id);
#line 90
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id);
#line 108
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id);
#line 130
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 150
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );
#line 163
static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );










static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id);
#line 187
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
#line 199
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void );

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested(void );


static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested(void );


static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id);
# 97 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430I2C0P__HplUsart__resetUsart(bool reset);
# 49 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static volatile uint8_t HplMsp430I2C0P__U0CTL __asm ("0x0070");





static inline bool HplMsp430I2C0P__HplI2C__isI2C(void );



static inline void HplMsp430I2C0P__HplI2C__clearModeI2C(void );
# 51 "/opt/tinyos-2.x/tos/system/ActiveMessageAddressC.nc"
am_addr_t ActiveMessageAddressC__addr = TOS_AM_ADDRESS;


am_group_t ActiveMessageAddressC__group = TOS_AM_GROUP;






static inline am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );
#line 82
static inline am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void );
#line 95
static am_addr_t ActiveMessageAddressC__amAddress(void );
# 81 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420TransmitP__RadioBackoff__requestInitialBackoff(message_t * msg);






static void CC2420TransmitP__RadioBackoff__requestCongestionBackoff(message_t * msg);
# 59 "/opt/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
static void CC2420TransmitP__PacketTimeStamp__clear(
#line 55
message_t * msg);
#line 67
static void CC2420TransmitP__PacketTimeStamp__set(
#line 62
message_t * msg, 




CC2420TransmitP__PacketTimeStamp__size_type value);
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__STXONCCA__strobe(void );
# 43 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
static error_t CC2420TransmitP__CaptureSFD__captureFallingEdge(void );
#line 55
static void CC2420TransmitP__CaptureSFD__disable(void );
#line 42
static error_t CC2420TransmitP__CaptureSFD__captureRisingEdge(void );
# 98 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static CC2420TransmitP__BackoffTimer__size_type CC2420TransmitP__BackoffTimer__getNow(void );
#line 55
static void CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__BackoffTimer__size_type dt);






static void CC2420TransmitP__BackoffTimer__stop(void );
# 63 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420TransmitP__TXFIFO_RAM__write(uint8_t offset, uint8_t * data, uint8_t length);
# 55 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420TransmitP__TXCTRL__write(uint16_t data);
# 55 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420TransmitP__CC2420Receive__sfd_dropped(void );
#line 49
static void CC2420TransmitP__CC2420Receive__sfd(uint32_t time);
# 73 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static void CC2420TransmitP__Send__sendDone(message_t * p_msg, error_t error);
# 31 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420TransmitP__ChipSpiResource__abortRelease(void );







static error_t CC2420TransmitP__ChipSpiResource__attemptRelease(void );
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__SFLUSHTX__strobe(void );
# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__CSN__makeOutput(void );
#line 29
static void CC2420TransmitP__CSN__set(void );
static void CC2420TransmitP__CSN__clr(void );
# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420TransmitP__CC2420PacketBody__getHeader(message_t * msg);




static cc2420_metadata_t * CC2420TransmitP__CC2420PacketBody__getMetadata(message_t * msg);
# 47 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
static uint8_t CC2420TransmitP__PacketTimeSyncOffset__get(
#line 42
message_t * msg);
#line 39
static bool CC2420TransmitP__PacketTimeSyncOffset__isSet(
#line 35
message_t * msg);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420TransmitP__SpiResource__release(void );
#line 87
static error_t CC2420TransmitP__SpiResource__immediateRequest(void );
#line 78
static error_t CC2420TransmitP__SpiResource__request(void );
# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__CCA__makeInput(void );
#line 32
static bool CC2420TransmitP__CCA__get(void );
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__SNOP__strobe(void );
# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__SFD__makeInput(void );
#line 32
static bool CC2420TransmitP__SFD__get(void );
# 82 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static cc2420_status_t CC2420TransmitP__TXFIFO__write(uint8_t * data, uint8_t length);
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__STXON__strobe(void );
# 99 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
#line 89
typedef enum CC2420TransmitP____nesc_unnamed4344 {
  CC2420TransmitP__S_STOPPED, 
  CC2420TransmitP__S_STARTED, 
  CC2420TransmitP__S_LOAD, 
  CC2420TransmitP__S_SAMPLE_CCA, 
  CC2420TransmitP__S_BEGIN_TRANSMIT, 
  CC2420TransmitP__S_SFD, 
  CC2420TransmitP__S_EFD, 
  CC2420TransmitP__S_ACK_WAIT, 
  CC2420TransmitP__S_CANCEL
} CC2420TransmitP__cc2420_transmit_state_t;





enum CC2420TransmitP____nesc_unnamed4345 {
  CC2420TransmitP__CC2420_ABORT_PERIOD = 320
};
#line 120
message_t * CC2420TransmitP__m_msg;

bool CC2420TransmitP__m_cca;

uint8_t CC2420TransmitP__m_tx_power;

CC2420TransmitP__cc2420_transmit_state_t CC2420TransmitP__m_state = CC2420TransmitP__S_STOPPED;

bool CC2420TransmitP__m_receiving = FALSE;

uint16_t CC2420TransmitP__m_prev_time;


bool CC2420TransmitP__sfdHigh;


bool CC2420TransmitP__abortSpiRelease;


int8_t CC2420TransmitP__totalCcaChecks;


uint16_t CC2420TransmitP__myInitialBackoff;


uint16_t CC2420TransmitP__myCongestionBackoff;



static inline error_t CC2420TransmitP__send(message_t * p_msg, bool cca);

static void CC2420TransmitP__loadTXFIFO(void );
static void CC2420TransmitP__attemptSend(void );
static void CC2420TransmitP__congestionBackoff(void );
static error_t CC2420TransmitP__acquireSpiResource(void );
static inline error_t CC2420TransmitP__releaseSpiResource(void );
static void CC2420TransmitP__signalDone(error_t err);



static inline error_t CC2420TransmitP__Init__init(void );







static inline error_t CC2420TransmitP__StdControl__start(void );










static inline error_t CC2420TransmitP__StdControl__stop(void );
#line 192
static inline error_t CC2420TransmitP__Send__send(message_t * p_msg, bool useCca);
#line 243
static inline void CC2420TransmitP__RadioBackoff__setInitialBackoff(uint16_t backoffTime);







static inline void CC2420TransmitP__RadioBackoff__setCongestionBackoff(uint16_t backoffTime);







static __inline uint32_t CC2420TransmitP__getTime32(uint16_t time);
#line 278
static inline void CC2420TransmitP__CaptureSFD__captured(uint16_t time);
#line 375
static inline void CC2420TransmitP__ChipSpiResource__releasing(void );
#line 387
static inline void CC2420TransmitP__CC2420Receive__receive(uint8_t type, message_t *ack_msg);
#line 414
static inline void CC2420TransmitP__SpiResource__granted(void );
#line 452
static inline void CC2420TransmitP__TXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error);
#line 484
static inline void CC2420TransmitP__TXFIFO__readDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error);










static inline void CC2420TransmitP__BackoffTimer__fired(void );
#line 545
static inline error_t CC2420TransmitP__send(message_t * p_msg, bool cca);
#line 735
static void CC2420TransmitP__attemptSend(void );
#line 780
static void CC2420TransmitP__congestionBackoff(void );






static error_t CC2420TransmitP__acquireSpiResource(void );







static inline error_t CC2420TransmitP__releaseSpiResource(void );
#line 817
static void CC2420TransmitP__loadTXFIFO(void );
#line 842
static void CC2420TransmitP__signalDone(error_t err);
# 32 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static bool CC2420ReceiveP__FIFO__get(void );
# 86 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
static bool CC2420ReceiveP__CC2420Config__isAddressRecognitionEnabled(void );
#line 110
static bool CC2420ReceiveP__CC2420Config__isAutoAckEnabled(void );
#line 105
static bool CC2420ReceiveP__CC2420Config__isHwAutoAckDefault(void );
#line 64
static uint16_t CC2420ReceiveP__CC2420Config__getShortAddr(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t CC2420ReceiveP__receiveDone_task__postTask(void );
# 59 "/opt/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
static void CC2420ReceiveP__PacketTimeStamp__clear(
#line 55
message_t * msg);
#line 67
static void CC2420ReceiveP__PacketTimeStamp__set(
#line 62
message_t * msg, 




CC2420ReceiveP__PacketTimeStamp__size_type value);
# 32 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static bool CC2420ReceiveP__FIFOP__get(void );
# 63 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420ReceiveP__CC2420Receive__receive(uint8_t type, message_t * message);
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ReceiveP__SACK__strobe(void );
# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void CC2420ReceiveP__CSN__set(void );
static void CC2420ReceiveP__CSN__clr(void );
# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420ReceiveP__CC2420PacketBody__getHeader(message_t * msg);




static cc2420_metadata_t * CC2420ReceiveP__CC2420PacketBody__getMetadata(message_t * msg);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420ReceiveP__Receive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420ReceiveP__SpiResource__release(void );
#line 87
static error_t CC2420ReceiveP__SpiResource__immediateRequest(void );
#line 78
static error_t CC2420ReceiveP__SpiResource__request(void );
#line 118
static bool CC2420ReceiveP__SpiResource__isOwner(void );
# 62 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static error_t CC2420ReceiveP__RXFIFO__continueRead(uint8_t * data, uint8_t length);
#line 51
static cc2420_status_t CC2420ReceiveP__RXFIFO__beginRead(uint8_t * data, uint8_t length);
# 50 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static error_t CC2420ReceiveP__InterruptFIFOP__disable(void );
#line 43
static error_t CC2420ReceiveP__InterruptFIFOP__enableFallingEdge(void );
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ReceiveP__SFLUSHRX__strobe(void );
# 148 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
enum CC2420ReceiveP____nesc_unnamed4346 {
#line 148
  CC2420ReceiveP__receiveDone_task = 8U
};
#line 148
typedef int CC2420ReceiveP____nesc_sillytask_receiveDone_task[CC2420ReceiveP__receiveDone_task];
#line 89
#line 81
typedef enum CC2420ReceiveP____nesc_unnamed4347 {
  CC2420ReceiveP__S_STOPPED, 
  CC2420ReceiveP__S_STARTED, 
  CC2420ReceiveP__S_RX_LENGTH, 
  CC2420ReceiveP__S_RX_DEC, 
  CC2420ReceiveP__S_RX_DEC_WAIT, 
  CC2420ReceiveP__S_RX_FCF, 
  CC2420ReceiveP__S_RX_PAYLOAD
} CC2420ReceiveP__cc2420_receive_state_t;

enum CC2420ReceiveP____nesc_unnamed4348 {
  CC2420ReceiveP__RXFIFO_SIZE = 128, 
  CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE = 8, 
  CC2420ReceiveP__SACK_HEADER_LENGTH = 7
};

uint32_t CC2420ReceiveP__m_timestamp_queue[CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE];

uint8_t CC2420ReceiveP__m_timestamp_head;

uint8_t CC2420ReceiveP__m_timestamp_size;





uint8_t CC2420ReceiveP__m_missed_packets;



bool CC2420ReceiveP__receivingPacket;


uint8_t CC2420ReceiveP__rxFrameLength;

uint8_t CC2420ReceiveP__m_bytes_left;

message_t * CC2420ReceiveP__m_p_rx_buf;

message_t CC2420ReceiveP__m_rx_buf;
#line 137
CC2420ReceiveP__cc2420_receive_state_t CC2420ReceiveP__m_state;



static void CC2420ReceiveP__reset_state(void );
static void CC2420ReceiveP__beginReceive(void );
static void CC2420ReceiveP__receive(void );
static void CC2420ReceiveP__waitForNextPacket(void );
static void CC2420ReceiveP__flush(void );
static inline bool CC2420ReceiveP__passesAddressCheck(message_t * msg);




static inline error_t CC2420ReceiveP__Init__init(void );





static inline error_t CC2420ReceiveP__StdControl__start(void );
#line 171
static inline error_t CC2420ReceiveP__StdControl__stop(void );
#line 186
static inline void CC2420ReceiveP__CC2420Receive__sfd(uint32_t time);








static inline void CC2420ReceiveP__CC2420Receive__sfd_dropped(void );
#line 212
static inline void CC2420ReceiveP__InterruptFIFOP__fired(void );
#line 508
static inline void CC2420ReceiveP__SpiResource__granted(void );
#line 525
static inline void CC2420ReceiveP__RXFIFO__readDone(uint8_t *rx_buf, uint8_t rx_len, 
error_t error);
#line 663
static inline void CC2420ReceiveP__RXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error);







static inline void CC2420ReceiveP__receiveDone_task__runTask(void );
#line 704
static inline void CC2420ReceiveP__CC2420Config__syncDone(error_t error);






static void CC2420ReceiveP__beginReceive(void );
#line 728
static void CC2420ReceiveP__flush(void );
#line 754
static void CC2420ReceiveP__receive(void );









static void CC2420ReceiveP__waitForNextPacket(void );
#line 808
static void CC2420ReceiveP__reset_state(void );










static inline bool CC2420ReceiveP__passesAddressCheck(message_t *msg);
# 80 "/opt/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline void CC2420PacketP__CC2420Packet__setPower(message_t *p_msg, uint8_t power);









static inline int8_t CC2420PacketP__CC2420Packet__getRssi(message_t *p_msg);







static uint8_t CC2420PacketP__CC2420Packet__getNetwork(message_t *p_msg);







static void CC2420PacketP__CC2420Packet__setNetwork(message_t *p_msg, uint8_t networkId);







static inline cc2420_header_t * CC2420PacketP__CC2420PacketBody__getHeader(message_t * msg);



static inline cc2420_metadata_t *CC2420PacketP__CC2420PacketBody__getMetadata(message_t *msg);
#line 137
static void CC2420PacketP__PacketTimeStamp32khz__clear(message_t *msg);





static inline void CC2420PacketP__PacketTimeStamp32khz__set(message_t *msg, uint32_t value);
#line 176
static inline bool CC2420PacketP__PacketTimeSyncOffset__isSet(message_t *msg);








static inline uint8_t CC2420PacketP__PacketTimeSyncOffset__get(message_t *msg);
# 47 "/opt/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void );
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void );
# 46 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void );
#line 36
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void );
#line 33
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void );
# 42 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Init__init(void );
#line 54
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__overflow(void );
# 56 "/opt/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC__1__upper_count_type /*CounterMilli32C.Transform*/TransformCounterC__1__m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC__1____nesc_unnamed4349 {

  TransformCounterC__1__LOW_SHIFT_RIGHT = 5, 
  TransformCounterC__1__HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC__1__LOW_SHIFT_RIGHT, 
  TransformCounterC__1__NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type ) + 5, 



  TransformCounterC__1__OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC__1__NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC__1__upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC__1__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__get(void );
#line 122
static inline void /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void );
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__fired(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__stop(void );
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__get(void );
# 66 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1____nesc_unnamed4350 {

  TransformAlarmC__1__MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_size_type ) - 1 - 5, 
  TransformAlarmC__1__MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__MAX_DELAY_LOG2
};

static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getNow(void );




static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getAlarm(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__set_alarm(void );
#line 136
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type dt);
#line 151
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__fired(void );
#line 166
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__overflow(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void );
# 98 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt);
#line 105
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void );
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void );
# 63 "/opt/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_unnamed4351 {
#line 63
  AlarmToTimerC__0__fired = 9U
};
#line 63
typedef int /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired];
#line 44
uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt;
bool /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot);
#line 60
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );


static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );






static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
#line 82
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);


static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void );
# 125 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void );
#line 118
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(
# 37 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0xf6b05030);
#line 60
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4352 {
#line 60
  VirtualizeTimerC__0__updateFromTimer = 10U
};
#line 60
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer];
#line 42
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4353 {

  VirtualizeTimerC__0__NUM_TIMERS = 12U, 
  VirtualizeTimerC__0__END_OF_LIST = 255
};








#line 48
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4354 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t;

/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS];




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now);
#line 89
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
#line 128
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);









static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num);




static inline bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(uint8_t num);
#line 193
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num);
# 47 "/opt/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void );
# 41 "/opt/tinyos-2.x/tos/system/RandomMlcgC.nc"
uint32_t RandomMlcgC__seed;


static inline error_t RandomMlcgC__Init__init(void );
#line 58
static uint32_t RandomMlcgC__Random__rand32(void );
#line 78
static inline uint16_t RandomMlcgC__Random__rand16(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static error_t UniqueSendP__SubSend__send(
#line 56
message_t * msg, 







uint8_t len);
#line 101
static uint8_t UniqueSendP__SubSend__maxPayloadLength(void );
#line 89
static void UniqueSendP__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 41 "/opt/tinyos-2.x/tos/interfaces/Random.nc"
static uint16_t UniqueSendP__Random__rand16(void );
# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * UniqueSendP__CC2420PacketBody__getHeader(message_t * msg);
# 56 "/opt/tinyos-2.x/tos/interfaces/State.nc"
static void UniqueSendP__State__toIdle(void );
#line 45
static error_t UniqueSendP__State__requestState(uint8_t reqState);
# 54 "/opt/tinyos-2.x/tos/chips/cc2420/unique/UniqueSendP.nc"
uint8_t UniqueSendP__localSendId;

enum UniqueSendP____nesc_unnamed4355 {
  UniqueSendP__S_IDLE, 
  UniqueSendP__S_SENDING
};


static inline error_t UniqueSendP__Init__init(void );
#line 75
static error_t UniqueSendP__Send__send(message_t *msg, uint8_t len);
#line 95
static inline uint8_t UniqueSendP__Send__maxPayloadLength(void );








static inline void UniqueSendP__SubSend__sendDone(message_t *msg, error_t error);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



UniqueReceiveP__Receive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * UniqueReceiveP__CC2420PacketBody__getHeader(message_t * msg);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



UniqueReceiveP__DuplicateReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 59 "/opt/tinyos-2.x/tos/chips/cc2420/unique/UniqueReceiveP.nc"
#line 56
struct UniqueReceiveP____nesc_unnamed4356 {
  am_addr_t source;
  uint8_t dsn;
} UniqueReceiveP__receivedMessages[4];

uint8_t UniqueReceiveP__writeIndex = 0;


uint8_t UniqueReceiveP__recycleSourceElement;

enum UniqueReceiveP____nesc_unnamed4357 {
  UniqueReceiveP__INVALID_ELEMENT = 0xFF
};


static inline error_t UniqueReceiveP__Init__init(void );









static inline bool UniqueReceiveP__hasSeen(uint16_t msgSource, uint8_t msgDsn);
static inline void UniqueReceiveP__insert(uint16_t msgSource, uint8_t msgDsn);


static inline message_t *UniqueReceiveP__SubReceive__receive(message_t *msg, void *payload, 
uint8_t len);
#line 111
static inline bool UniqueReceiveP__hasSeen(uint16_t msgSource, uint8_t msgDsn);
#line 137
static inline void UniqueReceiveP__insert(uint16_t msgSource, uint8_t msgDsn);
#line 158
static inline message_t *UniqueReceiveP__DuplicateReceive__default__receive(message_t *msg, void *payload, uint8_t len);
# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static error_t CC2420TinyosNetworkP__SubSend__send(
#line 56
message_t * msg, 







uint8_t len);
#line 101
static uint8_t CC2420TinyosNetworkP__SubSend__maxPayloadLength(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t CC2420TinyosNetworkP__grantTask__postTask(void );
# 77 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Packet.nc"
static void CC2420TinyosNetworkP__CC2420Packet__setNetwork(message_t *p_msg, uint8_t networkId);
#line 75
static uint8_t CC2420TinyosNetworkP__CC2420Packet__getNetwork(message_t *p_msg);
# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__ActiveSend__sendDone(
#line 85
message_t * msg, 



error_t error);
# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static error_t CC2420TinyosNetworkP__Queue__enqueue(resource_client_id_t id);
#line 43
static bool CC2420TinyosNetworkP__Queue__isEmpty(void );
#line 60
static resource_client_id_t CC2420TinyosNetworkP__Queue__dequeue(void );
# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420TinyosNetworkP__CC2420PacketBody__getHeader(message_t * msg);




static cc2420_metadata_t * CC2420TinyosNetworkP__CC2420PacketBody__getMetadata(message_t * msg);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420TinyosNetworkP__BareReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void CC2420TinyosNetworkP__Resource__granted(
# 46 "/opt/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0xf6a68628);
# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__BareSend__sendDone(
#line 85
message_t * msg, 



error_t error);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420TinyosNetworkP__ActiveReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 148 "/opt/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
enum CC2420TinyosNetworkP____nesc_unnamed4358 {
#line 148
  CC2420TinyosNetworkP__grantTask = 11U
};
#line 148
typedef int CC2420TinyosNetworkP____nesc_sillytask_grantTask[CC2420TinyosNetworkP__grantTask];
#line 66
enum CC2420TinyosNetworkP____nesc_unnamed4359 {
  CC2420TinyosNetworkP__OWNER_NONE = 0xff, 
  CC2420TinyosNetworkP__TINYOS_N_NETWORKS = 1U
};

uint8_t CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__OWNER_NONE;
#line 71
uint8_t CC2420TinyosNetworkP__next_owner;

static inline error_t CC2420TinyosNetworkP__ActiveSend__send(message_t *msg, uint8_t len);








static inline uint8_t CC2420TinyosNetworkP__ActiveSend__maxPayloadLength(void );



static void *CC2420TinyosNetworkP__ActiveSend__getPayload(message_t *msg, uint8_t len);
#line 118
static inline void CC2420TinyosNetworkP__SubSend__sendDone(message_t *msg, error_t error);








static inline message_t *CC2420TinyosNetworkP__SubReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 148
static inline void CC2420TinyosNetworkP__grantTask__runTask(void );
#line 167
static inline error_t CC2420TinyosNetworkP__Resource__request(uint8_t id);
#line 184
static inline error_t CC2420TinyosNetworkP__Resource__immediateRequest(uint8_t id);
#line 198
static inline error_t CC2420TinyosNetworkP__Resource__release(uint8_t id);
#line 210
static inline message_t *CC2420TinyosNetworkP__BareReceive__default__receive(message_t *msg, void *payload, uint8_t len);


static inline void CC2420TinyosNetworkP__BareSend__default__sendDone(message_t *msg, error_t error);








static inline void CC2420TinyosNetworkP__Resource__default__granted(uint8_t client);
# 39 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
enum /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0____nesc_unnamed4360 {
#line 39
  FcfsResourceQueueC__0__NO_ENTRY = 0xFF
};
uint8_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[1];
uint8_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;
uint8_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;

static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init(void );




static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );



static inline bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
#line 72
static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static error_t CC2420ActiveMessageP__SubSend__send(
#line 56
message_t * msg, 







uint8_t len);
#line 114
static 
#line 112
void * 

CC2420ActiveMessageP__SubSend__getPayload(
#line 111
message_t * msg, 


uint8_t len);
#line 101
static uint8_t CC2420ActiveMessageP__SubSend__maxPayloadLength(void );
# 70 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
static uint16_t CC2420ActiveMessageP__CC2420Config__getPanAddr(void );
# 95 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420ActiveMessageP__RadioBackoff__requestCca(
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0xf6a1d4b8, 
# 95 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);
#line 81
static void CC2420ActiveMessageP__RadioBackoff__requestInitialBackoff(
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0xf6a1d4b8, 
# 81 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);






static void CC2420ActiveMessageP__RadioBackoff__requestCongestionBackoff(
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0xf6a1d4b8, 
# 88 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);
# 59 "/opt/tinyos-2.x/tos/interfaces/SendNotifier.nc"
static void CC2420ActiveMessageP__SendNotifier__aboutToSend(
# 44 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0xf6a1ee20, 
# 59 "/opt/tinyos-2.x/tos/interfaces/SendNotifier.nc"
am_addr_t dest, 
#line 57
message_t * msg);
# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static void CC2420ActiveMessageP__AMSend__sendDone(
# 39 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0xf6a213f8, 
# 92 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420ActiveMessageP__Snoop__receive(
# 41 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0xf6a1f490, 
# 60 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 50 "/opt/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t CC2420ActiveMessageP__ActiveMessageAddress__amAddress(void );
# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420ActiveMessageP__CC2420PacketBody__getHeader(message_t * msg);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420ActiveMessageP__Receive__receive(
# 40 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0xf6a21db8, 
# 60 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420ActiveMessageP__RadioResource__release(void );
#line 87
static error_t CC2420ActiveMessageP__RadioResource__immediateRequest(void );
#line 78
static error_t CC2420ActiveMessageP__RadioResource__request(void );
# 62 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
uint16_t CC2420ActiveMessageP__pending_length;
message_t *CC2420ActiveMessageP__pending_message = (void *)0;

static void CC2420ActiveMessageP__RadioResource__granted(void );
#line 78
static error_t CC2420ActiveMessageP__AMSend__send(am_id_t id, am_addr_t addr, 
message_t *msg, 
uint8_t len);
#line 122
static inline am_addr_t CC2420ActiveMessageP__AMPacket__address(void );



static am_addr_t CC2420ActiveMessageP__AMPacket__destination(message_t *amsg);




static am_addr_t CC2420ActiveMessageP__AMPacket__source(message_t *amsg);




static void CC2420ActiveMessageP__AMPacket__setDestination(message_t *amsg, am_addr_t addr);









static inline bool CC2420ActiveMessageP__AMPacket__isForMe(message_t *amsg);




static inline am_id_t CC2420ActiveMessageP__AMPacket__type(message_t *amsg);




static void CC2420ActiveMessageP__AMPacket__setType(message_t *amsg, am_id_t type);
#line 181
static inline uint8_t CC2420ActiveMessageP__Packet__payloadLength(message_t *msg);



static inline void CC2420ActiveMessageP__Packet__setPayloadLength(message_t *msg, uint8_t len);



static inline uint8_t CC2420ActiveMessageP__Packet__maxPayloadLength(void );



static inline void *CC2420ActiveMessageP__Packet__getPayload(message_t *msg, uint8_t len);





static inline void CC2420ActiveMessageP__SubSend__sendDone(message_t *msg, error_t result);






static inline message_t *CC2420ActiveMessageP__SubReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 222
static inline void CC2420ActiveMessageP__CC2420Config__syncDone(error_t error);





static inline void CC2420ActiveMessageP__SubBackoff__requestInitialBackoff(message_t *msg);




static inline void CC2420ActiveMessageP__SubBackoff__requestCongestionBackoff(message_t *msg);



static inline void CC2420ActiveMessageP__SubBackoff__requestCca(message_t *msg);
#line 266
static inline message_t *CC2420ActiveMessageP__Receive__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len);



static inline message_t *CC2420ActiveMessageP__Snoop__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len);







static inline void CC2420ActiveMessageP__SendNotifier__default__aboutToSend(am_id_t amId, am_addr_t addr, message_t *msg);

static inline void CC2420ActiveMessageP__RadioBackoff__default__requestInitialBackoff(am_id_t id, 
message_t *msg);


static inline void CC2420ActiveMessageP__RadioBackoff__default__requestCongestionBackoff(am_id_t id, 
message_t *msg);


static inline void CC2420ActiveMessageP__RadioBackoff__default__requestCca(am_id_t id, 
message_t *msg);
# 99 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
static void SendingMoteC__CC2420Config__setAutoAck(bool enableAutoAck, bool hwAutoAck);
#line 52
static error_t SendingMoteC__CC2420Config__sync(void );
# 53 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void SendingMoteC__SendTimer__startPeriodic(uint32_t dt);
#line 67
static void SendingMoteC__SendTimer__stop(void );
#line 62
static void SendingMoteC__BootupTimer__startOneShot(uint32_t dt);




static void SendingMoteC__BootupTimer__stop(void );
#line 62
static void SendingMoteC__NoiseFloorTimer__startOneShot(uint32_t dt);




static void SendingMoteC__NoiseFloorTimer__stop(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t SendingMoteC__sendIdentification__postTask(void );
# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static error_t SendingMoteC__Light__read(void );
#line 55
static error_t SendingMoteC__ReadRssi__read(void );
# 115 "/opt/tinyos-2.x/tos/interfaces/Packet.nc"
static 
#line 112
void * 


SendingMoteC__Packet__getPayload(
#line 110
message_t * msg, 




uint8_t len);
# 56 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Packet.nc"
static void SendingMoteC__CC2420Packet__setPower(message_t *p_msg, uint8_t power);







static int8_t SendingMoteC__CC2420Packet__getRssi(message_t *p_msg);
# 81 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static bool SendingMoteC__SensorReadingTimer__isRunning(void );
#line 53
static void SendingMoteC__SensorReadingTimer__startPeriodic(uint32_t dt);
#line 67
static void SendingMoteC__SensorReadingTimer__stop(void );
# 41 "/opt/tinyos-2.x/tos/interfaces/Random.nc"
static uint16_t SendingMoteC__Random__rand16(void );
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static error_t SendingMoteC__RadioControl__start(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t SendingMoteC__sendReport__postTask(void );
#line 56
static error_t SendingMoteC__sendEchoReply__postTask(void );
#line 56
static error_t SendingMoteC__sendCommandACK__postTask(void );
# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static error_t SendingMoteC__Humidity__read(void );
# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void SendingMoteC__Pin0__set(void );




static void SendingMoteC__Pin0__clr(void );
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static error_t SendingMoteC__TinyReportMsgSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
#line 69
static error_t SendingMoteC__ReportMsgSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
#line 69
static error_t SendingMoteC__CommandMsgSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 53 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void SendingMoteC__SignalTimer__startPeriodic(uint32_t dt);
#line 67
static void SendingMoteC__SignalTimer__stop(void );
# 50 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
static void SendingMoteC__Leds__led0Off(void );





static void SendingMoteC__Leds__led0Toggle(void );
#line 72
static void SendingMoteC__Leds__led1Toggle(void );
#line 89
static void SendingMoteC__Leds__led2Toggle(void );
#line 66
static void SendingMoteC__Leds__led1Off(void );
#line 83
static void SendingMoteC__Leds__led2Off(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t SendingMoteC__sendReading__postTask(void );
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static error_t SendingMoteC__RssiMsgSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void SendingMoteC__Pin1__set(void );




static void SendingMoteC__Pin1__clr(void );
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static error_t SendingMoteC__MultiPingResponseSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static error_t SendingMoteC__Temperature__read(void );
# 77 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
static am_addr_t SendingMoteC__AMPacket__source(
#line 73
message_t * amsg);
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void SendingMoteC__FlushTimer__startOneShot(uint32_t dt);




static void SendingMoteC__FlushTimer__stop(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t SendingMoteC__sendMultipleEcho__postTask(void );
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void SendingMoteC__GapTimer__startOneShot(uint32_t dt);




static void SendingMoteC__GapTimer__stop(void );
# 293 "SendingMoteC.nc"
enum SendingMoteC____nesc_unnamed4361 {
#line 293
  SendingMoteC__sendMultipleEcho = 12U
};
#line 293
typedef int SendingMoteC____nesc_sillytask_sendMultipleEcho[SendingMoteC__sendMultipleEcho];
enum SendingMoteC____nesc_unnamed4362 {
#line 294
  SendingMoteC__sendReport = 13U
};
#line 294
typedef int SendingMoteC____nesc_sillytask_sendReport[SendingMoteC__sendReport];






enum SendingMoteC____nesc_unnamed4363 {
#line 301
  SendingMoteC__sendIdentification = 14U
};
#line 301
typedef int SendingMoteC____nesc_sillytask_sendIdentification[SendingMoteC__sendIdentification];





enum SendingMoteC____nesc_unnamed4364 {
#line 307
  SendingMoteC__sendReading = 15U
};
#line 307
typedef int SendingMoteC____nesc_sillytask_sendReading[SendingMoteC__sendReading];
#line 716
enum SendingMoteC____nesc_unnamed4365 {
#line 716
  SendingMoteC__sendCommandACK = 16U
};
#line 716
typedef int SendingMoteC____nesc_sillytask_sendCommandACK[SendingMoteC__sendCommandACK];
#line 1377
enum SendingMoteC____nesc_unnamed4366 {
#line 1377
  SendingMoteC__sendEchoReply = 17U
};
#line 1377
typedef int SendingMoteC____nesc_sillytask_sendEchoReply[SendingMoteC__sendEchoReply];
#line 141
enum SendingMoteC____nesc_unnamed4367 {


  SendingMoteC__RADIO_QUEUE_LEN = 100, 



  SendingMoteC__QUEUE_TIMEOUT = 1000, 


  SendingMoteC__REPORT_SEND_GAP = 15, 


  SendingMoteC__REPORT_SEND_THRESH = 15, 


  SendingMoteC__NOISEFLOOR_READINGS = 10, 


  SendingMoteC__NOISEFLOOR_DELAY = 100
};


MultiPingResponseReportStruct SendingMoteC__radioBuff[100];
uint8_t SendingMoteC__radioIn;
#line 165
uint8_t SendingMoteC__radioOut;
bool SendingMoteC__radioBusy;
#line 166
bool SendingMoteC__radioFull;
uint8_t SendingMoteC__queueFlushThreshold = 4;


bool SendingMoteC__queueTimeout = FALSE;


message_t SendingMoteC__pkt;



message_t SendingMoteC__pkt_priv;


message_t SendingMoteC__pkt_response;



uint16_t SendingMoteC__command_dest = 1;




uint16_t SendingMoteC__sensorReadingDest = 1;



bool SendingMoteC__busy = FALSE;


uint16_t SendingMoteC__counter = 0;
uint16_t SendingMoteC__counter_ping = 0;


uint8_t SendingMoteC__cur_tx_power;
uint8_t SendingMoteC__tx_power = 31;
uint8_t SendingMoteC__base_tx_power = 31;


uint8_t SendingMoteC__cur_channel = 0;
uint8_t SendingMoteC__channel = 0;


uint8_t SendingMoteC__blinkCnSend = 0;
uint8_t SendingMoteC__blinkCnRecv = 0;
uint8_t SendingMoteC__blinkCnFail = 0;


uint16_t SendingMoteC__delay = 100;
uint16_t SendingMoteC__packets2send = 0;


uint16_t SendingMoteC__baseid = 1;




bool SendingMoteC__doReporting = TRUE;



bool SendingMoteC__doRandomizedThresholding = TRUE;


uint16_t SendingMoteC__signalCounter = 0;



uint8_t SendingMoteC__operationMode = NODE_REPORTING;


bool SendingMoteC__doSensorReadingSampling = FALSE;





uint8_t SendingMoteC__reportProtocol = REPORTING_MEDIUM;




bool SendingMoteC__reportMassTrigered = FALSE;





uint16_t SendingMoteC__massReportQueueThreshold = 0;


bool SendingMoteC__doNoiseFloorReading = TRUE;



uint8_t SendingMoteC__noiseFloorMeasured = 0;


uint8_t SendingMoteC__reportingGap = SendingMoteC__REPORT_SEND_GAP;




bool SendingMoteC__tabulaRasa = TRUE;


uint8_t SendingMoteC__bootupCounter = 0;





uint8_t SendingMoteC__readMode = 1;

uint16_t SendingMoteC__temperature;
uint16_t SendingMoteC__humidity;
uint16_t SendingMoteC__light;

uint16_t SendingMoteC__tmpReading = 0;
uint16_t SendingMoteC__msgReadingLong = 0;
uint16_t SendingMoteC__numConditionValide = 0;




static inline uint16_t SendingMoteC__getRssi(message_t *msg);
static message_t *SendingMoteC__receive(message_t *msg, void *payload, uint8_t len, am_id_t id);


static inline void SendingMoteC__setAutoAck(bool enableAutoAck, bool hwAutoAck);

static void SendingMoteC__reset_node(void );
static void SendingMoteC__abort(void );
static inline void SendingMoteC__sendQueueFlushCommand(void );




static void SendingMoteC__setPower(message_t *msg, uint8_t power);

static inline error_t SendingMoteC__readNoiseFloor(void );

static void SendingMoteC__readSensors(void );
static inline bool SendingMoteC__setGIO(uint8_t pin, bool enabled);
static bool SendingMoteC__message2sampleReceived(uint16_t rssi, uint16_t cn, uint16_t source);










static uint16_t SendingMoteC__queueSize(void );






static void SendingMoteC__rcvBlink(void );








static void SendingMoteC__sendBlink(void );









static void SendingMoteC__failBlink(void );
#line 361
static void SendingMoteC__signalize(uint8_t num);
#line 375
static inline void SendingMoteC__Boot__booted(void );
#line 403
static inline void SendingMoteC__RadioControl__startDone(error_t result);
#line 417
static inline void SendingMoteC__RadioControl__stopDone(error_t result);





static inline void SendingMoteC__BootupTimer__fired(void );








static inline void SendingMoteC__FlushTimer__fired(void );
#line 450
static inline void SendingMoteC__SignalTimer__fired(void );
#line 468
static inline void SendingMoteC__SendTimer__fired(void );
#line 496
static inline void SendingMoteC__GapTimer__fired(void );







static inline void SendingMoteC__NoiseFloorTimer__fired(void );






static inline void SendingMoteC__SensorReadingTimer__fired(void );








static inline void SendingMoteC__sendMultipleEcho__runTask(void );
#line 546
static inline bool SendingMoteC__doQueueSend(void );
#line 610
static inline void SendingMoteC__sendReport__runTask(void );
#line 716
static inline void SendingMoteC__sendCommandACK__runTask(void );
#line 753
static inline void SendingMoteC__sendIdentification__runTask(void );
#line 814
static inline void SendingMoteC__clearQueue(void );
#line 829
static void SendingMoteC__abort(void );
#line 859
static inline void SendingMoteC__reset_hard(void );









static void SendingMoteC__reset_node(void );
#line 954
static inline void SendingMoteC__ReportMsgSend__sendDone(message_t *m, error_t error);
#line 1021
static inline void SendingMoteC__TinyReportMsgSend__sendDone(message_t *m, error_t error);
#line 1045
static inline void SendingMoteC__RssiMsgSend__sendDone(message_t *m, error_t error);










static inline void SendingMoteC__MultiPingResponseSend__sendDone(message_t *m, error_t error);
#line 1074
static inline void SendingMoteC__CommandMsgSend__sendDone(message_t *m, error_t error);
#line 1095
static inline message_t *SendingMoteC__CommandReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 1368
static inline message_t *SendingMoteC__PingReceive__receive(message_t *msg, void *payload, uint8_t len);








static inline void SendingMoteC__sendEchoReply__runTask(void );
#line 1404
static inline message_t *SendingMoteC__SimplePingReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 1448
static inline message_t *SendingMoteC__ResponseReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 1462
static message_t *SendingMoteC__receive(message_t *msg, void *payload, uint8_t len, am_id_t id);
#line 1539
static bool SendingMoteC__message2sampleReceived(uint16_t rssi, uint16_t cn, uint16_t source);
#line 1617
static inline void SendingMoteC__sendQueueFlushCommand(void );
#line 1641
static void SendingMoteC__readSensors(void );
#line 1656
static void SendingMoteC__Temperature__readDone(error_t ok, uint16_t val);
#line 1678
static void SendingMoteC__Light__readDone(error_t ok, uint16_t val);
#line 1691
static void SendingMoteC__Humidity__readDone(error_t ok, uint16_t val);
#line 1711
static inline void SendingMoteC__sendReading__runTask(void );
#line 1749
static inline bool SendingMoteC__setGIO(uint8_t pin, bool enabled);
#line 1781
static inline uint16_t SendingMoteC__getRssi(message_t *msg);
#line 1810
static void SendingMoteC__setPower(message_t *msg, uint8_t power);








static inline void SendingMoteC__CC2420Config__syncDone(error_t error);
#line 1841
static inline void SendingMoteC__setAutoAck(bool enableAutoAck, bool hwAutoAck);
#line 1854
static inline error_t SendingMoteC__readNoiseFloor(void );










static inline void SendingMoteC__ReadRssi__readDone(error_t error, uint16_t data);
# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static void /*SendingMoteAppC.RssiMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static error_t /*SendingMoteAppC.RssiMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(
#line 56
message_t * msg, 







uint8_t len);
# 92 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
static void /*SendingMoteAppC.RssiMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 151
static void /*SendingMoteAppC.RssiMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
# 45 "/opt/tinyos-2.x/tos/system/AMQueueEntryP.nc"
static inline error_t /*SendingMoteAppC.RssiMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len);









static inline void /*SendingMoteAppC.RssiMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err);
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(
# 40 "/opt/tinyos-2.x/tos/system/AMQueueImplP.nc"
am_id_t arg_0xf68a3bc8, 
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(
# 38 "/opt/tinyos-2.x/tos/system/AMQueueImplP.nc"
uint8_t arg_0xf68a31e0, 
# 85 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 67 "/opt/tinyos-2.x/tos/interfaces/Packet.nc"
static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(
#line 63
message_t * msg);
#line 83
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(
#line 79
message_t * msg, 



uint8_t len);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask(void );
# 67 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
static am_addr_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(
#line 63
message_t * amsg);
#line 136
static am_id_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(
#line 132
message_t * amsg);
# 118 "/opt/tinyos-2.x/tos/system/AMQueueImplP.nc"
enum /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4368 {
#line 118
  AMQueueImplP__0__CancelTask = 18U
};
#line 118
typedef int /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_CancelTask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask];
#line 161
enum /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4369 {
#line 161
  AMQueueImplP__0__errorTask = 19U
};
#line 161
typedef int /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_errorTask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask];
#line 49
#line 47
typedef struct /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4370 {
  message_t * msg;
} /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t;

uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 5;
/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[5];
uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[5 / 8 + 1];

static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void );

static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket(void );
#line 82
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len);
#line 118
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void );
#line 155
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(uint8_t last, message_t * msg, error_t err);





static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void );




static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void );
#line 181
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(am_id_t id, message_t *msg, error_t err);
#line 207
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err);
# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static void /*SendingMoteAppC.MultiPingResponseSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static error_t /*SendingMoteAppC.MultiPingResponseSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__Send__send(
#line 56
message_t * msg, 







uint8_t len);
# 92 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
static void /*SendingMoteAppC.MultiPingResponseSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 151
static void /*SendingMoteAppC.MultiPingResponseSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
# 45 "/opt/tinyos-2.x/tos/system/AMQueueEntryP.nc"
static inline error_t /*SendingMoteAppC.MultiPingResponseSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len);









static inline void /*SendingMoteAppC.MultiPingResponseSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__Send__sendDone(message_t *m, error_t err);
# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static void /*SendingMoteAppC.ReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static error_t /*SendingMoteAppC.ReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__2__Send__send(
#line 56
message_t * msg, 







uint8_t len);
# 92 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
static void /*SendingMoteAppC.ReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 151
static void /*SendingMoteAppC.ReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
# 45 "/opt/tinyos-2.x/tos/system/AMQueueEntryP.nc"
static inline error_t /*SendingMoteAppC.ReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len);









static inline void /*SendingMoteAppC.ReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__2__Send__sendDone(message_t *m, error_t err);
# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static void /*SendingMoteAppC.TinyReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static error_t /*SendingMoteAppC.TinyReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__3__Send__send(
#line 56
message_t * msg, 







uint8_t len);
# 92 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
static void /*SendingMoteAppC.TinyReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 151
static void /*SendingMoteAppC.TinyReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
# 45 "/opt/tinyos-2.x/tos/system/AMQueueEntryP.nc"
static inline error_t /*SendingMoteAppC.TinyReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len);









static inline void /*SendingMoteAppC.TinyReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__3__Send__sendDone(message_t *m, error_t err);
# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static void /*SendingMoteAppC.CommandMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__4__AMSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static error_t /*SendingMoteAppC.CommandMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__4__Send__send(
#line 56
message_t * msg, 







uint8_t len);
# 92 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
static void /*SendingMoteAppC.CommandMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__4__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 151
static void /*SendingMoteAppC.CommandMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__4__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
# 45 "/opt/tinyos-2.x/tos/system/AMQueueEntryP.nc"
static error_t /*SendingMoteAppC.CommandMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__4__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len);









static inline void /*SendingMoteAppC.CommandMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__4__Send__sendDone(message_t *m, error_t err);
# 76 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static error_t /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureHumidity(void );
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__readDone(error_t result, /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__val_t val);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__release(void );
#line 78
static error_t /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__request(void );
# 61 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static error_t /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureTemperature(void );
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__HumResource__release(void );
#line 78
static error_t /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__HumResource__request(void );
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__readDone(error_t result, /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__val_t val);
# 60 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline error_t /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__read(void );




static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__granted(void );







static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureTemperatureDone(error_t result, uint16_t val);






static inline error_t /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__read(void );




static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__HumResource__granted(void );







static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureHumidityDone(error_t result, uint16_t val);




static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__resetDone(error_t result);
static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureHumidityDone(error_t result, uint16_t val);
static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__readStatusRegDone(error_t result, uint8_t val);
static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__writeStatusRegDone(error_t result);

static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__resetDone(error_t result);
static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureTemperatureDone(error_t result, uint16_t val);
static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__readStatusRegDone(error_t result, uint8_t val);
static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__writeStatusRegDone(error_t result);
# 50 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__disable(void );
#line 43
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__enableFallingEdge(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readSensor__postTask(void );
#line 56
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone__postTask(void );
# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__makeOutput(void );
#line 29
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__set(void );
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr(void );
# 84 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureHumidityDone(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0xf6823938, 
# 84 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t result, uint16_t val);
#line 116
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__writeStatusRegDone(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0xf6823938, 
# 116 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t result);
#line 100
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__readStatusRegDone(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0xf6823938, 
# 100 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t result, uint8_t val);
#line 54
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__resetDone(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0xf6823938, 
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t result);
#line 69
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureTemperatureDone(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0xf6823938, 
# 69 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t result, uint16_t val);
# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeInput(void );
#line 32
static bool /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__get(void );


static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeOutput(void );
#line 29
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set(void );
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__clr(void );
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__startOneShot(uint32_t dt);




static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__stop(void );
# 102 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
enum /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0____nesc_unnamed4371 {
#line 102
  SensirionSht11LogicP__0__readSensor = 20U
};
#line 102
typedef int /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0____nesc_sillytask_readSensor[/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readSensor];
enum /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0____nesc_unnamed4372 {
#line 103
  SensirionSht11LogicP__0__signalStatusDone = 21U
};
#line 103
typedef int /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0____nesc_sillytask_signalStatusDone[/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone];
#line 72
#line 66
typedef enum /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0____nesc_unnamed4373 {
  SensirionSht11LogicP__0__CMD_MEASURE_TEMPERATURE = 0x3, 
  SensirionSht11LogicP__0__CMD_MEASURE_HUMIDITY = 0x5, 
  SensirionSht11LogicP__0__CMD_READ_STATUS = 0x7, 
  SensirionSht11LogicP__0__CMD_WRITE_STATUS = 0x6, 
  SensirionSht11LogicP__0__CMD_SOFT_RESET = 0x1E
} /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__sht_cmd_t;

enum /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0____nesc_unnamed4374 {
  SensirionSht11LogicP__0__TIMEOUT_RESET = 11, 
  SensirionSht11LogicP__0__TIMEOUT_14BIT = 250, 
  SensirionSht11LogicP__0__TIMEOUT_12BIT = 250, 
  SensirionSht11LogicP__0__TIMEOUT_8BIT = 250
};

bool /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__on = TRUE;
bool /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = FALSE;
uint8_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__status = 0;
/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__sht_cmd_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__cmd;
uint8_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__newStatus;
bool /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__writeFail = FALSE;

uint8_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__currentClient;

static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__performCommand(void );
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__initPins(void );
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__resetDevice(void );
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__transmissionStart(void );
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__sendCommand(uint8_t _cmd);
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__writeByte(uint8_t byte);
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__waitForResponse(void );
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__enableInterrupt(void );
static uint8_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readByte(void );
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__ack(void );
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__endTransmission(void );
#line 113
static inline error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureTemperature(uint8_t client);







static inline error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureHumidity(uint8_t client);
#line 149
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__performCommand(void );
#line 220
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__initPins(void );







static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__resetDevice(void );










static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__transmissionStart(void );
#line 251
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__sendCommand(uint8_t _cmd);



static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__writeByte(uint8_t byte);
#line 268
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__waitForResponse(void );
#line 281
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__enableInterrupt(void );





static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__fired(void );
#line 315
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__fired(void );




static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readSensor__runTask(void );
#line 355
static uint8_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readByte(void );
#line 372
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__ack(void );








static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__endTransmission(void );






static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone__runTask(void );
#line 406
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__resetDone(uint8_t client, error_t result);
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__measureTemperatureDone(uint8_t client, error_t result, uint16_t val);
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__measureHumidityDone(uint8_t client, error_t result, uint16_t val);
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__readStatusRegDone(uint8_t client, error_t result, uint8_t val);
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__writeStatusRegDone(uint8_t client, error_t result);
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__makeInput(void );






static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__makeOutput(void );
#line 59
static bool /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__get(void );
#line 34
static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__set(void );




static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__clr(void );
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__set(void );
static inline void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__clr(void );

static inline bool /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__get(void );
static inline void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__makeInput(void );

static inline void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__makeOutput(void );
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__makeInput(void );






static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__makeOutput(void );
#line 34
static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__set(void );




static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__clr(void );
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__set(void );
static inline void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__clr(void );


static inline void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__makeInput(void );

static inline void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__makeOutput(void );
# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__HplGeneralIO__makeOutput(void );
#line 34
static void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__HplGeneralIO__set(void );




static void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__HplGeneralIO__clr(void );
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__set(void );
static inline void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__clr(void );




static inline void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__makeOutput(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static void HplSensirionSht11P__SplitControl__startDone(error_t error);
#line 117
static void HplSensirionSht11P__SplitControl__stopDone(error_t error);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t HplSensirionSht11P__stopTask__postTask(void );
# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void HplSensirionSht11P__SCK__makeInput(void );
#line 30
static void HplSensirionSht11P__SCK__clr(void );




static void HplSensirionSht11P__PWR__makeOutput(void );
#line 29
static void HplSensirionSht11P__PWR__set(void );
static void HplSensirionSht11P__PWR__clr(void );


static void HplSensirionSht11P__DATA__makeInput(void );
#line 30
static void HplSensirionSht11P__DATA__clr(void );
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void HplSensirionSht11P__Timer__startOneShot(uint32_t dt);
# 50 "/opt/tinyos-2.x/tos/platforms/telosa/chips/sht11/HplSensirionSht11P.nc"
enum HplSensirionSht11P____nesc_unnamed4375 {
#line 50
  HplSensirionSht11P__stopTask = 22U
};
#line 50
typedef int HplSensirionSht11P____nesc_sillytask_stopTask[HplSensirionSht11P__stopTask];

static error_t HplSensirionSht11P__SplitControl__start(void );






static inline void HplSensirionSht11P__Timer__fired(void );



static inline error_t HplSensirionSht11P__SplitControl__stop(void );









static inline void HplSensirionSht11P__stopTask__runTask(void );
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__clear(void );
#line 36
static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__disable(void );
#line 56
static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__edge(bool low_to_high);
#line 31
static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__enable(void );
# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__fired(void );
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__enable(bool rising);
#line 54
static inline error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__enableFallingEdge(void );



static error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__disable(void );







static inline void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__fired(void );
# 39 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
enum /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2____nesc_unnamed4376 {
#line 39
  FcfsResourceQueueC__2__NO_ENTRY = 0xFF
};
uint8_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__resQ[2U];
uint8_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qHead = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY;
uint8_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qTail = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY;

static inline error_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__Init__init(void );




static inline bool /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEmpty(void );



static inline bool /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__dequeue(void );
#line 72
static inline error_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(resource_client_id_t id);
# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceRequested__requested(
# 55 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4c770);
# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(
# 60 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4bb38);
# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__configure(
# 60 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4bb38);
# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Queue__enqueue(resource_client_id_t id);
#line 43
static bool /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Queue__isEmpty(void );
#line 60
static resource_client_id_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Queue__dequeue(void );
# 73 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested(void );
#line 46
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__granted(
# 54 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0xf6d4ece8);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask__postTask(void );
# 75 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
enum /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1____nesc_unnamed4377 {
#line 75
  ArbiterP__1__grantedTask = 23U
};
#line 75
typedef int /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1____nesc_sillytask_grantedTask[/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask];
#line 67
enum /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1____nesc_unnamed4378 {
#line 67
  ArbiterP__1__RES_CONTROLLED, ArbiterP__1__RES_GRANTING, ArbiterP__1__RES_IMM_GRANTING, ArbiterP__1__RES_BUSY
};
#line 68
enum /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1____nesc_unnamed4379 {
#line 68
  ArbiterP__1__default_owner_id = 2U
};
#line 69
enum /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1____nesc_unnamed4380 {
#line 69
  ArbiterP__1__NO_RES = 0xFF
};
uint8_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_CONTROLLED;
uint8_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__default_owner_id;
uint8_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__reqResId;



static error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__request(uint8_t id);
#line 108
static error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__release(uint8_t id);
#line 130
static inline error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void );
#line 181
static inline uint8_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__isOwner(void );





static inline void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask__runTask(void );
#line 199
static inline void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__default__granted(uint8_t id);

static inline void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(uint8_t id);
#line 213
static inline void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(uint8_t id);

static inline void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(uint8_t id);
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__start(void );
#line 109
static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__stop(void );
# 52 "/opt/tinyos-2.x/tos/lib/power/PowerDownCleanup.nc"
static void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__PowerDownCleanup__cleanup(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopTask__postTask(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__release(void );








static bool /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__isOwner(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__startTask__postTask(void );
# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__start(void );









static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__stop(void );
# 63 "/opt/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
enum /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0____nesc_unnamed4381 {
#line 63
  PowerManagerP__0__startTask = 24U
};
#line 63
typedef int /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0____nesc_sillytask_startTask[/*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__startTask];




enum /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0____nesc_unnamed4382 {
#line 68
  PowerManagerP__0__stopTask = 25U
};
#line 68
typedef int /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0____nesc_sillytask_stopTask[/*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopTask];
#line 60
bool /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopping = FALSE;
bool /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__requested = FALSE;

static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__startTask__runTask(void );




static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopTask__runTask(void );





static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__requested(void );









static inline error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__default__start(void );







static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__startDone(error_t error);




static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__granted(void );




static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__stopDone(error_t error);










static inline error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__default__stop(void );







static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__PowerDownCleanup__default__cleanup(void );
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static void AdcP__Read__readDone(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0xf676c668, 
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
error_t result, AdcP__Read__val_t val);
# 66 "/opt/tinyos-2.x/tos/interfaces/ReadNow.nc"
static void AdcP__ReadNow__readDone(
# 39 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0xf6768b08, 
# 66 "/opt/tinyos-2.x/tos/interfaces/ReadNow.nc"
error_t result, AdcP__ReadNow__val_t val);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static AdcP__Config__adc_config_t AdcP__Config__getConfiguration(
# 48 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0xf6766df0);
# 189 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcP__SingleChannel__getData(
# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0xf6758e40);
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcP__SingleChannel__configureSingle(
# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0xf6758e40, 
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t AdcP__ResourceRead__release(
# 44 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0xf6769670);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t AdcP__ResourceRead__request(
# 44 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0xf6769670);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t AdcP__readDone__postTask(void );
# 136 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
enum AdcP____nesc_unnamed4383 {
#line 136
  AdcP__readDone = 26U
};
#line 136
typedef int AdcP____nesc_sillytask_readDone[AdcP__readDone];
#line 54
enum AdcP____nesc_unnamed4384 {
  AdcP__STATE_READ, 
  AdcP__STATE_READNOW, 
  AdcP__STATE_READNOW_INVALID_CONFIG
};


uint8_t AdcP__state;
uint8_t AdcP__owner;
uint16_t AdcP__value;

static inline error_t AdcP__configure(uint8_t client);









static inline error_t AdcP__Read__read(uint8_t client);




static void AdcP__ResourceRead__granted(uint8_t client);
#line 136
static inline void AdcP__readDone__runTask(void );





static error_t AdcP__SingleChannel__singleDataReady(uint8_t client, uint16_t data);
#line 161
static inline uint16_t *AdcP__SingleChannel__multipleDataReady(uint8_t client, 
uint16_t *buf, uint16_t numSamples);





static inline error_t AdcP__ResourceRead__default__request(uint8_t client);

static inline error_t AdcP__ResourceRead__default__release(uint8_t client);

static inline void AdcP__Read__default__readDone(uint8_t client, error_t result, uint16_t val);





static inline void AdcP__ReadNow__default__readDone(uint8_t client, error_t result, uint16_t val);

static inline error_t AdcP__SingleChannel__default__getData(uint8_t client);




const msp430adc12_channel_config_t AdcP__defaultConfig = { INPUT_CHANNEL_NONE, 0, 0, 0, 0, 0, 0, 0 };
static inline const msp430adc12_channel_config_t *
AdcP__Config__default__getConfiguration(uint8_t client);



static inline error_t AdcP__SingleChannel__default__configureSingle(uint8_t client, 
const msp430adc12_channel_config_t *config);
# 107 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
static void Msp430Adc12ImplP__MultiChannel__dataReady(
# 42 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0xf671d688, 
# 107 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
uint16_t *buffer, uint16_t numSamples);
# 63 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
static adc12ctl0_t Msp430Adc12ImplP__HplAdc12__getCtl0(void );
#line 82
static adc12memctl_t Msp430Adc12ImplP__HplAdc12__getMCtl(uint8_t idx);
#line 106
static void Msp430Adc12ImplP__HplAdc12__resetIFGs(void );
#line 75
static void Msp430Adc12ImplP__HplAdc12__setMCtl(uint8_t idx, adc12memctl_t memControl);
#line 128
static void Msp430Adc12ImplP__HplAdc12__startConversion(void );
#line 51
static void Msp430Adc12ImplP__HplAdc12__setCtl0(adc12ctl0_t control0);
#line 89
static uint16_t Msp430Adc12ImplP__HplAdc12__getMem(uint8_t idx);





static void Msp430Adc12ImplP__HplAdc12__setIEFlags(uint16_t mask);
#line 123
static void Msp430Adc12ImplP__HplAdc12__stopConversion(void );
#line 57
static void Msp430Adc12ImplP__HplAdc12__setCtl1(adc12ctl1_t control1);
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void Msp430Adc12ImplP__Port64__makeInput(void );
#line 85
static void Msp430Adc12ImplP__Port64__selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP__Port64__selectModuleFunc(void );
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void Msp430Adc12ImplP__CompareA1__setEvent(uint16_t time);
# 35 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void Msp430Adc12ImplP__ControlA0__setControl(msp430_compare_control_t control);
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void Msp430Adc12ImplP__Port62__makeInput(void );
#line 85
static void Msp430Adc12ImplP__Port62__selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP__Port62__selectModuleFunc(void );
# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static void Msp430Adc12ImplP__Overflow__memOverflow(
# 43 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0xf671c010);
# 54 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static void Msp430Adc12ImplP__Overflow__conversionTimeOverflow(
# 43 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0xf671c010);
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void Msp430Adc12ImplP__Port67__makeInput(void );
#line 85
static void Msp430Adc12ImplP__Port67__selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP__Port67__selectModuleFunc(void );
#line 64
static void Msp430Adc12ImplP__Port60__makeInput(void );
#line 85
static void Msp430Adc12ImplP__Port60__selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP__Port60__selectModuleFunc(void );
#line 64
static void Msp430Adc12ImplP__Port65__makeInput(void );
#line 85
static void Msp430Adc12ImplP__Port65__selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP__Port65__selectModuleFunc(void );
# 41 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void Msp430Adc12ImplP__TimerA__clear(void );


static void Msp430Adc12ImplP__TimerA__setClockSource(uint16_t clockSource);
#line 43
static void Msp430Adc12ImplP__TimerA__disableEvents(void );
#line 39
static void Msp430Adc12ImplP__TimerA__setMode(int mode);





static void Msp430Adc12ImplP__TimerA__setInputDivider(uint16_t inputDivider);
# 88 "/opt/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
static uint8_t Msp430Adc12ImplP__ADCArbiterInfo__userId(void );
# 35 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void Msp430Adc12ImplP__ControlA1__setControl(msp430_compare_control_t control);
# 227 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static uint16_t * Msp430Adc12ImplP__SingleChannel__multipleDataReady(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0xf671e908, 
# 227 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t * buffer, uint16_t numSamples);
#line 206
static error_t Msp430Adc12ImplP__SingleChannel__singleDataReady(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0xf671e908, 
# 206 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t data);
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void Msp430Adc12ImplP__Port63__makeInput(void );
#line 85
static void Msp430Adc12ImplP__Port63__selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP__Port63__selectModuleFunc(void );
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void Msp430Adc12ImplP__CompareA0__setEvent(uint16_t time);
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void Msp430Adc12ImplP__Port61__makeInput(void );
#line 85
static void Msp430Adc12ImplP__Port61__selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP__Port61__selectModuleFunc(void );
#line 64
static void Msp430Adc12ImplP__Port66__makeInput(void );
#line 85
static void Msp430Adc12ImplP__Port66__selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP__Port66__selectModuleFunc(void );
# 71 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
enum Msp430Adc12ImplP____nesc_unnamed4385 {
  Msp430Adc12ImplP__SINGLE_DATA = 1, 
  Msp430Adc12ImplP__SINGLE_DATA_REPEAT = 2, 
  Msp430Adc12ImplP__MULTIPLE_DATA = 4, 
  Msp430Adc12ImplP__MULTIPLE_DATA_REPEAT = 8, 
  Msp430Adc12ImplP__MULTI_CHANNEL = 16, 
  Msp430Adc12ImplP__CONVERSION_MODE_MASK = 0x1F, 

  Msp430Adc12ImplP__ADC_BUSY = 32, 
  Msp430Adc12ImplP__USE_TIMERA = 64, 
  Msp430Adc12ImplP__ADC_OVERFLOW = 128
};

uint8_t Msp430Adc12ImplP__state;

uint16_t Msp430Adc12ImplP__resultBufferLength;
uint16_t * Msp430Adc12ImplP__resultBufferStart;
uint16_t Msp430Adc12ImplP__resultBufferIndex;
uint8_t Msp430Adc12ImplP__numChannels;
uint8_t Msp430Adc12ImplP__clientID;

static inline error_t Msp430Adc12ImplP__Init__init(void );










static inline void Msp430Adc12ImplP__prepareTimerA(uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON);
#line 121
static inline void Msp430Adc12ImplP__startTimerA(void );
#line 142
static inline void Msp430Adc12ImplP__configureAdcPin(uint8_t inch);
#line 159
static void Msp430Adc12ImplP__resetAdcPin(uint8_t inch);
#line 176
static error_t Msp430Adc12ImplP__SingleChannel__configureSingle(uint8_t id, 
const msp430adc12_channel_config_t *config);
#line 271
static inline error_t Msp430Adc12ImplP__SingleChannel__configureMultiple(uint8_t id, 
const msp430adc12_channel_config_t *config, 
uint16_t *buf, uint16_t length, uint16_t jiffies);
#line 394
static error_t Msp430Adc12ImplP__SingleChannel__getData(uint8_t id);
#line 503
static void Msp430Adc12ImplP__stopConversion(void );
#line 540
static inline void Msp430Adc12ImplP__TimerA__overflow(void );
static inline void Msp430Adc12ImplP__CompareA0__fired(void );
static inline void Msp430Adc12ImplP__CompareA1__fired(void );

static inline void Msp430Adc12ImplP__HplAdc12__conversionDone(uint16_t iv);
#line 640
static inline error_t Msp430Adc12ImplP__SingleChannel__default__singleDataReady(uint8_t id, uint16_t data);




static inline uint16_t *Msp430Adc12ImplP__SingleChannel__default__multipleDataReady(uint8_t id, 
uint16_t *buf, uint16_t numSamples);




static inline void Msp430Adc12ImplP__MultiChannel__default__dataReady(uint8_t id, uint16_t *buffer, uint16_t numSamples);

static inline void Msp430Adc12ImplP__Overflow__default__memOverflow(uint8_t id);
static inline void Msp430Adc12ImplP__Overflow__default__conversionTimeOverflow(uint8_t id);
# 112 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
static void HplAdc12P__HplAdc12__conversionDone(uint16_t iv);
# 51 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static volatile uint16_t HplAdc12P__ADC12CTL0 __asm ("0x01A0");
static volatile uint16_t HplAdc12P__ADC12CTL1 __asm ("0x01A2");
static volatile uint16_t HplAdc12P__ADC12IFG __asm ("0x01A4");
static volatile uint16_t HplAdc12P__ADC12IE __asm ("0x01A6");
static volatile uint16_t HplAdc12P__ADC12IV __asm ("0x01A8");

static inline adc12ctl0_t HplAdc12P__int2adc12ctl0(uint16_t x)  ;

static inline uint16_t HplAdc12P__adc12ctl0cast2int(adc12ctl0_t x)  ;
static inline uint16_t HplAdc12P__adc12ctl1cast2int(adc12ctl1_t x)  ;
static inline uint8_t HplAdc12P__adc12memctl2int(adc12memctl_t x)  ;
static inline adc12memctl_t HplAdc12P__int2adc12memctl(uint8_t x)  ;

static inline void HplAdc12P__HplAdc12__setCtl0(adc12ctl0_t control0);



static inline void HplAdc12P__HplAdc12__setCtl1(adc12ctl1_t control1);



static inline adc12ctl0_t HplAdc12P__HplAdc12__getCtl0(void );







static inline void HplAdc12P__HplAdc12__setMCtl(uint8_t i, adc12memctl_t memCtl);



static inline adc12memctl_t HplAdc12P__HplAdc12__getMCtl(uint8_t i);



static inline uint16_t HplAdc12P__HplAdc12__getMem(uint8_t i);



static inline void HplAdc12P__HplAdc12__setIEFlags(uint16_t mask);


static inline void HplAdc12P__HplAdc12__resetIFGs(void );




static inline void HplAdc12P__HplAdc12__startConversion(void );




static void HplAdc12P__HplAdc12__stopConversion(void );
#line 118
static inline bool HplAdc12P__HplAdc12__isBusy(void );

void sig_ADC_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(14)))  ;
# 39 "/opt/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc"
enum /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0____nesc_unnamed4386 {
  RoundRobinResourceQueueC__0__NO_ENTRY = 0xFF, 
  RoundRobinResourceQueueC__0__SIZE = 2U ? (2U - 1) / 8 + 1 : 0
};

uint8_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ[/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__SIZE];
uint8_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__last = 0;

static inline void /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__clearEntry(uint8_t id);



static inline error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__Init__init(void );




static inline bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEmpty(void );








static bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__dequeue(void );
#line 87
static inline error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__enqueue(resource_client_id_t id);
# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceRequested__requested(
# 52 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf6667be0);
# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__unconfigure(
# 56 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf6666898);
# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__configure(
# 56 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf6666898);
# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__enqueue(resource_client_id_t id);
#line 43
static bool /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__isEmpty(void );
#line 60
static resource_client_id_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__dequeue(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__granted(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf66671b8);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__postTask(void );
# 69 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
enum /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0____nesc_unnamed4387 {
#line 69
  SimpleArbiterP__0__grantedTask = 27U
};
#line 69
typedef int /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0____nesc_sillytask_grantedTask[/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask];
#line 62
enum /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0____nesc_unnamed4388 {
#line 62
  SimpleArbiterP__0__RES_IDLE = 0, SimpleArbiterP__0__RES_GRANTING = 1, SimpleArbiterP__0__RES_BUSY = 2
};
#line 63
enum /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0____nesc_unnamed4389 {
#line 63
  SimpleArbiterP__0__NO_RES = 0xFF
};
uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_IDLE;
uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__NO_RES;
uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__reqResId;



static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__request(uint8_t id);
#line 97
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__release(uint8_t id);
#line 137
static uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ArbiterInfo__userId(void );
#line 155
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__runTask(void );









static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__default__granted(uint8_t id);

static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(uint8_t id);



static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id);
# 63 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
static adc12ctl0_t Msp430RefVoltGeneratorP__HplAdc12__getCtl0(void );
#line 118
static bool Msp430RefVoltGeneratorP__HplAdc12__isBusy(void );
#line 51
static void Msp430RefVoltGeneratorP__HplAdc12__setCtl0(adc12ctl0_t control0);
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void Msp430RefVoltGeneratorP__SwitchOffTimer__startOneShot(uint32_t dt);




static void Msp430RefVoltGeneratorP__SwitchOffTimer__stop(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static void Msp430RefVoltGeneratorP__RefVolt_2_5V__startDone(error_t error);
#line 117
static void Msp430RefVoltGeneratorP__RefVolt_2_5V__stopDone(error_t error);
#line 92
static void Msp430RefVoltGeneratorP__RefVolt_1_5V__startDone(error_t error);
#line 117
static void Msp430RefVoltGeneratorP__RefVolt_1_5V__stopDone(error_t error);
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void Msp430RefVoltGeneratorP__SwitchOnTimer__startOneShot(uint32_t dt);




static void Msp430RefVoltGeneratorP__SwitchOnTimer__stop(void );
# 65 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
#line 52
typedef enum Msp430RefVoltGeneratorP____nesc_unnamed4390 {

  Msp430RefVoltGeneratorP__GENERATOR_OFF = 0, 

  Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE = 1, 
  Msp430RefVoltGeneratorP__REFERENCE_2_5V_STABLE = 2, 

  Msp430RefVoltGeneratorP__REFERENCE_1_5V_ON_PENDING = 3, 
  Msp430RefVoltGeneratorP__REFERENCE_2_5V_ON_PENDING = 4, 

  Msp430RefVoltGeneratorP__REFERENCE_1_5V_OFF_PENDING = 5, 
  Msp430RefVoltGeneratorP__REFERENCE_2_5V_OFF_PENDING = 6
} 
Msp430RefVoltGeneratorP__state_t;

Msp430RefVoltGeneratorP__state_t Msp430RefVoltGeneratorP__m_state;


static error_t Msp430RefVoltGeneratorP__switchOn(uint8_t level);
static error_t Msp430RefVoltGeneratorP__switchOff(void );
static void Msp430RefVoltGeneratorP__signalStartDone(Msp430RefVoltGeneratorP__state_t state, error_t result);
static void Msp430RefVoltGeneratorP__signalStopDone(Msp430RefVoltGeneratorP__state_t state, error_t result);
static error_t Msp430RefVoltGeneratorP__start(Msp430RefVoltGeneratorP__state_t targetState);
static inline error_t Msp430RefVoltGeneratorP__stop(Msp430RefVoltGeneratorP__state_t nextState);


static inline error_t Msp430RefVoltGeneratorP__RefVolt_1_5V__start(void );



static inline error_t Msp430RefVoltGeneratorP__RefVolt_2_5V__start(void );



static inline error_t Msp430RefVoltGeneratorP__RefVolt_1_5V__stop(void );







static error_t Msp430RefVoltGeneratorP__start(Msp430RefVoltGeneratorP__state_t targetState);
#line 126
static inline error_t Msp430RefVoltGeneratorP__stop(Msp430RefVoltGeneratorP__state_t nextState);
#line 153
static void Msp430RefVoltGeneratorP__signalStartDone(Msp430RefVoltGeneratorP__state_t state, error_t result);






static void Msp430RefVoltGeneratorP__signalStopDone(Msp430RefVoltGeneratorP__state_t state, error_t result);







static inline void Msp430RefVoltGeneratorP__SwitchOnTimer__fired(void );
#line 185
static inline void Msp430RefVoltGeneratorP__SwitchOffTimer__fired(void );
#line 213
static inline void Msp430RefVoltGeneratorP__HplAdc12__conversionDone(uint16_t iv);



static error_t Msp430RefVoltGeneratorP__switchOn(uint8_t level);
#line 236
static error_t Msp430RefVoltGeneratorP__switchOff(void );
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static Msp430RefVoltArbiterImplP__Config__adc_config_t Msp430RefVoltArbiterImplP__Config__getConfiguration(
# 43 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0xf6606df0);
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static error_t Msp430RefVoltArbiterImplP__RefVolt_2_5V__start(void );
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP__AdcResource__release(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0xf66073a0);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP__AdcResource__request(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0xf66073a0);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void Msp430RefVoltArbiterImplP__ClientResource__granted(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0xf66388c0);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t Msp430RefVoltArbiterImplP__switchOff__postTask(void );
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static error_t Msp430RefVoltArbiterImplP__RefVolt_1_5V__start(void );
#line 109
static error_t Msp430RefVoltArbiterImplP__RefVolt_1_5V__stop(void );
# 51 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
enum Msp430RefVoltArbiterImplP____nesc_unnamed4391 {
#line 51
  Msp430RefVoltArbiterImplP__switchOff = 28U
};
#line 51
typedef int Msp430RefVoltArbiterImplP____nesc_sillytask_switchOff[Msp430RefVoltArbiterImplP__switchOff];
#line 46
enum Msp430RefVoltArbiterImplP____nesc_unnamed4392 {
  Msp430RefVoltArbiterImplP__NO_OWNER = 0xFF
};
uint8_t Msp430RefVoltArbiterImplP__syncOwner = Msp430RefVoltArbiterImplP__NO_OWNER;



static inline error_t Msp430RefVoltArbiterImplP__ClientResource__request(uint8_t client);
#line 70
static void Msp430RefVoltArbiterImplP__AdcResource__granted(uint8_t client);
#line 98
static inline void Msp430RefVoltArbiterImplP__RefVolt_1_5V__startDone(error_t error);








static void Msp430RefVoltArbiterImplP__RefVolt_2_5V__startDone(error_t error);








static error_t Msp430RefVoltArbiterImplP__ClientResource__release(uint8_t client);
#line 136
static inline void Msp430RefVoltArbiterImplP__switchOff__runTask(void );










static inline void Msp430RefVoltArbiterImplP__RefVolt_1_5V__stopDone(error_t error);



static inline void Msp430RefVoltArbiterImplP__RefVolt_2_5V__stopDone(error_t error);








static inline void Msp430RefVoltArbiterImplP__ClientResource__default__granted(uint8_t client);
static inline error_t Msp430RefVoltArbiterImplP__AdcResource__default__request(uint8_t client);








static inline error_t Msp430RefVoltArbiterImplP__AdcResource__default__release(uint8_t client);
const msp430adc12_channel_config_t Msp430RefVoltArbiterImplP__defaultConfig = { INPUT_CHANNEL_NONE, 0, 0, 0, 0, 0, 0, 0 };
static inline const msp430adc12_channel_config_t *
Msp430RefVoltArbiterImplP__Config__default__getConfiguration(uint8_t client);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfUp__adc_config_t /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfUp__getConfiguration(void );
# 47 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline const msp430adc12_channel_config_t */*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfSub__getConfiguration(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t AdcStreamP__bufferDone__postTask(void );
#line 56
static error_t AdcStreamP__readStreamDone__postTask(void );
#line 56
static error_t AdcStreamP__readStreamFail__postTask(void );
# 98 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static AdcStreamP__Alarm__size_type AdcStreamP__Alarm__getNow(void );
#line 92
static void AdcStreamP__Alarm__startAt(AdcStreamP__Alarm__size_type t0, AdcStreamP__Alarm__size_type dt);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static AdcStreamP__AdcConfigure__adc_config_t AdcStreamP__AdcConfigure__getConfiguration(
# 53 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0xf65bf010);
# 189 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcStreamP__SingleChannel__getData(
# 52 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0xf65c1170);
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcStreamP__SingleChannel__configureSingle(
# 52 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0xf65c1170, 
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config);
#line 138
static error_t AdcStreamP__SingleChannel__configureMultiple(
# 52 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0xf65c1170, 
# 138 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config, uint16_t * buffer, uint16_t numSamples, uint16_t jiffies);
# 89 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
static void AdcStreamP__ReadStream__bufferDone(
# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0xf65c3690, 
# 89 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
error_t result, 
#line 86
AdcStreamP__ReadStream__val_t * buf, 



uint16_t count);
#line 102
static void AdcStreamP__ReadStream__readDone(
# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0xf65c3690, 
# 102 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
error_t result, uint32_t usActualPeriod);
# 119 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
enum AdcStreamP____nesc_unnamed4393 {
#line 119
  AdcStreamP__readStreamDone = 29U
};
#line 119
typedef int AdcStreamP____nesc_sillytask_readStreamDone[AdcStreamP__readStreamDone];
#line 135
enum AdcStreamP____nesc_unnamed4394 {
#line 135
  AdcStreamP__readStreamFail = 30U
};
#line 135
typedef int AdcStreamP____nesc_sillytask_readStreamFail[AdcStreamP__readStreamFail];
#line 156
enum AdcStreamP____nesc_unnamed4395 {
#line 156
  AdcStreamP__bufferDone = 31U
};
#line 156
typedef int AdcStreamP____nesc_sillytask_bufferDone[AdcStreamP__bufferDone];
#line 58
enum AdcStreamP____nesc_unnamed4396 {
  AdcStreamP__NSTREAM = 1U
};




uint8_t AdcStreamP__client = AdcStreamP__NSTREAM;


struct AdcStreamP__list_entry_t {
  uint16_t count;
  struct AdcStreamP__list_entry_t * next;
};
struct AdcStreamP__list_entry_t *AdcStreamP__bufferQueue[AdcStreamP__NSTREAM];
struct AdcStreamP__list_entry_t * *AdcStreamP__bufferQueueEnd[AdcStreamP__NSTREAM];
uint16_t * AdcStreamP__lastBuffer;
#line 74
uint16_t AdcStreamP__lastCount;

uint16_t AdcStreamP__count;
uint16_t * AdcStreamP__buffer;
uint16_t * AdcStreamP__pos;
uint32_t AdcStreamP__now;
#line 79
uint32_t AdcStreamP__period;
bool AdcStreamP__periodModified;


static inline error_t AdcStreamP__Init__init(void );








static inline void AdcStreamP__sampleSingle(void );



static inline error_t AdcStreamP__postBuffer(uint8_t c, uint16_t *buf, uint16_t n);
#line 119
static inline void AdcStreamP__readStreamDone__runTask(void );
#line 135
static inline void AdcStreamP__readStreamFail__runTask(void );
#line 156
static inline void AdcStreamP__bufferDone__runTask(void );
#line 168
static inline void AdcStreamP__nextAlarm(void );




static inline void AdcStreamP__Alarm__fired(void );



static error_t AdcStreamP__nextBuffer(bool startNextAlarm);
#line 206
static void AdcStreamP__nextMultiple(uint8_t c);
#line 221
static error_t AdcStreamP__ReadStream__read(uint8_t c, uint32_t usPeriod);
#line 242
static error_t AdcStreamP__SingleChannel__singleDataReady(uint8_t streamClient, uint16_t data);
#line 281
static uint16_t *AdcStreamP__SingleChannel__multipleDataReady(uint8_t streamClient, 
uint16_t *buf, uint16_t length);
#line 304
const msp430adc12_channel_config_t AdcStreamP__defaultConfig = { 
.inch = SUPPLY_VOLTAGE_HALF_CHANNEL, 
.sref = REFERENCE_VREFplus_AVss, 
.ref2_5v = REFVOLT_LEVEL_1_5, 
.adc12ssel = SHT_SOURCE_ACLK, 
.adc12div = SHT_CLOCK_DIV_1, 
.sht = SAMPLE_HOLD_4_CYCLES, 
.sampcon_ssel = SAMPCON_SOURCE_SMCLK, 
.sampcon_id = SAMPCON_CLOCK_DIV_1 };

static inline const msp430adc12_channel_config_t *AdcStreamP__AdcConfigure__default__getConfiguration(uint8_t c);



static inline error_t AdcStreamP__SingleChannel__default__configureMultiple(uint8_t c, 
const msp430adc12_channel_config_t *config, uint16_t b[], 
uint16_t numSamples, uint16_t jiffies);



static inline error_t AdcStreamP__SingleChannel__default__getData(uint8_t c);



static inline error_t AdcStreamP__SingleChannel__default__configureSingle(uint8_t c, 
const msp430adc12_channel_config_t *config);
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEvent(uint16_t time);

static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEventFromNow(uint16_t delta);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__get(void );
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__fired(void );
# 46 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__enableEvents(void );
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__disableEvents(void );
#line 33
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__clearPendingInterrupt(void );
# 59 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired(void );










static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow(void );
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Alarm__fired(void );
#line 92
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__AlarmFrom__startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__AlarmFrom__size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__AlarmFrom__size_type dt);
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Counter__size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Counter__get(void );
# 66 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__m_t0;
/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__m_dt;

enum /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2____nesc_unnamed4397 {

  TransformAlarmC__2__MAX_DELAY_LOG2 = 8 * sizeof(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__from_size_type ) - 1 - 5, 
  TransformAlarmC__2__MAX_DELAY = (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__to_size_type )1 << /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__MAX_DELAY_LOG2
};

static inline /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Alarm__getNow(void );
#line 96
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__set_alarm(void );
#line 136
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Alarm__startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__to_size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__to_size_type dt);
#line 151
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__AlarmFrom__fired(void );
#line 166
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Counter__overflow(void );
# 78 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__read(
# 26 "/opt/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0xf657ed00, 
# 78 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
uint32_t usPeriod);










static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__bufferDone(
# 24 "/opt/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0xf657f258, 
# 89 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
error_t result, 
#line 86
/*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__val_t * buf, 



uint16_t count);
#line 102
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__readDone(
# 24 "/opt/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0xf657f258, 
# 102 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
error_t result, uint32_t usActualPeriod);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__release(
# 27 "/opt/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0xf657b678);



uint32_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__period[1U];
#line 48
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__bufferDone(uint8_t client, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t *buf, uint16_t count);




static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__readDone(uint8_t client, error_t result, uint32_t actualPeriod);





static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__granted(uint8_t client);







static inline error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__default__release(uint8_t client);
#line 79
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__bufferDone(uint8_t client, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t *buf, uint16_t count);



static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__readDone(uint8_t client, error_t result, uint32_t actualPeriod);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfUp__adc_config_t /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfUp__getConfiguration(void );
# 47 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline const msp430adc12_channel_config_t */*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfSub__getConfiguration(void );
# 48 "/opt/tinyos-2.x/tos/platforms/telosa/chips/s10871/HamamatsuS10871TsrP.nc"
msp430adc12_channel_config_t HamamatsuS10871TsrP__config = { 
.inch = INPUT_CHANNEL_A5, 
.sref = REFERENCE_VREFplus_AVss, 
.ref2_5v = REFVOLT_LEVEL_1_5, 
.adc12ssel = SHT_SOURCE_ACLK, 
.adc12div = SHT_CLOCK_DIV_1, 
.sht = SAMPLE_HOLD_4_CYCLES, 
.sampcon_ssel = SAMPCON_SOURCE_SMCLK, 
.sampcon_id = SAMPCON_CLOCK_DIV_1 };




static inline const msp430adc12_channel_config_t *HamamatsuS10871TsrP__AdcConfigure__getConfiguration(void );
# 212 "/opt/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_enable_interrupt(void )
{
   __asm volatile ("eint");}

# 185 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void )
{
}

# 540 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__TimerA__overflow(void )
#line 540
{
}

# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void ){
#line 37
  Msp430Adc12ImplP__TimerA__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow();
#line 37
}
#line 37
# 126 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n)
{
}

# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(uint8_t arg_0xf721cb28){
#line 28
  switch (arg_0xf721cb28) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired();
#line 28
      break;
#line 28
    case 1:
#line 28
      /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired();
#line 28
      break;
#line 28
    case 2:
#line 28
      /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired();
#line 28
      break;
#line 28
    case 5:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(arg_0xf721cb28);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 115 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(0);
}

# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA0__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired();
#line 28
}
#line 28
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4398 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(* (volatile uint16_t * )354U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void )
{
  return * (volatile uint16_t * )370U;
}

# 541 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__CompareA0__fired(void )
#line 541
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void ){
#line 34
  Msp430Adc12ImplP__CompareA0__fired();
#line 34
}
#line 34
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4399 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(* (volatile uint16_t * )356U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void )
{
  return * (volatile uint16_t * )372U;
}

# 542 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__CompareA1__fired(void )
#line 542
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void ){
#line 34
  Msp430Adc12ImplP__CompareA1__fired();
#line 34
}
#line 34
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2____nesc_unnamed4400 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(* (volatile uint16_t * )358U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void )
{
  return * (volatile uint16_t * )374U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired();
#line 34
}
#line 34
# 120 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )302U;

#line 123
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(n >> 1);
}

# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA1__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired();
#line 28
}
#line 28
# 115 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(0);
}

# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB0__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired();
#line 28
}
#line 28
# 185 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void )
{
}

# 103 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow(void )
{
}

#line 103
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void )
{
}

#line 103
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void )
{
}

# 166 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__overflow(void )
{
}

# 47 "/opt/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void )
{
}

# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static void /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__overflow(void ){
#line 71
  /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow();
#line 71
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__overflow();
#line 71
}
#line 71
# 122 "/opt/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static inline void /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*Counter32khz32C.Transform*/TransformCounterC__0__m_upper++;
    if ((/*Counter32khz32C.Transform*/TransformCounterC__0__m_upper & /*Counter32khz32C.Transform*/TransformCounterC__0__OVERFLOW_MASK) == 0) {
      /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__overflow();
      }
  }
}

# 47 "/opt/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void )
{
}

# 166 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__overflow(void )
{
}

#line 166
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Counter__overflow(void )
{
}

# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static void /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__overflow(void ){
#line 71
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Counter__overflow();
#line 71
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__overflow();
#line 71
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow();
#line 71
}
#line 71
# 122 "/opt/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static inline void /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC__1__m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC__1__m_upper & /*CounterMilli32C.Transform*/TransformCounterC__1__OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__overflow();
      }
  }
}

# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void ){
#line 71
  /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__overflow();
#line 71
  /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__overflow();
#line 71
}
#line 71
# 53 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow();
}

# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void ){
#line 37
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow();
#line 37
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow();
#line 37
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow();
#line 37
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow();
#line 37
}
#line 37
# 126 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow();
}

# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SpiResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ControlC.Spi*/CC2420SpiC__0__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 171 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Resource__request(void )
#line 171
{
  return CC2420ControlP__SpiResource__request();
}

# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420CsmaP__Resource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420ControlP__Resource__request();
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 207 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__CC2420Power__startVRegDone(void )
#line 207
{
  CC2420CsmaP__Resource__request();
}

# 56 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static void CC2420ControlP__CC2420Power__startVRegDone(void ){
#line 56
  CC2420CsmaP__CC2420Power__startVRegDone();
#line 56
}
#line 56
# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__4__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__4__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__4__HplGeneralIO__set();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__set(void ){
#line 29
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__4__GeneralIO__set();
#line 29
}
#line 29
# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__4__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__4__GeneralIO__clr(void )
#line 38
{
#line 38
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__4__HplGeneralIO__clr();
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__clr(void ){
#line 30
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__4__GeneralIO__clr();
#line 30
}
#line 30
# 408 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__StartupTimer__fired(void )
#line 408
{
  if (CC2420ControlP__m_state == CC2420ControlP__S_VREG_STARTING) {
      CC2420ControlP__m_state = CC2420ControlP__S_VREG_STARTED;
      CC2420ControlP__RSTN__clr();
      CC2420ControlP__RSTN__set();
      CC2420ControlP__CC2420Power__startVRegDone();
    }
}

# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 795 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__releaseSpiResource(void )
#line 795
{
  CC2420TransmitP__SpiResource__release();
  return SUCCESS;
}

# 50 "/opt/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge(void )
#line 50
{
  return /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(MSP430TIMER_CM_RISING);
}

# 42 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
inline static error_t CC2420TransmitP__CaptureSFD__captureRisingEdge(void ){
#line 42
  unsigned char __nesc_result;
#line 42

#line 42
  __nesc_result = /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge();
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420TransmitP__SFLUSHTX__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SFLUSHTX);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__get(void ){
#line 53
  unsigned long __nesc_result;
#line 53

#line 53
  __nesc_result = /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 75 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow(void )
{
  return /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__get();
}

#line 146
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type dt)
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow(), dt);
}

# 55 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__BackoffTimer__size_type dt){
#line 55
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(dt);
#line 55
}
#line 55
# 48 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )32U & (0x01 << 4);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw() != 0;
}

# 59 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__0__HplGeneralIO__get(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 40 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__0__GeneralIO__get(void )
#line 40
{
#line 40
  return /*HplCC2420PinsC.CCAM*/Msp430GpioC__0__HplGeneralIO__get();
}

# 32 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static bool CC2420TransmitP__CCA__get(void ){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = /*HplCC2420PinsC.CCAM*/Msp430GpioC__0__GeneralIO__get();
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 496 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__BackoffTimer__fired(void )
#line 496
{
  /* atomic removed: atomic calls only */
#line 497
  {
    switch (CC2420TransmitP__m_state) {

        case CC2420TransmitP__S_SAMPLE_CCA: 


          if (CC2420TransmitP__CCA__get()) {
              CC2420TransmitP__m_state = CC2420TransmitP__S_BEGIN_TRANSMIT;
              CC2420TransmitP__BackoffTimer__start(CC2420_TIME_ACK_TURNAROUND);
            }
          else {
              CC2420TransmitP__congestionBackoff();
            }
        break;

        case CC2420TransmitP__S_BEGIN_TRANSMIT: 
          case CC2420TransmitP__S_CANCEL: 
            if (CC2420TransmitP__acquireSpiResource() == SUCCESS) {
                CC2420TransmitP__attemptSend();
              }
        break;

        case CC2420TransmitP__S_ACK_WAIT: 
          CC2420TransmitP__signalDone(SUCCESS);
        break;

        case CC2420TransmitP__S_SFD: 


          CC2420TransmitP__SFLUSHTX__strobe();
        CC2420TransmitP__CaptureSFD__captureRisingEdge();
        CC2420TransmitP__releaseSpiResource();
        CC2420TransmitP__signalDone(ERETRY);
        break;

        default: 
          break;
      }
  }
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__fired(void ){
#line 67
  CC2420TransmitP__BackoffTimer__fired();
#line 67
  CC2420ControlP__StartupTimer__fired();
#line 67
}
#line 67
# 151 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt == 0) 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__fired();
      }
    else 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__set_alarm();
      }
  }
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void ){
#line 67
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__fired();
#line 67
}
#line 67
# 124 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void )
{
  * (volatile uint16_t * )386U &= ~0x0010;
}

# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents();
#line 47
}
#line 47
# 59 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired();
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void ){
#line 34
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void )
{
  return * (volatile uint16_t * )402U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4401 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(* (volatile uint16_t * )386U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired();
    }
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 38 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get();
}

# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__get(void ){
#line 53
  unsigned int __nesc_result;
#line 53

#line 53
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 70 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void )
{
  return * (volatile uint16_t * )384U & 1U;
}

# 35 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void ){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending();
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 43 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending();
}

# 60 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static bool /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 119 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void )
{
  * (volatile uint16_t * )386U |= 0x0010;
}

# 46 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents();
#line 46
}
#line 46
# 84 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )386U &= ~0x0001;
}

# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )402U = x;
}

# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(time);
#line 30
}
#line 30
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 154 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get() + x;
}

# 32 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 70 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 86
          /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 88
    /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt();
    /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents();
  }
}

# 92 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt){
#line 92
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 246 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_leuint8(const void * source)
#line 246
{
  const uint8_t *base = source;

#line 248
  return base[0];
}

# 284 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__RadioBackoff__default__requestCongestionBackoff(am_id_t id, 
message_t *msg)
#line 285
{
}

# 88 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420ActiveMessageP__RadioBackoff__requestCongestionBackoff(am_id_t arg_0xf6a1d4b8, message_t * msg){
#line 88
    CC2420ActiveMessageP__RadioBackoff__default__requestCongestionBackoff(arg_0xf6a1d4b8, msg);
#line 88
}
#line 88
# 233 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__SubBackoff__requestCongestionBackoff(message_t *msg)
#line 233
{
  CC2420ActiveMessageP__RadioBackoff__requestCongestionBackoff(__nesc_ntoh_leuint8(((cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t )))->type.data), msg);
}

# 88 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__RadioBackoff__requestCongestionBackoff(message_t * msg){
#line 88
  CC2420ActiveMessageP__SubBackoff__requestCongestionBackoff(msg);
#line 88
}
#line 88
# 78 "/opt/tinyos-2.x/tos/system/RandomMlcgC.nc"
static inline uint16_t RandomMlcgC__Random__rand16(void )
#line 78
{
  return (uint16_t )RandomMlcgC__Random__rand32();
}

# 41 "/opt/tinyos-2.x/tos/interfaces/Random.nc"
inline static uint16_t CC2420CsmaP__Random__rand16(void ){
#line 41
  unsigned int __nesc_result;
#line 41

#line 41
  __nesc_result = RandomMlcgC__Random__rand16();
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 251 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__RadioBackoff__setCongestionBackoff(uint16_t backoffTime)
#line 251
{
  CC2420TransmitP__myCongestionBackoff = backoffTime + 1;
}

# 66 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__SubBackoff__setCongestionBackoff(uint16_t backoffTime){
#line 66
  CC2420TransmitP__RadioBackoff__setCongestionBackoff(backoffTime);
#line 66
}
#line 66
# 227 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__SubBackoff__requestCongestionBackoff(message_t *msg)
#line 227
{
  CC2420CsmaP__SubBackoff__setCongestionBackoff(CC2420CsmaP__Random__rand16()
   % (0x7 * CC2420_BACKOFF_PERIOD) + CC2420_MIN_BACKOFF);

  CC2420CsmaP__RadioBackoff__requestCongestionBackoff(msg);
}

# 88 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420TransmitP__RadioBackoff__requestCongestionBackoff(message_t * msg){
#line 88
  CC2420CsmaP__SubBackoff__requestCongestionBackoff(msg);
#line 88
}
#line 88
# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = CC2420SpiP__Resource__immediateRequest(/*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 45 "/opt/tinyos-2.x/tos/interfaces/State.nc"
inline static error_t CC2420SpiP__WorkingState__requestState(uint8_t reqState){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = StateImplP__State__requestState(0U, reqState);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 111 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(uint8_t id)
#line 111
{
#line 111
  return FAIL;
}

# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__isOwner(uint8_t arg_0xf6e5cd90){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  switch (arg_0xf6e5cd90) {
#line 118
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 118
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 118
      break;
#line 118
    default:
#line 118
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(arg_0xf6e5cd90);
#line 118
      break;
#line 118
    }
#line 118

#line 118
  return __nesc_result;
#line 118
}
#line 118
# 77 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(uint8_t id)
#line 77
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__isOwner(id);
}

# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static bool CC2420SpiP__SpiResource__isOwner(void ){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 118

#line 118
  return __nesc_result;
#line 118
}
#line 118
# 115 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(uint8_t id)
#line 115
{
  return &msp430_spi_default_config;
}

# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
inline static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(uint8_t arg_0xf6e5b7f8){
#line 39
  union __nesc_unnamed4287 *__nesc_result;
#line 39

#line 39
    __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(arg_0xf6e5b7f8);
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 168 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(msp430_spi_union_config_t *config){
#line 168
  HplMsp430Usart0P__Usart__setModeSpi(config);
#line 168
}
#line 168
# 85 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(uint8_t id)
#line 85
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(id));
}

# 213 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 213
{
}

# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(uint8_t arg_0xf6d4bb38){
#line 49
  switch (arg_0xf6d4bb38) {
#line 49
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 49
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(arg_0xf6d4bb38);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 210 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested(void )
#line 210
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release();
}

# 81 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void ){
#line 81
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested();
#line 81
}
#line 81
# 203 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id)
#line 203
{
}

# 51 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(uint8_t arg_0xf6d4c770){
#line 51
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(arg_0xf6d4c770);
#line 51
}
#line 51
# 90 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id)
#line 90
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /* atomic removed: atomic calls only */
#line 92
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) {
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING;
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = id;
      }
    else {
        unsigned char __nesc_temp = 
#line 97
        FAIL;

#line 97
        return __nesc_temp;
      }
  }
#line 99
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested();
  if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
      return SUCCESS;
    }
  /* atomic removed: atomic calls only */
#line 104
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
  return FAIL;
}

# 113 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(uint8_t id)
#line 113
{
#line 113
  return FAIL;
}

# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__immediateRequest(uint8_t arg_0xf6e5cd90){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  switch (arg_0xf6e5cd90) {
#line 87
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 87
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 87
      break;
#line 87
    default:
#line 87
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(arg_0xf6e5cd90);
#line 87
      break;
#line 87
    }
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 69 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(uint8_t id)
#line 69
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__immediateRequest(id);
}

# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 86 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP__isWaiting(uint8_t id)
{
  return SchedulerBasicP__m_next[id] != SchedulerBasicP__NO_TASK || SchedulerBasicP__m_tail == id;
}

static inline bool SchedulerBasicP__pushTask(uint8_t id)
{
  if (!SchedulerBasicP__isWaiting(id)) 
    {
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_head = id;
          SchedulerBasicP__m_tail = id;
        }
      else 
        {
          SchedulerBasicP__m_next[SchedulerBasicP__m_tail] = id;
          SchedulerBasicP__m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

# 151 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__resetUsart(bool reset)
#line 151
{
  if (reset) {
      U0CTL = 0x01;
    }
  else {
      U0CTL &= ~0x01;
    }
}

# 97 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void HplMsp430I2C0P__HplUsart__resetUsart(bool reset){
#line 97
  HplMsp430Usart0P__Usart__resetUsart(reset);
#line 97
}
#line 97
# 59 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static inline void HplMsp430I2C0P__HplI2C__clearModeI2C(void )
#line 59
{
  /* atomic removed: atomic calls only */
#line 60
  {
    HplMsp430I2C0P__U0CTL &= ~((0x20 | 0x04) | 0x01);
    HplMsp430I2C0P__HplUsart__resetUsart(TRUE);
  }
}

# 7 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static void HplMsp430Usart0P__HplI2C__clearModeI2C(void ){
#line 7
  HplMsp430I2C0P__HplI2C__clearModeI2C();
#line 7
}
#line 7
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 5);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__URXD__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 4);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UTXD__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc();
#line 85
}
#line 85
# 207 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__disableUart(void )
#line 207
{
  /* atomic removed: atomic calls only */
#line 208
  {
    HplMsp430Usart0P__ME1 &= ~((1 << 7) | (1 << 6));
    HplMsp430Usart0P__UTXD__selectIOFunc();
    HplMsp430Usart0P__URXD__selectIOFunc();
  }
}

#line 143
static inline void HplMsp430Usart0P__Usart__setUmctl(uint8_t control)
#line 143
{
  U0MCTL = control;
}

#line 132
static inline void HplMsp430Usart0P__Usart__setUbr(uint16_t control)
#line 132
{
  /* atomic removed: atomic calls only */
#line 133
  {
    U0BR0 = control & 0x00FF;
    U0BR1 = (control >> 8) & 0x00FF;
  }
}

#line 256
static inline void HplMsp430Usart0P__configSpi(msp430_spi_union_config_t *config)
#line 256
{

  U0CTL = (config->spiRegisters.uctl | 0x04) | 0x01;
  HplMsp430Usart0P__U0TCTL = config->spiRegisters.utctl;

  HplMsp430Usart0P__Usart__setUbr(config->spiRegisters.ubr);
  HplMsp430Usart0P__Usart__setUmctl(0x00);
}

# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 3;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UCLK__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc();
#line 78
}
#line 78
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 2;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SOMI__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc();
#line 78
}
#line 78
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 1;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SIMO__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc();
#line 78
}
#line 78
# 238 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__enableSpi(void )
#line 238
{
  /* atomic removed: atomic calls only */
#line 239
  {
    HplMsp430Usart0P__SIMO__selectModuleFunc();
    HplMsp430Usart0P__SOMI__selectModuleFunc();
    HplMsp430Usart0P__UCLK__selectModuleFunc();
  }
  HplMsp430Usart0P__ME1 |= 1 << 6;
}

#line 345
static inline void HplMsp430Usart0P__Usart__clrIntr(void )
#line 345
{
  HplMsp430Usart0P__IFG1 &= ~((1 << 7) | (1 << 6));
}









static inline void HplMsp430Usart0P__Usart__disableIntr(void )
#line 357
{
  HplMsp430Usart0P__IE1 &= ~((1 << 7) | (1 << 6));
}

# 118 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
static inline void StateImplP__State__toIdle(uint8_t id)
#line 118
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 119
    StateImplP__state[id] = StateImplP__S_IDLE;
#line 119
    __nesc_atomic_end(__nesc_atomic); }
}

# 56 "/opt/tinyos-2.x/tos/interfaces/State.nc"
inline static void CC2420SpiP__WorkingState__toIdle(void ){
#line 56
  StateImplP__State__toIdle(0U);
#line 56
}
#line 56
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 207 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested(void )
#line 207
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release();
}

# 73 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested(void ){
#line 73
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested();
#line 73
}
#line 73
# 54 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 54
{
  /* atomic removed: atomic calls only */
#line 55
  {
    unsigned char __nesc_temp = 
#line 55
    /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[id] != /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY || /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail == id;

#line 55
    return __nesc_temp;
  }
}

#line 72
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id)
#line 72
{
  /* atomic removed: atomic calls only */
#line 73
  {
    if (!/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(id)) {
        if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = id;
          }
        else {
#line 78
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail] = id;
          }
#line 79
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = id;
        {
          unsigned char __nesc_temp = 
#line 80
          SUCCESS;

#line 80
          return __nesc_temp;
        }
      }
#line 82
    {
      unsigned char __nesc_temp = 
#line 82
      EBUSY;

#line 82
      return __nesc_temp;
    }
  }
}

# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(resource_client_id_t id){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(id);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 201 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(uint8_t id)
#line 201
{
}

# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(uint8_t arg_0xf6d4c770){
#line 43
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(arg_0xf6d4c770);
#line 43
}
#line 43
# 77 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(uint8_t id)
#line 77
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /* atomic removed: atomic calls only */
#line 79
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) {
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING;
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = id;
      }
    else {
        unsigned char __nesc_temp = 
#line 84
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(id);

#line 84
        return __nesc_temp;
      }
  }
#line 86
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested();
  return SUCCESS;
}

# 112 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(uint8_t id)
#line 112
{
#line 112
  return FAIL;
}

# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(uint8_t arg_0xf6e5cd90){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  switch (arg_0xf6e5cd90) {
#line 78
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 78
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 78
      break;
#line 78
    default:
#line 78
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(arg_0xf6e5cd90);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 73 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(uint8_t id)
#line 73
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(id);
}

# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 382 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__tx(uint8_t data)
#line 382
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 383
    HplMsp430Usart0P__U0TXBUF = data;
#line 383
    __nesc_atomic_end(__nesc_atomic); }
}

# 224 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(uint8_t data){
#line 224
  HplMsp430Usart0P__Usart__tx(data);
#line 224
}
#line 224
# 330 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline bool HplMsp430Usart0P__Usart__isRxIntrPending(void )
#line 330
{
  if (HplMsp430Usart0P__IFG1 & (1 << 6)) {
      return TRUE;
    }
  return FALSE;
}

# 192 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending(void ){
#line 192
  unsigned char __nesc_result;
#line 192

#line 192
  __nesc_result = HplMsp430Usart0P__Usart__isRxIntrPending();
#line 192

#line 192
  return __nesc_result;
#line 192
}
#line 192
# 341 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__clrRxIntr(void )
#line 341
{
  HplMsp430Usart0P__IFG1 &= ~(1 << 6);
}

# 197 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__clrRxIntr(void ){
#line 197
  HplMsp430Usart0P__Usart__clrRxIntr();
#line 197
}
#line 197
#line 231
inline static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx(void ){
#line 231
  unsigned char __nesc_result;
#line 231

#line 231
  __nesc_result = HplMsp430Usart0P__Usart__rx();
#line 231

#line 231
  return __nesc_result;
#line 231
}
#line 231
# 95 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__ChipSpiResource__abortRelease(void )
#line 95
{
  /* atomic removed: atomic calls only */
#line 96
  CC2420SpiP__release = FALSE;
}

# 31 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static void CC2420TransmitP__ChipSpiResource__abortRelease(void ){
#line 31
  CC2420SpiP__ChipSpiResource__abortRelease();
#line 31
}
#line 31
# 375 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__ChipSpiResource__releasing(void )
#line 375
{
  if (CC2420TransmitP__abortSpiRelease) {
      CC2420TransmitP__ChipSpiResource__abortRelease();
    }
}

# 24 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static void CC2420SpiP__ChipSpiResource__releasing(void ){
#line 24
  CC2420TransmitP__ChipSpiResource__releasing();
#line 24
}
#line 24
# 205 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void )
#line 205
{
}

# 46 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void ){
#line 46
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted();
#line 46
}
#line 46
# 97 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(bool reset){
#line 97
  HplMsp430Usart0P__Usart__resetUsart(reset);
#line 97
}
#line 97
#line 158
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi(void ){
#line 158
  HplMsp430Usart0P__Usart__disableSpi();
#line 158
}
#line 158
# 89 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(uint8_t id)
#line 89
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(TRUE);
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi();
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(FALSE);
}

# 215 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id)
#line 215
{
}

# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(uint8_t arg_0xf6d4bb38){
#line 55
  switch (arg_0xf6d4bb38) {
#line 55
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 55
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 55
      break;
#line 55
    default:
#line 55
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(arg_0xf6d4bb38);
#line 55
      break;
#line 55
    }
#line 55
}
#line 55
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 58 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead != /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
        uint8_t id = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead;

#line 62
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead];
        if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
          }
#line 65
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[id] = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 66
          id;

#line 66
          return __nesc_temp;
        }
      }
#line 68
    {
      unsigned char __nesc_temp = 
#line 68
      /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

#line 68
      return __nesc_temp;
    }
  }
}

# 60 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 50 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 51
  {
    unsigned char __nesc_temp = 
#line 51
    /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

#line 51
    return __nesc_temp;
  }
}

# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 108 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id)
#line 108
{
  /* atomic removed: atomic calls only */
#line 109
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY && /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
        if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty() == FALSE) {
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue();
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
          }
        else {
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted();
          }
        {
          unsigned char __nesc_temp = 
#line 124
          SUCCESS;

#line 124
          return __nesc_temp;
        }
      }
  }
#line 127
  return FAIL;
}

# 114 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(uint8_t id)
#line 114
{
#line 114
  return FAIL;
}

# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(uint8_t arg_0xf6e5cd90){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  switch (arg_0xf6e5cd90) {
#line 110
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 110
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 110
      break;
#line 110
    default:
#line 110
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(arg_0xf6e5cd90);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 81 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(uint8_t id)
#line 81
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(id);
}

# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 1);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SIMO__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 2);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SOMI__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 3);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UCLK__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc();
#line 85
}
#line 85
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420TransmitP__STXONCCA__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_STXONCCA);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
inline static cc2420_status_t CC2420TransmitP__STXON__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_STXON);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
inline static cc2420_status_t CC2420TransmitP__SNOP__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SNOP);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 102 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline error_t CC2420SpiP__ChipSpiResource__attemptRelease(void )
#line 102
{
  return CC2420SpiP__attemptRelease();
}

# 39 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static error_t CC2420TransmitP__ChipSpiResource__attemptRelease(void ){
#line 39
  unsigned char __nesc_result;
#line 39

#line 39
  __nesc_result = CC2420SpiP__ChipSpiResource__attemptRelease();
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )31U |= 0x01 << 1;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc();
#line 78
}
#line 78
# 46 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4402 {
#line 46
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 61
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__captureControl(uint8_t l_cm)
{
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x = { 
  .cm = l_cm & 0x03, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 1, 
  .scs = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(x);
}

#line 99
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm)
{
  * (volatile uint16_t * )388U = /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__captureControl(cm);
}

# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(uint8_t cm){
#line 44
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(cm);
#line 44
}
#line 44
# 119 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void )
{
  * (volatile uint16_t * )388U |= 0x0010;
}

# 46 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents();
#line 46
}
#line 46
# 181 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void )
{
  return * (volatile uint16_t * )404U;
}

# 276 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_leuint16(const void * source)
#line 276
{
  const uint8_t *base = source;

#line 278
  return ((uint16_t )base[1] << 8) | base[0];
}

#line 301
static __inline  uint32_t __nesc_hton_uint32(void * target, uint32_t value)
#line 301
{
  uint8_t *base = target;

#line 303
  base[3] = value;
  base[2] = value >> 8;
  base[1] = value >> 16;
  base[0] = value >> 24;
  return value;
}

#line 294
static __inline  uint32_t __nesc_ntoh_uint32(const void * source)
#line 294
{
  const uint8_t *base = source;

#line 296
  return ((((uint32_t )base[0] << 24) | (
  (uint32_t )base[1] << 16)) | (
  (uint32_t )base[2] << 8)) | base[3];
}

# 59 "/opt/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420TransmitP__PacketTimeStamp__clear(message_t * msg){
#line 59
  CC2420PacketP__PacketTimeStamp32khz__clear(msg);
#line 59
}
#line 59
# 195 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__CC2420Receive__sfd_dropped(void )
#line 195
{
  if (CC2420ReceiveP__m_timestamp_size) {
      CC2420ReceiveP__m_timestamp_size--;
    }
}

# 55 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc"
inline static void CC2420TransmitP__CC2420Receive__sfd_dropped(void ){
#line 55
  CC2420ReceiveP__CC2420Receive__sfd_dropped();
#line 55
}
#line 55
# 48 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )28U & (0x01 << 1);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw() != 0;
}

# 59 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__5__HplGeneralIO__get(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 40 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__5__GeneralIO__get(void )
#line 40
{
#line 40
  return /*HplCC2420PinsC.SFDM*/Msp430GpioC__5__HplGeneralIO__get();
}

# 32 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static bool CC2420TransmitP__SFD__get(void ){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = /*HplCC2420PinsC.SFDM*/Msp430GpioC__5__GeneralIO__get();
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 186 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__CC2420Receive__sfd(uint32_t time)
#line 186
{
  if (CC2420ReceiveP__m_timestamp_size < CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE) {
      uint8_t tail = (CC2420ReceiveP__m_timestamp_head + CC2420ReceiveP__m_timestamp_size) % 
      CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE;

#line 190
      CC2420ReceiveP__m_timestamp_queue[tail] = time;
      CC2420ReceiveP__m_timestamp_size++;
    }
}

# 49 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc"
inline static void CC2420TransmitP__CC2420Receive__sfd(uint32_t time){
#line 49
  CC2420ReceiveP__CC2420Receive__sfd(time);
#line 49
}
#line 49
# 54 "/opt/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge(void )
#line 54
{
  return /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(MSP430TIMER_CM_FALLING);
}

# 43 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
inline static error_t CC2420TransmitP__CaptureSFD__captureFallingEdge(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 114 "/opt/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline cc2420_header_t * CC2420PacketP__CC2420PacketBody__getHeader(message_t * msg)
#line 114
{
  return (cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
}

# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420TransmitP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 54 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void ){
#line 62
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop();
#line 62
}
#line 62
# 91 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__stop(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__stop();
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void CC2420TransmitP__BackoffTimer__stop(void ){
#line 62
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__stop();
#line 62
}
#line 62
# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__HplGeneralIO__set();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CSN__set(void ){
#line 29
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__GeneralIO__set();
#line 29
}
#line 29
# 63 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc"
inline static cc2420_status_t CC2420TransmitP__TXFIFO_RAM__write(uint8_t offset, uint8_t * data, uint8_t length){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Ram__write(CC2420_RAM_TXFIFO, offset, data, length);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__GeneralIO__clr(void )
#line 38
{
#line 38
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__HplGeneralIO__clr();
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CSN__clr(void ){
#line 30
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__GeneralIO__clr();
#line 30
}
#line 30
# 185 "/opt/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline uint8_t CC2420PacketP__PacketTimeSyncOffset__get(message_t *msg)
{
  return __nesc_ntoh_leuint8(CC2420PacketP__CC2420PacketBody__getHeader(msg)->length.data)
   + (sizeof(cc2420_header_t ) - MAC_HEADER_SIZE)
   - MAC_FOOTER_SIZE
   - sizeof(timesync_radio_t );
}

# 47 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
inline static uint8_t CC2420TransmitP__PacketTimeSyncOffset__get(message_t * msg){
#line 47
  unsigned char __nesc_result;
#line 47

#line 47
  __nesc_result = CC2420PacketP__PacketTimeSyncOffset__get(msg);
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
# 235 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_uint8(const void * source)
#line 235
{
  const uint8_t *base = source;

#line 237
  return base[0];
}

#line 257
static __inline  int8_t __nesc_ntoh_int8(const void * source)
#line 257
{
#line 257
  return __nesc_ntoh_uint8(source);
}

# 118 "/opt/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline cc2420_metadata_t *CC2420PacketP__CC2420PacketBody__getMetadata(message_t *msg)
#line 118
{
  return (cc2420_metadata_t *)msg->metadata;
}

#line 176
static inline bool CC2420PacketP__PacketTimeSyncOffset__isSet(message_t *msg)
{
  return __nesc_ntoh_int8(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timesync.data);
}

# 39 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
inline static bool CC2420TransmitP__PacketTimeSyncOffset__isSet(message_t * msg){
#line 39
  unsigned char __nesc_result;
#line 39

#line 39
  __nesc_result = CC2420PacketP__PacketTimeSyncOffset__isSet(msg);
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 143 "/opt/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline void CC2420PacketP__PacketTimeStamp32khz__set(message_t *msg, uint32_t value)
{
  __nesc_hton_uint32(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timestamp.data, value);
}

# 67 "/opt/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420TransmitP__PacketTimeStamp__set(message_t * msg, CC2420TransmitP__PacketTimeStamp__size_type value){
#line 67
  CC2420PacketP__PacketTimeStamp32khz__set(msg, value);
#line 67
}
#line 67
# 98 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static CC2420TransmitP__BackoffTimer__size_type CC2420TransmitP__BackoffTimer__getNow(void ){
#line 98
  unsigned long __nesc_result;
#line 98

#line 98
  __nesc_result = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 259 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static __inline uint32_t CC2420TransmitP__getTime32(uint16_t time)
{
  uint32_t recent_time = CC2420TransmitP__BackoffTimer__getNow();

#line 262
  return recent_time + (int16_t )(time - recent_time);
}

#line 278
static inline void CC2420TransmitP__CaptureSFD__captured(uint16_t time)
#line 278
{
  unsigned char *__nesc_temp45;
  unsigned char *__nesc_temp44;
#line 279
  uint32_t time32;
  uint8_t sfd_state = 0;

  /* atomic removed: atomic calls only */
#line 281
  {
    time32 = CC2420TransmitP__getTime32(time);
    switch (CC2420TransmitP__m_state) {

        case CC2420TransmitP__S_SFD: 
          CC2420TransmitP__m_state = CC2420TransmitP__S_EFD;
        CC2420TransmitP__sfdHigh = TRUE;


        CC2420TransmitP__m_receiving = FALSE;
        CC2420TransmitP__CaptureSFD__captureFallingEdge();
        CC2420TransmitP__PacketTimeStamp__set(CC2420TransmitP__m_msg, time32);
        if (CC2420TransmitP__PacketTimeSyncOffset__isSet(CC2420TransmitP__m_msg)) {
            uint8_t absOffset = sizeof(message_header_t ) - sizeof(cc2420_header_t ) + CC2420TransmitP__PacketTimeSyncOffset__get(CC2420TransmitP__m_msg);
            timesync_radio_t *timesync = (timesync_radio_t *)((nx_uint8_t *)CC2420TransmitP__m_msg + absOffset);

            (__nesc_temp44 = (*timesync).data, __nesc_hton_uint32(__nesc_temp44, __nesc_ntoh_uint32(__nesc_temp44) - time32));
            CC2420TransmitP__CSN__clr();
            CC2420TransmitP__TXFIFO_RAM__write(absOffset, (uint8_t *)timesync, sizeof(timesync_radio_t ));
            CC2420TransmitP__CSN__set();

            (__nesc_temp45 = (*timesync).data, __nesc_hton_uint32(__nesc_temp45, __nesc_ntoh_uint32(__nesc_temp45) + time32));
          }

        if (__nesc_ntoh_leuint16(CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg)->fcf.data) & (1 << IEEE154_FCF_ACK_REQ)) {

            CC2420TransmitP__abortSpiRelease = TRUE;
          }
        CC2420TransmitP__releaseSpiResource();
        CC2420TransmitP__BackoffTimer__stop();

        if (CC2420TransmitP__SFD__get()) {
            break;
          }


        case CC2420TransmitP__S_EFD: 
          CC2420TransmitP__sfdHigh = FALSE;
        CC2420TransmitP__CaptureSFD__captureRisingEdge();

        if (__nesc_ntoh_leuint16(CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg)->fcf.data) & (1 << IEEE154_FCF_ACK_REQ)) {
            CC2420TransmitP__m_state = CC2420TransmitP__S_ACK_WAIT;
            CC2420TransmitP__BackoffTimer__start(CC2420_ACK_WAIT_DELAY);
          }
        else 
#line 324
          {
            CC2420TransmitP__signalDone(SUCCESS);
          }

        if (!CC2420TransmitP__SFD__get()) {
            break;
          }


        default: 

          if (!CC2420TransmitP__m_receiving && CC2420TransmitP__sfdHigh == FALSE) {
              CC2420TransmitP__sfdHigh = TRUE;
              CC2420TransmitP__CaptureSFD__captureFallingEdge();

              sfd_state = CC2420TransmitP__SFD__get();
              CC2420TransmitP__CC2420Receive__sfd(time32);
              CC2420TransmitP__m_receiving = TRUE;
              CC2420TransmitP__m_prev_time = time;
              if (CC2420TransmitP__SFD__get()) {

                  return;
                }
            }



        if (CC2420TransmitP__sfdHigh == TRUE) {
            CC2420TransmitP__sfdHigh = FALSE;
            CC2420TransmitP__CaptureSFD__captureRisingEdge();
            CC2420TransmitP__m_receiving = FALSE;








            if (sfd_state == 0 && time - CC2420TransmitP__m_prev_time < 10) {
                CC2420TransmitP__CC2420Receive__sfd_dropped();
                if (CC2420TransmitP__m_msg) {
                  CC2420TransmitP__PacketTimeStamp__clear(CC2420TransmitP__m_msg);
                  }
              }
#line 368
            break;
          }
      }
  }
}

# 50 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captured(uint16_t time){
#line 50
  CC2420TransmitP__CaptureSFD__captured(time);
#line 50
}
#line 50
# 164 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void )
{
  * (volatile uint16_t * )388U &= ~0x0002;
}

# 57 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow();
#line 57
}
#line 57
# 84 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )388U &= ~0x0001;
}

# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 65 "/opt/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time)
#line 65
{
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt();
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow();
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captured(time);
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time){
#line 75
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4403 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(* (volatile uint16_t * )388U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired();
    }
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 70 "/opt/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void )
{
#line 71
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask();
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__fired(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired();
#line 67
}
#line 67
# 151 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt == 0) 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__fired();
      }
    else 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__set_alarm();
      }
  }
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void ){
#line 67
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__fired();
#line 67
}
#line 67
# 124 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void )
{
  * (volatile uint16_t * )390U &= ~0x0010;
}

# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents();
#line 47
}
#line 47
# 59 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired();
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void ){
#line 34
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void )
{
  return * (volatile uint16_t * )406U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4404 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(* (volatile uint16_t * )390U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired();
    }
}

# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__get(void ){
#line 53
  unsigned int __nesc_result;
#line 53

#line 53
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53







inline static bool /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 119 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void )
{
  * (volatile uint16_t * )390U |= 0x0010;
}

# 46 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents();
#line 46
}
#line 46
# 84 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )390U &= ~0x0001;
}

# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )406U = x;
}

# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(time);
#line 30
}
#line 30
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 154 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )406U = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get() + x;
}

# 32 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 70 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 86
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 88
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents();
  }
}

# 92 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 324 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline error_t AdcStreamP__SingleChannel__default__getData(uint8_t c)
{
  return FAIL;
}

# 189 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t AdcStreamP__SingleChannel__getData(uint8_t arg_0xf65c1170){
#line 189
  unsigned char __nesc_result;
#line 189

#line 189
  switch (arg_0xf65c1170) {
#line 189
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT:
#line 189
      __nesc_result = Msp430Adc12ImplP__SingleChannel__getData(/*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID);
#line 189
      break;
#line 189
    default:
#line 189
      __nesc_result = AdcStreamP__SingleChannel__default__getData(arg_0xf65c1170);
#line 189
      break;
#line 189
    }
#line 189

#line 189
  return __nesc_result;
#line 189
}
#line 189
# 92 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline void AdcStreamP__sampleSingle(void )
#line 92
{
  AdcStreamP__SingleChannel__getData(AdcStreamP__client);
}

#line 173
static inline void AdcStreamP__Alarm__fired(void )
#line 173
{
  AdcStreamP__sampleSingle();
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Alarm__fired(void ){
#line 67
  AdcStreamP__Alarm__fired();
#line 67
}
#line 67
# 151 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__m_dt == 0) 
      {
        /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Alarm__fired();
      }
    else 
      {
        /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__set_alarm();
      }
  }
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__fired(void ){
#line 67
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__AlarmFrom__fired();
#line 67
}
#line 67
# 124 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__disableEvents(void )
{
  * (volatile uint16_t * )392U &= ~0x0010;
}

# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__disableEvents();
#line 47
}
#line 47
# 59 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired(void )
{
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__disableEvents();
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__fired();
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void ){
#line 34
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void )
{
  return * (volatile uint16_t * )408U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6____nesc_unnamed4405 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(* (volatile uint16_t * )392U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired();
    }
}

# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 7);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port67__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 7;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port67__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 6);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port66__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 6;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port66__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 5);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port65__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 5;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port65__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 4);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port64__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 4;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port64__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 3);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port63__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 3;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port63__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 2);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port62__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 2;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port62__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 1);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port61__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 1;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port61__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 0);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port60__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 0;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port60__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectModuleFunc();
#line 78
}
#line 78
# 142 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__configureAdcPin(uint8_t inch)
{

  switch (inch) 
    {
      case 0: Msp430Adc12ImplP__Port60__selectModuleFunc();
#line 147
      Msp430Adc12ImplP__Port60__makeInput();
#line 147
      break;
      case 1: Msp430Adc12ImplP__Port61__selectModuleFunc();
#line 148
      Msp430Adc12ImplP__Port61__makeInput();
#line 148
      break;
      case 2: Msp430Adc12ImplP__Port62__selectModuleFunc();
#line 149
      Msp430Adc12ImplP__Port62__makeInput();
#line 149
      break;
      case 3: Msp430Adc12ImplP__Port63__selectModuleFunc();
#line 150
      Msp430Adc12ImplP__Port63__makeInput();
#line 150
      break;
      case 4: Msp430Adc12ImplP__Port64__selectModuleFunc();
#line 151
      Msp430Adc12ImplP__Port64__makeInput();
#line 151
      break;
      case 5: Msp430Adc12ImplP__Port65__selectModuleFunc();
#line 152
      Msp430Adc12ImplP__Port65__makeInput();
#line 152
      break;
      case 6: Msp430Adc12ImplP__Port66__selectModuleFunc();
#line 153
      Msp430Adc12ImplP__Port66__makeInput();
#line 153
      break;
      case 7: Msp430Adc12ImplP__Port67__selectModuleFunc();
#line 154
      Msp430Adc12ImplP__Port67__makeInput();
#line 154
      break;
    }
}

# 100 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline void HplAdc12P__HplAdc12__startConversion(void )
#line 100
{
  HplAdc12P__ADC12CTL0 |= 0x0010;
  HplAdc12P__ADC12CTL0 |= 0x0001 + 0x0002;
}

# 128 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP__HplAdc12__startConversion(void ){
#line 128
  HplAdc12P__HplAdc12__startConversion();
#line 128
}
#line 128
# 39 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void Msp430Adc12ImplP__TimerA__setMode(int mode){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setMode(mode);
#line 39
}
#line 39
# 46 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__CC2int(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4406 {
#line 46
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 89
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__setControl(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t x)
{
  * (volatile uint16_t * )356U = /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__CC2int(x);
}

# 35 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void Msp430Adc12ImplP__ControlA1__setControl(msp430_compare_control_t control){
#line 35
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__setControl(control);
#line 35
}
#line 35
# 121 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__startTimerA(void )
{

  msp430_compare_control_t ccSetSHI = { 
  .ccifg = 0, .cov = 0, .out = 1, .cci = 0, .ccie = 0, 
  .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };
  msp430_compare_control_t ccResetSHI = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };
  msp430_compare_control_t ccRSOutmod = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 7, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };

  Msp430Adc12ImplP__ControlA1__setControl(ccResetSHI);
  Msp430Adc12ImplP__ControlA1__setControl(ccSetSHI);

  Msp430Adc12ImplP__ControlA1__setControl(ccRSOutmod);
  Msp430Adc12ImplP__TimerA__setMode(MSP430TIMER_UP_MODE);
}

# 119 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__enableEvents(void )
{
  * (volatile uint16_t * )392U |= 0x0010;
}

# 46 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__enableEvents();
#line 46
}
#line 46
# 84 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )392U &= ~0x0001;
}

# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )408U = x;
}

# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEvent(time);
#line 30
}
#line 30
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 154 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )408U = /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__get() + x;
}

# 32 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 70 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 86
          /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 88
    /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__clearPendingInterrupt();
    /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__enableEvents();
  }
}

# 92 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__AlarmFrom__startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__AlarmFrom__size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__AlarmFrom__size_type dt){
#line 92
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 181 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void )
{
  return * (volatile uint16_t * )410U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7____nesc_unnamed4407 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(* (volatile uint16_t * )394U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void )
{
  return * (volatile uint16_t * )412U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8____nesc_unnamed4408 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(* (volatile uint16_t * )396U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void )
{
  return * (volatile uint16_t * )414U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9____nesc_unnamed4409 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(* (volatile uint16_t * )398U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired();
    }
}

# 120 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )286U;

#line 123
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(n >> 1);
}

# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB1__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired();
#line 28
}
#line 28
# 113 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__Scheduler__init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP__m_next, SchedulerBasicP__NO_TASK, sizeof SchedulerBasicP__m_next);
    SchedulerBasicP__m_head = SchedulerBasicP__NO_TASK;
    SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
  }
}

# 46 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__init(void ){
#line 46
  SchedulerBasicP__Scheduler__init();
#line 46
}
#line 46
# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__GeneralIO__set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__HplGeneralIO__set();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__set(void ){
#line 29
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__GeneralIO__set();
#line 29
}
#line 29
# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__GeneralIO__set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__HplGeneralIO__set();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__set(void ){
#line 29
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__GeneralIO__set();
#line 29
}
#line 29
# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__GeneralIO__set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__HplGeneralIO__set();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__set(void ){
#line 29
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__GeneralIO__set();
#line 29
}
#line 29
# 52 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 6;
}

# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__HplGeneralIO__makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 5;
}

# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__HplGeneralIO__makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 4;
}

# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__HplGeneralIO__makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__GeneralIO__makeOutput();
#line 35
}
#line 35
# 45 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 46
  {
    ;
    LedsP__Led0__makeOutput();
    LedsP__Led1__makeOutput();
    LedsP__Led2__makeOutput();
    LedsP__Led0__set();
    LedsP__Led1__set();
    LedsP__Led2__set();
  }
  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t PlatformP__LedsInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = LedsP__Init__init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 36 "/opt/tinyos-2.x/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r |= 1 << 1;
}

#line 37
static inline  void TOSH_SET_UCLK0_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0019");

#line 37
  r |= 1 << 3;
}

#line 88
static inline  void TOSH_SET_FLASH_CS_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001D");

#line 88
  r |= 1 << 4;
}

#line 37
static inline  void TOSH_CLR_UCLK0_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0019");

#line 37
  r &= ~(1 << 3);
}

#line 88
static inline  void TOSH_CLR_FLASH_CS_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001D");

#line 88
  r &= ~(1 << 4);
}

# 11 "/opt/tinyos-2.x/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__TOSH_wait(void )
#line 11
{
   __asm volatile ("nop"); __asm volatile ("nop");}

# 89 "/opt/tinyos-2.x/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_FLASH_HOLD_PIN()
#line 89
{
#line 89
  static volatile uint8_t r __asm ("0x001D");

#line 89
  r |= 1 << 7;
}

#line 88
static inline  void TOSH_MAKE_FLASH_CS_OUTPUT()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001E");

#line 88
  r |= 1 << 4;
}

#line 89
static inline  void TOSH_MAKE_FLASH_HOLD_OUTPUT()
#line 89
{
#line 89
  static volatile uint8_t r __asm ("0x001E");

#line 89
  r |= 1 << 7;
}

#line 37
static inline  void TOSH_MAKE_UCLK0_OUTPUT()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x001A");

#line 37
  r |= 1 << 3;
}

#line 36
static inline  void TOSH_MAKE_SIMO0_OUTPUT()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x001A");

#line 36
  r |= 1 << 1;
}

# 27 "/opt/tinyos-2.x/tos/platforms/telosb/MotePlatformC.nc"
static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void )
#line 27
{

  TOSH_MAKE_SIMO0_OUTPUT();
  TOSH_MAKE_UCLK0_OUTPUT();
  TOSH_MAKE_FLASH_HOLD_OUTPUT();
  TOSH_MAKE_FLASH_CS_OUTPUT();
  TOSH_SET_FLASH_HOLD_PIN();
  TOSH_SET_FLASH_CS_PIN();

  MotePlatformC__TOSH_wait();


  TOSH_CLR_FLASH_CS_PIN();
  TOSH_CLR_UCLK0_PIN();

  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);

  TOSH_SET_FLASH_CS_PIN();
  TOSH_SET_UCLK0_PIN();
  TOSH_SET_SIMO0_PIN();
}

#line 6
static __inline void MotePlatformC__uwait(uint16_t u)
#line 6
{
  uint16_t t0 = TA0R;

#line 8
  while (TA0R - t0 <= u) ;
}

#line 56
static inline error_t MotePlatformC__Init__init(void )
#line 56
{
  /* atomic removed: atomic calls only */

  {
    P1SEL = 0;
    P2SEL = 0;
    P3SEL = 0;
    P4SEL = 0;
    P5SEL = 0;
    P6SEL = 0;

    P1OUT = 0x00;
    P1DIR = 0xe0;

    P2OUT = 0x30;
    P2DIR = 0x7b;

    P3OUT = 0x00;
    P3DIR = 0xf1;

    P4OUT = 0xdd;
    P4DIR = 0xfd;

    P5OUT = 0xff;
    P5DIR = 0xff;

    P6OUT = 0x00;
    P6DIR = 0xff;

    P1IE = 0;
    P2IE = 0;






    MotePlatformC__uwait(1024 * 10);

    MotePlatformC__TOSH_FLASH_M25P_DP();
  }

  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = MotePlatformC__Init__init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 152 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__startTimerB(void )
{

  Msp430ClockP__TBCTL = 0x0020 | (Msp430ClockP__TBCTL & ~(0x0020 | 0x0010));
}

#line 140
static inline void Msp430ClockP__startTimerA(void )
{

  Msp430ClockP__TA0CTL = 0x0020 | (Msp430ClockP__TA0CTL & ~(0x0020 | 0x0010));
}

#line 104
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void )
{
  TBR = 0;









  Msp430ClockP__TBCTL = 0x0100 | 0x0002;
}

#line 134
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerB();
}

# 32 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerB(void ){
#line 32
  Msp430ClockP__Msp430ClockInit__default__initTimerB();
#line 32
}
#line 32
# 89 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void )
{
  TA0R = 0;









  Msp430ClockP__TA0CTL = 0x0200 | 0x0002;
}

#line 129
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerA();
}

# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerA(void ){
#line 31
  Msp430ClockP__Msp430ClockInit__default__initTimerA();
#line 31
}
#line 31
# 68 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void )
{





  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));







  BCSCTL2 = 0x04;


  Msp430ClockP__IE1 &= ~(1 << 1);
}

#line 124
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitClocks();
}

# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initClocks(void ){
#line 30
  Msp430ClockP__Msp430ClockInit__default__initClocks();
#line 30
}
#line 30
# 170 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib)
{
  int8_t aclk_count = 2;
  uint16_t dco_prev = 0;
  uint16_t dco_curr = 0;

  Msp430ClockP__set_dco_calib(calib);

  while (aclk_count-- > 0) 
    {
      TBCCR0 = TBR + Msp430ClockP__ACLK_CALIB_PERIOD;
      TBCCTL0 &= ~0x0001;
      while ((TBCCTL0 & 0x0001) == 0) ;
      dco_prev = dco_curr;
      dco_curr = TA0R;
    }

  return dco_curr - dco_prev;
}




static inline void Msp430ClockP__busyCalibrateDco(void )
{

  int calib;
  int step;






  for (calib = 0, step = 0x800; step != 0; step >>= 1) 
    {

      if (Msp430ClockP__test_calib_busywait_delta(calib | step) <= Msp430ClockP__TARGET_DCO_DELTA) {
        calib |= step;
        }
    }

  if ((calib & 0x0e0) == 0x0e0) {
    calib &= ~0x01f;
    }
  Msp430ClockP__set_dco_calib(calib);
}

#line 56
static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void )
{



  Msp430ClockP__TA0CTL = 0x0200 | 0x0020;
  Msp430ClockP__TBCTL = 0x0100 | 0x0020;
  BCSCTL1 = 0x80 | 0x04;
  BCSCTL2 = 0;
  TBCCTL0 = 0x4000;
}

#line 119
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void )
{
  Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate();
}

# 29 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void ){
#line 29
  Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate();
#line 29
}
#line 29
# 218 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline error_t Msp430ClockP__Init__init(void )
{

  Msp430ClockP__TA0CTL = 0x0004;
  Msp430ClockP__TA0IV = 0;
  Msp430ClockP__TBCTL = 0x0004;
  Msp430ClockP__TBIV = 0;
  /* atomic removed: atomic calls only */

  {
    Msp430ClockP__Msp430ClockInit__setupDcoCalibrate();
    Msp430ClockP__busyCalibrateDco();
    Msp430ClockP__Msp430ClockInit__initClocks();
    Msp430ClockP__Msp430ClockInit__initTimerA();
    Msp430ClockP__Msp430ClockInit__initTimerB();
    Msp430ClockP__startTimerA();
    Msp430ClockP__startTimerB();
  }

  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteClockInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = Msp430ClockP__Init__init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 10 "/opt/tinyos-2.x/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void )
#line 10
{
  PlatformP__MoteClockInit__init();
  PlatformP__MoteInit__init();
  PlatformP__LedsInit__init();
  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t RealMainP__PlatformInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = PlatformP__Init__init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 36 "/opt/tinyos-2.x/tos/platforms/telosb/hardware.h"
static inline  void TOSH_CLR_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r &= ~(1 << 1);
}

# 54 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
inline static bool RealMainP__Scheduler__runNextTask(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = SchedulerBasicP__Scheduler__runNextTask();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 79 "/opt/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc"
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__bufferDone(uint8_t client, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t *buf, uint16_t count)
{
}

# 89 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
inline static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__bufferDone(uint8_t arg_0xf657f258, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__val_t * buf, uint16_t count){
#line 89
    /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__bufferDone(arg_0xf657f258, result, buf, count);
#line 89
}
#line 89
# 48 "/opt/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc"
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__bufferDone(uint8_t client, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t *buf, uint16_t count)
{
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__bufferDone(client, result, buf, count);
}

# 89 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
inline static void AdcStreamP__ReadStream__bufferDone(uint8_t arg_0xf65c3690, error_t result, AdcStreamP__ReadStream__val_t * buf, uint16_t count){
#line 89
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__bufferDone(arg_0xf65c3690, result, buf, count);
#line 89
}
#line 89
# 156 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline void AdcStreamP__bufferDone__runTask(void )
#line 156
{
  uint16_t *b;
#line 157
  uint16_t c;

#line 158
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      b = AdcStreamP__lastBuffer;
      c = AdcStreamP__lastCount;
      AdcStreamP__lastBuffer = (void *)0;
    }
#line 163
    __nesc_atomic_end(__nesc_atomic); }

  AdcStreamP__ReadStream__bufferDone(AdcStreamP__client, SUCCESS, b, c);
}

# 83 "/opt/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc"
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__readDone(uint8_t client, error_t result, uint32_t actualPeriod)
{
}

# 102 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
inline static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__readDone(uint8_t arg_0xf657f258, error_t result, uint32_t usActualPeriod){
#line 102
    /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__readDone(arg_0xf657f258, result, usActualPeriod);
#line 102
}
#line 102
# 67 "/opt/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc"
static inline error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__default__release(uint8_t client)
#line 67
{
#line 67
  return FAIL;
}

# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__release(uint8_t arg_0xf657b678){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  switch (arg_0xf657b678) {
#line 110
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT:
#line 110
      __nesc_result = Msp430RefVoltArbiterImplP__ClientResource__release(/*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID);
#line 110
      break;
#line 110
    default:
#line 110
      __nesc_result = /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__default__release(arg_0xf657b678);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 53 "/opt/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc"
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__readDone(uint8_t client, error_t result, uint32_t actualPeriod)
{
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__release(client);
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__readDone(client, result, actualPeriod);
}

# 102 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
inline static void AdcStreamP__ReadStream__readDone(uint8_t arg_0xf65c3690, error_t result, uint32_t usActualPeriod){
#line 102
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__readDone(arg_0xf65c3690, result, usActualPeriod);
#line 102
}
#line 102
# 135 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline void AdcStreamP__readStreamFail__runTask(void )
#line 135
{

  struct AdcStreamP__list_entry_t *entry;
  uint8_t c = AdcStreamP__client;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 140
    entry = AdcStreamP__bufferQueue[c];
#line 140
    __nesc_atomic_end(__nesc_atomic); }
  for (; entry; entry = entry->next) {
      uint16_t tmp_count __attribute((unused))  = entry->count;

#line 143
      AdcStreamP__ReadStream__bufferDone(c, FAIL, (uint16_t * )entry, entry->count);
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      AdcStreamP__bufferQueue[c] = (void *)0;
      AdcStreamP__bufferQueueEnd[c] = &AdcStreamP__bufferQueue[c];
    }
#line 150
    __nesc_atomic_end(__nesc_atomic); }

  AdcStreamP__client = AdcStreamP__NSTREAM;
  AdcStreamP__ReadStream__readDone(c, FAIL, 0);
}

# 170 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline error_t Msp430RefVoltArbiterImplP__AdcResource__default__release(uint8_t client)
#line 170
{
#line 170
  return FAIL;
}

# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t Msp430RefVoltArbiterImplP__AdcResource__release(uint8_t arg_0xf66073a0){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  switch (arg_0xf66073a0) {
#line 110
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID:
#line 110
      __nesc_result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__release(/*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID);
#line 110
      break;
#line 110
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID:
#line 110
      __nesc_result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__release(/*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID);
#line 110
      break;
#line 110
    default:
#line 110
      __nesc_result = Msp430RefVoltArbiterImplP__AdcResource__default__release(arg_0xf66073a0);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 56 "/opt/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc"
static inline bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEmpty(void )
#line 56
{
  int i;

  /* atomic removed: atomic calls only */
#line 58
  {
    for (i = 0; i < sizeof /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ; i++) 
      if (/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ[i] > 0) {
          unsigned char __nesc_temp = 
#line 60
          FALSE;

#line 60
          return __nesc_temp;
        }
#line 61
    {
      unsigned char __nesc_temp = 
#line 61
      TRUE;

#line 61
      return __nesc_temp;
    }
  }
}

# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__isEmpty(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEmpty();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 47 "/opt/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc"
static inline void /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__clearEntry(uint8_t id)
#line 47
{
  /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ[id / 8] &= ~(1 << id % 8);
}

#line 69
static inline resource_client_id_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__dequeue(void )
#line 69
{
  int i;

  /* atomic removed: atomic calls only */
#line 71
  {
    for (i = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__last + 1; ; i++) {
        if (i == 2U) {
          i = 0;
          }
#line 75
        if (/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEnqueued(i)) {
            /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__clearEntry(i);
            /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__last = i;
            {
              unsigned char __nesc_temp = 
#line 78
              i;

#line 78
              return __nesc_temp;
            }
          }
#line 80
        if (i == /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__last) {
          break;
          }
      }
#line 83
    {
      unsigned char __nesc_temp = 
#line 83
      /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__NO_ENTRY;

#line 83
      return __nesc_temp;
    }
  }
}

# 60 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__dequeue(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__dequeue();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 173 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id)
#line 173
{
}

# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__unconfigure(uint8_t arg_0xf6666898){
#line 55
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__unconfigure(arg_0xf6666898);
#line 55
}
#line 55
# 119 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline void AdcStreamP__readStreamDone__runTask(void )
#line 119
{
  uint8_t c = AdcStreamP__client;
  uint32_t actualPeriod = AdcStreamP__period;

#line 122
  if (AdcStreamP__periodModified) {
    actualPeriod = AdcStreamP__period - AdcStreamP__period % 1000;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      AdcStreamP__bufferQueue[c] = (void *)0;
      AdcStreamP__bufferQueueEnd[c] = &AdcStreamP__bufferQueue[c];
    }
#line 129
    __nesc_atomic_end(__nesc_atomic); }

  AdcStreamP__client = AdcStreamP__NSTREAM;
  AdcStreamP__ReadStream__readDone(c, SUCCESS, actualPeriod);
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t Msp430RefVoltArbiterImplP__switchOff__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(Msp430RefVoltArbiterImplP__switchOff);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 153 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num].isrunning = FALSE;
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void Msp430RefVoltGeneratorP__SwitchOnTimer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(10U);
#line 67
}
#line 67
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__get(void ){
#line 53
  unsigned long __nesc_result;
#line 53

#line 53
  __nesc_result = /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 75 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__get();
}

# 98 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void ){
#line 98
  unsigned long __nesc_result;
#line 98

#line 98
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getNow();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 85 "/opt/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void )
{
#line 86
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow();
}

# 125 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void ){
#line 125
  unsigned long __nesc_result;
#line 125

#line 125
  __nesc_result = /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow();
#line 125

#line 125
  return __nesc_result;
#line 125
}
#line 125
# 148 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, TRUE);
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void Msp430RefVoltGeneratorP__SwitchOffTimer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(11U, dt);
#line 62
}
#line 62
# 126 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline error_t Msp430RefVoltGeneratorP__stop(Msp430RefVoltGeneratorP__state_t nextState)
#line 126
{
  error_t result;

  if (Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__GENERATOR_OFF) {
    result = EALREADY;
    }
  else {
#line 131
    if (Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE || Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_2_5V_STABLE) {
        if ((result = Msp430RefVoltGeneratorP__switchOff()) == SUCCESS) {
            Msp430RefVoltGeneratorP__m_state = nextState;
            Msp430RefVoltGeneratorP__SwitchOffTimer__startOneShot(20);
          }
      }
    else {
#line 136
      if (Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_ON_PENDING || Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_2_5V_ON_PENDING) {
          if ((result = Msp430RefVoltGeneratorP__switchOff()) == SUCCESS) {

              Msp430RefVoltGeneratorP__state_t oldState = Msp430RefVoltGeneratorP__m_state;

#line 140
              Msp430RefVoltGeneratorP__SwitchOnTimer__stop();
              Msp430RefVoltGeneratorP__m_state = Msp430RefVoltGeneratorP__GENERATOR_OFF;
              Msp430RefVoltGeneratorP__signalStartDone(oldState, FAIL);
              Msp430RefVoltGeneratorP__signalStopDone(nextState, SUCCESS);
            }
        }
      else {
#line 145
        if (Msp430RefVoltGeneratorP__m_state == nextState) {
          result = SUCCESS;
          }
        else {
#line 148
          result = EBUSY;
          }
        }
      }
    }
#line 150
  return result;
}

#line 86
static inline error_t Msp430RefVoltGeneratorP__RefVolt_1_5V__stop(void )
#line 86
{
  return Msp430RefVoltGeneratorP__stop(Msp430RefVoltGeneratorP__REFERENCE_1_5V_OFF_PENDING);
}

# 109 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static error_t Msp430RefVoltArbiterImplP__RefVolt_1_5V__stop(void ){
#line 109
  unsigned char __nesc_result;
#line 109

#line 109
  __nesc_result = Msp430RefVoltGeneratorP__RefVolt_1_5V__stop();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 136 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline void Msp430RefVoltArbiterImplP__switchOff__runTask(void )
{

  if (Msp430RefVoltArbiterImplP__syncOwner != Msp430RefVoltArbiterImplP__NO_OWNER) {
      if (Msp430RefVoltArbiterImplP__RefVolt_1_5V__stop() == SUCCESS) {
          Msp430RefVoltArbiterImplP__syncOwner = Msp430RefVoltArbiterImplP__NO_OWNER;
        }
      else {
#line 143
        Msp430RefVoltArbiterImplP__switchOff__postTask();
        }
    }
}

# 118 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline bool HplAdc12P__HplAdc12__isBusy(void )
#line 118
{
#line 118
  return HplAdc12P__ADC12CTL1 & 0x0001;
}

# 118 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static bool Msp430RefVoltGeneratorP__HplAdc12__isBusy(void ){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  __nesc_result = HplAdc12P__HplAdc12__isBusy();
#line 118

#line 118
  return __nesc_result;
#line 118
}
#line 118
# 57 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline  adc12ctl0_t HplAdc12P__int2adc12ctl0(uint16_t x)
#line 57
{
#line 57
  union __nesc_unnamed4410 {
#line 57
    uint16_t f;
#line 57
    adc12ctl0_t t;
  } 
#line 57
  c = { .f = x };

#line 57
  return c.t;
}

#line 72
static inline adc12ctl0_t HplAdc12P__HplAdc12__getCtl0(void )
#line 72
{
  return HplAdc12P__int2adc12ctl0(HplAdc12P__ADC12CTL0);
}

# 63 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static adc12ctl0_t Msp430RefVoltGeneratorP__HplAdc12__getCtl0(void ){
#line 63
  struct __nesc_unnamed4254 __nesc_result;
#line 63

#line 63
  __nesc_result = HplAdc12P__HplAdc12__getCtl0();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 59 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline  uint16_t HplAdc12P__adc12ctl0cast2int(adc12ctl0_t x)
#line 59
{
#line 59
  union __nesc_unnamed4411 {
#line 59
    adc12ctl0_t f;
#line 59
    uint16_t t;
  } 
#line 59
  c = { .f = x };

#line 59
  return c.t;
}



static inline void HplAdc12P__HplAdc12__setCtl0(adc12ctl0_t control0)
#line 64
{
  HplAdc12P__ADC12CTL0 = HplAdc12P__adc12ctl0cast2int(control0);
}

# 51 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430RefVoltGeneratorP__HplAdc12__setCtl0(adc12ctl0_t control0){
#line 51
  HplAdc12P__HplAdc12__setCtl0(control0);
#line 51
}
#line 51
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Counter__size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Counter__get(void ){
#line 53
  unsigned long __nesc_result;
#line 53

#line 53
  __nesc_result = /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 75 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Alarm__getNow(void )
{
  return /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Counter__get();
}

# 98 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static AdcStreamP__Alarm__size_type AdcStreamP__Alarm__getNow(void ){
#line 98
  unsigned long __nesc_result;
#line 98

#line 98
  __nesc_result = /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Alarm__getNow();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 328 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline error_t AdcStreamP__SingleChannel__default__configureSingle(uint8_t c, 
const msp430adc12_channel_config_t *config)
#line 329
{
#line 329
  return FAIL;
}

# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t AdcStreamP__SingleChannel__configureSingle(uint8_t arg_0xf65c1170, const msp430adc12_channel_config_t * config){
#line 84
  unsigned char __nesc_result;
#line 84

#line 84
  switch (arg_0xf65c1170) {
#line 84
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT:
#line 84
      __nesc_result = Msp430Adc12ImplP__SingleChannel__configureSingle(/*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID, config);
#line 84
      break;
#line 84
    default:
#line 84
      __nesc_result = AdcStreamP__SingleChannel__default__configureSingle(arg_0xf65c1170, config);
#line 84
      break;
#line 84
    }
#line 84

#line 84
  return __nesc_result;
#line 84
}
#line 84
# 61 "/opt/tinyos-2.x/tos/platforms/telosa/chips/s10871/HamamatsuS10871TsrP.nc"
static inline const msp430adc12_channel_config_t *HamamatsuS10871TsrP__AdcConfigure__getConfiguration(void )
#line 61
{
  return &HamamatsuS10871TsrP__config;
}

# 314 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline const msp430adc12_channel_config_t *AdcStreamP__AdcConfigure__default__getConfiguration(uint8_t c)
{
  return &AdcStreamP__defaultConfig;
}

# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
inline static AdcStreamP__AdcConfigure__adc_config_t AdcStreamP__AdcConfigure__getConfiguration(uint8_t arg_0xf65bf010){
#line 58
  struct __nesc_unnamed4301 const *__nesc_result;
#line 58

#line 58
  switch (arg_0xf65bf010) {
#line 58
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT:
#line 58
      __nesc_result = HamamatsuS10871TsrP__AdcConfigure__getConfiguration();
#line 58
      break;
#line 58
    default:
#line 58
      __nesc_result = AdcStreamP__AdcConfigure__default__getConfiguration(arg_0xf65bf010);
#line 58
      break;
#line 58
    }
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t AdcStreamP__readStreamDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(AdcStreamP__readStreamDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 136 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Alarm__startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__to_size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__to_size_type dt)
{
  /* atomic removed: atomic calls only */
  {
    /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__m_t0 = t0;
    /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__m_dt = dt;
    /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__set_alarm();
  }
}

# 92 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void AdcStreamP__Alarm__startAt(AdcStreamP__Alarm__size_type t0, AdcStreamP__Alarm__size_type dt){
#line 92
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 168 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline void AdcStreamP__nextAlarm(void )
#line 168
{
  AdcStreamP__Alarm__startAt(AdcStreamP__now, AdcStreamP__period);
  AdcStreamP__now += AdcStreamP__period;
}

# 144 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )372U = x;
}

# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void Msp430Adc12ImplP__CompareA1__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__setEvent(time);
#line 30
}
#line 30
# 144 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )370U = x;
}

# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void Msp430Adc12ImplP__CompareA0__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(time);
#line 30
}
#line 30
# 46 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__CC2int(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4412 {
#line 46
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 89
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControl(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t x)
{
  * (volatile uint16_t * )354U = /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__CC2int(x);
}

# 35 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void Msp430Adc12ImplP__ControlA0__setControl(msp430_compare_control_t control){
#line 35
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControl(control);
#line 35
}
#line 35
# 110 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setInputDivider(uint16_t inputDivider)
{
  * (volatile uint16_t * )352U = (* (volatile uint16_t * )352U & ~(0x0040 | 0x0080)) | ((inputDivider << 6) & (0x0040 | 0x0080));
}

# 45 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void Msp430Adc12ImplP__TimerA__setInputDivider(uint16_t inputDivider){
#line 45
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setInputDivider(inputDivider);
#line 45
}
#line 45
# 105 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setClockSource(uint16_t clockSource)
{
  * (volatile uint16_t * )352U = (* (volatile uint16_t * )352U & ~(256U | 512U)) | ((clockSource << 8) & (256U | 512U));
}

# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void Msp430Adc12ImplP__TimerA__setClockSource(uint16_t clockSource){
#line 44
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setClockSource(clockSource);
#line 44
}
#line 44
# 100 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__disableEvents(void )
{
  * (volatile uint16_t * )352U &= ~2U;
}

# 43 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void Msp430Adc12ImplP__TimerA__disableEvents(void ){
#line 43
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__disableEvents();
#line 43
}
#line 43
# 90 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__clear(void )
{
  * (volatile uint16_t * )352U |= 4U;
}

# 41 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void Msp430Adc12ImplP__TimerA__clear(void ){
#line 41
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__clear();
#line 41
}
#line 41
# 103 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__prepareTimerA(uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON)
{

  msp430_compare_control_t ccResetSHI = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };

  Msp430Adc12ImplP__TimerA__setMode(MSP430TIMER_STOP_MODE);
  Msp430Adc12ImplP__TimerA__clear();
  Msp430Adc12ImplP__TimerA__disableEvents();
  Msp430Adc12ImplP__TimerA__setClockSource(csSAMPCON);
  Msp430Adc12ImplP__TimerA__setInputDivider(cdSAMPCON);
  Msp430Adc12ImplP__ControlA0__setControl(ccResetSHI);
  Msp430Adc12ImplP__CompareA0__setEvent(interval - 1);
  Msp430Adc12ImplP__CompareA1__setEvent((interval - 1) / 2);
}

# 92 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline void HplAdc12P__HplAdc12__setIEFlags(uint16_t mask)
#line 92
{
#line 92
  HplAdc12P__ADC12IE = mask;
}

# 95 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP__HplAdc12__setIEFlags(uint16_t mask){
#line 95
  HplAdc12P__HplAdc12__setIEFlags(mask);
#line 95
}
#line 95
# 61 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline  uint8_t HplAdc12P__adc12memctl2int(adc12memctl_t x)
#line 61
{
#line 61
  union __nesc_unnamed4413 {
#line 61
    adc12memctl_t f;
#line 61
    uint8_t t;
  } 
#line 61
  c = { .f = x };

#line 61
  return c.t;
}

#line 80
static inline void HplAdc12P__HplAdc12__setMCtl(uint8_t i, adc12memctl_t memCtl)
#line 80
{
  ((char *)0x0080)[i] = HplAdc12P__adc12memctl2int(memCtl);
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP__HplAdc12__setMCtl(uint8_t idx, adc12memctl_t memControl){
#line 75
  HplAdc12P__HplAdc12__setMCtl(idx, memControl);
#line 75
}
#line 75
# 60 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline  uint16_t HplAdc12P__adc12ctl1cast2int(adc12ctl1_t x)
#line 60
{
#line 60
  union __nesc_unnamed4414 {
#line 60
    adc12ctl1_t f;
#line 60
    uint16_t t;
  } 
#line 60
  c = { .f = x };

#line 60
  return c.t;
}






static inline void HplAdc12P__HplAdc12__setCtl1(adc12ctl1_t control1)
#line 68
{
  HplAdc12P__ADC12CTL1 = HplAdc12P__adc12ctl1cast2int(control1);
}

# 57 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP__HplAdc12__setCtl1(adc12ctl1_t control1){
#line 57
  HplAdc12P__HplAdc12__setCtl1(control1);
#line 57
}
#line 57
#line 51
inline static void Msp430Adc12ImplP__HplAdc12__setCtl0(adc12ctl0_t control0){
#line 51
  HplAdc12P__HplAdc12__setCtl0(control0);
#line 51
}
#line 51
#line 63
inline static adc12ctl0_t Msp430Adc12ImplP__HplAdc12__getCtl0(void ){
#line 63
  struct __nesc_unnamed4254 __nesc_result;
#line 63

#line 63
  __nesc_result = HplAdc12P__HplAdc12__getCtl0();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 88 "/opt/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t Msp430Adc12ImplP__ADCArbiterInfo__userId(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ArbiterInfo__userId();
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 271 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline error_t Msp430Adc12ImplP__SingleChannel__configureMultiple(uint8_t id, 
const msp430adc12_channel_config_t *config, 
uint16_t *buf, uint16_t length, uint16_t jiffies)
{
  error_t result = ERESERVE;





  if ((((!config || !buf) || !length) || jiffies == 1) || jiffies == 2) {
    return EINVAL;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 284
    {
      if (Msp430Adc12ImplP__state & Msp430Adc12ImplP__ADC_BUSY) 
        {
          unsigned char __nesc_temp = 
#line 286
          EBUSY;

          {
#line 286
            __nesc_atomic_end(__nesc_atomic); 
#line 286
            return __nesc_temp;
          }
        }
#line 287
      if (Msp430Adc12ImplP__ADCArbiterInfo__userId() == id) {
          adc12ctl1_t ctl1 = { 
          .adc12busy = 0, 
          .conseq = length > 16 ? 3 : 1, 
          .adc12ssel = config->adc12ssel, 
          .adc12div = config->adc12div, 
          .issh = 0, 
          .shp = 1, 
          .shs = jiffies == 0 ? 0 : 1, 
          .cstartadd = 0 };

          adc12memctl_t memctl = { 
          .inch = config->inch, 
          .sref = config->sref, 
          .eos = 0 };

          uint16_t i;
#line 303
          uint16_t mask = 1;
          adc12ctl0_t ctl0 = Msp430Adc12ImplP__HplAdc12__getCtl0();

#line 305
          ctl0.msc = jiffies == 0 ? 1 : 0;
          ctl0.sht0 = config->sht;
          ctl0.sht1 = config->sht;

          Msp430Adc12ImplP__state = Msp430Adc12ImplP__MULTIPLE_DATA;
          Msp430Adc12ImplP__resultBufferStart = (void *)0;
          Msp430Adc12ImplP__resultBufferLength = length;
          Msp430Adc12ImplP__resultBufferStart = buf;
          Msp430Adc12ImplP__resultBufferIndex = 0;
          Msp430Adc12ImplP__HplAdc12__setCtl0(ctl0);
          Msp430Adc12ImplP__HplAdc12__setCtl1(ctl1);
          for (i = 0; i < length - 1 && i < 15; i++) 
            Msp430Adc12ImplP__HplAdc12__setMCtl(i, memctl);
          memctl.eos = 1;
          Msp430Adc12ImplP__HplAdc12__setMCtl(i, memctl);
          Msp430Adc12ImplP__HplAdc12__setIEFlags(mask << i);

          if (jiffies) {
              Msp430Adc12ImplP__state |= Msp430Adc12ImplP__USE_TIMERA;
              Msp430Adc12ImplP__prepareTimerA(jiffies, config->sampcon_ssel, config->sampcon_id);
            }
          result = SUCCESS;
        }
    }
#line 328
    __nesc_atomic_end(__nesc_atomic); }
  return result;
}

# 318 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline error_t AdcStreamP__SingleChannel__default__configureMultiple(uint8_t c, 
const msp430adc12_channel_config_t *config, uint16_t b[], 
uint16_t numSamples, uint16_t jiffies)
{
  return FAIL;
}

# 138 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t AdcStreamP__SingleChannel__configureMultiple(uint8_t arg_0xf65c1170, const msp430adc12_channel_config_t * config, uint16_t * buffer, uint16_t numSamples, uint16_t jiffies){
#line 138
  unsigned char __nesc_result;
#line 138

#line 138
  switch (arg_0xf65c1170) {
#line 138
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT:
#line 138
      __nesc_result = Msp430Adc12ImplP__SingleChannel__configureMultiple(/*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID, config, buffer, numSamples, jiffies);
#line 138
      break;
#line 138
    default:
#line 138
      __nesc_result = AdcStreamP__SingleChannel__default__configureMultiple(arg_0xf65c1170, config, buffer, numSamples, jiffies);
#line 138
      break;
#line 138
    }
#line 138

#line 138
  return __nesc_result;
#line 138
}
#line 138
# 96 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline error_t AdcStreamP__postBuffer(uint8_t c, uint16_t *buf, uint16_t n)
{
  if (n < sizeof(struct AdcStreamP__list_entry_t )) {
    return ESIZE;
    }
#line 100
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      struct AdcStreamP__list_entry_t * newEntry = (struct AdcStreamP__list_entry_t * )buf;

      if (!AdcStreamP__bufferQueueEnd[c]) 
        {
          unsigned char __nesc_temp = 
#line 105
          FAIL;

          {
#line 105
            __nesc_atomic_end(__nesc_atomic); 
#line 105
            return __nesc_temp;
          }
        }
#line 107
      newEntry->count = n;
      newEntry->next = (void *)0;
      *AdcStreamP__bufferQueueEnd[c] = newEntry;
      AdcStreamP__bufferQueueEnd[c] = & newEntry->next;
    }
#line 111
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t AdcStreamP__readStreamFail__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(AdcStreamP__readStreamFail);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 191 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline error_t AdcP__SingleChannel__default__configureSingle(uint8_t client, 
const msp430adc12_channel_config_t *config)
#line 192
{
#line 192
  return FAIL;
}

# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t AdcP__SingleChannel__configureSingle(uint8_t arg_0xf6758e40, const msp430adc12_channel_config_t * config){
#line 84
  unsigned char __nesc_result;
#line 84

#line 84
  switch (arg_0xf6758e40) {
#line 84
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC*/AdcReadClientC__0__CLIENT:
#line 84
      __nesc_result = Msp430Adc12ImplP__SingleChannel__configureSingle(/*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID, config);
#line 84
      break;
#line 84
    default:
#line 84
      __nesc_result = AdcP__SingleChannel__default__configureSingle(arg_0xf6758e40, config);
#line 84
      break;
#line 84
    }
#line 84

#line 84
  return __nesc_result;
#line 84
}
#line 84
# 186 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline const msp430adc12_channel_config_t *
AdcP__Config__default__getConfiguration(uint8_t client)
{
  return &AdcP__defaultConfig;
}

# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
inline static AdcP__Config__adc_config_t AdcP__Config__getConfiguration(uint8_t arg_0xf6766df0){
#line 58
  struct __nesc_unnamed4301 const *__nesc_result;
#line 58

#line 58
  switch (arg_0xf6766df0) {
#line 58
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC*/AdcReadClientC__0__CLIENT:
#line 58
      __nesc_result = HamamatsuS10871TsrP__AdcConfigure__getConfiguration();
#line 58
      break;
#line 58
    default:
#line 58
      __nesc_result = AdcP__Config__default__getConfiguration(arg_0xf6766df0);
#line 58
      break;
#line 58
    }
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 65 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline error_t AdcP__configure(uint8_t client)
{
  error_t result = EINVAL;
  const msp430adc12_channel_config_t * config;

#line 69
  config = AdcP__Config__getConfiguration(client);
  if (config->inch != INPUT_CHANNEL_NONE) {
    result = AdcP__SingleChannel__configureSingle(client, config);
    }
#line 72
  return result;
}

#line 180
static inline error_t AdcP__SingleChannel__default__getData(uint8_t client)
{
  return EINVAL;
}

# 189 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t AdcP__SingleChannel__getData(uint8_t arg_0xf6758e40){
#line 189
  unsigned char __nesc_result;
#line 189

#line 189
  switch (arg_0xf6758e40) {
#line 189
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC*/AdcReadClientC__0__CLIENT:
#line 189
      __nesc_result = Msp430Adc12ImplP__SingleChannel__getData(/*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID);
#line 189
      break;
#line 189
    default:
#line 189
      __nesc_result = AdcP__SingleChannel__default__getData(arg_0xf6758e40);
#line 189
      break;
#line 189
    }
#line 189

#line 189
  return __nesc_result;
#line 189
}
#line 189
# 165 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__default__granted(uint8_t id)
#line 165
{
}

# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__granted(uint8_t arg_0xf66671b8){
#line 92
  switch (arg_0xf66671b8) {
#line 92
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID:
#line 92
      Msp430RefVoltArbiterImplP__AdcResource__granted(/*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID);
#line 92
      break;
#line 92
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID:
#line 92
      Msp430RefVoltArbiterImplP__AdcResource__granted(/*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID);
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__default__granted(arg_0xf66671b8);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 171 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 171
{
}

# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__configure(uint8_t arg_0xf6666898){
#line 49
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(arg_0xf6666898);
#line 49
}
#line 49
# 155 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__runTask(void )
#line 155
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 156
    {
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__reqResId;
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_BUSY;
    }
#line 159
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__configure(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId);
  /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__granted(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId);
}

# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
inline static /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfUp__adc_config_t /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfUp__getConfiguration(void ){
#line 58
  struct __nesc_unnamed4301 const *__nesc_result;
#line 58

#line 58
  __nesc_result = HamamatsuS10871TsrP__AdcConfigure__getConfiguration();
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 47 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline const msp430adc12_channel_config_t */*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfSub__getConfiguration(void )
{
  return /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfUp__getConfiguration();
}

# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
inline static /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfUp__adc_config_t /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfUp__getConfiguration(void ){
#line 58
  struct __nesc_unnamed4301 const *__nesc_result;
#line 58

#line 58
  __nesc_result = HamamatsuS10871TsrP__AdcConfigure__getConfiguration();
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 47 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline const msp430adc12_channel_config_t */*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfSub__getConfiguration(void )
{
  return /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfUp__getConfiguration();
}

# 172 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline const msp430adc12_channel_config_t *
Msp430RefVoltArbiterImplP__Config__default__getConfiguration(uint8_t client)
{
  return &Msp430RefVoltArbiterImplP__defaultConfig;
}

# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
inline static Msp430RefVoltArbiterImplP__Config__adc_config_t Msp430RefVoltArbiterImplP__Config__getConfiguration(uint8_t arg_0xf6606df0){
#line 58
  struct __nesc_unnamed4301 const *__nesc_result;
#line 58

#line 58
  switch (arg_0xf6606df0) {
#line 58
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID:
#line 58
      __nesc_result = /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfSub__getConfiguration();
#line 58
      break;
#line 58
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID:
#line 58
      __nesc_result = /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfSub__getConfiguration();
#line 58
      break;
#line 58
    default:
#line 58
      __nesc_result = Msp430RefVoltArbiterImplP__Config__default__getConfiguration(arg_0xf6606df0);
#line 58
      break;
#line 58
    }
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 167 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(uint8_t id)
#line 167
{
}

# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceRequested__requested(uint8_t arg_0xf6667be0){
#line 43
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(arg_0xf6667be0);
#line 43
}
#line 43
# 87 "/opt/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc"
static inline error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__enqueue(resource_client_id_t id)
#line 87
{
  /* atomic removed: atomic calls only */
#line 88
  {
    if (!/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEnqueued(id)) {
        /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ[id / 8] |= 1 << id % 8;
        {
          unsigned char __nesc_temp = 
#line 91
          SUCCESS;

#line 91
          return __nesc_temp;
        }
      }
#line 93
    {
      unsigned char __nesc_temp = 
#line 93
      EBUSY;

#line 93
      return __nesc_temp;
    }
  }
}

# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__enqueue(resource_client_id_t id){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__enqueue(id);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 78 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline error_t Msp430RefVoltGeneratorP__RefVolt_1_5V__start(void )
#line 78
{
  return Msp430RefVoltGeneratorP__start(Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE);
}

# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static error_t Msp430RefVoltArbiterImplP__RefVolt_1_5V__start(void ){
#line 83
  unsigned char __nesc_result;
#line 83

#line 83
  __nesc_result = Msp430RefVoltGeneratorP__RefVolt_1_5V__start();
#line 83

#line 83
  return __nesc_result;
#line 83
}
#line 83
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void Msp430RefVoltGeneratorP__SwitchOnTimer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(10U, dt);
#line 62
}
#line 62





inline static void Msp430RefVoltGeneratorP__SwitchOffTimer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(11U);
#line 67
}
#line 67
# 82 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline error_t Msp430RefVoltGeneratorP__RefVolt_2_5V__start(void )
#line 82
{
  return Msp430RefVoltGeneratorP__start(Msp430RefVoltGeneratorP__REFERENCE_2_5V_STABLE);
}

# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static error_t Msp430RefVoltArbiterImplP__RefVolt_2_5V__start(void ){
#line 83
  unsigned char __nesc_result;
#line 83

#line 83
  __nesc_result = Msp430RefVoltGeneratorP__RefVolt_2_5V__start();
#line 83

#line 83
  return __nesc_result;
#line 83
}
#line 83
# 172 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline void AdcP__Read__default__readDone(uint8_t client, error_t result, uint16_t val)
#line 172
{
}

# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
inline static void AdcP__Read__readDone(uint8_t arg_0xf676c668, error_t result, AdcP__Read__val_t val){
#line 63
  switch (arg_0xf676c668) {
#line 63
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC*/AdcReadClientC__0__CLIENT:
#line 63
      SendingMoteC__Light__readDone(result, val);
#line 63
      break;
#line 63
    default:
#line 63
      AdcP__Read__default__readDone(arg_0xf676c668, result, val);
#line 63
      break;
#line 63
    }
#line 63
}
#line 63
# 170 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline error_t AdcP__ResourceRead__default__release(uint8_t client)
#line 170
{
#line 170
  return FAIL;
}

# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t AdcP__ResourceRead__release(uint8_t arg_0xf6769670){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  switch (arg_0xf6769670) {
#line 110
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC*/AdcReadClientC__0__CLIENT:
#line 110
      __nesc_result = Msp430RefVoltArbiterImplP__ClientResource__release(/*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID);
#line 110
      break;
#line 110
    default:
#line 110
      __nesc_result = AdcP__ResourceRead__default__release(arg_0xf6769670);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 136 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline void AdcP__readDone__runTask(void )
{
  AdcP__ResourceRead__release(AdcP__owner);
  AdcP__Read__readDone(AdcP__owner, SUCCESS, AdcP__value);
}

# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__start(void ){
#line 83
  unsigned char __nesc_result;
#line 83

#line 83
  __nesc_result = HplSensirionSht11P__SplitControl__start();
#line 83

#line 83
  return __nesc_result;
#line 83
}
#line 83
# 84 "/opt/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__default__start(void )
#line 84
{
  return SUCCESS;
}

# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
inline static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__start(void ){
#line 74
  unsigned char __nesc_result;
#line 74

#line 74
  __nesc_result = /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__default__start();
#line 74

#line 74
  return __nesc_result;
#line 74
}
#line 74
# 63 "/opt/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__startTask__runTask(void )
#line 63
{
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__start();
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__start();
}

# 52 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__makeOutput(void )
#line 52
{
#line 52
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 52
    * (volatile uint8_t * )34U |= 0x01 << 7;
#line 52
    __nesc_atomic_end(__nesc_atomic); }
}

# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__HplGeneralIO__makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void HplSensirionSht11P__PWR__makeOutput(void ){
#line 35
  /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__makeOutput();
#line 35
}
#line 35
# 45 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )33U |= 0x01 << 7;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__HplGeneralIO__set();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void HplSensirionSht11P__PWR__set(void ){
#line 29
  /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__set();
#line 29
}
#line 29
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void HplSensirionSht11P__Timer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(8U, dt);
#line 62
}
#line 62
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t HplSensirionSht11P__stopTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(HplSensirionSht11P__stopTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 46 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )33U &= ~(0x01 << 7);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__clr(void )
#line 38
{
#line 38
  /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__HplGeneralIO__clr();
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void HplSensirionSht11P__PWR__clr(void ){
#line 30
  /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__clr();
#line 30
}
#line 30
# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__clr(void )
#line 38
{
#line 38
  /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__clr();
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void HplSensirionSht11P__DATA__clr(void ){
#line 30
  /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__clr();
#line 30
}
#line 30
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__makeInput();
#line 64
}
#line 64
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__makeInput(void )
#line 41
{
#line 41
  /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__makeInput();
}

# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void HplSensirionSht11P__DATA__makeInput(void ){
#line 33
  /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__makeInput();
#line 33
}
#line 33
# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__clr(void )
#line 38
{
#line 38
  /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__clr();
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void HplSensirionSht11P__SCK__clr(void ){
#line 30
  /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__clr();
#line 30
}
#line 30
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__makeInput(void )
#line 50
{
#line 50
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 50
    * (volatile uint8_t * )34U &= ~(0x01 << 6);
#line 50
    __nesc_atomic_end(__nesc_atomic); }
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__makeInput();
#line 64
}
#line 64
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__makeInput(void )
#line 41
{
#line 41
  /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__makeInput();
}

# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void HplSensirionSht11P__SCK__makeInput(void ){
#line 33
  /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__makeInput();
#line 33
}
#line 33
# 63 "/opt/tinyos-2.x/tos/platforms/telosa/chips/sht11/HplSensirionSht11P.nc"
static inline error_t HplSensirionSht11P__SplitControl__stop(void )
#line 63
{
  HplSensirionSht11P__SCK__makeInput();
  HplSensirionSht11P__SCK__clr();
  HplSensirionSht11P__DATA__makeInput();
  HplSensirionSht11P__DATA__clr();
  HplSensirionSht11P__PWR__clr();
  HplSensirionSht11P__stopTask__postTask();
  return SUCCESS;
}

# 109 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__stop(void ){
#line 109
  unsigned char __nesc_result;
#line 109

#line 109
  __nesc_result = HplSensirionSht11P__SplitControl__stop();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 113 "/opt/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__default__stop(void )
#line 113
{
  return SUCCESS;
}

# 84 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
inline static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__stop(void ){
#line 84
  unsigned char __nesc_result;
#line 84

#line 84
  __nesc_result = /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__default__stop();
#line 84

#line 84
  return __nesc_result;
#line 84
}
#line 84
# 121 "/opt/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__PowerDownCleanup__default__cleanup(void )
#line 121
{
}

# 52 "/opt/tinyos-2.x/tos/lib/power/PowerDownCleanup.nc"
inline static void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__PowerDownCleanup__cleanup(void ){
#line 52
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__PowerDownCleanup__default__cleanup();
#line 52
}
#line 52
# 68 "/opt/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopTask__runTask(void )
#line 68
{
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__PowerDownCleanup__cleanup();
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__stop();
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__stop();
}

# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
inline static void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__readDone(error_t result, /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__val_t val){
#line 63
  SendingMoteC__Temperature__readDone(result, val);
#line 63
}
#line 63
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__release(/*SendingMoteAppC.SensirionSht11C*/SensirionSht11C__0__TEMP_KEY);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 113 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureTemperature(uint8_t client)
#line 113
{
  if (!/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__on) {
#line 114
      return EOFF;
    }
#line 115
  if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy) {
#line 115
      return EBUSY;
    }
  else 
#line 115
    {
#line 115
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = TRUE;
    }
#line 116
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__cmd = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_MEASURE_TEMPERATURE;
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__currentClient = client;
  return /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__performCommand();
}

# 61 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
inline static error_t /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureTemperature(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureTemperature(/*SendingMoteAppC.SensirionSht11C*/SensirionSht11C__0__TEMP_KEY);
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 65 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__granted(void )
#line 65
{
  error_t result;

#line 67
  if ((result = /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureTemperature()) != SUCCESS) {
      /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__release();
      /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__readDone(result, 0);
    }
}

# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
inline static void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__readDone(error_t result, /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__val_t val){
#line 63
  SendingMoteC__Humidity__readDone(result, val);
#line 63
}
#line 63
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__HumResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__release(/*SendingMoteAppC.SensirionSht11C*/SensirionSht11C__0__HUM_KEY);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 121 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureHumidity(uint8_t client)
#line 121
{
  if (!/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__on) {
#line 122
      return EOFF;
    }
#line 123
  if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy) {
#line 123
      return EBUSY;
    }
  else 
#line 123
    {
#line 123
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = TRUE;
    }
#line 124
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__cmd = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_MEASURE_HUMIDITY;
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__currentClient = client;
  return /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__performCommand();
}

# 76 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
inline static error_t /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureHumidity(void ){
#line 76
  unsigned char __nesc_result;
#line 76

#line 76
  __nesc_result = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureHumidity(/*SendingMoteAppC.SensirionSht11C*/SensirionSht11C__0__HUM_KEY);
#line 76

#line 76
  return __nesc_result;
#line 76
}
#line 76
# 85 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__HumResource__granted(void )
#line 85
{
  error_t result;

#line 87
  if ((result = /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureHumidity()) != SUCCESS) {
      /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__HumResource__release();
      /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__readDone(result, 0);
    }
}

# 199 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__default__granted(uint8_t id)
#line 199
{
}

# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__granted(uint8_t arg_0xf6d4ece8){
#line 92
  switch (arg_0xf6d4ece8) {
#line 92
    case /*SendingMoteAppC.SensirionSht11C*/SensirionSht11C__0__TEMP_KEY:
#line 92
      /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__granted();
#line 92
      break;
#line 92
    case /*SendingMoteAppC.SensirionSht11C*/SensirionSht11C__0__HUM_KEY:
#line 92
      /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__HumResource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__default__granted(arg_0xf6d4ece8);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 213 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(uint8_t id)
#line 213
{
}

# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__configure(uint8_t arg_0xf6d4bb38){
#line 49
    /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(arg_0xf6d4bb38);
#line 49
}
#line 49
# 187 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask__runTask(void )
#line 187
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 188
    {
      /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__reqResId;
      /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_BUSY;
    }
#line 191
    __nesc_atomic_end(__nesc_atomic); }
  /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__configure(/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId);
  /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__granted(/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId);
}

# 50 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__disable(void ){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__disable();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__set();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set(void ){
#line 29
  /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__set();
#line 29
}
#line 29




inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeInput(void ){
#line 33
  /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__makeInput();
#line 33
}
#line 33
#line 30
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr(void ){
#line 30
  /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__clr();
#line 30
}
#line 30
# 52 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__makeOutput(void )
#line 52
{
#line 52
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 52
    * (volatile uint8_t * )34U |= 0x01 << 6;
#line 52
    __nesc_atomic_end(__nesc_atomic); }
}

# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__makeOutput(void ){
#line 35
  /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__makeOutput();
#line 35
}
#line 35
# 220 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__initPins(void )
#line 220
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__makeOutput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeInput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__disable();
}

# 88 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port15__disable(void )
#line 88
{
#line 88
  P1IE &= ~(1 << 5);
}

# 36 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__disable(void ){
#line 36
  HplMsp430InterruptP__Port15__disable();
#line 36
}
#line 36
# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__set();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__set(void ){
#line 29
  /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__set();
#line 29
}
#line 29
# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeOutput(void ){
#line 35
  /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__makeOutput();
#line 35
}
#line 35
# 228 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__resetDevice(void )
#line 228
{
  uint8_t i;

#line 230
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeOutput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
  for (i = 0; i < 9; i++) {
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__set();
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
    }
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__clr(void ){
#line 30
  /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__clr();
#line 30
}
#line 30
# 239 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__transmissionStart(void )
#line 239
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeOutput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__clr();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
}

static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__sendCommand(uint8_t _cmd)
#line 251
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__writeByte(_cmd);
}

# 48 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )32U & (0x01 << 5);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__getRaw() != 0;
}

# 59 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__get(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__get();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 40 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__get(void )
#line 40
{
#line 40
  return /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__get();
}

# 32 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static bool /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__get(void ){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__get();
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 80 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port15__enable(void )
#line 80
{
#line 80
  P1IE |= 1 << 5;
}

# 31 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__enable(void ){
#line 31
  HplMsp430InterruptP__Port15__enable();
#line 31
}
#line 31
# 137 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port15__edge(bool l2h)
#line 137
{
  /* atomic removed: atomic calls only */
#line 138
  {
    if (l2h) {
#line 139
      P1IES &= ~(1 << 5);
      }
    else {
#line 140
      P1IES |= 1 << 5;
      }
  }
}

# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__edge(bool low_to_high){
#line 56
  HplMsp430InterruptP__Port15__edge(low_to_high);
#line 56
}
#line 56
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__enable(bool rising)
#line 41
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 42
    {
      /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__disable();
      /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__edge(rising);
      /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__enable();
    }
#line 46
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}





static inline error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__enableFallingEdge(void )
#line 54
{
  return /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__enable(FALSE);
}

# 43 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__enableFallingEdge(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__enableFallingEdge();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 372 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__ack(void )
#line 372
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeOutput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__clr();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeInput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set();
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 50 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline bool /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEmpty(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 51
  {
    unsigned char __nesc_temp = 
#line 51
    /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qHead == /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY;

#line 51
    return __nesc_temp;
  }
}

# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static bool /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Queue__isEmpty(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEmpty();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 58 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__dequeue(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    if (/*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qHead != /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY) {
        uint8_t id = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qHead;

#line 62
        /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qHead = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__resQ[/*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qHead];
        if (/*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qHead == /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY) {
          /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qTail = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY;
          }
#line 65
        /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__resQ[id] = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 66
          id;

#line 66
          return __nesc_temp;
        }
      }
#line 68
    {
      unsigned char __nesc_temp = 
#line 68
      /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY;

#line 68
      return __nesc_temp;
    }
  }
}

# 60 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Queue__dequeue(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__dequeue();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 215 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(uint8_t id)
#line 215
{
}

# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(uint8_t arg_0xf6d4bb38){
#line 55
    /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(arg_0xf6d4bb38);
#line 55
}
#line 55
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 97 "/opt/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__granted(void )
#line 97
{
  /* atomic removed: atomic calls only */
#line 98
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopping = TRUE;
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopTask__postTask();
}

# 46 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted(void ){
#line 46
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__granted();
#line 46
}
#line 46
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__HumResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__request(/*SendingMoteAppC.SensirionSht11C*/SensirionSht11C__0__HUM_KEY);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 80 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline error_t /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__read(void )
#line 80
{
  /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__HumResource__request();
  return SUCCESS;
}

# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
inline static error_t SendingMoteC__Humidity__read(void ){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__read();
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 201 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(uint8_t id)
#line 201
{
}

# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
inline static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceRequested__requested(uint8_t arg_0xf6d4c770){
#line 43
    /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(arg_0xf6d4c770);
#line 43
}
#line 43
# 54 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline bool /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 54
{
  /* atomic removed: atomic calls only */
#line 55
  {
    unsigned char __nesc_temp = 
#line 55
    /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__resQ[id] != /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY || /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qTail == id;

#line 55
    return __nesc_temp;
  }
}

#line 72
static inline error_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(resource_client_id_t id)
#line 72
{
  /* atomic removed: atomic calls only */
#line 73
  {
    if (!/*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(id)) {
        if (/*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qHead == /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY) {
          /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qHead = id;
          }
        else {
#line 78
          /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__resQ[/*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qTail] = id;
          }
#line 79
        /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qTail = id;
        {
          unsigned char __nesc_temp = 
#line 80
          SUCCESS;

#line 80
          return __nesc_temp;
        }
      }
#line 82
    {
      unsigned char __nesc_temp = 
#line 82
      EBUSY;

#line 82
      return __nesc_temp;
    }
  }
}

# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Queue__enqueue(resource_client_id_t id){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(id);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__startTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__startTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 74 "/opt/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__requested(void )
#line 74
{
  if (/*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopping == FALSE) {
      /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__startTask__postTask();
    }
  else {
#line 78
    /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__requested = TRUE;
    }
}

# 73 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested(void ){
#line 73
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__requested();
#line 73
}
#line 73
# 102 "/opt/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__stopDone(error_t error)
#line 102
{
  if (/*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__requested == TRUE) {
      /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__start();
      /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__start();
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 107
    {
      /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__requested = FALSE;
      /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopping = FALSE;
    }
#line 110
    __nesc_atomic_end(__nesc_atomic); }
}

# 117 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static void HplSensirionSht11P__SplitControl__stopDone(error_t error){
#line 117
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__stopDone(error);
#line 117
}
#line 117
# 73 "/opt/tinyos-2.x/tos/platforms/telosa/chips/sht11/HplSensirionSht11P.nc"
static inline void HplSensirionSht11P__stopTask__runTask(void )
#line 73
{
  HplSensirionSht11P__SplitControl__stopDone(SUCCESS);
}

# 99 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureHumidityDone(error_t result, uint16_t val)
#line 99
{
}

#line 93
static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureHumidityDone(error_t result, uint16_t val)
#line 93
{
  /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__HumResource__release();
  /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__readDone(result, val);
}

# 408 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__measureHumidityDone(uint8_t client, error_t result, uint16_t val)
#line 408
{
}

# 84 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureHumidityDone(uint8_t arg_0xf6823938, error_t result, uint16_t val){
#line 84
  switch (arg_0xf6823938) {
#line 84
    case /*SendingMoteAppC.SensirionSht11C*/SensirionSht11C__0__TEMP_KEY:
#line 84
      /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureHumidityDone(result, val);
#line 84
      break;
#line 84
    case /*SendingMoteAppC.SensirionSht11C*/SensirionSht11C__0__HUM_KEY:
#line 84
      /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureHumidityDone(result, val);
#line 84
      break;
#line 84
    default:
#line 84
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__measureHumidityDone(arg_0xf6823938, result, val);
#line 84
      break;
#line 84
    }
#line 84
}
#line 84
# 73 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureTemperatureDone(error_t result, uint16_t val)
#line 73
{
  /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__release();
  /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__readDone(result, val);
}

#line 104
static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureTemperatureDone(error_t result, uint16_t val)
#line 104
{
}

# 407 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__measureTemperatureDone(uint8_t client, error_t result, uint16_t val)
#line 407
{
}

# 69 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureTemperatureDone(uint8_t arg_0xf6823938, error_t result, uint16_t val){
#line 69
  switch (arg_0xf6823938) {
#line 69
    case /*SendingMoteAppC.SensirionSht11C*/SensirionSht11C__0__TEMP_KEY:
#line 69
      /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureTemperatureDone(result, val);
#line 69
      break;
#line 69
    case /*SendingMoteAppC.SensirionSht11C*/SensirionSht11C__0__HUM_KEY:
#line 69
      /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureTemperatureDone(result, val);
#line 69
      break;
#line 69
    default:
#line 69
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__measureTemperatureDone(arg_0xf6823938, result, val);
#line 69
      break;
#line 69
    }
#line 69
}
#line 69
# 67 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(9U);
#line 67
}
#line 67
# 320 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readSensor__runTask(void )
#line 320
{
  uint16_t data = 0;
  uint8_t crc = 0;

  if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy == FALSE) {


      return;
    }

  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__stop();

  data = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readByte() << 8;
  data |= /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readByte();

  crc = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readByte();

  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__endTransmission();

  switch (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__cmd) {
      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_MEASURE_TEMPERATURE: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureTemperatureDone(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__currentClient, SUCCESS, data);
      break;

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_MEASURE_HUMIDITY: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureHumidityDone(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__currentClient, SUCCESS, data);
      break;

      default: 
        break;
    }
}

# 101 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__writeStatusRegDone(error_t result)
#line 101
{
}



static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__writeStatusRegDone(error_t result)
#line 106
{
}

# 410 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__writeStatusRegDone(uint8_t client, error_t result)
#line 410
{
}

# 116 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__writeStatusRegDone(uint8_t arg_0xf6823938, error_t result){
#line 116
  switch (arg_0xf6823938) {
#line 116
    case /*SendingMoteAppC.SensirionSht11C*/SensirionSht11C__0__TEMP_KEY:
#line 116
      /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__writeStatusRegDone(result);
#line 116
      break;
#line 116
    case /*SendingMoteAppC.SensirionSht11C*/SensirionSht11C__0__HUM_KEY:
#line 116
      /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__writeStatusRegDone(result);
#line 116
      break;
#line 116
    default:
#line 116
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__writeStatusRegDone(arg_0xf6823938, result);
#line 116
      break;
#line 116
    }
#line 116
}
#line 116
# 100 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__readStatusRegDone(error_t result, uint8_t val)
#line 100
{
}



static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__readStatusRegDone(error_t result, uint8_t val)
#line 105
{
}

# 409 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__readStatusRegDone(uint8_t client, error_t result, uint8_t val)
#line 409
{
}

# 100 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__readStatusRegDone(uint8_t arg_0xf6823938, error_t result, uint8_t val){
#line 100
  switch (arg_0xf6823938) {
#line 100
    case /*SendingMoteAppC.SensirionSht11C*/SensirionSht11C__0__TEMP_KEY:
#line 100
      /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__readStatusRegDone(result, val);
#line 100
      break;
#line 100
    case /*SendingMoteAppC.SensirionSht11C*/SensirionSht11C__0__HUM_KEY:
#line 100
      /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__readStatusRegDone(result, val);
#line 100
      break;
#line 100
    default:
#line 100
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__readStatusRegDone(arg_0xf6823938, result, val);
#line 100
      break;
#line 100
    }
#line 100
}
#line 100
# 388 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone__runTask(void )
#line 388
{
  bool _writeFail = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__writeFail;

#line 390
  switch (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__cmd) {
      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_READ_STATUS: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__readStatusRegDone(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__currentClient, SUCCESS, /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__status);
      break;
      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_WRITE_STATUS: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__writeFail = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__writeStatusRegDone(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__currentClient, _writeFail ? FAIL : SUCCESS);
      break;
      default: 

        break;
    }
}

# 118 "/opt/tinyos-2.x/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void )
#line 118
{
  uint8_t i;
#line 119
  uint8_t j;
#line 119
  uint8_t mask;
#line 119
  uint8_t last;
  message_t *msg;

#line 121
  for (i = 0; i < 5 / 8 + 1; i++) {
      if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i]) {
          for (mask = 1, j = 0; j < 8; j++) {
              if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i] & mask) {
                  last = i * 8 + j;
                  msg = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg = (void *)0;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i] &= ~mask;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(last, msg, ECANCEL);
                }
              mask <<= 1;
            }
        }
    }
}

# 1074 "SendingMoteC.nc"
static inline void SendingMoteC__CommandMsgSend__sendDone(message_t *m, error_t error)
#line 1074
{



  SendingMoteC__busy = FALSE;
}

# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static void /*SendingMoteAppC.CommandMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__4__AMSend__sendDone(message_t * msg, error_t error){
#line 99
  SendingMoteC__CommandMsgSend__sendDone(msg, error);
#line 99
}
#line 99
# 57 "/opt/tinyos-2.x/tos/system/AMQueueEntryP.nc"
static inline void /*SendingMoteAppC.CommandMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__4__Send__sendDone(message_t *m, error_t err)
#line 57
{
  /*SendingMoteAppC.CommandMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__4__AMSend__sendDone(m, err);
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void SendingMoteC__FlushTimer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(1U);
#line 67
}
#line 67
# 1021 "SendingMoteC.nc"
static inline void SendingMoteC__TinyReportMsgSend__sendDone(message_t *m, error_t error)
#line 1021
{
  SendingMoteC__busy = FALSE;
  if (error == SUCCESS) {

      SendingMoteC__sendBlink();


      SendingMoteC__FlushTimer__stop();


      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        if (TRUE) {

            SendingMoteC__radioOut = (SendingMoteC__radioOut + 1) % SendingMoteC__RADIO_QUEUE_LEN;


            SendingMoteC__radioFull = FALSE;
          }
#line 1038
        __nesc_atomic_end(__nesc_atomic); }
    }
  else {
      SendingMoteC__failBlink();
    }
}

# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static void /*SendingMoteAppC.TinyReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMSend__sendDone(message_t * msg, error_t error){
#line 99
  SendingMoteC__TinyReportMsgSend__sendDone(msg, error);
#line 99
}
#line 99
# 57 "/opt/tinyos-2.x/tos/system/AMQueueEntryP.nc"
static inline void /*SendingMoteAppC.TinyReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__3__Send__sendDone(message_t *m, error_t err)
#line 57
{
  /*SendingMoteAppC.TinyReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMSend__sendDone(m, err);
}

# 47 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle(void )
#line 47
{
#line 47
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 47
    * (volatile uint8_t * )49U ^= 0x01 << 4;
#line 47
    __nesc_atomic_end(__nesc_atomic); }
}

# 44 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__HplGeneralIO__toggle(void ){
#line 44
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle();
#line 44
}
#line 44
# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__GeneralIO__toggle(void )
#line 39
{
#line 39
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__HplGeneralIO__toggle();
}

# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__toggle(void ){
#line 31
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__GeneralIO__toggle();
#line 31
}
#line 31
# 73 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline void LedsP__Leds__led0Toggle(void )
#line 73
{
  LedsP__Led0__toggle();
  ;
#line 75
  ;
}

# 56 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void SendingMoteC__Leds__led0Toggle(void ){
#line 56
  LedsP__Leds__led0Toggle();
#line 56
}
#line 56
# 41 "/opt/tinyos-2.x/tos/interfaces/Random.nc"
inline static uint16_t SendingMoteC__Random__rand16(void ){
#line 41
  unsigned int __nesc_result;
#line 41

#line 41
  __nesc_result = RandomMlcgC__Random__rand16();
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 814 "SendingMoteC.nc"
static inline void SendingMoteC__clearQueue(void )
#line 814
{
}

#line 954
static inline void SendingMoteC__ReportMsgSend__sendDone(message_t *m, error_t error)
#line 954
{
  uint8_t tmpI = 0;

#line 956
  SendingMoteC__busy = FALSE;

  if (error == SUCCESS) {

      SendingMoteC__sendBlink();


      SendingMoteC__FlushTimer__stop();


      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        if (TRUE) {

            for (tmpI = 0; SendingMoteC__radioOut != SendingMoteC__radioIn && tmpI < SendingMoteC__queueFlushThreshold; tmpI++) {
                SendingMoteC__radioOut = (SendingMoteC__radioOut + 1) % SendingMoteC__RADIO_QUEUE_LEN;
              }


            SendingMoteC__radioFull = FALSE;


            if (SendingMoteC__doRandomizedThresholding) {


                SendingMoteC__queueFlushThreshold = 1 + SendingMoteC__Random__rand16() % 4;
              }
          }
#line 982
        __nesc_atomic_end(__nesc_atomic); }
    }
  else {

      SendingMoteC__failBlink();


      SendingMoteC__GapTimer__startOneShot(SendingMoteC__reportingGap + SendingMoteC__reportingGap / 5 - SendingMoteC__Random__rand16() % (SendingMoteC__reportingGap / 3));
    }




  if (SendingMoteC__reportProtocol == REPORTING_MASS && SendingMoteC__reportMassTrigered == TRUE) {

      uint16_t qsize = SendingMoteC__queueSize();

      if (qsize == 0) {
          SendingMoteC__reportMassTrigered = FALSE;


          SendingMoteC__radioIn = SendingMoteC__radioOut = 0;
          SendingMoteC__radioBusy = FALSE;
          SendingMoteC__radioFull = TRUE;


          SendingMoteC__clearQueue();
        }
      else {

          SendingMoteC__GapTimer__startOneShot(SendingMoteC__reportingGap + SendingMoteC__reportingGap / 5 - SendingMoteC__Random__rand16() % (SendingMoteC__reportingGap / 3));
        }
    }
}

# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static void /*SendingMoteAppC.ReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMSend__sendDone(message_t * msg, error_t error){
#line 99
  SendingMoteC__ReportMsgSend__sendDone(msg, error);
#line 99
}
#line 99
# 57 "/opt/tinyos-2.x/tos/system/AMQueueEntryP.nc"
static inline void /*SendingMoteAppC.ReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__2__Send__sendDone(message_t *m, error_t err)
#line 57
{
  /*SendingMoteAppC.ReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMSend__sendDone(m, err);
}

# 1056 "SendingMoteC.nc"
static inline void SendingMoteC__MultiPingResponseSend__sendDone(message_t *m, error_t error)
#line 1056
{
  SendingMoteC__busy = FALSE;

  if (error == SUCCESS) {
      SendingMoteC__sendBlink();


      SendingMoteC__counter_ping++;
    }
  else {
      SendingMoteC__failBlink();
    }
}

# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static void /*SendingMoteAppC.MultiPingResponseSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMSend__sendDone(message_t * msg, error_t error){
#line 99
  SendingMoteC__MultiPingResponseSend__sendDone(msg, error);
#line 99
}
#line 99
# 57 "/opt/tinyos-2.x/tos/system/AMQueueEntryP.nc"
static inline void /*SendingMoteAppC.MultiPingResponseSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__Send__sendDone(message_t *m, error_t err)
#line 57
{
  /*SendingMoteAppC.MultiPingResponseSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMSend__sendDone(m, err);
}

# 1045 "SendingMoteC.nc"
static inline void SendingMoteC__RssiMsgSend__sendDone(message_t *m, error_t error)
#line 1045
{
  SendingMoteC__busy = FALSE;

  if (error == SUCCESS) {
      SendingMoteC__sendBlink();
    }
  else {
      SendingMoteC__failBlink();
    }
}

# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static void /*SendingMoteAppC.RssiMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(message_t * msg, error_t error){
#line 99
  SendingMoteC__RssiMsgSend__sendDone(msg, error);
#line 99
}
#line 99
# 57 "/opt/tinyos-2.x/tos/system/AMQueueEntryP.nc"
static inline void /*SendingMoteAppC.RssiMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err)
#line 57
{
  /*SendingMoteAppC.RssiMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(m, err);
}

# 155 "/opt/tinyos-2.x/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(uint8_t last, message_t * msg, error_t err)
#line 155
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg = (void *)0;
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend();
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(last, msg, err);
}

static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void )
#line 161
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current, /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg, FAIL);
}

#line 57
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket(void )
#line 57
{
  uint8_t i;

#line 59
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 5;
  for (i = 0; i < 5; i++) {
      if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg == (void *)0 || 
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current / 8] & (1 << /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current % 8)) 
        {
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 5;
        }
      else {
          break;
        }
    }
  if (i >= 5) {
#line 70
    /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 5;
    }
}

# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420ActiveMessageP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 151 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline am_id_t CC2420ActiveMessageP__AMPacket__type(message_t *amsg)
#line 151
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(amsg);

#line 153
  return __nesc_ntoh_leuint8(header->type.data);
}

# 136 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static am_id_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(message_t * amsg){
#line 136
  unsigned char __nesc_result;
#line 136

#line 136
  __nesc_result = CC2420ActiveMessageP__AMPacket__type(amsg);
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
#line 67
inline static am_addr_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(message_t * amsg){
#line 67
  unsigned int __nesc_result;
#line 67

#line 67
  __nesc_result = CC2420ActiveMessageP__AMPacket__destination(amsg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 181 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline uint8_t CC2420ActiveMessageP__Packet__payloadLength(message_t *msg)
#line 181
{
  return __nesc_ntoh_leuint8(CC2420ActiveMessageP__CC2420PacketBody__getHeader(msg)->length.data) - CC2420_SIZE;
}

# 67 "/opt/tinyos-2.x/tos/interfaces/Packet.nc"
inline static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(message_t * msg){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = CC2420ActiveMessageP__Packet__payloadLength(msg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(am_id_t arg_0xf68a3bc8, am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = CC2420ActiveMessageP__AMSend__send(arg_0xf68a3bc8, addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 170 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline uint8_t CC2420CsmaP__Send__maxPayloadLength(void )
#line 170
{
  return 28;
}

# 101 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static uint8_t UniqueSendP__SubSend__maxPayloadLength(void ){
#line 101
  unsigned char __nesc_result;
#line 101

#line 101
  __nesc_result = CC2420CsmaP__Send__maxPayloadLength();
#line 101

#line 101
  return __nesc_result;
#line 101
}
#line 101
# 95 "/opt/tinyos-2.x/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline uint8_t UniqueSendP__Send__maxPayloadLength(void )
#line 95
{
  return UniqueSendP__SubSend__maxPayloadLength();
}

# 101 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static uint8_t CC2420TinyosNetworkP__SubSend__maxPayloadLength(void ){
#line 101
  unsigned char __nesc_result;
#line 101

#line 101
  __nesc_result = UniqueSendP__Send__maxPayloadLength();
#line 101

#line 101
  return __nesc_result;
#line 101
}
#line 101
# 82 "/opt/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline uint8_t CC2420TinyosNetworkP__ActiveSend__maxPayloadLength(void )
#line 82
{
  return CC2420TinyosNetworkP__SubSend__maxPayloadLength();
}

# 101 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static uint8_t CC2420ActiveMessageP__SubSend__maxPayloadLength(void ){
#line 101
  unsigned char __nesc_result;
#line 101

#line 101
  __nesc_result = CC2420TinyosNetworkP__ActiveSend__maxPayloadLength();
#line 101

#line 101
  return __nesc_result;
#line 101
}
#line 101
# 189 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline uint8_t CC2420ActiveMessageP__Packet__maxPayloadLength(void )
#line 189
{
  return CC2420ActiveMessageP__SubSend__maxPayloadLength();
}

# 287 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline uint16_t CC2420ControlP__CC2420Config__getPanAddr(void )
#line 287
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 288
    {
      unsigned int __nesc_temp = 
#line 288
      CC2420ControlP__m_pan;

      {
#line 288
        __nesc_atomic_end(__nesc_atomic); 
#line 288
        return __nesc_temp;
      }
    }
#line 290
    __nesc_atomic_end(__nesc_atomic); }
}

# 70 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static uint16_t CC2420ActiveMessageP__CC2420Config__getPanAddr(void ){
#line 70
  unsigned int __nesc_result;
#line 70

#line 70
  __nesc_result = CC2420ControlP__CC2420Config__getPanAddr();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static bool CC2420TinyosNetworkP__Queue__isEmpty(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 184 "/opt/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP__Resource__immediateRequest(uint8_t id)
#line 184
{
  if (CC2420TinyosNetworkP__resource_owner == id) {
#line 185
    return EALREADY;
    }
  if (CC2420TinyosNetworkP__TINYOS_N_NETWORKS > 1) {
      if (CC2420TinyosNetworkP__resource_owner == CC2420TinyosNetworkP__OWNER_NONE && CC2420TinyosNetworkP__Queue__isEmpty()) {
          CC2420TinyosNetworkP__resource_owner = id;
          return SUCCESS;
        }
      return FAIL;
    }
  else 
#line 193
    {
      CC2420TinyosNetworkP__resource_owner = id;
      return SUCCESS;
    }
}

# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420ActiveMessageP__RadioResource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = CC2420TinyosNetworkP__Resource__immediateRequest(CC2420ActiveMessageC__CC2420_AM_SEND_ID);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 278 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__SendNotifier__default__aboutToSend(am_id_t amId, am_addr_t addr, message_t *msg)
#line 278
{
}

# 59 "/opt/tinyos-2.x/tos/interfaces/SendNotifier.nc"
inline static void CC2420ActiveMessageP__SendNotifier__aboutToSend(am_id_t arg_0xf6a1ee20, am_addr_t dest, message_t * msg){
#line 59
    CC2420ActiveMessageP__SendNotifier__default__aboutToSend(arg_0xf6a1ee20, dest, msg);
#line 59
}
#line 59
# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static error_t CC2420TinyosNetworkP__SubSend__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = UniqueSendP__Send__send(msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 77 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Packet.nc"
inline static void CC2420TinyosNetworkP__CC2420Packet__setNetwork(message_t *p_msg, uint8_t networkId){
#line 77
  CC2420PacketP__CC2420Packet__setNetwork(p_msg, networkId);
#line 77
}
#line 77
# 73 "/opt/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP__ActiveSend__send(message_t *msg, uint8_t len)
#line 73
{
  CC2420TinyosNetworkP__CC2420Packet__setNetwork(msg, 0x3f);
  return CC2420TinyosNetworkP__SubSend__send(msg, len);
}

# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static error_t CC2420ActiveMessageP__SubSend__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = CC2420TinyosNetworkP__ActiveSend__send(msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 45 "/opt/tinyos-2.x/tos/interfaces/State.nc"
inline static error_t UniqueSendP__State__requestState(uint8_t reqState){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = StateImplP__State__requestState(2U, reqState);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * UniqueSendP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 240 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_hton_uint8(void * target, uint8_t value)
#line 240
{
  uint8_t *base = target;

#line 242
  base[0] = value;
  return value;
}

#line 257
static __inline  int8_t __nesc_hton_int8(void * target, int8_t value)
#line 257
{
#line 257
  __nesc_hton_uint8(target, value);
#line 257
  return value;
}

#line 281
static __inline  uint16_t __nesc_hton_leuint16(void * target, uint16_t value)
#line 281
{
  uint8_t *base = target;

#line 283
  base[0] = value;
  base[1] = value >> 8;
  return value;
}

#line 251
static __inline  uint8_t __nesc_hton_leuint8(void * target, uint8_t value)
#line 251
{
  uint8_t *base = target;

#line 253
  base[0] = value;
  return value;
}

# 545 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__send(message_t * p_msg, bool cca)
#line 545
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 546
    {
      if (CC2420TransmitP__m_state == CC2420TransmitP__S_CANCEL) {
          {
            unsigned char __nesc_temp = 
#line 548
            ECANCEL;

            {
#line 548
              __nesc_atomic_end(__nesc_atomic); 
#line 548
              return __nesc_temp;
            }
          }
        }
#line 551
      if (CC2420TransmitP__m_state != CC2420TransmitP__S_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 552
            FAIL;

            {
#line 552
              __nesc_atomic_end(__nesc_atomic); 
#line 552
              return __nesc_temp;
            }
          }
        }


      CC2420TransmitP__m_state = CC2420TransmitP__S_LOAD;
      CC2420TransmitP__m_cca = cca;
      CC2420TransmitP__m_msg = p_msg;
      CC2420TransmitP__totalCcaChecks = 0;
    }
#line 562
    __nesc_atomic_end(__nesc_atomic); }

  if (CC2420TransmitP__acquireSpiResource() == SUCCESS) {
      CC2420TransmitP__loadTXFIFO();
    }

  return SUCCESS;
}

#line 192
static inline error_t CC2420TransmitP__Send__send(message_t * p_msg, bool useCca)
#line 192
{
  return CC2420TransmitP__send(p_msg, useCca);
}

# 51 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
inline static error_t CC2420CsmaP__CC2420Transmit__send(message_t * p_msg, bool useCca){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420TransmitP__Send__send(p_msg, useCca);
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 288 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__RadioBackoff__default__requestCca(am_id_t id, 
message_t *msg)
#line 289
{
}

# 95 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420ActiveMessageP__RadioBackoff__requestCca(am_id_t arg_0xf6a1d4b8, message_t * msg){
#line 95
    CC2420ActiveMessageP__RadioBackoff__default__requestCca(arg_0xf6a1d4b8, msg);
#line 95
}
#line 95
# 237 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__SubBackoff__requestCca(message_t *msg)
#line 237
{

  CC2420ActiveMessageP__RadioBackoff__requestCca(__nesc_ntoh_leuint8(((cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t )))->type.data), msg);
}

# 95 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__RadioBackoff__requestCca(message_t * msg){
#line 95
  CC2420ActiveMessageP__SubBackoff__requestCca(msg);
#line 95
}
#line 95
# 111 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
static inline void StateImplP__State__forceState(uint8_t id, uint8_t reqState)
#line 111
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 112
    StateImplP__state[id] = reqState;
#line 112
    __nesc_atomic_end(__nesc_atomic); }
}

# 51 "/opt/tinyos-2.x/tos/interfaces/State.nc"
inline static void CC2420CsmaP__SplitControlState__forceState(uint8_t reqState){
#line 51
  StateImplP__State__forceState(1U, reqState);
#line 51
}
#line 51
#line 66
inline static bool CC2420CsmaP__SplitControlState__isState(uint8_t myState){
#line 66
  unsigned char __nesc_result;
#line 66

#line 66
  __nesc_result = StateImplP__State__isState(1U, myState);
#line 66

#line 66
  return __nesc_result;
#line 66
}
#line 66
# 47 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420CsmaP__CC2420PacketBody__getMetadata(message_t * msg){
#line 47
  nx_struct cc2420_metadata_t *__nesc_result;
#line 47

#line 47
  __nesc_result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
#line 42
inline static cc2420_header_t * CC2420CsmaP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 122 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline error_t CC2420CsmaP__Send__send(message_t *p_msg, uint8_t len)
#line 122
{
  unsigned char *__nesc_temp43;
  unsigned char *__nesc_temp42;
#line 124
  cc2420_header_t *header = CC2420CsmaP__CC2420PacketBody__getHeader(p_msg);
  cc2420_metadata_t *metadata = CC2420CsmaP__CC2420PacketBody__getMetadata(p_msg);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 127
    {
      if (!CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STARTED)) {
          {
            unsigned char __nesc_temp = 
#line 129
            FAIL;

            {
#line 129
              __nesc_atomic_end(__nesc_atomic); 
#line 129
              return __nesc_temp;
            }
          }
        }
#line 132
      CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_TRANSMITTING);
      CC2420CsmaP__m_msg = p_msg;
    }
#line 134
    __nesc_atomic_end(__nesc_atomic); }

  __nesc_hton_leuint8(header->length.data, len + CC2420_SIZE);



  (__nesc_temp42 = header->fcf.data, __nesc_hton_leuint16(__nesc_temp42, __nesc_ntoh_leuint16(__nesc_temp42) & (1 << IEEE154_FCF_ACK_REQ)));

  (__nesc_temp43 = header->fcf.data, __nesc_hton_leuint16(__nesc_temp43, __nesc_ntoh_leuint16(__nesc_temp43) | ((((IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE) | (
  1 << IEEE154_FCF_INTRAPAN)) | (
  IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE)) | (
  IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE))));

  __nesc_hton_int8(metadata->ack.data, FALSE);
  __nesc_hton_uint8(metadata->rssi.data, 0);
  __nesc_hton_uint8(metadata->lqi.data, 0);

  __nesc_hton_uint32(metadata->timestamp.data, CC2420_INVALID_TIMESTAMP);

  CC2420CsmaP__ccaOn = TRUE;
  CC2420CsmaP__RadioBackoff__requestCca(CC2420CsmaP__m_msg);

  CC2420CsmaP__CC2420Transmit__send(CC2420CsmaP__m_msg, CC2420CsmaP__ccaOn);
  return SUCCESS;
}

# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static error_t UniqueSendP__SubSend__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = CC2420CsmaP__Send__send(msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 55 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420TransmitP__TXCTRL__write(uint16_t data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP__Reg__write(CC2420_TXCTRL, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 59 "/opt/tinyos-2.x/tos/interfaces/SpiPacket.nc"
inline static error_t CC2420SpiP__SpiPacket__send(uint8_t * txBuf, uint8_t * rxBuf, uint16_t len){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID, txBuf, rxBuf, len);
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 34 "/opt/tinyos-2.x/tos/interfaces/SpiByte.nc"
inline static uint8_t CC2420SpiP__SpiByte__write(uint8_t tx){
#line 34
  unsigned char __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(tx);
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 126 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
static inline bool StateImplP__State__isIdle(uint8_t id)
#line 126
{
  return StateImplP__State__isState(id, StateImplP__S_IDLE);
}

# 61 "/opt/tinyos-2.x/tos/interfaces/State.nc"
inline static bool CC2420SpiP__WorkingState__isIdle(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = StateImplP__State__isIdle(0U);
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 214 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline cc2420_status_t CC2420SpiP__Fifo__write(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 215
{

  uint8_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 219
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 221
            status;

            {
#line 221
              __nesc_atomic_end(__nesc_atomic); 
#line 221
              return __nesc_temp;
            }
          }
        }
    }
#line 225
    __nesc_atomic_end(__nesc_atomic); }
#line 225
  CC2420SpiP__m_addr = addr;

  status = CC2420SpiP__SpiByte__write(CC2420SpiP__m_addr);
  CC2420SpiP__SpiPacket__send(data, (void *)0, len);

  return status;
}

# 82 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static cc2420_status_t CC2420TransmitP__TXFIFO__write(uint8_t * data, uint8_t length){
#line 82
  unsigned char __nesc_result;
#line 82

#line 82
  __nesc_result = CC2420SpiP__Fifo__write(CC2420_TXFIFO, data, length);
#line 82

#line 82
  return __nesc_result;
#line 82
}
#line 82
# 361 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__enableRxIntr(void )
#line 361
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 362
    {
      HplMsp430Usart0P__IFG1 &= ~(1 << 6);
      HplMsp430Usart0P__IE1 |= 1 << 6;
    }
#line 365
    __nesc_atomic_end(__nesc_atomic); }
}

# 180 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__enableRxIntr(void ){
#line 180
  HplMsp430Usart0P__Usart__enableRxIntr();
#line 180
}
#line 180
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 54 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 54
{
  /* atomic removed: atomic calls only */
#line 55
  {
    unsigned char __nesc_temp = 
#line 55
    /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[id] != /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY || /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail == id;

#line 55
    return __nesc_temp;
  }
}

#line 72
static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id)
#line 72
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 73
    {
      if (!/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(id)) {
          if (/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead == /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY) {
            /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead = id;
            }
          else {
#line 78
            /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail] = id;
            }
#line 79
          /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail = id;
          {
            unsigned char __nesc_temp = 
#line 80
            SUCCESS;

            {
#line 80
              __nesc_atomic_end(__nesc_atomic); 
#line 80
              return __nesc_temp;
            }
          }
        }
#line 82
      {
        unsigned char __nesc_temp = 
#line 82
        EBUSY;

        {
#line 82
          __nesc_atomic_end(__nesc_atomic); 
#line 82
          return __nesc_temp;
        }
      }
    }
#line 85
    __nesc_atomic_end(__nesc_atomic); }
}

# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static error_t CC2420TinyosNetworkP__Queue__enqueue(resource_client_id_t id){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__enqueue(id);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420TinyosNetworkP__grantTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420TinyosNetworkP__grantTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 167 "/opt/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP__Resource__request(uint8_t id)
#line 167
{

  CC2420TinyosNetworkP__grantTask__postTask();

  if (CC2420TinyosNetworkP__TINYOS_N_NETWORKS > 1) {

      return CC2420TinyosNetworkP__Queue__enqueue(id);
    }
  else 
#line 174
    {
      if (id == CC2420TinyosNetworkP__resource_owner) {
          return EALREADY;
        }
      else 
#line 177
        {
          CC2420TinyosNetworkP__next_owner = id;
          return SUCCESS;
        }
    }
}

# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420ActiveMessageP__RadioResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420TinyosNetworkP__Resource__request(CC2420ActiveMessageC__CC2420_AM_SEND_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
inline static error_t SendingMoteC__sendEchoReply__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SendingMoteC__sendEchoReply);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static error_t /*SendingMoteAppC.RssiMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(0U, msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 151 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static void /*SendingMoteAppC.RssiMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(message_t * amsg, am_id_t t){
#line 151
  CC2420ActiveMessageP__AMPacket__setType(amsg, t);
#line 151
}
#line 151
#line 92
inline static void /*SendingMoteAppC.RssiMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(message_t * amsg, am_addr_t addr){
#line 92
  CC2420ActiveMessageP__AMPacket__setDestination(amsg, addr);
#line 92
}
#line 92
# 45 "/opt/tinyos-2.x/tos/system/AMQueueEntryP.nc"
static inline error_t /*SendingMoteAppC.RssiMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 47
{
  /*SendingMoteAppC.RssiMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(msg, dest);
  /*SendingMoteAppC.RssiMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(msg, 10);
  return /*SendingMoteAppC.RssiMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(msg, len);
}

# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static error_t SendingMoteC__RssiMsgSend__send(am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*SendingMoteAppC.RssiMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 114 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static void * CC2420ActiveMessageP__SubSend__getPayload(message_t * msg, uint8_t len){
#line 114
  void *__nesc_result;
#line 114

#line 114
  __nesc_result = CC2420TinyosNetworkP__ActiveSend__getPayload(msg, len);
#line 114

#line 114
  return __nesc_result;
#line 114
}
#line 114
# 193 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void *CC2420ActiveMessageP__Packet__getPayload(message_t *msg, uint8_t len)
#line 193
{
  return CC2420ActiveMessageP__SubSend__getPayload(msg, len);
}

# 115 "/opt/tinyos-2.x/tos/interfaces/Packet.nc"
inline static void * SendingMoteC__Packet__getPayload(message_t * msg, uint8_t len){
#line 115
  void *__nesc_result;
#line 115

#line 115
  __nesc_result = CC2420ActiveMessageP__Packet__getPayload(msg, len);
#line 115

#line 115
  return __nesc_result;
#line 115
}
#line 115
# 1377 "SendingMoteC.nc"
static inline void SendingMoteC__sendEchoReply__runTask(void )
#line 1377
{
  if (!SendingMoteC__busy) {
      RssiMsg *btrpkt = (RssiMsg *)SendingMoteC__Packet__getPayload(&SendingMoteC__pkt, 0);

#line 1380
      __nesc_hton_uint8(btrpkt->nodeid.data, TOS_NODE_ID);


      SendingMoteC__setPower(&SendingMoteC__pkt, SendingMoteC__tx_power);




      if (SendingMoteC__RssiMsgSend__send(AM_BROADCAST_ADDR, &SendingMoteC__pkt, sizeof(RssiMsg )) == SUCCESS) {
          SendingMoteC__busy = TRUE;
        }
      else 
        {
          SendingMoteC__failBlink();
          SendingMoteC__sendEchoReply__postTask();
        }
    }
  else 
    {
      SendingMoteC__failBlink();
      SendingMoteC__sendEchoReply__postTask();
    }
}

# 80 "/opt/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline void CC2420PacketP__CC2420Packet__setPower(message_t *p_msg, uint8_t power)
#line 80
{
  if (power > 31) {
    power = 31;
    }
#line 83
  __nesc_hton_uint8(CC2420PacketP__CC2420PacketBody__getMetadata(p_msg)->tx_power.data, power);
}

# 56 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Packet.nc"
inline static void SendingMoteC__CC2420Packet__setPower(message_t *p_msg, uint8_t power){
#line 56
  CC2420PacketP__CC2420Packet__setPower(p_msg, power);
#line 56
}
#line 56
# 185 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__Packet__setPayloadLength(message_t *msg, uint8_t len)
#line 185
{
  __nesc_hton_leuint8(CC2420ActiveMessageP__CC2420PacketBody__getHeader(msg)->length.data, len + CC2420_SIZE);
}

# 83 "/opt/tinyos-2.x/tos/interfaces/Packet.nc"
inline static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(message_t * msg, uint8_t len){
#line 83
  CC2420ActiveMessageP__Packet__setPayloadLength(msg, len);
#line 83
}
#line 83
# 269 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_hton_uint16(void * target, uint16_t value)
#line 269
{
  uint8_t *base = target;

#line 271
  base[1] = value;
  base[0] = value >> 8;
  return value;
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t SendingMoteC__sendCommandACK__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SendingMoteC__sendCommandACK);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static error_t SendingMoteC__CommandMsgSend__send(am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*SendingMoteAppC.CommandMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__4__AMSend__send(addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 716 "SendingMoteC.nc"
static inline void SendingMoteC__sendCommandACK__runTask(void )
#line 716
{
  if (!SendingMoteC__busy) {
      CommandMsg *btrpkt = (CommandMsg *)SendingMoteC__Packet__getPayload(&SendingMoteC__pkt_priv, 0);


      __nesc_hton_uint16(btrpkt->command_id.data, SendingMoteC__counter);
#line 733
      if (SendingMoteC__CommandMsgSend__send(SendingMoteC__command_dest, &SendingMoteC__pkt_priv, sizeof(CommandMsg )) == SUCCESS) {
          SendingMoteC__busy = TRUE;
        }
      else 
        {
          SendingMoteC__failBlink();
          ;
          SendingMoteC__sendCommandACK__postTask();
        }
    }
  else 
    {
      SendingMoteC__failBlink();
      SendingMoteC__sendCommandACK__postTask();
    }
}

# 92 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static void /*SendingMoteAppC.CommandMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__4__AMPacket__setDestination(message_t * amsg, am_addr_t addr){
#line 92
  CC2420ActiveMessageP__AMPacket__setDestination(amsg, addr);
#line 92
}
#line 92
#line 151
inline static void /*SendingMoteAppC.CommandMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__4__AMPacket__setType(message_t * amsg, am_id_t t){
#line 151
  CC2420ActiveMessageP__AMPacket__setType(amsg, t);
#line 151
}
#line 151
# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static error_t /*SendingMoteAppC.CommandMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__4__Send__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(4U, msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t SendingMoteC__sendReading__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SendingMoteC__sendReading);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 1711 "SendingMoteC.nc"
static inline void SendingMoteC__sendReading__runTask(void )
#line 1711
{
  if (!SendingMoteC__busy) {
      CommandMsg *btrpkt = (CommandMsg *)SendingMoteC__Packet__getPayload(&SendingMoteC__pkt_priv, 0);


      __nesc_hton_uint16(btrpkt->command_id.data, 1);
      __nesc_hton_uint16(btrpkt->command_data.data, SendingMoteC__tmpReading);
      __nesc_hton_uint8(btrpkt->command_code.data, COMMAND_SENSORREADING);
      __nesc_hton_uint16(btrpkt->command_data_next[0].data, SendingMoteC__readMode);


      SendingMoteC__setPower(&SendingMoteC__pkt_priv, SendingMoteC__tx_power);




      if (SendingMoteC__CommandMsgSend__send(SendingMoteC__sensorReadingDest, &SendingMoteC__pkt_priv, sizeof(CommandMsg )) == SUCCESS) {
          SendingMoteC__busy = TRUE;
        }
      else 
#line 1729
        {
          SendingMoteC__failBlink();
          ;
          SendingMoteC__sendReading__postTask();
        }
    }
  else 
    {
      SendingMoteC__failBlink();
      SendingMoteC__sendReading__postTask();
    }
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t SendingMoteC__sendIdentification__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SendingMoteC__sendIdentification);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 753 "SendingMoteC.nc"
static inline void SendingMoteC__sendIdentification__runTask(void )
#line 753
{
  if (!SendingMoteC__busy) {
      CommandMsg *btrpkt = (CommandMsg *)SendingMoteC__Packet__getPayload(&SendingMoteC__pkt_priv, 0);


      __nesc_hton_uint16(btrpkt->command_id.data, SendingMoteC__counter);


      __nesc_hton_uint8(btrpkt->reply_on_command.data, COMMAND_IDENTIFY);


      __nesc_hton_uint8(btrpkt->command_code.data, COMMAND_ACK);


      __nesc_hton_uint16(btrpkt->command_data.data, SendingMoteC__operationMode);



      __nesc_hton_uint16(btrpkt->command_data_next[0].data, 1);









      __nesc_hton_uint16(btrpkt->command_data_next[1].data, TOS_NODE_ID);


      if (SendingMoteC__tabulaRasa) {
          __nesc_hton_uint16(btrpkt->command_data_next[2].data, 1);



          SendingMoteC__BootupTimer__startOneShot(SendingMoteC__bootupCounter < 1 ? 500 : 5000);
          SendingMoteC__bootupCounter += 1;
        }
      else 
#line 791
        {
          __nesc_hton_uint16(btrpkt->command_data_next[2].data, 0);
        }




      if (SendingMoteC__CommandMsgSend__send(SendingMoteC__baseid, &SendingMoteC__pkt_priv, sizeof(CommandMsg )) == SUCCESS) {
          SendingMoteC__busy = TRUE;
        }
      else {
          SendingMoteC__failBlink();
          ;
          SendingMoteC__sendIdentification__postTask();
        }
    }
  else {
      SendingMoteC__failBlink();
      SendingMoteC__sendIdentification__postTask();
    }
}

# 264 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_uint16(const void * source)
#line 264
{
  const uint8_t *base = source;

#line 266
  return ((uint16_t )base[0] << 8) | base[1];
}

#line 288
static __inline  int16_t __nesc_ntoh_int16(const void * source)
#line 288
{
#line 288
  return __nesc_ntoh_uint16(source);
}

#line 288
static __inline  int16_t __nesc_hton_int16(void * target, int16_t value)
#line 288
{
#line 288
  __nesc_hton_uint16(target, value);
#line 288
  return value;
}

# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static error_t /*SendingMoteAppC.ReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__2__Send__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(2U, msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 151 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static void /*SendingMoteAppC.ReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMPacket__setType(message_t * amsg, am_id_t t){
#line 151
  CC2420ActiveMessageP__AMPacket__setType(amsg, t);
#line 151
}
#line 151
#line 92
inline static void /*SendingMoteAppC.ReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMPacket__setDestination(message_t * amsg, am_addr_t addr){
#line 92
  CC2420ActiveMessageP__AMPacket__setDestination(amsg, addr);
#line 92
}
#line 92
# 45 "/opt/tinyos-2.x/tos/system/AMQueueEntryP.nc"
static inline error_t /*SendingMoteAppC.ReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 47
{
  /*SendingMoteAppC.ReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMPacket__setDestination(msg, dest);
  /*SendingMoteAppC.ReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMPacket__setType(msg, 16);
  return /*SendingMoteAppC.ReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__2__Send__send(msg, len);
}

# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static error_t SendingMoteC__ReportMsgSend__send(am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*SendingMoteAppC.ReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMSend__send(addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static error_t /*SendingMoteAppC.TinyReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__3__Send__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(3U, msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 151 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static void /*SendingMoteAppC.TinyReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMPacket__setType(message_t * amsg, am_id_t t){
#line 151
  CC2420ActiveMessageP__AMPacket__setType(amsg, t);
#line 151
}
#line 151
#line 92
inline static void /*SendingMoteAppC.TinyReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMPacket__setDestination(message_t * amsg, am_addr_t addr){
#line 92
  CC2420ActiveMessageP__AMPacket__setDestination(amsg, addr);
#line 92
}
#line 92
# 45 "/opt/tinyos-2.x/tos/system/AMQueueEntryP.nc"
static inline error_t /*SendingMoteAppC.TinyReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 47
{
  /*SendingMoteAppC.TinyReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMPacket__setDestination(msg, dest);
  /*SendingMoteAppC.TinyReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMPacket__setType(msg, 17);
  return /*SendingMoteAppC.TinyReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__3__Send__send(msg, len);
}

# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static error_t SendingMoteC__TinyReportMsgSend__send(am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*SendingMoteAppC.TinyReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMSend__send(addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 546 "SendingMoteC.nc"
static inline bool SendingMoteC__doQueueSend(void )
#line 546
{
  uint16_t qsize = 0;


  if (SendingMoteC__operationMode == NODE_DEAD) {
#line 550
    return FALSE;
    }

  if (SendingMoteC__doReporting == FALSE) {
#line 553
    return FALSE;
    }

  if (SendingMoteC__radioFull) {
#line 556
    return TRUE;
    }

  if (SendingMoteC__queueTimeout) {
#line 559
    return TRUE;
    }








  qsize = SendingMoteC__queueSize();


  if (SendingMoteC__reportProtocol == REPORTING_TINY && qsize >= 1) {
#line 572
    return TRUE;
    }

  if (SendingMoteC__reportProtocol == REPORTING_MEDIUM && qsize >= SendingMoteC__queueFlushThreshold) {
#line 575
    return TRUE;
    }

  if (SendingMoteC__reportProtocol == REPORTING_MASS && (SendingMoteC__reportMassTrigered || qsize >= SendingMoteC__massReportQueueThreshold)) {
#line 578
    return TRUE;
    }

  return FALSE;
}

#line 610
static inline void SendingMoteC__sendReport__runTask(void )
#line 610
{
  uint8_t tmpOut = 0;
  uint8_t tmpI = 0;

  if (!SendingMoteC__busy) {
      MultiPingResponseReportMsg *btrpkt = (void *)0;
      MultiPingResponseTinyReportMsg *btrpkt_tiny = (void *)0;


      if (!SendingMoteC__doQueueSend()) {
#line 619
        return;
        }


      if (SendingMoteC__radioIn == SendingMoteC__radioOut && !SendingMoteC__radioFull) {
          SendingMoteC__radioBusy = FALSE;
          return;
        }


      SendingMoteC__noiseFloorMeasured = 0;


      if (SendingMoteC__reportProtocol == REPORTING_TINY) {
          btrpkt_tiny = (MultiPingResponseTinyReportMsg *)SendingMoteC__Packet__getPayload(&SendingMoteC__pkt, 0);


          __nesc_hton_uint16(btrpkt_tiny->counter.data, SendingMoteC__counter);


          __nesc_hton_uint16(btrpkt_tiny->nodeid.data, __nesc_ntoh_uint8(SendingMoteC__radioBuff[tmpOut].nodeid.data));
          __nesc_hton_uint16(btrpkt_tiny->nodecounter.data, __nesc_ntoh_uint8(SendingMoteC__radioBuff[tmpOut].nodecounter.data));
          __nesc_hton_int16(btrpkt_tiny->rssi.data, __nesc_ntoh_int16(SendingMoteC__radioBuff[tmpOut].rssi.data));


          SendingMoteC__setPower(&SendingMoteC__pkt, SendingMoteC__base_tx_power);




          if (SendingMoteC__TinyReportMsgSend__send(SendingMoteC__baseid, &SendingMoteC__pkt, sizeof(MultiPingResponseTinyReportMsg )) == SUCCESS) {
              SendingMoteC__busy = TRUE;
              SendingMoteC__counter++;
            }
          else 
            {
              SendingMoteC__failBlink();
              ;


              SendingMoteC__GapTimer__startOneShot(SendingMoteC__reportingGap + SendingMoteC__reportingGap / 5 - SendingMoteC__Random__rand16() % (SendingMoteC__reportingGap / 3));
            }
        }
      else 

        {
          btrpkt = (MultiPingResponseReportMsg *)SendingMoteC__Packet__getPayload(&SendingMoteC__pkt, 0);


          __nesc_hton_uint16(btrpkt->counter.data, SendingMoteC__counter);


          for (tmpOut = SendingMoteC__radioOut, tmpI = 0; tmpOut != SendingMoteC__radioIn && tmpI < SendingMoteC__queueFlushThreshold; tmpI++, tmpOut = (tmpOut + 1) % SendingMoteC__RADIO_QUEUE_LEN) {
              __nesc_hton_uint16(btrpkt->nodeid[tmpI].data, __nesc_ntoh_uint8(SendingMoteC__radioBuff[tmpOut].nodeid.data));
              __nesc_hton_uint16(btrpkt->nodecounter[tmpI].data, __nesc_ntoh_uint8(SendingMoteC__radioBuff[tmpOut].nodecounter.data));
              __nesc_hton_int16(btrpkt->rssi[tmpI].data, __nesc_ntoh_int16(SendingMoteC__radioBuff[tmpOut].rssi.data));

              if (tmpI > SendingMoteC__queueFlushThreshold) {
#line 676
                break;
                }
            }

          __nesc_hton_uint8(btrpkt->datanum.data, tmpI);


          SendingMoteC__setPower(&SendingMoteC__pkt, SendingMoteC__base_tx_power);




          if (SendingMoteC__ReportMsgSend__send(SendingMoteC__baseid, &SendingMoteC__pkt, sizeof(MultiPingResponseReportMsg )) == SUCCESS) {
              SendingMoteC__busy = TRUE;
              SendingMoteC__counter++;
            }
          else 
            {
              SendingMoteC__failBlink();
              ;


              SendingMoteC__GapTimer__startOneShot(SendingMoteC__reportingGap + SendingMoteC__reportingGap / 5 - SendingMoteC__Random__rand16() % (SendingMoteC__reportingGap / 3));
            }
        }
    }
  else 
    {
      SendingMoteC__failBlink();



      SendingMoteC__GapTimer__startOneShot(SendingMoteC__reportingGap + SendingMoteC__reportingGap / 5 - SendingMoteC__Random__rand16() % (SendingMoteC__reportingGap / 3));
    }
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t SendingMoteC__sendMultipleEcho__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SendingMoteC__sendMultipleEcho);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static error_t /*SendingMoteAppC.MultiPingResponseSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__Send__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(1U, msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 151 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static void /*SendingMoteAppC.MultiPingResponseSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMPacket__setType(message_t * amsg, am_id_t t){
#line 151
  CC2420ActiveMessageP__AMPacket__setType(amsg, t);
#line 151
}
#line 151
#line 92
inline static void /*SendingMoteAppC.MultiPingResponseSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMPacket__setDestination(message_t * amsg, am_addr_t addr){
#line 92
  CC2420ActiveMessageP__AMPacket__setDestination(amsg, addr);
#line 92
}
#line 92
# 45 "/opt/tinyos-2.x/tos/system/AMQueueEntryP.nc"
static inline error_t /*SendingMoteAppC.MultiPingResponseSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 47
{
  /*SendingMoteAppC.MultiPingResponseSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMPacket__setDestination(msg, dest);
  /*SendingMoteAppC.MultiPingResponseSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMPacket__setType(msg, 13);
  return /*SendingMoteAppC.MultiPingResponseSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__Send__send(msg, len);
}

# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static error_t SendingMoteC__MultiPingResponseSend__send(am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*SendingMoteAppC.MultiPingResponseSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__AMSend__send(addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 520 "SendingMoteC.nc"
static inline void SendingMoteC__sendMultipleEcho__runTask(void )
#line 520
{
  if (!SendingMoteC__busy) {
      MultiPingResponseMsg *btrpkt = (MultiPingResponseMsg *)SendingMoteC__Packet__getPayload(&SendingMoteC__pkt_response, 0);


      SendingMoteC__setPower(&SendingMoteC__pkt_response, SendingMoteC__tx_power);


      __nesc_hton_uint16(btrpkt->counter.data, SendingMoteC__counter_ping);

      if (SendingMoteC__MultiPingResponseSend__send(AM_BROADCAST_ADDR, &SendingMoteC__pkt_response, sizeof(MultiPingResponseMsg )) == SUCCESS) {
          SendingMoteC__busy = TRUE;
        }
      else {
          SendingMoteC__failBlink();
          SendingMoteC__sendMultipleEcho__postTask();
        }
    }
  else 
#line 537
    {
      SendingMoteC__failBlink();
      SendingMoteC__sendMultipleEcho__postTask();
    }
}

# 198 "/opt/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP__Resource__release(uint8_t id)
#line 198
{
  if (CC2420TinyosNetworkP__TINYOS_N_NETWORKS > 1) {
      CC2420TinyosNetworkP__grantTask__postTask();
    }
  CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__OWNER_NONE;
  return SUCCESS;
}

#line 222
static inline void CC2420TinyosNetworkP__Resource__default__granted(uint8_t client)
#line 222
{
  CC2420TinyosNetworkP__Resource__release(client);
}

# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static void CC2420TinyosNetworkP__Resource__granted(uint8_t arg_0xf6a68628){
#line 92
  switch (arg_0xf6a68628) {
#line 92
    case CC2420ActiveMessageC__CC2420_AM_SEND_ID:
#line 92
      CC2420ActiveMessageP__RadioResource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      CC2420TinyosNetworkP__Resource__default__granted(arg_0xf6a68628);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 58 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void )
#line 58
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 59
    {
      if (/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead != /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY) {
          uint8_t id = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead;

#line 62
          /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead];
          if (/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead == /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY) {
            /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;
            }
#line 65
          /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[id] = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;
          {
            unsigned char __nesc_temp = 
#line 66
            id;

            {
#line 66
              __nesc_atomic_end(__nesc_atomic); 
#line 66
              return __nesc_temp;
            }
          }
        }
#line 68
      {
        unsigned char __nesc_temp = 
#line 68
        /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;

        {
#line 68
          __nesc_atomic_end(__nesc_atomic); 
#line 68
          return __nesc_temp;
        }
      }
    }
#line 71
    __nesc_atomic_end(__nesc_atomic); }
}

# 60 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t CC2420TinyosNetworkP__Queue__dequeue(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 148 "/opt/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP__grantTask__runTask(void )
#line 148
{


  if (CC2420TinyosNetworkP__TINYOS_N_NETWORKS > 1) {
      if (CC2420TinyosNetworkP__resource_owner == CC2420TinyosNetworkP__OWNER_NONE && !CC2420TinyosNetworkP__Queue__isEmpty()) {
          CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__Queue__dequeue();

          if (CC2420TinyosNetworkP__resource_owner != CC2420TinyosNetworkP__OWNER_NONE) {
              CC2420TinyosNetworkP__Resource__granted(CC2420TinyosNetworkP__resource_owner);
            }
        }
    }
  else 
#line 159
    {
      if (CC2420TinyosNetworkP__next_owner != CC2420TinyosNetworkP__resource_owner) {
          CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__next_owner;
          CC2420TinyosNetworkP__Resource__granted(CC2420TinyosNetworkP__resource_owner);
        }
    }
}

# 92 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 47 "/opt/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(t0, dt);
}

#line 82
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 83
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(t0, dt, TRUE);
}

# 118 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt){
#line 118
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(t0, dt);
#line 118
}
#line 118
# 54 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop();
#line 62
}
#line 62
# 91 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__stop();
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__stop();
#line 62
}
#line 62
# 60 "/opt/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void )
{
#line 61
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop();
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop();
#line 67
}
#line 67
# 89 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void )
{




  uint32_t now = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint8_t num;

  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop();

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(now);
        }
      else {
#line 124
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(now, min_remaining);
        }
    }
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t SendingMoteC__sendReport__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SendingMoteC__sendReport);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 1617 "SendingMoteC.nc"
static inline void SendingMoteC__sendQueueFlushCommand(void )
#line 1617
{

  if (SendingMoteC__reportMassTrigered == FALSE) {

      CommandMsg *btrpktresponse = (CommandMsg *)SendingMoteC__Packet__getPayload(&SendingMoteC__pkt_priv, 0);


      SendingMoteC__reportMassTrigered = TRUE;


      __nesc_hton_uint8(btrpktresponse->reply_on_command.data, 0);
      __nesc_hton_uint16(btrpktresponse->reply_on_command_id.data, 0);
      __nesc_hton_uint8(btrpktresponse->command_code.data, COMMAND_FLUSHREPORTQUEUE);
      SendingMoteC__command_dest = 65535U;


      SendingMoteC__sendCommandACK__postTask();
    }
}

#line 432
static inline void SendingMoteC__FlushTimer__fired(void )
#line 432
{
  SendingMoteC__queueTimeout = TRUE;


  if (SendingMoteC__reportProtocol == REPORTING_MASS) {

      if (SendingMoteC__reportMassTrigered == FALSE) {
          SendingMoteC__sendQueueFlushCommand();
        }

      SendingMoteC__reportMassTrigered = TRUE;
    }


  SendingMoteC__sendReport__postTask();
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void SendingMoteC__SendTimer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(2U);
#line 67
}
#line 67
# 468 "SendingMoteC.nc"
static inline void SendingMoteC__SendTimer__fired(void )
#line 468
{



  if (SendingMoteC__packets2send == 0) {
      SendingMoteC__sendMultipleEcho__postTask();
    }
  else 
    {
      if (SendingMoteC__packets2send >= SendingMoteC__counter_ping) {
          SendingMoteC__sendMultipleEcho__postTask();
        }
      else {

          SendingMoteC__SendTimer__stop();


          SendingMoteC__counter_ping = 0;
          SendingMoteC__failBlink();
        }
    }
}






static inline void SendingMoteC__GapTimer__fired(void )
#line 496
{

  SendingMoteC__sendReport__postTask();
}

# 44 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__HplGeneralIO__toggle(void ){
#line 44
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle();
#line 44
}
#line 44
# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__GeneralIO__toggle(void )
#line 39
{
#line 39
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__HplGeneralIO__toggle();
}

# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__toggle(void ){
#line 31
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__GeneralIO__toggle();
#line 31
}
#line 31
# 103 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2Toggle(void )
#line 103
{
  LedsP__Led2__toggle();
  ;
#line 105
  ;
}

# 89 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void SendingMoteC__Leds__led2Toggle(void ){
#line 89
  LedsP__Leds__led2Toggle();
#line 89
}
#line 89
# 44 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__HplGeneralIO__toggle(void ){
#line 44
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__toggle();
#line 44
}
#line 44
# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__GeneralIO__toggle(void )
#line 39
{
#line 39
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__HplGeneralIO__toggle();
}

# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__toggle(void ){
#line 31
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__GeneralIO__toggle();
#line 31
}
#line 31
# 88 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline void LedsP__Leds__led1Toggle(void )
#line 88
{
  LedsP__Led1__toggle();
  ;
#line 90
  ;
}

# 72 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void SendingMoteC__Leds__led1Toggle(void ){
#line 72
  LedsP__Leds__led1Toggle();
#line 72
}
#line 72
# 67 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void SendingMoteC__SignalTimer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(4U);
#line 67
}
#line 67
# 450 "SendingMoteC.nc"
static inline void SendingMoteC__SignalTimer__fired(void )
#line 450
{
  if (SendingMoteC__signalCounter <= 0) {
      SendingMoteC__SignalTimer__stop();
      return;
    }

  SendingMoteC__Leds__led1Toggle();
  SendingMoteC__Leds__led2Toggle();
  SendingMoteC__signalCounter--;
}

# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__RssiResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ControlC.RssiResource*/CC2420SpiC__2__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 371 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__ReadRssi__read(void )
#line 371
{
  return CC2420ControlP__RssiResource__request();
}

# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
inline static error_t SendingMoteC__ReadRssi__read(void ){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420ControlP__ReadRssi__read();
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 1854 "SendingMoteC.nc"
static inline error_t SendingMoteC__readNoiseFloor(void )
#line 1854
{
  if (SendingMoteC__busy == FALSE && SendingMoteC__radioFull == FALSE) {
      SendingMoteC__busy = TRUE;
      return SendingMoteC__ReadRssi__read();
    }

  SendingMoteC__failBlink();
  return FAIL;
}

#line 504
static inline void SendingMoteC__NoiseFloorTimer__fired(void )
#line 504
{
  SendingMoteC__readNoiseFloor();
}




static inline void SendingMoteC__SensorReadingTimer__fired(void )
#line 511
{
  SendingMoteC__readSensors();
}

#line 423
static inline void SendingMoteC__BootupTimer__fired(void )
#line 423
{
  SendingMoteC__sendIdentification__postTask();
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 130 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void )
#line 130
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 131
    {
      if (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__default_owner_id) {
          if (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_GRANTING) {
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask__postTask();
              {
                unsigned char __nesc_temp = 
#line 135
                SUCCESS;

                {
#line 135
                  __nesc_atomic_end(__nesc_atomic); 
#line 135
                  return __nesc_temp;
                }
              }
            }
          else {
#line 137
            if (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_IMM_GRANTING) {
                /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__reqResId;
                /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_BUSY;
                {
                  unsigned char __nesc_temp = 
#line 140
                  SUCCESS;

                  {
#line 140
                    __nesc_atomic_end(__nesc_atomic); 
#line 140
                    return __nesc_temp;
                  }
                }
              }
            }
        }
    }
#line 146
    __nesc_atomic_end(__nesc_atomic); }
#line 144
  return FAIL;
}

# 56 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
inline static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__release(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release();
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 181 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline uint8_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__isOwner(void )
#line 181
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 182
    {
      unsigned char __nesc_temp = 
#line 182
      /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_CONTROLLED
       || (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__default_owner_id
       && (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_GRANTING || /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_IMM_GRANTING));

      {
#line 182
        __nesc_atomic_end(__nesc_atomic); 
#line 182
        return __nesc_temp;
      }
    }
#line 184
    __nesc_atomic_end(__nesc_atomic); }
}

# 65 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
inline static bool /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__isOwner(void ){
#line 65
  unsigned char __nesc_result;
#line 65

#line 65
  __nesc_result = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__isOwner();
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 92 "/opt/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__startDone(error_t error)
#line 92
{
  if (/*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__isOwner()) {
    /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__release();
    }
}

# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static void HplSensirionSht11P__SplitControl__startDone(error_t error){
#line 92
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__startDone(error);
#line 92
}
#line 92
# 59 "/opt/tinyos-2.x/tos/platforms/telosa/chips/sht11/HplSensirionSht11P.nc"
static inline void HplSensirionSht11P__Timer__fired(void )
#line 59
{
  HplSensirionSht11P__SplitControl__startDone(SUCCESS);
}

# 98 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__resetDone(error_t result)
#line 98
{
}



static inline void /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__resetDone(error_t result)
#line 103
{
}

# 406 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__resetDone(uint8_t client, error_t result)
#line 406
{
}

# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__resetDone(uint8_t arg_0xf6823938, error_t result){
#line 54
  switch (arg_0xf6823938) {
#line 54
    case /*SendingMoteAppC.SensirionSht11C*/SensirionSht11C__0__TEMP_KEY:
#line 54
      /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__resetDone(result);
#line 54
      break;
#line 54
    case /*SendingMoteAppC.SensirionSht11C*/SensirionSht11C__0__HUM_KEY:
#line 54
      /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__resetDone(result);
#line 54
      break;
#line 54
    default:
#line 54
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__resetDone(arg_0xf6823938, result);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 287 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__fired(void )
#line 287
{

  switch (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__cmd) {

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_SOFT_RESET: 

        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__resetDone(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__currentClient, SUCCESS);
      break;

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_MEASURE_TEMPERATURE: 

        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureTemperatureDone(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__currentClient, FAIL, 0);
      break;

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_MEASURE_HUMIDITY: 

        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureHumidityDone(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__currentClient, FAIL, 0);
      break;

      default: 

        break;
    }
}

# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static void Msp430RefVoltGeneratorP__RefVolt_2_5V__startDone(error_t error){
#line 92
  Msp430RefVoltArbiterImplP__RefVolt_2_5V__startDone(error);
#line 92
}
#line 92
# 78 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
inline static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__read(uint8_t arg_0xf657ed00, uint32_t usPeriod){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = AdcStreamP__ReadStream__read(arg_0xf657ed00, usPeriod);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 59 "/opt/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc"
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__granted(uint8_t client)
#line 59
{
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__read(client, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__period[client]);
}

# 160 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline void Msp430RefVoltArbiterImplP__ClientResource__default__granted(uint8_t client)
#line 160
{
}

# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static void Msp430RefVoltArbiterImplP__ClientResource__granted(uint8_t arg_0xf66388c0){
#line 92
  switch (arg_0xf66388c0) {
#line 92
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID:
#line 92
      AdcP__ResourceRead__granted(/*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC*/AdcReadClientC__0__CLIENT);
#line 92
      break;
#line 92
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID:
#line 92
      /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__granted(/*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT);
#line 92
      break;
#line 92
    default:
#line 92
      Msp430RefVoltArbiterImplP__ClientResource__default__granted(arg_0xf66388c0);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 98 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline void Msp430RefVoltArbiterImplP__RefVolt_1_5V__startDone(error_t error)
{
  if (Msp430RefVoltArbiterImplP__syncOwner != Msp430RefVoltArbiterImplP__NO_OWNER) {


      Msp430RefVoltArbiterImplP__ClientResource__granted(Msp430RefVoltArbiterImplP__syncOwner);
    }
}

# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static void Msp430RefVoltGeneratorP__RefVolt_1_5V__startDone(error_t error){
#line 92
  Msp430RefVoltArbiterImplP__RefVolt_1_5V__startDone(error);
#line 92
}
#line 92
# 168 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline void Msp430RefVoltGeneratorP__SwitchOnTimer__fired(void )
#line 168
{
  switch (Msp430RefVoltGeneratorP__m_state) {
      case Msp430RefVoltGeneratorP__REFERENCE_1_5V_ON_PENDING: 
        Msp430RefVoltGeneratorP__m_state = Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE;
      Msp430RefVoltGeneratorP__RefVolt_1_5V__startDone(SUCCESS);
      break;

      case Msp430RefVoltGeneratorP__REFERENCE_2_5V_ON_PENDING: 
        Msp430RefVoltGeneratorP__m_state = Msp430RefVoltGeneratorP__REFERENCE_2_5V_STABLE;
      Msp430RefVoltGeneratorP__RefVolt_2_5V__startDone(SUCCESS);
      break;

      default: 
        return;
    }
}

# 151 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline void Msp430RefVoltArbiterImplP__RefVolt_2_5V__stopDone(error_t error)
{
}

# 117 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static void Msp430RefVoltGeneratorP__RefVolt_2_5V__stopDone(error_t error){
#line 117
  Msp430RefVoltArbiterImplP__RefVolt_2_5V__stopDone(error);
#line 117
}
#line 117
# 147 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline void Msp430RefVoltArbiterImplP__RefVolt_1_5V__stopDone(error_t error)
{
}

# 117 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static void Msp430RefVoltGeneratorP__RefVolt_1_5V__stopDone(error_t error){
#line 117
  Msp430RefVoltArbiterImplP__RefVolt_1_5V__stopDone(error);
#line 117
}
#line 117
# 185 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline void Msp430RefVoltGeneratorP__SwitchOffTimer__fired(void )
#line 185
{
  switch (Msp430RefVoltGeneratorP__m_state) {
      case Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE: 
        if (Msp430RefVoltGeneratorP__switchOff() == SUCCESS) {
            Msp430RefVoltGeneratorP__m_state = Msp430RefVoltGeneratorP__GENERATOR_OFF;
            Msp430RefVoltGeneratorP__RefVolt_1_5V__stopDone(SUCCESS);
          }
        else {
            Msp430RefVoltGeneratorP__SwitchOffTimer__startOneShot(20);
          }
      break;

      case Msp430RefVoltGeneratorP__REFERENCE_2_5V_STABLE: 
        if (Msp430RefVoltGeneratorP__switchOff() == SUCCESS) {
            Msp430RefVoltGeneratorP__m_state = Msp430RefVoltGeneratorP__GENERATOR_OFF;
            Msp430RefVoltGeneratorP__RefVolt_2_5V__stopDone(SUCCESS);
          }
        else {
            Msp430RefVoltGeneratorP__SwitchOffTimer__startOneShot(20);
          }
      break;

      default: 
        break;
    }
}

# 193 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num)
{
}

# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(uint8_t arg_0xf6b05030){
#line 72
  switch (arg_0xf6b05030) {
#line 72
    case 1U:
#line 72
      SendingMoteC__FlushTimer__fired();
#line 72
      break;
#line 72
    case 2U:
#line 72
      SendingMoteC__SendTimer__fired();
#line 72
      break;
#line 72
    case 3U:
#line 72
      SendingMoteC__GapTimer__fired();
#line 72
      break;
#line 72
    case 4U:
#line 72
      SendingMoteC__SignalTimer__fired();
#line 72
      break;
#line 72
    case 5U:
#line 72
      SendingMoteC__NoiseFloorTimer__fired();
#line 72
      break;
#line 72
    case 6U:
#line 72
      SendingMoteC__SensorReadingTimer__fired();
#line 72
      break;
#line 72
    case 7U:
#line 72
      SendingMoteC__BootupTimer__fired();
#line 72
      break;
#line 72
    case 8U:
#line 72
      HplSensirionSht11P__Timer__fired();
#line 72
      break;
#line 72
    case 9U:
#line 72
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__fired();
#line 72
      break;
#line 72
    case 10U:
#line 72
      Msp430RefVoltGeneratorP__SwitchOnTimer__fired();
#line 72
      break;
#line 72
    case 11U:
#line 72
      Msp430RefVoltGeneratorP__SwitchOffTimer__fired();
#line 72
      break;
#line 72
    default:
#line 72
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(arg_0xf6b05030);
#line 72
      break;
#line 72
    }
#line 72
}
#line 72
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__request(/*SendingMoteAppC.SensirionSht11C*/SensirionSht11C__0__TEMP_KEY);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 60 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline error_t /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__read(void )
#line 60
{
  /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__request();
  return SUCCESS;
}

# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
inline static error_t SendingMoteC__Temperature__read(void ){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = /*SendingMoteAppC.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__read();
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 161 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline error_t Msp430RefVoltArbiterImplP__AdcResource__default__request(uint8_t client)
{
  return FAIL;
}

# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t Msp430RefVoltArbiterImplP__AdcResource__request(uint8_t arg_0xf66073a0){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  switch (arg_0xf66073a0) {
#line 78
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID:
#line 78
      __nesc_result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__request(/*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID);
#line 78
      break;
#line 78
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID:
#line 78
      __nesc_result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__request(/*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID);
#line 78
      break;
#line 78
    default:
#line 78
      __nesc_result = Msp430RefVoltArbiterImplP__AdcResource__default__request(arg_0xf66073a0);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 53 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline error_t Msp430RefVoltArbiterImplP__ClientResource__request(uint8_t client)
{
  return Msp430RefVoltArbiterImplP__AdcResource__request(client);
}

# 168 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline error_t AdcP__ResourceRead__default__request(uint8_t client)
#line 168
{
#line 168
  return FAIL;
}

# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t AdcP__ResourceRead__request(uint8_t arg_0xf6769670){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  switch (arg_0xf6769670) {
#line 78
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC*/AdcReadClientC__0__CLIENT:
#line 78
      __nesc_result = Msp430RefVoltArbiterImplP__ClientResource__request(/*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID);
#line 78
      break;
#line 78
    default:
#line 78
      __nesc_result = AdcP__ResourceRead__default__request(arg_0xf6769670);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 75 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline error_t AdcP__Read__read(uint8_t client)
{
  return AdcP__ResourceRead__request(client);
}

# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
inline static error_t SendingMoteC__Light__read(void ){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = AdcP__Read__read(/*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC*/AdcReadClientC__0__CLIENT);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 128 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void )
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow());
}

# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void ){
#line 72
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired();
#line 72
}
#line 72
# 80 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getAlarm(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 82
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type __nesc_temp = 
#line 82
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt;

      {
#line 82
        __nesc_atomic_end(__nesc_atomic); 
#line 82
        return __nesc_temp;
      }
    }
#line 84
    __nesc_atomic_end(__nesc_atomic); }
}

# 105 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void ){
#line 105
  unsigned long __nesc_result;
#line 105

#line 105
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getAlarm();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 63 "/opt/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt, FALSE);
    }
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired();
}

# 210 "/opt/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline message_t *CC2420TinyosNetworkP__BareReceive__default__receive(message_t *msg, void *payload, uint8_t len)
#line 210
{
  return msg;
}

# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * CC2420TinyosNetworkP__BareReceive__receive(message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  __nesc_result = CC2420TinyosNetworkP__BareReceive__default__receive(msg, payload, len);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420TinyosNetworkP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 270 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline message_t *CC2420ActiveMessageP__Snoop__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
#line 270
{
  return msg;
}

# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * CC2420ActiveMessageP__Snoop__receive(am_id_t arg_0xf6a1f490, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
    __nesc_result = CC2420ActiveMessageP__Snoop__default__receive(arg_0xf6a1f490, msg, payload, len);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 90 "/opt/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline int8_t CC2420PacketP__CC2420Packet__getRssi(message_t *p_msg)
#line 90
{
  return __nesc_ntoh_uint8(CC2420PacketP__CC2420PacketBody__getMetadata(p_msg)->rssi.data);
}

# 64 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Packet.nc"
inline static int8_t SendingMoteC__CC2420Packet__getRssi(message_t *p_msg){
#line 64
  signed char __nesc_result;
#line 64

#line 64
  __nesc_result = CC2420PacketP__CC2420Packet__getRssi(p_msg);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 1781 "SendingMoteC.nc"
static inline uint16_t SendingMoteC__getRssi(message_t *msg)
#line 1781
{
  return (uint16_t )SendingMoteC__CC2420Packet__getRssi(msg);
}

#line 1404
static inline message_t *SendingMoteC__SimplePingReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 1404
{


  if (len == sizeof(PingMsg )) {
      PingMsg *btrpkt = (PingMsg *)payload;

      if (!SendingMoteC__busy) 
        {

          RssiMsg *btrpktresponse = (RssiMsg *)SendingMoteC__Packet__getPayload(&SendingMoteC__pkt, 0);


          SendingMoteC__rcvBlink();
          SendingMoteC__counter = __nesc_ntoh_uint16(btrpkt->counter.data);


          __nesc_hton_uint8(btrpktresponse->nodeid.data, TOS_NODE_ID);
          __nesc_hton_uint8(btrpktresponse->dstnodeid.data, __nesc_ntoh_uint8(btrpkt->nodeid.data));
          __nesc_hton_uint16(btrpktresponse->counter.data, __nesc_ntoh_uint16(btrpkt->counter.data));
          __nesc_hton_int16(btrpktresponse->rssiFromBase.data, SendingMoteC__getRssi(msg));


          SendingMoteC__tx_power = __nesc_ntoh_uint8(btrpkt->txpower.data);
          SendingMoteC__channel = __nesc_ntoh_uint8(btrpkt->channel.data);

          SendingMoteC__sendEchoReply__postTask();
        }
    }
  else 
    {
      SendingMoteC__failBlink();
    }
  return msg;
}

#line 1368
static inline message_t *SendingMoteC__PingReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 1368
{


  return SendingMoteC__receive(msg, payload, len, AM_MULTIPINGMSG);
}

#line 1448
static inline message_t *SendingMoteC__ResponseReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 1448
{



  if (SendingMoteC__operationMode == NODE_DEAD) {
#line 1452
    return msg;
    }
#line 1453
  if (SendingMoteC__operationMode != NODE_REPORTING) {
#line 1453
    return msg;
    }

  if (SendingMoteC__reportProtocol == REPORTING_MASS && SendingMoteC__reportMassTrigered == TRUE) {
#line 1456
    return msg;
    }
  return SendingMoteC__receive(msg, payload, len, AM_MULTIPINGRESPONSEMSG);
}

# 77 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static am_addr_t SendingMoteC__AMPacket__source(message_t * amsg){
#line 77
  unsigned int __nesc_result;
#line 77

#line 77
  __nesc_result = CC2420ActiveMessageP__AMPacket__source(amsg);
#line 77

#line 77
  return __nesc_result;
#line 77
}
#line 77
# 53 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void SendingMoteC__SensorReadingTimer__startPeriodic(uint32_t dt){
#line 53
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(6U, dt);
#line 53
}
#line 53
#line 67
inline static void SendingMoteC__SensorReadingTimer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(6U);
#line 67
}
#line 67
# 158 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(uint8_t num)
{
  return /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num].isrunning;
}

# 81 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static bool SendingMoteC__SensorReadingTimer__isRunning(void ){
#line 81
  unsigned char __nesc_result;
#line 81

#line 81
  __nesc_result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(6U);
#line 81

#line 81
  return __nesc_result;
#line 81
}
#line 81
# 46 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )53U &= ~(0x01 << 1);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void SendingMoteC__Pin1__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__clr();
#line 39
}
#line 39
# 45 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )53U |= 0x01 << 1;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void SendingMoteC__Pin1__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__set();
#line 34
}
#line 34
# 46 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )53U &= ~(0x01 << 0);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void SendingMoteC__Pin0__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__clr();
#line 39
}
#line 39
# 45 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )53U |= 0x01 << 0;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void SendingMoteC__Pin0__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__set();
#line 34
}
#line 34
# 1749 "SendingMoteC.nc"
static inline bool SendingMoteC__setGIO(uint8_t pin, bool enabled)
#line 1749
{




  if (pin == 0 && enabled == TRUE) {
      SendingMoteC__Pin0__set();
    }
  else {
#line 1756
    if (pin == 0 && enabled == FALSE) {
        SendingMoteC__Pin0__clr();
      }
    else {
#line 1758
      if (pin == 1 && enabled == TRUE) {
          SendingMoteC__Pin1__set();
        }
      else {
#line 1760
        if (pin == 1 && enabled == FALSE) {
            SendingMoteC__Pin1__clr();
          }
        else 
#line 1762
          {
            return FALSE;
          }
        }
      }
    }
#line 1766
  return TRUE;





  return FALSE;
}

# 54 "Reset.h"
static inline void resetMote()
{







  WDTCTL = 0;
}

# 859 "SendingMoteC.nc"
static inline void SendingMoteC__reset_hard(void )
#line 859
{

  SendingMoteC__failBlink();

  resetMote();
}

#line 1095
static inline message_t *SendingMoteC__CommandReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 1095
{
  if (len == sizeof(CommandMsg )) {

      CommandMsg *btrpkt = (CommandMsg *)payload;


      CommandMsg *btrpktresponse = (CommandMsg *)SendingMoteC__Packet__getPayload(&SendingMoteC__pkt_priv, 0);


      __nesc_hton_uint8(btrpktresponse->reply_on_command.data, __nesc_ntoh_uint8(btrpkt->command_code.data));
      __nesc_hton_uint16(btrpktresponse->reply_on_command_id.data, __nesc_ntoh_uint16(btrpkt->command_id.data));
      __nesc_hton_uint8(btrpktresponse->command_code.data, COMMAND_ACK);
      SendingMoteC__command_dest = SendingMoteC__baseid;



      SendingMoteC__tabulaRasa = FALSE;


      switch (__nesc_ntoh_uint8(btrpkt->command_code.data)) {
          case COMMAND_IDENTIFY: 

            SendingMoteC__sendIdentification__postTask();
          SendingMoteC__signalize(2);

          break;

          case COMMAND_RESET: 


            SendingMoteC__reset_hard();



          SendingMoteC__reset_node();

          __nesc_hton_uint8(btrpktresponse->command_code.data, COMMAND_ACK);



          SendingMoteC__signalize(1);

          break;

          case COMMAND_ABORT: 
            SendingMoteC__abort();

          __nesc_hton_uint8(btrpktresponse->command_code.data, COMMAND_ACK);
          SendingMoteC__signalize(2);
          SendingMoteC__sendCommandACK__postTask();
          break;

          case COMMAND_SETTX: 
            SendingMoteC__base_tx_power = __nesc_ntoh_uint16(btrpkt->command_data.data);

          __nesc_hton_uint8(btrpktresponse->command_code.data, COMMAND_ACK);
          SendingMoteC__sendCommandACK__postTask();
          SendingMoteC__signalize(2);
          break;

          case COMMAND_SETBS: 
            SendingMoteC__baseid = __nesc_ntoh_uint16(btrpkt->command_data.data);

          __nesc_hton_uint8(btrpktresponse->command_code.data, COMMAND_ACK);
          SendingMoteC__sendCommandACK__postTask();
          SendingMoteC__signalize(2);
          break;

          case COMMAND_GETREPORTINGSTATUS: 
            __nesc_hton_uint8(btrpktresponse->command_code.data, COMMAND_ACK);
          __nesc_hton_uint16(btrpktresponse->command_data.data, (unsigned int )SendingMoteC__doReporting);
          SendingMoteC__sendCommandACK__postTask();
          SendingMoteC__signalize(2);
          break;

          case COMMAND_SETREPORTINGSTATUS: 
            SendingMoteC__doReporting = (bool )__nesc_ntoh_uint16(btrpkt->command_data.data);

          __nesc_hton_uint8(btrpktresponse->command_code.data, COMMAND_ACK);
          SendingMoteC__sendCommandACK__postTask();
          SendingMoteC__signalize(2);
          break;


          case COMMAND_SETDORANDOMIZEDTHRESHOLDING: 
            SendingMoteC__doRandomizedThresholding = (bool )__nesc_ntoh_uint16(btrpkt->command_data.data);

          __nesc_hton_uint8(btrpktresponse->command_code.data, COMMAND_ACK);
          SendingMoteC__sendCommandACK__postTask();
          SendingMoteC__signalize(2);
          break;


          case COMMAND_SETQUEUEFLUSHTHRESHOLD: 
            if (SendingMoteC__reportProtocol == REPORTING_MEDIUM) {
              SendingMoteC__queueFlushThreshold = __nesc_ntoh_uint16(btrpkt->command_data.data);
              }
            else {
#line 1191
              if (SendingMoteC__reportProtocol == REPORTING_MASS) {
                SendingMoteC__massReportQueueThreshold = __nesc_ntoh_uint16(btrpkt->command_data.data);
                }
              else {
#line 1193
                return msg;
                }
              }
#line 1195
          __nesc_hton_uint8(btrpktresponse->command_code.data, COMMAND_ACK);
          SendingMoteC__sendCommandACK__postTask();
          SendingMoteC__signalize(2);
          break;


          case COMMAND_SETTINYREPORTS: 
            SendingMoteC__reportProtocol = REPORTING_TINY;


          SendingMoteC__queueFlushThreshold = 1;

          __nesc_hton_uint8(btrpktresponse->command_code.data, COMMAND_ACK);
          SendingMoteC__sendCommandACK__postTask();
          SendingMoteC__signalize(2);
          break;

          case COMMAND_SETOPERATIONMODE: 
            SendingMoteC__operationMode = __nesc_ntoh_uint16(btrpkt->command_data.data);

          __nesc_hton_uint8(btrpktresponse->command_code.data, COMMAND_ACK);
          SendingMoteC__sendCommandACK__postTask();
          SendingMoteC__signalize(2);
          break;

          case COMMAND_SETREPORTPROTOCOL: 
            SendingMoteC__reportProtocol = __nesc_ntoh_uint16(btrpkt->command_data.data);

          __nesc_hton_uint8(btrpktresponse->command_code.data, COMMAND_ACK);
          SendingMoteC__sendCommandACK__postTask();
          SendingMoteC__signalize(2);
          break;


          case COMMAND_FLUSHREPORTQUEUE: 

            if (SendingMoteC__operationMode == NODE_DEAD) {
#line 1231
              return msg;
              }
            else {
              if (SendingMoteC__operationMode == NODE_TALKING) {

                  SendingMoteC__abort();


                  SendingMoteC__signalize(1);
                }
              else {



                if (SendingMoteC__operationMode == NODE_REPORTING && SendingMoteC__reportProtocol == REPORTING_MASS && SendingMoteC__reportMassTrigered == FALSE) {


                    SendingMoteC__reportMassTrigered = TRUE;


                    SendingMoteC__signalize(1);


                    SendingMoteC__sendReport__postTask();
                  }
                }
              }
#line 1256
          break;


          case COMMAND_SETNOISEFLOORREADING: 
            SendingMoteC__doNoiseFloorReading = (bool )__nesc_ntoh_uint16(btrpkt->command_data.data);

          __nesc_hton_uint8(btrpktresponse->command_code.data, COMMAND_ACK);
          SendingMoteC__sendCommandACK__postTask();
          SendingMoteC__signalize(2);
          break;


          case COMMAND_SETREPORTGAP: 
            SendingMoteC__reportingGap = __nesc_ntoh_uint16(btrpkt->command_data.data);

          __nesc_hton_uint8(btrpktresponse->command_code.data, COMMAND_ACK);
          SendingMoteC__sendCommandACK__postTask();
          SendingMoteC__signalize(2);
          break;




          case COMMAND_SETPIN: 
            if (SendingMoteC__setGIO((uint8_t )__nesc_ntoh_uint16(btrpkt->command_data_next[0].data), __nesc_ntoh_uint16(btrpkt->command_data.data) == 1 ? TRUE : FALSE)) {
                __nesc_hton_uint8(btrpktresponse->command_code.data, COMMAND_ACK);
                SendingMoteC__sendCommandACK__postTask();
                SendingMoteC__signalize(2);
              }
          break;





          case COMMAND_GETSENSORREADING: 

            SendingMoteC__readMode = (uint8_t )__nesc_ntoh_uint16(btrpkt->command_data.data);





          if (__nesc_ntoh_uint16(btrpkt->command_data_next[0].data) == 1) {
              SendingMoteC__signalize(3);


              if (__nesc_ntoh_uint16(btrpkt->command_data_next[1].data) == 0) {

                  if (SendingMoteC__SensorReadingTimer__isRunning()) {
                      SendingMoteC__SensorReadingTimer__stop();
                    }
                }
              else 
#line 1308
                {

                  SendingMoteC__SensorReadingTimer__startPeriodic(__nesc_ntoh_uint16(btrpkt->command_data_next[1].data));
                }



              if ((__nesc_ntoh_uint16(btrpkt->command_data_next[2].data) & 1) > 0) {

                  SendingMoteC__sensorReadingDest = 65535U;
                }
              else 
#line 1318
                {

                  SendingMoteC__sensorReadingDest = SendingMoteC__command_dest;
                }
            }


          SendingMoteC__readSensors();



          break;




          case COMMAND_SETSAMPLESENSORREADING: 
            SendingMoteC__doSensorReadingSampling = __nesc_ntoh_uint16(btrpkt->command_data.data) > 0 ? TRUE : FALSE;

          __nesc_hton_uint8(btrpktresponse->command_code.data, COMMAND_ACK);
          SendingMoteC__sendCommandACK__postTask();
          SendingMoteC__signalize(2);
          break;



          case COMMAND_SENSORREADING: 
            if (SendingMoteC__doSensorReadingSampling) {

                SendingMoteC__message2sampleReceived(SendingMoteC__getRssi(msg), __nesc_ntoh_uint16(btrpkt->command_id.data), SendingMoteC__AMPacket__source(msg));
              }
            else 
#line 1348
              {


                ;
              }
          break;

          default: 
            SendingMoteC__failBlink();
          SendingMoteC__signalize(1);
        }
    }
  else {
      SendingMoteC__signalize(1);
      SendingMoteC__failBlink();
    }

  return msg;
}

# 266 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline message_t *CC2420ActiveMessageP__Receive__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
#line 266
{
  return msg;
}

# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * CC2420ActiveMessageP__Receive__receive(am_id_t arg_0xf6a21db8, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  switch (arg_0xf6a21db8) {
#line 67
    case 11:
#line 67
      __nesc_result = SendingMoteC__SimplePingReceive__receive(msg, payload, len);
#line 67
      break;
#line 67
    case 12:
#line 67
      __nesc_result = SendingMoteC__PingReceive__receive(msg, payload, len);
#line 67
      break;
#line 67
    case 13:
#line 67
      __nesc_result = SendingMoteC__ResponseReceive__receive(msg, payload, len);
#line 67
      break;
#line 67
    case 14:
#line 67
      __nesc_result = SendingMoteC__CommandReceive__receive(msg, payload, len);
#line 67
      break;
#line 67
    default:
#line 67
      __nesc_result = CC2420ActiveMessageP__Receive__default__receive(arg_0xf6a21db8, msg, payload, len);
#line 67
      break;
#line 67
    }
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 61 "/opt/tinyos-2.x/tos/system/ActiveMessageAddressC.nc"
static inline am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void )
#line 61
{
  return ActiveMessageAddressC__amAddress();
}

# 50 "/opt/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
inline static am_addr_t CC2420ActiveMessageP__ActiveMessageAddress__amAddress(void ){
#line 50
  unsigned int __nesc_result;
#line 50

#line 50
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amAddress();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 122 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline am_addr_t CC2420ActiveMessageP__AMPacket__address(void )
#line 122
{
  return CC2420ActiveMessageP__ActiveMessageAddress__amAddress();
}

#line 146
static inline bool CC2420ActiveMessageP__AMPacket__isForMe(message_t *amsg)
#line 146
{
  return CC2420ActiveMessageP__AMPacket__destination(amsg) == CC2420ActiveMessageP__AMPacket__address() || 
  CC2420ActiveMessageP__AMPacket__destination(amsg) == AM_BROADCAST_ADDR;
}

#line 206
static inline message_t *CC2420ActiveMessageP__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 206
{

  if (CC2420ActiveMessageP__AMPacket__isForMe(msg)) {
      return CC2420ActiveMessageP__Receive__receive(CC2420ActiveMessageP__AMPacket__type(msg), msg, payload, len);
    }
  else {
      return CC2420ActiveMessageP__Snoop__receive(CC2420ActiveMessageP__AMPacket__type(msg), msg, payload, len);
    }
}

# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * CC2420TinyosNetworkP__ActiveReceive__receive(message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  __nesc_result = CC2420ActiveMessageP__SubReceive__receive(msg, payload, len);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 75 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Packet.nc"
inline static uint8_t CC2420TinyosNetworkP__CC2420Packet__getNetwork(message_t *p_msg){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = CC2420PacketP__CC2420Packet__getNetwork(p_msg);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420TinyosNetworkP__CC2420PacketBody__getMetadata(message_t * msg){
#line 47
  nx_struct cc2420_metadata_t *__nesc_result;
#line 47

#line 47
  __nesc_result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
# 127 "/opt/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline message_t *CC2420TinyosNetworkP__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 127
{

  if (! __nesc_ntoh_int8(CC2420TinyosNetworkP__CC2420PacketBody__getMetadata(msg)->crc.data)) {
      return msg;
    }

  if (CC2420TinyosNetworkP__CC2420Packet__getNetwork(msg) == 0x3f) {
      return CC2420TinyosNetworkP__ActiveReceive__receive(msg, payload, len);
    }
  else 
#line 135
    {
      cc2420_header_t *hdr = CC2420TinyosNetworkP__CC2420PacketBody__getHeader(msg);

#line 137
      return CC2420TinyosNetworkP__BareReceive__receive(msg, & hdr->network, len + AM_OVERHEAD);
    }
}

# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * UniqueReceiveP__Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  __nesc_result = CC2420TinyosNetworkP__SubReceive__receive(msg, payload, len);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 137 "/opt/tinyos-2.x/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline void UniqueReceiveP__insert(uint16_t msgSource, uint8_t msgDsn)
#line 137
{
  uint8_t element = UniqueReceiveP__recycleSourceElement;
  bool increment = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 141
    {
      if (element == UniqueReceiveP__INVALID_ELEMENT || UniqueReceiveP__writeIndex == element) {

          element = UniqueReceiveP__writeIndex;
          increment = TRUE;
        }

      UniqueReceiveP__receivedMessages[element].source = msgSource;
      UniqueReceiveP__receivedMessages[element].dsn = msgDsn;
      if (increment) {
          UniqueReceiveP__writeIndex++;
          UniqueReceiveP__writeIndex %= 4;
        }
    }
#line 154
    __nesc_atomic_end(__nesc_atomic); }
}


static inline message_t *UniqueReceiveP__DuplicateReceive__default__receive(message_t *msg, void *payload, uint8_t len)
#line 158
{
  return msg;
}

# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * UniqueReceiveP__DuplicateReceive__receive(message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  __nesc_result = UniqueReceiveP__DuplicateReceive__default__receive(msg, payload, len);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 111 "/opt/tinyos-2.x/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline bool UniqueReceiveP__hasSeen(uint16_t msgSource, uint8_t msgDsn)
#line 111
{
  int i;

#line 113
  UniqueReceiveP__recycleSourceElement = UniqueReceiveP__INVALID_ELEMENT;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 115
    {
      for (i = 0; i < 4; i++) {
          if (UniqueReceiveP__receivedMessages[i].source == msgSource) {
              if (UniqueReceiveP__receivedMessages[i].dsn == msgDsn) {

                  {
                    unsigned char __nesc_temp = 
#line 120
                    TRUE;

                    {
#line 120
                      __nesc_atomic_end(__nesc_atomic); 
#line 120
                      return __nesc_temp;
                    }
                  }
                }
#line 123
              UniqueReceiveP__recycleSourceElement = i;
            }
        }
    }
#line 126
    __nesc_atomic_end(__nesc_atomic); }

  return FALSE;
}

# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * UniqueReceiveP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 85 "/opt/tinyos-2.x/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline message_t *UniqueReceiveP__SubReceive__receive(message_t *msg, void *payload, 
uint8_t len)
#line 86
{
  uint16_t msgSource = __nesc_ntoh_leuint16(UniqueReceiveP__CC2420PacketBody__getHeader(msg)->src.data);
  uint8_t msgDsn = __nesc_ntoh_leuint8(UniqueReceiveP__CC2420PacketBody__getHeader(msg)->dsn.data);

  if (UniqueReceiveP__hasSeen(msgSource, msgDsn)) {
      return UniqueReceiveP__DuplicateReceive__receive(msg, payload, len);
    }
  else {
      UniqueReceiveP__insert(msgSource, msgDsn);
      return UniqueReceiveP__Receive__receive(msg, payload, len);
    }
}

# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * CC2420ReceiveP__Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  __nesc_result = UniqueReceiveP__SubReceive__receive(msg, payload, len);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 64 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static uint16_t CC2420ReceiveP__CC2420Config__getShortAddr(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = CC2420ControlP__CC2420Config__getShortAddr();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 332 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline bool CC2420ControlP__CC2420Config__isAddressRecognitionEnabled(void )
#line 332
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 333
    {
      unsigned char __nesc_temp = 
#line 333
      CC2420ControlP__addressRecognition;

      {
#line 333
        __nesc_atomic_end(__nesc_atomic); 
#line 333
        return __nesc_temp;
      }
    }
#line 335
    __nesc_atomic_end(__nesc_atomic); }
}

# 86 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static bool CC2420ReceiveP__CC2420Config__isAddressRecognitionEnabled(void ){
#line 86
  unsigned char __nesc_result;
#line 86

#line 86
  __nesc_result = CC2420ControlP__CC2420Config__isAddressRecognitionEnabled();
#line 86

#line 86
  return __nesc_result;
#line 86
}
#line 86
# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420ReceiveP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 819 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline bool CC2420ReceiveP__passesAddressCheck(message_t *msg)
#line 819
{
  cc2420_header_t *header = CC2420ReceiveP__CC2420PacketBody__getHeader(msg);

  if (!CC2420ReceiveP__CC2420Config__isAddressRecognitionEnabled()) {
      return TRUE;
    }

  return __nesc_ntoh_leuint16(header->dest.data) == CC2420ReceiveP__CC2420Config__getShortAddr()
   || __nesc_ntoh_leuint16(header->dest.data) == AM_BROADCAST_ADDR;
}

# 47 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420ReceiveP__CC2420PacketBody__getMetadata(message_t * msg){
#line 47
  nx_struct cc2420_metadata_t *__nesc_result;
#line 47

#line 47
  __nesc_result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
# 671 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__receiveDone_task__runTask(void )
#line 671
{
  cc2420_metadata_t *metadata = CC2420ReceiveP__CC2420PacketBody__getMetadata(CC2420ReceiveP__m_p_rx_buf);
  cc2420_header_t *header = CC2420ReceiveP__CC2420PacketBody__getHeader(CC2420ReceiveP__m_p_rx_buf);
  uint8_t length = __nesc_ntoh_leuint8(header->length.data);
  uint8_t tmpLen __attribute((unused))  = sizeof(message_t ) - ((size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
  uint8_t * buf = (uint8_t * )header;

  __nesc_hton_int8(metadata->crc.data, buf[length] >> 7);
  __nesc_hton_uint8(metadata->lqi.data, buf[length] & 0x7f);
  __nesc_hton_uint8(metadata->rssi.data, buf[length - 1]);

  if (CC2420ReceiveP__passesAddressCheck(CC2420ReceiveP__m_p_rx_buf) && length >= CC2420_SIZE) {
#line 696
      CC2420ReceiveP__m_p_rx_buf = CC2420ReceiveP__Receive__receive(CC2420ReceiveP__m_p_rx_buf, CC2420ReceiveP__m_p_rx_buf->data, 
      length - CC2420_SIZE);
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 699
    CC2420ReceiveP__receivingPacket = FALSE;
#line 699
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ReceiveP__waitForNextPacket();
}

# 68 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline void LedsP__Leds__led0Off(void )
#line 68
{
  LedsP__Led0__set();
  ;
#line 70
  ;
}

# 50 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void SendingMoteC__Leds__led0Off(void ){
#line 50
  LedsP__Leds__led0Off();
#line 50
}
#line 50
# 83 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline void LedsP__Leds__led1Off(void )
#line 83
{
  LedsP__Led1__set();
  ;
#line 85
  ;
}

# 66 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void SendingMoteC__Leds__led1Off(void ){
#line 66
  LedsP__Leds__led1Off();
#line 66
}
#line 66
# 98 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2Off(void )
#line 98
{
  LedsP__Led2__set();
  ;
#line 100
  ;
}

# 83 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void SendingMoteC__Leds__led2Off(void ){
#line 83
  LedsP__Leds__led2Off();
#line 83
}
#line 83
# 53 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void SendingMoteC__SignalTimer__startPeriodic(uint32_t dt){
#line 53
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(4U, dt);
#line 53
}
#line 53
#line 67
inline static void SendingMoteC__GapTimer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(3U);
#line 67
}
#line 67
inline static void SendingMoteC__BootupTimer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(7U);
#line 67
}
#line 67
#line 62
inline static void SendingMoteC__FlushTimer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(1U, dt);
#line 62
}
#line 62
# 52 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static error_t SendingMoteC__CC2420Config__sync(void ){
#line 52
  unsigned char __nesc_result;
#line 52

#line 52
  __nesc_result = CC2420ControlP__CC2420Config__sync();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
#line 99
inline static void SendingMoteC__CC2420Config__setAutoAck(bool enableAutoAck, bool hwAutoAck){
#line 99
  CC2420ControlP__CC2420Config__setAutoAck(enableAutoAck, hwAutoAck);
#line 99
}
#line 99
# 1841 "SendingMoteC.nc"
static inline void SendingMoteC__setAutoAck(bool enableAutoAck, bool hwAutoAck)
#line 1841
{
  SendingMoteC__CC2420Config__setAutoAck(enableAutoAck, hwAutoAck);
  SendingMoteC__CC2420Config__sync();
}

# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SyncResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ControlC.SyncSpiC*/CC2420SpiC__1__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 53 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void SendingMoteC__SendTimer__startPeriodic(uint32_t dt){
#line 53
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(2U, dt);
#line 53
}
#line 53
# 178 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline uint8_t CC2420SpiP__Resource__isOwner(uint8_t id)
#line 178
{
  /* atomic removed: atomic calls only */
#line 179
  {
    unsigned char __nesc_temp = 
#line 179
    CC2420SpiP__m_holder == id;

#line 179
    return __nesc_temp;
  }
}

# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static bool CC2420ReceiveP__SpiResource__isOwner(void ){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  __nesc_result = CC2420SpiP__Resource__isOwner(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 118

#line 118
  return __nesc_result;
#line 118
}
#line 118
#line 87
inline static error_t CC2420ReceiveP__SpiResource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = CC2420SpiP__Resource__immediateRequest(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
#line 78
inline static error_t CC2420ReceiveP__SpiResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420SpiP__grant__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420SpiP__grant);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 184 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__SpiResource__granted(void )
#line 184
{
  CC2420SpiP__grant__postTask();
}

# 119 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(uint8_t id)
#line 119
{
}

# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(uint8_t arg_0xf6e5d030){
#line 92
  switch (arg_0xf6e5d030) {
#line 92
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 92
      CC2420SpiP__SpiResource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(arg_0xf6e5d030);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 95 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(uint8_t id)
#line 95
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(id);
}

# 199 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id)
#line 199
{
}

# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(uint8_t arg_0xf6d4ece8){
#line 92
  switch (arg_0xf6d4ece8) {
#line 92
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 92
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(arg_0xf6d4ece8);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 187 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void )
#line 187
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 188
    {
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
    }
#line 191
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
}

# 190 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error)
#line 190
{
}

# 71 "/opt/tinyos-2.x/tos/interfaces/SpiPacket.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(uint8_t arg_0xf6e5c598, uint8_t * txBuf, uint8_t * rxBuf, uint16_t len, error_t error){
#line 71
  switch (arg_0xf6e5c598) {
#line 71
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 71
      CC2420SpiP__SpiPacket__sendDone(txBuf, rxBuf, len, error);
#line 71
      break;
#line 71
    default:
#line 71
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(arg_0xf6e5c598, txBuf, rxBuf, len, error);
#line 71
      break;
#line 71
    }
#line 71
}
#line 71
# 183 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void )
#line 183
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_client, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len, 
  SUCCESS);
}

#line 166
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void )
#line 166
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 167
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone();
#line 167
    __nesc_atomic_end(__nesc_atomic); }
}

# 484 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__TXFIFO__readDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error)
#line 485
{
}

# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420ReceiveP__SpiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ReceiveP__CSN__set(void ){
#line 29
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__GeneralIO__set();
#line 29
}
#line 29
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420ReceiveP__receiveDone_task__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420ReceiveP__receiveDone_task);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 47 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420TransmitP__CC2420PacketBody__getMetadata(message_t * msg){
#line 47
  nx_struct cc2420_metadata_t *__nesc_result;
#line 47

#line 47
  __nesc_result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
# 387 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__CC2420Receive__receive(uint8_t type, message_t *ack_msg)
#line 387
{
  cc2420_header_t *ack_header;
  cc2420_header_t *msg_header;
  cc2420_metadata_t *msg_metadata;
  uint8_t *ack_buf;
  uint8_t length;

  if (type == IEEE154_TYPE_ACK && CC2420TransmitP__m_msg) {
      ack_header = CC2420TransmitP__CC2420PacketBody__getHeader(ack_msg);
      msg_header = CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg);

      if (CC2420TransmitP__m_state == CC2420TransmitP__S_ACK_WAIT && __nesc_ntoh_leuint8(msg_header->dsn.data) == __nesc_ntoh_leuint8(ack_header->dsn.data)) {
          CC2420TransmitP__BackoffTimer__stop();

          msg_metadata = CC2420TransmitP__CC2420PacketBody__getMetadata(CC2420TransmitP__m_msg);
          ack_buf = (uint8_t *)ack_header;
          length = __nesc_ntoh_leuint8(ack_header->length.data);

          __nesc_hton_int8(msg_metadata->ack.data, TRUE);
          __nesc_hton_uint8(msg_metadata->rssi.data, ack_buf[length - 1]);
          __nesc_hton_uint8(msg_metadata->lqi.data, ack_buf[length] & 0x7f);
          CC2420TransmitP__signalDone(SUCCESS);
        }
    }
}

# 63 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc"
inline static void CC2420ReceiveP__CC2420Receive__receive(uint8_t type, message_t * message){
#line 63
  CC2420TransmitP__CC2420Receive__receive(type, message);
#line 63
}
#line 63
# 59 "/opt/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420ReceiveP__PacketTimeStamp__clear(message_t * msg){
#line 59
  CC2420PacketP__PacketTimeStamp32khz__clear(msg);
#line 59
}
#line 59








inline static void CC2420ReceiveP__PacketTimeStamp__set(message_t * msg, CC2420ReceiveP__PacketTimeStamp__size_type value){
#line 67
  CC2420PacketP__PacketTimeStamp32khz__set(msg, value);
#line 67
}
#line 67
# 48 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )32U & (0x01 << 0);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw() != 0;
}

# 59 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__3__HplGeneralIO__get(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 40 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__3__GeneralIO__get(void )
#line 40
{
#line 40
  return /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__3__HplGeneralIO__get();
}

# 32 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static bool CC2420ReceiveP__FIFOP__get(void ){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__3__GeneralIO__get();
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 48 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )32U & (0x01 << 3);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw() != 0;
}

# 59 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__2__HplGeneralIO__get(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 40 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__2__GeneralIO__get(void )
#line 40
{
#line 40
  return /*HplCC2420PinsC.FIFOM*/Msp430GpioC__2__HplGeneralIO__get();
}

# 32 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static bool CC2420ReceiveP__FIFO__get(void ){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = /*HplCC2420PinsC.FIFOM*/Msp430GpioC__2__GeneralIO__get();
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 209 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline error_t CC2420SpiP__Fifo__continueRead(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 210
{
  return CC2420SpiP__SpiPacket__send((void *)0, data, len);
}

# 62 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static error_t CC2420ReceiveP__RXFIFO__continueRead(uint8_t * data, uint8_t length){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = CC2420SpiP__Fifo__continueRead(CC2420_RXFIFO, data, length);
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
#line 51
inline static cc2420_status_t CC2420ReceiveP__RXFIFO__beginRead(uint8_t * data, uint8_t length){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420SpiP__Fifo__beginRead(CC2420_RXFIFO, data, length);
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ReceiveP__CSN__clr(void ){
#line 30
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__GeneralIO__clr();
#line 30
}
#line 30
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ReceiveP__SACK__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SACK);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 359 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline bool CC2420ControlP__CC2420Config__isHwAutoAckDefault(void )
#line 359
{
  /* atomic removed: atomic calls only */
#line 360
  {
    unsigned char __nesc_temp = 
#line 360
    CC2420ControlP__hwAutoAckDefault;

#line 360
    return __nesc_temp;
  }
}

# 105 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static bool CC2420ReceiveP__CC2420Config__isHwAutoAckDefault(void ){
#line 105
  unsigned char __nesc_result;
#line 105

#line 105
  __nesc_result = CC2420ControlP__CC2420Config__isHwAutoAckDefault();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 366 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline bool CC2420ControlP__CC2420Config__isAutoAckEnabled(void )
#line 366
{
  /* atomic removed: atomic calls only */
#line 367
  {
    unsigned char __nesc_temp = 
#line 367
    CC2420ControlP__autoAckEnabled;

#line 367
    return __nesc_temp;
  }
}

# 110 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static bool CC2420ReceiveP__CC2420Config__isAutoAckEnabled(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420ControlP__CC2420Config__isAutoAckEnabled();
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 525 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__RXFIFO__readDone(uint8_t *rx_buf, uint8_t rx_len, 
error_t error)
#line 526
{
  cc2420_header_t *header = CC2420ReceiveP__CC2420PacketBody__getHeader(CC2420ReceiveP__m_p_rx_buf);
  uint8_t tmpLen __attribute((unused))  = sizeof(message_t ) - ((size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
  uint8_t * buf = (uint8_t * )header;

#line 530
  CC2420ReceiveP__rxFrameLength = buf[0];

  switch (CC2420ReceiveP__m_state) {

      case CC2420ReceiveP__S_RX_LENGTH: 
        CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_FCF;



      if (CC2420ReceiveP__rxFrameLength + 1 > CC2420ReceiveP__m_bytes_left) 



        {

          CC2420ReceiveP__flush();
        }
      else {
          if (!CC2420ReceiveP__FIFO__get() && !CC2420ReceiveP__FIFOP__get()) {
              CC2420ReceiveP__m_bytes_left -= CC2420ReceiveP__rxFrameLength + 1;
            }

          if (CC2420ReceiveP__rxFrameLength <= MAC_PACKET_SIZE) {
              if (CC2420ReceiveP__rxFrameLength > 0) {
                  if (CC2420ReceiveP__rxFrameLength > CC2420ReceiveP__SACK_HEADER_LENGTH) {

                      CC2420ReceiveP__RXFIFO__continueRead(buf + 1, CC2420ReceiveP__SACK_HEADER_LENGTH);
                    }
                  else {

                      CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_PAYLOAD;
                      CC2420ReceiveP__RXFIFO__continueRead(buf + 1, CC2420ReceiveP__rxFrameLength);
                    }
                }
              else {
                  /* atomic removed: atomic calls only */
                  CC2420ReceiveP__receivingPacket = FALSE;
                  CC2420ReceiveP__CSN__set();
                  CC2420ReceiveP__SpiResource__release();
                  CC2420ReceiveP__waitForNextPacket();
                }
            }
          else {

              CC2420ReceiveP__flush();
            }
        }
      break;

      case CC2420ReceiveP__S_RX_FCF: 
        CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_PAYLOAD;










      if (CC2420ReceiveP__CC2420Config__isAutoAckEnabled() && !CC2420ReceiveP__CC2420Config__isHwAutoAckDefault()) {



          if (((__nesc_ntoh_leuint16(
#line 592
          header->fcf.data) >> IEEE154_FCF_ACK_REQ) & 0x01) == 1
           && (__nesc_ntoh_leuint16(header->dest.data) == CC2420ReceiveP__CC2420Config__getShortAddr()
           || __nesc_ntoh_leuint16(header->dest.data) == AM_BROADCAST_ADDR)
           && ((__nesc_ntoh_leuint16(header->fcf.data) >> IEEE154_FCF_FRAME_TYPE) & 7) == IEEE154_TYPE_DATA) {

              CC2420ReceiveP__CSN__set();
              CC2420ReceiveP__CSN__clr();
              CC2420ReceiveP__SACK__strobe();
              CC2420ReceiveP__CSN__set();
              CC2420ReceiveP__CSN__clr();
              CC2420ReceiveP__RXFIFO__beginRead(buf + 1 + CC2420ReceiveP__SACK_HEADER_LENGTH, 
              CC2420ReceiveP__rxFrameLength - CC2420ReceiveP__SACK_HEADER_LENGTH);
              return;
            }
        }

      CC2420ReceiveP__RXFIFO__continueRead(buf + 1 + CC2420ReceiveP__SACK_HEADER_LENGTH, 
      CC2420ReceiveP__rxFrameLength - CC2420ReceiveP__SACK_HEADER_LENGTH);
      break;

      case CC2420ReceiveP__S_RX_PAYLOAD: 

        CC2420ReceiveP__CSN__set();
      if (!CC2420ReceiveP__m_missed_packets) {

          CC2420ReceiveP__SpiResource__release();
        }




      if ((((
#line 621
      CC2420ReceiveP__m_missed_packets && CC2420ReceiveP__FIFO__get()) || !CC2420ReceiveP__FIFOP__get())
       || !CC2420ReceiveP__m_timestamp_size)
       || CC2420ReceiveP__rxFrameLength <= 10) {
          CC2420ReceiveP__PacketTimeStamp__clear(CC2420ReceiveP__m_p_rx_buf);
        }
      else {
          if (CC2420ReceiveP__m_timestamp_size == 1) {
            CC2420ReceiveP__PacketTimeStamp__set(CC2420ReceiveP__m_p_rx_buf, CC2420ReceiveP__m_timestamp_queue[CC2420ReceiveP__m_timestamp_head]);
            }
#line 629
          CC2420ReceiveP__m_timestamp_head = (CC2420ReceiveP__m_timestamp_head + 1) % CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE;
          CC2420ReceiveP__m_timestamp_size--;

          if (CC2420ReceiveP__m_timestamp_size > 0) {
              CC2420ReceiveP__PacketTimeStamp__clear(CC2420ReceiveP__m_p_rx_buf);
              CC2420ReceiveP__m_timestamp_head = 0;
              CC2420ReceiveP__m_timestamp_size = 0;
            }
        }



      if (buf[CC2420ReceiveP__rxFrameLength] >> 7 && rx_buf) {
          uint8_t type = (__nesc_ntoh_leuint16(header->fcf.data) >> IEEE154_FCF_FRAME_TYPE) & 7;

#line 643
          CC2420ReceiveP__CC2420Receive__receive(type, CC2420ReceiveP__m_p_rx_buf);
          if (type == IEEE154_TYPE_DATA) {
              CC2420ReceiveP__receiveDone_task__postTask();
              return;
            }
        }

      CC2420ReceiveP__waitForNextPacket();
      break;

      default: /* atomic removed: atomic calls only */
        CC2420ReceiveP__receivingPacket = FALSE;
      CC2420ReceiveP__CSN__set();
      CC2420ReceiveP__SpiResource__release();
      break;
    }
}

# 370 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Fifo__default__readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error)
#line 370
{
}

# 71 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static void CC2420SpiP__Fifo__readDone(uint8_t arg_0xf6ef53f8, uint8_t * data, uint8_t length, error_t error){
#line 71
  switch (arg_0xf6ef53f8) {
#line 71
    case CC2420_TXFIFO:
#line 71
      CC2420TransmitP__TXFIFO__readDone(data, length, error);
#line 71
      break;
#line 71
    case CC2420_RXFIFO:
#line 71
      CC2420ReceiveP__RXFIFO__readDone(data, length, error);
#line 71
      break;
#line 71
    default:
#line 71
      CC2420SpiP__Fifo__default__readDone(arg_0xf6ef53f8, data, length, error);
#line 71
      break;
#line 71
    }
#line 71
}
#line 71
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ReceiveP__SFLUSHRX__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SFLUSHRX);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 280 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__RadioBackoff__default__requestInitialBackoff(am_id_t id, 
message_t *msg)
#line 281
{
}

# 81 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420ActiveMessageP__RadioBackoff__requestInitialBackoff(am_id_t arg_0xf6a1d4b8, message_t * msg){
#line 81
    CC2420ActiveMessageP__RadioBackoff__default__requestInitialBackoff(arg_0xf6a1d4b8, msg);
#line 81
}
#line 81
# 228 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__SubBackoff__requestInitialBackoff(message_t *msg)
#line 228
{
  CC2420ActiveMessageP__RadioBackoff__requestInitialBackoff(__nesc_ntoh_leuint8(((cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t )))->type.data), msg);
}

# 81 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__RadioBackoff__requestInitialBackoff(message_t * msg){
#line 81
  CC2420ActiveMessageP__SubBackoff__requestInitialBackoff(msg);
#line 81
}
#line 81
# 243 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__RadioBackoff__setInitialBackoff(uint16_t backoffTime)
#line 243
{
  CC2420TransmitP__myInitialBackoff = backoffTime + 1;
}

# 60 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__SubBackoff__setInitialBackoff(uint16_t backoffTime){
#line 60
  CC2420TransmitP__RadioBackoff__setInitialBackoff(backoffTime);
#line 60
}
#line 60
# 220 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__SubBackoff__requestInitialBackoff(message_t *msg)
#line 220
{
  CC2420CsmaP__SubBackoff__setInitialBackoff(CC2420CsmaP__Random__rand16()
   % (0x1F * CC2420_BACKOFF_PERIOD) + CC2420_MIN_BACKOFF);

  CC2420CsmaP__RadioBackoff__requestInitialBackoff(msg);
}

# 81 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420TransmitP__RadioBackoff__requestInitialBackoff(message_t * msg){
#line 81
  CC2420CsmaP__SubBackoff__requestInitialBackoff(msg);
#line 81
}
#line 81
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420CsmaP__sendDone_task__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420CsmaP__sendDone_task);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 202 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__CC2420Transmit__sendDone(message_t *p_msg, error_t err)
#line 202
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 203
    CC2420CsmaP__sendErr = err;
#line 203
    __nesc_atomic_end(__nesc_atomic); }
  CC2420CsmaP__sendDone_task__postTask();
}

# 73 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
inline static void CC2420TransmitP__Send__sendDone(message_t * p_msg, error_t error){
#line 73
  CC2420CsmaP__CC2420Transmit__sendDone(p_msg, error);
#line 73
}
#line 73
# 452 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__TXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error)
#line 453
{

  CC2420TransmitP__CSN__set();
  if (CC2420TransmitP__m_state == CC2420TransmitP__S_CANCEL) {
      /* atomic removed: atomic calls only */
#line 457
      {
        CC2420TransmitP__CSN__clr();
        CC2420TransmitP__SFLUSHTX__strobe();
        CC2420TransmitP__CSN__set();
      }
      CC2420TransmitP__releaseSpiResource();
      CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
      CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, ECANCEL);
    }
  else {
#line 466
    if (!CC2420TransmitP__m_cca) {
        /* atomic removed: atomic calls only */
#line 467
        {
          CC2420TransmitP__m_state = CC2420TransmitP__S_BEGIN_TRANSMIT;
        }
        CC2420TransmitP__attemptSend();
      }
    else {
        CC2420TransmitP__releaseSpiResource();
        /* atomic removed: atomic calls only */
#line 474
        {
          CC2420TransmitP__m_state = CC2420TransmitP__S_SAMPLE_CCA;
        }

        CC2420TransmitP__RadioBackoff__requestInitialBackoff(CC2420TransmitP__m_msg);
        CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__myInitialBackoff);
      }
    }
}

# 663 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__RXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 663
{
}

# 373 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Fifo__default__writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 373
{
}

# 91 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static void CC2420SpiP__Fifo__writeDone(uint8_t arg_0xf6ef53f8, uint8_t * data, uint8_t length, error_t error){
#line 91
  switch (arg_0xf6ef53f8) {
#line 91
    case CC2420_TXFIFO:
#line 91
      CC2420TransmitP__TXFIFO__writeDone(data, length, error);
#line 91
      break;
#line 91
    case CC2420_RXFIFO:
#line 91
      CC2420ReceiveP__RXFIFO__writeDone(data, length, error);
#line 91
      break;
#line 91
    default:
#line 91
      CC2420SpiP__Fifo__default__writeDone(arg_0xf6ef53f8, data, length, error);
#line 91
      break;
#line 91
    }
#line 91
}
#line 91
# 55 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__RXCTRL1__write(uint16_t data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP__Reg__write(CC2420_RXCTRL1, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
inline static cc2420_status_t CC2420ControlP__IOCFG0__write(uint16_t data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP__Reg__write(CC2420_IOCFG0, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ControlP__SXOSCON__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SXOSCON);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 79 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__enable(void )
#line 79
{
#line 79
  P1IE |= 1 << 4;
}

# 31 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__enable(void ){
#line 31
  HplMsp430InterruptP__Port14__enable();
#line 31
}
#line 31
# 131 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__edge(bool l2h)
#line 131
{
  /* atomic removed: atomic calls only */
#line 132
  {
    if (l2h) {
#line 133
      P1IES &= ~(1 << 4);
      }
    else {
#line 134
      P1IES |= 1 << 4;
      }
  }
}

# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__edge(bool low_to_high){
#line 56
  HplMsp430InterruptP__Port14__edge(low_to_high);
#line 56
}
#line 56
# 95 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__clear(void )
#line 95
{
#line 95
  P1IFG &= ~(1 << 4);
}

# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port14__clear();
#line 41
}
#line 41
# 87 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__disable(void )
#line 87
{
#line 87
  P1IE &= ~(1 << 4);
}

# 36 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__disable(void ){
#line 36
  HplMsp430InterruptP__Port14__disable();
#line 36
}
#line 36
# 58 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__disable();
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear();
  }
  return SUCCESS;
}

#line 41
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__enable(bool rising)
#line 41
{
  /* atomic removed: atomic calls only */
#line 42
  {
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable();
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__edge(rising);
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__enable();
  }
  return SUCCESS;
}

static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void )
#line 50
{
  return /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__enable(TRUE);
}

# 42 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ControlP__InterruptCCA__enableRisingEdge(void ){
#line 42
  unsigned char __nesc_result;
#line 42

#line 42
  __nesc_result = /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge();
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 55 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__IOCFG1__write(uint16_t data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP__Reg__write(CC2420_IOCFG1, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 207 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__startOscillator(void )
#line 207
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 208
    {
      if (CC2420ControlP__m_state != CC2420ControlP__S_VREG_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 210
            FAIL;

            {
#line 210
              __nesc_atomic_end(__nesc_atomic); 
#line 210
              return __nesc_temp;
            }
          }
        }
#line 213
      CC2420ControlP__m_state = CC2420ControlP__S_XOSC_STARTING;
      CC2420ControlP__IOCFG1__write(CC2420_SFDMUX_XOSC16M_STABLE << 
      CC2420_IOCFG1_CCAMUX);

      CC2420ControlP__InterruptCCA__enableRisingEdge();
      CC2420ControlP__SXOSCON__strobe();

      CC2420ControlP__IOCFG0__write((1 << CC2420_IOCFG0_FIFOP_POLARITY) | (
      127 << CC2420_IOCFG0_FIFOP_THR));

      CC2420ControlP__writeFsctrl();
      CC2420ControlP__writeMdmctrl0();

      CC2420ControlP__RXCTRL1__write(((((((1 << CC2420_RXCTRL1_RXBPF_LOCUR) | (
      1 << CC2420_RXCTRL1_LOW_LOWGAIN)) | (
      1 << CC2420_RXCTRL1_HIGH_HGM)) | (
      1 << CC2420_RXCTRL1_LNA_CAP_ARRAY)) | (
      1 << CC2420_RXCTRL1_RXMIX_TAIL)) | (
      1 << CC2420_RXCTRL1_RXMIX_VCM)) | (
      2 << CC2420_RXCTRL1_RXMIX_CURRENT));
    }
#line 233
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 71 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__startOscillator(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = CC2420ControlP__CC2420Power__startOscillator();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 211 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__Resource__granted(void )
#line 211
{
  CC2420CsmaP__CC2420Power__startOscillator();
}

# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static void CC2420ControlP__Resource__granted(void ){
#line 92
  CC2420CsmaP__Resource__granted();
#line 92
}
#line 92
# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__clr(void ){
#line 30
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__GeneralIO__clr();
#line 30
}
#line 30
# 390 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__SpiResource__granted(void )
#line 390
{
  CC2420ControlP__CSN__clr();
  CC2420ControlP__Resource__granted();
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420ControlP__syncDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420ControlP__syncDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SyncResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.SyncSpiC*/CC2420SpiC__1__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__set(void ){
#line 29
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__GeneralIO__set();
#line 29
}
#line 29
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ControlP__SRXON__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SRXON);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
inline static cc2420_status_t CC2420ControlP__SRFOFF__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SRFOFF);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 376 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__SyncResource__granted(void )
#line 376
{
  CC2420ControlP__CSN__clr();
  CC2420ControlP__SRFOFF__strobe();
  CC2420ControlP__writeFsctrl();
  CC2420ControlP__writeMdmctrl0();
  CC2420ControlP__writeId();
  CC2420ControlP__CSN__set();
  CC2420ControlP__CSN__clr();
  CC2420ControlP__SRXON__strobe();
  CC2420ControlP__CSN__set();
  CC2420ControlP__SyncResource__release();
  CC2420ControlP__syncDone__postTask();
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void SendingMoteC__NoiseFloorTimer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(5U);
#line 67
}
#line 67
# 1865 "SendingMoteC.nc"
static inline void SendingMoteC__ReadRssi__readDone(error_t error, uint16_t data)
#line 1865
{
  SendingMoteC__busy = FALSE;

  if (error == SUCCESS) {
      SendingMoteC__Leds__led2Toggle();



      __nesc_hton_uint8(SendingMoteC__radioBuff[SendingMoteC__radioIn].nodeid.data, TOS_NODE_ID);
      __nesc_hton_uint8(SendingMoteC__radioBuff[SendingMoteC__radioIn].nodecounter.data, SendingMoteC__noiseFloorMeasured);
      __nesc_hton_int16(SendingMoteC__radioBuff[SendingMoteC__radioIn].rssi.data, data);


      if (++SendingMoteC__radioIn >= SendingMoteC__RADIO_QUEUE_LEN) {
        SendingMoteC__radioIn = 0;
        }
#line 1880
      if (SendingMoteC__radioIn == SendingMoteC__radioOut) {
        SendingMoteC__radioFull = TRUE;
        }

      SendingMoteC__noiseFloorMeasured++;


      SendingMoteC__NoiseFloorTimer__startOneShot(SendingMoteC__NOISEFLOOR_DELAY);
    }
  else {
      SendingMoteC__failBlink();

      SendingMoteC__NoiseFloorTimer__startOneShot(SendingMoteC__NOISEFLOOR_DELAY);
    }


  if (SendingMoteC__noiseFloorMeasured >= SendingMoteC__NOISEFLOOR_READINGS) {

      SendingMoteC__NoiseFloorTimer__stop();


      SendingMoteC__GapTimer__startOneShot(SendingMoteC__REPORT_SEND_GAP);
    }
}

# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
inline static void CC2420ControlP__ReadRssi__readDone(error_t result, CC2420ControlP__ReadRssi__val_t val){
#line 63
  SendingMoteC__ReadRssi__readDone(result, val);
#line 63
}
#line 63
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__RssiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.RssiResource*/CC2420SpiC__2__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 287 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline cc2420_status_t CC2420SpiP__Reg__read(uint8_t addr, uint16_t *data)
#line 287
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 291
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 293
            status;

            {
#line 293
              __nesc_atomic_end(__nesc_atomic); 
#line 293
              return __nesc_temp;
            }
          }
        }
    }
#line 297
    __nesc_atomic_end(__nesc_atomic); }
#line 297
  status = CC2420SpiP__SpiByte__write(addr | 0x40);
  *data = (uint16_t )CC2420SpiP__SpiByte__write(0) << 8;
  *data |= CC2420SpiP__SpiByte__write(0);

  return status;
}

# 47 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__RSSI__read(uint16_t *data){
#line 47
  unsigned char __nesc_result;
#line 47

#line 47
  __nesc_result = CC2420SpiP__Reg__read(CC2420_RSSI, data);
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
# 395 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__RssiResource__granted(void )
#line 395
{
  uint16_t data = 0;

#line 397
  CC2420ControlP__CSN__clr();
  CC2420ControlP__RSSI__read(&data);
  CC2420ControlP__CSN__set();

  CC2420ControlP__RssiResource__release();
  data += 0x7f;
  data &= 0x00ff;
  CC2420ControlP__ReadRssi__readDone(SUCCESS, data);
}

# 414 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__SpiResource__granted(void )
#line 414
{
  uint8_t cur_state;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 417
    {
      cur_state = CC2420TransmitP__m_state;
    }
#line 419
    __nesc_atomic_end(__nesc_atomic); }

  switch (cur_state) {
      case CC2420TransmitP__S_LOAD: 
        CC2420TransmitP__loadTXFIFO();
      break;

      case CC2420TransmitP__S_BEGIN_TRANSMIT: 
        CC2420TransmitP__attemptSend();
      break;

      case CC2420TransmitP__S_CANCEL: 
        CC2420TransmitP__CSN__clr();
      CC2420TransmitP__SFLUSHTX__strobe();
      CC2420TransmitP__CSN__set();
      CC2420TransmitP__releaseSpiResource();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 435
        {
          CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
        }
#line 437
        __nesc_atomic_end(__nesc_atomic); }
      CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, ECANCEL);
      break;

      default: 
        CC2420TransmitP__releaseSpiResource();
      break;
    }
}

# 508 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__SpiResource__granted(void )
#line 508
{







  CC2420ReceiveP__receive();
}

# 367 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Resource__default__granted(uint8_t id)
#line 367
{
}

# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static void CC2420SpiP__Resource__granted(uint8_t arg_0xf6ef6910){
#line 92
  switch (arg_0xf6ef6910) {
#line 92
    case /*CC2420ControlC.Spi*/CC2420SpiC__0__CLIENT_ID:
#line 92
      CC2420ControlP__SpiResource__granted();
#line 92
      break;
#line 92
    case /*CC2420ControlC.SyncSpiC*/CC2420SpiC__1__CLIENT_ID:
#line 92
      CC2420ControlP__SyncResource__granted();
#line 92
      break;
#line 92
    case /*CC2420ControlC.RssiResource*/CC2420SpiC__2__CLIENT_ID:
#line 92
      CC2420ControlP__RssiResource__granted();
#line 92
      break;
#line 92
    case /*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID:
#line 92
      CC2420TransmitP__SpiResource__granted();
#line 92
      break;
#line 92
    case /*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID:
#line 92
      CC2420ReceiveP__SpiResource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      CC2420SpiP__Resource__default__granted(arg_0xf6ef6910);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 358 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__grant__runTask(void )
#line 358
{
  uint8_t holder;

#line 360
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 360
    {
      holder = CC2420SpiP__m_holder;
    }
#line 362
    __nesc_atomic_end(__nesc_atomic); }
  CC2420SpiP__Resource__granted(holder);
}

# 55 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__FSCTRL__write(uint16_t data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP__Reg__write(CC2420_FSCTRL, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
inline static cc2420_status_t CC2420ControlP__MDMCTRL0__write(uint16_t data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP__Reg__write(CC2420_MDMCTRL0, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 63 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc"
inline static cc2420_status_t CC2420ControlP__PANID__write(uint8_t offset, uint8_t * data, uint8_t length){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Ram__write(CC2420_RAM_PANID, offset, data, length);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 1819 "SendingMoteC.nc"
static inline void SendingMoteC__CC2420Config__syncDone(error_t error)
#line 1819
{
}

# 222 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__CC2420Config__syncDone(error_t error)
#line 222
{
}

# 704 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__CC2420Config__syncDone(error_t error)
#line 704
{
}

# 53 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static void CC2420ControlP__CC2420Config__syncDone(error_t error){
#line 53
  CC2420ReceiveP__CC2420Config__syncDone(error);
#line 53
  CC2420ActiveMessageP__CC2420Config__syncDone(error);
#line 53
  SendingMoteC__CC2420Config__syncDone(error);
#line 53
}
#line 53
# 446 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__syncDone__runTask(void )
#line 446
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 447
    CC2420ControlP__m_sync_busy = FALSE;
#line 447
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP__CC2420Config__syncDone(SUCCESS);
}

#line 442
static inline void CC2420ControlP__sync__runTask(void )
#line 442
{
  CC2420ControlP__CC2420Config__sync();
}

# 213 "/opt/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP__BareSend__default__sendDone(message_t *msg, error_t error)
#line 213
{
}

# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static void CC2420TinyosNetworkP__BareSend__sendDone(message_t * msg, error_t error){
#line 89
  CC2420TinyosNetworkP__BareSend__default__sendDone(msg, error);
#line 89
}
#line 89
# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static void CC2420ActiveMessageP__AMSend__sendDone(am_id_t arg_0xf6a213f8, message_t * msg, error_t error){
#line 99
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(arg_0xf6a213f8, msg, error);
#line 99
}
#line 99
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420ActiveMessageP__RadioResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420TinyosNetworkP__Resource__release(CC2420ActiveMessageC__CC2420_AM_SEND_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 199 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__SubSend__sendDone(message_t *msg, error_t result)
#line 199
{
  CC2420ActiveMessageP__RadioResource__release();
  CC2420ActiveMessageP__AMSend__sendDone(CC2420ActiveMessageP__AMPacket__type(msg), msg, result);
}

# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static void CC2420TinyosNetworkP__ActiveSend__sendDone(message_t * msg, error_t error){
#line 89
  CC2420ActiveMessageP__SubSend__sendDone(msg, error);
#line 89
}
#line 89
# 118 "/opt/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP__SubSend__sendDone(message_t *msg, error_t error)
#line 118
{
  if (CC2420TinyosNetworkP__CC2420Packet__getNetwork(msg) == 0x3f) {
      CC2420TinyosNetworkP__ActiveSend__sendDone(msg, error);
    }
  else 
#line 121
    {
      CC2420TinyosNetworkP__BareSend__sendDone(msg, error);
    }
}

# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static void UniqueSendP__Send__sendDone(message_t * msg, error_t error){
#line 89
  CC2420TinyosNetworkP__SubSend__sendDone(msg, error);
#line 89
}
#line 89
# 104 "/opt/tinyos-2.x/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline void UniqueSendP__SubSend__sendDone(message_t *msg, error_t error)
#line 104
{
  UniqueSendP__State__toIdle();
  UniqueSendP__Send__sendDone(msg, error);
}

# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static void CC2420CsmaP__Send__sendDone(message_t * msg, error_t error){
#line 89
  UniqueSendP__SubSend__sendDone(msg, error);
#line 89
}
#line 89
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420CsmaP__stopDone_task__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420CsmaP__stopDone_task);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 46 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )29U &= ~(0x01 << 5);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__6__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__6__GeneralIO__clr(void )
#line 38
{
#line 38
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__6__HplGeneralIO__clr();
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__clr(void ){
#line 30
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__6__GeneralIO__clr();
#line 30
}
#line 30
# 199 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__stopVReg(void )
#line 199
{
  CC2420ControlP__m_state = CC2420ControlP__S_VREG_STOPPED;
  CC2420ControlP__RSTN__clr();
  CC2420ControlP__VREN__clr();
  CC2420ControlP__RSTN__set();
  return SUCCESS;
}

# 63 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__stopVReg(void ){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420ControlP__CC2420Power__stopVReg();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 58 "/opt/tinyos-2.x/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 91 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__clear(void )
#line 91
{
#line 91
  P1IFG &= ~(1 << 0);
}

# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port10__clear();
#line 41
}
#line 41
# 83 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__disable(void )
#line 83
{
#line 83
  P1IE &= ~(1 << 0);
}

# 36 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__disable(void ){
#line 36
  HplMsp430InterruptP__Port10__disable();
#line 36
}
#line 36
# 58 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__disable();
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear();
  }
  return SUCCESS;
}

# 50 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ReceiveP__InterruptFIFOP__disable(void ){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 171 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP__StdControl__stop(void )
#line 171
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 172
    {
      CC2420ReceiveP__m_state = CC2420ReceiveP__S_STOPPED;
      CC2420ReceiveP__reset_state();
      CC2420ReceiveP__CSN__set();
      CC2420ReceiveP__InterruptFIFOP__disable();
    }
#line 177
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )31U &= ~(0x01 << 1);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc();
#line 85
}
#line 85
# 124 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void )
{
  * (volatile uint16_t * )388U &= ~0x0010;
}

# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents();
#line 47
}
#line 47
# 58 "/opt/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectIOFunc();
  }
}

# 55 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
inline static void CC2420TransmitP__CaptureSFD__disable(void ){
#line 55
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable();
#line 55
}
#line 55
# 179 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__StdControl__stop(void )
#line 179
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 180
    {
      CC2420TransmitP__m_state = CC2420TransmitP__S_STOPPED;
      CC2420TransmitP__BackoffTimer__stop();
      CC2420TransmitP__CaptureSFD__disable();
      CC2420TransmitP__SpiResource__release();
      CC2420TransmitP__CSN__set();
    }
#line 186
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 84 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
inline static error_t CC2420CsmaP__SubControl__stop(void ){
#line 84
  unsigned char __nesc_result;
#line 84

#line 84
  __nesc_result = CC2420TransmitP__StdControl__stop();
#line 84
  __nesc_result = ecombine(__nesc_result, CC2420ReceiveP__StdControl__stop());
#line 84

#line 84
  return __nesc_result;
#line 84
}
#line 84
# 272 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__shutdown(void )
#line 272
{
  CC2420CsmaP__SubControl__stop();
  CC2420CsmaP__CC2420Power__stopVReg();
  CC2420CsmaP__stopDone_task__postTask();
}

#line 241
static inline void CC2420CsmaP__sendDone_task__runTask(void )
#line 241
{
  error_t packetErr;

#line 243
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 243
    packetErr = CC2420CsmaP__sendErr;
#line 243
    __nesc_atomic_end(__nesc_atomic); }
  if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STOPPING)) {
      CC2420CsmaP__shutdown();
    }
  else {
      CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STARTED);
    }

  CC2420CsmaP__Send__sendDone(CC2420CsmaP__m_msg, packetErr);
}

# 417 "SendingMoteC.nc"
static inline void SendingMoteC__RadioControl__stopDone(error_t result)
#line 417
{
}

# 117 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static void CC2420CsmaP__SplitControl__stopDone(error_t error){
#line 117
  SendingMoteC__RadioControl__stopDone(error);
#line 117
}
#line 117
# 262 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__stopDone_task__runTask(void )
#line 262
{
  CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STOPPED);
  CC2420CsmaP__SplitControl__stopDone(SUCCESS);
}

# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static error_t SendingMoteC__RadioControl__start(void ){
#line 83
  unsigned char __nesc_result;
#line 83

#line 83
  __nesc_result = CC2420CsmaP__SplitControl__start();
#line 83

#line 83
  return __nesc_result;
#line 83
}
#line 83
# 403 "SendingMoteC.nc"
static inline void SendingMoteC__RadioControl__startDone(error_t result)
#line 403
{

  if (result == SUCCESS) {
      SendingMoteC__busy = FALSE;



      SendingMoteC__BootupTimer__startOneShot(500);
    }
  else {
      SendingMoteC__RadioControl__start();
    }
}

# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static void CC2420CsmaP__SplitControl__startDone(error_t error){
#line 92
  SendingMoteC__RadioControl__startDone(error);
#line 92
}
#line 92
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SpiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.Spi*/CC2420SpiC__0__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 179 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Resource__release(void )
#line 179
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 180
    {
      CC2420ControlP__CSN__set();
      {
        unsigned char __nesc_temp = 
#line 182
        CC2420ControlP__SpiResource__release();

        {
#line 182
          __nesc_atomic_end(__nesc_atomic); 
#line 182
          return __nesc_temp;
        }
      }
    }
#line 185
    __nesc_atomic_end(__nesc_atomic); }
}

# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420CsmaP__Resource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420ControlP__Resource__release();
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 249 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__rxOn(void )
#line 249
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 250
    {
      if (CC2420ControlP__m_state != CC2420ControlP__S_XOSC_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 252
            FAIL;

            {
#line 252
              __nesc_atomic_end(__nesc_atomic); 
#line 252
              return __nesc_temp;
            }
          }
        }
#line 254
      CC2420ControlP__SRXON__strobe();
    }
#line 255
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 90 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__rxOn(void ){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = CC2420ControlP__CC2420Power__rxOn();
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 75 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__enable(void )
#line 75
{
#line 75
  P1IE |= 1 << 0;
}

# 31 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__enable(void ){
#line 31
  HplMsp430InterruptP__Port10__enable();
#line 31
}
#line 31
# 107 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__edge(bool l2h)
#line 107
{
  /* atomic removed: atomic calls only */
#line 108
  {
    if (l2h) {
#line 109
      P1IES &= ~(1 << 0);
      }
    else {
#line 110
      P1IES |= 1 << 0;
      }
  }
}

# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__edge(bool low_to_high){
#line 56
  HplMsp430InterruptP__Port10__edge(low_to_high);
#line 56
}
#line 56
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__enable(bool rising)
#line 41
{
  /* atomic removed: atomic calls only */
#line 42
  {
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable();
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__edge(rising);
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__enable();
  }
  return SUCCESS;
}





static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge(void )
#line 54
{
  return /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__enable(FALSE);
}

# 43 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ReceiveP__InterruptFIFOP__enableFallingEdge(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 157 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP__StdControl__start(void )
#line 157
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 158
    {
      CC2420ReceiveP__reset_state();
      CC2420ReceiveP__m_state = CC2420ReceiveP__S_STARTED;
      CC2420ReceiveP__receivingPacket = FALSE;




      CC2420ReceiveP__InterruptFIFOP__enableFallingEdge();
    }
#line 167
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 168 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__StdControl__start(void )
#line 168
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 169
    {
      CC2420TransmitP__CaptureSFD__captureRisingEdge();
      CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
      CC2420TransmitP__m_receiving = FALSE;
      CC2420TransmitP__abortSpiRelease = FALSE;
      CC2420TransmitP__m_tx_power = 0;
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
inline static error_t CC2420CsmaP__SubControl__start(void ){
#line 74
  unsigned char __nesc_result;
#line 74

#line 74
  __nesc_result = CC2420TransmitP__StdControl__start();
#line 74
  __nesc_result = ecombine(__nesc_result, CC2420ReceiveP__StdControl__start());
#line 74

#line 74
  return __nesc_result;
#line 74
}
#line 74
# 254 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__startDone_task__runTask(void )
#line 254
{
  CC2420CsmaP__SubControl__start();
  CC2420CsmaP__CC2420Power__rxOn();
  CC2420CsmaP__Resource__release();
  CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STARTED);
  CC2420CsmaP__SplitControl__startDone(SUCCESS);
}

# 45 "/opt/tinyos-2.x/tos/interfaces/State.nc"
inline static error_t CC2420CsmaP__SplitControlState__requestState(uint8_t reqState){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = StateImplP__State__requestState(1U, reqState);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 55 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void CC2420ControlP__StartupTimer__start(CC2420ControlP__StartupTimer__size_type dt){
#line 55
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(dt);
#line 55
}
#line 55
# 45 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )29U |= 0x01 << 5;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__6__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__6__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__6__HplGeneralIO__set();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__set(void ){
#line 29
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__6__GeneralIO__set();
#line 29
}
#line 29
# 187 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__startVReg(void )
#line 187
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 188
    {
      if (CC2420ControlP__m_state != CC2420ControlP__S_VREG_STOPPED) {
          {
            unsigned char __nesc_temp = 
#line 190
            FAIL;

            {
#line 190
              __nesc_atomic_end(__nesc_atomic); 
#line 190
              return __nesc_temp;
            }
          }
        }
#line 192
      CC2420ControlP__m_state = CC2420ControlP__S_VREG_STARTING;
    }
#line 193
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP__VREN__set();
  CC2420ControlP__StartupTimer__start(CC2420_TIME_VREN);
  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__startVReg(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420ControlP__CC2420Power__startVReg();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 82 "/opt/tinyos-2.x/tos/system/ActiveMessageAddressC.nc"
static inline am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void )
#line 82
{
  am_group_t myGroup;

  /* atomic removed: atomic calls only */
#line 84
  myGroup = ActiveMessageAddressC__group;
  return myGroup;
}

# 55 "/opt/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
inline static am_group_t CC2420ControlP__ActiveMessageAddress__amGroup(void ){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amGroup();
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
#line 50
inline static am_addr_t CC2420ControlP__ActiveMessageAddress__amAddress(void ){
#line 50
  unsigned int __nesc_result;
#line 50

#line 50
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amAddress();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 52 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )30U |= 0x01 << 5;
}

# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__6__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__6__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__6__HplGeneralIO__makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__makeOutput(void ){
#line 35
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__6__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )30U |= 0x01 << 6;
}

# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__4__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__4__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__4__HplGeneralIO__makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__makeOutput(void ){
#line 35
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__4__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )30U |= 0x01 << 2;
}

# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__HplGeneralIO__makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__makeOutput(void ){
#line 35
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__GeneralIO__makeOutput();
#line 35
}
#line 35
# 121 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Init__init(void )
#line 121
{
  CC2420ControlP__CSN__makeOutput();
  CC2420ControlP__RSTN__makeOutput();
  CC2420ControlP__VREN__makeOutput();

  CC2420ControlP__m_short_addr = CC2420ControlP__ActiveMessageAddress__amAddress();
  CC2420ControlP__m_pan = CC2420ControlP__ActiveMessageAddress__amGroup();
  CC2420ControlP__m_tx_power = 31;
  CC2420ControlP__m_channel = 26;



  CC2420ControlP__addressRecognition = FALSE;







  CC2420ControlP__hwAddressRecognition = FALSE;




  CC2420ControlP__autoAckEnabled = FALSE;








  CC2420ControlP__hwAutoAckDefault = FALSE;



  return SUCCESS;
}

# 81 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
static inline error_t StateImplP__Init__init(void )
#line 81
{
  int i;

#line 83
  for (i = 0; i < 4U; i++) {
      StateImplP__state[i] = StateImplP__S_IDLE;
    }
  return SUCCESS;
}

# 45 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void )
#line 45
{
  memset(/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY, sizeof /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ);
  return SUCCESS;
}

# 46 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4415 {
#line 46
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(x);
}

#line 94
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )386U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl();
}

# 36 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void ){
#line 36
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare();
#line 36
}
#line 36
# 42 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Init__init(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )30U &= ~(0x01 << 1);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__5__HplGeneralIO__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput();
#line 64
}
#line 64
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.SFDM*/Msp430GpioC__5__GeneralIO__makeInput(void )
#line 41
{
#line 41
  /*HplCC2420PinsC.SFDM*/Msp430GpioC__5__HplGeneralIO__makeInput();
}

# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__SFD__makeInput(void ){
#line 33
  /*HplCC2420PinsC.SFDM*/Msp430GpioC__5__GeneralIO__makeInput();
#line 33
}
#line 33


inline static void CC2420TransmitP__CSN__makeOutput(void ){
#line 35
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__1__GeneralIO__makeOutput();
#line 35
}
#line 35
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )34U &= ~(0x01 << 4);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__0__HplGeneralIO__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput();
#line 64
}
#line 64
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CCAM*/Msp430GpioC__0__GeneralIO__makeInput(void )
#line 41
{
#line 41
  /*HplCC2420PinsC.CCAM*/Msp430GpioC__0__HplGeneralIO__makeInput();
}

# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CCA__makeInput(void ){
#line 33
  /*HplCC2420PinsC.CCAM*/Msp430GpioC__0__GeneralIO__makeInput();
#line 33
}
#line 33
# 160 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__Init__init(void )
#line 160
{
  CC2420TransmitP__CCA__makeInput();
  CC2420TransmitP__CSN__makeOutput();
  CC2420TransmitP__SFD__makeInput();
  return SUCCESS;
}

# 151 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP__Init__init(void )
#line 151
{
  CC2420ReceiveP__m_p_rx_buf = &CC2420ReceiveP__m_rx_buf;
  return SUCCESS;
}

# 46 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__CC2int(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4416 {
#line 46
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__CC2int(x);
}

#line 94
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )390U = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__compareControl();
}

# 36 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare(void ){
#line 36
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__setControlAsCompare();
#line 36
}
#line 36
# 42 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Init__init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 44 "/opt/tinyos-2.x/tos/system/RandomMlcgC.nc"
static inline error_t RandomMlcgC__Init__init(void )
#line 44
{
  /* atomic removed: atomic calls only */
#line 45
  RandomMlcgC__seed = (uint32_t )(TOS_NODE_ID + 1);

  return SUCCESS;
}

# 41 "/opt/tinyos-2.x/tos/interfaces/Random.nc"
inline static uint16_t UniqueSendP__Random__rand16(void ){
#line 41
  unsigned int __nesc_result;
#line 41

#line 41
  __nesc_result = RandomMlcgC__Random__rand16();
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 62 "/opt/tinyos-2.x/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline error_t UniqueSendP__Init__init(void )
#line 62
{
  UniqueSendP__localSendId = UniqueSendP__Random__rand16();
  return SUCCESS;
}

# 71 "/opt/tinyos-2.x/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline error_t UniqueReceiveP__Init__init(void )
#line 71
{
  int i;

#line 73
  for (i = 0; i < 4; i++) {
      UniqueReceiveP__receivedMessages[i].source = (am_addr_t )0xFFFF;
      UniqueReceiveP__receivedMessages[i].dsn = 0;
    }
  return SUCCESS;
}

# 45 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init(void )
#line 45
{
  memset(/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ, /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY, sizeof /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ);
  return SUCCESS;
}

#line 45
static inline error_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__Init__init(void )
#line 45
{
  memset(/*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__resQ, /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY, sizeof /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__resQ);
  return SUCCESS;
}

# 123 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP__HplAdc12__stopConversion(void ){
#line 123
  HplAdc12P__HplAdc12__stopConversion();
#line 123
}
#line 123
# 92 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline error_t Msp430Adc12ImplP__Init__init(void )
{
  adc12ctl0_t ctl0;

#line 95
  Msp430Adc12ImplP__HplAdc12__stopConversion();
  ctl0 = Msp430Adc12ImplP__HplAdc12__getCtl0();
  ctl0.adc12tovie = 1;
  ctl0.adc12ovie = 1;
  Msp430Adc12ImplP__HplAdc12__setCtl0(ctl0);
  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc"
static inline error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__Init__init(void )
#line 51
{
  memset(/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ, 0, sizeof /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ);
  return SUCCESS;
}

# 83 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline error_t AdcStreamP__Init__init(void )
#line 83
{
  uint8_t i;

  for (i = 0; i != AdcStreamP__NSTREAM; i++) 
    AdcStreamP__bufferQueueEnd[i] = &AdcStreamP__bufferQueue[i];

  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = AdcStreamP__Init__init();
#line 51
  __nesc_result = ecombine(__nesc_result, /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, Msp430Adc12ImplP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, UniqueReceiveP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, UniqueSendP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, RandomMlcgC__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, CC2420ReceiveP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, CC2420TransmitP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, StateImplP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, CC2420ControlP__Init__init());
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 375 "SendingMoteC.nc"
static inline void SendingMoteC__Boot__booted(void )
#line 375
{
  uint8_t i = 0;

#line 377
  SendingMoteC__busy = TRUE;


  for (i = 0; i < SendingMoteC__RADIO_QUEUE_LEN; i++) {
      __nesc_hton_uint8(SendingMoteC__radioBuff[i].nodeid.data, 0);
      __nesc_hton_uint8(SendingMoteC__radioBuff[i].nodecounter.data, 0);
      __nesc_hton_int16(SendingMoteC__radioBuff[i].rssi.data, 0);
    }
  SendingMoteC__radioIn = SendingMoteC__radioOut = 0;
  SendingMoteC__radioBusy = FALSE;
  SendingMoteC__radioFull = TRUE;

  SendingMoteC__RadioControl__start();


  SendingMoteC__reset_node();



  SendingMoteC__signalize(2);
}

# 49 "/opt/tinyos-2.x/tos/interfaces/Boot.nc"
inline static void RealMainP__Boot__booted(void ){
#line 49
  SendingMoteC__Boot__booted();
#line 49
}
#line 49
# 206 "/opt/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_disable_interrupt(void )
{
   __asm volatile ("dint");
   __asm volatile ("nop");}

# 52 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void )
#line 52
{
  return MSP430_POWER_LPM3;
}

# 54 "/opt/tinyos-2.x/tos/interfaces/McuPowerOverride.nc"
inline static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = Msp430ClockP__McuPowerOverride__lowestState();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 66 "/opt/tinyos-2.x/tos/chips/msp430/McuSleepC.nc"
static inline mcu_power_t McuSleepC__getPowerState(void )
#line 66
{
  mcu_power_t pState = MSP430_POWER_LPM4;









  if ((((((
#line 69
  TA0CCTL0 & 0x0010 || 
  TA0CCTL1 & 0x0010) || 
  TA0CCTL2 & 0x0010) && (
  TA0CTL & (3 << 8)) == 2 << 8) || (
  ME1 & ((1 << 7) | (1 << 6)) && U0TCTL & 0x20)) || (
  ME2 & ((1 << 5) | (1 << 4)) && U1TCTL & 0x20))


   || (U0CTLnr & 0x01 && I2CTCTLnr & 0x20 && 
  I2CDCTLnr & 0x20 && U0CTLnr & 0x04 && U0CTLnr & 0x20)) {


    pState = MSP430_POWER_LPM1;
    }


  if (ADC12CTL0 & 0x0010) {
      if (ADC12CTL1 & (2 << 3)) {

          if (ADC12CTL1 & (1 << 3)) {
            pState = MSP430_POWER_LPM1;
            }
          else {
#line 91
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
#line 92
        if (ADC12CTL1 & 0x0400 && (TA0CTL & (3 << 8)) == 2 << 8) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

# 194 "/opt/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 194
{
  return m1 < m2 ? m1 : m2;
}

# 104 "/opt/tinyos-2.x/tos/chips/msp430/McuSleepC.nc"
static inline void McuSleepC__computePowerState(void )
#line 104
{
  McuSleepC__powerState = mcombine(McuSleepC__getPowerState(), 
  McuSleepC__McuPowerOverride__lowestState());
}

static inline void McuSleepC__McuSleep__sleep(void )
#line 109
{
  uint16_t temp;

#line 111
  if (McuSleepC__dirty) {
      McuSleepC__computePowerState();
    }

  temp = McuSleepC__msp430PowerBits[McuSleepC__powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));

   __asm volatile ("" :  :  : "memory");
  __nesc_disable_interrupt();
}

# 59 "/opt/tinyos-2.x/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP__McuSleep__sleep(void ){
#line 59
  McuSleepC__McuSleep__sleep();
#line 59
}
#line 59
# 67 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP__popTask(void )
{
  if (SchedulerBasicP__m_head != SchedulerBasicP__NO_TASK) 
    {
      uint8_t id = SchedulerBasicP__m_head;

#line 72
      SchedulerBasicP__m_head = SchedulerBasicP__m_next[SchedulerBasicP__m_head];
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
        }
      SchedulerBasicP__m_next[id] = SchedulerBasicP__NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP__NO_TASK;
    }
}

#line 138
static inline void SchedulerBasicP__Scheduler__taskLoop(void )
{
  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP__popTask()) == SchedulerBasicP__NO_TASK) 
            {
              SchedulerBasicP__McuSleep__sleep();
            }
        }
#line 150
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP__TaskBasic__runTask(nextTask);
    }
}

# 61 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__taskLoop(void ){
#line 61
  SchedulerBasicP__Scheduler__taskLoop();
#line 61
}
#line 61
# 212 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__InterruptFIFOP__fired(void )
#line 212
{
  if (CC2420ReceiveP__m_state == CC2420ReceiveP__S_STARTED) {

      CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_LENGTH;
      CC2420ReceiveP__beginReceive();
    }
  else 



    {
      CC2420ReceiveP__m_missed_packets++;
    }
}

# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__fired(void ){
#line 57
  CC2420ReceiveP__InterruptFIFOP__fired();
#line 57
}
#line 57
# 66 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired(void )
#line 66
{
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear();
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__fired();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port10__fired(void ){
#line 61
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired();
#line 61
}
#line 61
# 92 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port11__clear(void )
#line 92
{
#line 92
  P1IFG &= ~(1 << 1);
}

#line 68
static inline void HplMsp430InterruptP__Port11__default__fired(void )
#line 68
{
#line 68
  HplMsp430InterruptP__Port11__clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port11__fired(void ){
#line 61
  HplMsp430InterruptP__Port11__default__fired();
#line 61
}
#line 61
# 93 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port12__clear(void )
#line 93
{
#line 93
  P1IFG &= ~(1 << 2);
}

#line 69
static inline void HplMsp430InterruptP__Port12__default__fired(void )
#line 69
{
#line 69
  HplMsp430InterruptP__Port12__clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port12__fired(void ){
#line 61
  HplMsp430InterruptP__Port12__default__fired();
#line 61
}
#line 61
# 94 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port13__clear(void )
#line 94
{
#line 94
  P1IFG &= ~(1 << 3);
}

#line 70
static inline void HplMsp430InterruptP__Port13__default__fired(void )
#line 70
{
#line 70
  HplMsp430InterruptP__Port13__clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port13__fired(void ){
#line 61
  HplMsp430InterruptP__Port13__default__fired();
#line 61
}
#line 61
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420CsmaP__startDone_task__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420CsmaP__startDone_task);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 215 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__CC2420Power__startOscillatorDone(void )
#line 215
{
  CC2420CsmaP__startDone_task__postTask();
}

# 76 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static void CC2420ControlP__CC2420Power__startOscillatorDone(void ){
#line 76
  CC2420CsmaP__CC2420Power__startOscillatorDone();
#line 76
}
#line 76
# 50 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ControlP__InterruptCCA__disable(void ){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 418 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__InterruptCCA__fired(void )
#line 418
{
  CC2420ControlP__m_state = CC2420ControlP__S_XOSC_STARTED;
  CC2420ControlP__InterruptCCA__disable();
  CC2420ControlP__IOCFG1__write(0);
  CC2420ControlP__writeId();
  CC2420ControlP__CSN__set();
  CC2420ControlP__CSN__clr();
  CC2420ControlP__CC2420Power__startOscillatorDone();
}

# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__fired(void ){
#line 57
  CC2420ControlP__InterruptCCA__fired();
#line 57
}
#line 57
# 66 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired(void )
#line 66
{
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear();
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__fired();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port14__fired(void ){
#line 61
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired();
#line 61
}
#line 61
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readSensor__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readSensor);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 315 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__fired(void )
#line 315
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__disable();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readSensor__postTask();
}

# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__fired(void ){
#line 57
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__fired();
#line 57
}
#line 57
# 96 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port15__clear(void )
#line 96
{
#line 96
  P1IFG &= ~(1 << 5);
}

# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port15__clear();
#line 41
}
#line 41
# 66 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__fired(void )
#line 66
{
  /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__clear();
  /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__fired();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port15__fired(void ){
#line 61
  /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__fired();
#line 61
}
#line 61
# 97 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port16__clear(void )
#line 97
{
#line 97
  P1IFG &= ~(1 << 6);
}

#line 73
static inline void HplMsp430InterruptP__Port16__default__fired(void )
#line 73
{
#line 73
  HplMsp430InterruptP__Port16__clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port16__fired(void ){
#line 61
  HplMsp430InterruptP__Port16__default__fired();
#line 61
}
#line 61
# 98 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port17__clear(void )
#line 98
{
#line 98
  P1IFG &= ~(1 << 7);
}

#line 74
static inline void HplMsp430InterruptP__Port17__default__fired(void )
#line 74
{
#line 74
  HplMsp430InterruptP__Port17__clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port17__fired(void ){
#line 61
  HplMsp430InterruptP__Port17__default__fired();
#line 61
}
#line 61
# 195 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port20__clear(void )
#line 195
{
#line 195
  P2IFG &= ~(1 << 0);
}

#line 171
static inline void HplMsp430InterruptP__Port20__default__fired(void )
#line 171
{
#line 171
  HplMsp430InterruptP__Port20__clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port20__fired(void ){
#line 61
  HplMsp430InterruptP__Port20__default__fired();
#line 61
}
#line 61
# 196 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port21__clear(void )
#line 196
{
#line 196
  P2IFG &= ~(1 << 1);
}

#line 172
static inline void HplMsp430InterruptP__Port21__default__fired(void )
#line 172
{
#line 172
  HplMsp430InterruptP__Port21__clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port21__fired(void ){
#line 61
  HplMsp430InterruptP__Port21__default__fired();
#line 61
}
#line 61
# 197 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port22__clear(void )
#line 197
{
#line 197
  P2IFG &= ~(1 << 2);
}

#line 173
static inline void HplMsp430InterruptP__Port22__default__fired(void )
#line 173
{
#line 173
  HplMsp430InterruptP__Port22__clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port22__fired(void ){
#line 61
  HplMsp430InterruptP__Port22__default__fired();
#line 61
}
#line 61
# 198 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port23__clear(void )
#line 198
{
#line 198
  P2IFG &= ~(1 << 3);
}

#line 174
static inline void HplMsp430InterruptP__Port23__default__fired(void )
#line 174
{
#line 174
  HplMsp430InterruptP__Port23__clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port23__fired(void ){
#line 61
  HplMsp430InterruptP__Port23__default__fired();
#line 61
}
#line 61
# 199 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port24__clear(void )
#line 199
{
#line 199
  P2IFG &= ~(1 << 4);
}

#line 175
static inline void HplMsp430InterruptP__Port24__default__fired(void )
#line 175
{
#line 175
  HplMsp430InterruptP__Port24__clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port24__fired(void ){
#line 61
  HplMsp430InterruptP__Port24__default__fired();
#line 61
}
#line 61
# 200 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port25__clear(void )
#line 200
{
#line 200
  P2IFG &= ~(1 << 5);
}

#line 176
static inline void HplMsp430InterruptP__Port25__default__fired(void )
#line 176
{
#line 176
  HplMsp430InterruptP__Port25__clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port25__fired(void ){
#line 61
  HplMsp430InterruptP__Port25__default__fired();
#line 61
}
#line 61
# 201 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port26__clear(void )
#line 201
{
#line 201
  P2IFG &= ~(1 << 6);
}

#line 177
static inline void HplMsp430InterruptP__Port26__default__fired(void )
#line 177
{
#line 177
  HplMsp430InterruptP__Port26__clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port26__fired(void ){
#line 61
  HplMsp430InterruptP__Port26__default__fired();
#line 61
}
#line 61
# 202 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port27__clear(void )
#line 202
{
#line 202
  P2IFG &= ~(1 << 7);
}

#line 178
static inline void HplMsp430InterruptP__Port27__default__fired(void )
#line 178
{
#line 178
  HplMsp430InterruptP__Port27__clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port27__fired(void ){
#line 61
  HplMsp430InterruptP__Port27__default__fired();
#line 61
}
#line 61
# 88 "/opt/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId();
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 349 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__disableRxIntr(void )
#line 349
{
  HplMsp430Usart0P__IE1 &= ~(1 << 6);
}

# 177 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr(void ){
#line 177
  HplMsp430Usart0P__Usart__disableRxIntr();
#line 177
}
#line 177
# 170 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data)
#line 170
{

  if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf) {
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos - 1] = data;
    }
  if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos < /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len) {
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp();
    }
  else 
#line 177
    {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr();
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone();
    }
}

# 65 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 65
{
}

# 54 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(uint8_t arg_0xf6d72758, uint8_t data){
#line 54
  switch (arg_0xf6d72758) {
#line 54
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 54
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(data);
#line 54
      break;
#line 54
    default:
#line 54
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(arg_0xf6d72758, data);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 80 "/opt/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
inline static bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void ){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse();
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 54 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data)
#line 54
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(), data);
    }
}

# 54 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart0P__Interrupts__rxDone(uint8_t data){
#line 54
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(data);
#line 54
}
#line 54
# 55 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static inline bool HplMsp430I2C0P__HplI2C__isI2C(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 56
  {
    unsigned char __nesc_temp = 
#line 56
    HplMsp430I2C0P__U0CTL & 0x20 && HplMsp430I2C0P__U0CTL & 0x04 && HplMsp430I2C0P__U0CTL & 0x01;

#line 56
    return __nesc_temp;
  }
}

# 6 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static bool HplMsp430Usart0P__HplI2C__isI2C(void ){
#line 6
  unsigned char __nesc_result;
#line 6

#line 6
  __nesc_result = HplMsp430I2C0P__HplI2C__isI2C();
#line 6

#line 6
  return __nesc_result;
#line 6
}
#line 6
# 66 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(uint8_t id)
#line 66
{
}

# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__fired(uint8_t arg_0xf6d71010){
#line 39
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(arg_0xf6d71010);
#line 39
}
#line 39
# 59 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired(void )
#line 59
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__fired(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId());
    }
}

# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static void HplMsp430Usart0P__I2CInterrupts__fired(void ){
#line 39
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired();
#line 39
}
#line 39
# 188 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void )
#line 188
{
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id)
#line 64
{
}

# 49 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(uint8_t arg_0xf6d72758){
#line 49
  switch (arg_0xf6d72758) {
#line 49
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 49
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone();
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(arg_0xf6d72758);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 49 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void )
#line 49
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId());
    }
}

# 49 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart0P__Interrupts__txDone(void ){
#line 49
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone();
#line 49
}
#line 49
# 161 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline uint16_t *AdcP__SingleChannel__multipleDataReady(uint8_t client, 
uint16_t *buf, uint16_t numSamples)
{

  return 0;
}

# 645 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline uint16_t *Msp430Adc12ImplP__SingleChannel__default__multipleDataReady(uint8_t id, 
uint16_t *buf, uint16_t numSamples)
{
  return 0;
}

# 227 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static uint16_t * Msp430Adc12ImplP__SingleChannel__multipleDataReady(uint8_t arg_0xf671e908, uint16_t * buffer, uint16_t numSamples){
#line 227
  unsigned int *__nesc_result;
#line 227

#line 227
  switch (arg_0xf671e908) {
#line 227
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID:
#line 227
      __nesc_result = AdcP__SingleChannel__multipleDataReady(/*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC*/AdcReadClientC__0__CLIENT, buffer, numSamples);
#line 227
      break;
#line 227
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID:
#line 227
      __nesc_result = AdcStreamP__SingleChannel__multipleDataReady(/*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT, buffer, numSamples);
#line 227
      break;
#line 227
    default:
#line 227
      __nesc_result = Msp430Adc12ImplP__SingleChannel__default__multipleDataReady(arg_0xf671e908, buffer, numSamples);
#line 227
      break;
#line 227
    }
#line 227

#line 227
  return __nesc_result;
#line 227
}
#line 227
# 88 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline uint16_t HplAdc12P__HplAdc12__getMem(uint8_t i)
#line 88
{
  return ((int *)0x0140)[i];
}

# 89 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static uint16_t Msp430Adc12ImplP__HplAdc12__getMem(uint8_t idx){
#line 89
  unsigned int __nesc_result;
#line 89

#line 89
  __nesc_result = HplAdc12P__HplAdc12__getMem(idx);
#line 89

#line 89
  return __nesc_result;
#line 89
}
#line 89
# 62 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline  adc12memctl_t HplAdc12P__int2adc12memctl(uint8_t x)
#line 62
{
#line 62
  union __nesc_unnamed4417 {
#line 62
    uint8_t f;
#line 62
    adc12memctl_t t;
  } 
#line 62
  c = { .f = x };

#line 62
  return c.t;
}

#line 84
static inline adc12memctl_t HplAdc12P__HplAdc12__getMCtl(uint8_t i)
#line 84
{
  return HplAdc12P__int2adc12memctl(((char *)0x0080)[i]);
}

# 82 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static adc12memctl_t Msp430Adc12ImplP__HplAdc12__getMCtl(uint8_t idx){
#line 82
  struct __nesc_unnamed4302 __nesc_result;
#line 82

#line 82
  __nesc_result = HplAdc12P__HplAdc12__getMCtl(idx);
#line 82

#line 82
  return __nesc_result;
#line 82
}
#line 82
# 651 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__MultiChannel__default__dataReady(uint8_t id, uint16_t *buffer, uint16_t numSamples)
#line 651
{
}

# 107 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
inline static void Msp430Adc12ImplP__MultiChannel__dataReady(uint8_t arg_0xf671d688, uint16_t *buffer, uint16_t numSamples){
#line 107
    Msp430Adc12ImplP__MultiChannel__default__dataReady(arg_0xf671d688, buffer, numSamples);
#line 107
}
#line 107
# 640 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline error_t Msp430Adc12ImplP__SingleChannel__default__singleDataReady(uint8_t id, uint16_t data)
{
  return FAIL;
}

# 206 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t Msp430Adc12ImplP__SingleChannel__singleDataReady(uint8_t arg_0xf671e908, uint16_t data){
#line 206
  unsigned char __nesc_result;
#line 206

#line 206
  switch (arg_0xf671e908) {
#line 206
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID:
#line 206
      __nesc_result = AdcP__SingleChannel__singleDataReady(/*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadClientC*/AdcReadClientC__0__CLIENT, data);
#line 206
      break;
#line 206
    case /*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID:
#line 206
      __nesc_result = AdcStreamP__SingleChannel__singleDataReady(/*SendingMoteAppC.HamamatsuS10871TsrC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT, data);
#line 206
      break;
#line 206
    default:
#line 206
      __nesc_result = Msp430Adc12ImplP__SingleChannel__default__singleDataReady(arg_0xf671e908, data);
#line 206
      break;
#line 206
    }
#line 206

#line 206
  return __nesc_result;
#line 206
}
#line 206
# 654 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__Overflow__default__conversionTimeOverflow(uint8_t id)
#line 654
{
}

# 54 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
inline static void Msp430Adc12ImplP__Overflow__conversionTimeOverflow(uint8_t arg_0xf671c010){
#line 54
    Msp430Adc12ImplP__Overflow__default__conversionTimeOverflow(arg_0xf671c010);
#line 54
}
#line 54
# 653 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__Overflow__default__memOverflow(uint8_t id)
#line 653
{
}

# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
inline static void Msp430Adc12ImplP__Overflow__memOverflow(uint8_t arg_0xf671c010){
#line 49
    Msp430Adc12ImplP__Overflow__default__memOverflow(arg_0xf671c010);
#line 49
}
#line 49
# 544 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__HplAdc12__conversionDone(uint16_t iv)
{
  bool overflow = FALSE;
  uint16_t *resultBuffer;

  if (iv <= 4) {
      if (iv == 2) {
        Msp430Adc12ImplP__Overflow__memOverflow(Msp430Adc12ImplP__clientID);
        }
      else {
#line 553
        Msp430Adc12ImplP__Overflow__conversionTimeOverflow(Msp430Adc12ImplP__clientID);
        }
      if (! Msp430Adc12ImplP__HplAdc12__getCtl0().msc) {
        overflow = TRUE;
        }
    }
#line 558
  switch (Msp430Adc12ImplP__state & Msp430Adc12ImplP__CONVERSION_MODE_MASK) 
    {
      case Msp430Adc12ImplP__SINGLE_DATA: 
        Msp430Adc12ImplP__stopConversion();
      Msp430Adc12ImplP__SingleChannel__singleDataReady(Msp430Adc12ImplP__clientID, Msp430Adc12ImplP__HplAdc12__getMem(0));
      break;
      case Msp430Adc12ImplP__SINGLE_DATA_REPEAT: 
        {
          error_t repeatContinue;

#line 567
          repeatContinue = Msp430Adc12ImplP__SingleChannel__singleDataReady(Msp430Adc12ImplP__clientID, 
          Msp430Adc12ImplP__HplAdc12__getMem(0));
          if (repeatContinue != SUCCESS) {
            Msp430Adc12ImplP__stopConversion();
            }
#line 571
          break;
        }

      case Msp430Adc12ImplP__MULTI_CHANNEL: 
        {
          uint16_t i = 0;
#line 576
          uint16_t k;

#line 577
          resultBuffer = Msp430Adc12ImplP__resultBufferStart + Msp430Adc12ImplP__resultBufferIndex;
          do {
              * resultBuffer++ = Msp430Adc12ImplP__HplAdc12__getMem(i);
            }
          while (
#line 580
          ++i < Msp430Adc12ImplP__numChannels);
          Msp430Adc12ImplP__resultBufferIndex += Msp430Adc12ImplP__numChannels;
          if (overflow || Msp430Adc12ImplP__resultBufferLength == Msp430Adc12ImplP__resultBufferIndex) {
              Msp430Adc12ImplP__stopConversion();
              resultBuffer -= Msp430Adc12ImplP__resultBufferIndex;
              k = Msp430Adc12ImplP__resultBufferIndex - Msp430Adc12ImplP__numChannels;
              Msp430Adc12ImplP__resultBufferIndex = 0;
              Msp430Adc12ImplP__MultiChannel__dataReady(Msp430Adc12ImplP__clientID, resultBuffer, 
              overflow ? k : Msp430Adc12ImplP__resultBufferLength);
            }
        }
      break;
      case Msp430Adc12ImplP__MULTIPLE_DATA: 
        {
          uint16_t i = 0;
#line 594
          uint16_t length;
#line 594
          uint16_t k;

#line 595
          resultBuffer = Msp430Adc12ImplP__resultBufferStart + Msp430Adc12ImplP__resultBufferIndex;
          if (Msp430Adc12ImplP__resultBufferLength - Msp430Adc12ImplP__resultBufferIndex > 16) {
            length = 16;
            }
          else {
#line 599
            length = Msp430Adc12ImplP__resultBufferLength - Msp430Adc12ImplP__resultBufferIndex;
            }
#line 600
          do {
              * resultBuffer++ = Msp430Adc12ImplP__HplAdc12__getMem(i);
            }
          while (
#line 602
          ++i < length);
          Msp430Adc12ImplP__resultBufferIndex += length;
          if (overflow || Msp430Adc12ImplP__resultBufferLength == Msp430Adc12ImplP__resultBufferIndex) {
              Msp430Adc12ImplP__stopConversion();
              resultBuffer -= Msp430Adc12ImplP__resultBufferIndex;
              k = Msp430Adc12ImplP__resultBufferIndex - length;
              Msp430Adc12ImplP__resultBufferIndex = 0;
              Msp430Adc12ImplP__SingleChannel__multipleDataReady(Msp430Adc12ImplP__clientID, resultBuffer, 
              overflow ? k : Msp430Adc12ImplP__resultBufferLength);
            }
          else {
#line 611
            if (Msp430Adc12ImplP__resultBufferLength - Msp430Adc12ImplP__resultBufferIndex > 15) {
              return;
              }
            else 
#line 613
              {

                adc12memctl_t memctl = Msp430Adc12ImplP__HplAdc12__getMCtl(0);

#line 616
                memctl.eos = 1;
                Msp430Adc12ImplP__HplAdc12__setMCtl(Msp430Adc12ImplP__resultBufferLength - Msp430Adc12ImplP__resultBufferIndex, memctl);
              }
            }
        }
#line 620
      break;
      case Msp430Adc12ImplP__MULTIPLE_DATA_REPEAT: 
        {
          uint8_t i = 0;

#line 624
          resultBuffer = Msp430Adc12ImplP__resultBufferStart;
          do {
              * resultBuffer++ = Msp430Adc12ImplP__HplAdc12__getMem(i);
            }
          while (
#line 627
          ++i < Msp430Adc12ImplP__resultBufferLength);

          Msp430Adc12ImplP__resultBufferStart = Msp430Adc12ImplP__SingleChannel__multipleDataReady(Msp430Adc12ImplP__clientID, 
          resultBuffer - Msp430Adc12ImplP__resultBufferLength, 
          overflow ? 0 : Msp430Adc12ImplP__resultBufferLength);
          if (!Msp430Adc12ImplP__resultBufferStart) {
            Msp430Adc12ImplP__stopConversion();
            }
#line 634
          break;
        }
    }
}

# 213 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline void Msp430RefVoltGeneratorP__HplAdc12__conversionDone(uint16_t iv)
#line 213
{
}

# 112 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void HplAdc12P__HplAdc12__conversionDone(uint16_t iv){
#line 112
  Msp430RefVoltGeneratorP__HplAdc12__conversionDone(iv);
#line 112
  Msp430Adc12ImplP__HplAdc12__conversionDone(iv);
#line 112
}
#line 112
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 0);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port60__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 1);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port61__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 2);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port62__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 3);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port63__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 4);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port64__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 5);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port65__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 6);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port66__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 7);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port67__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectIOFunc();
#line 85
}
#line 85
# 95 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline void HplAdc12P__HplAdc12__resetIFGs(void )
#line 95
{
  HplAdc12P__ADC12IV = 0;
  HplAdc12P__ADC12IFG = 0;
}

# 106 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP__HplAdc12__resetIFGs(void ){
#line 106
  HplAdc12P__HplAdc12__resetIFGs();
#line 106
}
#line 106
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t AdcStreamP__bufferDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(AdcStreamP__bufferDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
inline static error_t AdcP__readDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(AdcP__readDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 178 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline void AdcP__ReadNow__default__readDone(uint8_t client, error_t result, uint16_t val)
#line 178
{
}

# 66 "/opt/tinyos-2.x/tos/interfaces/ReadNow.nc"
inline static void AdcP__ReadNow__readDone(uint8_t arg_0xf6768b08, error_t result, AdcP__ReadNow__val_t val){
#line 66
    AdcP__ReadNow__default__readDone(arg_0xf6768b08, result, val);
#line 66
}
#line 66
# 226 "/opt/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
  __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (({
#line 228
    uint16_t __x;

#line 228
     __asm volatile ("mov	r2, %0" : "=r"((uint16_t )__x));__x;
  }
  )
#line 228
   & 0x0008) != 0;

#line 229
  __nesc_disable_interrupt();
   __asm volatile ("" :  :  : "memory");
  return result;
}

  void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
   __asm volatile ("" :  :  : "memory");
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

# 11 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(12)))  void sig_TIMERA0_VECTOR(void )
#line 11
{
#line 11
  Msp430TimerCommonP__VectorTimerA0__fired();
}

# 169 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired();
    }
}

#line 169
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired();
    }
}

#line 169
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired();
    }
}

# 12 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(10)))  void sig_TIMERA1_VECTOR(void )
#line 12
{
#line 12
  Msp430TimerCommonP__VectorTimerA1__fired();
}

#line 13
__attribute((wakeup)) __attribute((interrupt(26)))  void sig_TIMERB0_VECTOR(void )
#line 13
{
#line 13
  Msp430TimerCommonP__VectorTimerB0__fired();
}

# 135 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n)
{
}

# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(uint8_t arg_0xf721cb28){
#line 28
  switch (arg_0xf721cb28) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired();
#line 28
      break;
#line 28
    case 1:
#line 28
      /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired();
#line 28
      break;
#line 28
    case 2:
#line 28
      /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired();
#line 28
      break;
#line 28
    case 3:
#line 28
      /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired();
#line 28
      break;
#line 28
    case 4:
#line 28
      /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired();
#line 28
      break;
#line 28
    case 5:
#line 28
      /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired();
#line 28
      break;
#line 28
    case 6:
#line 28
      /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired();
#line 28
      break;
#line 28
    case 7:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(arg_0xf721cb28);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 136 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 = t0;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt = dt;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

#line 96
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__set_alarm(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type now = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__get();
#line 98
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type expires;
#line 98
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type remaining;




  expires = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 + /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt;


  remaining = (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type )(expires - now);


  if (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 <= now) 
    {
      if (expires >= /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY) 
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 = now + /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt = remaining - /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      remaining = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY;
    }
  else 
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 += /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt = 0;
    }
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt((/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type )now << 0, 
  (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type )remaining << 0);
}

# 69 "/opt/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get(void )
{
  /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type rv = 0;

#line 72
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*Counter32khz32C.Transform*/TransformCounterC__0__upper_count_type high = /*Counter32khz32C.Transform*/TransformCounterC__0__m_upper;
      /*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type low = /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__get();

#line 76
      if (/*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__get();
        }
      {
        /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type high_to = high;
        /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type low_to = low >> /*Counter32khz32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT;

#line 90
        rv = (high_to << /*Counter32khz32C.Transform*/TransformCounterC__0__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 51 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void )
{




  if (1) {
      /* atomic removed: atomic calls only */
#line 58
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t * )400U;

#line 61
        do {
#line 61
            t0 = t1;
#line 61
            t1 = * (volatile uint16_t * )400U;
          }
        while (
#line 61
        t0 != t1);
        {
          unsigned int __nesc_temp = 
#line 62
          t1;

#line 62
          return __nesc_temp;
        }
      }
    }
  else 
#line 65
    {
      return * (volatile uint16_t * )400U;
    }
}

# 780 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__congestionBackoff(void )
#line 780
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 781
    {
      CC2420TransmitP__RadioBackoff__requestCongestionBackoff(CC2420TransmitP__m_msg);
      CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__myCongestionBackoff);
    }
#line 784
    __nesc_atomic_end(__nesc_atomic); }
}

# 58 "/opt/tinyos-2.x/tos/system/RandomMlcgC.nc"
static uint32_t RandomMlcgC__Random__rand32(void )
#line 58
{
  uint32_t mlcg;
#line 59
  uint32_t p;
#line 59
  uint32_t q;
  uint64_t tmpseed;

#line 61
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      tmpseed = (uint64_t )33614U * (uint64_t )RandomMlcgC__seed;
      q = tmpseed;
      q = q >> 1;
      p = tmpseed >> 32;
      mlcg = p + q;
      if (mlcg & 0x80000000) {
          mlcg = mlcg & 0x7FFFFFFF;
          mlcg++;
        }
      RandomMlcgC__seed = mlcg;
    }
#line 73
    __nesc_atomic_end(__nesc_atomic); }
  return mlcg;
}

# 787 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static error_t CC2420TransmitP__acquireSpiResource(void )
#line 787
{
  error_t error = CC2420TransmitP__SpiResource__immediateRequest();

#line 789
  if (error != SUCCESS) {
      CC2420TransmitP__SpiResource__request();
    }
  return error;
}

# 126 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__immediateRequest(uint8_t id)
#line 126
{
  error_t error;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 129
    {
      if (CC2420SpiP__WorkingState__requestState(CC2420SpiP__S_BUSY) != SUCCESS) {
          {
            unsigned char __nesc_temp = 
#line 131
            EBUSY;

            {
#line 131
              __nesc_atomic_end(__nesc_atomic); 
#line 131
              return __nesc_temp;
            }
          }
        }
      if (CC2420SpiP__SpiResource__isOwner()) {
          CC2420SpiP__m_holder = id;
          error = SUCCESS;
        }
      else {
#line 139
        if ((error = CC2420SpiP__SpiResource__immediateRequest()) == SUCCESS) {
            CC2420SpiP__m_holder = id;
          }
        else {
            CC2420SpiP__WorkingState__toIdle();
          }
        }
    }
#line 146
    __nesc_atomic_end(__nesc_atomic); }
#line 146
  return error;
}

# 96 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
static error_t StateImplP__State__requestState(uint8_t id, uint8_t reqState)
#line 96
{
  error_t returnVal = FAIL;

#line 98
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 98
    {
      if (reqState == StateImplP__S_IDLE || StateImplP__state[id] == StateImplP__S_IDLE) {
          StateImplP__state[id] = reqState;
          returnVal = SUCCESS;
        }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
  return returnVal;
}

# 174 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id)
#line 174
{
  /* atomic removed: atomic calls only */
#line 175
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId == id && /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) {
        unsigned char __nesc_temp = 
#line 176
        TRUE;

#line 176
        return __nesc_temp;
      }
    else 
#line 177
      {
        unsigned char __nesc_temp = 
#line 177
        FALSE;

#line 177
        return __nesc_temp;
      }
  }
}

#line 130
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void )
#line 130
{
  /* atomic removed: atomic calls only */
#line 131
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id) {
        if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING) {
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
            {
              unsigned char __nesc_temp = 
#line 135
              SUCCESS;

#line 135
              return __nesc_temp;
            }
          }
        else {
#line 137
          if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING) {
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
              {
                unsigned char __nesc_temp = 
#line 140
                SUCCESS;

#line 140
                return __nesc_temp;
              }
            }
          }
      }
  }
#line 144
  return FAIL;
}

# 159 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 161
    {
#line 161
      {
        unsigned char __nesc_temp = 
#line 161
        SchedulerBasicP__pushTask(id) ? SUCCESS : EBUSY;

        {
#line 161
          __nesc_atomic_end(__nesc_atomic); 
#line 161
          return __nesc_temp;
        }
      }
    }
#line 164
    __nesc_atomic_end(__nesc_atomic); }
}

# 265 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config)
#line 265
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 267
    {
      HplMsp430Usart0P__Usart__resetUsart(TRUE);
      HplMsp430Usart0P__HplI2C__clearModeI2C();
      HplMsp430Usart0P__Usart__disableUart();
      HplMsp430Usart0P__configSpi(config);
      HplMsp430Usart0P__Usart__enableSpi();
      HplMsp430Usart0P__Usart__resetUsart(FALSE);
      HplMsp430Usart0P__Usart__clrIntr();
      HplMsp430Usart0P__Usart__disableIntr();
    }
#line 276
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 107 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__request(uint8_t id)
#line 107
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 109
    {
      if (CC2420SpiP__WorkingState__requestState(CC2420SpiP__S_BUSY) == SUCCESS) {
          CC2420SpiP__m_holder = id;
          if (CC2420SpiP__SpiResource__isOwner()) {
              CC2420SpiP__grant__postTask();
            }
          else {
              CC2420SpiP__SpiResource__request();
            }
        }
      else {
          CC2420SpiP__m_requests |= 1 << id;
        }
    }
#line 122
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 735 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__attemptSend(void )
#line 735
{
  uint8_t status;
  bool congestion = TRUE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 739
    {
      if (CC2420TransmitP__m_state == CC2420TransmitP__S_CANCEL) {
          CC2420TransmitP__SFLUSHTX__strobe();
          CC2420TransmitP__releaseSpiResource();
          CC2420TransmitP__CSN__set();
          CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
          CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, ECANCEL);
          {
#line 746
            __nesc_atomic_end(__nesc_atomic); 
#line 746
            return;
          }
        }





      CC2420TransmitP__CSN__clr();
      status = CC2420TransmitP__m_cca ? CC2420TransmitP__STXONCCA__strobe() : CC2420TransmitP__STXON__strobe();
      if (!(status & CC2420_STATUS_TX_ACTIVE)) {
          status = CC2420TransmitP__SNOP__strobe();
          if (status & CC2420_STATUS_TX_ACTIVE) {
              congestion = FALSE;
            }
        }

      CC2420TransmitP__m_state = congestion ? CC2420TransmitP__S_SAMPLE_CCA : CC2420TransmitP__S_SFD;
      CC2420TransmitP__CSN__set();
    }
#line 765
    __nesc_atomic_end(__nesc_atomic); }

  if (congestion) {
      CC2420TransmitP__totalCcaChecks = 0;
      CC2420TransmitP__releaseSpiResource();
      CC2420TransmitP__congestionBackoff();
    }
  else 
#line 771
    {
      CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__CC2420_ABORT_PERIOD);
    }
}

# 318 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Strobe__strobe(uint8_t addr)
#line 318
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 319
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 321
            0;

            {
#line 321
              __nesc_atomic_end(__nesc_atomic); 
#line 321
              return __nesc_temp;
            }
          }
        }
    }
#line 325
    __nesc_atomic_end(__nesc_atomic); }
#line 325
  return CC2420SpiP__SpiByte__write(addr);
}

# 133 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
static bool StateImplP__State__isState(uint8_t id, uint8_t myState)
#line 133
{
  bool isState;

#line 135
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 135
    isState = StateImplP__state[id] == myState;
#line 135
    __nesc_atomic_end(__nesc_atomic); }
  return isState;
}

# 99 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(uint8_t tx)
#line 99
{
  uint8_t byte;


  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(tx);
  while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending()) ;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__clrRxIntr();
  byte = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx();

  return byte;
}

# 386 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static uint8_t HplMsp430Usart0P__Usart__rx(void )
#line 386
{
  uint8_t value;

#line 388
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 388
    value = U0RXBUF;
#line 388
    __nesc_atomic_end(__nesc_atomic); }
  return value;
}

# 149 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__release(uint8_t id)
#line 149
{
  uint8_t i;

#line 151
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 151
    {
      if (CC2420SpiP__m_holder != id) {
          {
            unsigned char __nesc_temp = 
#line 153
            FAIL;

            {
#line 153
              __nesc_atomic_end(__nesc_atomic); 
#line 153
              return __nesc_temp;
            }
          }
        }
#line 156
      CC2420SpiP__m_holder = CC2420SpiP__NO_HOLDER;
      if (!CC2420SpiP__m_requests) {
          CC2420SpiP__WorkingState__toIdle();
          CC2420SpiP__attemptRelease();
        }
      else {
          for (i = CC2420SpiP__m_holder + 1; ; i++) {
              i %= CC2420SpiP__RESOURCE_COUNT;

              if (CC2420SpiP__m_requests & (1 << i)) {
                  CC2420SpiP__m_holder = i;
                  CC2420SpiP__m_requests &= ~(1 << i);
                  CC2420SpiP__grant__postTask();
                  {
                    unsigned char __nesc_temp = 
#line 169
                    SUCCESS;

                    {
#line 169
                      __nesc_atomic_end(__nesc_atomic); 
#line 169
                      return __nesc_temp;
                    }
                  }
                }
            }
        }
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
#line 175
  return SUCCESS;
}

#line 339
static error_t CC2420SpiP__attemptRelease(void )
#line 339
{


  if ((
#line 340
  CC2420SpiP__m_requests > 0
   || CC2420SpiP__m_holder != CC2420SpiP__NO_HOLDER)
   || !CC2420SpiP__WorkingState__isIdle()) {
      return FAIL;
    }
  /* atomic removed: atomic calls only */
  CC2420SpiP__release = TRUE;
  CC2420SpiP__ChipSpiResource__releasing();
  /* atomic removed: atomic calls only */
#line 348
  {
    if (CC2420SpiP__release) {
        CC2420SpiP__SpiResource__release();
        {
          unsigned char __nesc_temp = 
#line 351
          SUCCESS;

#line 351
          return __nesc_temp;
        }
      }
  }
  return EBUSY;
}

# 247 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static void HplMsp430Usart0P__Usart__disableSpi(void )
#line 247
{
  /* atomic removed: atomic calls only */
#line 248
  {
    HplMsp430Usart0P__ME1 &= ~(1 << 6);
    HplMsp430Usart0P__SIMO__selectIOFunc();
    HplMsp430Usart0P__SOMI__selectIOFunc();
    HplMsp430Usart0P__UCLK__selectIOFunc();
  }
}

# 45 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )29U |= 0x01 << 2;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

#line 46
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )29U &= ~(0x01 << 2);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 842 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__signalDone(error_t err)
#line 842
{
  /* atomic removed: atomic calls only */
#line 843
  CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
  CC2420TransmitP__abortSpiRelease = FALSE;
  CC2420TransmitP__ChipSpiResource__attemptRelease();
  CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, err);
}

# 38 "/opt/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(uint8_t mode)
#line 38
{
  /* atomic removed: atomic calls only */
#line 39
  {
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectModuleFunc();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(mode);
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__enableEvents();
  }
  return SUCCESS;
}

# 46 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )29U &= ~(0x01 << 6);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

#line 45
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )29U |= 0x01 << 6;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 260 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Ram__write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len)
#line 262
{

  cc2420_status_t status = 0;
  uint8_t tmpLen = len;
  uint8_t * tmpData = (uint8_t * )data;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 268
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 270
            status;

            {
#line 270
              __nesc_atomic_end(__nesc_atomic); 
#line 270
              return __nesc_temp;
            }
          }
        }
    }
#line 274
    __nesc_atomic_end(__nesc_atomic); }
#line 274
  addr += offset;

  status = CC2420SpiP__SpiByte__write(addr | 0x80);
  CC2420SpiP__SpiByte__write((addr >> 1) & 0xc0);
  for (; len; len--) {
      CC2420SpiP__SpiByte__write(tmpData[tmpLen - len]);
    }

  return status;
}

# 137 "/opt/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc"
static void CC2420PacketP__PacketTimeStamp32khz__clear(message_t *msg)
{
  __nesc_hton_int8(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timesync.data, FALSE);
  __nesc_hton_uint32(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timestamp.data, CC2420_INVALID_TIMESTAMP);
}

# 96 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__set_alarm(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__get();
#line 98
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type expires;
#line 98
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type remaining;




  expires = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt;


  remaining = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type )(expires - now);


  if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 <= now) 
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__MAX_DELAY) 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 = now + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__MAX_DELAY;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt = remaining - /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__MAX_DELAY;
      remaining = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__MAX_DELAY;
    }
  else 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 += /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt = 0;
    }
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__startAt((/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_size_type )now << 5, 
  (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_size_type )remaining << 5);
}

# 69 "/opt/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__get(void )
{
  /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type rv = 0;

#line 72
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC__1__upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC__1__m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__get();

#line 76
      if (/*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC__1__LOW_SHIFT_RIGHT;

#line 90
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC__1__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 394 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static error_t Msp430Adc12ImplP__SingleChannel__getData(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 396
    {
      if (Msp430Adc12ImplP__ADCArbiterInfo__userId() == id) {
          if (Msp430Adc12ImplP__state & Msp430Adc12ImplP__MULTIPLE_DATA_REPEAT && !Msp430Adc12ImplP__resultBufferStart) 
            {
              unsigned char __nesc_temp = 
#line 399
              EINVAL;

              {
#line 399
                __nesc_atomic_end(__nesc_atomic); 
#line 399
                return __nesc_temp;
              }
            }
#line 400
          if (Msp430Adc12ImplP__state & Msp430Adc12ImplP__ADC_BUSY) 
            {
              unsigned char __nesc_temp = 
#line 401
              EBUSY;

              {
#line 401
                __nesc_atomic_end(__nesc_atomic); 
#line 401
                return __nesc_temp;
              }
            }
#line 402
          Msp430Adc12ImplP__state |= Msp430Adc12ImplP__ADC_BUSY;
          Msp430Adc12ImplP__clientID = id;
          Msp430Adc12ImplP__configureAdcPin(Msp430Adc12ImplP__HplAdc12__getMCtl(0).inch);
          Msp430Adc12ImplP__HplAdc12__startConversion();
          if (Msp430Adc12ImplP__state & Msp430Adc12ImplP__USE_TIMERA) {
            Msp430Adc12ImplP__startTimerA();
            }
#line 408
          {
            unsigned char __nesc_temp = 
#line 408
            SUCCESS;

            {
#line 408
              __nesc_atomic_end(__nesc_atomic); 
#line 408
              return __nesc_temp;
            }
          }
        }
    }
#line 412
    __nesc_atomic_end(__nesc_atomic); }
#line 411
  return FAIL;
}

# 137 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ArbiterInfo__userId(void )
#line 137
{
  /* atomic removed: atomic calls only */
#line 138
  {
    if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state != /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 140
        /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__NO_RES;

#line 140
        return __nesc_temp;
      }
#line 141
    {
      unsigned char __nesc_temp = 
#line 141
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId;

#line 141
      return __nesc_temp;
    }
  }
}

# 80 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setMode(int mode)
{
  * (volatile uint16_t * )352U = (* (volatile uint16_t * )352U & ~(0x0020 | 0x0010)) | ((mode << 4) & (0x0020 | 0x0010));
}

# 96 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__set_alarm(void )
{
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__to_size_type now = /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__Counter__get();
#line 98
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__to_size_type expires;
#line 98
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__to_size_type remaining;




  expires = /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__m_t0 + /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__m_dt;


  remaining = (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__to_size_type )(expires - now);


  if (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__m_t0 <= now) 
    {
      if (expires >= /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__MAX_DELAY) 
    {
      /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__m_t0 = now + /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__MAX_DELAY;
      /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__m_dt = remaining - /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__MAX_DELAY;
      remaining = /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__MAX_DELAY;
    }
  else 
    {
      /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__m_t0 += /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__m_dt;
      /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__m_dt = 0;
    }
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__AlarmFrom__startAt((/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__from_size_type )now << 5, 
  (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__2__from_size_type )remaining << 5);
}

# 14 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(24)))  void sig_TIMERB1_VECTOR(void )
#line 14
{
#line 14
  Msp430TimerCommonP__VectorTimerB1__fired();
}

# 52 "/opt/tinyos-2.x/tos/system/RealMainP.nc"
  int main(void )
#line 52
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
#line 60
      ;

      RealMainP__Scheduler__init();





      RealMainP__PlatformInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;





      RealMainP__SoftwareInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;
    }
#line 77
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP__Boot__booted();


  RealMainP__Scheduler__taskLoop();




  return -1;
}

# 164 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static void Msp430ClockP__set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

# 16 "/opt/tinyos-2.x/tos/platforms/telosb/MotePlatformC.nc"
static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set)
#line 16
{
  if (set) {
    TOSH_SET_SIMO0_PIN();
    }
  else {
#line 20
    TOSH_CLR_SIMO0_PIN();
    }
#line 21
  TOSH_SET_UCLK0_PIN();
  TOSH_CLR_UCLK0_PIN();
}

# 45 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )49U |= 0x01 << 4;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

#line 45
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )49U |= 0x01 << 5;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

#line 45
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )49U |= 0x01 << 6;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 123 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static bool SchedulerBasicP__Scheduler__runNextTask(void )
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 127
  {
    nextTask = SchedulerBasicP__popTask();
    if (nextTask == SchedulerBasicP__NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
#line 131
          FALSE;

#line 131
          return __nesc_temp;
        }
      }
  }
#line 134
  SchedulerBasicP__TaskBasic__runTask(nextTask);
  return TRUE;
}

#line 164
static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id)
{
}

# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0xf74014a0){
#line 64
  switch (arg_0xf74014a0) {
#line 64
    case CC2420CsmaP__startDone_task:
#line 64
      CC2420CsmaP__startDone_task__runTask();
#line 64
      break;
#line 64
    case CC2420CsmaP__stopDone_task:
#line 64
      CC2420CsmaP__stopDone_task__runTask();
#line 64
      break;
#line 64
    case CC2420CsmaP__sendDone_task:
#line 64
      CC2420CsmaP__sendDone_task__runTask();
#line 64
      break;
#line 64
    case CC2420ControlP__sync:
#line 64
      CC2420ControlP__sync__runTask();
#line 64
      break;
#line 64
    case CC2420ControlP__syncDone:
#line 64
      CC2420ControlP__syncDone__runTask();
#line 64
      break;
#line 64
    case CC2420SpiP__grant:
#line 64
      CC2420SpiP__grant__runTask();
#line 64
      break;
#line 64
    case /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task:
#line 64
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask();
#line 64
      break;
#line 64
    case /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask:
#line 64
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask();
#line 64
      break;
#line 64
    case CC2420ReceiveP__receiveDone_task:
#line 64
      CC2420ReceiveP__receiveDone_task__runTask();
#line 64
      break;
#line 64
    case /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired:
#line 64
      /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask();
#line 64
      break;
#line 64
    case /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer:
#line 64
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask();
#line 64
      break;
#line 64
    case CC2420TinyosNetworkP__grantTask:
#line 64
      CC2420TinyosNetworkP__grantTask__runTask();
#line 64
      break;
#line 64
    case SendingMoteC__sendMultipleEcho:
#line 64
      SendingMoteC__sendMultipleEcho__runTask();
#line 64
      break;
#line 64
    case SendingMoteC__sendReport:
#line 64
      SendingMoteC__sendReport__runTask();
#line 64
      break;
#line 64
    case SendingMoteC__sendIdentification:
#line 64
      SendingMoteC__sendIdentification__runTask();
#line 64
      break;
#line 64
    case SendingMoteC__sendReading:
#line 64
      SendingMoteC__sendReading__runTask();
#line 64
      break;
#line 64
    case SendingMoteC__sendCommandACK:
#line 64
      SendingMoteC__sendCommandACK__runTask();
#line 64
      break;
#line 64
    case SendingMoteC__sendEchoReply:
#line 64
      SendingMoteC__sendEchoReply__runTask();
#line 64
      break;
#line 64
    case /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask:
#line 64
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask();
#line 64
      break;
#line 64
    case /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask:
#line 64
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask();
#line 64
      break;
#line 64
    case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readSensor:
#line 64
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readSensor__runTask();
#line 64
      break;
#line 64
    case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone:
#line 64
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone__runTask();
#line 64
      break;
#line 64
    case HplSensirionSht11P__stopTask:
#line 64
      HplSensirionSht11P__stopTask__runTask();
#line 64
      break;
#line 64
    case /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask:
#line 64
      /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask__runTask();
#line 64
      break;
#line 64
    case /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__startTask:
#line 64
      /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__startTask__runTask();
#line 64
      break;
#line 64
    case /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopTask:
#line 64
      /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopTask__runTask();
#line 64
      break;
#line 64
    case AdcP__readDone:
#line 64
      AdcP__readDone__runTask();
#line 64
      break;
#line 64
    case /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask:
#line 64
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__runTask();
#line 64
      break;
#line 64
    case Msp430RefVoltArbiterImplP__switchOff:
#line 64
      Msp430RefVoltArbiterImplP__switchOff__runTask();
#line 64
      break;
#line 64
    case AdcStreamP__readStreamDone:
#line 64
      AdcStreamP__readStreamDone__runTask();
#line 64
      break;
#line 64
    case AdcStreamP__readStreamFail:
#line 64
      AdcStreamP__readStreamFail__runTask();
#line 64
      break;
#line 64
    case AdcStreamP__bufferDone:
#line 64
      AdcStreamP__bufferDone__runTask();
#line 64
      break;
#line 64
    default:
#line 64
      SchedulerBasicP__TaskBasic__default__runTask(arg_0xf74014a0);
#line 64
      break;
#line 64
    }
#line 64
}
#line 64
# 116 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static error_t Msp430RefVoltArbiterImplP__ClientResource__release(uint8_t client)
{
  error_t error;

#line 119
  if (Msp430RefVoltArbiterImplP__syncOwner == client) {
    Msp430RefVoltArbiterImplP__switchOff__postTask();
    }
#line 121
  error = Msp430RefVoltArbiterImplP__AdcResource__release(client);
#line 133
  return error;
}

# 97 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__release(uint8_t id)
#line 97
{
  bool released = FALSE;

#line 99
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 99
    {
      if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state == /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_BUSY && /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId == id) {
          if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__isEmpty() == FALSE) {
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__NO_RES;
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__reqResId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__dequeue();
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_GRANTING;
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__postTask();
            }
          else {
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__NO_RES;
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_IDLE;
            }
          released = TRUE;
        }
    }
#line 113
    __nesc_atomic_end(__nesc_atomic); }
  if (released == TRUE) {
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__unconfigure(id);
      return SUCCESS;
    }
  return FAIL;
}

# 65 "/opt/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc"
static bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEnqueued(resource_client_id_t id)
#line 65
{
  return /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ[id / 8] & (1 << id % 8);
}

# 236 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static error_t Msp430RefVoltGeneratorP__switchOff(void )
#line 236
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 237
    {
      if (Msp430RefVoltGeneratorP__HplAdc12__isBusy()) {
          {
            unsigned char __nesc_temp = 
#line 239
            EBUSY;

            {
#line 239
              __nesc_atomic_end(__nesc_atomic); 
#line 239
              return __nesc_temp;
            }
          }
        }
      else 
#line 241
        {
          adc12ctl0_t ctl0 = Msp430RefVoltGeneratorP__HplAdc12__getCtl0();

#line 243
          ctl0.enc = 0;
          Msp430RefVoltGeneratorP__HplAdc12__setCtl0(ctl0);
          ctl0.refon = 0;
          Msp430RefVoltGeneratorP__HplAdc12__setCtl0(ctl0);
          {
            unsigned char __nesc_temp = 
#line 247
            SUCCESS;

            {
#line 247
              __nesc_atomic_end(__nesc_atomic); 
#line 247
              return __nesc_temp;
            }
          }
        }
    }
#line 251
    __nesc_atomic_end(__nesc_atomic); }
}

# 133 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

#line 136
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

# 153 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static void Msp430RefVoltGeneratorP__signalStartDone(Msp430RefVoltGeneratorP__state_t state, error_t result)
#line 153
{
  if (state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE || state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_ON_PENDING) {
    Msp430RefVoltGeneratorP__RefVolt_1_5V__startDone(result);
    }
  else {
#line 157
    Msp430RefVoltGeneratorP__RefVolt_2_5V__startDone(result);
    }
}

# 221 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
static error_t AdcStreamP__ReadStream__read(uint8_t c, uint32_t usPeriod)
{
  if (usPeriod & 0xFFFF0000) {

      AdcStreamP__period = usPeriod / 1000;
      AdcStreamP__periodModified = TRUE;
      AdcStreamP__client = c;
      AdcStreamP__now = AdcStreamP__Alarm__getNow();
      AdcStreamP__SingleChannel__configureSingle(c, AdcStreamP__AdcConfigure__getConfiguration(c));
      if (AdcStreamP__nextBuffer(FALSE) == SUCCESS) {
        AdcStreamP__sampleSingle();
        }
    }
  else 
#line 232
    {
      AdcStreamP__period = usPeriod;
      AdcStreamP__periodModified = FALSE;
      AdcStreamP__client = c;
      AdcStreamP__nextMultiple(c);
    }
  return SUCCESS;
}

# 176 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static error_t Msp430Adc12ImplP__SingleChannel__configureSingle(uint8_t id, 
const msp430adc12_channel_config_t *config)
{
  error_t result = ERESERVE;

  if (!config) {
    return EINVAL;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 184
    {
      if (Msp430Adc12ImplP__state & Msp430Adc12ImplP__ADC_BUSY) 
        {
          unsigned char __nesc_temp = 
#line 186
          EBUSY;

          {
#line 186
            __nesc_atomic_end(__nesc_atomic); 
#line 186
            return __nesc_temp;
          }
        }
#line 187
      if (Msp430Adc12ImplP__ADCArbiterInfo__userId() == id) {
          adc12ctl1_t ctl1 = { 
          .adc12busy = 0, 
          .conseq = 0, 
          .adc12ssel = config->adc12ssel, 
          .adc12div = config->adc12div, 
          .issh = 0, 
          .shp = 1, 
          .shs = 0, 
          .cstartadd = 0 };

          adc12memctl_t memctl = { 
          .inch = config->inch, 
          .sref = config->sref, 
          .eos = 1 };

          adc12ctl0_t ctl0 = Msp430Adc12ImplP__HplAdc12__getCtl0();

#line 204
          ctl0.msc = 1;
          ctl0.sht0 = config->sht;
          ctl0.sht1 = config->sht;

          Msp430Adc12ImplP__state = Msp430Adc12ImplP__SINGLE_DATA;
          Msp430Adc12ImplP__HplAdc12__setCtl0(ctl0);
          Msp430Adc12ImplP__HplAdc12__setCtl1(ctl1);
          Msp430Adc12ImplP__HplAdc12__setMCtl(0, memctl);
          Msp430Adc12ImplP__HplAdc12__setIEFlags(0x01);
          result = SUCCESS;
        }
    }
#line 215
    __nesc_atomic_end(__nesc_atomic); }
  return result;
}

# 177 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
static error_t AdcStreamP__nextBuffer(bool startNextAlarm)
#line 177
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      struct AdcStreamP__list_entry_t *entry = AdcStreamP__bufferQueue[AdcStreamP__client];

      if (!entry) 
        {

          AdcStreamP__bufferQueueEnd[AdcStreamP__client] = (void *)0;
          AdcStreamP__readStreamDone__postTask();
          {
            unsigned char __nesc_temp = 
#line 187
            FAIL;

            {
#line 187
              __nesc_atomic_end(__nesc_atomic); 
#line 187
              return __nesc_temp;
            }
          }
        }
      else 
#line 190
        {
          uint16_t tmp_count;

#line 192
          AdcStreamP__bufferQueue[AdcStreamP__client] = entry->next;
          if (!AdcStreamP__bufferQueue[AdcStreamP__client]) {
            AdcStreamP__bufferQueueEnd[AdcStreamP__client] = &AdcStreamP__bufferQueue[AdcStreamP__client];
            }
#line 195
          AdcStreamP__pos = AdcStreamP__buffer = (void *)0;
          AdcStreamP__count = entry->count;
          tmp_count = AdcStreamP__count;
          AdcStreamP__pos = AdcStreamP__buffer = (uint16_t * )entry;
          if (startNextAlarm) {
            AdcStreamP__nextAlarm();
            }
#line 201
          {
            unsigned char __nesc_temp = 
#line 201
            SUCCESS;

            {
#line 201
              __nesc_atomic_end(__nesc_atomic); 
#line 201
              return __nesc_temp;
            }
          }
        }
    }
#line 205
    __nesc_atomic_end(__nesc_atomic); }
}

#line 206
static void AdcStreamP__nextMultiple(uint8_t c)
{
  if (AdcStreamP__nextBuffer(FALSE) == SUCCESS) {
      msp430adc12_channel_config_t config = *AdcStreamP__AdcConfigure__getConfiguration(c);

#line 210
      config.sampcon_ssel = SAMPCON_SOURCE_SMCLK;
      config.sampcon_id = SAMPCON_CLOCK_DIV_1;
      if (AdcStreamP__SingleChannel__configureMultiple(c, &config, AdcStreamP__pos, AdcStreamP__count, AdcStreamP__period) == SUCCESS) {
        AdcStreamP__SingleChannel__getData(c);
        }
      else 
#line 214
        {
          AdcStreamP__postBuffer(c, AdcStreamP__pos, AdcStreamP__count);
          AdcStreamP__readStreamFail__postTask();
        }
    }
}

# 80 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static void AdcP__ResourceRead__granted(uint8_t client)
{

  error_t result = AdcP__configure(client);

#line 84
  if (result == SUCCESS) {
      AdcP__state = AdcP__STATE_READ;
      result = AdcP__SingleChannel__getData(client);
    }
  else 
#line 87
    {
      AdcP__ResourceRead__release(client);
      AdcP__Read__readDone(client, result, 0);
    }
}

# 1678 "SendingMoteC.nc"
static void SendingMoteC__Light__readDone(error_t ok, uint16_t val)
#line 1678
{
  if (ok == SUCCESS) {
      SendingMoteC__light = val;
      if ((SendingMoteC__readMode & 15) == 2) {
          SendingMoteC__tmpReading = val;
          SendingMoteC__msgReadingLong = SendingMoteC__numConditionValide + 2 * SCALE_TYPE_READ;
        }
    }


  SendingMoteC__sendReading__postTask();
}

# 107 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static void Msp430RefVoltArbiterImplP__RefVolt_2_5V__startDone(error_t error)
{
  if (Msp430RefVoltArbiterImplP__syncOwner != Msp430RefVoltArbiterImplP__NO_OWNER) {


      Msp430RefVoltArbiterImplP__ClientResource__granted(Msp430RefVoltArbiterImplP__syncOwner);
    }
}

# 160 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static void Msp430RefVoltGeneratorP__signalStopDone(Msp430RefVoltGeneratorP__state_t state, error_t result)
#line 160
{
  if (state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE || state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_OFF_PENDING) {
    Msp430RefVoltGeneratorP__RefVolt_1_5V__stopDone(result);
    }
  else {
#line 164
    Msp430RefVoltGeneratorP__RefVolt_2_5V__stopDone(result);
    }
}

# 70 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static void Msp430RefVoltArbiterImplP__AdcResource__granted(uint8_t client)
{
  const msp430adc12_channel_config_t *settings = Msp430RefVoltArbiterImplP__Config__getConfiguration(client);

#line 73
  if (settings->sref == REFERENCE_VREFplus_AVss || 
  settings->sref == REFERENCE_VREFplus_VREFnegterm) {
      error_t started;

#line 76
      if (Msp430RefVoltArbiterImplP__syncOwner != Msp430RefVoltArbiterImplP__NO_OWNER) {



          Msp430RefVoltArbiterImplP__AdcResource__release(client);
          Msp430RefVoltArbiterImplP__AdcResource__request(client);
          return;
        }
      Msp430RefVoltArbiterImplP__syncOwner = client;
      if (settings->ref2_5v == REFVOLT_LEVEL_1_5) {
        started = Msp430RefVoltArbiterImplP__RefVolt_1_5V__start();
        }
      else {
#line 88
        started = Msp430RefVoltArbiterImplP__RefVolt_2_5V__start();
        }
#line 89
      if (started != SUCCESS) {
          Msp430RefVoltArbiterImplP__syncOwner = Msp430RefVoltArbiterImplP__NO_OWNER;
          Msp430RefVoltArbiterImplP__AdcResource__release(client);
          Msp430RefVoltArbiterImplP__AdcResource__request(client);
        }
    }
  else {
#line 95
    Msp430RefVoltArbiterImplP__ClientResource__granted(client);
    }
}

# 71 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__request(uint8_t id)
#line 71
{
  /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceRequested__requested(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 73
    {
      if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state == /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_IDLE) {
          /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_GRANTING;
          /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__reqResId = id;
          /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__postTask();
          {
            unsigned char __nesc_temp = 
#line 78
            SUCCESS;

            {
#line 78
              __nesc_atomic_end(__nesc_atomic); 
#line 78
              return __nesc_temp;
            }
          }
        }
#line 80
      {
        unsigned char __nesc_temp = 
#line 80
        /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__enqueue(id);

        {
#line 80
          __nesc_atomic_end(__nesc_atomic); 
#line 80
          return __nesc_temp;
        }
      }
    }
#line 83
    __nesc_atomic_end(__nesc_atomic); }
}

# 94 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static error_t Msp430RefVoltGeneratorP__start(Msp430RefVoltGeneratorP__state_t targetState)
#line 94
{
  error_t result;

  if (Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE || Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_2_5V_STABLE) {
      if (targetState == Msp430RefVoltGeneratorP__m_state) {
          result = EALREADY;
        }
      else {
#line 100
        if ((result = Msp430RefVoltGeneratorP__switchOn(targetState)) == SUCCESS) {
            Msp430RefVoltGeneratorP__m_state = targetState;
            Msp430RefVoltGeneratorP__signalStartDone(targetState, SUCCESS);
          }
        }
    }
  else {
#line 104
    if (Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__GENERATOR_OFF) {
        if ((result = Msp430RefVoltGeneratorP__switchOn(targetState)) == SUCCESS) {
            Msp430RefVoltGeneratorP__SwitchOnTimer__startOneShot(17);
            Msp430RefVoltGeneratorP__m_state = targetState + 2;
          }
      }
    else {
#line 109
      if (Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_OFF_PENDING || Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_2_5V_OFF_PENDING) {
          if ((result = Msp430RefVoltGeneratorP__switchOn(targetState)) == SUCCESS) {

              Msp430RefVoltGeneratorP__state_t oldState = Msp430RefVoltGeneratorP__m_state;

#line 113
              Msp430RefVoltGeneratorP__SwitchOffTimer__stop();
              Msp430RefVoltGeneratorP__m_state = targetState;
              Msp430RefVoltGeneratorP__signalStopDone(oldState, FAIL);
              Msp430RefVoltGeneratorP__signalStartDone(targetState, SUCCESS);
            }
        }
      else {
#line 118
        if (Msp430RefVoltGeneratorP__m_state == targetState + 2) {
          result = SUCCESS;
          }
        else {
#line 121
          result = EBUSY;
          }
        }
      }
    }
#line 123
  return result;
}

#line 217
static error_t Msp430RefVoltGeneratorP__switchOn(uint8_t level)
#line 217
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 218
    {
      if (Msp430RefVoltGeneratorP__HplAdc12__isBusy()) {
          {
            unsigned char __nesc_temp = 
#line 220
            EBUSY;

            {
#line 220
              __nesc_atomic_end(__nesc_atomic); 
#line 220
              return __nesc_temp;
            }
          }
        }
      else 
#line 222
        {
          adc12ctl0_t ctl0 = Msp430RefVoltGeneratorP__HplAdc12__getCtl0();

#line 224
          ctl0.enc = 0;
          Msp430RefVoltGeneratorP__HplAdc12__setCtl0(ctl0);
          ctl0.refon = 1;


          ctl0.r2_5v = level - 1;
          Msp430RefVoltGeneratorP__HplAdc12__setCtl0(ctl0);
          {
            unsigned char __nesc_temp = 
#line 231
            SUCCESS;

            {
#line 231
              __nesc_atomic_end(__nesc_atomic); 
#line 231
              return __nesc_temp;
            }
          }
        }
    }
#line 235
    __nesc_atomic_end(__nesc_atomic); }
}

# 52 "/opt/tinyos-2.x/tos/platforms/telosa/chips/sht11/HplSensirionSht11P.nc"
static error_t HplSensirionSht11P__SplitControl__start(void )
#line 52
{
  HplSensirionSht11P__PWR__makeOutput();
  HplSensirionSht11P__PWR__set();
  HplSensirionSht11P__Timer__startOneShot(11);
  return SUCCESS;
}

# 46 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )33U &= ~(0x01 << 6);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}


static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__makeInput(void )
#line 50
{
#line 50
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 50
    * (volatile uint8_t * )34U &= ~(0x01 << 5);
#line 50
    __nesc_atomic_end(__nesc_atomic); }
}

#line 46
static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )33U &= ~(0x01 << 5);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 149 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__performCommand(void )
#line 149
{

  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__initPins();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__resetDevice();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__transmissionStart();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__cmd &= 0x1F;
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__sendCommand(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__cmd);

  if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__waitForResponse() != SUCCESS) {
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = FALSE;
      return FAIL;
    }

  switch (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__cmd) {

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_SOFT_RESET: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__startOneShot(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__TIMEOUT_RESET);
      break;

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_MEASURE_TEMPERATURE: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__enableInterrupt();

      if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__status & SHT11_STATUS_LOW_RES_BIT) {
          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__startOneShot(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__TIMEOUT_12BIT);
        }
      else 
#line 173
        {
          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__startOneShot(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__TIMEOUT_14BIT);
        }

      break;

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_MEASURE_HUMIDITY: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__enableInterrupt();

      if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__status & SHT11_STATUS_LOW_RES_BIT) {
          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__startOneShot(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__TIMEOUT_8BIT);
        }
      else 
#line 184
        {
          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__startOneShot(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__TIMEOUT_12BIT);
        }

      break;

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_READ_STATUS: 
        {
          uint8_t tempStatus;
          uint8_t crc;

          tempStatus = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readByte();
          crc = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readByte();
          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__endTransmission();

          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__status = tempStatus;

          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone__postTask();
        }

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_WRITE_STATUS: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__writeByte(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__newStatus);

      if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__waitForResponse() != SUCCESS) {
          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__writeFail = TRUE;
        }
      else 
#line 209
        {
          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__status = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__newStatus;
        }

      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone__postTask();
    }


  return SUCCESS;
}

# 45 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )33U |= 0x01 << 5;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 58 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__disable(void )
#line 58
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 59
    {
      /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__disable();
      /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__clear();
    }
#line 62
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 52 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__makeOutput(void )
#line 52
{
#line 52
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 52
    * (volatile uint8_t * )34U |= 0x01 << 5;
#line 52
    __nesc_atomic_end(__nesc_atomic); }
}

#line 45
static void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )33U |= 0x01 << 6;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 255 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__writeByte(uint8_t byte)
#line 255
{
  uint8_t i;

#line 257
  for (i = 0; i < 8; i++) {
      if (byte & 0x80) {
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set();
        }
      else {
#line 261
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__clr();
        }
#line 262
      byte = byte << 1;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__set();
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
    }
}

static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__waitForResponse(void )
#line 268
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeInput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__set();
  if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__get()) {


      return FAIL;
    }
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
  return SUCCESS;
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(9U, dt);
#line 62
}
#line 62
# 281 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__enableInterrupt(void )
#line 281
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeInput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__enableFallingEdge();
}

#line 355
static uint8_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readByte(void )
#line 355
{
  uint8_t byte = 0;
  uint8_t i;

  for (i = 0; i < 8; i++) {
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__set();
      if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__get()) {
        byte |= 1;
        }
#line 363
      if (i != 7) {
        byte = byte << 1;
        }
#line 365
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
    }

  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__ack();
  return byte;
}










static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__endTransmission(void )
#line 381
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeOutput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
}

# 108 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__release(uint8_t id)
#line 108
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 109
    {
      if (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_BUSY && /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId == id) {
          if (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Queue__isEmpty() == FALSE) {
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__reqResId = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Queue__dequeue();
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__NO_RES;
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_GRANTING;
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask__postTask();
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(id);
            }
          else {
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__default_owner_id;
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_CONTROLLED;
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(id);
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted();
            }
          {
            unsigned char __nesc_temp = 
#line 124
            SUCCESS;

            {
#line 124
              __nesc_atomic_end(__nesc_atomic); 
#line 124
              return __nesc_temp;
            }
          }
        }
    }
#line 128
    __nesc_atomic_end(__nesc_atomic); }
#line 127
  return FAIL;
}

# 1691 "SendingMoteC.nc"
static void SendingMoteC__Humidity__readDone(error_t ok, uint16_t val)
#line 1691
{
  if (ok == SUCCESS) {
      SendingMoteC__humidity = val;
      if ((SendingMoteC__readMode & 15) == 3) {

          SendingMoteC__tmpReading = -4 + 0.0405 * val + -2.8 * 1e-6 * (val * val);
          SendingMoteC__msgReadingLong = SendingMoteC__numConditionValide + 3 * SCALE_TYPE_READ;
        }
      else {
#line 1698
        if ((SendingMoteC__readMode & 15) == 4) {

            uint8_t temperatureTmp = SendingMoteC__tmpReading / 10;

#line 1701
            SendingMoteC__tmpReading = (temperatureTmp - 25) * (0.01 + 0.00008 * val) + (-4 + 0.0405 * val + -2.8 * 1e-6 * (val * val));
            SendingMoteC__msgReadingLong = SendingMoteC__numConditionValide + 3 * SCALE_TYPE_READ;
          }
        }
    }

  SendingMoteC__sendReading__postTask();
}

#line 1656
static void SendingMoteC__Temperature__readDone(error_t ok, uint16_t val)
#line 1656
{
  if (ok == SUCCESS) {
      SendingMoteC__temperature = val;
      if ((SendingMoteC__readMode & 15) == 1 || (SendingMoteC__readMode & 15) == 4) {



          SendingMoteC__tmpReading = 10 * (-39.60 + 0.01 * val);
          SendingMoteC__msgReadingLong = SendingMoteC__numConditionValide + 1 * SCALE_TYPE_READ;
        }
    }


  if ((SendingMoteC__readMode & 15) == 4) {
      SendingMoteC__Humidity__read();
      return;
    }
  else 
#line 1672
    {

      SendingMoteC__sendReading__postTask();
    }
}

# 77 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__request(uint8_t id)
#line 77
{
  /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceRequested__requested(/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 79
    {
      if (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_CONTROLLED) {
          /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_GRANTING;
          /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__reqResId = id;
        }
      else {
          unsigned char __nesc_temp = 
#line 84
          /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Queue__enqueue(id);

          {
#line 84
            __nesc_atomic_end(__nesc_atomic); 
#line 84
            return __nesc_temp;
          }
        }
    }
#line 87
    __nesc_atomic_end(__nesc_atomic); }
#line 86
  /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested();
  return SUCCESS;
}

# 207 "/opt/tinyos-2.x/tos/system/AMQueueImplP.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err)
#line 207
{
}

# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(uint8_t arg_0xf68a31e0, message_t * msg, error_t error){
#line 89
  switch (arg_0xf68a31e0) {
#line 89
    case 0U:
#line 89
      /*SendingMoteAppC.RssiMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(msg, error);
#line 89
      break;
#line 89
    case 1U:
#line 89
      /*SendingMoteAppC.MultiPingResponseSender.SenderC.AMQueueEntryP*/AMQueueEntryP__1__Send__sendDone(msg, error);
#line 89
      break;
#line 89
    case 2U:
#line 89
      /*SendingMoteAppC.ReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__2__Send__sendDone(msg, error);
#line 89
      break;
#line 89
    case 3U:
#line 89
      /*SendingMoteAppC.TinyReportMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__3__Send__sendDone(msg, error);
#line 89
      break;
#line 89
    case 4U:
#line 89
      /*SendingMoteAppC.CommandMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__4__Send__sendDone(msg, error);
#line 89
      break;
#line 89
    default:
#line 89
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(arg_0xf68a31e0, msg, error);
#line 89
      break;
#line 89
    }
#line 89
}
#line 89
# 337 "SendingMoteC.nc"
static void SendingMoteC__sendBlink(void )
#line 337
{

  if (SendingMoteC__blinkCnSend == 0) {
      SendingMoteC__Leds__led1Toggle();
    }

  SendingMoteC__blinkCnSend = (SendingMoteC__blinkCnSend + 1) % 5;
}

# 47 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__toggle(void )
#line 47
{
#line 47
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 47
    * (volatile uint8_t * )49U ^= 0x01 << 5;
#line 47
    __nesc_atomic_end(__nesc_atomic); }
}

# 347 "SendingMoteC.nc"
static void SendingMoteC__failBlink(void )
#line 347
{
  if (SendingMoteC__blinkCnFail == 0) {
      SendingMoteC__Leds__led0Toggle();
    }

  SendingMoteC__blinkCnFail = (SendingMoteC__blinkCnFail + 1) % 1;
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void SendingMoteC__GapTimer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(3U, dt);
#line 62
}
#line 62
# 321 "SendingMoteC.nc"
static uint16_t SendingMoteC__queueSize(void )
#line 321
{
  return SendingMoteC__radioOut > SendingMoteC__radioIn ? SendingMoteC__RADIO_QUEUE_LEN - SendingMoteC__radioOut + SendingMoteC__radioIn : SendingMoteC__radioIn - SendingMoteC__radioOut;
}

# 166 "/opt/tinyos-2.x/tos/system/AMQueueImplP.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void )
#line 166
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket();
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current < 5) {
      error_t nextErr;
      message_t *nextMsg = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg;
      am_id_t nextId = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(nextMsg);
      am_addr_t nextDest = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(nextMsg);
      uint8_t len = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(nextMsg);

#line 174
      nextErr = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(nextId, nextDest, nextMsg, len);
      if (nextErr != SUCCESS) {
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask();
        }
    }
}

# 126 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static am_addr_t CC2420ActiveMessageP__AMPacket__destination(message_t *amsg)
#line 126
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(amsg);

#line 128
  return __nesc_ntoh_leuint16(header->dest.data);
}

#line 78
static error_t CC2420ActiveMessageP__AMSend__send(am_id_t id, am_addr_t addr, 
message_t *msg, 
uint8_t len)
#line 80
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(msg);

  if (len > CC2420ActiveMessageP__Packet__maxPayloadLength()) {
      return ESIZE;
    }

  __nesc_hton_leuint8(header->type.data, id);
  __nesc_hton_leuint16(header->dest.data, addr);
  __nesc_hton_leuint16(header->destpan.data, CC2420ActiveMessageP__CC2420Config__getPanAddr());
  __nesc_hton_leuint16(header->src.data, CC2420ActiveMessageP__AMPacket__address());

  if (CC2420ActiveMessageP__RadioResource__immediateRequest() == SUCCESS) {
      error_t rc;

#line 94
      CC2420ActiveMessageP__SendNotifier__aboutToSend(id, addr, msg);

      rc = CC2420ActiveMessageP__SubSend__send(msg, len);
      if (rc != SUCCESS) {
          CC2420ActiveMessageP__RadioResource__release();
        }

      return rc;
    }
  else 
#line 102
    {
      CC2420ActiveMessageP__pending_length = len;
      CC2420ActiveMessageP__pending_message = msg;
      return CC2420ActiveMessageP__RadioResource__request();
    }
}

# 95 "/opt/tinyos-2.x/tos/system/ActiveMessageAddressC.nc"
static am_addr_t ActiveMessageAddressC__amAddress(void )
#line 95
{
  am_addr_t myAddr;

#line 97
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 97
    myAddr = ActiveMessageAddressC__addr;
#line 97
    __nesc_atomic_end(__nesc_atomic); }
  return myAddr;
}

# 50 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void )
#line 50
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 51
    {
      unsigned char __nesc_temp = 
#line 51
      /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead == /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;

      {
#line 51
        __nesc_atomic_end(__nesc_atomic); 
#line 51
        return __nesc_temp;
      }
    }
#line 53
    __nesc_atomic_end(__nesc_atomic); }
}

# 106 "/opt/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc"
static void CC2420PacketP__CC2420Packet__setNetwork(message_t *p_msg, uint8_t networkId)
#line 106
{

  __nesc_hton_leuint8(CC2420PacketP__CC2420PacketBody__getHeader(p_msg)->network.data, networkId);
}

# 75 "/opt/tinyos-2.x/tos/chips/cc2420/unique/UniqueSendP.nc"
static error_t UniqueSendP__Send__send(message_t *msg, uint8_t len)
#line 75
{
  error_t error;

#line 77
  if (UniqueSendP__State__requestState(UniqueSendP__S_SENDING) == SUCCESS) {
      __nesc_hton_leuint8(UniqueSendP__CC2420PacketBody__getHeader(msg)->dsn.data, UniqueSendP__localSendId++);

      if ((error = UniqueSendP__SubSend__send(msg, len)) != SUCCESS) {
          UniqueSendP__State__toIdle();
        }

      return error;
    }

  return EBUSY;
}

# 817 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__loadTXFIFO(void )
#line 817
{
  cc2420_header_t *header = CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg);
  uint8_t tx_power = __nesc_ntoh_uint8(CC2420TransmitP__CC2420PacketBody__getMetadata(CC2420TransmitP__m_msg)->tx_power.data);

  if (!tx_power) {
      tx_power = 31;
    }

  CC2420TransmitP__CSN__clr();

  if (CC2420TransmitP__m_tx_power != tx_power) {
      CC2420TransmitP__TXCTRL__write((((2 << CC2420_TXCTRL_TXMIXBUF_CUR) | (
      3 << CC2420_TXCTRL_PA_CURRENT)) | (
      1 << CC2420_TXCTRL_RESERVED)) | ((
      tx_power & 0x1F) << CC2420_TXCTRL_PA_LEVEL));
    }

  CC2420TransmitP__m_tx_power = tx_power;

  {
    uint8_t tmpLen __attribute((unused))  = __nesc_ntoh_leuint8(header->length.data) - 1;

#line 838
    CC2420TransmitP__TXFIFO__write((uint8_t * )header, __nesc_ntoh_leuint8(header->length.data) - 1);
  }
}

# 305 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Reg__write(uint8_t addr, uint16_t data)
#line 305
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 306
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 308
            0;

            {
#line 308
              __nesc_atomic_end(__nesc_atomic); 
#line 308
              return __nesc_temp;
            }
          }
        }
    }
#line 312
    __nesc_atomic_end(__nesc_atomic); }
#line 311
  CC2420SpiP__SpiByte__write(addr);
  CC2420SpiP__SpiByte__write(data >> 8);
  return CC2420SpiP__SpiByte__write(data & 0xff);
}

# 144 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len)
#line 146
{

  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_client = id;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf = tx_buf;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf = rx_buf;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len = len;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos = 0;

  if (len) {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__enableRxIntr();
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp();
    }
  else {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__postTask();
    }

  return SUCCESS;
}

#line 121
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp(void )
#line 121
{

  uint8_t end;
  uint8_t tmp;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 126
    {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf ? /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos] : 0);

      end = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos + /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SPI_ATOMIC_SIZE;
      if (end > /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len) {
        end = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len;
        }
      while (++/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos < end) {
          while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending()) ;
          tmp = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx();
          if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf) {
            /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos - 1] = tmp;
            }
#line 138
          /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf ? /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos] : 0);
        }
    }
#line 140
    __nesc_atomic_end(__nesc_atomic); }
}

# 56 "/opt/tinyos-2.x/tos/interfaces/State.nc"
static void UniqueSendP__State__toIdle(void ){
#line 56
  StateImplP__State__toIdle(2U);
#line 56
}
#line 56
# 86 "/opt/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static void *CC2420TinyosNetworkP__ActiveSend__getPayload(message_t *msg, uint8_t len)
#line 86
{
  if (len <= CC2420TinyosNetworkP__ActiveSend__maxPayloadLength()) {
      return msg->data;
    }
  else 
#line 89
    {
      return (void *)0;
    }
}

# 1810 "SendingMoteC.nc"
static void SendingMoteC__setPower(message_t *msg, uint8_t power)
#line 1810
{
  if (power >= 1 && power <= 31) 
    {
      SendingMoteC__cur_tx_power = power;
      SendingMoteC__CC2420Packet__setPower(msg, power);
    }
}

# 136 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static void CC2420ActiveMessageP__AMPacket__setDestination(message_t *amsg, am_addr_t addr)
#line 136
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(amsg);

#line 138
  __nesc_hton_leuint16(header->dest.data, addr);
}

#line 156
static void CC2420ActiveMessageP__AMPacket__setType(message_t *amsg, am_id_t type)
#line 156
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(amsg);

#line 158
  __nesc_hton_leuint8(header->type.data, type);
}

# 82 "/opt/tinyos-2.x/tos/system/AMQueueImplP.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len)
#line 83
{
  if (clientId >= 5) {
      return FAIL;
    }
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg != (void *)0) {
      return EBUSY;
    }
  ;

  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg = msg;
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(msg, len);

  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current >= 5) {
      error_t err;
      am_id_t amId = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(msg);
      am_addr_t dest = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(msg);

      ;
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = clientId;

      err = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(amId, dest, msg, len);
      if (err != SUCCESS) {
          ;
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 5;
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg = (void *)0;
        }

      return err;
    }
  else {
      ;
    }
  return SUCCESS;
}

# 45 "/opt/tinyos-2.x/tos/system/AMQueueEntryP.nc"
static error_t /*SendingMoteAppC.CommandMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__4__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 47
{
  /*SendingMoteAppC.CommandMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__4__AMPacket__setDestination(msg, dest);
  /*SendingMoteAppC.CommandMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__4__AMPacket__setType(msg, 14);
  return /*SendingMoteAppC.CommandMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__4__Send__send(msg, len);
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void SendingMoteC__BootupTimer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(7U, dt);
#line 62
}
#line 62
# 65 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static void CC2420ActiveMessageP__RadioResource__granted(void )
#line 65
{
  uint8_t rc;
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(CC2420ActiveMessageP__pending_message);

  CC2420ActiveMessageP__SendNotifier__aboutToSend(__nesc_ntoh_leuint8(header->type.data), __nesc_ntoh_leuint16(header->dest.data), CC2420ActiveMessageP__pending_message);
  rc = CC2420ActiveMessageP__SubSend__send(CC2420ActiveMessageP__pending_message, CC2420ActiveMessageP__pending_length);
  if (rc != SUCCESS) {
      CC2420ActiveMessageP__RadioResource__release();
      CC2420ActiveMessageP__AMSend__sendDone(__nesc_ntoh_leuint8(header->type.data), CC2420ActiveMessageP__pending_message, rc);
    }
}

# 181 "/opt/tinyos-2.x/tos/system/AMQueueImplP.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(am_id_t id, message_t *msg, error_t err)
#line 181
{





  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current >= 5) {
      return;
    }
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg == msg) {
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current, msg, err);
    }
  else {
      ;
    }
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now)
{
  uint8_t num;

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 79
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(num);
              break;
            }
        }
    }
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

# 1641 "SendingMoteC.nc"
static void SendingMoteC__readSensors(void )
#line 1641
{


  if ((SendingMoteC__readMode & 15) == 1 || (SendingMoteC__readMode & 15) == 4) {
    SendingMoteC__Temperature__read();
    }
#line 1646
  if ((SendingMoteC__readMode & 15) == 2) {
    SendingMoteC__Light__read();
    }
#line 1648
  if ((SendingMoteC__readMode & 15) == 3) {
    SendingMoteC__Humidity__read();
    }
}

# 47 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle(void )
#line 47
{
#line 47
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 47
    * (volatile uint8_t * )49U ^= 0x01 << 6;
#line 47
    __nesc_atomic_end(__nesc_atomic); }
}

# 136 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

# 279 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static uint16_t CC2420ControlP__CC2420Config__getShortAddr(void )
#line 279
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 280
    {
      unsigned int __nesc_temp = 
#line 280
      CC2420ControlP__m_short_addr;

      {
#line 280
        __nesc_atomic_end(__nesc_atomic); 
#line 280
        return __nesc_temp;
      }
    }
#line 282
    __nesc_atomic_end(__nesc_atomic); }
}

# 98 "/opt/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc"
static uint8_t CC2420PacketP__CC2420Packet__getNetwork(message_t *p_msg)
#line 98
{



  return __nesc_ntoh_leuint8(CC2420PacketP__CC2420PacketBody__getHeader(p_msg)->network.data);
}

# 361 "SendingMoteC.nc"
static void SendingMoteC__signalize(uint8_t num)
#line 361
{
  SendingMoteC__signalCounter = 2 * num;


  SendingMoteC__Leds__led0Off();
  SendingMoteC__Leds__led1Off();
  SendingMoteC__Leds__led2Off();


  SendingMoteC__SignalTimer__startPeriodic(500);
}

# 143 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, FALSE);
}

# 869 "SendingMoteC.nc"
static void SendingMoteC__reset_node(void )
#line 869
{

  SendingMoteC__abort();

  SendingMoteC__queueFlushThreshold = 4;


  SendingMoteC__queueTimeout = FALSE;


  SendingMoteC__busy = FALSE;


  SendingMoteC__counter = 0;


  SendingMoteC__tx_power = 31;
  SendingMoteC__base_tx_power = 31;


  SendingMoteC__cur_channel = 0;
  SendingMoteC__channel = 0;


  SendingMoteC__blinkCnSend = 0;
  SendingMoteC__blinkCnRecv = 0;
  SendingMoteC__blinkCnFail = 0;


  SendingMoteC__delay = 100;
  SendingMoteC__packets2send = 0;
  SendingMoteC__baseid = 1;
  SendingMoteC__doReporting = TRUE;
  SendingMoteC__doRandomizedThresholding = TRUE;

  SendingMoteC__signalCounter = 0;


  SendingMoteC__Leds__led0Off();
  SendingMoteC__Leds__led1Off();
  SendingMoteC__Leds__led2Off();


  SendingMoteC__radioIn = SendingMoteC__radioOut = 0;
  SendingMoteC__radioBusy = FALSE;
  SendingMoteC__radioFull = TRUE;


  SendingMoteC__clearQueue();


  if (TOS_NODE_ID >= 200) {
      SendingMoteC__operationMode = NODE_TALKING;
    }
  else {
      SendingMoteC__operationMode = NODE_REPORTING;
      SendingMoteC__doSensorReadingSampling = TRUE;
    }


  SendingMoteC__reportProtocol = REPORTING_MEDIUM;


  SendingMoteC__reportMassTrigered = FALSE;


  SendingMoteC__massReportQueueThreshold = 100 * 0.8;


  SendingMoteC__command_dest = SendingMoteC__baseid;

  SendingMoteC__doNoiseFloorReading = TRUE;
  SendingMoteC__noiseFloorMeasured = 0;
  SendingMoteC__reportingGap = SendingMoteC__REPORT_SEND_GAP;
  SendingMoteC__readMode = 1;

  SendingMoteC__doSensorReadingSampling = FALSE;

  SendingMoteC__tabulaRasa = TRUE;
  SendingMoteC__bootupCounter = 0;
}

#line 829
static void SendingMoteC__abort(void )
#line 829
{

  SendingMoteC__SendTimer__stop();
  SendingMoteC__GapTimer__stop();
  SendingMoteC__FlushTimer__stop();
  SendingMoteC__SignalTimer__stop();
  SendingMoteC__SensorReadingTimer__stop();
  SendingMoteC__BootupTimer__stop();

  SendingMoteC__packets2send = 0;
  SendingMoteC__signalCounter = 0;
  SendingMoteC__noiseFloorMeasured = 0;


  SendingMoteC__radioIn = SendingMoteC__radioOut = 0;
  SendingMoteC__radioBusy = FALSE;
  SendingMoteC__radioFull = TRUE;


  SendingMoteC__clearQueue();


  SendingMoteC__Leds__led0Off();
  SendingMoteC__Leds__led1Off();
  SendingMoteC__Leds__led2Off();
}

#line 1539
static bool SendingMoteC__message2sampleReceived(uint16_t rssi, uint16_t cn, uint16_t source)
#line 1539
{
  bool returnValue = FALSE;


  if (SendingMoteC__doReporting == FALSE) {
      return FALSE;
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    if (!SendingMoteC__radioFull) {
        uint16_t qsize = SendingMoteC__queueSize();


        __nesc_hton_uint8(SendingMoteC__radioBuff[SendingMoteC__radioIn].nodeid.data, source);
        __nesc_hton_uint8(SendingMoteC__radioBuff[SendingMoteC__radioIn].nodecounter.data, cn);
        __nesc_hton_int16(SendingMoteC__radioBuff[SendingMoteC__radioIn].rssi.data, rssi);


        if (++SendingMoteC__radioIn >= SendingMoteC__RADIO_QUEUE_LEN) {
          SendingMoteC__radioIn = 0;
          }
#line 1559
        if (SendingMoteC__radioIn == SendingMoteC__radioOut) {
          SendingMoteC__radioFull = TRUE;
          }

        SendingMoteC__queueTimeout = FALSE;





        SendingMoteC__FlushTimer__startOneShot(SendingMoteC__QUEUE_TIMEOUT);


        if (!SendingMoteC__radioBusy) {
            SendingMoteC__sendReport__postTask();
            SendingMoteC__radioBusy = TRUE;
          }



        qsize = SendingMoteC__queueSize();
        if (SendingMoteC__reportProtocol == REPORTING_MASS && qsize >= SendingMoteC__massReportQueueThreshold) {

            CommandMsg *btrpktresponse = (CommandMsg *)SendingMoteC__Packet__getPayload(&SendingMoteC__pkt_priv, 0);


            SendingMoteC__reportMassTrigered = TRUE;


            __nesc_hton_uint8(btrpktresponse->reply_on_command.data, 0);
            __nesc_hton_uint16(btrpktresponse->reply_on_command_id.data, 0);
            __nesc_hton_uint8(btrpktresponse->command_code.data, COMMAND_FLUSHREPORTQUEUE);
            SendingMoteC__command_dest = 65535U;


            SendingMoteC__sendCommandACK__postTask();


            SendingMoteC__GapTimer__startOneShot(SendingMoteC__REPORT_SEND_THRESH);
          }

        returnValue = TRUE;
      }
    else 
#line 1601
      {
        SendingMoteC__failBlink();
        ;

        returnValue = FALSE;
      }
#line 1606
    __nesc_atomic_end(__nesc_atomic); }


  SendingMoteC__sendReport__postTask();

  return returnValue;
}

# 131 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static am_addr_t CC2420ActiveMessageP__AMPacket__source(message_t *amsg)
#line 131
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(amsg);

#line 133
  return __nesc_ntoh_leuint16(header->src.data);
}

# 1462 "SendingMoteC.nc"
static message_t *SendingMoteC__receive(message_t *msg, void *payload, uint8_t len, am_id_t id)
#line 1462
{


  if (id == AM_MULTIPINGMSG && len == sizeof(MultiPingMsg )) {

      MultiPingMsg *btrpkt = (MultiPingMsg *)payload;

      SendingMoteC__busy = FALSE;
      if (!SendingMoteC__busy) {

          MultiPingResponseMsg *btrpktresponse = (MultiPingResponseMsg *)SendingMoteC__Packet__getPayload(&SendingMoteC__pkt_response, 0);


          SendingMoteC__SendTimer__stop();


          SendingMoteC__rcvBlink();


          __nesc_hton_uint16(btrpktresponse->counter.data, __nesc_ntoh_uint16(btrpkt->counter.data));


          SendingMoteC__tx_power = __nesc_ntoh_uint8(btrpkt->txpower.data);


          SendingMoteC__delay = __nesc_ntoh_uint16(btrpkt->delay.data);
          SendingMoteC__packets2send = __nesc_ntoh_uint16(btrpkt->packets.data);


          SendingMoteC__counter_ping = 0;


          SendingMoteC__setAutoAck(FALSE, FALSE);


          SendingMoteC__SendTimer__startPeriodic(SendingMoteC__delay);
        }
    }
  else {





    if (id == AM_MULTIPINGRESPONSEMSG && len == sizeof(MultiPingResponseMsg )) {



        MultiPingResponseMsg *btrpkt = (MultiPingResponseMsg *)payload;

        if (!SendingMoteC__busy) {
            bool added2queue = FALSE;


            SendingMoteC__rcvBlink();


            SendingMoteC__setAutoAck(FALSE, FALSE);


            added2queue = SendingMoteC__message2sampleReceived(SendingMoteC__getRssi(msg), __nesc_ntoh_uint16(btrpkt->counter.data), SendingMoteC__AMPacket__source(msg));
          }
      }
    else 

      {

        SendingMoteC__failBlink();
        ;
      }
    }
#line 1532
  return msg;
}

#line 328
static void SendingMoteC__rcvBlink(void )
#line 328
{

  if (SendingMoteC__blinkCnRecv == 0) {
      SendingMoteC__Leds__led2Toggle();
    }

  SendingMoteC__blinkCnRecv = (SendingMoteC__blinkCnRecv + 1) % 20;
}

# 350 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static void CC2420ControlP__CC2420Config__setAutoAck(bool enableAutoAck, bool hwAutoAck)
#line 350
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 351
    CC2420ControlP__autoAckEnabled = enableAutoAck;
#line 351
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 352
    CC2420ControlP__hwAutoAckDefault = hwAutoAck;
#line 352
    __nesc_atomic_end(__nesc_atomic); }
}

#line 300
static error_t CC2420ControlP__CC2420Config__sync(void )
#line 300
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 301
    {
      if (CC2420ControlP__m_sync_busy) {
          {
            unsigned char __nesc_temp = 
#line 303
            FAIL;

            {
#line 303
              __nesc_atomic_end(__nesc_atomic); 
#line 303
              return __nesc_temp;
            }
          }
        }
#line 306
      CC2420ControlP__m_sync_busy = TRUE;
      if (CC2420ControlP__m_state == CC2420ControlP__S_XOSC_STARTED) {
          CC2420ControlP__SyncResource__request();
        }
      else 
#line 309
        {
          CC2420ControlP__syncDone__postTask();
        }
    }
#line 312
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 764 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static void CC2420ReceiveP__waitForNextPacket(void )
#line 764
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 765
    {
      if (CC2420ReceiveP__m_state == CC2420ReceiveP__S_STOPPED) {
          CC2420ReceiveP__SpiResource__release();
          {
#line 768
            __nesc_atomic_end(__nesc_atomic); 
#line 768
            return;
          }
        }
      CC2420ReceiveP__receivingPacket = FALSE;
#line 783
      if ((CC2420ReceiveP__m_missed_packets && CC2420ReceiveP__FIFO__get()) || !CC2420ReceiveP__FIFOP__get()) {

          if (CC2420ReceiveP__m_missed_packets) {
              CC2420ReceiveP__m_missed_packets--;
            }





          CC2420ReceiveP__beginReceive();
        }
      else 
        {

          CC2420ReceiveP__m_state = CC2420ReceiveP__S_STARTED;
          CC2420ReceiveP__m_missed_packets = 0;
          CC2420ReceiveP__SpiResource__release();
        }
    }
#line 802
    __nesc_atomic_end(__nesc_atomic); }
}

#line 711
static void CC2420ReceiveP__beginReceive(void )
#line 711
{
  CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_LENGTH;
  /* atomic removed: atomic calls only */
#line 713
  CC2420ReceiveP__receivingPacket = TRUE;
  if (CC2420ReceiveP__SpiResource__isOwner()) {
      CC2420ReceiveP__receive();
    }
  else {
#line 717
    if (CC2420ReceiveP__SpiResource__immediateRequest() == SUCCESS) {
        CC2420ReceiveP__receive();
      }
    else {
        CC2420ReceiveP__SpiResource__request();
      }
    }
}

#line 754
static void CC2420ReceiveP__receive(void )
#line 754
{
  CC2420ReceiveP__CSN__clr();
  CC2420ReceiveP__RXFIFO__beginRead((uint8_t *)CC2420ReceiveP__CC2420PacketBody__getHeader(CC2420ReceiveP__m_p_rx_buf), 1);
}

# 189 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Fifo__beginRead(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 190
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 194
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 196
            status;

            {
#line 196
              __nesc_atomic_end(__nesc_atomic); 
#line 196
              return __nesc_temp;
            }
          }
        }
    }
#line 200
    __nesc_atomic_end(__nesc_atomic); }
#line 200
  CC2420SpiP__m_addr = addr | 0x40;

  status = CC2420SpiP__SpiByte__write(CC2420SpiP__m_addr);
  CC2420SpiP__Fifo__continueRead(addr, data, len);

  return status;
}

#line 329
static void CC2420SpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error)
#line 330
{
  if (CC2420SpiP__m_addr & 0x40) {
      CC2420SpiP__Fifo__readDone(CC2420SpiP__m_addr & ~0x40, rx_buf, len, error);
    }
  else 
#line 333
    {
      CC2420SpiP__Fifo__writeDone(CC2420SpiP__m_addr, tx_buf, len, error);
    }
}

# 728 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static void CC2420ReceiveP__flush(void )
#line 728
{








  CC2420ReceiveP__reset_state();

  CC2420ReceiveP__CSN__set();
  CC2420ReceiveP__CSN__clr();
  CC2420ReceiveP__SFLUSHRX__strobe();
  CC2420ReceiveP__SFLUSHRX__strobe();
  CC2420ReceiveP__CSN__set();
  CC2420ReceiveP__SpiResource__release();
  CC2420ReceiveP__waitForNextPacket();
}

#line 808
static void CC2420ReceiveP__reset_state(void )
#line 808
{
  CC2420ReceiveP__m_bytes_left = CC2420ReceiveP__RXFIFO_SIZE;
  /* atomic removed: atomic calls only */
#line 810
  CC2420ReceiveP__receivingPacket = FALSE;
  CC2420ReceiveP__m_timestamp_head = 0;
  CC2420ReceiveP__m_timestamp_size = 0;
  CC2420ReceiveP__m_missed_packets = 0;
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void SendingMoteC__NoiseFloorTimer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(5U, dt);
#line 62
}
#line 62
# 456 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static void CC2420ControlP__writeFsctrl(void )
#line 456
{
  uint8_t channel;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 459
    {
      channel = CC2420ControlP__m_channel;
    }
#line 461
    __nesc_atomic_end(__nesc_atomic); }

  CC2420ControlP__FSCTRL__write((1 << CC2420_FSCTRL_LOCK_THR) | (((
  channel - 11) * 5 + 357) << CC2420_FSCTRL_FREQ));
}







static void CC2420ControlP__writeMdmctrl0(void )
#line 473
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 474
    {
      CC2420ControlP__MDMCTRL0__write((((((((1 << CC2420_MDMCTRL0_RESERVED_FRAME_MODE) | ((
      CC2420ControlP__addressRecognition && CC2420ControlP__hwAddressRecognition) << CC2420_MDMCTRL0_ADR_DECODE)) | (
      2 << CC2420_MDMCTRL0_CCA_HYST)) | (
      3 << CC2420_MDMCTRL0_CCA_MOD)) | (
      1 << CC2420_MDMCTRL0_AUTOCRC)) | ((
      CC2420ControlP__autoAckEnabled && CC2420ControlP__hwAutoAckDefault) << CC2420_MDMCTRL0_AUTOACK)) | (
      0 << CC2420_MDMCTRL0_AUTOACK)) | (
      2 << CC2420_MDMCTRL0_PREAMBLE_LENGTH));
    }
#line 483
    __nesc_atomic_end(__nesc_atomic); }
}







static void CC2420ControlP__writeId(void )
#line 492
{
  nxle_uint16_t id[2];

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 495
    {
      __nesc_hton_leuint16(id[0].data, CC2420ControlP__m_pan);
      __nesc_hton_leuint16(id[1].data, CC2420ControlP__m_short_addr);
    }
#line 498
    __nesc_atomic_end(__nesc_atomic); }

  CC2420ControlP__PANID__write(0, (uint8_t *)&id, sizeof id);
}

# 81 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static error_t CC2420CsmaP__SplitControl__start(void )
#line 81
{
  if (CC2420CsmaP__SplitControlState__requestState(CC2420CsmaP__S_STARTING) == SUCCESS) {
      CC2420CsmaP__CC2420Power__startVReg();
      return SUCCESS;
    }
  else {
#line 86
    if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STARTED)) {
        return EALREADY;
      }
    else {
#line 89
      if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STARTING)) {
          return SUCCESS;
        }
      }
    }
#line 93
  return EBUSY;
}

# 105 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static void HplAdc12P__HplAdc12__stopConversion(void )
#line 105
{

  uint16_t ctl1 = HplAdc12P__ADC12CTL1;

#line 108
  HplAdc12P__ADC12CTL1 &= ~(0x0002 | 0x0004);
  HplAdc12P__ADC12CTL0 &= ~(0x0001 + 0x0002);
  HplAdc12P__ADC12CTL0 &= ~0x0010;
  HplAdc12P__ADC12CTL1 |= ctl1 & (0x0002 | 0x0004);
}

# 53 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
__attribute((wakeup)) __attribute((interrupt(8)))  void sig_PORT1_VECTOR(void )
{
  volatile int n = P1IFG & P1IE;

  if (n & (1 << 0)) {
#line 57
      HplMsp430InterruptP__Port10__fired();
#line 57
      return;
    }
#line 58
  if (n & (1 << 1)) {
#line 58
      HplMsp430InterruptP__Port11__fired();
#line 58
      return;
    }
#line 59
  if (n & (1 << 2)) {
#line 59
      HplMsp430InterruptP__Port12__fired();
#line 59
      return;
    }
#line 60
  if (n & (1 << 3)) {
#line 60
      HplMsp430InterruptP__Port13__fired();
#line 60
      return;
    }
#line 61
  if (n & (1 << 4)) {
#line 61
      HplMsp430InterruptP__Port14__fired();
#line 61
      return;
    }
#line 62
  if (n & (1 << 5)) {
#line 62
      HplMsp430InterruptP__Port15__fired();
#line 62
      return;
    }
#line 63
  if (n & (1 << 6)) {
#line 63
      HplMsp430InterruptP__Port16__fired();
#line 63
      return;
    }
#line 64
  if (n & (1 << 7)) {
#line 64
      HplMsp430InterruptP__Port17__fired();
#line 64
      return;
    }
}

#line 158
__attribute((wakeup)) __attribute((interrupt(2)))  void sig_PORT2_VECTOR(void )
{
  volatile int n = P2IFG & P2IE;

  if (n & (1 << 0)) {
#line 162
      HplMsp430InterruptP__Port20__fired();
#line 162
      return;
    }
#line 163
  if (n & (1 << 1)) {
#line 163
      HplMsp430InterruptP__Port21__fired();
#line 163
      return;
    }
#line 164
  if (n & (1 << 2)) {
#line 164
      HplMsp430InterruptP__Port22__fired();
#line 164
      return;
    }
#line 165
  if (n & (1 << 3)) {
#line 165
      HplMsp430InterruptP__Port23__fired();
#line 165
      return;
    }
#line 166
  if (n & (1 << 4)) {
#line 166
      HplMsp430InterruptP__Port24__fired();
#line 166
      return;
    }
#line 167
  if (n & (1 << 5)) {
#line 167
      HplMsp430InterruptP__Port25__fired();
#line 167
      return;
    }
#line 168
  if (n & (1 << 6)) {
#line 168
      HplMsp430InterruptP__Port26__fired();
#line 168
      return;
    }
#line 169
  if (n & (1 << 7)) {
#line 169
      HplMsp430InterruptP__Port27__fired();
#line 169
      return;
    }
}

# 96 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
__attribute((wakeup)) __attribute((interrupt(18)))  void sig_UART0RX_VECTOR(void )
#line 96
{
  uint8_t temp = U0RXBUF;

#line 98
  HplMsp430Usart0P__Interrupts__rxDone(temp);
}

# 150 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void )
#line 150
{
  /* atomic removed: atomic calls only */
#line 151
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) 
      {
        unsigned char __nesc_temp = 
#line 153
        FALSE;

#line 153
        return __nesc_temp;
      }
  }
#line 155
  return TRUE;
}






static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void )
#line 163
{
  /* atomic removed: atomic calls only */
#line 164
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state != /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 166
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;

#line 166
        return __nesc_temp;
      }
#line 167
    {
      unsigned char __nesc_temp = 
#line 167
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId;

#line 167
      return __nesc_temp;
    }
  }
}

# 101 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
__attribute((wakeup)) __attribute((interrupt(16)))  void sig_UART0TX_VECTOR(void )
#line 101
{
  if (HplMsp430Usart0P__HplI2C__isI2C()) {
    HplMsp430Usart0P__I2CInterrupts__fired();
    }
  else {
#line 105
    HplMsp430Usart0P__Interrupts__txDone();
    }
}

# 120 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
__attribute((wakeup)) __attribute((interrupt(14)))  void sig_ADC_VECTOR(void )
#line 120
{
  HplAdc12P__HplAdc12__conversionDone(HplAdc12P__ADC12IV);
}

# 503 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static void Msp430Adc12ImplP__stopConversion(void )
{
  uint8_t i;

  if (Msp430Adc12ImplP__state & Msp430Adc12ImplP__USE_TIMERA) {
    Msp430Adc12ImplP__TimerA__setMode(MSP430TIMER_STOP_MODE);
    }
  Msp430Adc12ImplP__resetAdcPin(Msp430Adc12ImplP__HplAdc12__getMCtl(0).inch);
  if (Msp430Adc12ImplP__state & Msp430Adc12ImplP__MULTI_CHANNEL) {
      for (i = 1; i < Msp430Adc12ImplP__numChannels; i++) 
        Msp430Adc12ImplP__resetAdcPin(Msp430Adc12ImplP__HplAdc12__getMCtl(i).inch);
    }
  /* atomic removed: atomic calls only */
#line 515
  {
    Msp430Adc12ImplP__HplAdc12__stopConversion();
    Msp430Adc12ImplP__HplAdc12__resetIFGs();
    Msp430Adc12ImplP__state &= ~Msp430Adc12ImplP__ADC_BUSY;
  }
}

#line 159
static void Msp430Adc12ImplP__resetAdcPin(uint8_t inch)
{

  switch (inch) 
    {
      case 0: Msp430Adc12ImplP__Port60__selectIOFunc();
#line 164
      break;
      case 1: Msp430Adc12ImplP__Port61__selectIOFunc();
#line 165
      break;
      case 2: Msp430Adc12ImplP__Port62__selectIOFunc();
#line 166
      break;
      case 3: Msp430Adc12ImplP__Port63__selectIOFunc();
#line 167
      break;
      case 4: Msp430Adc12ImplP__Port64__selectIOFunc();
#line 168
      break;
      case 5: Msp430Adc12ImplP__Port65__selectIOFunc();
#line 169
      break;
      case 6: Msp430Adc12ImplP__Port66__selectIOFunc();
#line 170
      break;
      case 7: Msp430Adc12ImplP__Port67__selectIOFunc();
#line 171
      break;
    }
}

# 242 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
static error_t AdcStreamP__SingleChannel__singleDataReady(uint8_t streamClient, uint16_t data)
{
  if (AdcStreamP__client == AdcStreamP__NSTREAM) {
    return FAIL;
    }
  if (AdcStreamP__count == 0) 
    {
      AdcStreamP__now = AdcStreamP__Alarm__getNow();
      AdcStreamP__nextBuffer(TRUE);
    }
  else 
    {
      * AdcStreamP__pos++ = data;
      if (AdcStreamP__pos == AdcStreamP__buffer + AdcStreamP__count) 
        {
          /* atomic removed: atomic calls only */
          {
            if (AdcStreamP__lastBuffer) 
              {

                AdcStreamP__bufferQueueEnd[AdcStreamP__client] = (void *)0;
                AdcStreamP__readStreamFail__postTask();
                {
                  unsigned char __nesc_temp = 
#line 264
                  FAIL;

#line 264
                  return __nesc_temp;
                }
              }
            else {
                AdcStreamP__lastCount = AdcStreamP__count;
                AdcStreamP__lastBuffer = AdcStreamP__buffer;
              }
          }
          AdcStreamP__bufferDone__postTask();
          AdcStreamP__nextBuffer(TRUE);
        }
      else {
        AdcStreamP__nextAlarm();
        }
    }
#line 278
  return FAIL;
}

# 142 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static error_t AdcP__SingleChannel__singleDataReady(uint8_t client, uint16_t data)
{
  switch (AdcP__state) 
    {
      case AdcP__STATE_READ: 
        AdcP__owner = client;
      AdcP__value = data;
      AdcP__readDone__postTask();
      break;
      case AdcP__STATE_READNOW: 
        AdcP__ReadNow__readDone(client, SUCCESS, data);
      break;
      default: 

        break;
    }
  return SUCCESS;
}

# 281 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc"
static uint16_t *AdcStreamP__SingleChannel__multipleDataReady(uint8_t streamClient, 
uint16_t *buf, uint16_t length)
{
  /* atomic removed: atomic calls only */
  {
    if (AdcStreamP__lastBuffer) 
      {

        AdcStreamP__bufferQueueEnd[AdcStreamP__client] = (void *)0;
        AdcStreamP__readStreamFail__postTask();
        {
          unsigned int *__nesc_temp = 
#line 291
          0;

#line 291
          return __nesc_temp;
        }
      }
    else {
        AdcStreamP__lastBuffer = AdcStreamP__buffer;
        AdcStreamP__lastCount = AdcStreamP__pos - AdcStreamP__buffer;
      }
  }
  AdcStreamP__bufferDone__postTask();
  AdcStreamP__nextMultiple(streamClient);
  return 0;
}

