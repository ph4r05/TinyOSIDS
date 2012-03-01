#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 149 "/usr/lib/gcc/avr/4.5.3/include/stddef.h" 3
typedef int ptrdiff_t;
#line 211
typedef unsigned int size_t;
#line 323
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
# 121 "/usr/lib/gcc/avr/4.5.3/../../../../avr/include/stdint.h" 3
typedef int int8_t __attribute((__mode__(__QI__))) ;
typedef unsigned int uint8_t __attribute((__mode__(__QI__))) ;
typedef int int16_t __attribute((__mode__(__HI__))) ;
typedef unsigned int uint16_t __attribute((__mode__(__HI__))) ;
typedef int int32_t __attribute((__mode__(__SI__))) ;
typedef unsigned int uint32_t __attribute((__mode__(__SI__))) ;

typedef int int64_t __attribute((__mode__(__DI__))) ;
typedef unsigned int uint64_t __attribute((__mode__(__DI__))) ;
#line 142
typedef int16_t intptr_t;




typedef uint16_t uintptr_t;
#line 159
typedef int8_t int_least8_t;




typedef uint8_t uint_least8_t;




typedef int16_t int_least16_t;




typedef uint16_t uint_least16_t;




typedef int32_t int_least32_t;




typedef uint32_t uint_least32_t;







typedef int64_t int_least64_t;






typedef uint64_t uint_least64_t;
#line 213
typedef int8_t int_fast8_t;




typedef uint8_t uint_fast8_t;




typedef int16_t int_fast16_t;




typedef uint16_t uint_fast16_t;




typedef int32_t int_fast32_t;




typedef uint32_t uint_fast32_t;







typedef int64_t int_fast64_t;






typedef uint64_t uint_fast64_t;
#line 273
typedef int64_t intmax_t;




typedef uint64_t uintmax_t;
# 77 "/usr/lib/gcc/avr/4.5.3/../../../../avr/include/inttypes.h" 3
typedef int32_t int_farptr_t;



typedef uint32_t uint_farptr_t;
# 235 "/usr/lib/ncc/nesc_nx.h"
static __inline uint8_t __nesc_ntoh_uint8(const void * source)  ;




static __inline uint8_t __nesc_hton_uint8(void * target, uint8_t value)  ;





static __inline uint8_t __nesc_ntoh_leuint8(const void * source)  ;




static __inline uint8_t __nesc_hton_leuint8(void * target, uint8_t value)  ;
#line 264
static __inline uint16_t __nesc_ntoh_uint16(const void * source)  ;




static __inline uint16_t __nesc_hton_uint16(void * target, uint16_t value)  ;






static __inline uint16_t __nesc_ntoh_leuint16(const void * source)  ;




static __inline uint16_t __nesc_hton_leuint16(void * target, uint16_t value)  ;






static __inline int16_t __nesc_hton_int16(void * target, int16_t value)  ;





static __inline uint32_t __nesc_ntoh_uint32(const void * source)  ;






static __inline uint32_t __nesc_hton_uint32(void * target, uint32_t value)  ;
#line 326
static __inline int32_t __nesc_ntoh_int32(const void * source)  ;
#line 326
static __inline int32_t __nesc_hton_int32(void * target, int32_t value)  ;
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
# 121 "/usr/lib/gcc/avr/4.5.3/../../../../avr/include/string.h" 3
extern void *memset(void *arg_0xf762bee8, int arg_0xf762a068, size_t arg_0xf762a200);
# 71 "/usr/lib/gcc/avr/4.5.3/../../../../avr/include/stdlib.h" 3
#line 68
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;





#line 74
typedef struct __nesc_unnamed4243 {
  long quot;
  long rem;
} ldiv_t;


typedef int (*__compar_fn_t)(const void *arg_0xf7617320, const void *arg_0xf76174b8);
# 23 "/opt/tinyos-2.x/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4244 {
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
enum __nesc_unnamed4245 {
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
# 244 "/usr/lib/gcc/avr/4.5.3/../../../../avr/include/avr/fuse.h" 3
#line 239
typedef struct __nesc_unnamed4246 {

  unsigned char low;
  unsigned char high;
  unsigned char extended;
} __fuse_t;
# 217 "/usr/lib/gcc/avr/4.5.3/../../../../avr/include/avr/pgmspace.h" 3
typedef void prog_void __attribute((__progmem__)) ;
typedef char prog_char __attribute((__progmem__)) ;
typedef unsigned char prog_uchar __attribute((__progmem__)) ;

typedef int8_t prog_int8_t __attribute((__progmem__)) ;
typedef uint8_t prog_uint8_t __attribute((__progmem__)) ;
typedef int16_t prog_int16_t __attribute((__progmem__)) ;
typedef uint16_t prog_uint16_t __attribute((__progmem__)) ;
typedef int32_t prog_int32_t __attribute((__progmem__)) ;
typedef uint32_t prog_uint32_t __attribute((__progmem__)) ;

typedef int64_t prog_int64_t __attribute((__progmem__)) ;
typedef uint64_t prog_uint64_t __attribute((__progmem__)) ;
# 25 "/opt/tinyos-2.x/tos/chips/atm128/atm128const.h"
typedef uint8_t const_uint8_t __attribute((__progmem__)) ;
typedef uint16_t const_uint16_t __attribute((__progmem__)) ;
typedef uint32_t const_uint32_t __attribute((__progmem__)) ;
typedef int8_t const_int8_t __attribute((__progmem__)) ;
typedef int16_t const_int16_t __attribute((__progmem__)) ;
typedef int32_t const_int32_t __attribute((__progmem__)) ;
# 105 "/opt/tinyos-2.x/tos/chips/atm1281/atm128hardware.h"
static __inline void __nesc_enable_interrupt()  ;



static __inline void __nesc_disable_interrupt()  ;




typedef uint8_t __nesc_atomic_t;
__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t original_SREG);









#line 125
__inline __nesc_atomic_t 
__nesc_atomic_start(void )   ;









#line 135
__inline void 
__nesc_atomic_end(__nesc_atomic_t original_SREG)   ;







typedef uint8_t mcu_power_t  ;


enum __nesc_unnamed4247 {
  ATM128_POWER_IDLE = 0, 
  ATM128_POWER_ADC_NR = 1, 
  ATM128_POWER_EXT_STANDBY = 2, 
  ATM128_POWER_SAVE = 3, 
  ATM128_POWER_STANDBY = 4, 
  ATM128_POWER_DOWN = 5
};


static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)  ;
#line 170
#line 162
typedef struct __nesc_unnamed4248 {

  uint8_t porf : 1;
  uint8_t extrf : 1;
  uint8_t borf : 1;
  uint8_t wdrf : 1;
  uint8_t jtrf : 1;
  uint8_t resv1 : 3;
} Atm128_MCUSR_t;










#line 173
typedef struct __nesc_unnamed4249 {

  uint8_t srw00 : 1;
  uint8_t srw01 : 1;
  uint8_t srw10 : 1;
  uint8_t srw11 : 1;
  uint8_t srl : 3;
  uint8_t sre : 1;
} Atm128_XMCRA_t;







#line 184
typedef struct __nesc_unnamed4250 {

  uint8_t xmm : 3;
  uint8_t resv1 : 4;
  uint8_t xmbk : 1;
} Atm128_XMCRB_t;







typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_float;typedef float __nesc_nxbase_nx_float  ;
# 62 "/opt/tinyos-2.x/tos/chips/atm1281/adc/Atm128Adc.h"
enum __nesc_unnamed4251 {
  ATM128_ADC_VREF_OFF = 0, 
  ATM128_ADC_VREF_AVCC = 1, 
  ATM128_ADC_VREF_RSVD, 
  ATM128_ADC_VREF_2_56 = 3
};


enum __nesc_unnamed4252 {
  ATM128_ADC_RIGHT_ADJUST = 0, 
  ATM128_ADC_LEFT_ADJUST = 1
};



enum __nesc_unnamed4253 {
  ATM128_ADC_SNGL_ADC0 = 0, 
  ATM128_ADC_SNGL_ADC1, 
  ATM128_ADC_SNGL_ADC2, 
  ATM128_ADC_SNGL_ADC3, 
  ATM128_ADC_SNGL_ADC4, 
  ATM128_ADC_SNGL_ADC5, 
  ATM128_ADC_SNGL_ADC6, 
  ATM128_ADC_SNGL_ADC7, 
  ATM128_ADC_DIFF_ADC00_10x, 
  ATM128_ADC_DIFF_ADC10_10x, 
  ATM128_ADC_DIFF_ADC00_200x, 
  ATM128_ADC_DIFF_ADC10_200x, 
  ATM128_ADC_DIFF_ADC22_10x, 
  ATM128_ADC_DIFF_ADC32_10x, 
  ATM128_ADC_DIFF_ADC22_200x, 
  ATM128_ADC_DIFF_ADC32_200x, 
  ATM128_ADC_DIFF_ADC01_1x, 
  ATM128_ADC_DIFF_ADC11_1x, 
  ATM128_ADC_DIFF_ADC21_1x, 
  ATM128_ADC_DIFF_ADC31_1x, 
  ATM128_ADC_DIFF_ADC41_1x, 
  ATM128_ADC_DIFF_ADC51_1x, 
  ATM128_ADC_DIFF_ADC61_1x, 
  ATM128_ADC_DIFF_ADC71_1x, 
  ATM128_ADC_DIFF_ADC02_1x, 
  ATM128_ADC_DIFF_ADC12_1x, 
  ATM128_ADC_DIFF_ADC22_1x, 
  ATM128_ADC_DIFF_ADC32_1x, 
  ATM128_ADC_DIFF_ADC42_1x, 
  ATM128_ADC_DIFF_ADC52_1x, 
  ATM128_ADC_SNGL_1_23, 
  ATM128_ADC_SNGL_GND
};







#line 113
typedef struct __nesc_unnamed4254 {

  uint8_t mux : 5;
  uint8_t adlar : 1;
  uint8_t refs : 2;
} Atm128Admux_t;




enum __nesc_unnamed4255 {
  ATM128_ADC_PRESCALE_2 = 0, 
  ATM128_ADC_PRESCALE_2b, 
  ATM128_ADC_PRESCALE_4, 
  ATM128_ADC_PRESCALE_8, 
  ATM128_ADC_PRESCALE_16, 
  ATM128_ADC_PRESCALE_32, 
  ATM128_ADC_PRESCALE_64, 
  ATM128_ADC_PRESCALE_128, 



  ATM128_ADC_PRESCALE
};


enum __nesc_unnamed4256 {
  ATM128_ADC_ENABLE_OFF = 0, 
  ATM128_ADC_ENABLE_ON
};


enum __nesc_unnamed4257 {
  ATM128_ADC_START_CONVERSION_OFF = 0, 
  ATM128_ADC_START_CONVERSION_ON
};


enum __nesc_unnamed4258 {
  ATM128_ADC_FREE_RUNNING_OFF = 0, 
  ATM128_ADC_FREE_RUNNING_ON
};


enum __nesc_unnamed4259 {
  ATM128_ADC_INT_FLAG_OFF = 0, 
  ATM128_ADC_INT_FLAG_ON
};


enum __nesc_unnamed4260 {
  ATM128_ADC_INT_ENABLE_OFF = 0, 
  ATM128_ADC_INT_ENABLE_ON
};










#line 169
typedef struct __nesc_unnamed4261 {

  uint8_t adps : 3;
  uint8_t adie : 1;
  uint8_t adif : 1;
  uint8_t adate : 1;
  uint8_t adsc : 1;
  uint8_t aden : 1;
} Atm128Adcsra_t;









#line 180
typedef struct __nesc_unnamed4262 {

  uint8_t adts : 3;
  uint8_t mux5 : 1;
  uint8_t resv1 : 2;
  uint8_t acme : 1;
  uint8_t resv2 : 1;
} Atm128Adcsrb_t;


typedef uint8_t Atm128_ADCH_t;
typedef uint8_t Atm128_ADCL_t;
# 29 "/opt/tinyos-2.x/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4263 {
#line 29
  int notUsed;
} 
#line 29
TMilli;
typedef struct __nesc_unnamed4264 {
#line 30
  int notUsed;
} 
#line 30
T32khz;
typedef struct __nesc_unnamed4265 {
#line 31
  int notUsed;
} 
#line 31
TMicro;
# 61 "/opt/tinyos-2.x/tos/chips/atm1281/timer/Atm128Timer.h"
enum __nesc_unnamed4266 {
  ATM128_CLK8_OFF = 0x0, 
  ATM128_CLK8_NORMAL = 0x1, 
  ATM128_CLK8_DIVIDE_8 = 0x2, 
  ATM128_CLK8_DIVIDE_32 = 0x3, 
  ATM128_CLK8_DIVIDE_64 = 0x4, 
  ATM128_CLK8_DIVIDE_128 = 0x5, 
  ATM128_CLK8_DIVIDE_256 = 0x6, 
  ATM128_CLK8_DIVIDE_1024 = 0x7
};


enum __nesc_unnamed4267 {
  ATM128_CLK16_OFF = 0x0, 
  ATM128_CLK16_NORMAL = 0x1, 
  ATM128_CLK16_DIVIDE_8 = 0x2, 
  ATM128_CLK16_DIVIDE_64 = 0x3, 
  ATM128_CLK16_DIVIDE_256 = 0x4, 
  ATM128_CLK16_DIVIDE_1024 = 0x5, 
  ATM128_CLK16_EXTERNAL_FALL = 0x6, 
  ATM128_CLK16_EXTERNAL_RISE = 0x7
};


enum __nesc_unnamed4268 {
  AVR_CLOCK_OFF = 0, 
  AVR_CLOCK_ON = 1, 
  AVR_CLOCK_DIVIDE_8 = 2
};

enum __nesc_unnamed4269 {
  ATM128_TIMER_COMPARE_NORMAL = 0, 
  ATM128_TIMER_COMPARE_TOGGLE, 
  ATM128_TIMER_COMPARE_CLEAR, 
  ATM128_TIMER_COMPARE_SET
};



enum __nesc_unnamed4270 {
  ATM128_WAVE8_NORMAL = 0, 
  ATM128_WAVE8_PWM, 
  ATM128_WAVE8_CTC, 
  ATM128_WAVE8_PWM_FAST
};


enum __nesc_unnamed4271 {
  ATM128_WAVE16_NORMAL = 0, 
  ATM128_WAVE16_PWM_8BIT, 
  ATM128_WAVE16_PWM_9BIT, 
  ATM128_WAVE16_PWM_10BIT, 
  ATM128_WAVE16_CTC_COMPARE, 
  ATM128_WAVE16_PWM_FAST_8BIT, 
  ATM128_WAVE16_PWM_FAST_9BIT, 
  ATM128_WAVE16_PWM_FAST_10BIT, 
  ATM128_WAVE16_PWM_CAPTURE_LOW, 
  ATM128_WAVE16_PWM_COMPARE_LOW, 
  ATM128_WAVE16_PWM_CAPTURE_HIGH, 
  ATM128_WAVE16_PWM_COMPARE_HIGH, 
  ATM128_WAVE16_CTC_CAPTURE, 
  ATM128_WAVE16_RESERVED, 
  ATM128_WAVE16_PWM_FAST_CAPTURE, 
  ATM128_WAVE16_PWM_FAST_COMPARE
};


enum __nesc_unnamed4272 {
  ATM128_COMPARE_OFF = 0, 
  ATM128_COMPARE_TOGGLE, 
  ATM128_COMPARE_CLEAR, 
  ATM128_COMPARE_SET
};
#line 148
#line 136
typedef union __nesc_unnamed4273 {

  uint8_t flat;
  struct __nesc_unnamed4274 {
    uint8_t wgm00 : 1;
    uint8_t wgm01 : 1;
    uint8_t resv1 : 2;
    uint8_t com0b0 : 1;
    uint8_t com0b1 : 1;
    uint8_t com0a0 : 1;
    uint8_t com0a1 : 1;
  } bits;
} Atm128_TCCR0A_t;
#line 163
#line 151
typedef union __nesc_unnamed4275 {

  uint8_t flat;
  struct __nesc_unnamed4276 {
    uint8_t cs00 : 1;
    uint8_t cs01 : 1;
    uint8_t cs02 : 2;
    uint8_t wgm02 : 1;
    uint8_t resv1 : 2;
    uint8_t foc0b : 1;
    uint8_t foc0a : 1;
  } bits;
} Atm128_TCCR0B_t;
#line 175
#line 166
typedef union __nesc_unnamed4277 {

  uint8_t flat;
  struct __nesc_unnamed4278 {
    uint8_t toie0 : 1;
    uint8_t ocie0a : 1;
    uint8_t ocie0e : 1;
    uint8_t resv1 : 5;
  } bits;
} Atm128_TIMSK0_t;
#line 187
#line 178
typedef union __nesc_unnamed4279 {

  uint8_t flat;
  struct __nesc_unnamed4280 {
    uint8_t tov0 : 1;
    uint8_t ocf0a : 1;
    uint8_t ocf0b : 1;
    uint8_t resv1 : 5;
  } bits;
} Atm128_TIFR0_t;
#line 203
#line 190
typedef union __nesc_unnamed4281 {

  uint8_t flat;
  struct __nesc_unnamed4282 {
    uint8_t tcr2bub : 1;
    uint8_t tcr2aub : 1;
    uint8_t ocr2bub : 1;
    uint8_t ocr2aub : 1;
    uint8_t tcn2ub : 1;
    uint8_t as2 : 1;
    uint8_t exclk : 1;
    uint8_t resv1 : 1;
  } bits;
} Atm128_ASSR_t;
#line 216
#line 206
typedef union __nesc_unnamed4283 {

  uint8_t flat;
  struct __nesc_unnamed4284 {
    uint8_t wgm20 : 1;
    uint8_t wgm21 : 1;
    uint8_t resv1 : 2;
    uint8_t comb : 2;
    uint8_t coma : 2;
  } bits;
} Atm128_TCCR2A_t;
#line 229
#line 219
typedef union __nesc_unnamed4285 {

  uint8_t flat;
  struct __nesc_unnamed4286 {
    uint8_t cs : 3;
    uint8_t wgm22 : 1;
    uint8_t resv1 : 2;
    uint8_t foc2b : 1;
    uint8_t foc2a : 1;
  } bits;
} Atm128_TCCR2B_t;
#line 241
#line 232
typedef union __nesc_unnamed4287 {

  uint8_t flat;
  struct __nesc_unnamed4288 {
    uint8_t toie : 1;
    uint8_t ociea : 1;
    uint8_t ocieb : 1;
    uint8_t resv1 : 5;
  } bits;
} Atm128_TIMSK2_t;
#line 253
#line 244
typedef union __nesc_unnamed4289 {

  uint8_t flat;
  struct __nesc_unnamed4290 {
    uint8_t tov : 1;
    uint8_t ocfa : 1;
    uint8_t ocfb : 1;
    uint8_t resv1 : 5;
  } bits;
} Atm128_TIFR2_t;
#line 266
#line 257
typedef union __nesc_unnamed4291 {

  uint8_t flat;
  struct __nesc_unnamed4292 {
    uint8_t wgm01 : 2;
    uint8_t comc : 2;
    uint8_t comb : 2;
    uint8_t coma : 2;
  } bits;
} Atm128_TCCRA_t;
#line 279
#line 269
typedef union __nesc_unnamed4293 {

  uint8_t flat;
  struct __nesc_unnamed4294 {
    uint8_t cs : 3;
    uint8_t wgm23 : 2;
    uint8_t resv1 : 1;
    uint8_t ices : 1;
    uint8_t icnc : 1;
  } bits;
} Atm128_TCCRB_t;
#line 291
#line 282
typedef union __nesc_unnamed4295 {

  uint8_t flat;
  struct __nesc_unnamed4296 {
    uint8_t resv1 : 5;
    uint8_t focc : 1;
    uint8_t focb : 1;
    uint8_t foca : 1;
  } bits;
} Atm128_TCCRC_t;
#line 306
#line 294
typedef union __nesc_unnamed4297 {

  uint8_t flat;
  struct __nesc_unnamed4298 {
    uint8_t toie : 1;
    uint8_t ociea : 1;
    uint8_t ocieb : 1;
    uint8_t ociec : 1;
    uint8_t resv1 : 1;
    uint8_t icie : 1;
    uint8_t resv2 : 2;
  } bits;
} Atm128_TIMSK_t;
#line 321
#line 309
typedef union __nesc_unnamed4299 {

  uint8_t flat;
  struct __nesc_unnamed4300 {
    uint8_t tov : 1;
    uint8_t ocfa : 1;
    uint8_t ocfb : 1;
    uint8_t ocfc : 1;
    uint8_t resv1 : 1;
    uint8_t icf : 1;
    uint8_t resv2 : 2;
  } bits;
} Atm128_TIFR_t;
#line 333
#line 324
typedef union __nesc_unnamed4301 {

  uint8_t flat;
  struct __nesc_unnamed4302 {
    uint8_t psrsync : 1;
    uint8_t psrasy : 1;
    uint8_t resv1 : 5;
    uint8_t tsm : 1;
  } bits;
} Atm128_GTCCR_t;



typedef uint8_t Atm128_TCNT1H_t;
typedef uint8_t Atm128_TCNT1L_t;
typedef uint8_t Atm128_TCNT3H_t;
typedef uint8_t Atm128_TCNT3L_t;
typedef uint8_t Atm128_TCNT4H_t;
typedef uint8_t Atm128_TCNT4L_t;
typedef uint8_t Atm128_TCNT5H_t;
typedef uint8_t Atm128_TCNT5L_t;


typedef uint8_t Atm128_OCR1AH_t;
typedef uint8_t Atm128_OCR1AL_t;
typedef uint8_t Atm128_OCR1BH_t;
typedef uint8_t Atm128_OCR1BL_t;
typedef uint8_t Atm128_OCR1CH_t;
typedef uint8_t Atm128_OCR1CL_t;


typedef uint8_t Atm128_OCR3AH_t;
typedef uint8_t Atm128_OCR3AL_t;
typedef uint8_t Atm128_OCR3BH_t;
typedef uint8_t Atm128_OCR3BL_t;
typedef uint8_t Atm128_OCR3CH_t;
typedef uint8_t Atm128_OCR3CL_t;


typedef uint8_t Atm128_OCR4AH_t;
typedef uint8_t Atm128_OCR4AL_t;
typedef uint8_t Atm128_OCR4BH_t;
typedef uint8_t Atm128_OCR4BL_t;
typedef uint8_t Atm128_OCR4CH_t;
typedef uint8_t Atm128_OCR4CL_t;


typedef uint8_t Atm128_OCR5AH_t;
typedef uint8_t Atm128_OCR5AL_t;
typedef uint8_t Atm128_OCR5BH_t;
typedef uint8_t Atm128_OCR5BL_t;
typedef uint8_t Atm128_OCR5CH_t;
typedef uint8_t Atm128_OCR5CL_t;


typedef uint8_t Atm128_ICR1H_t;
typedef uint8_t Atm128_ICR1L_t;
typedef uint8_t Atm128_ICR3H_t;
typedef uint8_t Atm128_ICR3L_t;
typedef uint8_t Atm128_ICR4H_t;
typedef uint8_t Atm128_ICR4L_t;
typedef uint8_t Atm128_ICR5H_t;
typedef uint8_t Atm128_ICR5L_t;
# 74 "/opt/tinyos-2.x/tos/platforms/iris/MicaTimer.h"
typedef struct __nesc_unnamed4303 {
} 
#line 74
T64khz;
typedef struct __nesc_unnamed4304 {
} 
#line 75
T128khz;
typedef struct __nesc_unnamed4305 {
} 
#line 76
T2mhz;
typedef struct __nesc_unnamed4306 {
} 
#line 77
T4mhz;
#line 147
typedef TMicro TOne;
typedef TMicro TThree;
typedef uint32_t counter_one_overflow_t;
typedef uint16_t counter_three_overflow_t;
enum __nesc_unnamed4307 {
  MICA_PRESCALER_ONE = ATM128_CLK16_DIVIDE_8, 
  MICA_DIVIDE_ONE_FOR_32KHZ_LOG2 = 5, 
  MICA_PRESCALER_THREE = ATM128_CLK16_DIVIDE_8, 
  MICA_DIVIDE_THREE_FOR_MICRO_LOG2 = 0, 
  EXT_STANDBY_T0_THRESHOLD = 12
};





enum __nesc_unnamed4308 {
  PLATFORM_MHZ = 8
};
# 56 "/opt/tinyos-2.x/tos/platforms/micaz/hardware.h"
enum __nesc_unnamed4309 {
  TOS_SLEEP_NONE = ATM128_POWER_IDLE
};


enum __nesc_unnamed4310 {
  CHANNEL_THERMISTOR = ATM128_ADC_SNGL_ADC1
};

enum __nesc_unnamed4311 {
  PLATFORM_BAUDRATE = 57600L
};
# 43 "../RssiDemoMessages.h"
enum __nesc_unnamed4312 {
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
#line 150
#line 136
typedef nx_struct MultiPingResponseReportMsg {

  nx_uint16_t counter;


  nx_uint8_t datanum;





  nx_uint16_t nodeid[4];
  nx_uint16_t nodecounter[4];
  nx_int16_t rssi[4];
} __attribute__((packed)) MultiPingResponseReportMsg;
#line 168
#line 154
typedef nx_struct MassReportMsg {

  nx_uint16_t counter;


  nx_uint8_t datanum;





  nx_uint8_t nodeid[4];
  nx_uint16_t nodecounter[4];
  nx_int16_t rssi[4];
} __attribute__((packed)) MassReportMsg;
#line 183
#line 172
typedef nx_struct MultiPingResponseTinyReportMsg {

  nx_uint16_t counter;





  nx_uint16_t nodeid;
  nx_uint16_t nodecounter;
  nx_int16_t rssi;
} __attribute__((packed)) MultiPingResponseTinyReportMsg;
#line 253
#line 228
typedef nx_struct CommandMsg {

  nx_uint8_t command_code;



  nx_uint8_t command_version;





  nx_uint16_t command_id;



  nx_uint8_t reply_on_command;
  nx_uint16_t reply_on_command_id;


  nx_uint16_t command_data;



  nx_uint16_t command_data_next[4];
} __attribute__((packed)) CommandMsg;




enum __nesc_unnamed4313 {
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

  COMMAND_SETREPORTGAP = 19, 


  COMMAND_GETSENSORREADING = 20, 
  COMMAND_SENSORREADING = 21
};









enum __nesc_unnamed4314 {
  NODE_REPORTING = 1, 
  NODE_TALKING = 2, 
  NODE_BS = 3, 
  NODE_DEAD = 4
};





enum __nesc_unnamed4315 {
  REPORTING_MEDIUM = 1, 
  REPORTING_TINY = 2, 
  REPORTING_MASS = 3
};




enum __nesc_unnamed4316 {








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
# 30 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayer.h"
#line 27
typedef nx_struct rf230_header_t {

  nxle_uint8_t length;
} __attribute__((packed)) rf230_header_t;









#line 32
typedef struct rf230_metadata_t {

  uint8_t lqi;
  union  {

    uint8_t power;
    uint8_t rssi;
  } ;
} rf230_metadata_t;

enum rf230_registers_enum {

  RF230_TRX_STATUS = 0x01, 
  RF230_TRX_STATE = 0x02, 
  RF230_TRX_CTRL_0 = 0x03, 
  RF230_PHY_TX_PWR = 0x05, 
  RF230_PHY_RSSI = 0x06, 
  RF230_PHY_ED_LEVEL = 0x07, 
  RF230_PHY_CC_CCA = 0x08, 
  RF230_CCA_THRES = 0x09, 
  RF230_IRQ_MASK = 0x0E, 
  RF230_IRQ_STATUS = 0x0F, 
  RF230_VREG_CTRL = 0x10, 
  RF230_BATMON = 0x11, 
  RF230_XOSC_CTRL = 0x12, 
  RF230_PLL_CF = 0x1A, 
  RF230_PLL_DCU = 0x1B, 
  RF230_PART_NUM = 0x1C, 
  RF230_VERSION_NUM = 0x1D, 
  RF230_MAN_ID_0 = 0x1E, 
  RF230_MAN_ID_1 = 0x1F, 
  RF230_SHORT_ADDR_0 = 0x20, 
  RF230_SHORT_ADDR_1 = 0x21, 
  RF230_PAN_ID_0 = 0x22, 
  RF230_PAN_ID_1 = 0x23, 
  RF230_IEEE_ADDR_0 = 0x24, 
  RF230_IEEE_ADDR_1 = 0x25, 
  RF230_IEEE_ADDR_2 = 0x26, 
  RF230_IEEE_ADDR_3 = 0x27, 
  RF230_IEEE_ADDR_4 = 0x28, 
  RF230_IEEE_ADDR_5 = 0x29, 
  RF230_IEEE_ADDR_6 = 0x2A, 
  RF230_IEEE_ADDR_7 = 0x2B, 
  RF230_XAH_CTRL = 0x2C, 
  RF230_CSMA_SEED_0 = 0x2D, 
  RF230_CSMA_SEED_1 = 0x2E
};

enum rf230_trx_register_enums {

  RF230_CCA_DONE = 1 << 7, 
  RF230_CCA_STATUS = 1 << 6, 
  RF230_TRX_STATUS_MASK = 0x1F, 
  RF230_P_ON = 0, 
  RF230_BUSY_RX = 1, 
  RF230_BUSY_TX = 2, 
  RF230_RX_ON = 6, 
  RF230_TRX_OFF = 8, 
  RF230_PLL_ON = 9, 
  RF230_SLEEP = 15, 
  RF230_BUSY_RX_AACK = 17, 
  RF230_BUSR_TX_ARET = 18, 
  RF230_RX_AACK_ON = 22, 
  RF230_TX_ARET_ON = 25, 
  RF230_RX_ON_NOCLK = 28, 
  RF230_AACK_ON_NOCLK = 29, 
  RF230_BUSY_RX_AACK_NOCLK = 30, 
  RF230_STATE_TRANSITION_IN_PROGRESS = 31, 
  RF230_TRAC_STATUS_MASK = 0xE0, 
  RF230_TRAC_SUCCESS = 0, 
  RF230_TRAC_SUCCESS_DATA_PENDING = 1 << 5, 
  RF230_TRAC_CHANNEL_ACCESS_FAILURE = 3 << 5, 
  RF230_TRAC_NO_ACK = 5 << 5, 
  RF212_TRAC_INVALID = 7 << 5, 
  RF230_TRX_CMD_MASK = 0x1F, 
  RF230_NOP = 0, 
  RF230_TX_START = 2, 
  RF230_FORCE_TRX_OFF = 3
};

enum rf230_phy_register_enums {

  RF230_TX_AUTO_CRC_ON = 1 << 7, 
  RF230_TX_PWR_MASK = 0x0F, 
  RF230_RSSI_MASK = 0x1F, 
  RF230_CCA_REQUEST = 1 << 7, 
  RF230_CCA_MODE_0 = 0 << 5, 
  RF230_CCA_MODE_1 = 1 << 5, 
  RF230_CCA_MODE_2 = 2 << 5, 
  RF230_CCA_MODE_3 = 3 << 5, 
  RF230_CHANNEL_DEFAULT = 11, 
  RF230_CHANNEL_MASK = 0x1F, 
  RF230_CCA_CS_THRES_SHIFT = 4, 
  RF230_CCA_ED_THRES_SHIFT = 0
};

enum rf230_irq_register_enums {

  RF230_IRQ_BAT_LOW = 1 << 7, 
  RF230_IRQ_TRX_UR = 1 << 6, 
  RF230_IRQ_TRX_END = 1 << 3, 
  RF230_IRQ_RX_START = 1 << 2, 
  RF230_IRQ_PLL_UNLOCK = 1 << 1, 
  RF230_IRQ_PLL_LOCK = 1 << 0
};

enum rf230_control_register_enums {

  RF230_AVREG_EXT = 1 << 7, 
  RF230_AVDD_OK = 1 << 6, 
  RF230_DVREG_EXT = 1 << 3, 
  RF230_DVDD_OK = 1 << 2, 
  RF230_BATMON_OK = 1 << 5, 
  RF230_BATMON_VHR = 1 << 4, 
  RF230_BATMON_VTH_MASK = 0x0F, 
  RF230_XTAL_MODE_OFF = 0 << 4, 
  RF230_XTAL_MODE_EXTERNAL = 4 << 4, 
  RF230_XTAL_MODE_INTERNAL = 15 << 4
};

enum rf230_pll_register_enums {

  RF230_PLL_CF_START = 1 << 7, 
  RF230_PLL_DCU_START = 1 << 7
};

enum rf230_spi_command_enums {

  RF230_CMD_REGISTER_READ = 0x80, 
  RF230_CMD_REGISTER_WRITE = 0xC0, 
  RF230_CMD_REGISTER_MASK = 0x3F, 
  RF230_CMD_FRAME_READ = 0x20, 
  RF230_CMD_FRAME_WRITE = 0x60, 
  RF230_CMD_SRAM_READ = 0x00, 
  RF230_CMD_SRAM_WRITE = 0x40
};
# 251 "/usr/lib/gcc/avr/4.5.3/../../../../avr/include/util/crc16.h" 3
#line 250
static __inline uint16_t 
_crc_ccitt_update(uint16_t __crc, uint8_t __data);
# 31 "/opt/tinyos-2.x/tos/platforms/iris/chips/rf230/RadioConfig.h"
enum __nesc_unnamed4317 {





  RF230_TRX_CTRL_0_VALUE = 0, 





  RF230_CCA_MODE_VALUE = RF230_CCA_MODE_3, 





  RF230_CCA_THRES_VALUE = 0xC7
};
#line 71
static __inline uint16_t RF230_CRCBYTE_COMMAND(uint16_t crc, uint8_t data);







typedef TOne TRadio;
# 34 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TinyosNetworkLayer.h"
#line 31
typedef nx_struct network_header_t {

  nxle_uint8_t network;
} __attribute__((packed)) network_header_t;
# 34 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayer.h"
#line 27
typedef nx_struct ieee154_header_t {

  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;
} __attribute__((packed)) ieee154_header_t;

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
  IEEE154_TYPE_MAC_CMD = 3, 
  IEEE154_TYPE_MASK = 7
};

enum iee154_fcf_addr_mode_enums {
  IEEE154_ADDR_NONE = 0, 
  IEEE154_ADDR_SHORT = 2, 
  IEEE154_ADDR_EXT = 3, 
  IEEE154_ADDR_MASK = 3
};
# 6 "/opt/tinyos-2.x/tos/types/AM.h"
typedef nx_uint8_t nx_am_id_t;
typedef nx_uint8_t nx_am_group_t;
typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;
typedef uint8_t am_group_t;
typedef uint16_t am_addr_t;

enum __nesc_unnamed4318 {
  AM_BROADCAST_ADDR = 0xffff
};









enum __nesc_unnamed4319 {
  TOS_AM_GROUP = 0x22, 
  TOS_AM_ADDRESS = 1
};
# 32 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayer.h"
#line 29
typedef nx_struct activemessage_header_t {

  nx_am_id_t type;
} __attribute__((packed)) activemessage_header_t;
# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MetadataFlagsLayer.h"
#line 27
typedef struct flags_metadata_t {


  uint8_t flags;
} flags_metadata_t;
# 30 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TimeStampingLayer.h"
#line 27
typedef struct timestamp_metadata_t {

  uint32_t timestamp;
} timestamp_metadata_t;
# 30 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/LowPowerListeningLayer.h"
#line 27
typedef struct lpl_metadata_t {

  uint16_t sleepint;
} lpl_metadata_t;
# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/PacketLinkLayer.h"
#line 27
typedef struct link_metadata_t {

  uint16_t maxRetries;
  uint16_t retryDelay;
} link_metadata_t;
# 47 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230Radio.h"
#line 37
typedef nx_struct rf230packet_header_t {

  rf230_header_t rf230;
  ieee154_header_t ieee154;

  network_header_t network;


  activemessage_header_t am;
} __attribute__((packed)) 
rf230packet_header_t;




#line 49
typedef nx_struct rf230packet_footer_t {
} __attribute__((packed)) 

rf230packet_footer_t;
#line 65
#line 54
typedef struct rf230packet_metadata_t {





  link_metadata_t link;

  timestamp_metadata_t timestamp;
  flags_metadata_t flags;
  rf230_metadata_t rf230;
} rf230packet_metadata_t;
# 72 "/opt/tinyos-2.x/tos/lib/serial/Serial.h"
typedef uint8_t uart_id_t;



enum __nesc_unnamed4320 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4321 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4322 {
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
# 32 "/opt/tinyos-2.x/tos/platforms/iris/platform_message.h"
#line 29
typedef union message_header {
  rf230packet_header_t rf230;
  serial_header_t serial;
} message_header_t;



#line 34
typedef union message_footer {
  rf230packet_footer_t rf230;
} message_footer_t;



#line 38
typedef union message_metadata {
  rf230packet_metadata_t rf230;
} message_metadata_t;
# 19 "/opt/tinyos-2.x/tos/types/message.h"
#line 14
typedef nx_struct message_t {
  nx_uint8_t header[sizeof(message_header_t )];
  nx_uint8_t data[28];
  nx_uint8_t footer[sizeof(message_footer_t )];
  nx_uint8_t metadata[sizeof(message_metadata_t )];
} __attribute__((packed)) message_t;
# 32 "/opt/tinyos-2.x/tos/types/Leds.h"
enum __nesc_unnamed4323 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 54 "../InterceptBase/Reset.h"
static inline void resetMote();
# 33 "/opt/tinyos-2.x/tos/types/Ieee154.h"
typedef uint16_t ieee154_panid_t;
typedef uint16_t ieee154_saddr_t;

enum __nesc_unnamed4324 {
  IEEE154_BROADCAST_ADDR = 0xffff
};
# 33 "/opt/tinyos-2.x/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
# 34 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TimeSyncMessageLayer.h"
typedef nx_int32_t timesync_relative_t;


typedef uint32_t timesync_absolute_t;









#line 39
typedef struct timesync_footer_t {

  nx_am_id_t type;
  union  {

    timesync_relative_t relative;
    timesync_absolute_t absolute;
  } ;
} timesync_footer_t;
# 32 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128Spi.h"
enum __nesc_unnamed4325 {
  ATM128_SPI_CLK_DIVIDE_4 = 0, 
  ATM128_SPI_CLK_DIVIDE_16 = 1, 
  ATM128_SPI_CLK_DIVIDE_64 = 2, 
  ATM128_SPI_CLK_DIVIDE_128 = 3
};
#line 49
#line 40
typedef struct __nesc_unnamed4326 {
  uint8_t spie : 1;
  uint8_t spe : 1;
  uint8_t dord : 1;
  uint8_t mstr : 1;
  uint8_t cpol : 1;
  uint8_t cpha : 1;
  uint8_t spr : 2;
} 
Atm128SPIControl_s;



#line 50
typedef union __nesc_unnamed4327 {
  uint8_t flat;
  Atm128SPIControl_s bits;
} Atm128SPIControl_t;

typedef Atm128SPIControl_t Atm128_SPCR_t;








#line 58
typedef struct __nesc_unnamed4328 {
  uint8_t spif : 1;
  uint8_t wcol : 1;
  uint8_t rsvd : 5;
  uint8_t spi2x : 1;
} 
Atm128SPIStatus_s;



#line 65
typedef union __nesc_unnamed4329 {
  uint8_t flat;
  Atm128SPIStatus_s bits;
} Atm128SPIStatus_t;

typedef Atm128SPIStatus_t Atm128_SPSR_t;

typedef uint8_t Atm128_SPDR_t;
# 39 "/opt/tinyos-2.x/tos/chips/atm128/crc.h"
uint16_t crcTable[256] __attribute((__progmem__))  = { 
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 
0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 
0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 
0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 
0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 
0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 
0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 
0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 
0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 
0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 
0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 
0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 
0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 
0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 
0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 
0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 
0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 
0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067, 
0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 
0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 
0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 
0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 
0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 
0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 
0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 
0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 
0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 
0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 
0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 
0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 
0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 
0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 };









static uint16_t crcByte(uint16_t oldCrc, uint8_t byte) __attribute((noinline))  ;
# 32 "/opt/tinyos-2.x/tos/chips/atm128/Atm128Uart.h"
typedef uint8_t Atm128_UDR0_t;
typedef uint8_t Atm128_UDR1_t;
#line 48
#line 36
typedef union __nesc_unnamed4330 {
  struct Atm128_UCSRA_t {
    uint8_t mpcm : 1;
    uint8_t u2x : 1;
    uint8_t upe : 1;
    uint8_t dor : 1;
    uint8_t fe : 1;
    uint8_t udre : 1;
    uint8_t txc : 1;
    uint8_t rxc : 1;
  } bits;
  uint8_t flat;
} Atm128UartStatus_t;

typedef Atm128UartStatus_t Atm128_UCSR0A_t;
typedef Atm128UartStatus_t Atm128_UCSR1A_t;
#line 66
#line 54
typedef union __nesc_unnamed4331 {
  struct Atm128_UCSRB_t {
    uint8_t txb8 : 1;
    uint8_t rxb8 : 1;
    uint8_t ucsz2 : 1;
    uint8_t txen : 1;
    uint8_t rxen : 1;
    uint8_t udrie : 1;
    uint8_t txcie : 1;
    uint8_t rxcie : 1;
  } bits;
  uint8_t flat;
} Atm128UartControl_t;

typedef Atm128UartControl_t Atm128_UCSR0B_t;
typedef Atm128UartControl_t Atm128_UCSR1B_t;

enum __nesc_unnamed4332 {
  ATM128_UART_DATA_SIZE_5_BITS = 0, 
  ATM128_UART_DATA_SIZE_6_BITS = 1, 
  ATM128_UART_DATA_SIZE_7_BITS = 2, 
  ATM128_UART_DATA_SIZE_8_BITS = 3
};
#line 89
#line 79
typedef union __nesc_unnamed4333 {
  uint8_t flat;
  struct Atm128_UCSRC_t {
    uint8_t ucpol : 1;
    uint8_t ucsz : 2;
    uint8_t usbs : 1;
    uint8_t upm : 2;
    uint8_t umsel : 1;
    uint8_t rsvd : 1;
  } bits;
} Atm128UartMode_t;

typedef Atm128UartMode_t Atm128_UCSR0C_t;
typedef Atm128UartMode_t Atm128_UCSR1C_t;





enum __nesc_unnamed4334 {
  ATM128_19200_BAUD_4MHZ = 12, 
  ATM128_38400_BAUD_4MHZ = 6, 
  ATM128_57600_BAUD_4MHZ = 3, 

  ATM128_19200_BAUD_4MHZ_2X = 25, 
  ATM128_38400_BAUD_4MHZ_2X = 12, 
  ATM128_57600_BAUD_4MHZ_2X = 8, 

  ATM128_19200_BAUD_7MHZ = 23, 
  ATM128_38400_BAUD_7MHZ = 11, 
  ATM128_57600_BAUD_7MHZ = 7, 

  ATM128_19200_BAUD_7MHZ_2X = 47, 
  ATM128_38400_BAUD_7MHZ_2X = 23, 
  ATM128_57600_BAUD_7MHZ_2X = 15, 

  ATM128_19200_BAUD_8MHZ = 25, 
  ATM128_38400_BAUD_8MHZ = 12, 
  ATM128_57600_BAUD_8MHZ = 8, 

  ATM128_19200_BAUD_8MHZ_2X = 51, 
  ATM128_38400_BAUD_8MHZ_2X = 34, 
  ATM128_57600_BAUD_8MHZ_2X = 11
};

typedef uint8_t Atm128_UBRR0L_t;
typedef uint8_t Atm128_UBRR0H_t;

typedef uint8_t Atm128_UBRR1L_t;
typedef uint8_t Atm128_UBRR1H_t;

typedef uint8_t uart_parity_t;
typedef uint8_t uart_speed_t;
typedef uint8_t uart_duplex_t;

enum __nesc_unnamed4335 {
  TOS_UART_PARITY_NONE = 0, 
  TOS_UART_PARITY_EVEN = 1, 
  TOS_UART_PARITY_ODD = 2
};

enum __nesc_unnamed4336 {
  TOS_UART_19200 = 0, 
  TOS_UART_38400 = 1, 
  TOS_UART_57600 = 2
};

enum __nesc_unnamed4337 {
  TOS_UART_OFF = 0, 
  TOS_UART_RONLY = 1, 
  TOS_UART_TONLY = 2, 
  TOS_UART_DUPLEX = 3
};
typedef TMilli BaseStationP__RadioTimer__precision_tag;
typedef TMilli BaseStationP__ResetTimer__precision_tag;
typedef TMilli BaseStationP__UartTimer__precision_tag;
typedef TRadio RF230RadioP__PacketTimeStamp__precision_tag;
typedef uint32_t RF230RadioP__PacketTimeStamp__size_type;
typedef TRadio RadioAlarmP__Alarm__precision_tag;
typedef uint16_t RadioAlarmP__Alarm__size_type;
typedef TMilli PacketLinkLayerP__DelayTimer__precision_tag;
enum HilTimerMilliC____nesc_unnamed4338 {
  HilTimerMilliC__TIMER_COUNT = 5U
};
typedef TMilli /*AlarmCounterMilliP.Atm128AlarmAsyncC*/Atm128AlarmAsyncC__0__precision;
typedef /*AlarmCounterMilliP.Atm128AlarmAsyncC*/Atm128AlarmAsyncC__0__precision /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__precision;
typedef /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__precision /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__precision_tag;
typedef uint32_t /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__size_type;
typedef /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__precision /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Counter__precision_tag;
typedef uint32_t /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Counter__size_type;
typedef uint8_t /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Compare__size_type;
typedef uint8_t /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Timer__timer_size;
typedef uint8_t HplAtm1281Timer2AsyncP__Compare__size_type;
typedef uint8_t HplAtm1281Timer2AsyncP__Timer__timer_size;
typedef TMilli /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__precision_tag;
typedef uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__size_type;
typedef uint16_t RandomMlcgC__SeedInit__parameter;
typedef TRadio TimeStampingLayerP__PacketTimeStampRadio__precision_tag;
typedef uint32_t TimeStampingLayerP__PacketTimeStampRadio__size_type;
typedef TRadio TimeStampingLayerP__LocalTimeRadio__precision_tag;
typedef TMilli TimeStampingLayerP__LocalTimeMilli__precision_tag;
typedef TMilli TimeStampingLayerP__PacketTimeStampMilli__precision_tag;
typedef uint32_t TimeStampingLayerP__PacketTimeStampMilli__size_type;
typedef TMicro RF230DriverLayerP__BusyWait__precision_tag;
typedef uint16_t RF230DriverLayerP__BusyWait__size_type;
typedef uint8_t RF230DriverLayerP__PacketRSSI__value_type;
typedef TRadio RF230DriverLayerP__PacketTimeStamp__precision_tag;
typedef uint32_t RF230DriverLayerP__PacketTimeStamp__size_type;
typedef TRadio RF230DriverLayerP__LocalTime__precision_tag;
typedef uint8_t RF230DriverLayerP__PacketTransmitPower__value_type;
typedef uint8_t RF230DriverLayerP__PacketTimeSyncOffset__value_type;
typedef uint8_t RF230DriverLayerP__PacketLinkQuality__value_type;
typedef uint16_t HplRF230P__Capture__size_type;
typedef uint16_t HplAtm1281Timer1P__CompareA__size_type;
typedef uint16_t HplAtm1281Timer1P__Capture__size_type;
typedef uint16_t HplAtm1281Timer1P__CompareB__size_type;
typedef uint16_t HplAtm1281Timer1P__CompareC__size_type;
typedef uint16_t HplAtm1281Timer1P__Timer__timer_size;
typedef uint16_t /*InitOneP.InitOne*/Atm128TimerInitC__0__timer_size;
typedef /*InitOneP.InitOne*/Atm128TimerInitC__0__timer_size /*InitOneP.InitOne*/Atm128TimerInitC__0__Timer__timer_size;
typedef TOne /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__frequency_tag;
typedef uint16_t /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__timer_size;
typedef /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__frequency_tag /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__precision_tag;
typedef /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__timer_size /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__size_type;
typedef /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__timer_size /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Compare__size_type;
typedef /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__timer_size /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Timer__timer_size;
enum /*HplRF230C.AlarmC*/AlarmOne16C__0____nesc_unnamed4339 {
  AlarmOne16C__0__COMPARE_ID = 0U
};
typedef TOne /*CounterOne16C.NCounter*/Atm128CounterC__0__frequency_tag;
typedef uint16_t /*CounterOne16C.NCounter*/Atm128CounterC__0__timer_size;
typedef /*CounterOne16C.NCounter*/Atm128CounterC__0__frequency_tag /*CounterOne16C.NCounter*/Atm128CounterC__0__Counter__precision_tag;
typedef /*CounterOne16C.NCounter*/Atm128CounterC__0__timer_size /*CounterOne16C.NCounter*/Atm128CounterC__0__Counter__size_type;
typedef /*CounterOne16C.NCounter*/Atm128CounterC__0__timer_size /*CounterOne16C.NCounter*/Atm128CounterC__0__Timer__timer_size;
typedef TMicro /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__to_precision_tag;
typedef uint32_t /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__to_size_type;
typedef TMicro /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__from_precision_tag;
typedef uint16_t /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__from_size_type;
typedef uint32_t /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__upper_count_type;
typedef /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__from_precision_tag /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__CounterFrom__precision_tag;
typedef /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__from_size_type /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__CounterFrom__size_type;
typedef /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__to_precision_tag /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__Counter__precision_tag;
typedef /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__to_size_type /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__Counter__size_type;
typedef TMicro /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag;
typedef /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__precision_tag;
typedef /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__precision_tag;
typedef uint32_t /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__size_type;
typedef TMicro BusyWaitMicroC__BusyWait__precision_tag;
typedef uint16_t BusyWaitMicroC__BusyWait__size_type;
typedef TMicro /*Atm128Uart0C.UartP*/Atm128UartP__0__Counter__precision_tag;
typedef uint32_t /*Atm128Uart0C.UartP*/Atm128UartP__0__Counter__size_type;
typedef uint16_t HplAtm1281Timer3P__CompareA__size_type;
typedef uint16_t HplAtm1281Timer3P__Capture__size_type;
typedef uint16_t HplAtm1281Timer3P__CompareB__size_type;
typedef uint16_t HplAtm1281Timer3P__CompareC__size_type;
typedef uint16_t HplAtm1281Timer3P__Timer__timer_size;
typedef uint16_t /*InitThreeP.InitThree*/Atm128TimerInitC__1__timer_size;
typedef /*InitThreeP.InitThree*/Atm128TimerInitC__1__timer_size /*InitThreeP.InitThree*/Atm128TimerInitC__1__Timer__timer_size;
typedef TThree /*CounterThree16C.NCounter*/Atm128CounterC__1__frequency_tag;
typedef uint16_t /*CounterThree16C.NCounter*/Atm128CounterC__1__timer_size;
typedef /*CounterThree16C.NCounter*/Atm128CounterC__1__frequency_tag /*CounterThree16C.NCounter*/Atm128CounterC__1__Counter__precision_tag;
typedef /*CounterThree16C.NCounter*/Atm128CounterC__1__timer_size /*CounterThree16C.NCounter*/Atm128CounterC__1__Counter__size_type;
typedef /*CounterThree16C.NCounter*/Atm128CounterC__1__timer_size /*CounterThree16C.NCounter*/Atm128CounterC__1__Timer__timer_size;
typedef TMicro /*CounterMicro32C.Transform32*/TransformCounterC__1__to_precision_tag;
typedef uint32_t /*CounterMicro32C.Transform32*/TransformCounterC__1__to_size_type;
typedef TThree /*CounterMicro32C.Transform32*/TransformCounterC__1__from_precision_tag;
typedef uint16_t /*CounterMicro32C.Transform32*/TransformCounterC__1__from_size_type;
typedef counter_three_overflow_t /*CounterMicro32C.Transform32*/TransformCounterC__1__upper_count_type;
typedef /*CounterMicro32C.Transform32*/TransformCounterC__1__from_precision_tag /*CounterMicro32C.Transform32*/TransformCounterC__1__CounterFrom__precision_tag;
typedef /*CounterMicro32C.Transform32*/TransformCounterC__1__from_size_type /*CounterMicro32C.Transform32*/TransformCounterC__1__CounterFrom__size_type;
typedef /*CounterMicro32C.Transform32*/TransformCounterC__1__to_precision_tag /*CounterMicro32C.Transform32*/TransformCounterC__1__Counter__precision_tag;
typedef /*CounterMicro32C.Transform32*/TransformCounterC__1__to_size_type /*CounterMicro32C.Transform32*/TransformCounterC__1__Counter__size_type;
typedef TMilli RssiBaseC__SendTimer__precision_tag;
typedef uint8_t RssiBaseC__PacketRSSI__value_type;
enum AMQueueP____nesc_unnamed4340 {
  AMQueueP__NUM_CLIENTS = 1U
};
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
#line 51
static error_t MotePlatformP__PlatformInit__init(void );
# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void /*HplAtm128GeneralIOC.PortA.Bit0*/HplAtm128GeneralIOPinP__0__IO__toggle(void );



static void /*HplAtm128GeneralIOC.PortA.Bit0*/HplAtm128GeneralIOPinP__0__IO__makeOutput(void );
#line 29
static void /*HplAtm128GeneralIOC.PortA.Bit0*/HplAtm128GeneralIOPinP__0__IO__set(void );

static void /*HplAtm128GeneralIOC.PortA.Bit1*/HplAtm128GeneralIOPinP__1__IO__toggle(void );



static void /*HplAtm128GeneralIOC.PortA.Bit1*/HplAtm128GeneralIOPinP__1__IO__makeOutput(void );
#line 29
static void /*HplAtm128GeneralIOC.PortA.Bit1*/HplAtm128GeneralIOPinP__1__IO__set(void );

static void /*HplAtm128GeneralIOC.PortA.Bit2*/HplAtm128GeneralIOPinP__2__IO__toggle(void );



static void /*HplAtm128GeneralIOC.PortA.Bit2*/HplAtm128GeneralIOPinP__2__IO__makeOutput(void );
#line 29
static void /*HplAtm128GeneralIOC.PortA.Bit2*/HplAtm128GeneralIOPinP__2__IO__set(void );



static void /*HplAtm128GeneralIOC.PortA.Bit4*/HplAtm128GeneralIOPinP__4__IO__makeInput(void );
#line 30
static void /*HplAtm128GeneralIOC.PortA.Bit4*/HplAtm128GeneralIOPinP__4__IO__clr(void );




static void /*HplAtm128GeneralIOC.PortA.Bit6*/HplAtm128GeneralIOPinP__6__IO__makeOutput(void );
#line 29
static void /*HplAtm128GeneralIOC.PortA.Bit6*/HplAtm128GeneralIOPinP__6__IO__set(void );
static void /*HplAtm128GeneralIOC.PortA.Bit6*/HplAtm128GeneralIOPinP__6__IO__clr(void );




static void /*HplAtm128GeneralIOC.PortB.Bit0*/HplAtm128GeneralIOPinP__8__IO__makeOutput(void );
#line 29
static void /*HplAtm128GeneralIOC.PortB.Bit0*/HplAtm128GeneralIOPinP__8__IO__set(void );
static void /*HplAtm128GeneralIOC.PortB.Bit0*/HplAtm128GeneralIOPinP__8__IO__clr(void );




static void /*HplAtm128GeneralIOC.PortB.Bit1*/HplAtm128GeneralIOPinP__9__IO__makeOutput(void );
#line 35
static void /*HplAtm128GeneralIOC.PortB.Bit2*/HplAtm128GeneralIOPinP__10__IO__makeOutput(void );
#line 33
static void /*HplAtm128GeneralIOC.PortB.Bit3*/HplAtm128GeneralIOPinP__11__IO__makeInput(void );

static void /*HplAtm128GeneralIOC.PortB.Bit7*/HplAtm128GeneralIOPinP__15__IO__makeOutput(void );
#line 29
static void /*HplAtm128GeneralIOC.PortB.Bit7*/HplAtm128GeneralIOPinP__15__IO__set(void );
static void /*HplAtm128GeneralIOC.PortB.Bit7*/HplAtm128GeneralIOPinP__15__IO__clr(void );


static void /*HplAtm128GeneralIOC.PortD.Bit4*/HplAtm128GeneralIOPinP__28__IO__makeInput(void );
#line 30
static void /*HplAtm128GeneralIOC.PortD.Bit4*/HplAtm128GeneralIOPinP__28__IO__clr(void );


static void /*HplAtm128GeneralIOC.PortD.Bit6*/HplAtm128GeneralIOPinP__30__IO__makeInput(void );
#line 30
static void /*HplAtm128GeneralIOC.PortD.Bit6*/HplAtm128GeneralIOPinP__30__IO__clr(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t MeasureClockC__Init__init(void );
# 60 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128Calibrate.nc"
static uint16_t MeasureClockC__Atm128Calibrate__baudrateRegister(uint32_t baudrate);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
# 45 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0xf7540b38);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 45 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0xf7540b38);
# 46 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP__Scheduler__init(void );
#line 61
static void SchedulerBasicP__Scheduler__taskLoop(void );
#line 54
static bool SchedulerBasicP__Scheduler__runNextTask(void );
# 59 "/opt/tinyos-2.x/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
# 44 "/opt/tinyos-2.x/tos/interfaces/McuPowerState.nc"
static void McuSleepC__McuPowerState__update(void );
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void BaseStationP__RadioTimer__fired(void );
# 49 "/opt/tinyos-2.x/tos/interfaces/Boot.nc"
static void BaseStationP__Boot__booted(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static void BaseStationP__SerialControl__startDone(error_t error);
#line 117
static void BaseStationP__SerialControl__stopDone(error_t error);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



BaseStationP__UartReceive__receive(
# 71 "../InterceptBase/BaseStationP.nc"
am_id_t arg_0xf73ba5a8, 
# 60 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void BaseStationP__uartSendTask__runTask(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static void BaseStationP__RadioControl__startDone(error_t error);
#line 117
static void BaseStationP__RadioControl__stopDone(error_t error);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



BaseStationP__RadioReceive__receive(
# 76 "../InterceptBase/BaseStationP.nc"
am_id_t arg_0xf73c4d70, 
# 60 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void BaseStationP__radioSendTask__runTask(void );
# 31 "/opt/tinyos-2.x/tos/interfaces/Intercept.nc"
static bool BaseStationP__SerialIntercept__default__forward(
# 90 "../InterceptBase/BaseStationP.nc"
am_id_t arg_0xf7399218, 
# 20 "/opt/tinyos-2.x/tos/interfaces/Intercept.nc"
message_t * msg, 

void * payload, 








uint8_t len);
#line 31
static bool BaseStationP__RadioIntercept__default__forward(
# 89 "../InterceptBase/BaseStationP.nc"
am_id_t arg_0xf739aab8, 
# 20 "/opt/tinyos-2.x/tos/interfaces/Intercept.nc"
message_t * msg, 

void * payload, 








uint8_t len);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void BaseStationP__ResetTimer__fired(void );
# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static void BaseStationP__RadioSend__sendDone(
# 75 "../InterceptBase/BaseStationP.nc"
am_id_t arg_0xf73c4310, 
# 92 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
#line 99
static void BaseStationP__UartSend__sendDone(
# 70 "../InterceptBase/BaseStationP.nc"
am_id_t arg_0xf73c7108, 
# 92 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



BaseStationP__RadioSnoop__receive(
# 77 "../InterceptBase/BaseStationP.nc"
am_id_t arg_0xf73be450, 
# 60 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void BaseStationP__UartTimer__fired(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
static void LedsP__Leds__led0Toggle(void );
#line 72
static void LedsP__Leds__led1Toggle(void );
#line 89
static void LedsP__Leds__led2Toggle(void );
# 46 "../InterceptBase/Reset.nc"
static void ResetC__Reset__reset(void );
# 52 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageConfig.nc"
static error_t RF230RadioP__ActiveMessageConfig__checkFrame(message_t *msg);
#line 35
static am_addr_t RF230RadioP__ActiveMessageConfig__source(message_t *msg);
#line 29
static am_addr_t RF230RadioP__ActiveMessageConfig__destination(message_t *msg);








static void RF230RadioP__ActiveMessageConfig__setSource(message_t *msg, am_addr_t addr);
#line 32
static void RF230RadioP__ActiveMessageConfig__setDestination(message_t *msg, am_addr_t addr);
#line 44
static void RF230RadioP__ActiveMessageConfig__setGroup(message_t *msg, am_group_t grp);
# 48 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
static void RF230RadioP__RadioAlarm__fired(void );
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverConfig.nc"
static uint8_t RF230RadioP__RF230DriverConfig__maxPayloadLength(void );
#line 29
static uint8_t RF230RadioP__RF230DriverConfig__headerLength(message_t *msg);
#line 41
static uint8_t RF230RadioP__RF230DriverConfig__metadataLength(message_t *msg);






static uint8_t RF230RadioP__RF230DriverConfig__headerPreloadLength(void );





static bool RF230RadioP__RF230DriverConfig__requiresRssiCca(message_t *msg);
# 75 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/SoftwareAckConfig.nc"
static void RF230RadioP__SoftwareAckConfig__reportChannelError(void );
#line 69
static void RF230RadioP__SoftwareAckConfig__createAckPacket(message_t *data, message_t *ack);
#line 44
static bool RF230RadioP__SoftwareAckConfig__requiresAckWait(message_t *msg);






static bool RF230RadioP__SoftwareAckConfig__isAckPacket(message_t *msg);






static bool RF230RadioP__SoftwareAckConfig__verifyAckPacket(message_t *data, message_t *ack);
#line 32
static uint16_t RF230RadioP__SoftwareAckConfig__getAckTimeout(void );





static void RF230RadioP__SoftwareAckConfig__setAckRequired(message_t *msg, bool ack);
#line 64
static bool RF230RadioP__SoftwareAckConfig__requiresAckReply(message_t *msg);
# 41 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/UniqueConfig.nc"
static void RF230RadioP__UniqueConfig__setSequenceNumber(message_t *msg, uint8_t number);





static void RF230RadioP__UniqueConfig__reportChannelError(void );
#line 31
static uint8_t RF230RadioP__UniqueConfig__getSequenceNumber(message_t *msg);




static am_addr_t RF230RadioP__UniqueConfig__getSender(message_t *msg);
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/RandomCollisionConfig.nc"
static uint16_t RF230RadioP__RandomCollisionConfig__getCongestionBackoff(message_t *msg);
#line 29
static uint16_t RF230RadioP__RandomCollisionConfig__getInitialBackoff(message_t *msg);










static uint16_t RF230RadioP__RandomCollisionConfig__getMinimumBackoff(void );





static uint16_t RF230RadioP__RandomCollisionConfig__getTransmitBarrier(message_t *msg);
# 48 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
static void RadioAlarmP__RadioAlarm__default__fired(
# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarmP.nc"
uint8_t arg_0xf72599e0);
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
static void RadioAlarmP__RadioAlarm__wait(
# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarmP.nc"
uint8_t arg_0xf72599e0, 
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
uint16_t timeout);




static void RadioAlarmP__RadioAlarm__cancel(
# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarmP.nc"
uint8_t arg_0xf72599e0);
# 33 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
static bool RadioAlarmP__RadioAlarm__isFree(
# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarmP.nc"
uint8_t arg_0xf72599e0);
# 53 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
static uint16_t RadioAlarmP__RadioAlarm__getNow(
# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarmP.nc"
uint8_t arg_0xf72599e0);
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void RadioAlarmP__Alarm__fired(void );
# 37 "/opt/tinyos-2.x/tos/chips/rf2xx/util/Tasklet.nc"
static void RadioAlarmP__Tasklet__run(void );










static void TaskletC__Tasklet__schedule(void );
#line 61
static void TaskletC__Tasklet__suspend(void );






static void TaskletC__Tasklet__resume(void );
# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
static void ActiveMessageLayerP__SubSend__sendDone(message_t *msg, error_t error);
# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareReceive.nc"
static message_t *ActiveMessageLayerP__SubReceive__receive(message_t *msg);
# 59 "/opt/tinyos-2.x/tos/interfaces/SendNotifier.nc"
static void ActiveMessageLayerP__SendNotifier__default__aboutToSend(
# 36 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayerP.nc"
am_id_t arg_0xf7232c30, 
# 59 "/opt/tinyos-2.x/tos/interfaces/SendNotifier.nc"
am_addr_t dest, 
#line 57
message_t * msg);
# 67 "/opt/tinyos-2.x/tos/interfaces/Packet.nc"
static uint8_t ActiveMessageLayerP__Packet__payloadLength(
#line 63
message_t * msg);
#line 95
static uint8_t ActiveMessageLayerP__Packet__maxPayloadLength(void );
#line 83
static void ActiveMessageLayerP__Packet__setPayloadLength(
#line 79
message_t * msg, 



uint8_t len);
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
static uint8_t ActiveMessageLayerP__RadioPacket__payloadLength(message_t *msg);









static uint8_t ActiveMessageLayerP__RadioPacket__maxPayloadLength(void );
#line 43
static void ActiveMessageLayerP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 32
static uint8_t ActiveMessageLayerP__RadioPacket__headerLength(message_t *msg);
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static error_t ActiveMessageLayerP__AMSend__send(
# 33 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayerP.nc"
am_id_t arg_0xf7234540, 
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 77 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
static am_addr_t ActiveMessageLayerP__AMPacket__source(
#line 73
message_t * amsg);
#line 57
static am_addr_t ActiveMessageLayerP__AMPacket__address(void );









static am_addr_t ActiveMessageLayerP__AMPacket__destination(
#line 63
message_t * amsg);
#line 110
static void ActiveMessageLayerP__AMPacket__setSource(
#line 106
message_t * amsg, 



am_addr_t addr);
#line 92
static void ActiveMessageLayerP__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 136
static am_id_t ActiveMessageLayerP__AMPacket__type(
#line 132
message_t * amsg);
#line 151
static void ActiveMessageLayerP__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
#line 125
static bool ActiveMessageLayerP__AMPacket__isForMe(
#line 122
message_t * amsg);
#line 176
static void ActiveMessageLayerP__AMPacket__setGroup(
#line 173
message_t * amsg, 


am_group_t grp);







static am_group_t ActiveMessageLayerP__AMPacket__localGroup(void );
# 44 "/opt/tinyos-2.x/tos/system/ActiveMessageAddressC.nc"
static am_addr_t ActiveMessageAddressC__amAddress(void );
# 50 "/opt/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );




static am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void );
# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
static void /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__SubSend__sendDone(message_t *msg, error_t error);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__granted(void );
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
static error_t /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__BareSend__send(message_t *msg);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__Init__init(void );
# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static error_t /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
#line 43
static bool /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );








static bool /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(
# 52 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71c77d0);
# 51 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__immediateRequested(
# 52 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71c77d0);
# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__unconfigure(
# 56 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71c5430);
# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(
# 56 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71c5430);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__release(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71d8d90);
# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__immediateRequest(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71d8d90);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__request(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71d8d90);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__default__granted(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71d8d90);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__runTask(void );
# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
static void Ieee154MessageLayerC__SubSend__sendDone(message_t *msg, error_t error);
# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareReceive.nc"
static message_t *Ieee154MessageLayerC__SubReceive__receive(message_t *msg);
# 67 "/opt/tinyos-2.x/tos/interfaces/Packet.nc"
static uint8_t Ieee154MessageLayerC__Packet__payloadLength(
#line 63
message_t * msg);
# 86 "/opt/tinyos-2.x/tos/interfaces/Ieee154Send.nc"
static void Ieee154MessageLayerC__Ieee154Send__default__sendDone(message_t *msg, error_t error);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



Ieee154MessageLayerC__Ieee154Receive__default__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
static void TinyosNetworkLayerC__SubSend__sendDone(message_t *msg, error_t error);
# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareReceive.nc"
static message_t *TinyosNetworkLayerC__SubReceive__receive(message_t *msg);
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
static uint8_t TinyosNetworkLayerC__Ieee154Packet__payloadLength(message_t *msg);
#line 32
static uint8_t TinyosNetworkLayerC__Ieee154Packet__headerLength(message_t *msg);
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
static error_t TinyosNetworkLayerC__TinyosSend__send(message_t *msg);
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
static uint8_t TinyosNetworkLayerC__TinyosPacket__payloadLength(message_t *msg);









static uint8_t TinyosNetworkLayerC__TinyosPacket__maxPayloadLength(void );
#line 43
static void TinyosNetworkLayerC__TinyosPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 32
static uint8_t TinyosNetworkLayerC__TinyosPacket__headerLength(message_t *msg);
# 26 "/opt/tinyos-2.x/tos/interfaces/Ieee154Packet.nc"
static ieee154_saddr_t Ieee154PacketLayerP__Ieee154Packet__address(void );

static ieee154_saddr_t Ieee154PacketLayerP__Ieee154Packet__destination(message_t *msg);







static bool Ieee154PacketLayerP__Ieee154Packet__isForMe(message_t *msg);
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
static uint8_t Ieee154PacketLayerP__RadioPacket__payloadLength(message_t *msg);









static uint8_t Ieee154PacketLayerP__RadioPacket__maxPayloadLength(void );
#line 43
static void Ieee154PacketLayerP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 32
static uint8_t Ieee154PacketLayerP__RadioPacket__headerLength(message_t *msg);
# 64 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayer.nc"
static bool Ieee154PacketLayerP__Ieee154PacketLayer__isAckFrame(message_t *msg);
#line 109
static uint8_t Ieee154PacketLayerP__Ieee154PacketLayer__getDSN(message_t *msg);
#line 58
static void Ieee154PacketLayerP__Ieee154PacketLayer__createDataFrame(message_t *msg);
#line 114
static void Ieee154PacketLayerP__Ieee154PacketLayer__setDSN(message_t *msg, uint8_t dsn);
#line 77
static void Ieee154PacketLayerP__Ieee154PacketLayer__createAckReply(message_t *data, message_t *ack);
#line 145
static void Ieee154PacketLayerP__Ieee154PacketLayer__setSrcAddr(message_t *msg, uint16_t addr);
#line 140
static uint16_t Ieee154PacketLayerP__Ieee154PacketLayer__getSrcAddr(message_t *msg);
#line 52
static bool Ieee154PacketLayerP__Ieee154PacketLayer__isDataFrame(message_t *msg);
#line 83
static bool Ieee154PacketLayerP__Ieee154PacketLayer__verifyAckReply(message_t *data, message_t *ack);




static bool Ieee154PacketLayerP__Ieee154PacketLayer__getAckRequired(message_t *msg);
#line 151
static bool Ieee154PacketLayerP__Ieee154PacketLayer__requiresAckWait(message_t *msg);
#line 135
static void Ieee154PacketLayerP__Ieee154PacketLayer__setDestAddr(message_t *msg, uint16_t addr);
#line 125
static void Ieee154PacketLayerP__Ieee154PacketLayer__setDestPan(message_t *msg, uint16_t pan);
#line 94
static void Ieee154PacketLayerP__Ieee154PacketLayer__setAckRequired(message_t *msg, bool ack);
#line 157
static bool Ieee154PacketLayerP__Ieee154PacketLayer__requiresAckReply(message_t *msg);
#line 130
static uint16_t Ieee154PacketLayerP__Ieee154PacketLayer__getDestAddr(message_t *msg);
# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
static void UniqueLayerP__SubSend__sendDone(message_t *msg, error_t error);
# 42 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioReceive.nc"
static message_t *UniqueLayerP__SubReceive__receive(message_t *msg);
#line 35
static bool UniqueLayerP__SubReceive__header(message_t *msg);
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
static error_t UniqueLayerP__Send__send(message_t *msg);
# 69 "/opt/tinyos-2.x/tos/chips/rf2xx/util/Neighborhood.nc"
static void UniqueLayerP__Neighborhood__evicted(uint8_t index);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t UniqueLayerP__Init__init(void );
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/NeighborhoodFlag.nc"
static bool NeighborhoodP__NeighborhoodFlag__get(
# 32 "/opt/tinyos-2.x/tos/chips/rf2xx/util/NeighborhoodP.nc"
uint8_t arg_0xf7107010, 
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/NeighborhoodFlag.nc"
uint8_t index);




static void NeighborhoodP__NeighborhoodFlag__set(
# 32 "/opt/tinyos-2.x/tos/chips/rf2xx/util/NeighborhoodP.nc"
uint8_t arg_0xf7107010, 
# 40 "/opt/tinyos-2.x/tos/chips/rf2xx/util/NeighborhoodFlag.nc"
uint8_t index);
# 60 "/opt/tinyos-2.x/tos/chips/rf2xx/util/Neighborhood.nc"
static uint8_t NeighborhoodP__Neighborhood__insertNode(am_addr_t id);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t NeighborhoodP__Init__init(void );
# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
static void PacketLinkLayerP__SubSend__sendDone(message_t *msg, error_t error);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void PacketLinkLayerP__send__runTask(void );
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void PacketLinkLayerP__DelayTimer__fired(void );
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
static error_t PacketLinkLayerP__Send__send(message_t *msg);
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
static uint8_t PacketLinkLayerP__RadioPacket__payloadLength(message_t *msg);









static uint8_t PacketLinkLayerP__RadioPacket__maxPayloadLength(void );
#line 43
static void PacketLinkLayerP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 32
static uint8_t PacketLinkLayerP__RadioPacket__headerLength(message_t *msg);
#line 54
static uint8_t PacketLinkLayerP__RadioPacket__metadataLength(message_t *msg);
# 65 "/opt/tinyos-2.x/tos/interfaces/PacketLink.nc"
static uint16_t PacketLinkLayerP__PacketLink__getRetryDelay(
#line 62
message_t * msg);
#line 46
static void PacketLinkLayerP__PacketLink__setRetries(
#line 42
message_t * msg, 



uint16_t maxRetries);
#line 59
static uint16_t PacketLinkLayerP__PacketLink__getRetries(
#line 56
message_t * msg);
# 98 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__size_type /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__getNow(void );
#line 92
static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__startAt(/*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__size_type t0, /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__size_type dt);
#line 105
static /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__size_type /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__getAlarm(void );
#line 62
static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__stop(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Init__init(void );
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Counter__size_type /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Counter__get(void );
# 49 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Compare__fired(void );
# 61 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Timer__overflow(void );
# 69 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm128TimerCtrl8.nc"
static uint8_t HplAtm1281Timer2AsyncP__TimerCtrl__getInterruptFlag(void );
#line 62
static void HplAtm1281Timer2AsyncP__TimerCtrl__setControlB(uint8_t control);
#line 61
static void HplAtm1281Timer2AsyncP__TimerCtrl__setControlA(uint8_t control);
# 54 "/opt/tinyos-2.x/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t HplAtm1281Timer2AsyncP__McuPowerOverride__lowestState(void );
# 57 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm128TimerAsync.nc"
static void HplAtm1281Timer2AsyncP__TimerAsync__setTimer2Asynchronous(void );
#line 75
static int HplAtm1281Timer2AsyncP__TimerAsync__compareABusy(void );
# 39 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
static HplAtm1281Timer2AsyncP__Compare__size_type HplAtm1281Timer2AsyncP__Compare__get(void );





static void HplAtm1281Timer2AsyncP__Compare__set(HplAtm1281Timer2AsyncP__Compare__size_type t);










static void HplAtm1281Timer2AsyncP__Compare__start(void );
# 52 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
static HplAtm1281Timer2AsyncP__Timer__timer_size HplAtm1281Timer2AsyncP__Timer__get(void );
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
uint8_t arg_0xf6ff0d88);
# 81 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(
# 37 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0xf6ff0d88);
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(
# 37 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0xf6ff0d88, 
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
uint32_t dt);




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(
# 37 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0xf6ff0d88);
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static error_t MessageBufferLayerP__SplitControl__start(void );
#line 109
static error_t MessageBufferLayerP__SplitControl__stop(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t MessageBufferLayerP__SoftwareInit__init(void );
# 58 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioState.nc"
static void MessageBufferLayerP__RadioState__done(void );
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
static error_t MessageBufferLayerP__Send__send(message_t *msg);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void MessageBufferLayerP__stateDoneTask__runTask(void );
# 42 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioReceive.nc"
static message_t *MessageBufferLayerP__RadioReceive__receive(message_t *msg);
#line 35
static bool MessageBufferLayerP__RadioReceive__header(message_t *msg);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void MessageBufferLayerP__sendTask__runTask(void );
#line 64
static void MessageBufferLayerP__deliverTask__runTask(void );
# 52 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioSend.nc"
static void MessageBufferLayerP__RadioSend__ready(void );
#line 45
static void MessageBufferLayerP__RadioSend__sendDone(error_t error);
# 37 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioChannel.nc"
static void MessageBufferLayerP__RadioChannel__default__setChannelDone(void );
# 37 "/opt/tinyos-2.x/tos/chips/rf2xx/util/Tasklet.nc"
static void MessageBufferLayerP__Tasklet__run(void );
# 52 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioSend.nc"
static void RandomCollisionLayerP__SubSend__ready(void );
#line 45
static void RandomCollisionLayerP__SubSend__sendDone(error_t error);
# 42 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioReceive.nc"
static message_t *RandomCollisionLayerP__SubReceive__receive(message_t *msg);
#line 35
static bool RandomCollisionLayerP__SubReceive__header(message_t *msg);
# 48 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
static void RandomCollisionLayerP__RadioAlarm__fired(void );
# 37 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioSend.nc"
static error_t RandomCollisionLayerP__RadioSend__send(message_t *msg);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void RandomCollisionLayerP__calcNextRandom__runTask(void );
# 41 "/opt/tinyos-2.x/tos/interfaces/Random.nc"
static uint16_t RandomMlcgC__Random__rand16(void );
#line 35
static uint32_t RandomMlcgC__Random__rand32(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t RandomMlcgC__Init__init(void );
# 52 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioSend.nc"
static void SoftwareAckLayerP__SubSend__ready(void );
#line 45
static void SoftwareAckLayerP__SubSend__sendDone(error_t error);
# 42 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioReceive.nc"
static message_t *SoftwareAckLayerP__SubReceive__receive(message_t *msg);
#line 35
static bool SoftwareAckLayerP__SubReceive__header(message_t *msg);
# 48 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
static void SoftwareAckLayerP__RadioAlarm__fired(void );
# 37 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioSend.nc"
static error_t SoftwareAckLayerP__RadioSend__send(message_t *msg);
# 48 "/opt/tinyos-2.x/tos/interfaces/PacketAcknowledgements.nc"
static error_t SoftwareAckLayerP__PacketAcknowledgements__requestAck(
#line 42
message_t * msg);
#line 74
static bool SoftwareAckLayerP__PacketAcknowledgements__wasAcked(
#line 69
message_t * msg);
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
static uint8_t MetadataFlagsLayerC__RadioPacket__payloadLength(message_t *msg);









static uint8_t MetadataFlagsLayerC__RadioPacket__maxPayloadLength(void );
#line 43
static void MetadataFlagsLayerC__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 32
static uint8_t MetadataFlagsLayerC__RadioPacket__headerLength(message_t *msg);
#line 54
static uint8_t MetadataFlagsLayerC__RadioPacket__metadataLength(message_t *msg);
# 44 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketFlag.nc"
static void MetadataFlagsLayerC__PacketFlag__clear(
# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MetadataFlagsLayerC.nc"
uint8_t arg_0xf6f23948, 
# 44 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketFlag.nc"
message_t *msg);
#line 29
static bool MetadataFlagsLayerC__PacketFlag__get(
# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MetadataFlagsLayerC.nc"
uint8_t arg_0xf6f23948, 
# 29 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketFlag.nc"
message_t *msg);




static void MetadataFlagsLayerC__PacketFlag__setValue(
# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MetadataFlagsLayerC.nc"
uint8_t arg_0xf6f23948, 
# 34 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketFlag.nc"
message_t *msg, bool value);




static void MetadataFlagsLayerC__PacketFlag__set(
# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MetadataFlagsLayerC.nc"
uint8_t arg_0xf6f23948, 
# 39 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketFlag.nc"
message_t *msg);
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
static uint8_t TimeStampingLayerP__RadioPacket__payloadLength(message_t *msg);









static uint8_t TimeStampingLayerP__RadioPacket__maxPayloadLength(void );
#line 43
static void TimeStampingLayerP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 32
static uint8_t TimeStampingLayerP__RadioPacket__headerLength(message_t *msg);
#line 54
static uint8_t TimeStampingLayerP__RadioPacket__metadataLength(message_t *msg);
# 59 "/opt/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
static void TimeStampingLayerP__PacketTimeStampRadio__clear(
#line 55
message_t * msg);
#line 67
static void TimeStampingLayerP__PacketTimeStampRadio__set(
#line 62
message_t * msg, 




TimeStampingLayerP__PacketTimeStampRadio__size_type value);
# 41 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioCCA.nc"
static void RF230DriverLayerP__RadioCCA__default__done(error_t error);
# 40 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketField.nc"
static void RF230DriverLayerP__PacketRSSI__clear(message_t *msg);
#line 35
static RF230DriverLayerP__PacketRSSI__value_type RF230DriverLayerP__PacketRSSI__get(message_t *msg);
#line 29
static bool RF230DriverLayerP__PacketRSSI__isSet(message_t *msg);
#line 46
static void RF230DriverLayerP__PacketRSSI__set(message_t *msg, RF230DriverLayerP__PacketRSSI__value_type value);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t RF230DriverLayerP__SoftwareInit__init(void );
# 48 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
static void RF230DriverLayerP__RadioAlarm__fired(void );
# 33 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioState.nc"
static error_t RF230DriverLayerP__RadioState__turnOff(void );
#line 45
static error_t RF230DriverLayerP__RadioState__turnOn(void );
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketField.nc"
static RF230DriverLayerP__PacketTransmitPower__value_type RF230DriverLayerP__PacketTransmitPower__get(message_t *msg);
#line 29
static bool RF230DriverLayerP__PacketTransmitPower__isSet(message_t *msg);
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
static uint8_t RF230DriverLayerP__RadioPacket__payloadLength(message_t *msg);









static uint8_t RF230DriverLayerP__RadioPacket__maxPayloadLength(void );
#line 43
static void RF230DriverLayerP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 32
static uint8_t RF230DriverLayerP__RadioPacket__headerLength(message_t *msg);
#line 54
static uint8_t RF230DriverLayerP__RadioPacket__metadataLength(message_t *msg);
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketField.nc"
static RF230DriverLayerP__PacketTimeSyncOffset__value_type RF230DriverLayerP__PacketTimeSyncOffset__get(message_t *msg);
#line 29
static bool RF230DriverLayerP__PacketTimeSyncOffset__isSet(message_t *msg);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t RF230DriverLayerP__PlatformInit__init(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void RF230DriverLayerP__SpiResource__granted(void );
# 46 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketField.nc"
static void RF230DriverLayerP__PacketLinkQuality__set(message_t *msg, RF230DriverLayerP__PacketLinkQuality__value_type value);
# 37 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioSend.nc"
static error_t RF230DriverLayerP__RadioSend__send(message_t *msg);
# 50 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
static void RF230DriverLayerP__IRQ__captured(uint16_t time);
# 37 "/opt/tinyos-2.x/tos/chips/rf2xx/util/Tasklet.nc"
static void RF230DriverLayerP__Tasklet__run(void );
# 51 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Capture.nc"
static void HplRF230P__Capture__captured(HplRF230P__Capture__size_type t);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t HplRF230P__PlatformInit__init(void );
# 55 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
static void HplRF230P__IRQ__disable(void );
#line 42
static error_t HplRF230P__IRQ__captureRisingEdge(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void Atm128SpiP__zeroTask__runTask(void );
# 71 "/opt/tinyos-2.x/tos/interfaces/SpiPacket.nc"
static void Atm128SpiP__SpiPacket__default__sendDone(
#line 64
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void Atm128SpiP__ResourceArbiter__granted(
# 85 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
uint8_t arg_0xf6e13068);
# 34 "/opt/tinyos-2.x/tos/interfaces/SpiByte.nc"
static uint8_t Atm128SpiP__SpiByte__write(uint8_t tx);
# 92 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128Spi.nc"
static void Atm128SpiP__Spi__dataReady(uint8_t data);
# 46 "/opt/tinyos-2.x/tos/interfaces/FastSpiByte.nc"
static void Atm128SpiP__FastSpiByte__splitWrite(uint8_t data);
#line 58
static uint8_t Atm128SpiP__FastSpiByte__splitReadWrite(uint8_t data);
#line 52
static uint8_t Atm128SpiP__FastSpiByte__splitRead(void );
#line 65
static uint8_t Atm128SpiP__FastSpiByte__write(uint8_t data);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t Atm128SpiP__Resource__release(
# 81 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
uint8_t arg_0xf6e15398);
# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t Atm128SpiP__Resource__immediateRequest(
# 81 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
uint8_t arg_0xf6e15398);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t Atm128SpiP__Resource__request(
# 81 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
uint8_t arg_0xf6e15398);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void Atm128SpiP__Resource__default__granted(
# 81 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
uint8_t arg_0xf6e15398);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static bool Atm128SpiP__Resource__isOwner(
# 81 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
uint8_t arg_0xf6e15398);
# 72 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128Spi.nc"
static void HplAtm128SpiP__SPI__sleep(void );
#line 66
static void HplAtm128SpiP__SPI__initMaster(void );
#line 105
static void HplAtm128SpiP__SPI__setMasterBit(bool isMaster);
#line 96
static void HplAtm128SpiP__SPI__enableInterrupt(bool enabled);
#line 80
static uint8_t HplAtm128SpiP__SPI__read(void );
#line 125
static void HplAtm128SpiP__SPI__setMasterDoubleSpeed(bool on);
#line 114
static void HplAtm128SpiP__SPI__setClock(uint8_t speed);
#line 108
static void HplAtm128SpiP__SPI__setClockPolarity(bool highWhenIdle);
#line 86
static void HplAtm128SpiP__SPI__write(uint8_t data);
#line 99
static void HplAtm128SpiP__SPI__enableSpi(bool busOn);
#line 111
static void HplAtm128SpiP__SPI__setClockPhase(bool sampleOnTrailing);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__Init__init(void );
# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static error_t /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id);
#line 43
static bool /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void );








static bool /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void );
# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceRequested__default__requested(
# 52 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71c77d0);
# 51 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceRequested__default__immediateRequested(
# 52 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71c77d0);
# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceConfigure__default__unconfigure(
# 56 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71c5430);
# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceConfigure__default__configure(
# 56 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71c5430);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Resource__release(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71d8d90);
# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Resource__immediateRequest(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71d8d90);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Resource__request(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71d8d90);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static bool /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Resource__isOwner(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71d8d90);
# 80 "/opt/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
static bool /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ArbiterInfo__inUse(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__grantedTask__runTask(void );
# 71 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm128TimerCtrl16.nc"
static uint8_t HplAtm1281Timer1P__TimerCtrl__getInterruptFlag(void );
#line 63
static void HplAtm1281Timer1P__TimerCtrl__setControlB(uint8_t control);
#line 60
static uint8_t HplAtm1281Timer1P__TimerCtrl__getControlB(void );
# 53 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
static void HplAtm1281Timer1P__CompareA__reset(void );
#line 45
static void HplAtm1281Timer1P__CompareA__set(HplAtm1281Timer1P__CompareA__size_type t);










static void HplAtm1281Timer1P__CompareA__start(void );


static void HplAtm1281Timer1P__CompareA__stop(void );
# 79 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Capture.nc"
static void HplAtm1281Timer1P__Capture__setEdge(bool up);
#line 38
static HplAtm1281Timer1P__Capture__size_type HplAtm1281Timer1P__Capture__get(void );
#line 55
static void HplAtm1281Timer1P__Capture__reset(void );


static void HplAtm1281Timer1P__Capture__start(void );


static void HplAtm1281Timer1P__Capture__stop(void );
# 49 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
static void HplAtm1281Timer1P__CompareB__default__fired(void );
#line 49
static void HplAtm1281Timer1P__CompareC__default__fired(void );
# 78 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
static bool HplAtm1281Timer1P__Timer__test(void );
#line 52
static HplAtm1281Timer1P__Timer__timer_size HplAtm1281Timer1P__Timer__get(void );
#line 95
static void HplAtm1281Timer1P__Timer__setScale(uint8_t scale);
#line 58
static void HplAtm1281Timer1P__Timer__set(HplAtm1281Timer1P__Timer__timer_size t);










static void HplAtm1281Timer1P__Timer__start(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*InitOneP.InitOne*/Atm128TimerInitC__0__Init__init(void );
# 61 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
static void /*InitOneP.InitOne*/Atm128TimerInitC__0__Timer__overflow(void );
# 98 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__size_type /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__getNow(void );
#line 92
static void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__startAt(/*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__size_type t0, /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__size_type dt);
#line 55
static void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__start(/*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__size_type dt);






static void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__stop(void );
# 49 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
static void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Compare__fired(void );
# 61 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
static void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Timer__overflow(void );
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*CounterOne16C.NCounter*/Atm128CounterC__0__Counter__size_type /*CounterOne16C.NCounter*/Atm128CounterC__0__Counter__get(void );






static bool /*CounterOne16C.NCounter*/Atm128CounterC__0__Counter__isOverflowPending(void );
# 61 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
static void /*CounterOne16C.NCounter*/Atm128CounterC__0__Timer__overflow(void );
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__CounterFrom__overflow(void );
#line 53
static /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__Counter__size_type /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__Counter__get(void );
# 50 "/opt/tinyos-2.x/tos/lib/timer/LocalTime.nc"
static uint32_t /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__get(void );
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void );
# 55 "/opt/tinyos-2.x/tos/lib/timer/BusyWait.nc"
static void BusyWaitMicroC__BusyWait__wait(BusyWaitMicroC__BusyWait__size_type dt);
# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(
#line 85
message_t * msg, 



error_t error);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(
# 36 "/opt/tinyos-2.x/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0xf6caf148, 
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 67 "/opt/tinyos-2.x/tos/interfaces/Packet.nc"
static uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__payloadLength(
#line 63
message_t * msg);
#line 95
static uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength(void );
# 67 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
static am_addr_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__destination(
#line 63
message_t * amsg);
#line 110
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setSource(
#line 106
message_t * amsg, 



am_addr_t addr);
#line 136
static am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(
#line 132
message_t * amsg);
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static error_t SerialP__SplitControl__start(void );
#line 109
static error_t SerialP__SplitControl__stop(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SerialP__stopDoneTask__runTask(void );
#line 64
static void SerialP__RunTx__runTask(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t SerialP__Init__init(void );
# 43 "/opt/tinyos-2.x/tos/lib/serial/SerialFlush.nc"
static void SerialP__SerialFlush__flushDone(void );
#line 38
static void SerialP__SerialFlush__default__flush(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SerialP__startDoneTask__runTask(void );
# 83 "/opt/tinyos-2.x/tos/lib/serial/SerialFrameComm.nc"
static void SerialP__SerialFrameComm__dataReceived(uint8_t data);





static void SerialP__SerialFrameComm__putDone(void );
#line 74
static void SerialP__SerialFrameComm__delimiterReceived(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SerialP__defaultSerialFlushTask__runTask(void );
# 60 "/opt/tinyos-2.x/tos/lib/serial/SendBytePacket.nc"
static error_t SerialP__SendBytePacket__completeSend(void );
#line 51
static error_t SerialP__SendBytePacket__startSend(uint8_t first_byte);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(
# 40 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0xf6bea560, 
# 56 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 89
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(
# 40 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0xf6bea560, 
# 85 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask(void );
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(
# 39 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0xf6bede50, 
# 60 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 31 "/opt/tinyos-2.x/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(
# 43 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0xf6be9088, 
# 31 "/opt/tinyos-2.x/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(
# 43 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0xf6be9088);
# 23 "/opt/tinyos-2.x/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(
# 43 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0xf6be9088, 
# 23 "/opt/tinyos-2.x/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t upperLen);
# 70 "/opt/tinyos-2.x/tos/lib/serial/SendBytePacket.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte(void );









static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error_t error);
# 51 "/opt/tinyos-2.x/tos/lib/serial/ReceiveBytePacket.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket(void );






static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(uint8_t data);










static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(error_t result);
# 79 "/opt/tinyos-2.x/tos/interfaces/UartStream.nc"
static void HdlcTranslateC__UartStream__receivedByte(uint8_t byte);
#line 99
static void HdlcTranslateC__UartStream__receiveDone(
#line 95
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void HdlcTranslateC__UartStream__sendDone(
#line 53
uint8_t * buf, 



uint16_t len, error_t error);
# 45 "/opt/tinyos-2.x/tos/lib/serial/SerialFrameComm.nc"
static error_t HdlcTranslateC__SerialFrameComm__putDelimiter(void );
#line 68
static void HdlcTranslateC__SerialFrameComm__resetReceive(void );
#line 54
static error_t HdlcTranslateC__SerialFrameComm__putData(uint8_t data);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__Init__init(void );
# 48 "/opt/tinyos-2.x/tos/interfaces/UartStream.nc"
static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__UartStream__send(
#line 44
uint8_t * buf, 



uint16_t len);
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*Atm128Uart0C.UartP*/Atm128UartP__0__Counter__overflow(void );
# 49 "/opt/tinyos-2.x/tos/chips/atm128/HplAtm128Uart.nc"
static void /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__rxDone(uint8_t data);
#line 47
static void /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__txDone(void );
# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__StdControl__start(void );









static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__StdControl__stop(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t HplAtm128UartP__Uart0Init__init(void );
# 42 "/opt/tinyos-2.x/tos/chips/atm128/HplAtm128Uart.nc"
static error_t HplAtm128UartP__HplUart0__enableRxIntr(void );
#line 40
static error_t HplAtm128UartP__HplUart0__enableTxIntr(void );
static error_t HplAtm128UartP__HplUart0__disableTxIntr(void );

static error_t HplAtm128UartP__HplUart0__disableRxIntr(void );


static void HplAtm128UartP__HplUart0__tx(uint8_t data);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t HplAtm128UartP__Uart1Init__init(void );
# 49 "/opt/tinyos-2.x/tos/chips/atm128/HplAtm128Uart.nc"
static void HplAtm128UartP__HplUart1__default__rxDone(uint8_t data);
#line 47
static void HplAtm128UartP__HplUart1__default__txDone(void );
# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t HplAtm128UartP__Uart0RxControl__start(void );









static error_t HplAtm128UartP__Uart0RxControl__stop(void );
#line 74
static error_t HplAtm128UartP__Uart0TxControl__start(void );









static error_t HplAtm128UartP__Uart0TxControl__stop(void );
# 63 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm128TimerCtrl16.nc"
static void HplAtm1281Timer3P__TimerCtrl__setControlB(uint8_t control);
#line 60
static uint8_t HplAtm1281Timer3P__TimerCtrl__getControlB(void );
# 49 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
static void HplAtm1281Timer3P__CompareA__default__fired(void );
# 51 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Capture.nc"
static void HplAtm1281Timer3P__Capture__default__captured(HplAtm1281Timer3P__Capture__size_type t);
# 49 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
static void HplAtm1281Timer3P__CompareB__default__fired(void );
#line 49
static void HplAtm1281Timer3P__CompareC__default__fired(void );
# 52 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
static HplAtm1281Timer3P__Timer__timer_size HplAtm1281Timer3P__Timer__get(void );
#line 95
static void HplAtm1281Timer3P__Timer__setScale(uint8_t scale);
#line 58
static void HplAtm1281Timer3P__Timer__set(HplAtm1281Timer3P__Timer__timer_size t);










static void HplAtm1281Timer3P__Timer__start(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*InitThreeP.InitThree*/Atm128TimerInitC__1__Init__init(void );
# 61 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
static void /*InitThreeP.InitThree*/Atm128TimerInitC__1__Timer__overflow(void );
#line 61
static void /*CounterThree16C.NCounter*/Atm128CounterC__1__Timer__overflow(void );
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*CounterMicro32C.Transform32*/TransformCounterC__1__CounterFrom__overflow(void );
# 31 "/opt/tinyos-2.x/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t SerialPacketInfoActiveMessageP__Info__upperLength(message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t SerialPacketInfoActiveMessageP__Info__offset(void );







static uint8_t SerialPacketInfoActiveMessageP__Info__dataLinkLength(message_t *msg, uint8_t upperLen);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void RssiBaseC__SendTimer__fired(void );
# 49 "/opt/tinyos-2.x/tos/interfaces/Boot.nc"
static void RssiBaseC__Boot__booted(void );
# 31 "/opt/tinyos-2.x/tos/interfaces/Intercept.nc"
static bool RssiBaseC__RssiMsgIntercept__forward(
#line 20
message_t * msg, 

void * payload, 








uint8_t len);
#line 31
static bool RssiBaseC__SimpleRssiMsgIntercept__forward(
#line 20
message_t * msg, 

void * payload, 








uint8_t len);
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static void RssiBaseC__RadioControl__startDone(error_t error);
#line 117
static void RssiBaseC__RadioControl__stopDone(error_t error);
# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static void RssiBaseC__PingMsgSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static void /*RssiBaseAppC.PingMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(
# 40 "/opt/tinyos-2.x/tos/system/AMQueueImplP.nc"
am_id_t arg_0xf6a50e40, 
# 92 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(
# 38 "/opt/tinyos-2.x/tos/system/AMQueueImplP.nc"
uint8_t arg_0xf6a50458, 
# 85 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void );
#line 64
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t PlatformP__MoteInit__init(void );
#line 51
static error_t PlatformP__MeasureClock__init(void );
# 42 "/opt/tinyos-2.x/tos/platforms/mica/PlatformP.nc"
static inline void PlatformP__power_init(void );






static inline error_t PlatformP__Init__init(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t MotePlatformP__SubInit__init(void );
# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void MotePlatformP__SerialIdPin__makeInput(void );
#line 30
static void MotePlatformP__SerialIdPin__clr(void );
# 26 "/opt/tinyos-2.x/tos/platforms/micaz/MotePlatformP.nc"
static inline error_t MotePlatformP__PlatformInit__init(void );
# 46 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortA.Bit0*/HplAtm128GeneralIOPinP__0__IO__set(void );

static void /*HplAtm128GeneralIOC.PortA.Bit0*/HplAtm128GeneralIOPinP__0__IO__toggle(void );



static __inline void /*HplAtm128GeneralIOC.PortA.Bit0*/HplAtm128GeneralIOPinP__0__IO__makeOutput(void );
#line 46
static __inline void /*HplAtm128GeneralIOC.PortA.Bit1*/HplAtm128GeneralIOPinP__1__IO__set(void );

static void /*HplAtm128GeneralIOC.PortA.Bit1*/HplAtm128GeneralIOPinP__1__IO__toggle(void );



static __inline void /*HplAtm128GeneralIOC.PortA.Bit1*/HplAtm128GeneralIOPinP__1__IO__makeOutput(void );
#line 46
static __inline void /*HplAtm128GeneralIOC.PortA.Bit2*/HplAtm128GeneralIOPinP__2__IO__set(void );

static void /*HplAtm128GeneralIOC.PortA.Bit2*/HplAtm128GeneralIOPinP__2__IO__toggle(void );



static __inline void /*HplAtm128GeneralIOC.PortA.Bit2*/HplAtm128GeneralIOPinP__2__IO__makeOutput(void );
#line 47
static __inline void /*HplAtm128GeneralIOC.PortA.Bit4*/HplAtm128GeneralIOPinP__4__IO__clr(void );


static __inline void /*HplAtm128GeneralIOC.PortA.Bit4*/HplAtm128GeneralIOPinP__4__IO__makeInput(void );
#line 46
static __inline void /*HplAtm128GeneralIOC.PortA.Bit6*/HplAtm128GeneralIOPinP__6__IO__set(void );
static __inline void /*HplAtm128GeneralIOC.PortA.Bit6*/HplAtm128GeneralIOPinP__6__IO__clr(void );




static __inline void /*HplAtm128GeneralIOC.PortA.Bit6*/HplAtm128GeneralIOPinP__6__IO__makeOutput(void );
#line 46
static __inline void /*HplAtm128GeneralIOC.PortB.Bit0*/HplAtm128GeneralIOPinP__8__IO__set(void );
static __inline void /*HplAtm128GeneralIOC.PortB.Bit0*/HplAtm128GeneralIOPinP__8__IO__clr(void );




static __inline void /*HplAtm128GeneralIOC.PortB.Bit0*/HplAtm128GeneralIOPinP__8__IO__makeOutput(void );
#line 52
static __inline void /*HplAtm128GeneralIOC.PortB.Bit1*/HplAtm128GeneralIOPinP__9__IO__makeOutput(void );
#line 52
static __inline void /*HplAtm128GeneralIOC.PortB.Bit2*/HplAtm128GeneralIOPinP__10__IO__makeOutput(void );
#line 50
static __inline void /*HplAtm128GeneralIOC.PortB.Bit3*/HplAtm128GeneralIOPinP__11__IO__makeInput(void );
#line 46
static __inline void /*HplAtm128GeneralIOC.PortB.Bit7*/HplAtm128GeneralIOPinP__15__IO__set(void );
static __inline void /*HplAtm128GeneralIOC.PortB.Bit7*/HplAtm128GeneralIOPinP__15__IO__clr(void );




static __inline void /*HplAtm128GeneralIOC.PortB.Bit7*/HplAtm128GeneralIOPinP__15__IO__makeOutput(void );
#line 47
static __inline void /*HplAtm128GeneralIOC.PortD.Bit4*/HplAtm128GeneralIOPinP__28__IO__clr(void );


static __inline void /*HplAtm128GeneralIOC.PortD.Bit4*/HplAtm128GeneralIOPinP__28__IO__makeInput(void );
#line 47
static __inline void /*HplAtm128GeneralIOC.PortD.Bit6*/HplAtm128GeneralIOPinP__30__IO__clr(void );


static __inline void /*HplAtm128GeneralIOC.PortD.Bit6*/HplAtm128GeneralIOPinP__30__IO__makeInput(void );
# 56 "/opt/tinyos-2.x/tos/platforms/iris/MeasureClockC.nc"
enum MeasureClockC____nesc_unnamed4341 {


  MeasureClockC__MAGIC = 488 / (16 / PLATFORM_MHZ)
};

uint16_t MeasureClockC__cycles;

static inline error_t MeasureClockC__Init__init(void );
#line 143
static inline uint16_t MeasureClockC__Atm128Calibrate__baudrateRegister(uint32_t baudrate);
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
uint8_t arg_0xf7540b38);
# 59 "/opt/tinyos-2.x/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
# 50 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4342 {

  SchedulerBasicP__NUM_TASKS = 20U, 
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
# 54 "/opt/tinyos-2.x/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void );
# 80 "/opt/tinyos-2.x/tos/chips/atm1281/McuSleepC.nc"
const_uint8_t McuSleepC__atm128PowerBits[ATM128_POWER_DOWN + 1] = { 
0, 
1 << 1, ((
1 << 3) | (1 << 2)) | (1 << 1), (
1 << 2) | (1 << 1), (
1 << 3) | (1 << 2), 
1 << 2 };

static inline mcu_power_t McuSleepC__getPowerState(void );
#line 128
static inline void McuSleepC__McuSleep__sleep(void );
#line 142
static inline void McuSleepC__McuPowerState__update(void );
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void BaseStationP__RadioTimer__startOneShot(uint32_t dt);
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static error_t BaseStationP__SerialControl__start(void );
#line 109
static error_t BaseStationP__SerialControl__stop(void );
# 77 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
static am_addr_t BaseStationP__RadioAMPacket__source(
#line 73
message_t * amsg);
#line 67
static am_addr_t BaseStationP__RadioAMPacket__destination(
#line 63
message_t * amsg);
#line 136
static am_id_t BaseStationP__RadioAMPacket__type(
#line 132
message_t * amsg);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t BaseStationP__uartSendTask__postTask(void );
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static error_t BaseStationP__RadioControl__start(void );
#line 109
static error_t BaseStationP__RadioControl__stop(void );
# 67 "/opt/tinyos-2.x/tos/interfaces/Packet.nc"
static uint8_t BaseStationP__RadioPacket__payloadLength(
#line 63
message_t * msg);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t BaseStationP__radioSendTask__postTask(void );
# 31 "/opt/tinyos-2.x/tos/interfaces/Intercept.nc"
static bool BaseStationP__SerialIntercept__forward(
# 90 "../InterceptBase/BaseStationP.nc"
am_id_t arg_0xf7399218, 
# 20 "/opt/tinyos-2.x/tos/interfaces/Intercept.nc"
message_t * msg, 

void * payload, 








uint8_t len);
#line 31
static bool BaseStationP__RadioIntercept__forward(
# 89 "../InterceptBase/BaseStationP.nc"
am_id_t arg_0xf739aab8, 
# 20 "/opt/tinyos-2.x/tos/interfaces/Intercept.nc"
message_t * msg, 

void * payload, 








uint8_t len);
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void BaseStationP__ResetTimer__startOneShot(uint32_t dt);




static void BaseStationP__ResetTimer__stop(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
static void BaseStationP__Leds__led0Toggle(void );
#line 72
static void BaseStationP__Leds__led1Toggle(void );
#line 89
static void BaseStationP__Leds__led2Toggle(void );
# 67 "/opt/tinyos-2.x/tos/interfaces/Packet.nc"
static uint8_t BaseStationP__UartPacket__payloadLength(
#line 63
message_t * msg);
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static error_t BaseStationP__RadioSend__send(
# 75 "../InterceptBase/BaseStationP.nc"
am_id_t arg_0xf73c4310, 
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 46 "../InterceptBase/Reset.nc"
static void BaseStationP__Reset__reset(void );
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static error_t BaseStationP__UartSend__send(
# 70 "../InterceptBase/BaseStationP.nc"
am_id_t arg_0xf73c7108, 
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 67 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
static am_addr_t BaseStationP__UartAMPacket__destination(
#line 63
message_t * amsg);
#line 110
static void BaseStationP__UartAMPacket__setSource(
#line 106
message_t * amsg, 



am_addr_t addr);
#line 136
static am_id_t BaseStationP__UartAMPacket__type(
#line 132
message_t * amsg);
# 81 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static bool BaseStationP__UartTimer__isRunning(void );
#line 62
static void BaseStationP__UartTimer__startOneShot(uint32_t dt);




static void BaseStationP__UartTimer__stop(void );
# 126 "../InterceptBase/BaseStationP.nc"
enum BaseStationP____nesc_unnamed4343 {
#line 126
  BaseStationP__uartSendTask = 0U
};
#line 126
typedef int BaseStationP____nesc_sillytask_uartSendTask[BaseStationP__uartSendTask];
enum BaseStationP____nesc_unnamed4344 {
#line 127
  BaseStationP__radioSendTask = 1U
};
#line 127
typedef int BaseStationP____nesc_sillytask_radioSendTask[BaseStationP__radioSendTask];
#line 95
enum BaseStationP____nesc_unnamed4345 {
  BaseStationP__UART_QUEUE_LEN = 32, 
  BaseStationP__RADIO_QUEUE_LEN = 32, 
  BaseStationP__TIME_TO_RESET = 10000, 
  BaseStationP__UART_TIME = 10, 
  BaseStationP__RADIO_TIME = 5, 
  BaseStationP__RESET_TIME = 300, 

  BaseStationP__UART_RESET_THRESHOLD = 20
};


message_t BaseStationP__uartQueueBufs[BaseStationP__UART_QUEUE_LEN];
message_t *BaseStationP__uartQueue[BaseStationP__UART_QUEUE_LEN];
uint8_t BaseStationP__uartIn;
#line 109
uint8_t BaseStationP__uartOut;
bool BaseStationP__uartBusy;
#line 110
bool BaseStationP__uartFull;


message_t BaseStationP__radioQueueBufs[BaseStationP__RADIO_QUEUE_LEN];
message_t *BaseStationP__radioQueue[BaseStationP__RADIO_QUEUE_LEN];
uint8_t BaseStationP__radioIn;
#line 115
uint8_t BaseStationP__radioOut;
bool BaseStationP__radioBusy;
#line 116
bool BaseStationP__radioFull;


uint8_t BaseStationP__tmpLen;

uint8_t BaseStationP__uartFailCounter = 0;
bool BaseStationP__inReset = FALSE;
uint8_t BaseStationP__resetPhase = 0;




static void BaseStationP__timedUartSendTask(void );
static message_t *BaseStationP__receive(message_t *msg, void *payload, uint8_t len, am_id_t id);

static inline void BaseStationP__sucBlink(void );






static inline void BaseStationP__sucRadioBlink(void );







static inline void BaseStationP__dropBlink(void );




static inline void BaseStationP__failBlink(void );



static inline void BaseStationP__UartTimer__fired(void );
#line 173
static inline void BaseStationP__RadioTimer__fired(void );



static inline void BaseStationP__ResetTimer__fired(void );
#line 328
static inline void BaseStationP__Boot__booted(void );
#line 364
static inline void BaseStationP__RadioControl__startDone(error_t error);
#line 392
static inline void BaseStationP__SerialControl__startDone(error_t error);
#line 418
static inline void BaseStationP__SerialControl__stopDone(error_t error);
#line 434
static inline void BaseStationP__RadioControl__stopDone(error_t error);
#line 451
static inline message_t *BaseStationP__RadioSnoop__receive(am_id_t id, message_t *msg, void *payload, uint8_t len);



static inline message_t *BaseStationP__RadioReceive__receive(am_id_t id, message_t *msg, void *payload, uint8_t len);





static message_t *BaseStationP__receive(message_t *msg, void *payload, uint8_t len, am_id_t id);
#line 518
static void BaseStationP__timedUartSendTask(void );
#line 531
static inline void BaseStationP__uartSendTask__runTask(void );
#line 594
static inline void BaseStationP__UartSend__sendDone(am_id_t id, message_t *msg, error_t error);
#line 622
static inline message_t *BaseStationP__UartReceive__receive(am_id_t id, message_t *msg, void *payload, uint8_t len);
#line 662
static inline void BaseStationP__radioSendTask__runTask(void );
#line 692
static inline void BaseStationP__RadioSend__sendDone(am_id_t id, message_t *msg, error_t error);
#line 725
static inline bool BaseStationP__RadioIntercept__default__forward(am_id_t amid, message_t *msg, void *payload, uint8_t len);




static inline bool BaseStationP__SerialIntercept__default__forward(am_id_t amid, message_t *msg, void *payload, uint8_t len);
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
#line 73
static inline void LedsP__Leds__led0Toggle(void );
#line 88
static inline void LedsP__Leds__led1Toggle(void );
#line 103
static inline void LedsP__Leds__led2Toggle(void );
# 52 "../InterceptBase/ResetC.nc"
static inline void ResetC__Reset__reset(void );
# 53 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
static uint16_t RF230RadioP__RadioAlarm__getNow(void );
# 64 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayer.nc"
static bool RF230RadioP__Ieee154PacketLayer__isAckFrame(message_t *msg);
#line 109
static uint8_t RF230RadioP__Ieee154PacketLayer__getDSN(message_t *msg);
#line 58
static void RF230RadioP__Ieee154PacketLayer__createDataFrame(message_t *msg);
#line 114
static void RF230RadioP__Ieee154PacketLayer__setDSN(message_t *msg, uint8_t dsn);
#line 77
static void RF230RadioP__Ieee154PacketLayer__createAckReply(message_t *data, message_t *ack);
#line 145
static void RF230RadioP__Ieee154PacketLayer__setSrcAddr(message_t *msg, uint16_t addr);
#line 140
static uint16_t RF230RadioP__Ieee154PacketLayer__getSrcAddr(message_t *msg);
#line 52
static bool RF230RadioP__Ieee154PacketLayer__isDataFrame(message_t *msg);
#line 83
static bool RF230RadioP__Ieee154PacketLayer__verifyAckReply(message_t *data, message_t *ack);
#line 151
static bool RF230RadioP__Ieee154PacketLayer__requiresAckWait(message_t *msg);
#line 135
static void RF230RadioP__Ieee154PacketLayer__setDestAddr(message_t *msg, uint16_t addr);
#line 125
static void RF230RadioP__Ieee154PacketLayer__setDestPan(message_t *msg, uint16_t pan);
#line 94
static void RF230RadioP__Ieee154PacketLayer__setAckRequired(message_t *msg, bool ack);
#line 157
static bool RF230RadioP__Ieee154PacketLayer__requiresAckReply(message_t *msg);
#line 130
static uint16_t RF230RadioP__Ieee154PacketLayer__getDestAddr(message_t *msg);
# 62 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline uint8_t RF230RadioP__RF230DriverConfig__headerLength(message_t *msg);




static inline uint8_t RF230RadioP__RF230DriverConfig__maxPayloadLength(void );




static inline uint8_t RF230RadioP__RF230DriverConfig__metadataLength(message_t *msg);




static inline uint8_t RF230RadioP__RF230DriverConfig__headerPreloadLength(void );





static inline bool RF230RadioP__RF230DriverConfig__requiresRssiCca(message_t *msg);






static inline bool RF230RadioP__SoftwareAckConfig__requiresAckWait(message_t *msg);




static inline bool RF230RadioP__SoftwareAckConfig__isAckPacket(message_t *msg);




static inline bool RF230RadioP__SoftwareAckConfig__verifyAckPacket(message_t *data, message_t *ack);




static inline void RF230RadioP__SoftwareAckConfig__setAckRequired(message_t *msg, bool ack);




static inline bool RF230RadioP__SoftwareAckConfig__requiresAckReply(message_t *msg);




static inline void RF230RadioP__SoftwareAckConfig__createAckPacket(message_t *data, message_t *ack);








static inline uint16_t RF230RadioP__SoftwareAckConfig__getAckTimeout(void );




static inline void RF230RadioP__SoftwareAckConfig__reportChannelError(void );








static inline uint8_t RF230RadioP__UniqueConfig__getSequenceNumber(message_t *msg);




static inline void RF230RadioP__UniqueConfig__setSequenceNumber(message_t *msg, uint8_t dsn);




static inline am_addr_t RF230RadioP__UniqueConfig__getSender(message_t *msg);




static inline void RF230RadioP__UniqueConfig__reportChannelError(void );








static inline am_addr_t RF230RadioP__ActiveMessageConfig__destination(message_t *msg);




static inline void RF230RadioP__ActiveMessageConfig__setDestination(message_t *msg, am_addr_t addr);




static inline am_addr_t RF230RadioP__ActiveMessageConfig__source(message_t *msg);




static inline void RF230RadioP__ActiveMessageConfig__setSource(message_t *msg, am_addr_t addr);









static inline void RF230RadioP__ActiveMessageConfig__setGroup(message_t *msg, am_group_t grp);




static inline error_t RF230RadioP__ActiveMessageConfig__checkFrame(message_t *msg);
#line 209
enum RF230RadioP____nesc_unnamed4346 {

  RF230RadioP__TRAFFIC_UPDATE_PERIOD = 100, 
  RF230RadioP__TRAFFIC_MAX_BYTES = (uint16_t )(RF230RadioP__TRAFFIC_UPDATE_PERIOD * 1000UL / 32)
};
#line 249
static inline uint16_t RF230RadioP__RandomCollisionConfig__getMinimumBackoff(void );




static inline uint16_t RF230RadioP__RandomCollisionConfig__getInitialBackoff(message_t *msg);




static inline uint16_t RF230RadioP__RandomCollisionConfig__getCongestionBackoff(message_t *msg);




static inline uint16_t RF230RadioP__RandomCollisionConfig__getTransmitBarrier(message_t *msg);
#line 280
static inline void RF230RadioP__RadioAlarm__fired(void );
# 48 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
static void RadioAlarmP__RadioAlarm__fired(
# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarmP.nc"
uint8_t arg_0xf72599e0);
# 98 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static RadioAlarmP__Alarm__size_type RadioAlarmP__Alarm__getNow(void );
#line 55
static void RadioAlarmP__Alarm__start(RadioAlarmP__Alarm__size_type dt);






static void RadioAlarmP__Alarm__stop(void );
# 48 "/opt/tinyos-2.x/tos/chips/rf2xx/util/Tasklet.nc"
static void RadioAlarmP__Tasklet__schedule(void );
# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarmP.nc"
uint8_t RadioAlarmP__state;
enum RadioAlarmP____nesc_unnamed4347 {

  RadioAlarmP__STATE_READY = 0, 
  RadioAlarmP__STATE_WAIT = 1, 
  RadioAlarmP__STATE_FIRED = 2
};

uint8_t RadioAlarmP__alarm;

static inline void RadioAlarmP__Alarm__fired(void );










static __inline uint16_t RadioAlarmP__RadioAlarm__getNow(uint8_t id);




static inline void RadioAlarmP__Tasklet__run(void );








static inline void RadioAlarmP__RadioAlarm__default__fired(uint8_t id);



static __inline bool RadioAlarmP__RadioAlarm__isFree(uint8_t id);




static inline void RadioAlarmP__RadioAlarm__wait(uint8_t id, uint16_t timeout);








static inline void RadioAlarmP__RadioAlarm__cancel(uint8_t id);
# 37 "/opt/tinyos-2.x/tos/chips/rf2xx/util/Tasklet.nc"
static void TaskletC__Tasklet__run(void );
# 61 "/opt/tinyos-2.x/tos/chips/rf2xx/util/TaskletC.nc"
uint8_t TaskletC__state;

static void TaskletC__doit(void );
#line 83
static __inline void TaskletC__Tasklet__suspend(void );




static void TaskletC__Tasklet__resume(void );
#line 101
static void TaskletC__Tasklet__schedule(void );
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
static error_t ActiveMessageLayerP__SubSend__send(message_t *msg);
# 59 "/opt/tinyos-2.x/tos/interfaces/SendNotifier.nc"
static void ActiveMessageLayerP__SendNotifier__aboutToSend(
# 36 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayerP.nc"
am_id_t arg_0xf7232c30, 
# 59 "/opt/tinyos-2.x/tos/interfaces/SendNotifier.nc"
am_addr_t dest, 
#line 57
message_t * msg);
# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static void ActiveMessageLayerP__AMSend__sendDone(
# 33 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayerP.nc"
am_id_t arg_0xf7234540, 
# 92 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



ActiveMessageLayerP__Snoop__receive(
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayerP.nc"
am_id_t arg_0xf72325b8, 
# 60 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
static uint8_t ActiveMessageLayerP__SubPacket__payloadLength(message_t *msg);









static uint8_t ActiveMessageLayerP__SubPacket__maxPayloadLength(void );
#line 43
static void ActiveMessageLayerP__SubPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 32
static uint8_t ActiveMessageLayerP__SubPacket__headerLength(message_t *msg);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



ActiveMessageLayerP__Receive__receive(
# 34 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayerP.nc"
am_id_t arg_0xf7234f00, 
# 60 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 52 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageConfig.nc"
static error_t ActiveMessageLayerP__Config__checkFrame(message_t *msg);
#line 35
static am_addr_t ActiveMessageLayerP__Config__source(message_t *msg);
#line 29
static am_addr_t ActiveMessageLayerP__Config__destination(message_t *msg);








static void ActiveMessageLayerP__Config__setSource(message_t *msg, am_addr_t addr);
#line 32
static void ActiveMessageLayerP__Config__setDestination(message_t *msg, am_addr_t addr);
#line 44
static void ActiveMessageLayerP__Config__setGroup(message_t *msg, am_group_t grp);
# 50 "/opt/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t ActiveMessageLayerP__ActiveMessageAddress__amAddress(void );




static am_group_t ActiveMessageLayerP__ActiveMessageAddress__amGroup(void );
# 51 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayerP.nc"
static inline activemessage_header_t *ActiveMessageLayerP__getHeader(message_t *msg);




static inline void *ActiveMessageLayerP__getPayload(message_t *msg);






static error_t ActiveMessageLayerP__AMSend__send(am_id_t id, am_addr_t addr, message_t *msg, uint8_t len);
#line 82
static __inline void ActiveMessageLayerP__SubSend__sendDone(message_t *msg, error_t error);
#line 106
static inline void ActiveMessageLayerP__SendNotifier__default__aboutToSend(am_id_t id, am_addr_t addr, message_t *msg);





static inline message_t *ActiveMessageLayerP__SubReceive__receive(message_t *msg);
#line 137
static __inline am_addr_t ActiveMessageLayerP__AMPacket__address(void );




static __inline am_group_t ActiveMessageLayerP__AMPacket__localGroup(void );




static __inline bool ActiveMessageLayerP__AMPacket__isForMe(message_t *msg);





static __inline am_addr_t ActiveMessageLayerP__AMPacket__destination(message_t *msg);




static __inline void ActiveMessageLayerP__AMPacket__setDestination(message_t *msg, am_addr_t addr);




static __inline am_addr_t ActiveMessageLayerP__AMPacket__source(message_t *msg);




static __inline void ActiveMessageLayerP__AMPacket__setSource(message_t *msg, am_addr_t addr);




static __inline am_id_t ActiveMessageLayerP__AMPacket__type(message_t *msg);




static __inline void ActiveMessageLayerP__AMPacket__setType(message_t *msg, am_id_t type);









static __inline void ActiveMessageLayerP__AMPacket__setGroup(message_t *msg, am_group_t grp);










static inline uint8_t ActiveMessageLayerP__RadioPacket__headerLength(message_t *msg);




static inline uint8_t ActiveMessageLayerP__RadioPacket__payloadLength(message_t *msg);




static inline void ActiveMessageLayerP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);




static inline uint8_t ActiveMessageLayerP__RadioPacket__maxPayloadLength(void );
#line 236
static inline uint8_t ActiveMessageLayerP__Packet__payloadLength(message_t *msg);




static inline void ActiveMessageLayerP__Packet__setPayloadLength(message_t *msg, uint8_t len);




static inline uint8_t ActiveMessageLayerP__Packet__maxPayloadLength(void );
# 51 "/opt/tinyos-2.x/tos/system/ActiveMessageAddressC.nc"
am_addr_t ActiveMessageAddressC__addr = TOS_AM_ADDRESS;


am_group_t ActiveMessageAddressC__group = TOS_AM_GROUP;






static inline am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );
#line 82
static inline am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void );
#line 95
static am_addr_t ActiveMessageAddressC__amAddress(void );
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
static error_t /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__SubSend__send(message_t *msg);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__release(void );
#line 87
static error_t /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__immediateRequest(void );
#line 78
static error_t /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__request(void );
# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
static void /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__BareSend__sendDone(message_t *msg, error_t error);
# 40 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/AutoResourceAcquireLayerC.nc"
message_t */*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__pending;

static inline error_t /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__BareSend__send(message_t *msg);
#line 57
static inline void /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__granted(void );









static inline void /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__SubSend__sendDone(message_t *msg, error_t result);
# 39 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
enum /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0____nesc_unnamed4348 {
#line 39
  FcfsResourceQueueC__0__NO_ENTRY = 0xFF
};
uint8_t /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__resQ[1U];
uint8_t /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qHead = /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
uint8_t /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qTail = /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

static inline error_t /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__Init__init(void );




static inline bool /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );



static inline bool /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
#line 72
static inline error_t /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__requested(
# 52 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71c77d0);
# 51 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__immediateRequested(
# 52 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71c77d0);
# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__unconfigure(
# 56 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71c5430);
# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__configure(
# 56 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71c5430);
# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static error_t /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Queue__enqueue(resource_client_id_t id);
#line 43
static bool /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Queue__isEmpty(void );
#line 60
static resource_client_id_t /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Queue__dequeue(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__granted(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71d8d90);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__postTask(void );
# 69 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
enum /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0____nesc_unnamed4349 {
#line 69
  SimpleArbiterP__0__grantedTask = 2U
};
#line 69
typedef int /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0____nesc_sillytask_grantedTask[/*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask];
#line 62
enum /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0____nesc_unnamed4350 {
#line 62
  SimpleArbiterP__0__RES_IDLE = 0, SimpleArbiterP__0__RES_GRANTING = 1, SimpleArbiterP__0__RES_BUSY = 2
};
#line 63
enum /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0____nesc_unnamed4351 {
#line 63
  SimpleArbiterP__0__NO_RES = 0xFF
};
uint8_t /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__state = /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_IDLE;
uint8_t /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId = /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__NO_RES;
uint8_t /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__reqResId;



static inline error_t /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__request(uint8_t id);
#line 84
static inline error_t /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__immediateRequest(uint8_t id);
#line 97
static error_t /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__release(uint8_t id);
#line 155
static inline void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__runTask(void );









static inline void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__default__granted(uint8_t id);

static inline void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(uint8_t id);

static inline void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id);

static inline void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(uint8_t id);

static inline void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id);
# 36 "/opt/tinyos-2.x/tos/interfaces/Ieee154Packet.nc"
static bool Ieee154MessageLayerC__Ieee154Packet__isForMe(message_t *msg);
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
static uint8_t Ieee154MessageLayerC__RadioPacket__payloadLength(message_t *msg);
#line 32
static uint8_t Ieee154MessageLayerC__RadioPacket__headerLength(message_t *msg);
# 86 "/opt/tinyos-2.x/tos/interfaces/Ieee154Send.nc"
static void Ieee154MessageLayerC__Ieee154Send__sendDone(message_t *msg, error_t error);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



Ieee154MessageLayerC__Ieee154Receive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 46 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154MessageLayerC.nc"
static inline void *Ieee154MessageLayerC__getPayload(message_t *msg);
#line 58
static inline uint8_t Ieee154MessageLayerC__Packet__payloadLength(message_t *msg);
#line 117
static inline void Ieee154MessageLayerC__SubSend__sendDone(message_t *msg, error_t error);




static inline void Ieee154MessageLayerC__Ieee154Send__default__sendDone(message_t *msg, error_t error);









static inline message_t *Ieee154MessageLayerC__SubReceive__receive(message_t *msg);








static inline message_t *Ieee154MessageLayerC__Ieee154Receive__default__receive(message_t *msg, void *payload, uint8_t len);
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
static error_t TinyosNetworkLayerC__SubSend__send(message_t *msg);
# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareReceive.nc"
static message_t *TinyosNetworkLayerC__TinyosReceive__receive(message_t *msg);
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
static uint8_t TinyosNetworkLayerC__SubPacket__payloadLength(message_t *msg);









static uint8_t TinyosNetworkLayerC__SubPacket__maxPayloadLength(void );
#line 43
static void TinyosNetworkLayerC__SubPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 32
static uint8_t TinyosNetworkLayerC__SubPacket__headerLength(message_t *msg);
# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
static void TinyosNetworkLayerC__TinyosSend__sendDone(message_t *msg, error_t error);
#line 43
static void TinyosNetworkLayerC__Ieee154Send__sendDone(message_t *msg, error_t error);
# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareReceive.nc"
static message_t *TinyosNetworkLayerC__Ieee154Receive__receive(message_t *msg);
# 80 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TinyosNetworkLayerC.nc"
static inline uint8_t TinyosNetworkLayerC__Ieee154Packet__headerLength(message_t *msg);




static inline uint8_t TinyosNetworkLayerC__Ieee154Packet__payloadLength(message_t *msg);
#line 116
static network_header_t *TinyosNetworkLayerC__getHeader(message_t *msg);




static error_t TinyosNetworkLayerC__TinyosSend__send(message_t *msg);
#line 134
enum TinyosNetworkLayerC____nesc_unnamed4352 {


  TinyosNetworkLayerC__PAYLOAD_OFFSET = sizeof(network_header_t )
};




static inline uint8_t TinyosNetworkLayerC__TinyosPacket__headerLength(message_t *msg);




static inline uint8_t TinyosNetworkLayerC__TinyosPacket__payloadLength(message_t *msg);




static inline void TinyosNetworkLayerC__TinyosPacket__setPayloadLength(message_t *msg, uint8_t length);




static inline uint8_t TinyosNetworkLayerC__TinyosPacket__maxPayloadLength(void );
#line 203
static inline void TinyosNetworkLayerC__SubSend__sendDone(message_t *msg, error_t result);







static inline message_t *TinyosNetworkLayerC__SubReceive__receive(message_t *msg);
# 50 "/opt/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t Ieee154PacketLayerP__ActiveMessageAddress__amAddress(void );
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
static uint8_t Ieee154PacketLayerP__SubPacket__payloadLength(message_t *msg);









static uint8_t Ieee154PacketLayerP__SubPacket__maxPayloadLength(void );
#line 43
static void Ieee154PacketLayerP__SubPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 32
static uint8_t Ieee154PacketLayerP__SubPacket__headerLength(message_t *msg);
# 46 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
enum Ieee154PacketLayerP____nesc_unnamed4353 {

  Ieee154PacketLayerP__IEEE154_DATA_FRAME_MASK = (((IEEE154_TYPE_MASK << IEEE154_FCF_FRAME_TYPE)
   | (1 << IEEE154_FCF_INTRAPAN))
   | (IEEE154_ADDR_MASK << IEEE154_FCF_DEST_ADDR_MODE))
   | (IEEE154_ADDR_MASK << IEEE154_FCF_SRC_ADDR_MODE), 

  Ieee154PacketLayerP__IEEE154_DATA_FRAME_VALUE = (((IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE)
   | (1 << IEEE154_FCF_INTRAPAN))
   | (IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE))
   | (IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE), 

  Ieee154PacketLayerP__IEEE154_ACK_FRAME_LENGTH = 3, 
  Ieee154PacketLayerP__IEEE154_ACK_FRAME_MASK = IEEE154_TYPE_MASK << IEEE154_FCF_FRAME_TYPE, 
  Ieee154PacketLayerP__IEEE154_ACK_FRAME_VALUE = IEEE154_TYPE_ACK << IEEE154_FCF_FRAME_TYPE
};

static ieee154_header_t *Ieee154PacketLayerP__getHeader(message_t *msg);
#line 83
static inline bool Ieee154PacketLayerP__Ieee154PacketLayer__isDataFrame(message_t *msg);




static inline void Ieee154PacketLayerP__Ieee154PacketLayer__createDataFrame(message_t *msg);




static inline bool Ieee154PacketLayerP__Ieee154PacketLayer__isAckFrame(message_t *msg);










static inline void Ieee154PacketLayerP__Ieee154PacketLayer__createAckReply(message_t *data, message_t *ack);








static inline bool Ieee154PacketLayerP__Ieee154PacketLayer__verifyAckReply(message_t *data, message_t *ack);







static inline bool Ieee154PacketLayerP__Ieee154PacketLayer__getAckRequired(message_t *msg);




static void Ieee154PacketLayerP__Ieee154PacketLayer__setAckRequired(message_t *msg, bool ack);
#line 147
static inline uint8_t Ieee154PacketLayerP__Ieee154PacketLayer__getDSN(message_t *msg);




static inline void Ieee154PacketLayerP__Ieee154PacketLayer__setDSN(message_t *msg, uint8_t dsn);









static inline void Ieee154PacketLayerP__Ieee154PacketLayer__setDestPan(message_t *msg, uint16_t pan);




static inline uint16_t Ieee154PacketLayerP__Ieee154PacketLayer__getDestAddr(message_t *msg);




static inline void Ieee154PacketLayerP__Ieee154PacketLayer__setDestAddr(message_t *msg, uint16_t addr);




static inline uint16_t Ieee154PacketLayerP__Ieee154PacketLayer__getSrcAddr(message_t *msg);




static inline void Ieee154PacketLayerP__Ieee154PacketLayer__setSrcAddr(message_t *msg, uint16_t addr);




static inline bool Ieee154PacketLayerP__Ieee154PacketLayer__requiresAckWait(message_t *msg);






static bool Ieee154PacketLayerP__Ieee154PacketLayer__requiresAckReply(message_t *msg);
#line 207
static inline ieee154_saddr_t Ieee154PacketLayerP__Ieee154Packet__address(void );




static inline ieee154_saddr_t Ieee154PacketLayerP__Ieee154Packet__destination(message_t *msg);
#line 232
static inline bool Ieee154PacketLayerP__Ieee154Packet__isForMe(message_t *msg);
#line 255
static inline uint8_t Ieee154PacketLayerP__RadioPacket__headerLength(message_t *msg);




static inline uint8_t Ieee154PacketLayerP__RadioPacket__payloadLength(message_t *msg);




static inline void Ieee154PacketLayerP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);




static inline uint8_t Ieee154PacketLayerP__RadioPacket__maxPayloadLength(void );
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
static error_t UniqueLayerP__SubSend__send(message_t *msg);
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/NeighborhoodFlag.nc"
static bool UniqueLayerP__NeighborhoodFlag__get(uint8_t index);




static void UniqueLayerP__NeighborhoodFlag__set(uint8_t index);
# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
static void UniqueLayerP__Send__sendDone(message_t *msg, error_t error);
# 60 "/opt/tinyos-2.x/tos/chips/rf2xx/util/Neighborhood.nc"
static uint8_t UniqueLayerP__Neighborhood__insertNode(am_addr_t id);
# 42 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioReceive.nc"
static message_t *UniqueLayerP__RadioReceive__receive(message_t *msg);
#line 35
static bool UniqueLayerP__RadioReceive__header(message_t *msg);
# 41 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/UniqueConfig.nc"
static void UniqueLayerP__UniqueConfig__setSequenceNumber(message_t *msg, uint8_t number);





static void UniqueLayerP__UniqueConfig__reportChannelError(void );
#line 31
static uint8_t UniqueLayerP__UniqueConfig__getSequenceNumber(message_t *msg);




static am_addr_t UniqueLayerP__UniqueConfig__getSender(message_t *msg);
# 50 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/UniqueLayerP.nc"
uint8_t UniqueLayerP__sequenceNumber;

static inline error_t UniqueLayerP__Init__init(void );





static inline error_t UniqueLayerP__Send__send(message_t *msg);










static inline void UniqueLayerP__SubSend__sendDone(message_t *msg, error_t error);




static inline bool UniqueLayerP__SubReceive__header(message_t *msg);





uint8_t UniqueLayerP__receivedNumbers[5];

static inline message_t *UniqueLayerP__SubReceive__receive(message_t *msg);
#line 105
static inline void UniqueLayerP__Neighborhood__evicted(uint8_t index);
# 69 "/opt/tinyos-2.x/tos/chips/rf2xx/util/Neighborhood.nc"
static void NeighborhoodP__Neighborhood__evicted(uint8_t index);
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/NeighborhoodP.nc"
am_addr_t NeighborhoodP__nodes[5];
uint8_t NeighborhoodP__ages[5];
uint8_t NeighborhoodP__flags[5];
uint8_t NeighborhoodP__time;
uint8_t NeighborhoodP__last;

static inline error_t NeighborhoodP__Init__init(void );
#line 83
static inline uint8_t NeighborhoodP__Neighborhood__insertNode(am_addr_t node);
#line 147
static __inline bool NeighborhoodP__NeighborhoodFlag__get(uint8_t bit, uint8_t index);




static __inline void NeighborhoodP__NeighborhoodFlag__set(uint8_t bit, uint8_t index);
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
static error_t PacketLinkLayerP__SubSend__send(message_t *msg);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t PacketLinkLayerP__send__postTask(void );
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void PacketLinkLayerP__DelayTimer__startOneShot(uint32_t dt);




static void PacketLinkLayerP__DelayTimer__stop(void );
# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
static void PacketLinkLayerP__Send__sendDone(message_t *msg, error_t error);
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
static uint8_t PacketLinkLayerP__SubPacket__payloadLength(message_t *msg);









static uint8_t PacketLinkLayerP__SubPacket__maxPayloadLength(void );
#line 43
static void PacketLinkLayerP__SubPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 32
static uint8_t PacketLinkLayerP__SubPacket__headerLength(message_t *msg);
#line 54
static uint8_t PacketLinkLayerP__SubPacket__metadataLength(message_t *msg);
# 48 "/opt/tinyos-2.x/tos/interfaces/PacketAcknowledgements.nc"
static error_t PacketLinkLayerP__PacketAcknowledgements__requestAck(
#line 42
message_t * msg);
#line 74
static bool PacketLinkLayerP__PacketAcknowledgements__wasAcked(
#line 69
message_t * msg);
# 87 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/PacketLinkLayerP.nc"
enum PacketLinkLayerP____nesc_unnamed4354 {
#line 87
  PacketLinkLayerP__send = 3U
};
#line 87
typedef int PacketLinkLayerP____nesc_sillytask_send[PacketLinkLayerP__send];
#line 80
message_t *PacketLinkLayerP__currentSendMsg;


uint16_t PacketLinkLayerP__totalRetries;




static void PacketLinkLayerP__signalDone(error_t error);




static inline link_metadata_t *PacketLinkLayerP__getMeta(message_t *msg);










static inline void PacketLinkLayerP__PacketLink__setRetries(message_t *msg, uint16_t maxRetries);
#line 120
static inline uint16_t PacketLinkLayerP__PacketLink__getRetries(message_t *msg);






static uint16_t PacketLinkLayerP__PacketLink__getRetryDelay(message_t *msg);
#line 146
static inline error_t PacketLinkLayerP__Send__send(message_t *msg);
#line 172
static inline void PacketLinkLayerP__SubSend__sendDone(message_t *msg, error_t error);
#line 200
static inline void PacketLinkLayerP__DelayTimer__fired(void );






static inline void PacketLinkLayerP__send__runTask(void );










static void PacketLinkLayerP__signalDone(error_t error);
#line 231
static inline uint8_t PacketLinkLayerP__RadioPacket__headerLength(message_t *msg);



static inline uint8_t PacketLinkLayerP__RadioPacket__payloadLength(message_t *msg);



static inline void PacketLinkLayerP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);



static inline uint8_t PacketLinkLayerP__RadioPacket__maxPayloadLength(void );



static inline uint8_t PacketLinkLayerP__RadioPacket__metadataLength(message_t *msg);
# 69 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm128TimerCtrl8.nc"
static uint8_t /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__TimerCtrl__getInterruptFlag(void );
#line 62
static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__TimerCtrl__setControlB(uint8_t control);
#line 61
static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__TimerCtrl__setControlA(uint8_t control);
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__fired(void );
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Counter__overflow(void );
# 57 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm128TimerAsync.nc"
static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__TimerAsync__setTimer2Asynchronous(void );
#line 75
static int /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__TimerAsync__compareABusy(void );
# 39 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
static /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Compare__size_type /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Compare__get(void );





static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Compare__set(/*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Compare__size_type t);










static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Compare__start(void );
# 52 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
static /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Timer__timer_size /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Timer__get(void );
# 62 "/opt/tinyos-2.x/tos/chips/atm1281/timer/Atm1281AlarmAsyncP.nc"
uint8_t /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__set;
uint32_t /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__t0;
#line 63
uint32_t /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__dt;
uint32_t /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__base;



enum /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0____nesc_unnamed4355 {
  Atm1281AlarmAsyncP__0__MINDT = 2, 
  Atm1281AlarmAsyncP__0__MAXT = 230
};



static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__setInterrupt(void );


static inline error_t /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Init__init(void );
#line 101
static inline void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__setOcr2A(uint8_t n);
#line 117
static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__setInterrupt(void );
#line 176
static inline void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Compare__fired(void );
#line 188
static uint32_t /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Counter__get(void );
#line 231
static inline void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__stop(void );







static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__startAt(uint32_t nt0, uint32_t ndt);









static inline uint32_t /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__getNow(void );



static inline uint32_t /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__getAlarm(void );



static inline void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Timer__overflow(void );
# 49 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
static void HplAtm1281Timer2AsyncP__Compare__fired(void );
# 61 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
static void HplAtm1281Timer2AsyncP__Timer__overflow(void );
# 79 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer2AsyncP.nc"
static inline uint8_t HplAtm1281Timer2AsyncP__Timer__get(void );
#line 111
static inline void HplAtm1281Timer2AsyncP__TimerCtrl__setControlA(uint8_t x);





static inline void HplAtm1281Timer2AsyncP__TimerCtrl__setControlB(uint8_t x);
#line 134
static inline uint8_t HplAtm1281Timer2AsyncP__TimerCtrl__getInterruptFlag(void );
#line 161
static inline void HplAtm1281Timer2AsyncP__Compare__start(void );









static inline uint8_t HplAtm1281Timer2AsyncP__Compare__get(void );


static inline void HplAtm1281Timer2AsyncP__Compare__set(uint8_t t);









static __inline void HplAtm1281Timer2AsyncP__stabiliseTimer2(void );
#line 199
static inline mcu_power_t HplAtm1281Timer2AsyncP__McuPowerOverride__lowestState(void );
#line 222
void __vector_13(void ) __attribute((signal))   ;







void __vector_15(void ) __attribute((signal))   ;
#line 246
static inline void HplAtm1281Timer2AsyncP__TimerAsync__setTimer2Asynchronous(void );
#line 258
static inline int HplAtm1281Timer2AsyncP__TimerAsync__compareABusy(void );
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
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_unnamed4356 {
#line 63
  AlarmToTimerC__0__fired = 4U
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
uint8_t arg_0xf6ff0d88);
#line 60
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4357 {
#line 60
  VirtualizeTimerC__0__updateFromTimer = 5U
};
#line 60
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer];
#line 42
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4358 {

  VirtualizeTimerC__0__NUM_TIMERS = 5, 
  VirtualizeTimerC__0__END_OF_LIST = 255
};








#line 48
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4359 {

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
#line 148
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num);




static inline bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(uint8_t num);
#line 193
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num);
# 47 "/opt/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static void MessageBufferLayerP__SplitControl__startDone(error_t error);
#line 117
static void MessageBufferLayerP__SplitControl__stopDone(error_t error);
# 33 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioState.nc"
static error_t MessageBufferLayerP__RadioState__turnOff(void );
#line 45
static error_t MessageBufferLayerP__RadioState__turnOn(void );
# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
static void MessageBufferLayerP__Send__sendDone(message_t *msg, error_t error);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t MessageBufferLayerP__stateDoneTask__postTask(void );
#line 56
static error_t MessageBufferLayerP__sendTask__postTask(void );
# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareReceive.nc"
static message_t *MessageBufferLayerP__Receive__receive(message_t *msg);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t MessageBufferLayerP__deliverTask__postTask(void );
# 37 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioSend.nc"
static error_t MessageBufferLayerP__RadioSend__send(message_t *msg);
# 37 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioChannel.nc"
static void MessageBufferLayerP__RadioChannel__setChannelDone(void );
# 61 "/opt/tinyos-2.x/tos/chips/rf2xx/util/Tasklet.nc"
static void MessageBufferLayerP__Tasklet__suspend(void );






static void MessageBufferLayerP__Tasklet__resume(void );
# 125 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MessageBufferLayerP.nc"
enum MessageBufferLayerP____nesc_unnamed4360 {
#line 125
  MessageBufferLayerP__stateDoneTask = 6U
};
#line 125
typedef int MessageBufferLayerP____nesc_sillytask_stateDoneTask[MessageBufferLayerP__stateDoneTask];
#line 170
enum MessageBufferLayerP____nesc_unnamed4361 {
#line 170
  MessageBufferLayerP__sendTask = 7U
};
#line 170
typedef int MessageBufferLayerP____nesc_sillytask_sendTask[MessageBufferLayerP__sendTask];
#line 280
enum MessageBufferLayerP____nesc_unnamed4362 {
#line 280
  MessageBufferLayerP__deliverTask = 8U
};
#line 280
typedef int MessageBufferLayerP____nesc_sillytask_deliverTask[MessageBufferLayerP__deliverTask];
#line 51
uint8_t MessageBufferLayerP__state;
enum MessageBufferLayerP____nesc_unnamed4363 {

  MessageBufferLayerP__STATE_READY = 0, 
  MessageBufferLayerP__STATE_TX_PENDING = 1, 
  MessageBufferLayerP__STATE_TX_SEND = 2, 
  MessageBufferLayerP__STATE_TX_DONE = 3, 
  MessageBufferLayerP__STATE_TURN_ON = 4, 
  MessageBufferLayerP__STATE_TURN_OFF = 5, 
  MessageBufferLayerP__STATE_CHANNEL = 6
};

static error_t MessageBufferLayerP__SplitControl__start(void );
#line 82
static error_t MessageBufferLayerP__SplitControl__stop(void );
#line 125
static inline void MessageBufferLayerP__stateDoneTask__runTask(void );
#line 144
static inline void MessageBufferLayerP__RadioState__done(void );
#line 157
static inline void MessageBufferLayerP__RadioChannel__default__setChannelDone(void );





message_t *MessageBufferLayerP__txMsg;
error_t MessageBufferLayerP__txError;
uint8_t MessageBufferLayerP__retries;


enum MessageBufferLayerP____nesc_unnamed4364 {
#line 168
  MessageBufferLayerP__MAX_RETRIES = 5
};
static inline void MessageBufferLayerP__sendTask__runTask(void );
#line 201
static void MessageBufferLayerP__RadioSend__sendDone(error_t error);







static error_t MessageBufferLayerP__Send__send(message_t *msg);
#line 222
static inline void MessageBufferLayerP__RadioSend__ready(void );





static inline void MessageBufferLayerP__Tasklet__run(void );
#line 249
enum MessageBufferLayerP____nesc_unnamed4365 {

  MessageBufferLayerP__RECEIVE_QUEUE_SIZE = 3
};

message_t MessageBufferLayerP__receiveQueueData[MessageBufferLayerP__RECEIVE_QUEUE_SIZE];
message_t *MessageBufferLayerP__receiveQueue[MessageBufferLayerP__RECEIVE_QUEUE_SIZE];

uint8_t MessageBufferLayerP__receiveQueueHead;
uint8_t MessageBufferLayerP__receiveQueueSize;

static inline error_t MessageBufferLayerP__SoftwareInit__init(void );









static inline bool MessageBufferLayerP__RadioReceive__header(message_t *msg);









static inline void MessageBufferLayerP__deliverTask__runTask(void );
#line 309
static inline message_t *MessageBufferLayerP__RadioReceive__receive(message_t *msg);
# 37 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioSend.nc"
static error_t RandomCollisionLayerP__SubSend__send(message_t *msg);
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
static void RandomCollisionLayerP__RadioAlarm__wait(uint16_t timeout);
#line 33
static bool RandomCollisionLayerP__RadioAlarm__isFree(void );
#line 53
static uint16_t RandomCollisionLayerP__RadioAlarm__getNow(void );
# 41 "/opt/tinyos-2.x/tos/interfaces/Random.nc"
static uint16_t RandomCollisionLayerP__Random__rand16(void );
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/RandomCollisionConfig.nc"
static uint16_t RandomCollisionLayerP__Config__getCongestionBackoff(message_t *msg);
#line 29
static uint16_t RandomCollisionLayerP__Config__getInitialBackoff(message_t *msg);










static uint16_t RandomCollisionLayerP__Config__getMinimumBackoff(void );





static uint16_t RandomCollisionLayerP__Config__getTransmitBarrier(message_t *msg);
# 42 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioReceive.nc"
static message_t *RandomCollisionLayerP__RadioReceive__receive(message_t *msg);
#line 35
static bool RandomCollisionLayerP__RadioReceive__header(message_t *msg);
# 52 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioSend.nc"
static void RandomCollisionLayerP__RadioSend__ready(void );
#line 45
static void RandomCollisionLayerP__RadioSend__sendDone(error_t error);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t RandomCollisionLayerP__calcNextRandom__postTask(void );
# 67 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/RandomCollisionLayerP.nc"
enum RandomCollisionLayerP____nesc_unnamed4366 {
#line 67
  RandomCollisionLayerP__calcNextRandom = 9U
};
#line 67
typedef int RandomCollisionLayerP____nesc_sillytask_calcNextRandom[RandomCollisionLayerP__calcNextRandom];
#line 46
uint8_t RandomCollisionLayerP__state;
enum RandomCollisionLayerP____nesc_unnamed4367 {

  RandomCollisionLayerP__STATE_READY = 0, 
  RandomCollisionLayerP__STATE_TX_PENDING_FIRST = 1, 
  RandomCollisionLayerP__STATE_TX_PENDING_SECOND = 2, 
  RandomCollisionLayerP__STATE_TX_SENDING = 3, 

  RandomCollisionLayerP__STATE_BARRIER = 0x80
};

message_t *RandomCollisionLayerP__txMsg;
uint16_t RandomCollisionLayerP__txBarrier;

static inline void RandomCollisionLayerP__SubSend__ready(void );





uint16_t RandomCollisionLayerP__nextRandom;
static inline void RandomCollisionLayerP__calcNextRandom__runTask(void );





static uint16_t RandomCollisionLayerP__getBackoff(uint16_t maxBackoff);
#line 87
static inline error_t RandomCollisionLayerP__RadioSend__send(message_t *msg);
#line 99
static inline void RandomCollisionLayerP__RadioAlarm__fired(void );
#line 144
static inline void RandomCollisionLayerP__SubSend__sendDone(error_t error);







static inline bool RandomCollisionLayerP__SubReceive__header(message_t *msg);




static inline message_t *RandomCollisionLayerP__SubReceive__receive(message_t *msg);
# 41 "/opt/tinyos-2.x/tos/system/RandomMlcgC.nc"
uint32_t RandomMlcgC__seed;


static inline error_t RandomMlcgC__Init__init(void );
#line 58
static uint32_t RandomMlcgC__Random__rand32(void );
#line 78
static inline uint16_t RandomMlcgC__Random__rand16(void );
# 37 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioSend.nc"
static error_t SoftwareAckLayerP__SubSend__send(message_t *msg);
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
static void SoftwareAckLayerP__RadioAlarm__wait(uint16_t timeout);




static void SoftwareAckLayerP__RadioAlarm__cancel(void );
#line 33
static bool SoftwareAckLayerP__RadioAlarm__isFree(void );
# 44 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketFlag.nc"
static void SoftwareAckLayerP__AckReceivedFlag__clear(message_t *msg);
#line 29
static bool SoftwareAckLayerP__AckReceivedFlag__get(message_t *msg);




static void SoftwareAckLayerP__AckReceivedFlag__setValue(message_t *msg, bool value);
# 42 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioReceive.nc"
static message_t *SoftwareAckLayerP__RadioReceive__receive(message_t *msg);
#line 35
static bool SoftwareAckLayerP__RadioReceive__header(message_t *msg);
# 75 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/SoftwareAckConfig.nc"
static void SoftwareAckLayerP__SoftwareAckConfig__reportChannelError(void );
#line 69
static void SoftwareAckLayerP__SoftwareAckConfig__createAckPacket(message_t *data, message_t *ack);
#line 44
static bool SoftwareAckLayerP__SoftwareAckConfig__requiresAckWait(message_t *msg);






static bool SoftwareAckLayerP__SoftwareAckConfig__isAckPacket(message_t *msg);






static bool SoftwareAckLayerP__SoftwareAckConfig__verifyAckPacket(message_t *data, message_t *ack);
#line 32
static uint16_t SoftwareAckLayerP__SoftwareAckConfig__getAckTimeout(void );





static void SoftwareAckLayerP__SoftwareAckConfig__setAckRequired(message_t *msg, bool ack);
#line 64
static bool SoftwareAckLayerP__SoftwareAckConfig__requiresAckReply(message_t *msg);
# 52 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioSend.nc"
static void SoftwareAckLayerP__RadioSend__ready(void );
#line 45
static void SoftwareAckLayerP__RadioSend__sendDone(error_t error);
# 49 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/SoftwareAckLayerP.nc"
uint8_t SoftwareAckLayerP__state;
enum SoftwareAckLayerP____nesc_unnamed4368 {

  SoftwareAckLayerP__STATE_READY = 0, 
  SoftwareAckLayerP__STATE_DATA_SEND = 1, 
  SoftwareAckLayerP__STATE_ACK_WAIT = 2, 
  SoftwareAckLayerP__STATE_ACK_SEND = 3
};

message_t *SoftwareAckLayerP__txMsg;
message_t SoftwareAckLayerP__ackMsg;

static inline void SoftwareAckLayerP__SubSend__ready(void );





static inline error_t SoftwareAckLayerP__RadioSend__send(message_t *msg);
#line 86
static inline void SoftwareAckLayerP__SubSend__sendDone(error_t error);
#line 113
static inline void SoftwareAckLayerP__RadioAlarm__fired(void );









static inline bool SoftwareAckLayerP__SubReceive__header(message_t *msg);







static inline message_t *SoftwareAckLayerP__SubReceive__receive(message_t *msg);
#line 167
static inline error_t SoftwareAckLayerP__PacketAcknowledgements__requestAck(message_t *msg);
#line 181
static inline bool SoftwareAckLayerP__PacketAcknowledgements__wasAcked(message_t *msg);
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
static uint8_t MetadataFlagsLayerC__SubPacket__payloadLength(message_t *msg);









static uint8_t MetadataFlagsLayerC__SubPacket__maxPayloadLength(void );
#line 43
static void MetadataFlagsLayerC__SubPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 32
static uint8_t MetadataFlagsLayerC__SubPacket__headerLength(message_t *msg);
#line 54
static uint8_t MetadataFlagsLayerC__SubPacket__metadataLength(message_t *msg);
# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MetadataFlagsLayerC.nc"
static flags_metadata_t *MetadataFlagsLayerC__getMeta(message_t *msg);






static inline bool MetadataFlagsLayerC__PacketFlag__get(uint8_t bit, message_t *msg);




static inline void MetadataFlagsLayerC__PacketFlag__set(uint8_t bit, message_t *msg);






static inline void MetadataFlagsLayerC__PacketFlag__clear(uint8_t bit, message_t *msg);






static inline void MetadataFlagsLayerC__PacketFlag__setValue(uint8_t bit, message_t *msg, bool value);









static inline uint8_t MetadataFlagsLayerC__RadioPacket__headerLength(message_t *msg);




static inline uint8_t MetadataFlagsLayerC__RadioPacket__payloadLength(message_t *msg);




static inline void MetadataFlagsLayerC__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);




static inline uint8_t MetadataFlagsLayerC__RadioPacket__maxPayloadLength(void );




static inline uint8_t MetadataFlagsLayerC__RadioPacket__metadataLength(message_t *msg);
# 44 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketFlag.nc"
static void TimeStampingLayerP__TimeStampFlag__clear(message_t *msg);
#line 39
static void TimeStampingLayerP__TimeStampFlag__set(message_t *msg);
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
static uint8_t TimeStampingLayerP__SubPacket__payloadLength(message_t *msg);









static uint8_t TimeStampingLayerP__SubPacket__maxPayloadLength(void );
#line 43
static void TimeStampingLayerP__SubPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 32
static uint8_t TimeStampingLayerP__SubPacket__headerLength(message_t *msg);
#line 54
static uint8_t TimeStampingLayerP__SubPacket__metadataLength(message_t *msg);
# 49 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TimeStampingLayerP.nc"
static timestamp_metadata_t *TimeStampingLayerP__getMeta(message_t *msg);
#line 66
static inline void TimeStampingLayerP__PacketTimeStampRadio__clear(message_t *msg);




static inline void TimeStampingLayerP__PacketTimeStampRadio__set(message_t *msg, uint32_t value);
#line 105
static inline uint8_t TimeStampingLayerP__RadioPacket__headerLength(message_t *msg);




static inline uint8_t TimeStampingLayerP__RadioPacket__payloadLength(message_t *msg);




static inline void TimeStampingLayerP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);




static inline uint8_t TimeStampingLayerP__RadioPacket__maxPayloadLength(void );




static inline uint8_t TimeStampingLayerP__RadioPacket__metadataLength(message_t *msg);
# 41 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioCCA.nc"
static void RF230DriverLayerP__RadioCCA__done(error_t error);
# 55 "/opt/tinyos-2.x/tos/lib/timer/BusyWait.nc"
static void RF230DriverLayerP__BusyWait__wait(RF230DriverLayerP__BusyWait__size_type dt);
# 44 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketFlag.nc"
static void RF230DriverLayerP__RSSIFlag__clear(message_t *msg);
#line 29
static bool RF230DriverLayerP__RSSIFlag__get(message_t *msg);









static void RF230DriverLayerP__RSSIFlag__set(message_t *msg);
# 59 "/opt/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
static void RF230DriverLayerP__PacketTimeStamp__clear(
#line 55
message_t * msg);
#line 67
static void RF230DriverLayerP__PacketTimeStamp__set(
#line 62
message_t * msg, 




RF230DriverLayerP__PacketTimeStamp__size_type value);
# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void RF230DriverLayerP__RSTN__makeOutput(void );
#line 29
static void RF230DriverLayerP__RSTN__set(void );
static void RF230DriverLayerP__RSTN__clr(void );
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
static void RF230DriverLayerP__RadioAlarm__wait(uint16_t timeout);
#line 33
static bool RF230DriverLayerP__RadioAlarm__isFree(void );
#line 53
static uint16_t RF230DriverLayerP__RadioAlarm__getNow(void );
# 58 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioState.nc"
static void RF230DriverLayerP__RadioState__done(void );
# 50 "/opt/tinyos-2.x/tos/lib/timer/LocalTime.nc"
static uint32_t RF230DriverLayerP__LocalTime__get(void );
# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void RF230DriverLayerP__SLP_TR__makeOutput(void );
#line 29
static void RF230DriverLayerP__SLP_TR__set(void );
static void RF230DriverLayerP__SLP_TR__clr(void );
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverConfig.nc"
static uint8_t RF230DriverLayerP__Config__maxPayloadLength(void );
#line 29
static uint8_t RF230DriverLayerP__Config__headerLength(message_t *msg);
#line 41
static uint8_t RF230DriverLayerP__Config__metadataLength(message_t *msg);






static uint8_t RF230DriverLayerP__Config__headerPreloadLength(void );





static bool RF230DriverLayerP__Config__requiresRssiCca(message_t *msg);
# 42 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioReceive.nc"
static message_t *RF230DriverLayerP__RadioReceive__receive(message_t *msg);
#line 35
static bool RF230DriverLayerP__RadioReceive__header(message_t *msg);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t RF230DriverLayerP__SpiResource__release(void );
#line 87
static error_t RF230DriverLayerP__SpiResource__immediateRequest(void );
#line 78
static error_t RF230DriverLayerP__SpiResource__request(void );
#line 118
static bool RF230DriverLayerP__SpiResource__isOwner(void );
# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void RF230DriverLayerP__SELN__makeOutput(void );
#line 29
static void RF230DriverLayerP__SELN__set(void );
static void RF230DriverLayerP__SELN__clr(void );
# 52 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioSend.nc"
static void RF230DriverLayerP__RadioSend__ready(void );
#line 45
static void RF230DriverLayerP__RadioSend__sendDone(error_t error);
# 55 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
static void RF230DriverLayerP__IRQ__disable(void );
#line 42
static error_t RF230DriverLayerP__IRQ__captureRisingEdge(void );
# 46 "/opt/tinyos-2.x/tos/interfaces/FastSpiByte.nc"
static void RF230DriverLayerP__FastSpiByte__splitWrite(uint8_t data);
#line 58
static uint8_t RF230DriverLayerP__FastSpiByte__splitReadWrite(uint8_t data);
#line 52
static uint8_t RF230DriverLayerP__FastSpiByte__splitRead(void );
#line 65
static uint8_t RF230DriverLayerP__FastSpiByte__write(uint8_t data);
# 29 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketFlag.nc"
static bool RF230DriverLayerP__TimeSyncFlag__get(message_t *msg);
#line 44
static void RF230DriverLayerP__TransmitPowerFlag__clear(message_t *msg);
#line 29
static bool RF230DriverLayerP__TransmitPowerFlag__get(message_t *msg);
# 48 "/opt/tinyos-2.x/tos/chips/rf2xx/util/Tasklet.nc"
static void RF230DriverLayerP__Tasklet__schedule(void );
# 83 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static inline rf230_header_t *RF230DriverLayerP__getHeader(message_t *msg);




static void *RF230DriverLayerP__getPayload(message_t *msg);




static rf230_metadata_t *RF230DriverLayerP__getMeta(message_t *msg);






uint8_t RF230DriverLayerP__state;
enum RF230DriverLayerP____nesc_unnamed4369 {

  RF230DriverLayerP__STATE_P_ON = 0, 
  RF230DriverLayerP__STATE_SLEEP = 1, 
  RF230DriverLayerP__STATE_SLEEP_2_TRX_OFF = 2, 
  RF230DriverLayerP__STATE_TRX_OFF = 3, 
  RF230DriverLayerP__STATE_TRX_OFF_2_RX_ON = 4, 
  RF230DriverLayerP__STATE_RX_ON = 5, 
  RF230DriverLayerP__STATE_BUSY_TX_2_RX_ON = 6, 
  RF230DriverLayerP__STATE_PLL_ON_2_RX_ON = 7
};

uint8_t RF230DriverLayerP__cmd;
enum RF230DriverLayerP____nesc_unnamed4370 {

  RF230DriverLayerP__CMD_NONE = 0, 
  RF230DriverLayerP__CMD_TURNOFF = 1, 
  RF230DriverLayerP__CMD_STANDBY = 2, 
  RF230DriverLayerP__CMD_TURNON = 3, 
  RF230DriverLayerP__CMD_TRANSMIT = 4, 
  RF230DriverLayerP__CMD_RECEIVE = 5, 
  RF230DriverLayerP__CMD_CCA = 6, 
  RF230DriverLayerP__CMD_CHANNEL = 7, 
  RF230DriverLayerP__CMD_SIGNAL_DONE = 8, 
  RF230DriverLayerP__CMD_DOWNLOAD = 9
};

bool RF230DriverLayerP__radioIrq;

uint8_t RF230DriverLayerP__txPower;
uint8_t RF230DriverLayerP__channel;

message_t *RF230DriverLayerP__rxMsg;
message_t RF230DriverLayerP__rxMsgBuffer;

uint16_t RF230DriverLayerP__capturedTime;

uint8_t RF230DriverLayerP__rssiClear;
uint8_t RF230DriverLayerP__rssiBusy;



static __inline void RF230DriverLayerP__writeRegister(uint8_t reg, uint8_t value);
#line 155
static __inline uint8_t RF230DriverLayerP__readRegister(uint8_t reg);
#line 171
enum RF230DriverLayerP____nesc_unnamed4371 {

  RF230DriverLayerP__SLEEP_WAKEUP_TIME = (uint16_t )(880 * (73728UL / 8 / 32) * (1 << MICA_DIVIDE_ONE_FOR_32KHZ_LOG2) / 10000UL), 
  RF230DriverLayerP__CCA_REQUEST_TIME = (uint16_t )(140 * (73728UL / 8 / 32) * (1 << MICA_DIVIDE_ONE_FOR_32KHZ_LOG2) / 10000UL), 

  RF230DriverLayerP__TX_SFD_DELAY = (uint16_t )(176 * (73728UL / 8 / 32) * (1 << MICA_DIVIDE_ONE_FOR_32KHZ_LOG2) / 10000UL), 
  RF230DriverLayerP__RX_SFD_DELAY = (uint16_t )(8 * (73728UL / 8 / 32) * (1 << MICA_DIVIDE_ONE_FOR_32KHZ_LOG2) / 10000UL)
};

static inline void RF230DriverLayerP__RadioAlarm__fired(void );
#line 206
static inline error_t RF230DriverLayerP__PlatformInit__init(void );
#line 224
static inline error_t RF230DriverLayerP__SoftwareInit__init(void );





static inline void RF230DriverLayerP__initRadio(void );
#line 258
static inline void RF230DriverLayerP__SpiResource__granted(void );
#line 272
static bool RF230DriverLayerP__isSpiAcquired(void );
#line 312
static __inline void RF230DriverLayerP__changeChannel(void );
#line 330
static __inline void RF230DriverLayerP__changeState(void );
#line 374
static inline error_t RF230DriverLayerP__RadioState__turnOff(void );
#line 400
static inline error_t RF230DriverLayerP__RadioState__turnOn(void );
#line 417
static error_t RF230DriverLayerP__RadioSend__send(message_t *msg);
#line 575
static inline void RF230DriverLayerP__RadioCCA__default__done(error_t error);



static __inline void RF230DriverLayerP__downloadMessage(void );
#line 661
static inline void RF230DriverLayerP__IRQ__captured(uint16_t time);
#line 674
static inline void RF230DriverLayerP__serviceRadio(void );
#line 832
static inline void RF230DriverLayerP__Tasklet__run(void );
#line 862
static inline uint8_t RF230DriverLayerP__RadioPacket__headerLength(message_t *msg);




static inline uint8_t RF230DriverLayerP__RadioPacket__payloadLength(message_t *msg);




static void RF230DriverLayerP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);








static inline uint8_t RF230DriverLayerP__RadioPacket__maxPayloadLength(void );






static inline uint8_t RF230DriverLayerP__RadioPacket__metadataLength(message_t *msg);
#line 900
static inline bool RF230DriverLayerP__PacketTransmitPower__isSet(message_t *msg);




static inline uint8_t RF230DriverLayerP__PacketTransmitPower__get(message_t *msg);
#line 923
static inline bool RF230DriverLayerP__PacketRSSI__isSet(message_t *msg);




static inline uint8_t RF230DriverLayerP__PacketRSSI__get(message_t *msg);




static inline void RF230DriverLayerP__PacketRSSI__clear(message_t *msg);




static inline void RF230DriverLayerP__PacketRSSI__set(message_t *msg, uint8_t value);










static inline bool RF230DriverLayerP__PacketTimeSyncOffset__isSet(message_t *msg);




static inline uint8_t RF230DriverLayerP__PacketTimeSyncOffset__get(message_t *msg);
#line 988
static inline void RF230DriverLayerP__PacketLinkQuality__set(message_t *msg, uint8_t value);
# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void HplRF230P__PortCLKM__makeInput(void );
#line 30
static void HplRF230P__PortCLKM__clr(void );
# 79 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Capture.nc"
static void HplRF230P__Capture__setEdge(bool up);
#line 38
static HplRF230P__Capture__size_type HplRF230P__Capture__get(void );
#line 55
static void HplRF230P__Capture__reset(void );


static void HplRF230P__Capture__start(void );


static void HplRF230P__Capture__stop(void );
# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void HplRF230P__PortIRQ__makeInput(void );
#line 30
static void HplRF230P__PortIRQ__clr(void );
# 50 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
static void HplRF230P__IRQ__captured(uint16_t time);
# 42 "/opt/tinyos-2.x/tos/platforms/iris/chips/rf230/HplRF230P.nc"
static inline error_t HplRF230P__PlatformInit__init(void );










static inline void HplRF230P__Capture__captured(uint16_t time);









static inline error_t HplRF230P__IRQ__captureRisingEdge(void );
#line 78
static inline void HplRF230P__IRQ__disable(void );
# 71 "/opt/tinyos-2.x/tos/interfaces/SpiPacket.nc"
static void Atm128SpiP__SpiPacket__sendDone(
#line 64
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t Atm128SpiP__ResourceArbiter__release(
# 85 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
uint8_t arg_0xf6e13068);
# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t Atm128SpiP__ResourceArbiter__immediateRequest(
# 85 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
uint8_t arg_0xf6e13068);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t Atm128SpiP__ResourceArbiter__request(
# 85 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
uint8_t arg_0xf6e13068);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static bool Atm128SpiP__ResourceArbiter__isOwner(
# 85 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
uint8_t arg_0xf6e13068);
# 72 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128Spi.nc"
static void Atm128SpiP__Spi__sleep(void );
#line 66
static void Atm128SpiP__Spi__initMaster(void );
#line 96
static void Atm128SpiP__Spi__enableInterrupt(bool enabled);
#line 80
static uint8_t Atm128SpiP__Spi__read(void );
#line 125
static void Atm128SpiP__Spi__setMasterDoubleSpeed(bool on);
#line 114
static void Atm128SpiP__Spi__setClock(uint8_t speed);
#line 108
static void Atm128SpiP__Spi__setClockPolarity(bool highWhenIdle);
#line 86
static void Atm128SpiP__Spi__write(uint8_t data);
#line 99
static void Atm128SpiP__Spi__enableSpi(bool busOn);
#line 111
static void Atm128SpiP__Spi__setClockPhase(bool sampleOnTrailing);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void Atm128SpiP__Resource__granted(
# 81 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
uint8_t arg_0xf6e15398);
# 80 "/opt/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
static bool Atm128SpiP__ArbiterInfo__inUse(void );
# 44 "/opt/tinyos-2.x/tos/interfaces/McuPowerState.nc"
static void Atm128SpiP__McuPowerState__update(void );
# 241 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
enum Atm128SpiP____nesc_unnamed4372 {
#line 241
  Atm128SpiP__zeroTask = 10U
};
#line 241
typedef int Atm128SpiP____nesc_sillytask_zeroTask[Atm128SpiP__zeroTask];
#line 91
uint16_t Atm128SpiP__len;
uint8_t * Atm128SpiP__txBuffer;
uint8_t * Atm128SpiP__rxBuffer;
uint16_t Atm128SpiP__pos;

enum Atm128SpiP____nesc_unnamed4373 {
  Atm128SpiP__SPI_IDLE, 
  Atm128SpiP__SPI_BUSY, 
  Atm128SpiP__SPI_ATOMIC_SIZE = 10
};





static void Atm128SpiP__startSpi(void );
#line 120
static inline void Atm128SpiP__stopSpi(void );







static uint8_t Atm128SpiP__SpiByte__write(uint8_t tx);
#line 142
static __inline void Atm128SpiP__FastSpiByte__splitWrite(uint8_t data);



static __inline uint8_t Atm128SpiP__FastSpiByte__splitRead(void );





static __inline uint8_t Atm128SpiP__FastSpiByte__splitReadWrite(uint8_t data);










static __inline uint8_t Atm128SpiP__FastSpiByte__write(uint8_t data);
#line 196
static inline error_t Atm128SpiP__sendNextPart(void );
#line 241
static inline void Atm128SpiP__zeroTask__runTask(void );
#line 296
static inline 
#line 295
void Atm128SpiP__SpiPacket__default__sendDone(
uint8_t *_txbuffer, uint8_t *_rxbuffer, 
uint16_t _length, error_t _success);

static inline void Atm128SpiP__Spi__dataReady(uint8_t data);
#line 339
static inline error_t Atm128SpiP__Resource__immediateRequest(uint8_t id);







static error_t Atm128SpiP__Resource__request(uint8_t id);








static error_t Atm128SpiP__Resource__release(uint8_t id);









static inline uint8_t Atm128SpiP__Resource__isOwner(uint8_t id);



static inline void Atm128SpiP__ResourceArbiter__granted(uint8_t id);



static inline void Atm128SpiP__Resource__default__granted(uint8_t id);
# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void HplAtm128SpiP__MISO__makeInput(void );

static void HplAtm128SpiP__SCK__makeOutput(void );
# 44 "/opt/tinyos-2.x/tos/interfaces/McuPowerState.nc"
static void HplAtm128SpiP__Mcu__update(void );
# 92 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128Spi.nc"
static void HplAtm128SpiP__SPI__dataReady(uint8_t data);
# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void HplAtm128SpiP__MOSI__makeOutput(void );
# 79 "/opt/tinyos-2.x/tos/chips/atm128/spi/HplAtm128SpiP.nc"
static inline void HplAtm128SpiP__SPI__initMaster(void );
#line 94
static inline void HplAtm128SpiP__SPI__sleep(void );



static inline uint8_t HplAtm128SpiP__SPI__read(void );
static inline void HplAtm128SpiP__SPI__write(uint8_t d);


void __vector_24(void ) __attribute((signal))   ;
#line 115
static void HplAtm128SpiP__SPI__enableInterrupt(bool enabled);
#line 130
static void HplAtm128SpiP__SPI__enableSpi(bool enabled);
#line 156
static inline void HplAtm128SpiP__SPI__setMasterBit(bool isMaster);
#line 169
static inline void HplAtm128SpiP__SPI__setClockPolarity(bool highWhenIdle);
#line 183
static inline void HplAtm128SpiP__SPI__setClockPhase(bool sampleOnTrailing);
#line 200
static inline void HplAtm128SpiP__SPI__setClock(uint8_t v);
#line 213
static inline void HplAtm128SpiP__SPI__setMasterDoubleSpeed(bool on);
# 39 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
enum /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1____nesc_unnamed4374 {
#line 39
  FcfsResourceQueueC__1__NO_ENTRY = 0xFF
};
uint8_t /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__resQ[1U];
uint8_t /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__qHead = /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
uint8_t /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__qTail = /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

static inline error_t /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__Init__init(void );




static inline bool /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void );



static inline bool /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void );
#line 72
static inline error_t /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id);
# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceRequested__requested(
# 52 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71c77d0);
# 51 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceRequested__immediateRequested(
# 52 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71c77d0);
# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceConfigure__unconfigure(
# 56 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71c5430);
# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceConfigure__configure(
# 56 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71c5430);
# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static error_t /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Queue__enqueue(resource_client_id_t id);
#line 43
static bool /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Queue__isEmpty(void );
#line 60
static resource_client_id_t /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Queue__dequeue(void );
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Resource__granted(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0xf71d8d90);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__grantedTask__postTask(void );
# 69 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
enum /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1____nesc_unnamed4375 {
#line 69
  SimpleArbiterP__1__grantedTask = 11U
};
#line 69
typedef int /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1____nesc_sillytask_grantedTask[/*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__grantedTask];
#line 62
enum /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1____nesc_unnamed4376 {
#line 62
  SimpleArbiterP__1__RES_IDLE = 0, SimpleArbiterP__1__RES_GRANTING = 1, SimpleArbiterP__1__RES_BUSY = 2
};
#line 63
enum /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1____nesc_unnamed4377 {
#line 63
  SimpleArbiterP__1__NO_RES = 0xFF
};
uint8_t /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__state = /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__RES_IDLE;
uint8_t /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__resId = /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__NO_RES;
uint8_t /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__reqResId;



static inline error_t /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Resource__request(uint8_t id);
#line 84
static inline error_t /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Resource__immediateRequest(uint8_t id);
#line 97
static inline error_t /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Resource__release(uint8_t id);
#line 124
static bool /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ArbiterInfo__inUse(void );
#line 148
static inline uint8_t /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Resource__isOwner(uint8_t id);






static inline void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__grantedTask__runTask(void );
#line 167
static inline void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceRequested__default__requested(uint8_t id);

static inline void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceRequested__default__immediateRequested(uint8_t id);

static inline void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceConfigure__default__unconfigure(uint8_t id);
# 49 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
static void HplAtm1281Timer1P__CompareA__fired(void );
# 51 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Capture.nc"
static void HplAtm1281Timer1P__Capture__captured(HplAtm1281Timer1P__Capture__size_type t);
# 49 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
static void HplAtm1281Timer1P__CompareB__fired(void );
#line 49
static void HplAtm1281Timer1P__CompareC__fired(void );
# 61 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
static void HplAtm1281Timer1P__Timer__overflow(void );
# 70 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer1P.nc"
static inline uint16_t HplAtm1281Timer1P__Timer__get(void );


static inline void HplAtm1281Timer1P__Timer__set(uint16_t t);








static inline void HplAtm1281Timer1P__Timer__setScale(uint8_t s);










static inline uint8_t HplAtm1281Timer1P__TimerCtrl__getControlB(void );
#line 106
static inline void HplAtm1281Timer1P__TimerCtrl__setControlB(uint8_t x);
#line 125
static inline uint8_t HplAtm1281Timer1P__TimerCtrl__getInterruptFlag(void );









static inline void HplAtm1281Timer1P__Capture__setEdge(bool up);



static inline void HplAtm1281Timer1P__Capture__reset(void );
static inline void HplAtm1281Timer1P__CompareA__reset(void );



static inline void HplAtm1281Timer1P__Timer__start(void );
static inline void HplAtm1281Timer1P__Capture__start(void );
static inline void HplAtm1281Timer1P__CompareA__start(void );




static inline void HplAtm1281Timer1P__Capture__stop(void );
static inline void HplAtm1281Timer1P__CompareA__stop(void );



static inline bool HplAtm1281Timer1P__Timer__test(void );
#line 194
static inline void HplAtm1281Timer1P__CompareA__set(uint16_t t);




static inline uint16_t HplAtm1281Timer1P__Capture__get(void );






void __vector_17(void ) __attribute((interrupt))   ;


static inline void HplAtm1281Timer1P__CompareB__default__fired(void );
void __vector_18(void ) __attribute((interrupt))   ;


static inline void HplAtm1281Timer1P__CompareC__default__fired(void );
void __vector_19(void ) __attribute((interrupt))   ;



void __vector_16(void ) __attribute((interrupt))   ;



void __vector_20(void ) __attribute((interrupt))   ;
# 95 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
static void /*InitOneP.InitOne*/Atm128TimerInitC__0__Timer__setScale(uint8_t scale);
#line 58
static void /*InitOneP.InitOne*/Atm128TimerInitC__0__Timer__set(/*InitOneP.InitOne*/Atm128TimerInitC__0__Timer__timer_size t);










static void /*InitOneP.InitOne*/Atm128TimerInitC__0__Timer__start(void );
# 42 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128TimerInitC.nc"
static inline error_t /*InitOneP.InitOne*/Atm128TimerInitC__0__Init__init(void );








static inline void /*InitOneP.InitOne*/Atm128TimerInitC__0__Timer__overflow(void );
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__fired(void );
# 53 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
static void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Compare__reset(void );
#line 45
static void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Compare__set(/*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Compare__size_type t);










static void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Compare__start(void );


static void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Compare__stop(void );
# 52 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
static /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Timer__timer_size /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Timer__get(void );
# 53 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128AlarmC.nc"
static inline /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__timer_size /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__getNow(void );
#line 65
static inline void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__stop(void );



static inline void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__start(/*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__timer_size dt);




static void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__startAt(/*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__timer_size t0, /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__timer_size dt);
#line 110
static inline void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Compare__fired(void );






static inline void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Timer__overflow(void );
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*CounterOne16C.NCounter*/Atm128CounterC__0__Counter__overflow(void );
# 78 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
static bool /*CounterOne16C.NCounter*/Atm128CounterC__0__Timer__test(void );
#line 52
static /*CounterOne16C.NCounter*/Atm128CounterC__0__Timer__timer_size /*CounterOne16C.NCounter*/Atm128CounterC__0__Timer__get(void );
# 41 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128CounterC.nc"
static inline /*CounterOne16C.NCounter*/Atm128CounterC__0__timer_size /*CounterOne16C.NCounter*/Atm128CounterC__0__Counter__get(void );




static inline bool /*CounterOne16C.NCounter*/Atm128CounterC__0__Counter__isOverflowPending(void );









static inline void /*CounterOne16C.NCounter*/Atm128CounterC__0__Timer__overflow(void );
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__CounterFrom__size_type /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__CounterFrom__get(void );






static bool /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__CounterFrom__isOverflowPending(void );










static void /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__Counter__overflow(void );
# 56 "/opt/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
/*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__upper_count_type /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__m_upper;

enum /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0____nesc_unnamed4378 {

  TransformCounterC__0__LOW_SHIFT_RIGHT = 0, 
  TransformCounterC__0__HIGH_SHIFT_LEFT = 8 * sizeof(/*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__from_size_type ) - /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__LOW_SHIFT_RIGHT, 
  TransformCounterC__0__NUM_UPPER_BITS = 8 * sizeof(/*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__to_size_type ) - 8 * sizeof(/*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__from_size_type ) + 0, 



  TransformCounterC__0__OVERFLOW_MASK = /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__NUM_UPPER_BITS ? ((/*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__upper_count_type )2 << (/*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__to_size_type /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__Counter__get(void );
#line 122
static inline void /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__CounterFrom__overflow(void );
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__size_type /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__get(void );
# 42 "/opt/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline uint32_t /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__get(void );




static inline void /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void );
# 35 "/opt/tinyos-2.x/tos/platforms/micaz/BusyWaitMicroC.nc"
static __inline void BusyWaitMicroC__BusyWait__wait(uint16_t dt);
# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__send(
#line 56
message_t * msg, 







uint8_t len);
# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(
# 36 "/opt/tinyos-2.x/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0xf6caf148, 
# 92 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(
# 37 "/opt/tinyos-2.x/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0xf6cafb08, 
# 60 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 49 "/opt/tinyos-2.x/tos/lib/serial/SerialActiveMessageP.nc"
static inline serial_header_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(message_t * msg);







static inline error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(am_id_t id, am_addr_t dest, 
message_t *msg, 
uint8_t len);
#line 90
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(message_t *msg, error_t result);
#line 102
static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(message_t *msg, void *payload, uint8_t len);








static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__payloadLength(message_t *msg);








static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength(void );
#line 137
static inline am_addr_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__destination(message_t *amsg);
#line 152
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setSource(message_t *amsg, am_addr_t addr);








static inline am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(message_t *amsg);
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static void SerialP__SplitControl__startDone(error_t error);
#line 117
static void SerialP__SplitControl__stopDone(error_t error);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t SerialP__stopDoneTask__postTask(void );
# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t SerialP__SerialControl__start(void );









static error_t SerialP__SerialControl__stop(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t SerialP__RunTx__postTask(void );
# 38 "/opt/tinyos-2.x/tos/lib/serial/SerialFlush.nc"
static void SerialP__SerialFlush__flush(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t SerialP__startDoneTask__postTask(void );
# 45 "/opt/tinyos-2.x/tos/lib/serial/SerialFrameComm.nc"
static error_t SerialP__SerialFrameComm__putDelimiter(void );
#line 68
static void SerialP__SerialFrameComm__resetReceive(void );
#line 54
static error_t SerialP__SerialFrameComm__putData(uint8_t data);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t SerialP__defaultSerialFlushTask__postTask(void );
# 70 "/opt/tinyos-2.x/tos/lib/serial/SendBytePacket.nc"
static uint8_t SerialP__SendBytePacket__nextByte(void );









static void SerialP__SendBytePacket__sendCompleted(error_t error);
# 51 "/opt/tinyos-2.x/tos/lib/serial/ReceiveBytePacket.nc"
static error_t SerialP__ReceiveBytePacket__startPacket(void );






static void SerialP__ReceiveBytePacket__byteReceived(uint8_t data);










static void SerialP__ReceiveBytePacket__endPacket(error_t result);
# 189 "/opt/tinyos-2.x/tos/lib/serial/SerialP.nc"
enum SerialP____nesc_unnamed4379 {
#line 189
  SerialP__RunTx = 12U
};
#line 189
typedef int SerialP____nesc_sillytask_RunTx[SerialP__RunTx];
#line 320
enum SerialP____nesc_unnamed4380 {
#line 320
  SerialP__startDoneTask = 13U
};
#line 320
typedef int SerialP____nesc_sillytask_startDoneTask[SerialP__startDoneTask];





enum SerialP____nesc_unnamed4381 {
#line 326
  SerialP__stopDoneTask = 14U
};
#line 326
typedef int SerialP____nesc_sillytask_stopDoneTask[SerialP__stopDoneTask];








enum SerialP____nesc_unnamed4382 {
#line 335
  SerialP__defaultSerialFlushTask = 15U
};
#line 335
typedef int SerialP____nesc_sillytask_defaultSerialFlushTask[SerialP__defaultSerialFlushTask];
#line 79
enum SerialP____nesc_unnamed4383 {
  SerialP__RX_DATA_BUFFER_SIZE = 2, 
  SerialP__TX_DATA_BUFFER_SIZE = 4, 
  SerialP__SERIAL_MTU = 255, 
  SerialP__SERIAL_VERSION = 1, 
  SerialP__ACK_QUEUE_SIZE = 5
};

enum SerialP____nesc_unnamed4384 {
  SerialP__RXSTATE_NOSYNC, 
  SerialP__RXSTATE_PROTO, 
  SerialP__RXSTATE_TOKEN, 
  SerialP__RXSTATE_INFO, 
  SerialP__RXSTATE_INACTIVE
};

enum SerialP____nesc_unnamed4385 {
  SerialP__TXSTATE_IDLE, 
  SerialP__TXSTATE_PROTO, 
  SerialP__TXSTATE_SEQNO, 
  SerialP__TXSTATE_INFO, 
  SerialP__TXSTATE_FCS1, 
  SerialP__TXSTATE_FCS2, 
  SerialP__TXSTATE_ENDFLAG, 
  SerialP__TXSTATE_ENDWAIT, 
  SerialP__TXSTATE_FINISH, 
  SerialP__TXSTATE_ERROR, 
  SerialP__TXSTATE_INACTIVE
};





#line 109
typedef enum SerialP____nesc_unnamed4386 {
  SerialP__BUFFER_AVAILABLE, 
  SerialP__BUFFER_FILLING, 
  SerialP__BUFFER_COMPLETE
} SerialP__tx_data_buffer_states_t;

enum SerialP____nesc_unnamed4387 {
  SerialP__TX_ACK_INDEX = 0, 
  SerialP__TX_DATA_INDEX = 1, 
  SerialP__TX_BUFFER_COUNT = 2
};






#line 122
typedef struct SerialP____nesc_unnamed4388 {
  uint8_t writePtr;
  uint8_t readPtr;
  uint8_t buf[SerialP__RX_DATA_BUFFER_SIZE + 1];
} SerialP__rx_buf_t;




#line 128
typedef struct SerialP____nesc_unnamed4389 {
  uint8_t state;
  uint8_t buf;
} SerialP__tx_buf_t;





#line 133
typedef struct SerialP____nesc_unnamed4390 {
  uint8_t writePtr;
  uint8_t readPtr;
  uint8_t buf[SerialP__ACK_QUEUE_SIZE + 1];
} SerialP__ack_queue_t;



SerialP__rx_buf_t SerialP__rxBuf;
SerialP__tx_buf_t SerialP__txBuf[SerialP__TX_BUFFER_COUNT];



uint8_t SerialP__rxState;
uint8_t SerialP__rxByteCnt;
uint8_t SerialP__rxProto;
uint8_t SerialP__rxSeqno;
uint16_t SerialP__rxCRC;



uint8_t SerialP__txState;
uint8_t SerialP__txByteCnt;
uint8_t SerialP__txProto;
uint8_t SerialP__txSeqno;
uint16_t SerialP__txCRC;
uint8_t SerialP__txPending;
uint8_t SerialP__txIndex;


SerialP__ack_queue_t SerialP__ackQ;

bool SerialP__offPending = FALSE;



static __inline void SerialP__txInit(void );
static __inline void SerialP__rxInit(void );
static __inline void SerialP__ackInit(void );

static __inline bool SerialP__ack_queue_is_full(void );
static __inline bool SerialP__ack_queue_is_empty(void );
static __inline void SerialP__ack_queue_push(uint8_t token);
static __inline uint8_t SerialP__ack_queue_top(void );
static inline uint8_t SerialP__ack_queue_pop(void );




static __inline void SerialP__rx_buffer_push(uint8_t data);
static __inline uint8_t SerialP__rx_buffer_top(void );
static __inline uint8_t SerialP__rx_buffer_pop(void );
static __inline uint16_t SerialP__rx_current_crc(void );

static void SerialP__rx_state_machine(bool isDelimeter, uint8_t data);
static void SerialP__MaybeScheduleTx(void );




static __inline void SerialP__txInit(void );
#line 205
static __inline void SerialP__rxInit(void );








static __inline void SerialP__ackInit(void );



static inline error_t SerialP__Init__init(void );
#line 232
static __inline bool SerialP__ack_queue_is_full(void );









static __inline bool SerialP__ack_queue_is_empty(void );





static __inline void SerialP__ack_queue_push(uint8_t token);









static __inline uint8_t SerialP__ack_queue_top(void );









static inline uint8_t SerialP__ack_queue_pop(void );
#line 295
static __inline void SerialP__rx_buffer_push(uint8_t data);



static __inline uint8_t SerialP__rx_buffer_top(void );



static __inline uint8_t SerialP__rx_buffer_pop(void );





static __inline uint16_t SerialP__rx_current_crc(void );










static inline void SerialP__startDoneTask__runTask(void );





static inline void SerialP__stopDoneTask__runTask(void );



static inline void SerialP__SerialFlush__flushDone(void );




static inline void SerialP__defaultSerialFlushTask__runTask(void );


static inline void SerialP__SerialFlush__default__flush(void );



static inline error_t SerialP__SplitControl__start(void );




static void SerialP__testOff(void );
#line 364
static inline error_t SerialP__SplitControl__stop(void );
#line 384
static inline void SerialP__SerialFrameComm__delimiterReceived(void );


static inline void SerialP__SerialFrameComm__dataReceived(uint8_t data);



static inline bool SerialP__valid_rx_proto(uint8_t proto);










static void SerialP__rx_state_machine(bool isDelimeter, uint8_t data);
#line 502
static void SerialP__MaybeScheduleTx(void );










static inline error_t SerialP__SendBytePacket__completeSend(void );








static inline error_t SerialP__SendBytePacket__startSend(uint8_t b);
#line 539
static inline void SerialP__RunTx__runTask(void );
#line 642
static inline void SerialP__SerialFrameComm__putDone(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__postTask(void );
# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__sendDone(
# 40 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0xf6bea560, 
# 85 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__postTask(void );
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__receive(
# 39 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0xf6bede50, 
# 60 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 31 "/opt/tinyos-2.x/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(
# 43 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0xf6be9088, 
# 31 "/opt/tinyos-2.x/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(
# 43 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0xf6be9088);
# 23 "/opt/tinyos-2.x/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(
# 43 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0xf6be9088, 
# 23 "/opt/tinyos-2.x/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t upperLen);
# 60 "/opt/tinyos-2.x/tos/lib/serial/SendBytePacket.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__completeSend(void );
#line 51
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__startSend(uint8_t first_byte);
# 147 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4391 {
#line 147
  SerialDispatcherP__0__signalSendDone = 16U
};
#line 147
typedef int /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_sillytask_signalSendDone[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone];
#line 264
enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4392 {
#line 264
  SerialDispatcherP__0__receiveTask = 17U
};
#line 264
typedef int /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_sillytask_receiveTask[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask];
#line 55
#line 51
typedef enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4393 {
  SerialDispatcherP__0__SEND_STATE_IDLE = 0, 
  SerialDispatcherP__0__SEND_STATE_BEGIN = 1, 
  SerialDispatcherP__0__SEND_STATE_DATA = 2
} /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__send_state_t;

enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4394 {
  SerialDispatcherP__0__RECV_STATE_IDLE = 0, 
  SerialDispatcherP__0__RECV_STATE_BEGIN = 1, 
  SerialDispatcherP__0__RECV_STATE_DATA = 2
};






#line 63
typedef struct /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4395 {
  uint8_t which : 1;
  uint8_t bufZeroLocked : 1;
  uint8_t bufOneLocked : 1;
  uint8_t state : 2;
} /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recv_state_t;



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recv_state_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState = { 0, 0, 0, /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_IDLE };
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType = TOS_SERIAL_UNKNOWN_ID;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex = 0;


message_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[2];
message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messagePtrs[2] = { &/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[0], &/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[1] };




uint8_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer = (uint8_t * )&/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[0];

uint8_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer = (void *)0;
/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__send_state_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendLen = 0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex = 0;
error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError = SUCCESS;
bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendCancelled = FALSE;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendId = 0;


uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending = FALSE;
uart_id_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskType = 0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskWhich;
message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskBuf = (void *)0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskSize = 0;

static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(uint8_t id, message_t *msg, uint8_t len);
#line 147
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask(void );
#line 167
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte(void );
#line 183
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error_t error);




static inline bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__isCurrentBufferLocked(void );



static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__lockCurrentBuffer(void );








static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(uint8_t which);








static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBufferSwap(void );




static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket(void );
#line 233
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(uint8_t b);
#line 264
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask(void );
#line 285
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(error_t result);
#line 347
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(uart_id_t id);


static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(uart_id_t id, message_t *msg, 
uint8_t upperLen);


static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(uart_id_t id, message_t *msg, 
uint8_t dataLinkLen);




static inline message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(uart_id_t idxxx, message_t *msg, 
void *payload, 
uint8_t len);


static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(uart_id_t idxxx, message_t *msg, error_t error);
# 48 "/opt/tinyos-2.x/tos/interfaces/UartStream.nc"
static error_t HdlcTranslateC__UartStream__send(
#line 44
uint8_t * buf, 



uint16_t len);
# 83 "/opt/tinyos-2.x/tos/lib/serial/SerialFrameComm.nc"
static void HdlcTranslateC__SerialFrameComm__dataReceived(uint8_t data);





static void HdlcTranslateC__SerialFrameComm__putDone(void );
#line 74
static void HdlcTranslateC__SerialFrameComm__delimiterReceived(void );
# 47 "/opt/tinyos-2.x/tos/lib/serial/HdlcTranslateC.nc"
#line 44
typedef struct HdlcTranslateC____nesc_unnamed4396 {
  uint8_t sendEscape : 1;
  uint8_t receiveEscape : 1;
} HdlcTranslateC__HdlcState;


HdlcTranslateC__HdlcState HdlcTranslateC__state = { 0, 0 };
uint8_t HdlcTranslateC__txTemp;
uint8_t HdlcTranslateC__m_data;


static inline void HdlcTranslateC__SerialFrameComm__resetReceive(void );





static inline void HdlcTranslateC__UartStream__receivedByte(uint8_t data);
#line 86
static error_t HdlcTranslateC__SerialFrameComm__putDelimiter(void );





static error_t HdlcTranslateC__SerialFrameComm__putData(uint8_t data);
#line 104
static inline void HdlcTranslateC__UartStream__sendDone(uint8_t *buf, uint16_t len, 
error_t error);










static inline void HdlcTranslateC__UartStream__receiveDone(uint8_t *buf, uint16_t len, error_t error);
# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUartTxControl__start(void );









static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUartTxControl__stop(void );
# 79 "/opt/tinyos-2.x/tos/interfaces/UartStream.nc"
static void /*Atm128Uart0C.UartP*/Atm128UartP__0__UartStream__receivedByte(uint8_t byte);
#line 99
static void /*Atm128Uart0C.UartP*/Atm128UartP__0__UartStream__receiveDone(
#line 95
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void /*Atm128Uart0C.UartP*/Atm128UartP__0__UartStream__sendDone(
#line 53
uint8_t * buf, 



uint16_t len, error_t error);
# 42 "/opt/tinyos-2.x/tos/chips/atm128/HplAtm128Uart.nc"
static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__enableRxIntr(void );
#line 40
static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__enableTxIntr(void );
static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__disableTxIntr(void );

static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__disableRxIntr(void );


static void /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__tx(uint8_t data);
# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUartRxControl__start(void );









static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUartRxControl__stop(void );
# 62 "/opt/tinyos-2.x/tos/chips/atm128/Atm128UartP.nc"
uint16_t /*Atm128Uart0C.UartP*/Atm128UartP__0__m_tx_len;
#line 62
uint16_t /*Atm128Uart0C.UartP*/Atm128UartP__0__m_rx_len;
uint8_t * /*Atm128Uart0C.UartP*/Atm128UartP__0__m_tx_buf;
#line 63
uint8_t * /*Atm128Uart0C.UartP*/Atm128UartP__0__m_rx_buf;
uint16_t /*Atm128Uart0C.UartP*/Atm128UartP__0__m_tx_pos;
#line 64
uint16_t /*Atm128Uart0C.UartP*/Atm128UartP__0__m_rx_pos;
uint16_t /*Atm128Uart0C.UartP*/Atm128UartP__0__m_byte_time;
uint8_t /*Atm128Uart0C.UartP*/Atm128UartP__0__m_rx_intr;
uint8_t /*Atm128Uart0C.UartP*/Atm128UartP__0__m_tx_intr;

static inline error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__Init__init(void );







static inline error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__StdControl__start(void );
#line 93
static inline error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__StdControl__stop(void );
#line 133
static inline void /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__rxDone(uint8_t data);
#line 155
static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__UartStream__send(uint8_t *buf, uint16_t len);
#line 173
static inline void /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__txDone(void );
#line 216
static inline void /*Atm128Uart0C.UartP*/Atm128UartP__0__Counter__overflow(void );
# 49 "/opt/tinyos-2.x/tos/chips/atm128/HplAtm128Uart.nc"
static void HplAtm128UartP__HplUart0__rxDone(uint8_t data);
#line 47
static void HplAtm128UartP__HplUart0__txDone(void );

static void HplAtm128UartP__HplUart1__rxDone(uint8_t data);
#line 47
static void HplAtm128UartP__HplUart1__txDone(void );
# 60 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128Calibrate.nc"
static uint16_t HplAtm128UartP__Atm128Calibrate__baudrateRegister(uint32_t baudrate);
# 44 "/opt/tinyos-2.x/tos/interfaces/McuPowerState.nc"
static void HplAtm128UartP__McuPowerState__update(void );
# 110 "/opt/tinyos-2.x/tos/chips/atm1281/HplAtm128UartP.nc"
static inline error_t HplAtm128UartP__Uart0Init__init(void );
#line 130
static inline error_t HplAtm128UartP__Uart0TxControl__start(void );





static inline error_t HplAtm128UartP__Uart0TxControl__stop(void );





static inline error_t HplAtm128UartP__Uart0RxControl__start(void );





static inline error_t HplAtm128UartP__Uart0RxControl__stop(void );





static inline error_t HplAtm128UartP__HplUart0__enableTxIntr(void );





static inline error_t HplAtm128UartP__HplUart0__disableTxIntr(void );




static inline error_t HplAtm128UartP__HplUart0__enableRxIntr(void );




static inline error_t HplAtm128UartP__HplUart0__disableRxIntr(void );
#line 187
static void HplAtm128UartP__HplUart0__tx(uint8_t data);






void __vector_25(void ) __attribute((signal))   ;





void __vector_27(void ) __attribute((interrupt))   ;



static inline error_t HplAtm128UartP__Uart1Init__init(void );
#line 288
void __vector_36(void ) __attribute((signal))   ;




void __vector_38(void ) __attribute((interrupt))   ;





static inline void HplAtm128UartP__HplUart1__default__txDone(void );
static inline void HplAtm128UartP__HplUart1__default__rxDone(uint8_t data);
# 49 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
static void HplAtm1281Timer3P__CompareA__fired(void );
# 51 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Capture.nc"
static void HplAtm1281Timer3P__Capture__captured(HplAtm1281Timer3P__Capture__size_type t);
# 49 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
static void HplAtm1281Timer3P__CompareB__fired(void );
#line 49
static void HplAtm1281Timer3P__CompareC__fired(void );
# 61 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
static void HplAtm1281Timer3P__Timer__overflow(void );
# 70 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer3P.nc"
static inline uint16_t HplAtm1281Timer3P__Timer__get(void );


static inline void HplAtm1281Timer3P__Timer__set(uint16_t t);








static inline void HplAtm1281Timer3P__Timer__setScale(uint8_t s);










static inline uint8_t HplAtm1281Timer3P__TimerCtrl__getControlB(void );
#line 106
static inline void HplAtm1281Timer3P__TimerCtrl__setControlB(uint8_t x);
#line 144
static inline void HplAtm1281Timer3P__Timer__start(void );
#line 205
static inline void HplAtm1281Timer3P__CompareA__default__fired(void );
void __vector_32(void ) __attribute((interrupt))   ;


static inline void HplAtm1281Timer3P__CompareB__default__fired(void );
void __vector_33(void ) __attribute((interrupt))   ;


static inline void HplAtm1281Timer3P__CompareC__default__fired(void );
void __vector_34(void ) __attribute((interrupt))   ;


static inline void HplAtm1281Timer3P__Capture__default__captured(uint16_t time);
void __vector_31(void ) __attribute((interrupt))   ;



void __vector_35(void ) __attribute((interrupt))   ;
# 95 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
static void /*InitThreeP.InitThree*/Atm128TimerInitC__1__Timer__setScale(uint8_t scale);
#line 58
static void /*InitThreeP.InitThree*/Atm128TimerInitC__1__Timer__set(/*InitThreeP.InitThree*/Atm128TimerInitC__1__Timer__timer_size t);










static void /*InitThreeP.InitThree*/Atm128TimerInitC__1__Timer__start(void );
# 42 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128TimerInitC.nc"
static inline error_t /*InitThreeP.InitThree*/Atm128TimerInitC__1__Init__init(void );








static inline void /*InitThreeP.InitThree*/Atm128TimerInitC__1__Timer__overflow(void );
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*CounterThree16C.NCounter*/Atm128CounterC__1__Counter__overflow(void );
# 56 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128CounterC.nc"
static inline void /*CounterThree16C.NCounter*/Atm128CounterC__1__Timer__overflow(void );
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*CounterMicro32C.Transform32*/TransformCounterC__1__Counter__overflow(void );
# 56 "/opt/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
/*CounterMicro32C.Transform32*/TransformCounterC__1__upper_count_type /*CounterMicro32C.Transform32*/TransformCounterC__1__m_upper;

enum /*CounterMicro32C.Transform32*/TransformCounterC__1____nesc_unnamed4397 {

  TransformCounterC__1__LOW_SHIFT_RIGHT = 0, 
  TransformCounterC__1__HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMicro32C.Transform32*/TransformCounterC__1__from_size_type ) - /*CounterMicro32C.Transform32*/TransformCounterC__1__LOW_SHIFT_RIGHT, 
  TransformCounterC__1__NUM_UPPER_BITS = 8 * sizeof(/*CounterMicro32C.Transform32*/TransformCounterC__1__to_size_type ) - 8 * sizeof(/*CounterMicro32C.Transform32*/TransformCounterC__1__from_size_type ) + 0, 



  TransformCounterC__1__OVERFLOW_MASK = /*CounterMicro32C.Transform32*/TransformCounterC__1__NUM_UPPER_BITS ? ((/*CounterMicro32C.Transform32*/TransformCounterC__1__upper_count_type )2 << (/*CounterMicro32C.Transform32*/TransformCounterC__1__NUM_UPPER_BITS - 1)) - 1 : 0
};
#line 122
static inline void /*CounterMicro32C.Transform32*/TransformCounterC__1__CounterFrom__overflow(void );
# 40 "/opt/tinyos-2.x/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__offset(void );


static inline uint8_t SerialPacketInfoActiveMessageP__Info__dataLinkLength(message_t *msg, uint8_t upperLen);


static inline uint8_t SerialPacketInfoActiveMessageP__Info__upperLength(message_t *msg, uint8_t dataLinkLen);
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketField.nc"
static RssiBaseC__PacketRSSI__value_type RssiBaseC__PacketRSSI__get(message_t *msg);
#line 29
static bool RssiBaseC__PacketRSSI__isSet(message_t *msg);
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static error_t RssiBaseC__RadioControl__start(void );
# 72 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
static void RssiBaseC__Leds__led1Toggle(void );
# 67 "RssiBaseC.nc"
bool RssiBaseC__busy = FALSE;


uint8_t RssiBaseC__blinkCnSend = 0;


static void RssiBaseC__sendBlink(void );
static inline uint16_t RssiBaseC__getRssi(message_t *msg);







static inline bool RssiBaseC__RssiMsgIntercept__forward(message_t *msg, void *payload, uint8_t len);
#line 96
static inline bool RssiBaseC__SimpleRssiMsgIntercept__forward(message_t *msg, void *payload, uint8_t len);
#line 117
static void RssiBaseC__sendBlink(void );
#line 131
static inline void RssiBaseC__Boot__booted(void );




static inline void RssiBaseC__SendTimer__fired(void );








static inline void RssiBaseC__RadioControl__startDone(error_t result);
#line 158
static inline void RssiBaseC__RadioControl__stopDone(error_t result);


static inline void RssiBaseC__PingMsgSend__sendDone(message_t *m, error_t error);
#line 199
static inline uint16_t RssiBaseC__getRssi(message_t *msg);
# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static void /*RssiBaseAppC.PingMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 57 "/opt/tinyos-2.x/tos/system/AMQueueEntryP.nc"
static inline void /*RssiBaseAppC.PingMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err);
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(
# 40 "/opt/tinyos-2.x/tos/system/AMQueueImplP.nc"
am_id_t arg_0xf6a50e40, 
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(
# 38 "/opt/tinyos-2.x/tos/system/AMQueueImplP.nc"
uint8_t arg_0xf6a50458, 
# 85 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 67 "/opt/tinyos-2.x/tos/interfaces/Packet.nc"
static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(
#line 63
message_t * msg);
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
enum /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4398 {
#line 118
  AMQueueImplP__0__CancelTask = 18U
};
#line 118
typedef int /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_CancelTask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask];
#line 161
enum /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4399 {
#line 161
  AMQueueImplP__0__errorTask = 19U
};
#line 161
typedef int /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_errorTask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask];
#line 49
#line 47
typedef struct /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4400 {
  message_t * msg;
} /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t;

uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[1];
uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[1 / 8 + 1];

static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void );

static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket(void );
#line 118
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void );
#line 155
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(uint8_t last, message_t * msg, error_t err);





static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void );




static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void );
#line 181
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(am_id_t id, message_t *msg, error_t err);
#line 207
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err);
# 109 "/opt/tinyos-2.x/tos/chips/atm1281/atm128hardware.h"
static __inline  void __nesc_disable_interrupt()
#line 109
{
   __asm volatile ("cli" :  :  : "memory");}

#line 126
#line 125
__inline   __nesc_atomic_t 
__nesc_atomic_start(void )
{
  __nesc_atomic_t result = * (volatile uint8_t *)(0x3F + 0x20);

#line 129
  __nesc_disable_interrupt();
   __asm volatile ("" :  :  : "memory");
  return result;
}



#line 135
__inline   void 
__nesc_atomic_end(__nesc_atomic_t original_SREG)
{
   __asm volatile ("" :  :  : "memory");
  * (volatile uint8_t *)(0x3F + 0x20) = original_SREG;
}

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
# 58 "/opt/tinyos-2.x/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 42 "/opt/tinyos-2.x/tos/platforms/mica/PlatformP.nc"
static inline void PlatformP__power_init(void )
#line 42
{
  /* atomic removed: atomic calls only */
#line 43
  {
    * (volatile uint8_t *)(0X35 + 0x20) = 1 << 0;
  }
}

# 106 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer1P.nc"
static inline void HplAtm1281Timer1P__TimerCtrl__setControlB(uint8_t x)
#line 106
{
  * (volatile uint8_t *)0x81 = x;
}

#line 93
static inline uint8_t HplAtm1281Timer1P__TimerCtrl__getControlB(void )
#line 93
{
  return * (volatile uint8_t *)0x81;
}

#line 82
static inline void HplAtm1281Timer1P__Timer__setScale(uint8_t s)
#line 82
{
  Atm128_TCCRB_t x = (Atm128_TCCRB_t )HplAtm1281Timer1P__TimerCtrl__getControlB();

#line 84
  x.bits.cs = s;
  HplAtm1281Timer1P__TimerCtrl__setControlB(x.flat);
}

# 95 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
inline static void /*InitOneP.InitOne*/Atm128TimerInitC__0__Timer__setScale(uint8_t scale){
#line 95
  HplAtm1281Timer1P__Timer__setScale(scale);
#line 95
}
#line 95
# 144 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer1P.nc"
static inline void HplAtm1281Timer1P__Timer__start(void )
#line 144
{
#line 144
  * (volatile uint8_t *)0x6F |= 1 << 0;
}

# 69 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
inline static void /*InitOneP.InitOne*/Atm128TimerInitC__0__Timer__start(void ){
#line 69
  HplAtm1281Timer1P__Timer__start();
#line 69
}
#line 69
# 73 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer1P.nc"
static inline void HplAtm1281Timer1P__Timer__set(uint16_t t)
#line 73
{
#line 73
  * (volatile uint16_t *)0x84 = t;
}

# 58 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
inline static void /*InitOneP.InitOne*/Atm128TimerInitC__0__Timer__set(/*InitOneP.InitOne*/Atm128TimerInitC__0__Timer__timer_size t){
#line 58
  HplAtm1281Timer1P__Timer__set(t);
#line 58
}
#line 58
# 42 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128TimerInitC.nc"
static inline error_t /*InitOneP.InitOne*/Atm128TimerInitC__0__Init__init(void )
#line 42
{
  /* atomic removed: atomic calls only */
#line 43
  {
    /*InitOneP.InitOne*/Atm128TimerInitC__0__Timer__set(0);
    /*InitOneP.InitOne*/Atm128TimerInitC__0__Timer__start();
    /*InitOneP.InitOne*/Atm128TimerInitC__0__Timer__setScale(2);
  }
  return SUCCESS;
}

# 106 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer3P.nc"
static inline void HplAtm1281Timer3P__TimerCtrl__setControlB(uint8_t x)
#line 106
{
  * (volatile uint8_t *)0x91 = x;
}

#line 93
static inline uint8_t HplAtm1281Timer3P__TimerCtrl__getControlB(void )
#line 93
{
  return * (volatile uint8_t *)0x91;
}

#line 82
static inline void HplAtm1281Timer3P__Timer__setScale(uint8_t s)
#line 82
{
  Atm128_TCCRB_t x = (Atm128_TCCRB_t )HplAtm1281Timer3P__TimerCtrl__getControlB();

#line 84
  x.bits.cs = s;
  HplAtm1281Timer3P__TimerCtrl__setControlB(x.flat);
}

# 95 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
inline static void /*InitThreeP.InitThree*/Atm128TimerInitC__1__Timer__setScale(uint8_t scale){
#line 95
  HplAtm1281Timer3P__Timer__setScale(scale);
#line 95
}
#line 95
# 144 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer3P.nc"
static inline void HplAtm1281Timer3P__Timer__start(void )
#line 144
{
#line 144
  * (volatile uint8_t *)0x71 |= 1 << 0;
}

# 69 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
inline static void /*InitThreeP.InitThree*/Atm128TimerInitC__1__Timer__start(void ){
#line 69
  HplAtm1281Timer3P__Timer__start();
#line 69
}
#line 69
# 73 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer3P.nc"
static inline void HplAtm1281Timer3P__Timer__set(uint16_t t)
#line 73
{
#line 73
  * (volatile uint16_t *)0x94 = t;
}

# 58 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
inline static void /*InitThreeP.InitThree*/Atm128TimerInitC__1__Timer__set(/*InitThreeP.InitThree*/Atm128TimerInitC__1__Timer__timer_size t){
#line 58
  HplAtm1281Timer3P__Timer__set(t);
#line 58
}
#line 58
# 42 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128TimerInitC.nc"
static inline error_t /*InitThreeP.InitThree*/Atm128TimerInitC__1__Init__init(void )
#line 42
{
  /* atomic removed: atomic calls only */
#line 43
  {
    /*InitThreeP.InitThree*/Atm128TimerInitC__1__Timer__set(0);
    /*InitThreeP.InitThree*/Atm128TimerInitC__1__Timer__start();
    /*InitThreeP.InitThree*/Atm128TimerInitC__1__Timer__setScale(2);
  }
  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t MotePlatformP__SubInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = /*InitThreeP.InitThree*/Atm128TimerInitC__1__Init__init();
#line 51
  __nesc_result = ecombine(__nesc_result, /*InitOneP.InitOne*/Atm128TimerInitC__0__Init__init());
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 47 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortA.Bit4*/HplAtm128GeneralIOPinP__4__IO__clr(void )
#line 47
{
#line 47
  * (volatile uint8_t * )34U &= ~(1 << 4);
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void MotePlatformP__SerialIdPin__clr(void ){
#line 30
  /*HplAtm128GeneralIOC.PortA.Bit4*/HplAtm128GeneralIOPinP__4__IO__clr();
#line 30
}
#line 30
# 50 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortA.Bit4*/HplAtm128GeneralIOPinP__4__IO__makeInput(void )
#line 50
{
#line 50
  * (volatile uint8_t * )33U &= ~(1 << 4);
}

# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void MotePlatformP__SerialIdPin__makeInput(void ){
#line 33
  /*HplAtm128GeneralIOC.PortA.Bit4*/HplAtm128GeneralIOPinP__4__IO__makeInput();
#line 33
}
#line 33
# 26 "/opt/tinyos-2.x/tos/platforms/micaz/MotePlatformP.nc"
static inline error_t MotePlatformP__PlatformInit__init(void )
#line 26
{

  * (volatile uint8_t *)(0x08 + 0x20) = 0;
  * (volatile uint8_t *)(0x07 + 0x20) = 0xff;


  MotePlatformP__SerialIdPin__makeInput();
  MotePlatformP__SerialIdPin__clr();

  return MotePlatformP__SubInit__init();
}

# 46 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortA.Bit0*/HplAtm128GeneralIOPinP__0__IO__set(void )
#line 46
{
#line 46
  * (volatile uint8_t * )34U |= 1 << 0;
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__set(void ){
#line 29
  /*HplAtm128GeneralIOC.PortA.Bit0*/HplAtm128GeneralIOPinP__0__IO__set();
#line 29
}
#line 29
# 46 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortA.Bit1*/HplAtm128GeneralIOPinP__1__IO__set(void )
#line 46
{
#line 46
  * (volatile uint8_t * )34U |= 1 << 1;
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__set(void ){
#line 29
  /*HplAtm128GeneralIOC.PortA.Bit1*/HplAtm128GeneralIOPinP__1__IO__set();
#line 29
}
#line 29
# 46 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortA.Bit2*/HplAtm128GeneralIOPinP__2__IO__set(void )
#line 46
{
#line 46
  * (volatile uint8_t * )34U |= 1 << 2;
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__set(void ){
#line 29
  /*HplAtm128GeneralIOC.PortA.Bit2*/HplAtm128GeneralIOPinP__2__IO__set();
#line 29
}
#line 29
# 52 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortA.Bit0*/HplAtm128GeneralIOPinP__0__IO__makeOutput(void )
#line 52
{
#line 52
  * (volatile uint8_t * )33U |= 1 << 0;
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__makeOutput(void ){
#line 35
  /*HplAtm128GeneralIOC.PortA.Bit0*/HplAtm128GeneralIOPinP__0__IO__makeOutput();
#line 35
}
#line 35
# 52 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortA.Bit1*/HplAtm128GeneralIOPinP__1__IO__makeOutput(void )
#line 52
{
#line 52
  * (volatile uint8_t * )33U |= 1 << 1;
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__makeOutput(void ){
#line 35
  /*HplAtm128GeneralIOC.PortA.Bit1*/HplAtm128GeneralIOPinP__1__IO__makeOutput();
#line 35
}
#line 35
# 52 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortA.Bit2*/HplAtm128GeneralIOPinP__2__IO__makeOutput(void )
#line 52
{
#line 52
  * (volatile uint8_t * )33U |= 1 << 2;
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__makeOutput(void ){
#line 35
  /*HplAtm128GeneralIOC.PortA.Bit2*/HplAtm128GeneralIOPinP__2__IO__makeOutput();
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
inline static error_t PlatformP__MoteInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = LedsP__Init__init();
#line 51
  __nesc_result = ecombine(__nesc_result, MotePlatformP__PlatformInit__init());
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 64 "/opt/tinyos-2.x/tos/platforms/iris/MeasureClockC.nc"
static inline error_t MeasureClockC__Init__init(void )
#line 64
{
  /* atomic removed: atomic calls only */



  {
    uint8_t now;
#line 70
    uint8_t wraps;
    uint16_t start;


    * (volatile uint8_t *)0x81 = 1 << 0;
    * (volatile uint8_t *)0xB6 = 1 << 5;
    * (volatile uint8_t *)0xB1 = (1 << 1) | (1 << 0);




    start = * (volatile uint16_t *)0x84;
    for (wraps = MeasureClockC__MAGIC / 2; wraps; ) 
      {
        uint16_t next = * (volatile uint16_t *)0x84;

        if (next < start) {
          wraps--;
          }
#line 88
        start = next;
      }


    now = * (volatile uint8_t *)0xB2;
    while (* (volatile uint8_t *)0xB2 == now) ;


    start = * (volatile uint16_t *)0x84;
    now = * (volatile uint8_t *)0xB2;
    while (* (volatile uint8_t *)0xB2 == now) ;
    MeasureClockC__cycles = * (volatile uint16_t *)0x84;

    MeasureClockC__cycles = (MeasureClockC__cycles - start + 16) >> 5;


    * (volatile uint8_t *)0xB6 = * (volatile uint8_t *)0x81 = * (volatile uint8_t *)0xB1 = 0;
    * (volatile uint8_t *)0xB2 = 0;
    * (volatile uint16_t *)0x84 = 0;
    * (volatile uint8_t *)(0x16 + 0x20) = * (volatile uint8_t *)(0x17 + 0x20) = 0xff;
    while (* (volatile uint8_t *)0xB6 & (((1 << 4) | (1 << 2)) | (1 << 0))) 
      ;
  }
  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t PlatformP__MeasureClock__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = MeasureClockC__Init__init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 49 "/opt/tinyos-2.x/tos/platforms/mica/PlatformP.nc"
static inline error_t PlatformP__Init__init(void )
{
  error_t ok;


  ok = PlatformP__MeasureClock__init();
  ok = ecombine(ok, PlatformP__MoteInit__init());

  if (ok != SUCCESS) {
    return ok;
    }
  PlatformP__power_init();

  return SUCCESS;
}

# 46 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortA.Bit6*/HplAtm128GeneralIOPinP__6__IO__set(void )
#line 46
{
#line 46
  * (volatile uint8_t * )34U |= 1 << 6;
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void RF230DriverLayerP__RSTN__set(void ){
#line 29
  /*HplAtm128GeneralIOC.PortA.Bit6*/HplAtm128GeneralIOPinP__6__IO__set();
#line 29
}
#line 29
# 52 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortA.Bit6*/HplAtm128GeneralIOPinP__6__IO__makeOutput(void )
#line 52
{
#line 52
  * (volatile uint8_t * )33U |= 1 << 6;
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void RF230DriverLayerP__RSTN__makeOutput(void ){
#line 35
  /*HplAtm128GeneralIOC.PortA.Bit6*/HplAtm128GeneralIOPinP__6__IO__makeOutput();
#line 35
}
#line 35
# 47 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortB.Bit7*/HplAtm128GeneralIOPinP__15__IO__clr(void )
#line 47
{
#line 47
  * (volatile uint8_t * )37U &= ~(1 << 7);
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void RF230DriverLayerP__SLP_TR__clr(void ){
#line 30
  /*HplAtm128GeneralIOC.PortB.Bit7*/HplAtm128GeneralIOPinP__15__IO__clr();
#line 30
}
#line 30
# 52 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortB.Bit7*/HplAtm128GeneralIOPinP__15__IO__makeOutput(void )
#line 52
{
#line 52
  * (volatile uint8_t * )36U |= 1 << 7;
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void RF230DriverLayerP__SLP_TR__makeOutput(void ){
#line 35
  /*HplAtm128GeneralIOC.PortB.Bit7*/HplAtm128GeneralIOPinP__15__IO__makeOutput();
#line 35
}
#line 35
# 46 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortB.Bit0*/HplAtm128GeneralIOPinP__8__IO__set(void )
#line 46
{
#line 46
  * (volatile uint8_t * )37U |= 1 << 0;
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void RF230DriverLayerP__SELN__set(void ){
#line 29
  /*HplAtm128GeneralIOC.PortB.Bit0*/HplAtm128GeneralIOPinP__8__IO__set();
#line 29
}
#line 29
# 52 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortB.Bit0*/HplAtm128GeneralIOPinP__8__IO__makeOutput(void )
#line 52
{
#line 52
  * (volatile uint8_t * )36U |= 1 << 0;
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void RF230DriverLayerP__SELN__makeOutput(void ){
#line 35
  /*HplAtm128GeneralIOC.PortB.Bit0*/HplAtm128GeneralIOPinP__8__IO__makeOutput();
#line 35
}
#line 35
# 206 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static inline error_t RF230DriverLayerP__PlatformInit__init(void )
{
  RF230DriverLayerP__SELN__makeOutput();
  RF230DriverLayerP__SELN__set();
  RF230DriverLayerP__SLP_TR__makeOutput();
  RF230DriverLayerP__SLP_TR__clr();
  RF230DriverLayerP__RSTN__makeOutput();
  RF230DriverLayerP__RSTN__set();

  RF230DriverLayerP__rxMsg = &RF230DriverLayerP__rxMsgBuffer;


  RF230DriverLayerP__rssiClear = 0;
  RF230DriverLayerP__rssiBusy = 90;

  return SUCCESS;
}

# 151 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer1P.nc"
static inline void HplAtm1281Timer1P__Capture__stop(void )
#line 151
{
#line 151
  * (volatile uint8_t *)0x6F &= ~(1 << 5);
}

# 61 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Capture.nc"
inline static void HplRF230P__Capture__stop(void ){
#line 61
  HplAtm1281Timer1P__Capture__stop();
#line 61
}
#line 61
# 47 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortD.Bit4*/HplAtm128GeneralIOPinP__28__IO__clr(void )
#line 47
{
#line 47
  * (volatile uint8_t * )43U &= ~(1 << 4);
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void HplRF230P__PortIRQ__clr(void ){
#line 30
  /*HplAtm128GeneralIOC.PortD.Bit4*/HplAtm128GeneralIOPinP__28__IO__clr();
#line 30
}
#line 30
# 50 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortD.Bit4*/HplAtm128GeneralIOPinP__28__IO__makeInput(void )
#line 50
{
#line 50
  * (volatile uint8_t * )42U &= ~(1 << 4);
}

# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void HplRF230P__PortIRQ__makeInput(void ){
#line 33
  /*HplAtm128GeneralIOC.PortD.Bit4*/HplAtm128GeneralIOPinP__28__IO__makeInput();
#line 33
}
#line 33
# 47 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortD.Bit6*/HplAtm128GeneralIOPinP__30__IO__clr(void )
#line 47
{
#line 47
  * (volatile uint8_t * )43U &= ~(1 << 6);
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void HplRF230P__PortCLKM__clr(void ){
#line 30
  /*HplAtm128GeneralIOC.PortD.Bit6*/HplAtm128GeneralIOPinP__30__IO__clr();
#line 30
}
#line 30
# 50 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortD.Bit6*/HplAtm128GeneralIOPinP__30__IO__makeInput(void )
#line 50
{
#line 50
  * (volatile uint8_t * )42U &= ~(1 << 6);
}

# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void HplRF230P__PortCLKM__makeInput(void ){
#line 33
  /*HplAtm128GeneralIOC.PortD.Bit6*/HplAtm128GeneralIOPinP__30__IO__makeInput();
#line 33
}
#line 33
# 42 "/opt/tinyos-2.x/tos/platforms/iris/chips/rf230/HplRF230P.nc"
static inline error_t HplRF230P__PlatformInit__init(void )
{
  HplRF230P__PortCLKM__makeInput();
  HplRF230P__PortCLKM__clr();
  HplRF230P__PortIRQ__makeInput();
  HplRF230P__PortIRQ__clr();
  HplRF230P__Capture__stop();

  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t RealMainP__PlatformInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = HplRF230P__PlatformInit__init();
#line 51
  __nesc_result = ecombine(__nesc_result, RF230DriverLayerP__PlatformInit__init());
#line 51
  __nesc_result = ecombine(__nesc_result, PlatformP__Init__init());
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
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
# 161 "RssiBaseC.nc"
static inline void RssiBaseC__PingMsgSend__sendDone(message_t *m, error_t error)
#line 161
{
  RssiBaseC__busy = FALSE;
}

# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static void /*RssiBaseAppC.PingMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(message_t * msg, error_t error){
#line 99
  RssiBaseC__PingMsgSend__sendDone(msg, error);
#line 99
}
#line 99
# 57 "/opt/tinyos-2.x/tos/system/AMQueueEntryP.nc"
static inline void /*RssiBaseAppC.PingMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err)
#line 57
{
  /*RssiBaseAppC.PingMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(m, err);
}

# 207 "/opt/tinyos-2.x/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err)
#line 207
{
}

# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(uint8_t arg_0xf6a50458, message_t * msg, error_t error){
#line 89
  switch (arg_0xf6a50458) {
#line 89
    case 0U:
#line 89
      /*RssiBaseAppC.PingMsgSender.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(msg, error);
#line 89
      break;
#line 89
    default:
#line 89
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(arg_0xf6a50458, msg, error);
#line 89
      break;
#line 89
    }
#line 89
}
#line 89
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
  for (i = 0; i < 1 / 8 + 1; i++) {
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

#line 161
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void )
#line 161
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current, /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg, FAIL);
}

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
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(am_id_t arg_0xf6a50e40, am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = ActiveMessageLayerP__AMSend__send(arg_0xf6a50e40, addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 246 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_leuint8(const void * source)
#line 246
{
  const uint8_t *base = source;

#line 248
  return base[0];
}

# 62 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline uint8_t RF230RadioP__RF230DriverConfig__headerLength(message_t *msg)
{
  return (unsigned short )& ((message_t *)0)->data - sizeof(rf230packet_header_t );
}

# 29 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverConfig.nc"
inline static uint8_t RF230DriverLayerP__Config__headerLength(message_t *msg){
#line 29
  unsigned char __nesc_result;
#line 29

#line 29
  __nesc_result = RF230RadioP__RF230DriverConfig__headerLength(msg);
#line 29

#line 29
  return __nesc_result;
#line 29
}
#line 29
# 83 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static inline rf230_header_t *RF230DriverLayerP__getHeader(message_t *msg)
{
  return (void *)msg + RF230DriverLayerP__Config__headerLength(msg);
}

#line 867
static inline uint8_t RF230DriverLayerP__RadioPacket__payloadLength(message_t *msg)
{
  return __nesc_ntoh_leuint8(RF230DriverLayerP__getHeader(msg)->length.data) - 2;
}

# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t MetadataFlagsLayerC__SubPacket__payloadLength(message_t *msg){
#line 38
  unsigned char __nesc_result;
#line 38

#line 38
  __nesc_result = RF230DriverLayerP__RadioPacket__payloadLength(msg);
#line 38

#line 38
  return __nesc_result;
#line 38
}
#line 38
# 84 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MetadataFlagsLayerC.nc"
static inline uint8_t MetadataFlagsLayerC__RadioPacket__payloadLength(message_t *msg)
{
  return MetadataFlagsLayerC__SubPacket__payloadLength(msg);
}

# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t TimeStampingLayerP__SubPacket__payloadLength(message_t *msg){
#line 38
  unsigned char __nesc_result;
#line 38

#line 38
  __nesc_result = MetadataFlagsLayerC__RadioPacket__payloadLength(msg);
#line 38

#line 38
  return __nesc_result;
#line 38
}
#line 38
# 110 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TimeStampingLayerP.nc"
static inline uint8_t TimeStampingLayerP__RadioPacket__payloadLength(message_t *msg)
{
  return TimeStampingLayerP__SubPacket__payloadLength(msg);
}

# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t PacketLinkLayerP__SubPacket__payloadLength(message_t *msg){
#line 38
  unsigned char __nesc_result;
#line 38

#line 38
  __nesc_result = TimeStampingLayerP__RadioPacket__payloadLength(msg);
#line 38

#line 38
  return __nesc_result;
#line 38
}
#line 38
# 235 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/PacketLinkLayerP.nc"
static inline uint8_t PacketLinkLayerP__RadioPacket__payloadLength(message_t *msg)
#line 235
{
  return PacketLinkLayerP__SubPacket__payloadLength(msg);
}

# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t Ieee154PacketLayerP__SubPacket__payloadLength(message_t *msg){
#line 38
  unsigned char __nesc_result;
#line 38

#line 38
  __nesc_result = PacketLinkLayerP__RadioPacket__payloadLength(msg);
#line 38

#line 38
  return __nesc_result;
#line 38
}
#line 38
# 260 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
static inline uint8_t Ieee154PacketLayerP__RadioPacket__payloadLength(message_t *msg)
{
  return Ieee154PacketLayerP__SubPacket__payloadLength(msg) - sizeof(ieee154_header_t );
}

# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t TinyosNetworkLayerC__SubPacket__payloadLength(message_t *msg){
#line 38
  unsigned char __nesc_result;
#line 38

#line 38
  __nesc_result = Ieee154PacketLayerP__RadioPacket__payloadLength(msg);
#line 38

#line 38
  return __nesc_result;
#line 38
}
#line 38
# 148 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TinyosNetworkLayerC.nc"
static inline uint8_t TinyosNetworkLayerC__TinyosPacket__payloadLength(message_t *msg)
{
  return TinyosNetworkLayerC__SubPacket__payloadLength(msg) - TinyosNetworkLayerC__PAYLOAD_OFFSET;
}

# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t ActiveMessageLayerP__SubPacket__payloadLength(message_t *msg){
#line 38
  unsigned char __nesc_result;
#line 38

#line 38
  __nesc_result = TinyosNetworkLayerC__TinyosPacket__payloadLength(msg);
#line 38

#line 38
  return __nesc_result;
#line 38
}
#line 38
# 204 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayerP.nc"
static inline uint8_t ActiveMessageLayerP__RadioPacket__payloadLength(message_t *msg)
{
  return ActiveMessageLayerP__SubPacket__payloadLength(msg) - sizeof(activemessage_header_t );
}

#line 236
static inline uint8_t ActiveMessageLayerP__Packet__payloadLength(message_t *msg)
{
  return ActiveMessageLayerP__RadioPacket__payloadLength(msg);
}

# 67 "/opt/tinyos-2.x/tos/interfaces/Packet.nc"
inline static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(message_t * msg){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = ActiveMessageLayerP__Packet__payloadLength(msg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 276 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_leuint16(const void * source)
#line 276
{
  const uint8_t *base = source;

#line 278
  return ((uint16_t )base[1] << 8) | base[0];
}

# 167 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
static inline uint16_t Ieee154PacketLayerP__Ieee154PacketLayer__getDestAddr(message_t *msg)
{
  return __nesc_ntoh_leuint16(Ieee154PacketLayerP__getHeader(msg)->dest.data);
}

# 130 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayer.nc"
inline static uint16_t RF230RadioP__Ieee154PacketLayer__getDestAddr(message_t *msg){
#line 130
  unsigned short __nesc_result;
#line 130

#line 130
  __nesc_result = Ieee154PacketLayerP__Ieee154PacketLayer__getDestAddr(msg);
#line 130

#line 130
  return __nesc_result;
#line 130
}
#line 130
# 162 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline am_addr_t RF230RadioP__ActiveMessageConfig__destination(message_t *msg)
{
  return RF230RadioP__Ieee154PacketLayer__getDestAddr(msg);
}

# 29 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageConfig.nc"
inline static am_addr_t ActiveMessageLayerP__Config__destination(message_t *msg){
#line 29
  unsigned short __nesc_result;
#line 29

#line 29
  __nesc_result = RF230RadioP__ActiveMessageConfig__destination(msg);
#line 29

#line 29
  return __nesc_result;
#line 29
}
#line 29
# 153 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayerP.nc"
static __inline am_addr_t ActiveMessageLayerP__AMPacket__destination(message_t *msg)
{
  return ActiveMessageLayerP__Config__destination(msg);
}

# 67 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static am_addr_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(message_t * amsg){
#line 67
  unsigned short __nesc_result;
#line 67

#line 67
  __nesc_result = ActiveMessageLayerP__AMPacket__destination(amsg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 235 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_uint8(const void * source)
#line 235
{
  const uint8_t *base = source;

#line 237
  return base[0];
}

# 862 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static inline uint8_t RF230DriverLayerP__RadioPacket__headerLength(message_t *msg)
{
  return RF230DriverLayerP__Config__headerLength(msg) + sizeof(rf230_header_t );
}

# 32 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t MetadataFlagsLayerC__SubPacket__headerLength(message_t *msg){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = RF230DriverLayerP__RadioPacket__headerLength(msg);
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 79 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MetadataFlagsLayerC.nc"
static inline uint8_t MetadataFlagsLayerC__RadioPacket__headerLength(message_t *msg)
{
  return MetadataFlagsLayerC__SubPacket__headerLength(msg);
}

# 32 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t TimeStampingLayerP__SubPacket__headerLength(message_t *msg){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = MetadataFlagsLayerC__RadioPacket__headerLength(msg);
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 105 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TimeStampingLayerP.nc"
static inline uint8_t TimeStampingLayerP__RadioPacket__headerLength(message_t *msg)
{
  return TimeStampingLayerP__SubPacket__headerLength(msg);
}

# 32 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t PacketLinkLayerP__SubPacket__headerLength(message_t *msg){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = TimeStampingLayerP__RadioPacket__headerLength(msg);
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 231 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/PacketLinkLayerP.nc"
static inline uint8_t PacketLinkLayerP__RadioPacket__headerLength(message_t *msg)
#line 231
{
  return PacketLinkLayerP__SubPacket__headerLength(msg);
}

# 32 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t Ieee154PacketLayerP__SubPacket__headerLength(message_t *msg){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = PacketLinkLayerP__RadioPacket__headerLength(msg);
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 255 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
static inline uint8_t Ieee154PacketLayerP__RadioPacket__headerLength(message_t *msg)
{
  return Ieee154PacketLayerP__SubPacket__headerLength(msg) + sizeof(ieee154_header_t );
}

# 32 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t TinyosNetworkLayerC__SubPacket__headerLength(message_t *msg){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = Ieee154PacketLayerP__RadioPacket__headerLength(msg);
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 143 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TinyosNetworkLayerC.nc"
static inline uint8_t TinyosNetworkLayerC__TinyosPacket__headerLength(message_t *msg)
{
  return TinyosNetworkLayerC__SubPacket__headerLength(msg) + TinyosNetworkLayerC__PAYLOAD_OFFSET;
}

# 32 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t ActiveMessageLayerP__SubPacket__headerLength(message_t *msg){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = TinyosNetworkLayerC__TinyosPacket__headerLength(msg);
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 51 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayerP.nc"
static inline activemessage_header_t *ActiveMessageLayerP__getHeader(message_t *msg)
{
  return (void *)msg + ActiveMessageLayerP__SubPacket__headerLength(msg);
}

#line 173
static __inline am_id_t ActiveMessageLayerP__AMPacket__type(message_t *msg)
{
  return __nesc_ntoh_uint8(ActiveMessageLayerP__getHeader(msg)->type.data);
}

# 136 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static am_id_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(message_t * amsg){
#line 136
  unsigned char __nesc_result;
#line 136

#line 136
  __nesc_result = ActiveMessageLayerP__AMPacket__type(amsg);
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 57 "/opt/tinyos-2.x/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket(void )
#line 57
{
  uint8_t i;

#line 59
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 1;
  for (i = 0; i < 1; i++) {
      if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg == (void *)0 || 
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current / 8] & (1 << /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current % 8)) 
        {
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 1;
        }
      else {
          break;
        }
    }
  if (i >= 1) {
#line 70
    /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
    }
}

#line 166
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void )
#line 166
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket();
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current < 1) {
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

# 67 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline uint8_t RF230RadioP__RF230DriverConfig__maxPayloadLength(void )
{
  return sizeof(rf230packet_header_t ) + 28;
}

# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverConfig.nc"
inline static uint8_t RF230DriverLayerP__Config__maxPayloadLength(void ){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = RF230RadioP__RF230DriverConfig__maxPayloadLength();
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 881 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static inline uint8_t RF230DriverLayerP__RadioPacket__maxPayloadLength(void )
{
  for (; 0; ) ;

  return RF230DriverLayerP__Config__maxPayloadLength() - sizeof(rf230_header_t );
}

# 48 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t MetadataFlagsLayerC__SubPacket__maxPayloadLength(void ){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = RF230DriverLayerP__RadioPacket__maxPayloadLength();
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 94 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MetadataFlagsLayerC.nc"
static inline uint8_t MetadataFlagsLayerC__RadioPacket__maxPayloadLength(void )
{
  return MetadataFlagsLayerC__SubPacket__maxPayloadLength();
}

# 48 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t TimeStampingLayerP__SubPacket__maxPayloadLength(void ){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = MetadataFlagsLayerC__RadioPacket__maxPayloadLength();
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 120 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TimeStampingLayerP.nc"
static inline uint8_t TimeStampingLayerP__RadioPacket__maxPayloadLength(void )
{
  return TimeStampingLayerP__SubPacket__maxPayloadLength();
}

# 48 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t PacketLinkLayerP__SubPacket__maxPayloadLength(void ){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = TimeStampingLayerP__RadioPacket__maxPayloadLength();
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 243 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/PacketLinkLayerP.nc"
static inline uint8_t PacketLinkLayerP__RadioPacket__maxPayloadLength(void )
#line 243
{
  return PacketLinkLayerP__SubPacket__maxPayloadLength();
}

# 48 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t Ieee154PacketLayerP__SubPacket__maxPayloadLength(void ){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = PacketLinkLayerP__RadioPacket__maxPayloadLength();
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 270 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
static inline uint8_t Ieee154PacketLayerP__RadioPacket__maxPayloadLength(void )
{
  return Ieee154PacketLayerP__SubPacket__maxPayloadLength() - sizeof(ieee154_header_t );
}

# 48 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t TinyosNetworkLayerC__SubPacket__maxPayloadLength(void ){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = Ieee154PacketLayerP__RadioPacket__maxPayloadLength();
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 158 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TinyosNetworkLayerC.nc"
static inline uint8_t TinyosNetworkLayerC__TinyosPacket__maxPayloadLength(void )
{
  return TinyosNetworkLayerC__SubPacket__maxPayloadLength() - TinyosNetworkLayerC__PAYLOAD_OFFSET;
}

# 48 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t ActiveMessageLayerP__SubPacket__maxPayloadLength(void ){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = TinyosNetworkLayerC__TinyosPacket__maxPayloadLength();
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 214 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayerP.nc"
static inline uint8_t ActiveMessageLayerP__RadioPacket__maxPayloadLength(void )
{
  return ActiveMessageLayerP__SubPacket__maxPayloadLength() - sizeof(activemessage_header_t );
}

#line 246
static inline uint8_t ActiveMessageLayerP__Packet__maxPayloadLength(void )
{
  return ActiveMessageLayerP__RadioPacket__maxPayloadLength();
}

# 281 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_hton_leuint16(void * target, uint16_t value)
#line 281
{
  uint8_t *base = target;

#line 283
  base[0] = value;
  base[1] = value >> 8;
  return value;
}

# 88 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
static inline void Ieee154PacketLayerP__Ieee154PacketLayer__createDataFrame(message_t *msg)
{
  __nesc_hton_leuint16(Ieee154PacketLayerP__getHeader(msg)->fcf.data, Ieee154PacketLayerP__IEEE154_DATA_FRAME_VALUE);
}

# 58 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayer.nc"
inline static void RF230RadioP__Ieee154PacketLayer__createDataFrame(message_t *msg){
#line 58
  Ieee154PacketLayerP__Ieee154PacketLayer__createDataFrame(msg);
#line 58
}
#line 58
# 83 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
static inline bool Ieee154PacketLayerP__Ieee154PacketLayer__isDataFrame(message_t *msg)
{
  return (__nesc_ntoh_leuint16(Ieee154PacketLayerP__getHeader(msg)->fcf.data) & Ieee154PacketLayerP__IEEE154_DATA_FRAME_MASK) == Ieee154PacketLayerP__IEEE154_DATA_FRAME_VALUE;
}

# 52 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayer.nc"
inline static bool RF230RadioP__Ieee154PacketLayer__isDataFrame(message_t *msg){
#line 52
  unsigned char __nesc_result;
#line 52

#line 52
  __nesc_result = Ieee154PacketLayerP__Ieee154PacketLayer__isDataFrame(msg);
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 192 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline error_t RF230RadioP__ActiveMessageConfig__checkFrame(message_t *msg)
{
  if (!RF230RadioP__Ieee154PacketLayer__isDataFrame(msg)) {
    RF230RadioP__Ieee154PacketLayer__createDataFrame(msg);
    }
  return SUCCESS;
}

# 52 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageConfig.nc"
inline static error_t ActiveMessageLayerP__Config__checkFrame(message_t *msg){
#line 52
  unsigned char __nesc_result;
#line 52

#line 52
  __nesc_result = RF230RadioP__ActiveMessageConfig__checkFrame(msg);
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static void MetadataFlagsLayerC__SubPacket__setPayloadLength(message_t *msg, uint8_t length){
#line 43
  RF230DriverLayerP__RadioPacket__setPayloadLength(msg, length);
#line 43
}
#line 43
# 89 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MetadataFlagsLayerC.nc"
static inline void MetadataFlagsLayerC__RadioPacket__setPayloadLength(message_t *msg, uint8_t length)
{
  MetadataFlagsLayerC__SubPacket__setPayloadLength(msg, length);
}

# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static void TimeStampingLayerP__SubPacket__setPayloadLength(message_t *msg, uint8_t length){
#line 43
  MetadataFlagsLayerC__RadioPacket__setPayloadLength(msg, length);
#line 43
}
#line 43
# 115 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TimeStampingLayerP.nc"
static inline void TimeStampingLayerP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length)
{
  TimeStampingLayerP__SubPacket__setPayloadLength(msg, length);
}

# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static void PacketLinkLayerP__SubPacket__setPayloadLength(message_t *msg, uint8_t length){
#line 43
  TimeStampingLayerP__RadioPacket__setPayloadLength(msg, length);
#line 43
}
#line 43
# 239 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/PacketLinkLayerP.nc"
static inline void PacketLinkLayerP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length)
#line 239
{
  PacketLinkLayerP__SubPacket__setPayloadLength(msg, length);
}

# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static void Ieee154PacketLayerP__SubPacket__setPayloadLength(message_t *msg, uint8_t length){
#line 43
  PacketLinkLayerP__RadioPacket__setPayloadLength(msg, length);
#line 43
}
#line 43
# 265 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
static inline void Ieee154PacketLayerP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length)
{
  Ieee154PacketLayerP__SubPacket__setPayloadLength(msg, length + sizeof(ieee154_header_t ));
}

# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static void TinyosNetworkLayerC__SubPacket__setPayloadLength(message_t *msg, uint8_t length){
#line 43
  Ieee154PacketLayerP__RadioPacket__setPayloadLength(msg, length);
#line 43
}
#line 43
# 153 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TinyosNetworkLayerC.nc"
static inline void TinyosNetworkLayerC__TinyosPacket__setPayloadLength(message_t *msg, uint8_t length)
{
  TinyosNetworkLayerC__SubPacket__setPayloadLength(msg, length + TinyosNetworkLayerC__PAYLOAD_OFFSET);
}

# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static void ActiveMessageLayerP__SubPacket__setPayloadLength(message_t *msg, uint8_t length){
#line 43
  TinyosNetworkLayerC__TinyosPacket__setPayloadLength(msg, length);
#line 43
}
#line 43
# 209 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayerP.nc"
static inline void ActiveMessageLayerP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length)
{
  ActiveMessageLayerP__SubPacket__setPayloadLength(msg, length + sizeof(activemessage_header_t ));
}

#line 241
static inline void ActiveMessageLayerP__Packet__setPayloadLength(message_t *msg, uint8_t len)
{
  ActiveMessageLayerP__RadioPacket__setPayloadLength(msg, len);
}

# 182 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
static inline void Ieee154PacketLayerP__Ieee154PacketLayer__setSrcAddr(message_t *msg, uint16_t addr)
{
  __nesc_hton_leuint16(Ieee154PacketLayerP__getHeader(msg)->src.data, addr);
}

# 145 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayer.nc"
inline static void RF230RadioP__Ieee154PacketLayer__setSrcAddr(message_t *msg, uint16_t addr){
#line 145
  Ieee154PacketLayerP__Ieee154PacketLayer__setSrcAddr(msg, addr);
#line 145
}
#line 145
# 177 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline void RF230RadioP__ActiveMessageConfig__setSource(message_t *msg, am_addr_t addr)
{
  RF230RadioP__Ieee154PacketLayer__setSrcAddr(msg, addr);
}

# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageConfig.nc"
inline static void ActiveMessageLayerP__Config__setSource(message_t *msg, am_addr_t addr){
#line 38
  RF230RadioP__ActiveMessageConfig__setSource(msg, addr);
#line 38
}
#line 38
# 168 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayerP.nc"
static __inline void ActiveMessageLayerP__AMPacket__setSource(message_t *msg, am_addr_t addr)
{
  ActiveMessageLayerP__Config__setSource(msg, addr);
}

# 162 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
static inline void Ieee154PacketLayerP__Ieee154PacketLayer__setDestPan(message_t *msg, uint16_t pan)
{
  __nesc_hton_leuint16(Ieee154PacketLayerP__getHeader(msg)->destpan.data, pan);
}

# 125 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayer.nc"
inline static void RF230RadioP__Ieee154PacketLayer__setDestPan(message_t *msg, uint16_t pan){
#line 125
  Ieee154PacketLayerP__Ieee154PacketLayer__setDestPan(msg, pan);
#line 125
}
#line 125
# 187 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline void RF230RadioP__ActiveMessageConfig__setGroup(message_t *msg, am_group_t grp)
{
  RF230RadioP__Ieee154PacketLayer__setDestPan(msg, grp);
}

# 44 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageConfig.nc"
inline static void ActiveMessageLayerP__Config__setGroup(message_t *msg, am_group_t grp){
#line 44
  RF230RadioP__ActiveMessageConfig__setGroup(msg, grp);
#line 44
}
#line 44
# 188 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayerP.nc"
static __inline void ActiveMessageLayerP__AMPacket__setGroup(message_t *msg, am_group_t grp)
{
  ActiveMessageLayerP__Config__setGroup(msg, grp);
}

# 82 "/opt/tinyos-2.x/tos/system/ActiveMessageAddressC.nc"
static inline am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void )
#line 82
{
  am_group_t myGroup;

#line 84
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 84
    myGroup = ActiveMessageAddressC__group;
#line 84
    __nesc_atomic_end(__nesc_atomic); }
  return myGroup;
}

# 55 "/opt/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
inline static am_group_t ActiveMessageLayerP__ActiveMessageAddress__amGroup(void ){
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
# 142 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayerP.nc"
static __inline am_group_t ActiveMessageLayerP__AMPacket__localGroup(void )
{
  return ActiveMessageLayerP__ActiveMessageAddress__amGroup();
}

# 240 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_hton_uint8(void * target, uint8_t value)
#line 240
{
  uint8_t *base = target;

#line 242
  base[0] = value;
  return value;
}

# 178 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayerP.nc"
static __inline void ActiveMessageLayerP__AMPacket__setType(message_t *msg, am_id_t type)
{
  __nesc_hton_uint8(ActiveMessageLayerP__getHeader(msg)->type.data, type);
}

# 172 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
static inline void Ieee154PacketLayerP__Ieee154PacketLayer__setDestAddr(message_t *msg, uint16_t addr)
{
  __nesc_hton_leuint16(Ieee154PacketLayerP__getHeader(msg)->dest.data, addr);
}

# 135 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayer.nc"
inline static void RF230RadioP__Ieee154PacketLayer__setDestAddr(message_t *msg, uint16_t addr){
#line 135
  Ieee154PacketLayerP__Ieee154PacketLayer__setDestAddr(msg, addr);
#line 135
}
#line 135
# 167 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline void RF230RadioP__ActiveMessageConfig__setDestination(message_t *msg, am_addr_t addr)
{
  RF230RadioP__Ieee154PacketLayer__setDestAddr(msg, addr);
}

# 32 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageConfig.nc"
inline static void ActiveMessageLayerP__Config__setDestination(message_t *msg, am_addr_t addr){
#line 32
  RF230RadioP__ActiveMessageConfig__setDestination(msg, addr);
#line 32
}
#line 32
# 158 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayerP.nc"
static __inline void ActiveMessageLayerP__AMPacket__setDestination(message_t *msg, am_addr_t addr)
{
  ActiveMessageLayerP__Config__setDestination(msg, addr);
}

#line 106
static inline void ActiveMessageLayerP__SendNotifier__default__aboutToSend(am_id_t id, am_addr_t addr, message_t *msg)
{
}

# 59 "/opt/tinyos-2.x/tos/interfaces/SendNotifier.nc"
inline static void ActiveMessageLayerP__SendNotifier__aboutToSend(am_id_t arg_0xf7232c30, am_addr_t dest, message_t * msg){
#line 59
    ActiveMessageLayerP__SendNotifier__default__aboutToSend(arg_0xf7232c30, dest, msg);
#line 59
}
#line 59
# 54 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline bool /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 54
{
  /* atomic removed: atomic calls only */
#line 55
  {
    unsigned char __nesc_temp = 
#line 55
    /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__resQ[id] != /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY || /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qTail == id;

#line 55
    return __nesc_temp;
  }
}

#line 72
static inline error_t /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id)
#line 72
{
  /* atomic removed: atomic calls only */
#line 73
  {
    if (!/*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(id)) {
        if (/*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qHead == /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
          /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qHead = id;
          }
        else {
#line 78
          /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__resQ[/*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qTail] = id;
          }
#line 79
        /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qTail = id;
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
inline static error_t /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Queue__enqueue(resource_client_id_t id){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(id);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 167 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(uint8_t id)
#line 167
{
}

# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
inline static void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__requested(uint8_t arg_0xf71c77d0){
#line 43
    /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(arg_0xf71c77d0);
#line 43
}
#line 43
# 71 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline error_t /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__request(uint8_t id)
#line 71
{
  /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__requested(/*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 73
    {
      if (/*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__state == /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_IDLE) {
          /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__state = /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_GRANTING;
          /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__reqResId = id;
          /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__postTask();
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
        /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Queue__enqueue(id);

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

# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__request(0U);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
#line 110
inline static error_t /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__release(0U);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
inline static error_t /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__SubSend__send(message_t *msg){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = TinyosNetworkLayerC__TinyosSend__send(msg);
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 171 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 171
{
}

# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__configure(uint8_t arg_0xf71c5430){
#line 49
    /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(arg_0xf71c5430);
#line 49
}
#line 49
# 169 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id)
#line 169
{
}

# 51 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
inline static void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__immediateRequested(uint8_t arg_0xf71c77d0){
#line 51
    /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__immediateRequested(arg_0xf71c77d0);
#line 51
}
#line 51
# 84 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline error_t /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__immediateRequest(uint8_t id)
#line 84
{
  /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__immediateRequested(/*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 86
    {
      if (/*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__state == /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_IDLE) {
          /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__state = /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_BUSY;
          /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId = id;
          /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__configure(/*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId);
          {
            unsigned char __nesc_temp = 
#line 91
            SUCCESS;

            {
#line 91
              __nesc_atomic_end(__nesc_atomic); 
#line 91
              return __nesc_temp;
            }
          }
        }
#line 93
      {
        unsigned char __nesc_temp = 
#line 93
        FAIL;

        {
#line 93
          __nesc_atomic_end(__nesc_atomic); 
#line 93
          return __nesc_temp;
        }
      }
    }
#line 96
    __nesc_atomic_end(__nesc_atomic); }
}

# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__immediateRequest(0U);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 42 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/AutoResourceAcquireLayerC.nc"
static inline error_t /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__BareSend__send(message_t *msg)
{
  if (/*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__immediateRequest() == SUCCESS) 
    {
      error_t result = /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__SubSend__send(msg);

#line 47
      if (result != SUCCESS) {
        /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__release();
        }
      return result;
    }

  /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__pending = msg;
  return /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__request();
}

# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
inline static error_t ActiveMessageLayerP__SubSend__send(message_t *msg){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__BareSend__send(msg);
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
inline static error_t PacketLinkLayerP__SubSend__send(message_t *msg){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = MessageBufferLayerP__Send__send(msg);
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 94 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayer.nc"
inline static void RF230RadioP__Ieee154PacketLayer__setAckRequired(message_t *msg, bool ack){
#line 94
  Ieee154PacketLayerP__Ieee154PacketLayer__setAckRequired(msg, ack);
#line 94
}
#line 94
# 105 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline void RF230RadioP__SoftwareAckConfig__setAckRequired(message_t *msg, bool ack)
{
  RF230RadioP__Ieee154PacketLayer__setAckRequired(msg, ack);
}

# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/SoftwareAckConfig.nc"
inline static void SoftwareAckLayerP__SoftwareAckConfig__setAckRequired(message_t *msg, bool ack){
#line 38
  RF230RadioP__SoftwareAckConfig__setAckRequired(msg, ack);
#line 38
}
#line 38
# 167 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/SoftwareAckLayerP.nc"
static inline error_t SoftwareAckLayerP__PacketAcknowledgements__requestAck(message_t *msg)
{
  SoftwareAckLayerP__SoftwareAckConfig__setAckRequired(msg, TRUE);

  return SUCCESS;
}

# 48 "/opt/tinyos-2.x/tos/interfaces/PacketAcknowledgements.nc"
inline static error_t PacketLinkLayerP__PacketAcknowledgements__requestAck(message_t * msg){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = SoftwareAckLayerP__PacketAcknowledgements__requestAck(msg);
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 72 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline uint8_t RF230RadioP__RF230DriverConfig__metadataLength(message_t *msg)
{
  return 0;
}

# 41 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverConfig.nc"
inline static uint8_t RF230DriverLayerP__Config__metadataLength(message_t *msg){
#line 41
  unsigned char __nesc_result;
#line 41

#line 41
  __nesc_result = RF230RadioP__RF230DriverConfig__metadataLength(msg);
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 888 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static inline uint8_t RF230DriverLayerP__RadioPacket__metadataLength(message_t *msg)
{
  return RF230DriverLayerP__Config__metadataLength(msg) + sizeof(rf230_metadata_t );
}

# 54 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t MetadataFlagsLayerC__SubPacket__metadataLength(message_t *msg){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = RF230DriverLayerP__RadioPacket__metadataLength(msg);
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 99 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MetadataFlagsLayerC.nc"
static inline uint8_t MetadataFlagsLayerC__RadioPacket__metadataLength(message_t *msg)
{
  return MetadataFlagsLayerC__SubPacket__metadataLength(msg) + sizeof(flags_metadata_t );
}

# 54 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t TimeStampingLayerP__SubPacket__metadataLength(message_t *msg){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = MetadataFlagsLayerC__RadioPacket__metadataLength(msg);
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 125 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TimeStampingLayerP.nc"
static inline uint8_t TimeStampingLayerP__RadioPacket__metadataLength(message_t *msg)
{
  return TimeStampingLayerP__SubPacket__metadataLength(msg) + sizeof(timestamp_metadata_t );
}

# 54 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t PacketLinkLayerP__SubPacket__metadataLength(message_t *msg){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = TimeStampingLayerP__RadioPacket__metadataLength(msg);
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 247 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/PacketLinkLayerP.nc"
static inline uint8_t PacketLinkLayerP__RadioPacket__metadataLength(message_t *msg)
#line 247
{
  return PacketLinkLayerP__SubPacket__metadataLength(msg) + sizeof(link_metadata_t );
}

#line 93
static inline link_metadata_t *PacketLinkLayerP__getMeta(message_t *msg)
#line 93
{
  return (void *)msg + sizeof(message_t ) - PacketLinkLayerP__RadioPacket__metadataLength(msg);
}

#line 120
static inline uint16_t PacketLinkLayerP__PacketLink__getRetries(message_t *msg)
#line 120
{
  return PacketLinkLayerP__getMeta(msg)->maxRetries;
}

#line 146
static inline error_t PacketLinkLayerP__Send__send(message_t *msg)
#line 146
{
  error_t error = EBUSY;

#line 148
  if (PacketLinkLayerP__currentSendMsg == (void *)0) {

      if (PacketLinkLayerP__PacketLink__getRetries(msg) > 0) {
          PacketLinkLayerP__PacketAcknowledgements__requestAck(msg);
        }

      if ((error = PacketLinkLayerP__SubSend__send(msg)) == SUCCESS) {
          PacketLinkLayerP__currentSendMsg = msg;
          PacketLinkLayerP__totalRetries = 0;
        }
    }
  return error;
}

# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
inline static error_t UniqueLayerP__SubSend__send(message_t *msg){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = PacketLinkLayerP__Send__send(msg);
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 251 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_hton_leuint8(void * target, uint8_t value)
#line 251
{
  uint8_t *base = target;

#line 253
  base[0] = value;
  return value;
}

# 152 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
static inline void Ieee154PacketLayerP__Ieee154PacketLayer__setDSN(message_t *msg, uint8_t dsn)
{
  __nesc_hton_leuint8(Ieee154PacketLayerP__getHeader(msg)->dsn.data, dsn);
}

# 114 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayer.nc"
inline static void RF230RadioP__Ieee154PacketLayer__setDSN(message_t *msg, uint8_t dsn){
#line 114
  Ieee154PacketLayerP__Ieee154PacketLayer__setDSN(msg, dsn);
#line 114
}
#line 114
# 143 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline void RF230RadioP__UniqueConfig__setSequenceNumber(message_t *msg, uint8_t dsn)
{
  RF230RadioP__Ieee154PacketLayer__setDSN(msg, dsn);
}

# 41 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/UniqueConfig.nc"
inline static void UniqueLayerP__UniqueConfig__setSequenceNumber(message_t *msg, uint8_t number){
#line 41
  RF230RadioP__UniqueConfig__setSequenceNumber(msg, number);
#line 41
}
#line 41
# 58 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/UniqueLayerP.nc"
static inline error_t UniqueLayerP__Send__send(message_t *msg)
{
  UniqueLayerP__UniqueConfig__setSequenceNumber(msg, ++UniqueLayerP__sequenceNumber);
  return UniqueLayerP__SubSend__send(msg);
}

# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
inline static error_t TinyosNetworkLayerC__SubSend__send(message_t *msg){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = UniqueLayerP__Send__send(msg);
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
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

# 50 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline bool /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 51
  {
    unsigned char __nesc_temp = 
#line 51
    /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qHead == /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

#line 51
    return __nesc_temp;
  }
}

# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static bool /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Queue__isEmpty(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 58 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    if (/*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qHead != /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
        uint8_t id = /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qHead;

#line 62
        /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qHead = /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__resQ[/*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qHead];
        if (/*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qHead == /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
          /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qTail = /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
          }
#line 65
        /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__resQ[id] = /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
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
      /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

#line 68
      return __nesc_temp;
    }
  }
}

# 60 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Queue__dequeue(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 173 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id)
#line 173
{
}

# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__unconfigure(uint8_t arg_0xf71c5430){
#line 55
    /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__unconfigure(arg_0xf71c5430);
#line 55
}
#line 55
# 49 "/opt/tinyos-2.x/tos/lib/serial/SerialActiveMessageP.nc"
static inline serial_header_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(message_t * msg)
#line 49
{
  return (serial_header_t * )((uint8_t *)msg + (unsigned short )& ((message_t *)0)->data - sizeof(serial_header_t ));
}

#line 161
static inline am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(message_t *amsg)
#line 161
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 163
  return __nesc_ntoh_uint8(header->type.data);
}

# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__toggle(void ){
#line 31
  /*HplAtm128GeneralIOC.PortA.Bit2*/HplAtm128GeneralIOPinP__2__IO__toggle();
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
inline static void BaseStationP__Leds__led0Toggle(void ){
#line 56
  LedsP__Leds__led0Toggle();
#line 56
}
#line 56
# 131 "../InterceptBase/BaseStationP.nc"
static inline void BaseStationP__sucBlink(void )
#line 131
{



  BaseStationP__Leds__led0Toggle();
}

# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__toggle(void ){
#line 31
  /*HplAtm128GeneralIOC.PortA.Bit0*/HplAtm128GeneralIOPinP__0__IO__toggle();
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
inline static void BaseStationP__Leds__led2Toggle(void ){
#line 89
  LedsP__Leds__led2Toggle();
#line 89
}
#line 89
# 151 "../InterceptBase/BaseStationP.nc"
static inline void BaseStationP__failBlink(void )
#line 151
{
  BaseStationP__Leds__led2Toggle();
}

#line 594
static inline void BaseStationP__UartSend__sendDone(am_id_t id, message_t *msg, error_t error)
#line 594
{
  if (error != SUCCESS) {
      BaseStationP__failBlink();
      ++BaseStationP__uartFailCounter;
    }
  else 
#line 598
    {
      BaseStationP__uartFailCounter = 0;
      BaseStationP__sucBlink();


      BaseStationP__uartBusy = FALSE;



      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        if (msg == BaseStationP__uartQueue[BaseStationP__uartOut]) {
            if (++BaseStationP__uartOut >= BaseStationP__UART_QUEUE_LEN) {
              BaseStationP__uartOut = 0;
              }
#line 611
            if (BaseStationP__uartFull) {
              BaseStationP__uartFull = FALSE;
              }
          }
#line 614
        __nesc_atomic_end(__nesc_atomic); }
    }


  BaseStationP__timedUartSendTask();
}

# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(am_id_t arg_0xf6caf148, message_t * msg, error_t error){
#line 99
  BaseStationP__UartSend__sendDone(arg_0xf6caf148, msg, error);
#line 99
}
#line 99
# 90 "/opt/tinyos-2.x/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(message_t *msg, error_t result)
#line 90
{
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(msg), msg, result);
}

# 365 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(uart_id_t idxxx, message_t *msg, error_t error)
#line 365
{
  return;
}

# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__sendDone(uart_id_t arg_0xf6bea560, message_t * msg, error_t error){
#line 89
  switch (arg_0xf6bea560) {
#line 89
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 89
      /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(msg, error);
#line 89
      break;
#line 89
    default:
#line 89
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(arg_0xf6bea560, msg, error);
#line 89
      break;
#line 89
    }
#line 89
}
#line 89
# 147 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask(void )
#line 147
{
  error_t error;

  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 151
    error = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError;
#line 151
    __nesc_atomic_end(__nesc_atomic); }

  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendCancelled) {
#line 153
    error = ECANCEL;
    }
#line 154
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__sendDone(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendId, (message_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer, error);
}

# 158 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(uint8_t num)
{
  return /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num].isrunning;
}

# 81 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static bool BaseStationP__UartTimer__isRunning(void ){
#line 81
  unsigned char __nesc_result;
#line 81

#line 81
  __nesc_result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(2U);
#line 81

#line 81
  return __nesc_result;
#line 81
}
#line 81
# 249 "/opt/tinyos-2.x/tos/chips/atm1281/timer/Atm1281AlarmAsyncP.nc"
static inline uint32_t /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__getNow(void )
#line 249
{
  return /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Counter__get();
}

# 98 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void ){
#line 98
  unsigned long __nesc_result;
#line 98

#line 98
  __nesc_result = /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__getNow();
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
inline static void BaseStationP__UartTimer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(2U, dt);
#line 62
}
#line 62
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
# 134 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer2AsyncP.nc"
static inline uint8_t HplAtm1281Timer2AsyncP__TimerCtrl__getInterruptFlag(void )
#line 134
{
  return * (volatile uint8_t *)(0x17 + 0x20);
}

# 69 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm128TimerCtrl8.nc"
inline static uint8_t /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__TimerCtrl__getInterruptFlag(void ){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = HplAtm1281Timer2AsyncP__TimerCtrl__getInterruptFlag();
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 201 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(uint8_t which)
#line 201
{
  if (which) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufOneLocked = 0;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufZeroLocked = 0;
    }
}

# 730 "../InterceptBase/BaseStationP.nc"
static inline bool BaseStationP__SerialIntercept__default__forward(am_id_t amid, message_t *msg, void *payload, uint8_t len)
#line 730
{
  return TRUE;
}

# 31 "/opt/tinyos-2.x/tos/interfaces/Intercept.nc"
inline static bool BaseStationP__SerialIntercept__forward(am_id_t arg_0xf7399218, message_t * msg, void * payload, uint8_t len){
#line 31
  unsigned char __nesc_result;
#line 31

#line 31
    __nesc_result = BaseStationP__SerialIntercept__default__forward(arg_0xf7399218, msg, payload, len);
#line 31

#line 31
  return __nesc_result;
#line 31
}
#line 31
# 622 "../InterceptBase/BaseStationP.nc"
static inline message_t *BaseStationP__UartReceive__receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
#line 622
{
  message_t *ret = msg;
  bool reflectToken = FALSE;




  if (!BaseStationP__SerialIntercept__forward(id, msg, payload, len)) {
    return ret;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    if (!BaseStationP__radioFull) {
        reflectToken = TRUE;
        ret = BaseStationP__radioQueue[BaseStationP__radioIn];
        BaseStationP__radioQueue[BaseStationP__radioIn] = msg;
        if (++BaseStationP__radioIn >= BaseStationP__RADIO_QUEUE_LEN) {
          BaseStationP__radioIn = 0;
          }
#line 639
        if (BaseStationP__radioIn == BaseStationP__radioOut) {
          BaseStationP__radioFull = TRUE;
          }
        if (!BaseStationP__radioBusy) {


            BaseStationP__RadioTimer__startOneShot(BaseStationP__RADIO_TIME);
            BaseStationP__radioBusy = TRUE;
          }
      }
    else {
      }
#line 650
    __nesc_atomic_end(__nesc_atomic); }


  if (reflectToken) {
    }


  return ret;
}

# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(am_id_t arg_0xf6cafb08, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  __nesc_result = BaseStationP__UartReceive__receive(arg_0xf6cafb08, msg, payload, len);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 102 "/opt/tinyos-2.x/tos/lib/serial/SerialActiveMessageP.nc"
static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 102
{
  return /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(msg), msg, msg->data, len);
}

# 360 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
static inline message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(uart_id_t idxxx, message_t *msg, 
void *payload, 
uint8_t len)
#line 362
{
  return msg;
}

# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__receive(uart_id_t arg_0xf6bede50, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  switch (arg_0xf6bede50) {
#line 67
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 67
      __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(msg, payload, len);
#line 67
      break;
#line 67
    default:
#line 67
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(arg_0xf6bede50, msg, payload, len);
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
# 46 "/opt/tinyos-2.x/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__upperLength(message_t *msg, uint8_t dataLinkLen)
#line 46
{
  return dataLinkLen - sizeof(serial_header_t );
}

# 354 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(uart_id_t id, message_t *msg, 
uint8_t dataLinkLen)
#line 355
{
  return 0;
}

# 31 "/opt/tinyos-2.x/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(uart_id_t arg_0xf6be9088, message_t *msg, uint8_t dataLinkLen){
#line 31
  unsigned char __nesc_result;
#line 31

#line 31
  switch (arg_0xf6be9088) {
#line 31
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 31
      __nesc_result = SerialPacketInfoActiveMessageP__Info__upperLength(msg, dataLinkLen);
#line 31
      break;
#line 31
    default:
#line 31
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(arg_0xf6be9088, msg, dataLinkLen);
#line 31
      break;
#line 31
    }
#line 31

#line 31
  return __nesc_result;
#line 31
}
#line 31
# 40 "/opt/tinyos-2.x/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__offset(void )
#line 40
{
  return (uint8_t )(sizeof(message_header_t ) - sizeof(serial_header_t ));
}

# 347 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(uart_id_t id)
#line 347
{
  return 0;
}

# 15 "/opt/tinyos-2.x/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(uart_id_t arg_0xf6be9088){
#line 15
  unsigned char __nesc_result;
#line 15

#line 15
  switch (arg_0xf6be9088) {
#line 15
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 15
      __nesc_result = SerialPacketInfoActiveMessageP__Info__offset();
#line 15
      break;
#line 15
    default:
#line 15
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(arg_0xf6be9088);
#line 15
      break;
#line 15
    }
#line 15

#line 15
  return __nesc_result;
#line 15
}
#line 15
# 264 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask(void )
#line 264
{
  uart_id_t myType;
  message_t *myBuf;
  uint8_t mySize;
  uint8_t myWhich;

#line 269
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 269
    {
      myType = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskType;
      myBuf = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskBuf;
      mySize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskSize;
      myWhich = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskWhich;
    }
#line 274
    __nesc_atomic_end(__nesc_atomic); }
  mySize -= /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(myType);
  mySize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(myType, myBuf, mySize);
  myBuf = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__receive(myType, myBuf, myBuf, mySize);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 278
    {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messagePtrs[myWhich] = myBuf;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(myWhich);
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending = FALSE;
    }
#line 282
    __nesc_atomic_end(__nesc_atomic); }
}

# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__toggle(void ){
#line 31
  /*HplAtm128GeneralIOC.PortA.Bit1*/HplAtm128GeneralIOPinP__1__IO__toggle();
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
inline static void BaseStationP__Leds__led1Toggle(void ){
#line 72
  LedsP__Leds__led1Toggle();
#line 72
}
#line 72
# 146 "../InterceptBase/BaseStationP.nc"
static inline void BaseStationP__dropBlink(void )
#line 146
{
  BaseStationP__Leds__led1Toggle();
}

#line 418
static inline void BaseStationP__SerialControl__stopDone(error_t error)
#line 418
{
  if (error == SUCCESS) {
      BaseStationP__sucBlink();


      BaseStationP__resetPhase++;
      BaseStationP__ResetTimer__startOneShot(BaseStationP__RESET_TIME);
    }
  else 
#line 425
    {
      BaseStationP__failBlink();
      BaseStationP__dropBlink();


      BaseStationP__ResetTimer__startOneShot(BaseStationP__RESET_TIME);
    }
}

# 117 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static void SerialP__SplitControl__stopDone(error_t error){
#line 117
  BaseStationP__SerialControl__stopDone(error);
#line 117
}
#line 117
# 142 "/opt/tinyos-2.x/tos/chips/atm1281/McuSleepC.nc"
static inline void McuSleepC__McuPowerState__update(void )
#line 142
{
}

# 44 "/opt/tinyos-2.x/tos/interfaces/McuPowerState.nc"
inline static void HplAtm128UartP__McuPowerState__update(void ){
#line 44
  McuSleepC__McuPowerState__update();
#line 44
}
#line 44
# 148 "/opt/tinyos-2.x/tos/chips/atm1281/HplAtm128UartP.nc"
static inline error_t HplAtm128UartP__Uart0RxControl__stop(void )
#line 148
{
  * (volatile uint8_t *)0XC1 &= ~(1 << 4);
  HplAtm128UartP__McuPowerState__update();
  return SUCCESS;
}

# 84 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
inline static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUartRxControl__stop(void ){
#line 84
  unsigned char __nesc_result;
#line 84

#line 84
  __nesc_result = HplAtm128UartP__Uart0RxControl__stop();
#line 84

#line 84
  return __nesc_result;
#line 84
}
#line 84
# 136 "/opt/tinyos-2.x/tos/chips/atm1281/HplAtm128UartP.nc"
static inline error_t HplAtm128UartP__Uart0TxControl__stop(void )
#line 136
{
  * (volatile uint8_t *)0XC1 &= ~(1 << 3);
  HplAtm128UartP__McuPowerState__update();
  return SUCCESS;
}

# 84 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
inline static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUartTxControl__stop(void ){
#line 84
  unsigned char __nesc_result;
#line 84

#line 84
  __nesc_result = HplAtm128UartP__Uart0TxControl__stop();
#line 84

#line 84
  return __nesc_result;
#line 84
}
#line 84
# 93 "/opt/tinyos-2.x/tos/chips/atm128/Atm128UartP.nc"
static inline error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__StdControl__stop(void )
#line 93
{
  /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUartTxControl__stop();
  /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUartRxControl__stop();
  return SUCCESS;
}

# 84 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
inline static error_t SerialP__SerialControl__stop(void ){
#line 84
  unsigned char __nesc_result;
#line 84

#line 84
  __nesc_result = /*Atm128Uart0C.UartP*/Atm128UartP__0__StdControl__stop();
#line 84

#line 84
  return __nesc_result;
#line 84
}
#line 84
# 330 "/opt/tinyos-2.x/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFlush__flushDone(void )
#line 330
{
  SerialP__SerialControl__stop();
  SerialP__SplitControl__stopDone(SUCCESS);
}

static inline void SerialP__defaultSerialFlushTask__runTask(void )
#line 335
{
  SerialP__SerialFlush__flushDone();
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__defaultSerialFlushTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__defaultSerialFlushTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 338 "/opt/tinyos-2.x/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFlush__default__flush(void )
#line 338
{
  SerialP__defaultSerialFlushTask__postTask();
}

# 38 "/opt/tinyos-2.x/tos/lib/serial/SerialFlush.nc"
inline static void SerialP__SerialFlush__flush(void ){
#line 38
  SerialP__SerialFlush__default__flush();
#line 38
}
#line 38
# 326 "/opt/tinyos-2.x/tos/lib/serial/SerialP.nc"
static inline void SerialP__stopDoneTask__runTask(void )
#line 326
{
  SerialP__SerialFlush__flush();
}

# 392 "../InterceptBase/BaseStationP.nc"
static inline void BaseStationP__SerialControl__startDone(error_t error)
#line 392
{
  uint8_t i;

#line 394
  if (error == SUCCESS) {
      BaseStationP__sucBlink();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 396
        {

          for (i = 0; i < BaseStationP__UART_QUEUE_LEN; i++) {
              BaseStationP__uartQueue[i] = &BaseStationP__uartQueueBufs[i];
            }
          BaseStationP__uartIn = BaseStationP__uartOut = 0;
          BaseStationP__uartBusy = FALSE;
          BaseStationP__uartFull = FALSE;


          BaseStationP__resetPhase++;
          BaseStationP__ResetTimer__startOneShot(BaseStationP__RESET_TIME);
        }
#line 408
        __nesc_atomic_end(__nesc_atomic); }
    }
  else 
#line 409
    {
      BaseStationP__failBlink();
      BaseStationP__dropBlink();


      BaseStationP__ResetTimer__startOneShot(BaseStationP__RESET_TIME);
    }
}

# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static void SerialP__SplitControl__startDone(error_t error){
#line 92
  BaseStationP__SerialControl__startDone(error);
#line 92
}
#line 92
# 165 "/opt/tinyos-2.x/tos/chips/atm1281/HplAtm128UartP.nc"
static inline error_t HplAtm128UartP__HplUart0__enableRxIntr(void )
#line 165
{
  * (volatile uint8_t *)0XC1 |= 1 << 7;
  return SUCCESS;
}

# 42 "/opt/tinyos-2.x/tos/chips/atm128/HplAtm128Uart.nc"
inline static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__enableRxIntr(void ){
#line 42
  unsigned char __nesc_result;
#line 42

#line 42
  __nesc_result = HplAtm128UartP__HplUart0__enableRxIntr();
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 142 "/opt/tinyos-2.x/tos/chips/atm1281/HplAtm128UartP.nc"
static inline error_t HplAtm128UartP__Uart0RxControl__start(void )
#line 142
{
  * (volatile uint8_t *)0XC1 |= 1 << 4;
  HplAtm128UartP__McuPowerState__update();
  return SUCCESS;
}

# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
inline static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUartRxControl__start(void ){
#line 74
  unsigned char __nesc_result;
#line 74

#line 74
  __nesc_result = HplAtm128UartP__Uart0RxControl__start();
#line 74

#line 74
  return __nesc_result;
#line 74
}
#line 74
# 130 "/opt/tinyos-2.x/tos/chips/atm1281/HplAtm128UartP.nc"
static inline error_t HplAtm128UartP__Uart0TxControl__start(void )
#line 130
{
  * (volatile uint8_t *)0XC1 |= 1 << 3;
  HplAtm128UartP__McuPowerState__update();
  return SUCCESS;
}

# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
inline static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUartTxControl__start(void ){
#line 74
  unsigned char __nesc_result;
#line 74

#line 74
  __nesc_result = HplAtm128UartP__Uart0TxControl__start();
#line 74

#line 74
  return __nesc_result;
#line 74
}
#line 74
# 170 "/opt/tinyos-2.x/tos/chips/atm1281/HplAtm128UartP.nc"
static inline error_t HplAtm128UartP__HplUart0__disableRxIntr(void )
#line 170
{
  * (volatile uint8_t *)0XC1 &= ~(1 << 7);
  return SUCCESS;
}

# 43 "/opt/tinyos-2.x/tos/chips/atm128/HplAtm128Uart.nc"
inline static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__disableRxIntr(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = HplAtm128UartP__HplUart0__disableRxIntr();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 160 "/opt/tinyos-2.x/tos/chips/atm1281/HplAtm128UartP.nc"
static inline error_t HplAtm128UartP__HplUart0__disableTxIntr(void )
#line 160
{
  * (volatile uint8_t *)0XC1 &= ~(1 << 6);
  return SUCCESS;
}

# 41 "/opt/tinyos-2.x/tos/chips/atm128/HplAtm128Uart.nc"
inline static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__disableTxIntr(void ){
#line 41
  unsigned char __nesc_result;
#line 41

#line 41
  __nesc_result = HplAtm128UartP__HplUart0__disableTxIntr();
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 77 "/opt/tinyos-2.x/tos/chips/atm128/Atm128UartP.nc"
static inline error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__StdControl__start(void )
#line 77
{

  /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__disableTxIntr();
  /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__disableRxIntr();
  /*Atm128Uart0C.UartP*/Atm128UartP__0__m_rx_intr = 0;
  /*Atm128Uart0C.UartP*/Atm128UartP__0__m_tx_intr = 0;


  /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUartTxControl__start();
  /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUartRxControl__start();


  /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__enableRxIntr();
  return SUCCESS;
}

# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
inline static error_t SerialP__SerialControl__start(void ){
#line 74
  unsigned char __nesc_result;
#line 74

#line 74
  __nesc_result = /*Atm128Uart0C.UartP*/Atm128UartP__0__StdControl__start();
#line 74

#line 74
  return __nesc_result;
#line 74
}
#line 74
# 320 "/opt/tinyos-2.x/tos/lib/serial/SerialP.nc"
static inline void SerialP__startDoneTask__runTask(void )
#line 320
{
  SerialP__SerialControl__start();
  SerialP__SplitControl__startDone(SUCCESS);
}

# 45 "/opt/tinyos-2.x/tos/lib/serial/SerialFrameComm.nc"
inline static error_t SerialP__SerialFrameComm__putDelimiter(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = HdlcTranslateC__SerialFrameComm__putDelimiter();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 183 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error_t error)
#line 183
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 184
    /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError = error;
#line 184
    __nesc_atomic_end(__nesc_atomic); }
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__postTask();
}

# 80 "/opt/tinyos-2.x/tos/lib/serial/SendBytePacket.nc"
inline static void SerialP__SendBytePacket__sendCompleted(error_t error){
#line 80
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error);
#line 80
}
#line 80
# 242 "/opt/tinyos-2.x/tos/lib/serial/SerialP.nc"
static __inline bool SerialP__ack_queue_is_empty(void )
#line 242
{
  bool ret;

#line 244
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 244
    ret = SerialP__ackQ.writePtr == SerialP__ackQ.readPtr;
#line 244
    __nesc_atomic_end(__nesc_atomic); }
  return ret;
}











static __inline uint8_t SerialP__ack_queue_top(void )
#line 258
{
  uint8_t tmp = 0;

  /* atomic removed: atomic calls only */
#line 260
  {
    if (!SerialP__ack_queue_is_empty()) {
        tmp = SerialP__ackQ.buf[SerialP__ackQ.readPtr];
      }
  }
  return tmp;
}

static inline uint8_t SerialP__ack_queue_pop(void )
#line 268
{
  uint8_t retval = 0;

#line 270
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 270
    {
      if (SerialP__ackQ.writePtr != SerialP__ackQ.readPtr) {
          retval = SerialP__ackQ.buf[SerialP__ackQ.readPtr];
          if (++ SerialP__ackQ.readPtr > SerialP__ACK_QUEUE_SIZE) {
#line 273
            SerialP__ackQ.readPtr = 0;
            }
        }
    }
#line 276
    __nesc_atomic_end(__nesc_atomic); }
#line 276
  return retval;
}

#line 539
static inline void SerialP__RunTx__runTask(void )
#line 539
{
  uint8_t idle;
  uint8_t done;
  uint8_t fail;









  error_t result = SUCCESS;
  bool send_completed = FALSE;
  bool start_it = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 556
    {
      SerialP__txPending = 0;
      idle = SerialP__txState == SerialP__TXSTATE_IDLE;
      done = SerialP__txState == SerialP__TXSTATE_FINISH;
      fail = SerialP__txState == SerialP__TXSTATE_ERROR;
      if (done || fail) {
          SerialP__txState = SerialP__TXSTATE_IDLE;
          SerialP__txBuf[SerialP__txIndex].state = SerialP__BUFFER_AVAILABLE;
        }
    }
#line 565
    __nesc_atomic_end(__nesc_atomic); }


  if (done || fail) {
      SerialP__txSeqno++;
      if (SerialP__txProto == SERIAL_PROTO_ACK) {
          SerialP__ack_queue_pop();
        }
      else {
          result = done ? SUCCESS : FAIL;
          send_completed = TRUE;
        }
      idle = TRUE;
    }


  if (idle) {
      bool goInactive;

#line 583
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 583
        goInactive = SerialP__offPending;
#line 583
        __nesc_atomic_end(__nesc_atomic); }
      if (goInactive) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 585
            SerialP__txState = SerialP__TXSTATE_INACTIVE;
#line 585
            __nesc_atomic_end(__nesc_atomic); }
        }
      else {

          uint8_t myAckState;
          uint8_t myDataState;

#line 591
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 591
            {
              myAckState = SerialP__txBuf[SerialP__TX_ACK_INDEX].state;
              myDataState = SerialP__txBuf[SerialP__TX_DATA_INDEX].state;
            }
#line 594
            __nesc_atomic_end(__nesc_atomic); }
          if (!SerialP__ack_queue_is_empty() && myAckState == SerialP__BUFFER_AVAILABLE) {
              { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 596
                {
                  SerialP__txBuf[SerialP__TX_ACK_INDEX].state = SerialP__BUFFER_COMPLETE;
                  SerialP__txBuf[SerialP__TX_ACK_INDEX].buf = SerialP__ack_queue_top();
                }
#line 599
                __nesc_atomic_end(__nesc_atomic); }
              SerialP__txProto = SERIAL_PROTO_ACK;
              SerialP__txIndex = SerialP__TX_ACK_INDEX;
              start_it = TRUE;
            }
          else {
#line 604
            if (myDataState == SerialP__BUFFER_FILLING || myDataState == SerialP__BUFFER_COMPLETE) {
                SerialP__txProto = SERIAL_PROTO_PACKET_NOACK;
                SerialP__txIndex = SerialP__TX_DATA_INDEX;
                start_it = TRUE;
              }
            else {
              }
            }
        }
    }
  else {
    }


  if (send_completed) {
      SerialP__SendBytePacket__sendCompleted(result);
    }

  if (SerialP__txState == SerialP__TXSTATE_INACTIVE) {
      SerialP__testOff();
      return;
    }

  if (start_it) {

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 629
        {
          SerialP__txCRC = 0;
          SerialP__txByteCnt = 0;
          SerialP__txState = SerialP__TXSTATE_PROTO;
        }
#line 633
        __nesc_atomic_end(__nesc_atomic); }
      if (SerialP__SerialFrameComm__putDelimiter() != SUCCESS) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 635
            SerialP__txState = SerialP__TXSTATE_ERROR;
#line 635
            __nesc_atomic_end(__nesc_atomic); }
          SerialP__MaybeScheduleTx();
        }
    }
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__stopDoneTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__stopDoneTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 154 "/opt/tinyos-2.x/tos/chips/atm1281/HplAtm128UartP.nc"
static inline error_t HplAtm128UartP__HplUart0__enableTxIntr(void )
#line 154
{
  * (volatile uint8_t *)0xC0 |= 1 << 6;
  * (volatile uint8_t *)0XC1 |= 1 << 6;
  return SUCCESS;
}

# 40 "/opt/tinyos-2.x/tos/chips/atm128/HplAtm128Uart.nc"
inline static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__enableTxIntr(void ){
#line 40
  unsigned char __nesc_result;
#line 40

#line 40
  __nesc_result = HplAtm128UartP__HplUart0__enableTxIntr();
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__RunTx__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__RunTx);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 48 "/opt/tinyos-2.x/tos/chips/rf2xx/util/Tasklet.nc"
inline static void RF230DriverLayerP__Tasklet__schedule(void ){
#line 48
  TaskletC__Tasklet__schedule();
#line 48
}
#line 48
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t RF230DriverLayerP__SpiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = Atm128SpiP__Resource__release(0U);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 46 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortB.Bit7*/HplAtm128GeneralIOPinP__15__IO__set(void )
#line 46
{
#line 46
  * (volatile uint8_t * )37U |= 1 << 7;
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void RF230DriverLayerP__SLP_TR__set(void ){
#line 29
  /*HplAtm128GeneralIOC.PortB.Bit7*/HplAtm128GeneralIOPinP__15__IO__set();
#line 29
}
#line 29
# 98 "/opt/tinyos-2.x/tos/chips/atm128/spi/HplAtm128SpiP.nc"
static inline uint8_t HplAtm128SpiP__SPI__read(void )
#line 98
{
#line 98
  return * (volatile uint8_t *)(0X2E + 0x20);
}

# 80 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128Spi.nc"
inline static uint8_t Atm128SpiP__Spi__read(void ){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = HplAtm128SpiP__SPI__read();
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 146 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
static __inline uint8_t Atm128SpiP__FastSpiByte__splitRead(void )
#line 146
{
  while (!(* (volatile uint8_t *)(0x2D + 0x20) & 0x80)) 
    ;
  return Atm128SpiP__Spi__read();
}

# 52 "/opt/tinyos-2.x/tos/interfaces/FastSpiByte.nc"
inline static uint8_t RF230DriverLayerP__FastSpiByte__splitRead(void ){
#line 52
  unsigned char __nesc_result;
#line 52

#line 52
  __nesc_result = Atm128SpiP__FastSpiByte__splitRead();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 99 "/opt/tinyos-2.x/tos/chips/atm128/spi/HplAtm128SpiP.nc"
static inline void HplAtm128SpiP__SPI__write(uint8_t d)
#line 99
{
#line 99
  * (volatile uint8_t *)(0X2E + 0x20) = d;
}

# 86 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128Spi.nc"
inline static void Atm128SpiP__Spi__write(uint8_t data){
#line 86
  HplAtm128SpiP__SPI__write(data);
#line 86
}
#line 86
# 152 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
static __inline uint8_t Atm128SpiP__FastSpiByte__splitReadWrite(uint8_t data)
#line 152
{
  uint8_t b;

  while (!(* (volatile uint8_t *)(0x2D + 0x20) & 0x80)) 
    ;
  b = Atm128SpiP__Spi__read();
  Atm128SpiP__Spi__write(data);

  return b;
}

# 58 "/opt/tinyos-2.x/tos/interfaces/FastSpiByte.nc"
inline static uint8_t RF230DriverLayerP__FastSpiByte__splitReadWrite(uint8_t data){
#line 58
  unsigned char __nesc_result;
#line 58

#line 58
  __nesc_result = Atm128SpiP__FastSpiByte__splitReadWrite(data);
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 142 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
static __inline void Atm128SpiP__FastSpiByte__splitWrite(uint8_t data)
#line 142
{
  Atm128SpiP__Spi__write(data);
}

# 46 "/opt/tinyos-2.x/tos/interfaces/FastSpiByte.nc"
inline static void RF230DriverLayerP__FastSpiByte__splitWrite(uint8_t data){
#line 46
  Atm128SpiP__FastSpiByte__splitWrite(data);
#line 46
}
#line 46
# 47 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortB.Bit0*/HplAtm128GeneralIOPinP__8__IO__clr(void )
#line 47
{
#line 47
  * (volatile uint8_t * )37U &= ~(1 << 0);
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void RF230DriverLayerP__SELN__clr(void ){
#line 30
  /*HplAtm128GeneralIOC.PortB.Bit0*/HplAtm128GeneralIOPinP__8__IO__clr();
#line 30
}
#line 30
# 143 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static __inline void RF230DriverLayerP__writeRegister(uint8_t reg, uint8_t value)
{
  for (; 0; ) ;
  for (; 0; ) ;

  RF230DriverLayerP__SELN__clr();
  RF230DriverLayerP__FastSpiByte__splitWrite(RF230_CMD_REGISTER_WRITE | reg);
  RF230DriverLayerP__FastSpiByte__splitReadWrite(value);
  RF230DriverLayerP__FastSpiByte__splitRead();
  RF230DriverLayerP__SELN__set();
}

# 35 "/opt/tinyos-2.x/tos/platforms/micaz/BusyWaitMicroC.nc"
static __inline void BusyWaitMicroC__BusyWait__wait(uint16_t dt)
#line 35
{

  if (dt) {


     __asm volatile (
    "1:	sbiw	%0,1\n"
    "	adiw	%0,1\n"
    "	sbiw	%0,1\n"
    "	brne	1b" : "+w"(dt));
    }
}

# 55 "/opt/tinyos-2.x/tos/lib/timer/BusyWait.nc"
inline static void RF230DriverLayerP__BusyWait__wait(RF230DriverLayerP__BusyWait__size_type dt){
#line 55
  BusyWaitMicroC__BusyWait__wait(dt);
#line 55
}
#line 55
# 47 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortA.Bit6*/HplAtm128GeneralIOPinP__6__IO__clr(void )
#line 47
{
#line 47
  * (volatile uint8_t * )34U &= ~(1 << 6);
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void RF230DriverLayerP__RSTN__clr(void ){
#line 30
  /*HplAtm128GeneralIOC.PortA.Bit6*/HplAtm128GeneralIOPinP__6__IO__clr();
#line 30
}
#line 30
# 230 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static inline void RF230DriverLayerP__initRadio(void )
{
  RF230DriverLayerP__BusyWait__wait(510);

  RF230DriverLayerP__RSTN__clr();
  RF230DriverLayerP__SLP_TR__clr();
  RF230DriverLayerP__BusyWait__wait(6);
  RF230DriverLayerP__RSTN__set();

  RF230DriverLayerP__writeRegister(RF230_TRX_CTRL_0, RF230_TRX_CTRL_0_VALUE);
  RF230DriverLayerP__writeRegister(RF230_TRX_STATE, RF230_TRX_OFF);

  RF230DriverLayerP__BusyWait__wait(510);

  RF230DriverLayerP__writeRegister(RF230_IRQ_MASK, ((RF230_IRQ_TRX_UR | RF230_IRQ_PLL_LOCK) | RF230_IRQ_TRX_END) | RF230_IRQ_RX_START);
  RF230DriverLayerP__writeRegister(RF230_CCA_THRES, RF230_CCA_THRES_VALUE);
  RF230DriverLayerP__writeRegister(RF230_PHY_TX_PWR, RF230_TX_AUTO_CRC_ON | (0 & RF230_TX_PWR_MASK));

  RF230DriverLayerP__txPower = 0 & RF230_TX_PWR_MASK;
  RF230DriverLayerP__channel = 11 & RF230_CHANNEL_MASK;
  RF230DriverLayerP__writeRegister(RF230_PHY_CC_CCA, RF230_CCA_MODE_VALUE | RF230DriverLayerP__channel);

  RF230DriverLayerP__SLP_TR__set();
  RF230DriverLayerP__state = RF230DriverLayerP__STATE_SLEEP;
}



static inline void RF230DriverLayerP__SpiResource__granted(void )
{
  RF230DriverLayerP__SELN__makeOutput();
  RF230DriverLayerP__SELN__set();

  if (RF230DriverLayerP__state == RF230DriverLayerP__STATE_P_ON) 
    {
      RF230DriverLayerP__initRadio();
      RF230DriverLayerP__SpiResource__release();
    }
  else {
    RF230DriverLayerP__Tasklet__schedule();
    }
}

# 374 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
static inline void Atm128SpiP__Resource__default__granted(uint8_t id)
#line 374
{
}

# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static void Atm128SpiP__Resource__granted(uint8_t arg_0xf6e15398){
#line 92
  switch (arg_0xf6e15398) {
#line 92
    case 0U:
#line 92
      RF230DriverLayerP__SpiResource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      Atm128SpiP__Resource__default__granted(arg_0xf6e15398);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 370 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
static inline void Atm128SpiP__ResourceArbiter__granted(uint8_t id)
#line 370
{
  Atm128SpiP__Resource__granted(id);
}

# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Resource__granted(uint8_t arg_0xf71d8d90){
#line 92
  Atm128SpiP__ResourceArbiter__granted(arg_0xf71d8d90);
#line 92
}
#line 92
# 171 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceConfigure__default__configure(uint8_t id)
#line 171
{
}

# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceConfigure__configure(uint8_t arg_0xf71c5430){
#line 49
    /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceConfigure__default__configure(arg_0xf71c5430);
#line 49
}
#line 49
# 155 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__grantedTask__runTask(void )
#line 155
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 156
    {
      /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__resId = /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__reqResId;
      /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__state = /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__RES_BUSY;
    }
#line 159
    __nesc_atomic_end(__nesc_atomic); }
  /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceConfigure__configure(/*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__resId);
  /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Resource__granted(/*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__resId);
}










static inline void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceConfigure__default__unconfigure(uint8_t id)
#line 173
{
}

# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceConfigure__unconfigure(uint8_t arg_0xf71c5430){
#line 55
    /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceConfigure__default__unconfigure(arg_0xf71c5430);
#line 55
}
#line 55
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__grantedTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__grantedTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 58 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    if (/*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__qHead != /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
        uint8_t id = /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__qHead;

#line 62
        /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__qHead = /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__resQ[/*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__qHead];
        if (/*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__qHead == /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
          /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__qTail = /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
          }
#line 65
        /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__resQ[id] = /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
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
      /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

#line 68
      return __nesc_temp;
    }
  }
}

# 60 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Queue__dequeue(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 50 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 51
  {
    unsigned char __nesc_temp = 
#line 51
    /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__qHead == /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

#line 51
    return __nesc_temp;
  }
}

# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static bool /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Queue__isEmpty(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 97 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline error_t /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Resource__release(uint8_t id)
#line 97
{
  bool released = FALSE;

#line 99
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 99
    {
      if (/*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__state == /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__RES_BUSY && /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__resId == id) {
          if (/*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Queue__isEmpty() == FALSE) {
              /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__resId = /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__NO_RES;
              /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__reqResId = /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Queue__dequeue();
              /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__state = /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__RES_GRANTING;
              /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__grantedTask__postTask();
            }
          else {
              /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__resId = /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__NO_RES;
              /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__state = /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__RES_IDLE;
            }
          released = TRUE;
        }
    }
#line 113
    __nesc_atomic_end(__nesc_atomic); }
  if (released == TRUE) {
      /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceConfigure__unconfigure(id);
      return SUCCESS;
    }
  return FAIL;
}

# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t Atm128SpiP__ResourceArbiter__release(uint8_t arg_0xf6e13068){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Resource__release(arg_0xf6e13068);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 80 "/opt/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
inline static bool Atm128SpiP__ArbiterInfo__inUse(void ){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ArbiterInfo__inUse();
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 44 "/opt/tinyos-2.x/tos/interfaces/McuPowerState.nc"
inline static void Atm128SpiP__McuPowerState__update(void ){
#line 44
  McuSleepC__McuPowerState__update();
#line 44
}
#line 44
# 94 "/opt/tinyos-2.x/tos/chips/atm128/spi/HplAtm128SpiP.nc"
static inline void HplAtm128SpiP__SPI__sleep(void )
#line 94
{
}

# 72 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128Spi.nc"
inline static void Atm128SpiP__Spi__sleep(void ){
#line 72
  HplAtm128SpiP__SPI__sleep();
#line 72
}
#line 72
#line 99
inline static void Atm128SpiP__Spi__enableSpi(bool busOn){
#line 99
  HplAtm128SpiP__SPI__enableSpi(busOn);
#line 99
}
#line 99
# 120 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
static inline void Atm128SpiP__stopSpi(void )
#line 120
{
  Atm128SpiP__Spi__enableSpi(FALSE);
  /* atomic removed: atomic calls only */
#line 122
  {
    Atm128SpiP__Spi__sleep();
  }
  Atm128SpiP__McuPowerState__update();
}

# 44 "/opt/tinyos-2.x/tos/interfaces/McuPowerState.nc"
inline static void HplAtm128SpiP__Mcu__update(void ){
#line 44
  McuSleepC__McuPowerState__update();
#line 44
}
#line 44
# 280 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline void RF230RadioP__RadioAlarm__fired(void )
{
}

# 45 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioSend.nc"
inline static void RandomCollisionLayerP__RadioSend__sendDone(error_t error){
#line 45
  MessageBufferLayerP__RadioSend__sendDone(error);
#line 45
}
#line 45
# 259 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline uint16_t RF230RadioP__RandomCollisionConfig__getCongestionBackoff(message_t *msg)
{
  return (uint16_t )(2240 * (73728UL / 8 / 32) * (1 << MICA_DIVIDE_ONE_FOR_32KHZ_LOG2) / 10000UL);
}

# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/RandomCollisionConfig.nc"
inline static uint16_t RandomCollisionLayerP__Config__getCongestionBackoff(message_t *msg){
#line 35
  unsigned short __nesc_result;
#line 35

#line 35
  __nesc_result = RF230RadioP__RandomCollisionConfig__getCongestionBackoff(msg);
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 62 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MetadataFlagsLayerC.nc"
static inline void MetadataFlagsLayerC__PacketFlag__clear(uint8_t bit, message_t *msg)
{
  for (; 0; ) ;

  MetadataFlagsLayerC__getMeta(msg)->flags &= ~(1 << bit);
}

# 44 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketFlag.nc"
inline static void SoftwareAckLayerP__AckReceivedFlag__clear(message_t *msg){
#line 44
  MetadataFlagsLayerC__PacketFlag__clear(0U, msg);
#line 44
}
#line 44
# 37 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioSend.nc"
inline static error_t SoftwareAckLayerP__SubSend__send(message_t *msg){
#line 37
  unsigned char __nesc_result;
#line 37

#line 37
  __nesc_result = RF230DriverLayerP__RadioSend__send(msg);
#line 37

#line 37
  return __nesc_result;
#line 37
}
#line 37
# 67 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/SoftwareAckLayerP.nc"
static inline error_t SoftwareAckLayerP__RadioSend__send(message_t *msg)
{
  error_t error;

  if (SoftwareAckLayerP__state == SoftwareAckLayerP__STATE_READY) 
    {
      if ((error = SoftwareAckLayerP__SubSend__send(msg)) == SUCCESS) 
        {
          SoftwareAckLayerP__AckReceivedFlag__clear(msg);
          SoftwareAckLayerP__state = SoftwareAckLayerP__STATE_DATA_SEND;
          SoftwareAckLayerP__txMsg = msg;
        }
    }
  else {
    error = EBUSY;
    }
  return error;
}

# 37 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioSend.nc"
inline static error_t RandomCollisionLayerP__SubSend__send(message_t *msg){
#line 37
  unsigned char __nesc_result;
#line 37

#line 37
  __nesc_result = SoftwareAckLayerP__RadioSend__send(msg);
#line 37

#line 37
  return __nesc_result;
#line 37
}
#line 37
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t MessageBufferLayerP__sendTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(MessageBufferLayerP__sendTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 222 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MessageBufferLayerP.nc"
static inline void MessageBufferLayerP__RadioSend__ready(void )
{
  if (MessageBufferLayerP__state == MessageBufferLayerP__STATE_TX_PENDING) {
    MessageBufferLayerP__sendTask__postTask();
    }
}

# 52 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioSend.nc"
inline static void RandomCollisionLayerP__RadioSend__ready(void ){
#line 52
  MessageBufferLayerP__RadioSend__ready();
#line 52
}
#line 52
# 70 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer1P.nc"
static inline uint16_t HplAtm1281Timer1P__Timer__get(void )
#line 70
{
#line 70
  return * (volatile uint16_t *)0x84;
}

# 52 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
inline static /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Timer__timer_size /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Timer__get(void ){
#line 52
  unsigned short __nesc_result;
#line 52

#line 52
  __nesc_result = HplAtm1281Timer1P__Timer__get();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 53 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128AlarmC.nc"
static inline /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__timer_size /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__getNow(void )
#line 53
{
  return /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Timer__get();
}

# 98 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static RadioAlarmP__Alarm__size_type RadioAlarmP__Alarm__getNow(void ){
#line 98
  unsigned short __nesc_result;
#line 98

#line 98
  __nesc_result = /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__getNow();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 64 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarmP.nc"
static __inline uint16_t RadioAlarmP__RadioAlarm__getNow(uint8_t id)
{
  return RadioAlarmP__Alarm__getNow();
}

# 53 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
inline static uint16_t RandomCollisionLayerP__RadioAlarm__getNow(void ){
#line 53
  unsigned short __nesc_result;
#line 53

#line 53
  __nesc_result = RadioAlarmP__RadioAlarm__getNow(1U);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 99 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/RandomCollisionLayerP.nc"
static inline void RandomCollisionLayerP__RadioAlarm__fired(void )
{
  error_t error;
  int16_t delay;

  for (; 0; ) ;

  delay = (int16_t )RandomCollisionLayerP__txBarrier - RandomCollisionLayerP__RadioAlarm__getNow();

  if (RandomCollisionLayerP__state == RandomCollisionLayerP__STATE_BARRIER) 
    {
      RandomCollisionLayerP__state = RandomCollisionLayerP__STATE_READY;

      RandomCollisionLayerP__RadioSend__ready();
      return;
    }
  else {
#line 115
    if (RandomCollisionLayerP__state & RandomCollisionLayerP__STATE_BARRIER && delay > 0) {
      error = EBUSY;
      }
    else {
#line 118
      error = RandomCollisionLayerP__SubSend__send(RandomCollisionLayerP__txMsg);
      }
    }
#line 120
  if (error != SUCCESS) 
    {
      if ((RandomCollisionLayerP__state & ~RandomCollisionLayerP__STATE_BARRIER) == RandomCollisionLayerP__STATE_TX_PENDING_FIRST) 
        {
          RandomCollisionLayerP__state = (RandomCollisionLayerP__state & RandomCollisionLayerP__STATE_BARRIER) | RandomCollisionLayerP__STATE_TX_PENDING_SECOND;
          RandomCollisionLayerP__RadioAlarm__wait(RandomCollisionLayerP__getBackoff(RandomCollisionLayerP__Config__getCongestionBackoff(RandomCollisionLayerP__txMsg)));
        }
      else 
        {
          if (RandomCollisionLayerP__state & RandomCollisionLayerP__STATE_BARRIER && delay > 0) 
            {
              RandomCollisionLayerP__state = RandomCollisionLayerP__STATE_BARRIER;
              RandomCollisionLayerP__RadioAlarm__wait(delay);
            }
          else {
            RandomCollisionLayerP__state = RandomCollisionLayerP__STATE_READY;
            }
          RandomCollisionLayerP__RadioSend__sendDone(error);
        }
    }
  else {
    RandomCollisionLayerP__state = RandomCollisionLayerP__STATE_TX_SENDING;
    }
}

#line 144
static inline void RandomCollisionLayerP__SubSend__sendDone(error_t error)
{
  for (; 0; ) ;

  RandomCollisionLayerP__state = RandomCollisionLayerP__STATE_READY;
  RandomCollisionLayerP__RadioSend__sendDone(error);
}

# 45 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioSend.nc"
inline static void SoftwareAckLayerP__RadioSend__sendDone(error_t error){
#line 45
  RandomCollisionLayerP__SubSend__sendDone(error);
#line 45
}
#line 45
# 129 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline void RF230RadioP__SoftwareAckConfig__reportChannelError(void )
{
}

# 75 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/SoftwareAckConfig.nc"
inline static void SoftwareAckLayerP__SoftwareAckConfig__reportChannelError(void ){
#line 75
  RF230RadioP__SoftwareAckConfig__reportChannelError();
#line 75
}
#line 75
# 113 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/SoftwareAckLayerP.nc"
static inline void SoftwareAckLayerP__RadioAlarm__fired(void )
{
  for (; 0; ) ;

  SoftwareAckLayerP__SoftwareAckConfig__reportChannelError();

  SoftwareAckLayerP__state = SoftwareAckLayerP__STATE_READY;
  SoftwareAckLayerP__RadioSend__sendDone(SUCCESS);
}

# 575 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static inline void RF230DriverLayerP__RadioCCA__default__done(error_t error)
#line 575
{
}

# 41 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioCCA.nc"
inline static void RF230DriverLayerP__RadioCCA__done(error_t error){
#line 41
  RF230DriverLayerP__RadioCCA__default__done(error);
#line 41
}
#line 41
# 155 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static __inline uint8_t RF230DriverLayerP__readRegister(uint8_t reg)
{
  for (; 0; ) ;
  for (; 0; ) ;

  RF230DriverLayerP__SELN__clr();
  RF230DriverLayerP__FastSpiByte__splitWrite(RF230_CMD_REGISTER_READ | reg);
  RF230DriverLayerP__FastSpiByte__splitReadWrite(0);
  reg = RF230DriverLayerP__FastSpiByte__splitRead();
  RF230DriverLayerP__SELN__set();

  return reg;
}

#line 180
static inline void RF230DriverLayerP__RadioAlarm__fired(void )
{
  if (RF230DriverLayerP__state == RF230DriverLayerP__STATE_SLEEP_2_TRX_OFF) {
    RF230DriverLayerP__state = RF230DriverLayerP__STATE_TRX_OFF;
    }
  else {
#line 184
    if (RF230DriverLayerP__cmd == RF230DriverLayerP__CMD_CCA) 
      {
        uint8_t cca;

        for (; 0; ) ;

        RF230DriverLayerP__cmd = RF230DriverLayerP__CMD_NONE;
        cca = RF230DriverLayerP__readRegister(RF230_TRX_STATUS);

        for (; 0; ) ;

        RF230DriverLayerP__RadioCCA__done(cca & RF230_CCA_DONE ? cca & RF230_CCA_STATUS ? SUCCESS : EBUSY : FAIL);
      }
    else {
      for (; 0; ) ;
      }
    }
  RF230DriverLayerP__Tasklet__schedule();
}

# 78 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarmP.nc"
static inline void RadioAlarmP__RadioAlarm__default__fired(uint8_t id)
{
}

# 48 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
inline static void RadioAlarmP__RadioAlarm__fired(uint8_t arg_0xf72599e0){
#line 48
  switch (arg_0xf72599e0) {
#line 48
    case 0U:
#line 48
      RF230RadioP__RadioAlarm__fired();
#line 48
      break;
#line 48
    case 1U:
#line 48
      RandomCollisionLayerP__RadioAlarm__fired();
#line 48
      break;
#line 48
    case 2U:
#line 48
      SoftwareAckLayerP__RadioAlarm__fired();
#line 48
      break;
#line 48
    case 3U:
#line 48
      RF230DriverLayerP__RadioAlarm__fired();
#line 48
      break;
#line 48
    default:
#line 48
      RadioAlarmP__RadioAlarm__default__fired(arg_0xf72599e0);
#line 48
      break;
#line 48
    }
#line 48
}
#line 48
# 69 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarmP.nc"
static inline void RadioAlarmP__Tasklet__run(void )
{
  if (RadioAlarmP__state == RadioAlarmP__STATE_FIRED) 
    {
      RadioAlarmP__state = RadioAlarmP__STATE_READY;
      RadioAlarmP__RadioAlarm__fired(RadioAlarmP__alarm);
    }
}

# 228 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MessageBufferLayerP.nc"
static inline void MessageBufferLayerP__Tasklet__run(void )
{
}

# 82 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarmP.nc"
static __inline bool RadioAlarmP__RadioAlarm__isFree(uint8_t id)
{
  return RadioAlarmP__state == RadioAlarmP__STATE_READY;
}

# 33 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
inline static bool RandomCollisionLayerP__RadioAlarm__isFree(void ){
#line 33
  unsigned char __nesc_result;
#line 33

#line 33
  __nesc_result = RadioAlarmP__RadioAlarm__isFree(1U);
#line 33

#line 33
  return __nesc_result;
#line 33
}
#line 33
# 60 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/RandomCollisionLayerP.nc"
static inline void RandomCollisionLayerP__SubSend__ready(void )
{
  if (RandomCollisionLayerP__state == RandomCollisionLayerP__STATE_READY && RandomCollisionLayerP__RadioAlarm__isFree()) {
    RandomCollisionLayerP__RadioSend__ready();
    }
}

# 52 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioSend.nc"
inline static void SoftwareAckLayerP__RadioSend__ready(void ){
#line 52
  RandomCollisionLayerP__SubSend__ready();
#line 52
}
#line 52
# 61 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/SoftwareAckLayerP.nc"
static inline void SoftwareAckLayerP__SubSend__ready(void )
{
  if (SoftwareAckLayerP__state == SoftwareAckLayerP__STATE_READY) {
    SoftwareAckLayerP__RadioSend__ready();
    }
}

# 52 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioSend.nc"
inline static void RF230DriverLayerP__RadioSend__ready(void ){
#line 52
  SoftwareAckLayerP__SubSend__ready();
#line 52
}
#line 52
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t MessageBufferLayerP__stateDoneTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(MessageBufferLayerP__stateDoneTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 144 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MessageBufferLayerP.nc"
static inline void MessageBufferLayerP__RadioState__done(void )
{
  MessageBufferLayerP__stateDoneTask__postTask();
}

# 58 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioState.nc"
inline static void RF230DriverLayerP__RadioState__done(void ){
#line 58
  MessageBufferLayerP__RadioState__done();
#line 58
}
#line 58
# 312 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static __inline void RF230DriverLayerP__changeChannel(void )
{
  for (; 0; ) ;
  for (; 0; ) ;

  if (RF230DriverLayerP__isSpiAcquired()) 
    {
      RF230DriverLayerP__writeRegister(RF230_PHY_CC_CCA, RF230_CCA_MODE_VALUE | RF230DriverLayerP__channel);

      if (RF230DriverLayerP__state == RF230DriverLayerP__STATE_RX_ON) {
        RF230DriverLayerP__state = RF230DriverLayerP__STATE_TRX_OFF_2_RX_ON;
        }
      else {
#line 324
        RF230DriverLayerP__cmd = RF230DriverLayerP__CMD_SIGNAL_DONE;
        }
    }
}

# 78 "/opt/tinyos-2.x/tos/platforms/iris/chips/rf230/HplRF230P.nc"
static inline void HplRF230P__IRQ__disable(void )
{
  HplRF230P__Capture__stop();
}

# 55 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
inline static void RF230DriverLayerP__IRQ__disable(void ){
#line 55
  HplRF230P__IRQ__disable();
#line 55
}
#line 55
# 145 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer1P.nc"
static inline void HplAtm1281Timer1P__Capture__start(void )
#line 145
{
#line 145
  * (volatile uint8_t *)0x6F |= 1 << 5;
}

# 58 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Capture.nc"
inline static void HplRF230P__Capture__start(void ){
#line 58
  HplAtm1281Timer1P__Capture__start();
#line 58
}
#line 58
# 139 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer1P.nc"
static inline void HplAtm1281Timer1P__Capture__reset(void )
#line 139
{
#line 139
  * (volatile uint8_t *)(0x16 + 0x20) = 1 << 5;
}

# 55 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Capture.nc"
inline static void HplRF230P__Capture__reset(void ){
#line 55
  HplAtm1281Timer1P__Capture__reset();
#line 55
}
#line 55
# 135 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer1P.nc"
static inline void HplAtm1281Timer1P__Capture__setEdge(bool up)
#line 135
{
#line 135
  if (up) {
#line 135
    * (volatile uint8_t *)0x81 |= 1 << 6;
    }
  else {
#line 135
    * (volatile uint8_t *)0x81 &= ~(1 << 6);
    }
}

# 79 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Capture.nc"
inline static void HplRF230P__Capture__setEdge(bool up){
#line 79
  HplAtm1281Timer1P__Capture__setEdge(up);
#line 79
}
#line 79
# 63 "/opt/tinyos-2.x/tos/platforms/iris/chips/rf230/HplRF230P.nc"
static inline error_t HplRF230P__IRQ__captureRisingEdge(void )
{
  HplRF230P__Capture__setEdge(TRUE);
  HplRF230P__Capture__reset();
  HplRF230P__Capture__start();

  return SUCCESS;
}

# 42 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
inline static error_t RF230DriverLayerP__IRQ__captureRisingEdge(void ){
#line 42
  unsigned char __nesc_result;
#line 42

#line 42
  __nesc_result = HplRF230P__IRQ__captureRisingEdge();
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 69 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128AlarmC.nc"
static inline void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__start(/*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__timer_size dt)
{
  /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__startAt(/*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Timer__get(), dt);
}

# 55 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void RadioAlarmP__Alarm__start(RadioAlarmP__Alarm__size_type dt){
#line 55
  /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__start(dt);
#line 55
}
#line 55
# 87 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarmP.nc"
static inline void RadioAlarmP__RadioAlarm__wait(uint8_t id, uint16_t timeout)
{
  for (; 0; ) ;

  RadioAlarmP__alarm = id;
  RadioAlarmP__state = RadioAlarmP__STATE_WAIT;
  RadioAlarmP__Alarm__start(timeout);
}

# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
inline static void RF230DriverLayerP__RadioAlarm__wait(uint16_t timeout){
#line 38
  RadioAlarmP__RadioAlarm__wait(3U, timeout);
#line 38
}
#line 38
#line 33
inline static bool RF230DriverLayerP__RadioAlarm__isFree(void ){
#line 33
  unsigned char __nesc_result;
#line 33

#line 33
  __nesc_result = RadioAlarmP__RadioAlarm__isFree(3U);
#line 33

#line 33
  return __nesc_result;
#line 33
}
#line 33
# 330 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static __inline void RF230DriverLayerP__changeState(void )
{

  if ((
#line 332
  RF230DriverLayerP__cmd == RF230DriverLayerP__CMD_STANDBY || RF230DriverLayerP__cmd == RF230DriverLayerP__CMD_TURNON)
   && RF230DriverLayerP__state == RF230DriverLayerP__STATE_SLEEP && RF230DriverLayerP__RadioAlarm__isFree()) 
    {
      RF230DriverLayerP__SLP_TR__clr();

      RF230DriverLayerP__RadioAlarm__wait(RF230DriverLayerP__SLEEP_WAKEUP_TIME);
      RF230DriverLayerP__state = RF230DriverLayerP__STATE_SLEEP_2_TRX_OFF;
    }
  else {
#line 340
    if (RF230DriverLayerP__cmd == RF230DriverLayerP__CMD_TURNON && RF230DriverLayerP__state == RF230DriverLayerP__STATE_TRX_OFF && RF230DriverLayerP__isSpiAcquired()) 
      {
        for (; 0; ) ;

        RF230DriverLayerP__readRegister(RF230_IRQ_STATUS);
        RF230DriverLayerP__IRQ__captureRisingEdge();


        RF230DriverLayerP__writeRegister(RF230_PHY_CC_CCA, RF230_CCA_MODE_VALUE | RF230DriverLayerP__channel);

        RF230DriverLayerP__writeRegister(RF230_TRX_STATE, RF230_RX_ON);
        RF230DriverLayerP__state = RF230DriverLayerP__STATE_TRX_OFF_2_RX_ON;
      }
    else {
      if ((
#line 353
      RF230DriverLayerP__cmd == RF230DriverLayerP__CMD_TURNOFF || RF230DriverLayerP__cmd == RF230DriverLayerP__CMD_STANDBY)
       && RF230DriverLayerP__state == RF230DriverLayerP__STATE_RX_ON && RF230DriverLayerP__isSpiAcquired()) 
        {
          RF230DriverLayerP__writeRegister(RF230_TRX_STATE, RF230_FORCE_TRX_OFF);

          RF230DriverLayerP__IRQ__disable();
          RF230DriverLayerP__radioIrq = FALSE;

          RF230DriverLayerP__state = RF230DriverLayerP__STATE_TRX_OFF;
        }
      }
    }
#line 364
  if (RF230DriverLayerP__cmd == RF230DriverLayerP__CMD_TURNOFF && RF230DriverLayerP__state == RF230DriverLayerP__STATE_TRX_OFF) 
    {
      RF230DriverLayerP__SLP_TR__set();
      RF230DriverLayerP__state = RF230DriverLayerP__STATE_SLEEP;
      RF230DriverLayerP__cmd = RF230DriverLayerP__CMD_SIGNAL_DONE;
    }
  else {
#line 370
    if (RF230DriverLayerP__cmd == RF230DriverLayerP__CMD_STANDBY && RF230DriverLayerP__state == RF230DriverLayerP__STATE_TRX_OFF) {
      RF230DriverLayerP__cmd = RF230DriverLayerP__CMD_SIGNAL_DONE;
      }
    }
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t MessageBufferLayerP__deliverTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(MessageBufferLayerP__deliverTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 309 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MessageBufferLayerP.nc"
static inline message_t *MessageBufferLayerP__RadioReceive__receive(message_t *msg)
{
  message_t *m;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      if (MessageBufferLayerP__receiveQueueSize >= MessageBufferLayerP__RECEIVE_QUEUE_SIZE) {
        m = msg;
        }
      else {
          uint8_t index = MessageBufferLayerP__receiveQueueHead + MessageBufferLayerP__receiveQueueSize;

#line 320
          if (index >= MessageBufferLayerP__RECEIVE_QUEUE_SIZE) {
            index -= MessageBufferLayerP__RECEIVE_QUEUE_SIZE;
            }
          m = MessageBufferLayerP__receiveQueue[index];
          MessageBufferLayerP__receiveQueue[index] = msg;

          ++MessageBufferLayerP__receiveQueueSize;
          MessageBufferLayerP__deliverTask__postTask();
        }
    }
#line 329
    __nesc_atomic_end(__nesc_atomic); }

  return m;
}

# 42 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioReceive.nc"
inline static message_t *UniqueLayerP__RadioReceive__receive(message_t *msg){
#line 42
  nx_struct message_t *__nesc_result;
#line 42

#line 42
  __nesc_result = MessageBufferLayerP__RadioReceive__receive(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 152 "/opt/tinyos-2.x/tos/chips/rf2xx/util/NeighborhoodP.nc"
static __inline void NeighborhoodP__NeighborhoodFlag__set(uint8_t bit, uint8_t index)
{
  NeighborhoodP__flags[index] |= 1 << bit;
}

# 40 "/opt/tinyos-2.x/tos/chips/rf2xx/util/NeighborhoodFlag.nc"
inline static void UniqueLayerP__NeighborhoodFlag__set(uint8_t index){
#line 40
  NeighborhoodP__NeighborhoodFlag__set(0U, index);
#line 40
}
#line 40
# 153 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline void RF230RadioP__UniqueConfig__reportChannelError(void )
{
}

# 47 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/UniqueConfig.nc"
inline static void UniqueLayerP__UniqueConfig__reportChannelError(void ){
#line 47
  RF230RadioP__UniqueConfig__reportChannelError();
#line 47
}
#line 47
# 147 "/opt/tinyos-2.x/tos/chips/rf2xx/util/NeighborhoodP.nc"
static __inline bool NeighborhoodP__NeighborhoodFlag__get(uint8_t bit, uint8_t index)
{
  return NeighborhoodP__flags[index] & (1 << bit);
}

# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/NeighborhoodFlag.nc"
inline static bool UniqueLayerP__NeighborhoodFlag__get(uint8_t index){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = NeighborhoodP__NeighborhoodFlag__get(0U, index);
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 147 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
static inline uint8_t Ieee154PacketLayerP__Ieee154PacketLayer__getDSN(message_t *msg)
{
  return __nesc_ntoh_leuint8(Ieee154PacketLayerP__getHeader(msg)->dsn.data);
}

# 109 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayer.nc"
inline static uint8_t RF230RadioP__Ieee154PacketLayer__getDSN(message_t *msg){
#line 109
  unsigned char __nesc_result;
#line 109

#line 109
  __nesc_result = Ieee154PacketLayerP__Ieee154PacketLayer__getDSN(msg);
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 138 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline uint8_t RF230RadioP__UniqueConfig__getSequenceNumber(message_t *msg)
{
  return RF230RadioP__Ieee154PacketLayer__getDSN(msg);
}

# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/UniqueConfig.nc"
inline static uint8_t UniqueLayerP__UniqueConfig__getSequenceNumber(message_t *msg){
#line 31
  unsigned char __nesc_result;
#line 31

#line 31
  __nesc_result = RF230RadioP__UniqueConfig__getSequenceNumber(msg);
#line 31

#line 31
  return __nesc_result;
#line 31
}
#line 31
# 177 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
static inline uint16_t Ieee154PacketLayerP__Ieee154PacketLayer__getSrcAddr(message_t *msg)
{
  return __nesc_ntoh_leuint16(Ieee154PacketLayerP__getHeader(msg)->src.data);
}

# 140 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayer.nc"
inline static uint16_t RF230RadioP__Ieee154PacketLayer__getSrcAddr(message_t *msg){
#line 140
  unsigned short __nesc_result;
#line 140

#line 140
  __nesc_result = Ieee154PacketLayerP__Ieee154PacketLayer__getSrcAddr(msg);
#line 140

#line 140
  return __nesc_result;
#line 140
}
#line 140
# 148 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline am_addr_t RF230RadioP__UniqueConfig__getSender(message_t *msg)
{
  return RF230RadioP__Ieee154PacketLayer__getSrcAddr(msg);
}

# 36 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/UniqueConfig.nc"
inline static am_addr_t UniqueLayerP__UniqueConfig__getSender(message_t *msg){
#line 36
  unsigned short __nesc_result;
#line 36

#line 36
  __nesc_result = RF230RadioP__UniqueConfig__getSender(msg);
#line 36

#line 36
  return __nesc_result;
#line 36
}
#line 36
# 105 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/UniqueLayerP.nc"
static inline void UniqueLayerP__Neighborhood__evicted(uint8_t index)
#line 105
{
}

# 69 "/opt/tinyos-2.x/tos/chips/rf2xx/util/Neighborhood.nc"
inline static void NeighborhoodP__Neighborhood__evicted(uint8_t index){
#line 69
  UniqueLayerP__Neighborhood__evicted(index);
#line 69
}
#line 69
# 83 "/opt/tinyos-2.x/tos/chips/rf2xx/util/NeighborhoodP.nc"
static inline uint8_t NeighborhoodP__Neighborhood__insertNode(am_addr_t node)
{
  uint8_t i;
  uint8_t maxAge;

  if (NeighborhoodP__nodes[NeighborhoodP__last] == node) 
    {
      if (NeighborhoodP__ages[NeighborhoodP__last] == NeighborhoodP__time) {
        return NeighborhoodP__last;
        }
      NeighborhoodP__ages[NeighborhoodP__last] = ++NeighborhoodP__time;
      maxAge = 0x80;
    }
  else 
    {
      uint8_t oldest = 0;

#line 99
      maxAge = 0;

      for (i = 0; i < 5; ++i) 
        {
          uint8_t age;

          if (NeighborhoodP__nodes[i] == node) 
            {
              NeighborhoodP__last = i;
              if (NeighborhoodP__ages[i] == NeighborhoodP__time) {
                return i;
                }
              NeighborhoodP__ages[i] = ++NeighborhoodP__time;
              maxAge = 0x80;
              break;
            }

          age = NeighborhoodP__time - NeighborhoodP__ages[i];
          if (age > maxAge) 
            {
              maxAge = age;
              oldest = i;
            }
        }

      if (i == 5) 
        {
          NeighborhoodP__Neighborhood__evicted(oldest);

          NeighborhoodP__last = oldest;
          NeighborhoodP__nodes[oldest] = node;
          NeighborhoodP__ages[oldest] = ++NeighborhoodP__time;
          NeighborhoodP__flags[oldest] = 0;
        }
    }

  if ((NeighborhoodP__time & 0x7F) == 0x7F && maxAge >= 0x7F) 
    {
      for (i = 0; i < 5; ++i) 
        {
          if ((NeighborhoodP__ages[i] | 0x7F) != NeighborhoodP__time) {
            NeighborhoodP__ages[i] = NeighborhoodP__time & 0x80;
            }
        }
    }
  return NeighborhoodP__last;
}

# 60 "/opt/tinyos-2.x/tos/chips/rf2xx/util/Neighborhood.nc"
inline static uint8_t UniqueLayerP__Neighborhood__insertNode(am_addr_t id){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = NeighborhoodP__Neighborhood__insertNode(id);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 82 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/UniqueLayerP.nc"
static inline message_t *UniqueLayerP__SubReceive__receive(message_t *msg)
{
  uint8_t index = UniqueLayerP__Neighborhood__insertNode(UniqueLayerP__UniqueConfig__getSender(msg));
  uint8_t dsn = UniqueLayerP__UniqueConfig__getSequenceNumber(msg);

  if (UniqueLayerP__NeighborhoodFlag__get(index)) 
    {
      uint8_t diff = dsn - UniqueLayerP__receivedNumbers[index];

      if (diff == 0) 
        {
          UniqueLayerP__UniqueConfig__reportChannelError();
          return msg;
        }
    }
  else {
    UniqueLayerP__NeighborhoodFlag__set(index);
    }
  UniqueLayerP__receivedNumbers[index] = dsn;

  return UniqueLayerP__RadioReceive__receive(msg);
}

# 42 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioReceive.nc"
inline static message_t *RandomCollisionLayerP__RadioReceive__receive(message_t *msg){
#line 42
  nx_struct message_t *__nesc_result;
#line 42

#line 42
  __nesc_result = UniqueLayerP__SubReceive__receive(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 157 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayer.nc"
inline static bool RF230RadioP__Ieee154PacketLayer__requiresAckReply(message_t *msg){
#line 157
  unsigned char __nesc_result;
#line 157

#line 157
  __nesc_result = Ieee154PacketLayerP__Ieee154PacketLayer__requiresAckReply(msg);
#line 157

#line 157
  return __nesc_result;
#line 157
}
#line 157
# 53 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
inline static uint16_t RF230RadioP__RadioAlarm__getNow(void ){
#line 53
  unsigned short __nesc_result;
#line 53

#line 53
  __nesc_result = RadioAlarmP__RadioAlarm__getNow(0U);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 264 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline uint16_t RF230RadioP__RandomCollisionConfig__getTransmitBarrier(message_t *msg)
{
  uint16_t time;


  time = RF230RadioP__RadioAlarm__getNow();


  if (RF230RadioP__Ieee154PacketLayer__requiresAckReply(msg)) {
    time += (uint16_t )(32 * (-5 + 16 + 11 + 5) * (73728UL / 8 / 32) * (1 << MICA_DIVIDE_ONE_FOR_32KHZ_LOG2) / 10000UL);
    }
  else {
#line 275
    time += (uint16_t )(32 * (-5 + 5) * (73728UL / 8 / 32) * (1 << MICA_DIVIDE_ONE_FOR_32KHZ_LOG2) / 10000UL);
    }
  return time;
}

# 46 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/RandomCollisionConfig.nc"
inline static uint16_t RandomCollisionLayerP__Config__getTransmitBarrier(message_t *msg){
#line 46
  unsigned short __nesc_result;
#line 46

#line 46
  __nesc_result = RF230RadioP__RandomCollisionConfig__getTransmitBarrier(msg);
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 157 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/RandomCollisionLayerP.nc"
static inline message_t *RandomCollisionLayerP__SubReceive__receive(message_t *msg)
{
  int16_t delay;

  RandomCollisionLayerP__txBarrier = RandomCollisionLayerP__Config__getTransmitBarrier(msg);
  delay = RandomCollisionLayerP__txBarrier - RandomCollisionLayerP__RadioAlarm__getNow();

  if (delay > 0) 
    {
      if (RandomCollisionLayerP__state == RandomCollisionLayerP__STATE_READY) 
        {
          RandomCollisionLayerP__RadioAlarm__wait(delay);
          RandomCollisionLayerP__state = RandomCollisionLayerP__STATE_BARRIER;
        }
      else {
        RandomCollisionLayerP__state |= RandomCollisionLayerP__STATE_BARRIER;
        }
    }
  return RandomCollisionLayerP__RadioReceive__receive(msg);
}

# 42 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioReceive.nc"
inline static message_t *SoftwareAckLayerP__RadioReceive__receive(message_t *msg){
#line 42
  nx_struct message_t *__nesc_result;
#line 42

#line 42
  __nesc_result = RandomCollisionLayerP__SubReceive__receive(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 104 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
static inline void Ieee154PacketLayerP__Ieee154PacketLayer__createAckReply(message_t *data, message_t *ack)
{
  ieee154_header_t *header = Ieee154PacketLayerP__getHeader(ack);

#line 107
  Ieee154PacketLayerP__SubPacket__setPayloadLength(ack, Ieee154PacketLayerP__IEEE154_ACK_FRAME_LENGTH);

  __nesc_hton_leuint16(header->fcf.data, Ieee154PacketLayerP__IEEE154_ACK_FRAME_VALUE);
  __nesc_hton_leuint8(header->dsn.data, __nesc_ntoh_leuint8(Ieee154PacketLayerP__getHeader(data)->dsn.data));
}

# 77 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayer.nc"
inline static void RF230RadioP__Ieee154PacketLayer__createAckReply(message_t *data, message_t *ack){
#line 77
  Ieee154PacketLayerP__Ieee154PacketLayer__createAckReply(data, ack);
#line 77
}
#line 77
# 115 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline void RF230RadioP__SoftwareAckConfig__createAckPacket(message_t *data, message_t *ack)
{
  RF230RadioP__Ieee154PacketLayer__createAckReply(data, ack);
}

# 69 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/SoftwareAckConfig.nc"
inline static void SoftwareAckLayerP__SoftwareAckConfig__createAckPacket(message_t *data, message_t *ack){
#line 69
  RF230RadioP__SoftwareAckConfig__createAckPacket(data, ack);
#line 69
}
#line 69
# 110 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline bool RF230RadioP__SoftwareAckConfig__requiresAckReply(message_t *msg)
{
  return RF230RadioP__Ieee154PacketLayer__requiresAckReply(msg);
}

# 64 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/SoftwareAckConfig.nc"
inline static bool SoftwareAckLayerP__SoftwareAckConfig__requiresAckReply(message_t *msg){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = RF230RadioP__SoftwareAckConfig__requiresAckReply(msg);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 55 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MetadataFlagsLayerC.nc"
static inline void MetadataFlagsLayerC__PacketFlag__set(uint8_t bit, message_t *msg)
{
  for (; 0; ) ;

  MetadataFlagsLayerC__getMeta(msg)->flags |= 1 << bit;
}








static inline void MetadataFlagsLayerC__PacketFlag__setValue(uint8_t bit, message_t *msg, bool value)
{
  if (value) {
    MetadataFlagsLayerC__PacketFlag__set(bit, msg);
    }
  else {
#line 74
    MetadataFlagsLayerC__PacketFlag__clear(bit, msg);
    }
}

# 34 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketFlag.nc"
inline static void SoftwareAckLayerP__AckReceivedFlag__setValue(message_t *msg, bool value){
#line 34
  MetadataFlagsLayerC__PacketFlag__setValue(0U, msg, value);
#line 34
}
#line 34
# 152 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer1P.nc"
static inline void HplAtm1281Timer1P__CompareA__stop(void )
#line 152
{
#line 152
  * (volatile uint8_t *)0x6F &= ~(1 << 1);
}

# 59 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
inline static void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Compare__stop(void ){
#line 59
  HplAtm1281Timer1P__CompareA__stop();
#line 59
}
#line 59
# 65 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128AlarmC.nc"
static inline void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__stop(void )
#line 65
{
  /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Compare__stop();
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void RadioAlarmP__Alarm__stop(void ){
#line 62
  /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__stop();
#line 62
}
#line 62
# 96 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarmP.nc"
static inline void RadioAlarmP__RadioAlarm__cancel(uint8_t id)
{
  for (; 0; ) ;
  for (; 0; ) ;

  RadioAlarmP__Alarm__stop();
  RadioAlarmP__state = RadioAlarmP__STATE_READY;
}

# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
inline static void SoftwareAckLayerP__RadioAlarm__cancel(void ){
#line 43
  RadioAlarmP__RadioAlarm__cancel(2U);
#line 43
}
#line 43
# 93 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
static inline bool Ieee154PacketLayerP__Ieee154PacketLayer__isAckFrame(message_t *msg)
{
  return (__nesc_ntoh_leuint16(Ieee154PacketLayerP__getHeader(msg)->fcf.data) & Ieee154PacketLayerP__IEEE154_ACK_FRAME_MASK) == Ieee154PacketLayerP__IEEE154_ACK_FRAME_VALUE;
}

# 64 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayer.nc"
inline static bool RF230RadioP__Ieee154PacketLayer__isAckFrame(message_t *msg){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = Ieee154PacketLayerP__Ieee154PacketLayer__isAckFrame(msg);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 95 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline bool RF230RadioP__SoftwareAckConfig__isAckPacket(message_t *msg)
{
  return RF230RadioP__Ieee154PacketLayer__isAckFrame(msg);
}

# 51 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/SoftwareAckConfig.nc"
inline static bool SoftwareAckLayerP__SoftwareAckConfig__isAckPacket(message_t *msg){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = RF230RadioP__SoftwareAckConfig__isAckPacket(msg);
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 131 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/SoftwareAckLayerP.nc"
static inline message_t *SoftwareAckLayerP__SubReceive__receive(message_t *msg)
{
  bool ack = SoftwareAckLayerP__SoftwareAckConfig__isAckPacket(msg);

  for (; 0; ) ;

  if (SoftwareAckLayerP__state == SoftwareAckLayerP__STATE_ACK_WAIT) 
    {
      for (; 0; ) ;

      SoftwareAckLayerP__RadioAlarm__cancel();
      SoftwareAckLayerP__AckReceivedFlag__setValue(SoftwareAckLayerP__txMsg, ack);

      SoftwareAckLayerP__state = SoftwareAckLayerP__STATE_READY;
      SoftwareAckLayerP__RadioSend__sendDone(SUCCESS);
    }

  if (ack) {
    return msg;
    }
  if (SoftwareAckLayerP__SoftwareAckConfig__requiresAckReply(msg)) 
    {
      SoftwareAckLayerP__SoftwareAckConfig__createAckPacket(msg, &SoftwareAckLayerP__ackMsg);


      if (SoftwareAckLayerP__SubSend__send(&SoftwareAckLayerP__ackMsg) == SUCCESS) {
        SoftwareAckLayerP__state = SoftwareAckLayerP__STATE_ACK_SEND;
        }
      else {
#line 159
        for (; 0; ) ;
        }
    }
  return SoftwareAckLayerP__RadioReceive__receive(msg);
}

# 42 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioReceive.nc"
inline static message_t *RF230DriverLayerP__RadioReceive__receive(message_t *msg){
#line 42
  nx_struct message_t *__nesc_result;
#line 42

#line 42
  __nesc_result = SoftwareAckLayerP__SubReceive__receive(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 988 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static inline void RF230DriverLayerP__PacketLinkQuality__set(message_t *msg, uint8_t value)
{
  RF230DriverLayerP__getMeta(msg)->lqi = value;
}

# 251 "/usr/lib/gcc/avr/4.5.3/../../../../avr/include/util/crc16.h" 3
#line 250
static __inline uint16_t 
_crc_ccitt_update(uint16_t __crc, uint8_t __data)
{
  uint16_t __ret;

   __asm volatile (
  "eor    %A0,%1""\n\t"

  "mov    __tmp_reg__,%A0""\n\t"
  "swap   %A0""\n\t"
  "andi   %A0,0xf0""\n\t"
  "eor    %A0,__tmp_reg__""\n\t"

  "mov    __tmp_reg__,%B0""\n\t"

  "mov    %B0,%A0""\n\t"

  "swap   %A0""\n\t"
  "andi   %A0,0x0f""\n\t"
  "eor    __tmp_reg__,%A0""\n\t"

  "lsr    %A0""\n\t"
  "eor    %B0,%A0""\n\t"

  "eor    %A0,%B0""\n\t"
  "lsl    %A0""\n\t"
  "lsl    %A0""\n\t"
  "lsl    %A0""\n\t"
  "eor    %A0,__tmp_reg__" : 

  "=d"(__ret) : 
  "r"(__data), "0"(__crc) : 
  "r0");

  return __ret;
}

# 71 "/opt/tinyos-2.x/tos/platforms/iris/chips/rf230/RadioConfig.h"
static __inline uint16_t RF230_CRCBYTE_COMMAND(uint16_t crc, uint8_t data)
{
  return _crc_ccitt_update(crc, data);
}

# 270 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MessageBufferLayerP.nc"
static inline bool MessageBufferLayerP__RadioReceive__header(message_t *msg)
{
  bool notFull;


  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 275
    notFull = MessageBufferLayerP__receiveQueueSize < MessageBufferLayerP__RECEIVE_QUEUE_SIZE;
#line 275
    __nesc_atomic_end(__nesc_atomic); }

  return notFull;
}

# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioReceive.nc"
inline static bool UniqueLayerP__RadioReceive__header(message_t *msg){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = MessageBufferLayerP__RadioReceive__header(msg);
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 74 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/UniqueLayerP.nc"
static inline bool UniqueLayerP__SubReceive__header(message_t *msg)
{

  return UniqueLayerP__RadioReceive__header(msg);
}

# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioReceive.nc"
inline static bool RandomCollisionLayerP__RadioReceive__header(message_t *msg){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = UniqueLayerP__SubReceive__header(msg);
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 152 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/RandomCollisionLayerP.nc"
static inline bool RandomCollisionLayerP__SubReceive__header(message_t *msg)
{
  return RandomCollisionLayerP__RadioReceive__header(msg);
}

# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioReceive.nc"
inline static bool SoftwareAckLayerP__RadioReceive__header(message_t *msg){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = RandomCollisionLayerP__SubReceive__header(msg);
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 113 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
static inline bool Ieee154PacketLayerP__Ieee154PacketLayer__verifyAckReply(message_t *data, message_t *ack)
{
  ieee154_header_t *header = Ieee154PacketLayerP__getHeader(ack);

  return __nesc_ntoh_leuint8(header->dsn.data) == __nesc_ntoh_leuint8(Ieee154PacketLayerP__getHeader(data)->dsn.data)
   && (__nesc_ntoh_leuint16(header->fcf.data) & Ieee154PacketLayerP__IEEE154_ACK_FRAME_MASK) == Ieee154PacketLayerP__IEEE154_ACK_FRAME_VALUE;
}

# 83 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayer.nc"
inline static bool RF230RadioP__Ieee154PacketLayer__verifyAckReply(message_t *data, message_t *ack){
#line 83
  unsigned char __nesc_result;
#line 83

#line 83
  __nesc_result = Ieee154PacketLayerP__Ieee154PacketLayer__verifyAckReply(data, ack);
#line 83

#line 83
  return __nesc_result;
#line 83
}
#line 83
# 100 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline bool RF230RadioP__SoftwareAckConfig__verifyAckPacket(message_t *data, message_t *ack)
{
  return RF230RadioP__Ieee154PacketLayer__verifyAckReply(data, ack);
}

# 58 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/SoftwareAckConfig.nc"
inline static bool SoftwareAckLayerP__SoftwareAckConfig__verifyAckPacket(message_t *data, message_t *ack){
#line 58
  unsigned char __nesc_result;
#line 58

#line 58
  __nesc_result = RF230RadioP__SoftwareAckConfig__verifyAckPacket(data, ack);
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 123 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/SoftwareAckLayerP.nc"
static inline bool SoftwareAckLayerP__SubReceive__header(message_t *msg)
{
  if (SoftwareAckLayerP__SoftwareAckConfig__isAckPacket(msg)) {
    return SoftwareAckLayerP__state == SoftwareAckLayerP__STATE_ACK_WAIT && SoftwareAckLayerP__SoftwareAckConfig__verifyAckPacket(SoftwareAckLayerP__txMsg, msg);
    }
  else {
#line 128
    return SoftwareAckLayerP__RadioReceive__header(msg);
    }
}

# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioReceive.nc"
inline static bool RF230DriverLayerP__RadioReceive__header(message_t *msg){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = SoftwareAckLayerP__SubReceive__header(msg);
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 77 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline uint8_t RF230RadioP__RF230DriverConfig__headerPreloadLength(void )
{

  return 7;
}

# 48 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverConfig.nc"
inline static uint8_t RF230DriverLayerP__Config__headerPreloadLength(void ){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = RF230RadioP__RF230DriverConfig__headerPreloadLength();
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 163 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
static __inline uint8_t Atm128SpiP__FastSpiByte__write(uint8_t data)
#line 163
{
  Atm128SpiP__Spi__write(data);
  while (!(* (volatile uint8_t *)(0x2D + 0x20) & 0x80)) 
    ;
  return Atm128SpiP__Spi__read();
}

# 65 "/opt/tinyos-2.x/tos/interfaces/FastSpiByte.nc"
inline static uint8_t RF230DriverLayerP__FastSpiByte__write(uint8_t data){
#line 65
  unsigned char __nesc_result;
#line 65

#line 65
  __nesc_result = Atm128SpiP__FastSpiByte__write(data);
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 579 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static __inline void RF230DriverLayerP__downloadMessage(void )
{
  uint8_t length;
  uint16_t crc;

  RF230DriverLayerP__SELN__clr();
  RF230DriverLayerP__FastSpiByte__write(RF230_CMD_FRAME_READ);


  length = RF230DriverLayerP__FastSpiByte__write(0);


  if (length >= 3 && length <= RF230DriverLayerP__RadioPacket__maxPayloadLength() + 2) 
    {
      uint8_t read;
      uint8_t *data;


      RF230DriverLayerP__FastSpiByte__splitWrite(0);

      data = RF230DriverLayerP__getPayload(RF230DriverLayerP__rxMsg);
      __nesc_hton_leuint8(RF230DriverLayerP__getHeader(RF230DriverLayerP__rxMsg)->length.data, length);
      crc = 0;


      length -= 2;

      read = RF230DriverLayerP__Config__headerPreloadLength();
      if (length < read) {
        read = length;
        }
      length -= read;

      do {
          crc = RF230_CRCBYTE_COMMAND(crc, * data++ = RF230DriverLayerP__FastSpiByte__splitReadWrite(0));
        }
      while (--read != 0);

      if (RF230DriverLayerP__RadioReceive__header(RF230DriverLayerP__rxMsg)) 
        {
          while (length-- != 0) 
            crc = RF230_CRCBYTE_COMMAND(crc, * data++ = RF230DriverLayerP__FastSpiByte__splitReadWrite(0));

          crc = RF230_CRCBYTE_COMMAND(crc, RF230DriverLayerP__FastSpiByte__splitReadWrite(0));
          crc = RF230_CRCBYTE_COMMAND(crc, RF230DriverLayerP__FastSpiByte__splitReadWrite(0));

          RF230DriverLayerP__PacketLinkQuality__set(RF230DriverLayerP__rxMsg, RF230DriverLayerP__FastSpiByte__splitRead());
        }
      else {
        crc = 1;
        }
    }
  else {
#line 631
    crc = 1;
    }
  RF230DriverLayerP__SELN__set();
  RF230DriverLayerP__state = RF230DriverLayerP__STATE_RX_ON;
#line 652
  RF230DriverLayerP__cmd = RF230DriverLayerP__CMD_NONE;


  if (crc == 0) {
    RF230DriverLayerP__rxMsg = RF230DriverLayerP__RadioReceive__receive(RF230DriverLayerP__rxMsg);
    }
}

# 124 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline uint16_t RF230RadioP__SoftwareAckConfig__getAckTimeout(void )
{
  return (uint16_t )(800 * (73728UL / 8 / 32) * (1 << MICA_DIVIDE_ONE_FOR_32KHZ_LOG2) / 10000UL);
}

# 32 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/SoftwareAckConfig.nc"
inline static uint16_t SoftwareAckLayerP__SoftwareAckConfig__getAckTimeout(void ){
#line 32
  unsigned short __nesc_result;
#line 32

#line 32
  __nesc_result = RF230RadioP__SoftwareAckConfig__getAckTimeout();
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
inline static void SoftwareAckLayerP__RadioAlarm__wait(uint16_t timeout){
#line 38
  RadioAlarmP__RadioAlarm__wait(2U, timeout);
#line 38
}
#line 38
#line 33
inline static bool SoftwareAckLayerP__RadioAlarm__isFree(void ){
#line 33
  unsigned char __nesc_result;
#line 33

#line 33
  __nesc_result = RadioAlarmP__RadioAlarm__isFree(2U);
#line 33

#line 33
  return __nesc_result;
#line 33
}
#line 33
# 121 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
static inline bool Ieee154PacketLayerP__Ieee154PacketLayer__getAckRequired(message_t *msg)
{
  return __nesc_ntoh_leuint16(Ieee154PacketLayerP__getHeader(msg)->fcf.data) & (1 << IEEE154_FCF_ACK_REQ);
}

#line 187
static inline bool Ieee154PacketLayerP__Ieee154PacketLayer__requiresAckWait(message_t *msg)
{
  return Ieee154PacketLayerP__Ieee154PacketLayer__getAckRequired(msg)
   && Ieee154PacketLayerP__Ieee154PacketLayer__isDataFrame(msg)
   && Ieee154PacketLayerP__Ieee154PacketLayer__getDestAddr(msg) != 0xFFFF;
}

# 151 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayer.nc"
inline static bool RF230RadioP__Ieee154PacketLayer__requiresAckWait(message_t *msg){
#line 151
  unsigned char __nesc_result;
#line 151

#line 151
  __nesc_result = Ieee154PacketLayerP__Ieee154PacketLayer__requiresAckWait(msg);
#line 151

#line 151
  return __nesc_result;
#line 151
}
#line 151
# 90 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline bool RF230RadioP__SoftwareAckConfig__requiresAckWait(message_t *msg)
{
  return RF230RadioP__Ieee154PacketLayer__requiresAckWait(msg);
}

# 44 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/SoftwareAckConfig.nc"
inline static bool SoftwareAckLayerP__SoftwareAckConfig__requiresAckWait(message_t *msg){
#line 44
  unsigned char __nesc_result;
#line 44

#line 44
  __nesc_result = RF230RadioP__SoftwareAckConfig__requiresAckWait(msg);
#line 44

#line 44
  return __nesc_result;
#line 44
}
#line 44
# 86 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/SoftwareAckLayerP.nc"
static inline void SoftwareAckLayerP__SubSend__sendDone(error_t error)
{
  if (SoftwareAckLayerP__state == SoftwareAckLayerP__STATE_ACK_SEND) 
    {

      for (; 0; ) ;

      SoftwareAckLayerP__state = SoftwareAckLayerP__STATE_READY;
    }
  else 
    {
      for (; 0; ) ;
      for (; 0; ) ;

      if (error == SUCCESS && SoftwareAckLayerP__SoftwareAckConfig__requiresAckWait(SoftwareAckLayerP__txMsg) && SoftwareAckLayerP__RadioAlarm__isFree()) 
        {
          SoftwareAckLayerP__RadioAlarm__wait(SoftwareAckLayerP__SoftwareAckConfig__getAckTimeout());
          SoftwareAckLayerP__state = SoftwareAckLayerP__STATE_ACK_WAIT;
        }
      else 
        {
          SoftwareAckLayerP__state = SoftwareAckLayerP__STATE_READY;
          SoftwareAckLayerP__RadioSend__sendDone(error);
        }
    }
}

# 45 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioSend.nc"
inline static void RF230DriverLayerP__RadioSend__sendDone(error_t error){
#line 45
  SoftwareAckLayerP__SubSend__sendDone(error);
#line 45
}
#line 45
# 44 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketFlag.nc"
inline static void TimeStampingLayerP__TimeStampFlag__clear(message_t *msg){
#line 44
  MetadataFlagsLayerC__PacketFlag__clear(1U, msg);
#line 44
}
#line 44
# 66 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TimeStampingLayerP.nc"
static inline void TimeStampingLayerP__PacketTimeStampRadio__clear(message_t *msg)
{
  TimeStampingLayerP__TimeStampFlag__clear(msg);
}

# 59 "/opt/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
inline static void RF230DriverLayerP__PacketTimeStamp__clear(message_t * msg){
#line 59
  TimeStampingLayerP__PacketTimeStampRadio__clear(msg);
#line 59
}
#line 59
# 71 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TimeStampingLayerP.nc"
static inline void TimeStampingLayerP__PacketTimeStampRadio__set(message_t *msg, uint32_t value)
{
  TimeStampingLayerP__TimeStampFlag__set(msg);
  TimeStampingLayerP__getMeta(msg)->timestamp = value;
}

# 67 "/opt/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
inline static void RF230DriverLayerP__PacketTimeStamp__set(message_t * msg, RF230DriverLayerP__PacketTimeStamp__size_type value){
#line 67
  TimeStampingLayerP__PacketTimeStampRadio__set(msg, value);
#line 67
}
#line 67
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__size_type /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__get(void ){
#line 53
  unsigned long __nesc_result;
#line 53

#line 53
  __nesc_result = /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 42 "/opt/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline uint32_t /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__get(void )
{
  return /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__get();
}

# 50 "/opt/tinyos-2.x/tos/lib/timer/LocalTime.nc"
inline static uint32_t RF230DriverLayerP__LocalTime__get(void ){
#line 50
  unsigned long __nesc_result;
#line 50

#line 50
  __nesc_result = /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__get();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 44 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketFlag.nc"
inline static void RF230DriverLayerP__RSSIFlag__clear(message_t *msg){
#line 44
  MetadataFlagsLayerC__PacketFlag__clear(3U, msg);
#line 44
}
#line 44
# 933 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static inline void RF230DriverLayerP__PacketRSSI__clear(message_t *msg)
{
  RF230DriverLayerP__RSSIFlag__clear(msg);
}

# 39 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketFlag.nc"
inline static void RF230DriverLayerP__RSSIFlag__set(message_t *msg){
#line 39
  MetadataFlagsLayerC__PacketFlag__set(3U, msg);
#line 39
}
#line 39





inline static void RF230DriverLayerP__TransmitPowerFlag__clear(message_t *msg){
#line 44
  MetadataFlagsLayerC__PacketFlag__clear(2U, msg);
#line 44
}
#line 44
# 938 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static inline void RF230DriverLayerP__PacketRSSI__set(message_t *msg, uint8_t value)
{

  RF230DriverLayerP__TransmitPowerFlag__clear(msg);

  RF230DriverLayerP__RSSIFlag__set(msg);
  RF230DriverLayerP__getMeta(msg)->rssi = value;
}

#line 674
static inline void RF230DriverLayerP__serviceRadio(void )
{
  if (RF230DriverLayerP__isSpiAcquired()) 
    {
      uint16_t time;
      uint32_t time32;
      uint8_t irq;
      uint8_t temp;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 683
        time = RF230DriverLayerP__capturedTime;
#line 683
        __nesc_atomic_end(__nesc_atomic); }
      RF230DriverLayerP__radioIrq = FALSE;
      irq = RF230DriverLayerP__readRegister(RF230_IRQ_STATUS);
#line 706
      if (irq & RF230_IRQ_TRX_END) 
        {
          if (irq == RF230_IRQ_TRX_END || (
          irq == (RF230_IRQ_RX_START | RF230_IRQ_TRX_END) && RF230DriverLayerP__cmd == RF230DriverLayerP__CMD_NONE)) {
            RF230DriverLayerP__PacketRSSI__set(RF230DriverLayerP__rxMsg, RF230DriverLayerP__readRegister(RF230_PHY_ED_LEVEL));
            }
          else {
#line 712
            RF230DriverLayerP__PacketRSSI__clear(RF230DriverLayerP__rxMsg);
            }
        }

      if (irq & RF230_IRQ_PLL_LOCK) 
        {
          if (RF230DriverLayerP__cmd == RF230DriverLayerP__CMD_TURNON || RF230DriverLayerP__cmd == RF230DriverLayerP__CMD_CHANNEL) 
            {
              for (; 0; ) ;

              RF230DriverLayerP__state = RF230DriverLayerP__STATE_RX_ON;
              RF230DriverLayerP__cmd = RF230DriverLayerP__CMD_SIGNAL_DONE;
            }
          else {
#line 725
            if (RF230DriverLayerP__cmd == RF230DriverLayerP__CMD_TRANSMIT) 
              {
                for (; 0; ) ;
              }
            else {
              for (; 0; ) ;
              }
            }
        }
#line 733
      if (irq & RF230_IRQ_RX_START) 
        {
          if (RF230DriverLayerP__cmd == RF230DriverLayerP__CMD_CCA) 
            {
              RF230DriverLayerP__RadioCCA__done(FAIL);
              RF230DriverLayerP__cmd = RF230DriverLayerP__CMD_NONE;
            }

          if (RF230DriverLayerP__cmd == RF230DriverLayerP__CMD_NONE) 
            {
              for (; 0; ) ;


              if (irq == RF230_IRQ_RX_START) 
                {
                  temp = RF230DriverLayerP__readRegister(RF230_PHY_RSSI) & RF230_RSSI_MASK;
                  RF230DriverLayerP__rssiBusy += temp - (RF230DriverLayerP__rssiBusy >> 2);
                }
#line 767
              if (irq == RF230_IRQ_RX_START) 
                {
                  time32 = RF230DriverLayerP__LocalTime__get();
                  time32 += (int16_t )(time - RF230DriverLayerP__RX_SFD_DELAY) - (int16_t )time32;
                  RF230DriverLayerP__PacketTimeStamp__set(RF230DriverLayerP__rxMsg, time32);
                }
              else {
                RF230DriverLayerP__PacketTimeStamp__clear(RF230DriverLayerP__rxMsg);
                }
              RF230DriverLayerP__cmd = RF230DriverLayerP__CMD_RECEIVE;
            }
          else {
            for (; 0; ) ;
            }
        }
      if (irq & RF230_IRQ_TRX_END) 
        {
          if (RF230DriverLayerP__cmd == RF230DriverLayerP__CMD_TRANSMIT) 
            {
              for (; 0; ) ;

              RF230DriverLayerP__state = RF230DriverLayerP__STATE_RX_ON;
              RF230DriverLayerP__cmd = RF230DriverLayerP__CMD_NONE;
              RF230DriverLayerP__RadioSend__sendDone(SUCCESS);


              for (; 0; ) ;
            }
          else {
#line 795
            if (RF230DriverLayerP__cmd == RF230DriverLayerP__CMD_RECEIVE) 
              {
                for (; 0; ) ;

                if (RF230DriverLayerP__state == RF230DriverLayerP__STATE_PLL_ON_2_RX_ON) 
                  {
                    for (; 0; ) ;

                    RF230DriverLayerP__writeRegister(RF230_TRX_STATE, RF230_RX_ON);
                    RF230DriverLayerP__state = RF230DriverLayerP__STATE_RX_ON;
                  }
                else 
                  {

                    RF230DriverLayerP__rssiClear += (RF230DriverLayerP__readRegister(RF230_PHY_RSSI) & RF230_RSSI_MASK) - (RF230DriverLayerP__rssiClear >> 2);
                  }

                RF230DriverLayerP__cmd = RF230DriverLayerP__CMD_DOWNLOAD;
              }
            else {
              for (; 0; ) ;
              }
            }
        }
    }
}











static inline void RF230DriverLayerP__Tasklet__run(void )
{
  if (RF230DriverLayerP__radioIrq) {
    RF230DriverLayerP__serviceRadio();
    }
  if (RF230DriverLayerP__cmd != RF230DriverLayerP__CMD_NONE) 
    {
      if (RF230DriverLayerP__cmd == RF230DriverLayerP__CMD_DOWNLOAD) {
        RF230DriverLayerP__downloadMessage();
        }
      else {
#line 841
        if (RF230DriverLayerP__CMD_TURNOFF <= RF230DriverLayerP__cmd && RF230DriverLayerP__cmd <= RF230DriverLayerP__CMD_TURNON) {
          RF230DriverLayerP__changeState();
          }
        else {
#line 843
          if (RF230DriverLayerP__cmd == RF230DriverLayerP__CMD_CHANNEL) {
            RF230DriverLayerP__changeChannel();
            }
          }
        }
#line 846
      if (RF230DriverLayerP__cmd == RF230DriverLayerP__CMD_SIGNAL_DONE) 
        {
          RF230DriverLayerP__cmd = RF230DriverLayerP__CMD_NONE;
          RF230DriverLayerP__RadioState__done();
        }
    }

  if (RF230DriverLayerP__cmd == RF230DriverLayerP__CMD_NONE && RF230DriverLayerP__state == RF230DriverLayerP__STATE_RX_ON && !RF230DriverLayerP__radioIrq) {
    RF230DriverLayerP__RadioSend__ready();
    }
  if (RF230DriverLayerP__cmd == RF230DriverLayerP__CMD_NONE) {
    RF230DriverLayerP__SpiResource__release();
    }
}

# 37 "/opt/tinyos-2.x/tos/chips/rf2xx/util/Tasklet.nc"
inline static void TaskletC__Tasklet__run(void ){
#line 37
  RF230DriverLayerP__Tasklet__run();
#line 37
  MessageBufferLayerP__Tasklet__run();
#line 37
  RadioAlarmP__Tasklet__run();
#line 37
}
#line 37
# 148 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline uint8_t /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Resource__isOwner(uint8_t id)
#line 148
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 149
    {
      if (/*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__resId == id && /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__state == /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__RES_BUSY) {
          unsigned char __nesc_temp = 
#line 150
          TRUE;

          {
#line 150
            __nesc_atomic_end(__nesc_atomic); 
#line 150
            return __nesc_temp;
          }
        }
      else 
#line 151
        {
          unsigned char __nesc_temp = 
#line 151
          FALSE;

          {
#line 151
            __nesc_atomic_end(__nesc_atomic); 
#line 151
            return __nesc_temp;
          }
        }
    }
#line 154
    __nesc_atomic_end(__nesc_atomic); }
}

# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static bool Atm128SpiP__ResourceArbiter__isOwner(uint8_t arg_0xf6e13068){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  __nesc_result = /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Resource__isOwner(arg_0xf6e13068);
#line 118

#line 118
  return __nesc_result;
#line 118
}
#line 118
# 366 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
static inline uint8_t Atm128SpiP__Resource__isOwner(uint8_t id)
#line 366
{
  return Atm128SpiP__ResourceArbiter__isOwner(id);
}

# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static bool RF230DriverLayerP__SpiResource__isOwner(void ){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  __nesc_result = Atm128SpiP__Resource__isOwner(0U);
#line 118

#line 118
  return __nesc_result;
#line 118
}
#line 118
# 169 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceRequested__default__immediateRequested(uint8_t id)
#line 169
{
}

# 51 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
inline static void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceRequested__immediateRequested(uint8_t arg_0xf71c77d0){
#line 51
    /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceRequested__default__immediateRequested(arg_0xf71c77d0);
#line 51
}
#line 51
# 84 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline error_t /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Resource__immediateRequest(uint8_t id)
#line 84
{
  /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceRequested__immediateRequested(/*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 86
    {
      if (/*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__state == /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__RES_IDLE) {
          /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__state = /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__RES_BUSY;
          /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__resId = id;
          /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceConfigure__configure(/*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__resId);
          {
            unsigned char __nesc_temp = 
#line 91
            SUCCESS;

            {
#line 91
              __nesc_atomic_end(__nesc_atomic); 
#line 91
              return __nesc_temp;
            }
          }
        }
#line 93
      {
        unsigned char __nesc_temp = 
#line 93
        FAIL;

        {
#line 93
          __nesc_atomic_end(__nesc_atomic); 
#line 93
          return __nesc_temp;
        }
      }
    }
#line 96
    __nesc_atomic_end(__nesc_atomic); }
}

# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t Atm128SpiP__ResourceArbiter__immediateRequest(uint8_t arg_0xf6e13068){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Resource__immediateRequest(arg_0xf6e13068);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 339 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
static inline error_t Atm128SpiP__Resource__immediateRequest(uint8_t id)
#line 339
{
  error_t result = Atm128SpiP__ResourceArbiter__immediateRequest(id);

#line 341
  if (result == SUCCESS) {
      Atm128SpiP__startSpi();
    }
  return result;
}

# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t RF230DriverLayerP__SpiResource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = Atm128SpiP__Resource__immediateRequest(0U);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 156 "/opt/tinyos-2.x/tos/chips/atm128/spi/HplAtm128SpiP.nc"
static inline void HplAtm128SpiP__SPI__setMasterBit(bool isMaster)
#line 156
{
  if (isMaster) {
      * (volatile uint8_t *)(0x2C + 0x20) |= 1 << 4;
    }
  else {
      * (volatile uint8_t *)(0x2C + 0x20) &= ~(1 << 4);
    }
}

# 52 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortB.Bit1*/HplAtm128GeneralIOPinP__9__IO__makeOutput(void )
#line 52
{
#line 52
  * (volatile uint8_t * )36U |= 1 << 1;
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void HplAtm128SpiP__SCK__makeOutput(void ){
#line 35
  /*HplAtm128GeneralIOC.PortB.Bit1*/HplAtm128GeneralIOPinP__9__IO__makeOutput();
#line 35
}
#line 35
# 50 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortB.Bit3*/HplAtm128GeneralIOPinP__11__IO__makeInput(void )
#line 50
{
#line 50
  * (volatile uint8_t * )36U &= ~(1 << 3);
}

# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void HplAtm128SpiP__MISO__makeInput(void ){
#line 33
  /*HplAtm128GeneralIOC.PortB.Bit3*/HplAtm128GeneralIOPinP__11__IO__makeInput();
#line 33
}
#line 33
# 52 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static __inline void /*HplAtm128GeneralIOC.PortB.Bit2*/HplAtm128GeneralIOPinP__10__IO__makeOutput(void )
#line 52
{
#line 52
  * (volatile uint8_t * )36U |= 1 << 2;
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void HplAtm128SpiP__MOSI__makeOutput(void ){
#line 35
  /*HplAtm128GeneralIOC.PortB.Bit2*/HplAtm128GeneralIOPinP__10__IO__makeOutput();
#line 35
}
#line 35
# 79 "/opt/tinyos-2.x/tos/chips/atm128/spi/HplAtm128SpiP.nc"
static inline void HplAtm128SpiP__SPI__initMaster(void )
#line 79
{
  HplAtm128SpiP__MOSI__makeOutput();
  HplAtm128SpiP__MISO__makeInput();
  HplAtm128SpiP__SCK__makeOutput();
  HplAtm128SpiP__SPI__setMasterBit(TRUE);
}

# 66 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128Spi.nc"
inline static void Atm128SpiP__Spi__initMaster(void ){
#line 66
  HplAtm128SpiP__SPI__initMaster();
#line 66
}
#line 66
# 213 "/opt/tinyos-2.x/tos/chips/atm128/spi/HplAtm128SpiP.nc"
static inline void HplAtm128SpiP__SPI__setMasterDoubleSpeed(bool on)
#line 213
{
  if (on) {
      * (volatile uint8_t *)(0x2D + 0x20) |= 1 << 0;
    }
  else {
      * (volatile uint8_t *)(0x2D + 0x20) &= ~(1 << 0);
    }
}

# 125 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128Spi.nc"
inline static void Atm128SpiP__Spi__setMasterDoubleSpeed(bool on){
#line 125
  HplAtm128SpiP__SPI__setMasterDoubleSpeed(on);
#line 125
}
#line 125
# 169 "/opt/tinyos-2.x/tos/chips/atm128/spi/HplAtm128SpiP.nc"
static inline void HplAtm128SpiP__SPI__setClockPolarity(bool highWhenIdle)
#line 169
{
  if (highWhenIdle) {
      * (volatile uint8_t *)(0x2C + 0x20) |= 1 << 3;
    }
  else {
      * (volatile uint8_t *)(0x2C + 0x20) &= ~(1 << 3);
    }
}

# 108 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128Spi.nc"
inline static void Atm128SpiP__Spi__setClockPolarity(bool highWhenIdle){
#line 108
  HplAtm128SpiP__SPI__setClockPolarity(highWhenIdle);
#line 108
}
#line 108
# 183 "/opt/tinyos-2.x/tos/chips/atm128/spi/HplAtm128SpiP.nc"
static inline void HplAtm128SpiP__SPI__setClockPhase(bool sampleOnTrailing)
#line 183
{
  if (sampleOnTrailing) {
      * (volatile uint8_t *)(0x2C + 0x20) |= 1 << 2;
    }
  else {
      * (volatile uint8_t *)(0x2C + 0x20) &= ~(1 << 2);
    }
}

# 111 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128Spi.nc"
inline static void Atm128SpiP__Spi__setClockPhase(bool sampleOnTrailing){
#line 111
  HplAtm128SpiP__SPI__setClockPhase(sampleOnTrailing);
#line 111
}
#line 111
# 200 "/opt/tinyos-2.x/tos/chips/atm128/spi/HplAtm128SpiP.nc"
static inline void HplAtm128SpiP__SPI__setClock(uint8_t v)
#line 200
{
  v &= 1 | 0;
  * (volatile uint8_t *)(0x2C + 0x20) = (* (volatile uint8_t *)(0x2C + 0x20) & ~(1 | 0)) | v;
}

# 114 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128Spi.nc"
inline static void Atm128SpiP__Spi__setClock(uint8_t speed){
#line 114
  HplAtm128SpiP__SPI__setClock(speed);
#line 114
}
#line 114
# 54 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 54
{
  /* atomic removed: atomic calls only */
#line 55
  {
    unsigned char __nesc_temp = 
#line 55
    /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__resQ[id] != /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__NO_ENTRY || /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__qTail == id;

#line 55
    return __nesc_temp;
  }
}

#line 72
static inline error_t /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id)
#line 72
{
  /* atomic removed: atomic calls only */
#line 73
  {
    if (!/*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(id)) {
        if (/*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__qHead == /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
          /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__qHead = id;
          }
        else {
#line 78
          /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__resQ[/*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__qTail] = id;
          }
#line 79
        /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__qTail = id;
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
inline static error_t /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Queue__enqueue(resource_client_id_t id){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(id);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 167 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceRequested__default__requested(uint8_t id)
#line 167
{
}

# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
inline static void /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceRequested__requested(uint8_t arg_0xf71c77d0){
#line 43
    /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceRequested__default__requested(arg_0xf71c77d0);
#line 43
}
#line 43
# 71 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline error_t /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Resource__request(uint8_t id)
#line 71
{
  /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ResourceRequested__requested(/*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 73
    {
      if (/*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__state == /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__RES_IDLE) {
          /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__state = /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__RES_GRANTING;
          /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__reqResId = id;
          /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__grantedTask__postTask();
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
        /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Queue__enqueue(id);

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

# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t Atm128SpiP__ResourceArbiter__request(uint8_t arg_0xf6e13068){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__Resource__request(arg_0xf6e13068);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 52 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
inline static /*CounterOne16C.NCounter*/Atm128CounterC__0__Timer__timer_size /*CounterOne16C.NCounter*/Atm128CounterC__0__Timer__get(void ){
#line 52
  unsigned short __nesc_result;
#line 52

#line 52
  __nesc_result = HplAtm1281Timer1P__Timer__get();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 41 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128CounterC.nc"
static inline /*CounterOne16C.NCounter*/Atm128CounterC__0__timer_size /*CounterOne16C.NCounter*/Atm128CounterC__0__Counter__get(void )
{
  return /*CounterOne16C.NCounter*/Atm128CounterC__0__Timer__get();
}

# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__CounterFrom__size_type /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__CounterFrom__get(void ){
#line 53
  unsigned short __nesc_result;
#line 53

#line 53
  __nesc_result = /*CounterOne16C.NCounter*/Atm128CounterC__0__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 125 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer1P.nc"
static inline uint8_t HplAtm1281Timer1P__TimerCtrl__getInterruptFlag(void )
#line 125
{
  return * (volatile uint8_t *)(0x16 + 0x20);
}

#line 156
static inline bool HplAtm1281Timer1P__Timer__test(void )
#line 156
{
  return ((Atm128_TIFR_t )HplAtm1281Timer1P__TimerCtrl__getInterruptFlag()).bits.tov;
}

# 78 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
inline static bool /*CounterOne16C.NCounter*/Atm128CounterC__0__Timer__test(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = HplAtm1281Timer1P__Timer__test();
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 46 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128CounterC.nc"
static inline bool /*CounterOne16C.NCounter*/Atm128CounterC__0__Counter__isOverflowPending(void )
{
  return /*CounterOne16C.NCounter*/Atm128CounterC__0__Timer__test();
}

# 60 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static bool /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__CounterFrom__isOverflowPending(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*CounterOne16C.NCounter*/Atm128CounterC__0__Counter__isOverflowPending();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 194 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer1P.nc"
static inline void HplAtm1281Timer1P__CompareA__set(uint16_t t)
#line 194
{
#line 194
  * (volatile uint16_t *)0x88 = t;
}

# 45 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
inline static void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Compare__set(/*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Compare__size_type t){
#line 45
  HplAtm1281Timer1P__CompareA__set(t);
#line 45
}
#line 45
# 140 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer1P.nc"
static inline void HplAtm1281Timer1P__CompareA__reset(void )
#line 140
{
#line 140
  * (volatile uint8_t *)(0x16 + 0x20) = 1 << 1;
}

# 53 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
inline static void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Compare__reset(void ){
#line 53
  HplAtm1281Timer1P__CompareA__reset();
#line 53
}
#line 53
# 146 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer1P.nc"
static inline void HplAtm1281Timer1P__CompareA__start(void )
#line 146
{
#line 146
  * (volatile uint8_t *)0x6F |= 1 << 1;
}

# 56 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
inline static void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Compare__start(void ){
#line 56
  HplAtm1281Timer1P__CompareA__start();
#line 56
}
#line 56
# 50 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MetadataFlagsLayerC.nc"
static inline bool MetadataFlagsLayerC__PacketFlag__get(uint8_t bit, message_t *msg)
{
  return MetadataFlagsLayerC__getMeta(msg)->flags & (1 << bit);
}

# 29 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketFlag.nc"
inline static bool RF230DriverLayerP__TransmitPowerFlag__get(message_t *msg){
#line 29
  unsigned char __nesc_result;
#line 29

#line 29
  __nesc_result = MetadataFlagsLayerC__PacketFlag__get(2U, msg);
#line 29

#line 29
  return __nesc_result;
#line 29
}
#line 29
# 900 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static inline bool RF230DriverLayerP__PacketTransmitPower__isSet(message_t *msg)
{
  return RF230DriverLayerP__TransmitPowerFlag__get(msg);
}

static inline uint8_t RF230DriverLayerP__PacketTransmitPower__get(message_t *msg)
{
  return RF230DriverLayerP__getMeta(msg)->power;
}

# 83 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline bool RF230RadioP__RF230DriverConfig__requiresRssiCca(message_t *msg)
{
  return RF230RadioP__Ieee154PacketLayer__isDataFrame(msg);
}

# 54 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverConfig.nc"
inline static bool RF230DriverLayerP__Config__requiresRssiCca(message_t *msg){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = RF230RadioP__RF230DriverConfig__requiresRssiCca(msg);
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 29 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketFlag.nc"
inline static bool RF230DriverLayerP__TimeSyncFlag__get(message_t *msg){
#line 29
  unsigned char __nesc_result;
#line 29

#line 29
  __nesc_result = MetadataFlagsLayerC__PacketFlag__get(4U, msg);
#line 29

#line 29
  return __nesc_result;
#line 29
}
#line 29
# 949 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static inline bool RF230DriverLayerP__PacketTimeSyncOffset__isSet(message_t *msg)
{
  return RF230DriverLayerP__TimeSyncFlag__get(msg);
}

static inline uint8_t RF230DriverLayerP__PacketTimeSyncOffset__get(message_t *msg)
{
  return RF230DriverLayerP__RadioPacket__headerLength(msg) + RF230DriverLayerP__RadioPacket__payloadLength(msg) - sizeof(timesync_absolute_t );
}

# 53 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
inline static uint16_t RF230DriverLayerP__RadioAlarm__getNow(void ){
#line 53
  unsigned short __nesc_result;
#line 53

#line 53
  __nesc_result = RadioAlarmP__RadioAlarm__getNow(3U);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 301 "/usr/lib/ncc/nesc_nx.h"
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

#line 326
static __inline  int32_t __nesc_hton_int32(void * target, int32_t value)
#line 326
{
#line 326
  __nesc_hton_uint32(target, value);
#line 326
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

#line 326
static __inline  int32_t __nesc_ntoh_int32(const void * source)
#line 326
{
#line 326
  return __nesc_ntoh_uint32(source);
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t RandomCollisionLayerP__calcNextRandom__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(RandomCollisionLayerP__calcNextRandom);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 249 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline uint16_t RF230RadioP__RandomCollisionConfig__getMinimumBackoff(void )
{
  return (uint16_t )(320 * (73728UL / 8 / 32) * (1 << MICA_DIVIDE_ONE_FOR_32KHZ_LOG2) / 10000UL);
}

# 40 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/RandomCollisionConfig.nc"
inline static uint16_t RandomCollisionLayerP__Config__getMinimumBackoff(void ){
#line 40
  unsigned short __nesc_result;
#line 40

#line 40
  __nesc_result = RF230RadioP__RandomCollisionConfig__getMinimumBackoff();
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 296 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
static inline 
#line 295
void Atm128SpiP__SpiPacket__default__sendDone(
uint8_t *_txbuffer, uint8_t *_rxbuffer, 
uint16_t _length, error_t _success)
#line 297
{
}

# 71 "/opt/tinyos-2.x/tos/interfaces/SpiPacket.nc"
inline static void Atm128SpiP__SpiPacket__sendDone(uint8_t * txBuf, uint8_t * rxBuf, uint16_t len, error_t error){
#line 71
  Atm128SpiP__SpiPacket__default__sendDone(txBuf, rxBuf, len, error);
#line 71
}
#line 71
# 241 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
static inline void Atm128SpiP__zeroTask__runTask(void )
#line 241
{
  uint16_t myLen;
  uint8_t * rx;
  uint8_t * tx;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 246
    {
      myLen = Atm128SpiP__len;
      rx = Atm128SpiP__rxBuffer;
      tx = Atm128SpiP__txBuffer;
      Atm128SpiP__rxBuffer = (void *)0;
      Atm128SpiP__txBuffer = (void *)0;
      Atm128SpiP__len = 0;
      Atm128SpiP__pos = 0;
      Atm128SpiP__SpiPacket__sendDone(tx, rx, myLen, SUCCESS);
    }
#line 255
    __nesc_atomic_end(__nesc_atomic); }
}

# 78 "/opt/tinyos-2.x/tos/system/RandomMlcgC.nc"
static inline uint16_t RandomMlcgC__Random__rand16(void )
#line 78
{
  return (uint16_t )RandomMlcgC__Random__rand32();
}

# 41 "/opt/tinyos-2.x/tos/interfaces/Random.nc"
inline static uint16_t RandomCollisionLayerP__Random__rand16(void ){
#line 41
  unsigned short __nesc_result;
#line 41

#line 41
  __nesc_result = RandomMlcgC__Random__rand16();
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 67 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/RandomCollisionLayerP.nc"
static inline void RandomCollisionLayerP__calcNextRandom__runTask(void )
{
  uint16_t a = RandomCollisionLayerP__Random__rand16();

#line 70
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 70
    RandomCollisionLayerP__nextRandom = a;
#line 70
    __nesc_atomic_end(__nesc_atomic); }
}

# 85 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TinyosNetworkLayerC.nc"
static inline uint8_t TinyosNetworkLayerC__Ieee154Packet__payloadLength(message_t *msg)
{
  return TinyosNetworkLayerC__SubPacket__payloadLength(msg);
}

# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t Ieee154MessageLayerC__RadioPacket__payloadLength(message_t *msg){
#line 38
  unsigned char __nesc_result;
#line 38

#line 38
  __nesc_result = TinyosNetworkLayerC__Ieee154Packet__payloadLength(msg);
#line 38

#line 38
  return __nesc_result;
#line 38
}
#line 38
# 58 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154MessageLayerC.nc"
static inline uint8_t Ieee154MessageLayerC__Packet__payloadLength(message_t *msg)
{
  return Ieee154MessageLayerC__RadioPacket__payloadLength(msg);
}

# 80 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TinyosNetworkLayerC.nc"
static inline uint8_t TinyosNetworkLayerC__Ieee154Packet__headerLength(message_t *msg)
{
  return TinyosNetworkLayerC__SubPacket__headerLength(msg);
}

# 32 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioPacket.nc"
inline static uint8_t Ieee154MessageLayerC__RadioPacket__headerLength(message_t *msg){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = TinyosNetworkLayerC__Ieee154Packet__headerLength(msg);
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 46 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154MessageLayerC.nc"
static inline void *Ieee154MessageLayerC__getPayload(message_t *msg)
{
  return (void *)msg + Ieee154MessageLayerC__RadioPacket__headerLength(msg);
}

#line 141
static inline message_t *Ieee154MessageLayerC__Ieee154Receive__default__receive(message_t *msg, void *payload, uint8_t len)
{
  return msg;
}

# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * Ieee154MessageLayerC__Ieee154Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  __nesc_result = Ieee154MessageLayerC__Ieee154Receive__default__receive(msg, payload, len);
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
inline static am_addr_t Ieee154PacketLayerP__ActiveMessageAddress__amAddress(void ){
#line 50
  unsigned short __nesc_result;
#line 50

#line 50
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amAddress();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 207 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
static inline ieee154_saddr_t Ieee154PacketLayerP__Ieee154Packet__address(void )
{
  return Ieee154PacketLayerP__ActiveMessageAddress__amAddress();
}

static inline ieee154_saddr_t Ieee154PacketLayerP__Ieee154Packet__destination(message_t *msg)
{
  return Ieee154PacketLayerP__Ieee154PacketLayer__getDestAddr(msg);
}

#line 232
static inline bool Ieee154PacketLayerP__Ieee154Packet__isForMe(message_t *msg)
{
  ieee154_saddr_t addr = Ieee154PacketLayerP__Ieee154Packet__destination(msg);

#line 235
  return addr == Ieee154PacketLayerP__Ieee154Packet__address() || addr == IEEE154_BROADCAST_ADDR;
}

# 36 "/opt/tinyos-2.x/tos/interfaces/Ieee154Packet.nc"
inline static bool Ieee154MessageLayerC__Ieee154Packet__isForMe(message_t *msg){
#line 36
  unsigned char __nesc_result;
#line 36

#line 36
  __nesc_result = Ieee154PacketLayerP__Ieee154Packet__isForMe(msg);
#line 36

#line 36
  return __nesc_result;
#line 36
}
#line 36
# 132 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154MessageLayerC.nc"
static inline message_t *Ieee154MessageLayerC__SubReceive__receive(message_t *msg)
{
  if (Ieee154MessageLayerC__Ieee154Packet__isForMe(msg)) {
    return Ieee154MessageLayerC__Ieee154Receive__receive(msg, 
    Ieee154MessageLayerC__getPayload(msg), Ieee154MessageLayerC__Packet__payloadLength(msg));
    }
  else {
#line 138
    return msg;
    }
}

# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareReceive.nc"
inline static message_t *TinyosNetworkLayerC__Ieee154Receive__receive(message_t *msg){
#line 31
  nx_struct message_t *__nesc_result;
#line 31

#line 31
  __nesc_result = Ieee154MessageLayerC__SubReceive__receive(msg);
#line 31

#line 31
  return __nesc_result;
#line 31
}
#line 31
# 451 "../InterceptBase/BaseStationP.nc"
static inline message_t *BaseStationP__RadioSnoop__receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
#line 451
{
  return BaseStationP__receive(msg, payload, len, id);
}

# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * ActiveMessageLayerP__Snoop__receive(am_id_t arg_0xf72325b8, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  __nesc_result = BaseStationP__RadioSnoop__receive(arg_0xf72325b8, msg, payload, len);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 455 "../InterceptBase/BaseStationP.nc"
static inline message_t *BaseStationP__RadioReceive__receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
#line 455
{
  return BaseStationP__receive(msg, payload, len, id);
}

# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * ActiveMessageLayerP__Receive__receive(am_id_t arg_0xf7234f00, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  __nesc_result = BaseStationP__RadioReceive__receive(arg_0xf7234f00, msg, payload, len);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 50 "/opt/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
inline static am_addr_t ActiveMessageLayerP__ActiveMessageAddress__amAddress(void ){
#line 50
  unsigned short __nesc_result;
#line 50

#line 50
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amAddress();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 137 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayerP.nc"
static __inline am_addr_t ActiveMessageLayerP__AMPacket__address(void )
{
  return ActiveMessageLayerP__ActiveMessageAddress__amAddress();
}






static __inline bool ActiveMessageLayerP__AMPacket__isForMe(message_t *msg)
{
  am_addr_t addr = ActiveMessageLayerP__AMPacket__destination(msg);

#line 150
  return addr == ActiveMessageLayerP__AMPacket__address() || addr == AM_BROADCAST_ADDR;
}

#line 199
static inline uint8_t ActiveMessageLayerP__RadioPacket__headerLength(message_t *msg)
{
  return ActiveMessageLayerP__SubPacket__headerLength(msg) + sizeof(activemessage_header_t );
}

#line 56
static inline void *ActiveMessageLayerP__getPayload(message_t *msg)
{
  return (void *)msg + ActiveMessageLayerP__RadioPacket__headerLength(msg);
}

#line 112
static inline message_t *ActiveMessageLayerP__SubReceive__receive(message_t *msg)
{
  am_id_t id = ActiveMessageLayerP__AMPacket__type(msg);
  void *payload = ActiveMessageLayerP__getPayload(msg);
  uint8_t len = ActiveMessageLayerP__Packet__payloadLength(msg);

  msg = ActiveMessageLayerP__AMPacket__isForMe(msg) ? 
  ActiveMessageLayerP__Receive__receive(id, msg, payload, len) : 
  ActiveMessageLayerP__Snoop__receive(id, msg, payload, len);

  return msg;
}

# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareReceive.nc"
inline static message_t *TinyosNetworkLayerC__TinyosReceive__receive(message_t *msg){
#line 31
  nx_struct message_t *__nesc_result;
#line 31

#line 31
  __nesc_result = ActiveMessageLayerP__SubReceive__receive(msg);
#line 31

#line 31
  return __nesc_result;
#line 31
}
#line 31
# 211 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TinyosNetworkLayerC.nc"
static inline message_t *TinyosNetworkLayerC__SubReceive__receive(message_t *msg)
{
  if (__nesc_ntoh_leuint8(TinyosNetworkLayerC__getHeader(msg)->network.data) == 0x3f) {
    return TinyosNetworkLayerC__TinyosReceive__receive(msg);
    }
  else {
#line 216
    return TinyosNetworkLayerC__Ieee154Receive__receive(msg);
    }
}

# 31 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareReceive.nc"
inline static message_t *MessageBufferLayerP__Receive__receive(message_t *msg){
#line 31
  nx_struct message_t *__nesc_result;
#line 31

#line 31
  __nesc_result = TinyosNetworkLayerC__SubReceive__receive(msg);
#line 31

#line 31
  return __nesc_result;
#line 31
}
#line 31
# 280 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MessageBufferLayerP.nc"
static inline void MessageBufferLayerP__deliverTask__runTask(void )
{

  for (; ; ) 
    {
      message_t *msg;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          if (MessageBufferLayerP__receiveQueueSize == 0) {
            {
#line 290
              __nesc_atomic_end(__nesc_atomic); 
#line 290
              return;
            }
            }
#line 292
          msg = MessageBufferLayerP__receiveQueue[MessageBufferLayerP__receiveQueueHead];
        }
#line 293
        __nesc_atomic_end(__nesc_atomic); }

      msg = MessageBufferLayerP__Receive__receive(msg);

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          MessageBufferLayerP__receiveQueue[MessageBufferLayerP__receiveQueueHead] = msg;

          if (++MessageBufferLayerP__receiveQueueHead >= MessageBufferLayerP__RECEIVE_QUEUE_SIZE) {
            MessageBufferLayerP__receiveQueueHead = 0;
            }
          --MessageBufferLayerP__receiveQueueSize;
        }
#line 305
        __nesc_atomic_end(__nesc_atomic); }
    }
}

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

#line 288
static __inline  int16_t __nesc_hton_int16(void * target, int16_t value)
#line 288
{
#line 288
  __nesc_hton_uint16(target, value);
#line 288
  return value;
}

# 82 "RssiBaseC.nc"
static inline bool RssiBaseC__RssiMsgIntercept__forward(message_t *msg, void *payload, uint8_t len)
#line 82
{

  MultiPingResponseMsg *rssiMsg = (MultiPingResponseMsg *)payload;



  __nesc_hton_int16(rssiMsg->rssi.data, 0);

  RssiBaseC__sendBlink();


  return TRUE;
}

# 928 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static inline uint8_t RF230DriverLayerP__PacketRSSI__get(message_t *msg)
{
  return RF230DriverLayerP__getMeta(msg)->rssi;
}

# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketField.nc"
inline static RssiBaseC__PacketRSSI__value_type RssiBaseC__PacketRSSI__get(message_t *msg){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = RF230DriverLayerP__PacketRSSI__get(msg);
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 29 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketFlag.nc"
inline static bool RF230DriverLayerP__RSSIFlag__get(message_t *msg){
#line 29
  unsigned char __nesc_result;
#line 29

#line 29
  __nesc_result = MetadataFlagsLayerC__PacketFlag__get(3U, msg);
#line 29

#line 29
  return __nesc_result;
#line 29
}
#line 29
# 923 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static inline bool RF230DriverLayerP__PacketRSSI__isSet(message_t *msg)
{
  return RF230DriverLayerP__RSSIFlag__get(msg);
}

# 29 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketField.nc"
inline static bool RssiBaseC__PacketRSSI__isSet(message_t *msg){
#line 29
  unsigned char __nesc_result;
#line 29

#line 29
  __nesc_result = RF230DriverLayerP__PacketRSSI__isSet(msg);
#line 29

#line 29
  return __nesc_result;
#line 29
}
#line 29
# 199 "RssiBaseC.nc"
static inline uint16_t RssiBaseC__getRssi(message_t *msg)
#line 199
{
  if (RssiBaseC__PacketRSSI__isSet(msg)) {
    return (uint16_t )RssiBaseC__PacketRSSI__get(msg);
    }
  else {
#line 203
    return 0xFFFF;
    }
}

#line 96
static inline bool RssiBaseC__SimpleRssiMsgIntercept__forward(message_t *msg, void *payload, uint8_t len)
#line 96
{

  RssiMsg *rssiMsg = (RssiMsg *)payload;



  __nesc_hton_int16(rssiMsg->rssi.data, RssiBaseC__getRssi(msg));

  RssiBaseC__sendBlink();


  return TRUE;
}

# 725 "../InterceptBase/BaseStationP.nc"
static inline bool BaseStationP__RadioIntercept__default__forward(am_id_t amid, message_t *msg, void *payload, uint8_t len)
#line 725
{
  return TRUE;
}

# 31 "/opt/tinyos-2.x/tos/interfaces/Intercept.nc"
inline static bool BaseStationP__RadioIntercept__forward(am_id_t arg_0xf739aab8, message_t * msg, void * payload, uint8_t len){
#line 31
  unsigned char __nesc_result;
#line 31

#line 31
  switch (arg_0xf739aab8) {
#line 31
    case AM_RSSIMSG:
#line 31
      __nesc_result = RssiBaseC__SimpleRssiMsgIntercept__forward(msg, payload, len);
#line 31
      break;
#line 31
    case AM_MULTIPINGRESPONSEMSG:
#line 31
      __nesc_result = RssiBaseC__RssiMsgIntercept__forward(msg, payload, len);
#line 31
      break;
#line 31
    default:
#line 31
      __nesc_result = BaseStationP__RadioIntercept__default__forward(arg_0xf739aab8, msg, payload, len);
#line 31
      break;
#line 31
    }
#line 31

#line 31
  return __nesc_result;
#line 31
}
#line 31
# 72 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void RssiBaseC__Leds__led1Toggle(void ){
#line 72
  LedsP__Leds__led1Toggle();
#line 72
}
#line 72
# 153 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num].isrunning = FALSE;
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void BaseStationP__UartTimer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(2U);
#line 67
}
#line 67
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t PacketLinkLayerP__send__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(PacketLinkLayerP__send);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void PacketLinkLayerP__DelayTimer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(0U, dt);
#line 62
}
#line 62
# 29 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketFlag.nc"
inline static bool SoftwareAckLayerP__AckReceivedFlag__get(message_t *msg){
#line 29
  unsigned char __nesc_result;
#line 29

#line 29
  __nesc_result = MetadataFlagsLayerC__PacketFlag__get(0U, msg);
#line 29

#line 29
  return __nesc_result;
#line 29
}
#line 29
# 181 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/SoftwareAckLayerP.nc"
static inline bool SoftwareAckLayerP__PacketAcknowledgements__wasAcked(message_t *msg)
{
  return SoftwareAckLayerP__AckReceivedFlag__get(msg);
}

# 74 "/opt/tinyos-2.x/tos/interfaces/PacketAcknowledgements.nc"
inline static bool PacketLinkLayerP__PacketAcknowledgements__wasAcked(message_t * msg){
#line 74
  unsigned char __nesc_result;
#line 74

#line 74
  __nesc_result = SoftwareAckLayerP__PacketAcknowledgements__wasAcked(msg);
#line 74

#line 74
  return __nesc_result;
#line 74
}
#line 74
# 172 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/PacketLinkLayerP.nc"
static inline void PacketLinkLayerP__SubSend__sendDone(message_t *msg, error_t error)
#line 172
{
  PacketLinkLayerP__totalRetries++;

  if (PacketLinkLayerP__PacketAcknowledgements__wasAcked(msg)) {
      PacketLinkLayerP__signalDone(SUCCESS);
      return;
    }
  else {
#line 179
    if (PacketLinkLayerP__totalRetries < PacketLinkLayerP__PacketLink__getRetries(PacketLinkLayerP__currentSendMsg)) {

        if (PacketLinkLayerP__PacketLink__getRetryDelay(PacketLinkLayerP__currentSendMsg) > 0) {

            PacketLinkLayerP__DelayTimer__startOneShot(PacketLinkLayerP__PacketLink__getRetryDelay(PacketLinkLayerP__currentSendMsg));
          }
        else 
#line 184
          {

            PacketLinkLayerP__send__postTask();
          }

        return;
      }
    }
  PacketLinkLayerP__signalDone(error);
}

# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
inline static void MessageBufferLayerP__Send__sendDone(message_t *msg, error_t error){
#line 43
  PacketLinkLayerP__SubSend__sendDone(msg, error);
#line 43
}
#line 43
# 68 "/opt/tinyos-2.x/tos/chips/rf2xx/util/Tasklet.nc"
inline static void MessageBufferLayerP__Tasklet__resume(void ){
#line 68
  TaskletC__Tasklet__resume();
#line 68
}
#line 68
# 254 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline uint16_t RF230RadioP__RandomCollisionConfig__getInitialBackoff(message_t *msg)
{
  return (uint16_t )(9920 * (73728UL / 8 / 32) * (1 << MICA_DIVIDE_ONE_FOR_32KHZ_LOG2) / 10000UL);
}

# 29 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/RandomCollisionConfig.nc"
inline static uint16_t RandomCollisionLayerP__Config__getInitialBackoff(message_t *msg){
#line 29
  unsigned short __nesc_result;
#line 29

#line 29
  __nesc_result = RF230RadioP__RandomCollisionConfig__getInitialBackoff(msg);
#line 29

#line 29
  return __nesc_result;
#line 29
}
#line 29
# 87 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/RandomCollisionLayerP.nc"
static inline error_t RandomCollisionLayerP__RadioSend__send(message_t *msg)
{
  if (RandomCollisionLayerP__state != RandomCollisionLayerP__STATE_READY || !RandomCollisionLayerP__RadioAlarm__isFree()) {
    return EBUSY;
    }
  RandomCollisionLayerP__txMsg = msg;
  RandomCollisionLayerP__state = RandomCollisionLayerP__STATE_TX_PENDING_FIRST;
  RandomCollisionLayerP__RadioAlarm__wait(RandomCollisionLayerP__getBackoff(RandomCollisionLayerP__Config__getInitialBackoff(msg)));

  return SUCCESS;
}

# 37 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioSend.nc"
inline static error_t MessageBufferLayerP__RadioSend__send(message_t *msg){
#line 37
  unsigned char __nesc_result;
#line 37

#line 37
  __nesc_result = RandomCollisionLayerP__RadioSend__send(msg);
#line 37

#line 37
  return __nesc_result;
#line 37
}
#line 37
# 83 "/opt/tinyos-2.x/tos/chips/rf2xx/util/TaskletC.nc"
static __inline void TaskletC__Tasklet__suspend(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 85
    ++TaskletC__state;
#line 85
    __nesc_atomic_end(__nesc_atomic); }
}

# 61 "/opt/tinyos-2.x/tos/chips/rf2xx/util/Tasklet.nc"
inline static void MessageBufferLayerP__Tasklet__suspend(void ){
#line 61
  TaskletC__Tasklet__suspend();
#line 61
}
#line 61
# 170 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MessageBufferLayerP.nc"
static inline void MessageBufferLayerP__sendTask__runTask(void )
{
  error_t error;

  for (; 0; ) ;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 176
    error = MessageBufferLayerP__txError;
#line 176
    __nesc_atomic_end(__nesc_atomic); }
  if ((MessageBufferLayerP__state == MessageBufferLayerP__STATE_TX_SEND && error == SUCCESS) || ++MessageBufferLayerP__retries > MessageBufferLayerP__MAX_RETRIES) {
    MessageBufferLayerP__state = MessageBufferLayerP__STATE_TX_DONE;
    }
  else {
      MessageBufferLayerP__Tasklet__suspend();

      error = MessageBufferLayerP__RadioSend__send(MessageBufferLayerP__txMsg);
      if (error == SUCCESS) {
        MessageBufferLayerP__state = MessageBufferLayerP__STATE_TX_SEND;
        }
      else {
#line 186
        if (MessageBufferLayerP__retries == MessageBufferLayerP__MAX_RETRIES) {
          MessageBufferLayerP__state = MessageBufferLayerP__STATE_TX_DONE;
          }
        else {
#line 189
          MessageBufferLayerP__state = MessageBufferLayerP__STATE_TX_PENDING;
          }
        }
#line 191
      MessageBufferLayerP__Tasklet__resume();
    }

  if (MessageBufferLayerP__state == MessageBufferLayerP__STATE_TX_DONE) 
    {
      MessageBufferLayerP__state = MessageBufferLayerP__STATE_READY;
      MessageBufferLayerP__Send__sendDone(MessageBufferLayerP__txMsg, error);
    }
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void PacketLinkLayerP__DelayTimer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(0U);
#line 67
}
#line 67
# 104 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/PacketLinkLayerP.nc"
static inline void PacketLinkLayerP__PacketLink__setRetries(message_t *msg, uint16_t maxRetries)
#line 104
{
  PacketLinkLayerP__getMeta(msg)->maxRetries = maxRetries;
}

# 122 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154MessageLayerC.nc"
static inline void Ieee154MessageLayerC__Ieee154Send__default__sendDone(message_t *msg, error_t error)
{
}

# 86 "/opt/tinyos-2.x/tos/interfaces/Ieee154Send.nc"
inline static void Ieee154MessageLayerC__Ieee154Send__sendDone(message_t *msg, error_t error){
#line 86
  Ieee154MessageLayerC__Ieee154Send__default__sendDone(msg, error);
#line 86
}
#line 86
# 117 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154MessageLayerC.nc"
static inline void Ieee154MessageLayerC__SubSend__sendDone(message_t *msg, error_t error)
{
  Ieee154MessageLayerC__Ieee154Send__sendDone(msg, error);
}

# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
inline static void TinyosNetworkLayerC__Ieee154Send__sendDone(message_t *msg, error_t error){
#line 43
  Ieee154MessageLayerC__SubSend__sendDone(msg, error);
#line 43
}
#line 43
# 138 "../InterceptBase/BaseStationP.nc"
static inline void BaseStationP__sucRadioBlink(void )
#line 138
{



  BaseStationP__Leds__led0Toggle();
}

#line 692
static inline void BaseStationP__RadioSend__sendDone(am_id_t id, message_t *msg, error_t error)
#line 692
{
  if (error != SUCCESS) {
      BaseStationP__failBlink();
    }
  else {

      BaseStationP__sucRadioBlink();

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        if (msg == BaseStationP__radioQueue[BaseStationP__radioOut]) {
            if (++BaseStationP__radioOut >= BaseStationP__RADIO_QUEUE_LEN) {
              BaseStationP__radioOut = 0;
              }
#line 704
            if (BaseStationP__radioFull) {
              BaseStationP__radioFull = FALSE;
              }
          }
#line 707
        __nesc_atomic_end(__nesc_atomic); }
    }
#line 720
  BaseStationP__RadioTimer__startOneShot(BaseStationP__RADIO_TIME);
}

# 181 "/opt/tinyos-2.x/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(am_id_t id, message_t *msg, error_t err)
#line 181
{





  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current >= 1) {
      return;
    }
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg == msg) {
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current, msg, err);
    }
  else {
      ;
    }
}

# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static void ActiveMessageLayerP__AMSend__sendDone(am_id_t arg_0xf7234540, message_t * msg, error_t error){
#line 99
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(arg_0xf7234540, msg, error);
#line 99
  BaseStationP__RadioSend__sendDone(arg_0xf7234540, msg, error);
#line 99
}
#line 99
# 82 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayerP.nc"
static __inline void ActiveMessageLayerP__SubSend__sendDone(message_t *msg, error_t error)
{
  ActiveMessageLayerP__AMSend__sendDone(ActiveMessageLayerP__AMPacket__type(msg), msg, error);
}

# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
inline static void /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__BareSend__sendDone(message_t *msg, error_t error){
#line 43
  ActiveMessageLayerP__SubSend__sendDone(msg, error);
#line 43
}
#line 43
# 67 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/AutoResourceAcquireLayerC.nc"
static inline void /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__SubSend__sendDone(message_t *msg, error_t result)
{
  /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__release();
  /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__BareSend__sendDone(msg, result);
}

# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
inline static void TinyosNetworkLayerC__TinyosSend__sendDone(message_t *msg, error_t error){
#line 43
  /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__SubSend__sendDone(msg, error);
#line 43
}
#line 43
# 203 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TinyosNetworkLayerC.nc"
static inline void TinyosNetworkLayerC__SubSend__sendDone(message_t *msg, error_t result)
{
  if (__nesc_ntoh_leuint8(TinyosNetworkLayerC__getHeader(msg)->network.data) == 0x3f) {
    TinyosNetworkLayerC__TinyosSend__sendDone(msg, result);
    }
  else {
#line 208
    TinyosNetworkLayerC__Ieee154Send__sendDone(msg, result);
    }
}

# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
inline static void UniqueLayerP__Send__sendDone(message_t *msg, error_t error){
#line 43
  TinyosNetworkLayerC__SubSend__sendDone(msg, error);
#line 43
}
#line 43
# 69 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/UniqueLayerP.nc"
static inline void UniqueLayerP__SubSend__sendDone(message_t *msg, error_t error)
{
  UniqueLayerP__Send__sendDone(msg, error);
}

# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/util/BareSend.nc"
inline static void PacketLinkLayerP__Send__sendDone(message_t *msg, error_t error){
#line 43
  UniqueLayerP__SubSend__sendDone(msg, error);
#line 43
}
#line 43
# 157 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MessageBufferLayerP.nc"
static inline void MessageBufferLayerP__RadioChannel__default__setChannelDone(void )
{
}

# 37 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioChannel.nc"
inline static void MessageBufferLayerP__RadioChannel__setChannelDone(void ){
#line 37
  MessageBufferLayerP__RadioChannel__default__setChannelDone();
#line 37
}
#line 37
# 158 "RssiBaseC.nc"
static inline void RssiBaseC__RadioControl__stopDone(error_t result)
#line 158
{
}

# 109 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static error_t BaseStationP__RadioControl__stop(void ){
#line 109
  unsigned char __nesc_result;
#line 109

#line 109
  __nesc_result = MessageBufferLayerP__SplitControl__stop();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 434 "../InterceptBase/BaseStationP.nc"
static inline void BaseStationP__RadioControl__stopDone(error_t error)
#line 434
{
  if (error == SUCCESS) {
      BaseStationP__sucBlink();


      BaseStationP__resetPhase++;
      BaseStationP__ResetTimer__startOneShot(BaseStationP__RESET_TIME);
    }
  else 
#line 441
    {
      BaseStationP__RadioControl__stop();
      BaseStationP__failBlink();
      BaseStationP__dropBlink();


      BaseStationP__ResetTimer__startOneShot(BaseStationP__RESET_TIME);
    }
}

# 117 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static void MessageBufferLayerP__SplitControl__stopDone(error_t error){
#line 117
  BaseStationP__RadioControl__stopDone(error);
#line 117
  RssiBaseC__RadioControl__stopDone(error);
#line 117
}
#line 117
#line 83
inline static error_t RssiBaseC__RadioControl__start(void ){
#line 83
  unsigned char __nesc_result;
#line 83

#line 83
  __nesc_result = MessageBufferLayerP__SplitControl__start();
#line 83

#line 83
  return __nesc_result;
#line 83
}
#line 83
# 145 "RssiBaseC.nc"
static inline void RssiBaseC__RadioControl__startDone(error_t result)
#line 145
{

  if (result == SUCCESS) {
      ;
    }
  else {
      RssiBaseC__RadioControl__start();
    }

  RssiBaseC__busy = FALSE;
}

# 364 "../InterceptBase/BaseStationP.nc"
static inline void BaseStationP__RadioControl__startDone(error_t error)
#line 364
{
  uint8_t i;


  if (error == SUCCESS) {
      BaseStationP__sucBlink();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 370
        {
          for (i = 0; i < BaseStationP__RADIO_QUEUE_LEN; i++) {
              BaseStationP__radioQueue[i] = &BaseStationP__radioQueueBufs[i];
            }
          BaseStationP__radioIn = BaseStationP__radioOut = 0;
          BaseStationP__radioBusy = FALSE;
          BaseStationP__radioFull = FALSE;


          BaseStationP__resetPhase++;
          BaseStationP__ResetTimer__startOneShot(BaseStationP__RESET_TIME);
        }
#line 381
        __nesc_atomic_end(__nesc_atomic); }
    }
  else 
#line 382
    {
      BaseStationP__failBlink();
      BaseStationP__dropBlink();


      BaseStationP__ResetTimer__startOneShot(BaseStationP__RESET_TIME);
    }
}

# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static void MessageBufferLayerP__SplitControl__startDone(error_t error){
#line 92
  BaseStationP__RadioControl__startDone(error);
#line 92
  RssiBaseC__RadioControl__startDone(error);
#line 92
}
#line 92
# 125 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MessageBufferLayerP.nc"
static inline void MessageBufferLayerP__stateDoneTask__runTask(void )
{
  uint8_t s;

  s = MessageBufferLayerP__state;


  MessageBufferLayerP__state = MessageBufferLayerP__STATE_READY;

  if (s == MessageBufferLayerP__STATE_TURN_ON) {
    MessageBufferLayerP__SplitControl__startDone(SUCCESS);
    }
  else {
#line 136
    if (s == MessageBufferLayerP__STATE_TURN_OFF) {
      MessageBufferLayerP__SplitControl__stopDone(SUCCESS);
      }
    else {
#line 138
      if (s == MessageBufferLayerP__STATE_CHANNEL) {
        MessageBufferLayerP__RadioChannel__setChannelDone();
        }
      else {
#line 141
        MessageBufferLayerP__state = s;
        }
      }
    }
}

# 400 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static inline error_t RF230DriverLayerP__RadioState__turnOn(void )
{
  if (RF230DriverLayerP__cmd != RF230DriverLayerP__CMD_NONE || (RF230DriverLayerP__state == RF230DriverLayerP__STATE_SLEEP && !RF230DriverLayerP__RadioAlarm__isFree())) {
    return EBUSY;
    }
  else {
#line 404
    if (RF230DriverLayerP__state == RF230DriverLayerP__STATE_RX_ON) {
      return EALREADY;
      }
    }
#line 407
  RF230DriverLayerP__cmd = RF230DriverLayerP__CMD_TURNON;
  RF230DriverLayerP__Tasklet__schedule();

  return SUCCESS;
}

# 45 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioState.nc"
inline static error_t MessageBufferLayerP__RadioState__turnOn(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = RF230DriverLayerP__RadioState__turnOn();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 374 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static inline error_t RF230DriverLayerP__RadioState__turnOff(void )
{
  if (RF230DriverLayerP__cmd != RF230DriverLayerP__CMD_NONE) {
    return EBUSY;
    }
  else {
#line 378
    if (RF230DriverLayerP__state == RF230DriverLayerP__STATE_SLEEP) {
      return EALREADY;
      }
    }
#line 381
  RF230DriverLayerP__cmd = RF230DriverLayerP__CMD_TURNOFF;
  RF230DriverLayerP__Tasklet__schedule();

  return SUCCESS;
}

# 33 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioState.nc"
inline static error_t MessageBufferLayerP__RadioState__turnOff(void ){
#line 33
  unsigned char __nesc_result;
#line 33

#line 33
  __nesc_result = RF230DriverLayerP__RadioState__turnOff();
#line 33

#line 33
  return __nesc_result;
#line 33
}
#line 33
# 92 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt){
#line 92
  /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__startAt(t0, dt);
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
# 231 "/opt/tinyos-2.x/tos/chips/atm1281/timer/Atm1281AlarmAsyncP.nc"
static inline void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__stop(void )
#line 231
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 232
    /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__set = FALSE;
#line 232
    __nesc_atomic_end(__nesc_atomic); }
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void ){
#line 62
  /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__stop();
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

# 200 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/PacketLinkLayerP.nc"
static inline void PacketLinkLayerP__DelayTimer__fired(void )
#line 200
{
  if (PacketLinkLayerP__currentSendMsg != (void *)0) {
      PacketLinkLayerP__send__postTask();
    }
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void BaseStationP__ResetTimer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(1U);
#line 67
}
#line 67
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static error_t BaseStationP__RadioControl__start(void ){
#line 83
  unsigned char __nesc_result;
#line 83

#line 83
  __nesc_result = MessageBufferLayerP__SplitControl__start();
#line 83

#line 83
  return __nesc_result;
#line 83
}
#line 83
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__startDoneTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__startDoneTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 342 "/opt/tinyos-2.x/tos/lib/serial/SerialP.nc"
static inline error_t SerialP__SplitControl__start(void )
#line 342
{
  SerialP__startDoneTask__postTask();
  return SUCCESS;
}

# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static error_t BaseStationP__SerialControl__start(void ){
#line 83
  unsigned char __nesc_result;
#line 83

#line 83
  __nesc_result = SerialP__SplitControl__start();
#line 83

#line 83
  return __nesc_result;
#line 83
}
#line 83
# 364 "/opt/tinyos-2.x/tos/lib/serial/SerialP.nc"
static inline error_t SerialP__SplitControl__stop(void )
#line 364
{
  /* atomic removed: atomic calls only */
#line 365
  {
    if (SerialP__rxState == SerialP__RXSTATE_NOSYNC) {
        SerialP__rxState = SerialP__RXSTATE_INACTIVE;
      }
  }
  /* atomic removed: atomic calls only */
#line 370
  {
    if (SerialP__txState == SerialP__TXSTATE_IDLE) {
        SerialP__txState = SerialP__TXSTATE_INACTIVE;
      }
  }
  SerialP__testOff();
  return SUCCESS;
}

# 109 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static error_t BaseStationP__SerialControl__stop(void ){
#line 109
  unsigned char __nesc_result;
#line 109

#line 109
  __nesc_result = SerialP__SplitControl__stop();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 54 "../InterceptBase/Reset.h"
static inline void resetMote()
{
#line 68
   __asm volatile ("in __tmp_reg__,__SREG__""\n\t""cli""\n\t""wdr""\n\t""sts %0,%1""\n\t""out __SREG__,__tmp_reg__""\n\t""sts %0,%2""\n\t" :  : "M"((uint16_t )& * (volatile uint8_t *)0x60), "r"((1 << 4) | (1 << 3)), "r"((uint8_t )(((0 & 0x08 ? 1 << 5 : 0x00) | (1 << 3)) | (0 & 0x07))) : "r0");
  while (1) {
       __asm volatile ("nop""\n\t");}
}

# 52 "../InterceptBase/ResetC.nc"
static inline void ResetC__Reset__reset(void )
{
  resetMote();
}

# 46 "../InterceptBase/Reset.nc"
inline static void BaseStationP__Reset__reset(void ){
#line 46
  ResetC__Reset__reset();
#line 46
}
#line 46
# 177 "../InterceptBase/BaseStationP.nc"
static inline void BaseStationP__ResetTimer__fired(void )
#line 177
{
  error_t curError;


  if (BaseStationP__inReset == FALSE) {
      BaseStationP__ResetTimer__stop();
      return;
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 186
    {
      if (BaseStationP__resetPhase == -1) {

          BaseStationP__resetPhase = 4;
          BaseStationP__failBlink();
          BaseStationP__Reset__reset();

          BaseStationP__dropBlink();
          BaseStationP__sucBlink();
        }

      if (BaseStationP__resetPhase == 0) {

          curError = BaseStationP__RadioControl__stop();


          if (curError == EALREADY) {

              ++BaseStationP__resetPhase;
            }
          else {
#line 205
            if (curError == EBUSY) {


                BaseStationP__failBlink();
                BaseStationP__ResetTimer__startOneShot(BaseStationP__RESET_TIME);
                {
#line 210
                  __nesc_atomic_end(__nesc_atomic); 
#line 210
                  return;
                }
              }
            else {
#line 211
              if (curError == SUCCESS) {

                  BaseStationP__dropBlink();
                  {
#line 214
                    __nesc_atomic_end(__nesc_atomic); 
#line 214
                    return;
                  }
                }
              else 
#line 215
                {

                  BaseStationP__failBlink();
                  BaseStationP__ResetTimer__startOneShot(BaseStationP__RESET_TIME);
                  {
#line 219
                    __nesc_atomic_end(__nesc_atomic); 
#line 219
                    return;
                  }
                }
              }
            }
        }
#line 223
      if (BaseStationP__resetPhase == 1) {

          curError = BaseStationP__SerialControl__stop();


          if (curError == EALREADY) {
              ++BaseStationP__resetPhase;
            }
          else {
#line 230
            if (curError == EBUSY) {


                BaseStationP__failBlink();
                BaseStationP__ResetTimer__startOneShot(BaseStationP__RESET_TIME);
                {
#line 235
                  __nesc_atomic_end(__nesc_atomic); 
#line 235
                  return;
                }
              }
            else {
#line 236
              if (curError == SUCCESS) {

                  BaseStationP__dropBlink();
                  {
#line 239
                    __nesc_atomic_end(__nesc_atomic); 
#line 239
                    return;
                  }
                }
              else 
#line 240
                {

                  BaseStationP__failBlink();
                  BaseStationP__ResetTimer__startOneShot(BaseStationP__RESET_TIME);
                  {
#line 244
                    __nesc_atomic_end(__nesc_atomic); 
#line 244
                    return;
                  }
                }
              }
            }
        }
#line 248
      if (BaseStationP__resetPhase == 2) {

          BaseStationP__uartIn = BaseStationP__uartOut = 0;
          BaseStationP__uartBusy = FALSE;
          BaseStationP__uartFull = TRUE;


          BaseStationP__radioIn = BaseStationP__radioOut = 0;
          BaseStationP__radioBusy = FALSE;
          BaseStationP__radioFull = TRUE;

          curError = BaseStationP__SerialControl__start();


          if (curError == EALREADY) {
              ++BaseStationP__resetPhase;
              BaseStationP__ResetTimer__startOneShot(BaseStationP__RESET_TIME);
              {
#line 265
                __nesc_atomic_end(__nesc_atomic); 
#line 265
                return;
              }
            }
          else {
#line 266
            if (curError == EBUSY) {

                BaseStationP__failBlink();

                BaseStationP__resetPhase = 1;
                BaseStationP__ResetTimer__startOneShot(BaseStationP__RESET_TIME);
                {
#line 272
                  __nesc_atomic_end(__nesc_atomic); 
#line 272
                  return;
                }
              }
            else {
#line 273
              if (curError == SUCCESS) {

                  BaseStationP__dropBlink();
                  {
#line 276
                    __nesc_atomic_end(__nesc_atomic); 
#line 276
                    return;
                  }
                }
              else 
#line 277
                {

                  BaseStationP__failBlink();
                  BaseStationP__ResetTimer__startOneShot(BaseStationP__RESET_TIME);
                  {
#line 281
                    __nesc_atomic_end(__nesc_atomic); 
#line 281
                    return;
                  }
                }
              }
            }
        }
#line 285
      if (BaseStationP__resetPhase == 3) {

          curError = BaseStationP__RadioControl__start();


          if (curError == EALREADY) {
              ++BaseStationP__resetPhase;
              BaseStationP__ResetTimer__startOneShot(BaseStationP__RESET_TIME);
              {
#line 293
                __nesc_atomic_end(__nesc_atomic); 
#line 293
                return;
              }
            }
          else {
#line 294
            if (curError == EBUSY) {

                BaseStationP__resetPhase = 0;
                BaseStationP__failBlink();
                BaseStationP__ResetTimer__startOneShot(BaseStationP__RESET_TIME);
                {
#line 299
                  __nesc_atomic_end(__nesc_atomic); 
#line 299
                  return;
                }
              }
            else {
#line 300
              if (curError == SUCCESS) {

                  BaseStationP__dropBlink();
                  {
#line 303
                    __nesc_atomic_end(__nesc_atomic); 
#line 303
                    return;
                  }
                }
              else 
#line 304
                {

                  BaseStationP__failBlink();
                  BaseStationP__ResetTimer__startOneShot(BaseStationP__RESET_TIME);
                  {
#line 308
                    __nesc_atomic_end(__nesc_atomic); 
#line 308
                    return;
                  }
                }
              }
            }
        }
#line 312
      if (BaseStationP__resetPhase == 4) {

          BaseStationP__ResetTimer__stop();
          BaseStationP__inReset = FALSE;
          BaseStationP__resetPhase = 0;

          BaseStationP__sucBlink();
          BaseStationP__dropBlink();
          BaseStationP__failBlink();
        }
    }
#line 322
    __nesc_atomic_end(__nesc_atomic); }
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t BaseStationP__uartSendTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(BaseStationP__uartSendTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 155 "../InterceptBase/BaseStationP.nc"
static inline void BaseStationP__UartTimer__fired(void )
#line 155
{
  if (BaseStationP__inReset == TRUE) {
#line 156
    return;
    }

  if (BaseStationP__uartFailCounter > BaseStationP__UART_RESET_THRESHOLD) {
      BaseStationP__uartFailCounter = 0;
      BaseStationP__inReset = TRUE;

      BaseStationP__dropBlink();
      BaseStationP__Reset__reset();
    }
  else 

    {
      BaseStationP__uartSendTask__postTask();
    }
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t BaseStationP__radioSendTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(BaseStationP__radioSendTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 173 "../InterceptBase/BaseStationP.nc"
static inline void BaseStationP__RadioTimer__fired(void )
#line 173
{
  BaseStationP__radioSendTask__postTask();
}

# 136 "RssiBaseC.nc"
static inline void RssiBaseC__SendTimer__fired(void )
#line 136
{
}

# 193 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num)
{
}

# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(uint8_t arg_0xf6ff0d88){
#line 72
  switch (arg_0xf6ff0d88) {
#line 72
    case 0U:
#line 72
      PacketLinkLayerP__DelayTimer__fired();
#line 72
      break;
#line 72
    case 1U:
#line 72
      BaseStationP__ResetTimer__fired();
#line 72
      break;
#line 72
    case 2U:
#line 72
      BaseStationP__UartTimer__fired();
#line 72
      break;
#line 72
    case 3U:
#line 72
      BaseStationP__RadioTimer__fired();
#line 72
      break;
#line 72
    case 4U:
#line 72
      RssiBaseC__SendTimer__fired();
#line 72
      break;
#line 72
    default:
#line 72
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(arg_0xf6ff0d88);
#line 72
      break;
#line 72
    }
#line 72
}
#line 72
# 174 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer2AsyncP.nc"
static inline void HplAtm1281Timer2AsyncP__Compare__set(uint8_t t)
#line 174
{
  /* atomic removed: atomic calls only */
  {
    while (* (volatile uint8_t *)0xB6 & (1 << 3)) 
      ;
    * (volatile uint8_t *)0xB3 = t;
  }
}

# 45 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
inline static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Compare__set(/*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Compare__size_type t){
#line 45
  HplAtm1281Timer2AsyncP__Compare__set(t);
#line 45
}
#line 45
# 79 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer2AsyncP.nc"
static inline uint8_t HplAtm1281Timer2AsyncP__Timer__get(void )
#line 79
{
#line 79
  return * (volatile uint8_t *)0xB2;
}

# 52 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
inline static /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Timer__timer_size /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Timer__get(void ){
#line 52
  unsigned char __nesc_result;
#line 52

#line 52
  __nesc_result = HplAtm1281Timer2AsyncP__Timer__get();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 258 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer2AsyncP.nc"
static inline int HplAtm1281Timer2AsyncP__TimerAsync__compareABusy(void )
#line 258
{
  return (* (volatile uint8_t *)0xB6 & (1 << 3)) != 0;
}

# 75 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm128TimerAsync.nc"
inline static int /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__TimerAsync__compareABusy(void ){
#line 75
  int __nesc_result;
#line 75

#line 75
  __nesc_result = HplAtm1281Timer2AsyncP__TimerAsync__compareABusy();
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 101 "/opt/tinyos-2.x/tos/chips/atm1281/timer/Atm1281AlarmAsyncP.nc"
static inline void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__setOcr2A(uint8_t n)
#line 101
{
  while (/*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__TimerAsync__compareABusy()) 
    ;
  if (n == /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Timer__get()) {
    n++;
    }


  if (/*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__base + n + 1 < /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__base) {
    n = -/*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__base - 1;
    }
#line 111
  /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Compare__set(n);
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
inline static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__fired(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired();
#line 67
}
#line 67
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
# 253 "/opt/tinyos-2.x/tos/chips/atm1281/timer/Atm1281AlarmAsyncP.nc"
static inline uint32_t /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__getAlarm(void )
#line 253
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 254
    {
      unsigned long __nesc_temp = 
#line 254
      /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__t0 + /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__dt;

      {
#line 254
        __nesc_atomic_end(__nesc_atomic); 
#line 254
        return __nesc_temp;
      }
    }
#line 256
    __nesc_atomic_end(__nesc_atomic); }
}

# 105 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void ){
#line 105
  unsigned long __nesc_result;
#line 105

#line 105
  __nesc_result = /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__getAlarm();
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

# 207 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/PacketLinkLayerP.nc"
static inline void PacketLinkLayerP__send__runTask(void )
#line 207
{
  if (PacketLinkLayerP__PacketLink__getRetries(PacketLinkLayerP__currentSendMsg) > 0) {
      PacketLinkLayerP__PacketAcknowledgements__requestAck(PacketLinkLayerP__currentSendMsg);
    }

  if (PacketLinkLayerP__SubSend__send(PacketLinkLayerP__currentSendMsg) != SUCCESS) {
      PacketLinkLayerP__send__postTask();
    }
}

# 57 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/AutoResourceAcquireLayerC.nc"
static inline void /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__granted(void )
{
  error_t result = /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__SubSend__send(/*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__pending);

#line 60
  if (result != SUCCESS) 
    {
      /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__release();
      /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__BareSend__sendDone(/*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__pending, result);
    }
}

# 165 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__default__granted(uint8_t id)
#line 165
{
}

# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__granted(uint8_t arg_0xf71d8d90){
#line 92
  switch (arg_0xf71d8d90) {
#line 92
    case 0U:
#line 92
      /*RF230RadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__default__granted(arg_0xf71d8d90);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 155 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline void /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__runTask(void )
#line 155
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 156
    {
      /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId = /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__reqResId;
      /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__state = /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_BUSY;
    }
#line 159
    __nesc_atomic_end(__nesc_atomic); }
  /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__configure(/*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId);
  /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__granted(/*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId);
}

# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static error_t BaseStationP__RadioSend__send(am_id_t arg_0xf73c4310, am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = ActiveMessageLayerP__AMSend__send(arg_0xf73c4310, addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 136 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static am_id_t BaseStationP__UartAMPacket__type(message_t * amsg){
#line 136
  unsigned char __nesc_result;
#line 136

#line 136
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(amsg);
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 264 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_uint16(const void * source)
#line 264
{
  const uint8_t *base = source;

#line 266
  return ((uint16_t )base[0] << 8) | base[1];
}

# 137 "/opt/tinyos-2.x/tos/lib/serial/SerialActiveMessageP.nc"
static inline am_addr_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__destination(message_t *amsg)
#line 137
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 139
  return __nesc_ntoh_uint16(header->dest.data);
}

# 67 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static am_addr_t BaseStationP__UartAMPacket__destination(message_t * amsg){
#line 67
  unsigned short __nesc_result;
#line 67

#line 67
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__destination(amsg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 111 "/opt/tinyos-2.x/tos/lib/serial/SerialActiveMessageP.nc"
static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__payloadLength(message_t *msg)
#line 111
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(msg);

#line 113
  return __nesc_ntoh_uint8(header->length.data);
}

# 67 "/opt/tinyos-2.x/tos/interfaces/Packet.nc"
inline static uint8_t BaseStationP__UartPacket__payloadLength(message_t * msg){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__payloadLength(msg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 662 "../InterceptBase/BaseStationP.nc"
static inline void BaseStationP__radioSendTask__runTask(void )
#line 662
{
  uint8_t len;
  am_id_t id;
  am_addr_t addr;
  message_t *msg;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    if (BaseStationP__radioIn == BaseStationP__radioOut && !BaseStationP__radioFull) {
        BaseStationP__radioBusy = FALSE;
        {
#line 671
          __nesc_atomic_end(__nesc_atomic); 
#line 671
          return;
        }
      }
#line 673
    __nesc_atomic_end(__nesc_atomic); }
  msg = BaseStationP__radioQueue[BaseStationP__radioOut];
  len = BaseStationP__UartPacket__payloadLength(msg);
  addr = BaseStationP__UartAMPacket__destination(msg);
  id = BaseStationP__UartAMPacket__type(msg);

  if (BaseStationP__RadioSend__send(id, addr, msg, len) == SUCCESS) {
    }
  else 
    {
      BaseStationP__failBlink();



      BaseStationP__RadioTimer__startOneShot(BaseStationP__RADIO_TIME);
    }
}

# 522 "/opt/tinyos-2.x/tos/lib/serial/SerialP.nc"
static inline error_t SerialP__SendBytePacket__startSend(uint8_t b)
#line 522
{
  bool not_busy = FALSE;

#line 524
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 524
    {
      if (SerialP__txBuf[SerialP__TX_DATA_INDEX].state == SerialP__BUFFER_AVAILABLE) {
          SerialP__txBuf[SerialP__TX_DATA_INDEX].state = SerialP__BUFFER_FILLING;
          SerialP__txBuf[SerialP__TX_DATA_INDEX].buf = b;
          not_busy = TRUE;
        }
    }
#line 530
    __nesc_atomic_end(__nesc_atomic); }
  if (not_busy) {
      SerialP__MaybeScheduleTx();
      return SUCCESS;
    }
  return EBUSY;
}

# 51 "/opt/tinyos-2.x/tos/lib/serial/SendBytePacket.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__startSend(uint8_t first_byte){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = SerialP__SendBytePacket__startSend(first_byte);
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 43 "/opt/tinyos-2.x/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__dataLinkLength(message_t *msg, uint8_t upperLen)
#line 43
{
  return upperLen + sizeof(serial_header_t );
}

# 350 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(uart_id_t id, message_t *msg, 
uint8_t upperLen)
#line 351
{
  return 0;
}

# 23 "/opt/tinyos-2.x/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(uart_id_t arg_0xf6be9088, message_t *msg, uint8_t upperLen){
#line 23
  unsigned char __nesc_result;
#line 23

#line 23
  switch (arg_0xf6be9088) {
#line 23
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 23
      __nesc_result = SerialPacketInfoActiveMessageP__Info__dataLinkLength(msg, upperLen);
#line 23
      break;
#line 23
    default:
#line 23
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(arg_0xf6be9088, msg, upperLen);
#line 23
      break;
#line 23
    }
#line 23

#line 23
  return __nesc_result;
#line 23
}
#line 23
# 100 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(uint8_t id, message_t *msg, uint8_t len)
#line 100
{
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState != /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE) {
      return EBUSY;
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 105
    {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(id);
      if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex > sizeof(message_header_t )) {
          {
            unsigned char __nesc_temp = 
#line 108
            ESIZE;

            {
#line 108
              __nesc_atomic_end(__nesc_atomic); 
#line 108
              return __nesc_temp;
            }
          }
        }
#line 111
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError = SUCCESS;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer = (uint8_t *)msg;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_DATA;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendId = id;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendCancelled = FALSE;






      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendLen = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(id, msg, len) + /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex;
    }
#line 123
    __nesc_atomic_end(__nesc_atomic); }
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__startSend(id) == SUCCESS) {
      return SUCCESS;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE;
      return FAIL;
    }
}

# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(TOS_SERIAL_ACTIVE_MESSAGE_ID, msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 120 "/opt/tinyos-2.x/tos/lib/serial/SerialActiveMessageP.nc"
static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength(void )
#line 120
{
  return 28;
}

#line 57
static inline error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(am_id_t id, am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 59
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(msg);

  if (len > /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength()) {
      return ESIZE;
    }

  __nesc_hton_uint16(header->dest.data, dest);





  __nesc_hton_uint8(header->type.data, id);
  __nesc_hton_uint8(header->length.data, len);

  return /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__send(msg, len);
}

# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static error_t BaseStationP__UartSend__send(am_id_t arg_0xf73c7108, am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(arg_0xf73c7108, addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 152 "/opt/tinyos-2.x/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setSource(message_t *amsg, am_addr_t addr)
#line 152
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 154
  __nesc_hton_uint16(header->src.data, addr);
}

# 110 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static void BaseStationP__UartAMPacket__setSource(message_t * amsg, am_addr_t addr){
#line 110
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setSource(amsg, addr);
#line 110
}
#line 110
# 172 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230RadioP.nc"
static inline am_addr_t RF230RadioP__ActiveMessageConfig__source(message_t *msg)
{
  return RF230RadioP__Ieee154PacketLayer__getSrcAddr(msg);
}

# 35 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageConfig.nc"
inline static am_addr_t ActiveMessageLayerP__Config__source(message_t *msg){
#line 35
  unsigned short __nesc_result;
#line 35

#line 35
  __nesc_result = RF230RadioP__ActiveMessageConfig__source(msg);
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 163 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayerP.nc"
static __inline am_addr_t ActiveMessageLayerP__AMPacket__source(message_t *msg)
{
  return ActiveMessageLayerP__Config__source(msg);
}

# 77 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static am_addr_t BaseStationP__RadioAMPacket__source(message_t * amsg){
#line 77
  unsigned short __nesc_result;
#line 77

#line 77
  __nesc_result = ActiveMessageLayerP__AMPacket__source(amsg);
#line 77

#line 77
  return __nesc_result;
#line 77
}
#line 77
#line 67
inline static am_addr_t BaseStationP__RadioAMPacket__destination(message_t * amsg){
#line 67
  unsigned short __nesc_result;
#line 67

#line 67
  __nesc_result = ActiveMessageLayerP__AMPacket__destination(amsg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
#line 136
inline static am_id_t BaseStationP__RadioAMPacket__type(message_t * amsg){
#line 136
  unsigned char __nesc_result;
#line 136

#line 136
  __nesc_result = ActiveMessageLayerP__AMPacket__type(amsg);
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 67 "/opt/tinyos-2.x/tos/interfaces/Packet.nc"
inline static uint8_t BaseStationP__RadioPacket__payloadLength(message_t * msg){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = ActiveMessageLayerP__Packet__payloadLength(msg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 531 "../InterceptBase/BaseStationP.nc"
static inline void BaseStationP__uartSendTask__runTask(void )
#line 531
{
  uint8_t len;
  am_id_t id;
  am_addr_t addr;
#line 534
  am_addr_t src;
  error_t sendError;
  message_t *msg;



  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    if (BaseStationP__uartIn == BaseStationP__uartOut && !BaseStationP__uartFull) {
        BaseStationP__uartBusy = FALSE;
        {
#line 543
          __nesc_atomic_end(__nesc_atomic); 
#line 543
          return;
        }
      }
#line 545
    __nesc_atomic_end(__nesc_atomic); }
  msg = BaseStationP__uartQueue[BaseStationP__uartOut];
  BaseStationP__tmpLen = len = BaseStationP__RadioPacket__payloadLength(msg);
  id = BaseStationP__RadioAMPacket__type(msg);
  addr = BaseStationP__RadioAMPacket__destination(msg);
  src = BaseStationP__RadioAMPacket__source(msg);
  BaseStationP__UartAMPacket__setSource(msg, src);


  sendError = BaseStationP__UartSend__send(id, addr, BaseStationP__uartQueue[BaseStationP__uartOut], len);
  switch (sendError) {
      case SUCCESS: 

        BaseStationP__uartFailCounter = 0;
      break;


      case ESIZE: 

        ++BaseStationP__uartFailCounter;
      BaseStationP__failBlink();


      if (++BaseStationP__uartOut >= BaseStationP__UART_QUEUE_LEN) {
        BaseStationP__uartOut = 0;
        }
#line 570
      if (BaseStationP__uartFull) {
        BaseStationP__uartFull = FALSE;
        }
      break;

      case EBUSY: 


        break;

      case FAIL: 

        ++BaseStationP__uartFailCounter;
      BaseStationP__failBlink();
      break;
    }




  BaseStationP__timedUartSendTask();
}

# 45 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__Init__init(void )
#line 45
{
  memset(/*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__resQ, /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY, sizeof /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__resQ);
  return SUCCESS;
}

# 52 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/UniqueLayerP.nc"
static inline error_t UniqueLayerP__Init__init(void )
{
  UniqueLayerP__sequenceNumber = TOS_NODE_ID << 4;
  return SUCCESS;
}

# 44 "/opt/tinyos-2.x/tos/chips/rf2xx/util/NeighborhoodP.nc"
static inline error_t NeighborhoodP__Init__init(void )
{
  uint8_t i;

  for (i = 0; i < 5; ++i) 
    NeighborhoodP__nodes[i] = AM_BROADCAST_ADDR;

  return SUCCESS;
}

# 161 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer2AsyncP.nc"
static inline void HplAtm1281Timer2AsyncP__Compare__start(void )
#line 161
{
#line 161
  * (volatile uint8_t *)0x70 |= 1 << 1;
}

# 56 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
inline static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Compare__start(void ){
#line 56
  HplAtm1281Timer2AsyncP__Compare__start();
#line 56
}
#line 56
# 117 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer2AsyncP.nc"
static inline void HplAtm1281Timer2AsyncP__TimerCtrl__setControlB(uint8_t x)
#line 117
{
  while (* (volatile uint8_t *)0xB6 & (1 << 0)) 
    ;
  * (volatile uint8_t *)0xB1 = ((Atm128_TCCR2B_t )x).flat;
}

# 62 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm128TimerCtrl8.nc"
inline static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__TimerCtrl__setControlB(uint8_t control){
#line 62
  HplAtm1281Timer2AsyncP__TimerCtrl__setControlB(control);
#line 62
}
#line 62
# 111 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer2AsyncP.nc"
static inline void HplAtm1281Timer2AsyncP__TimerCtrl__setControlA(uint8_t x)
#line 111
{
  while (* (volatile uint8_t *)0xB6 & (1 << 1)) 
    ;
  * (volatile uint8_t *)0xB0 = ((Atm128_TCCR2A_t )x).flat;
}

# 61 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm128TimerCtrl8.nc"
inline static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__TimerCtrl__setControlA(uint8_t control){
#line 61
  HplAtm1281Timer2AsyncP__TimerCtrl__setControlA(control);
#line 61
}
#line 61
# 246 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer2AsyncP.nc"
static inline void HplAtm1281Timer2AsyncP__TimerAsync__setTimer2Asynchronous(void )
#line 246
{
  * (volatile uint8_t *)0xB6 |= 1 << 5;
}

# 57 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm128TimerAsync.nc"
inline static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__TimerAsync__setTimer2Asynchronous(void ){
#line 57
  HplAtm1281Timer2AsyncP__TimerAsync__setTimer2Asynchronous();
#line 57
}
#line 57
# 78 "/opt/tinyos-2.x/tos/chips/atm1281/timer/Atm1281AlarmAsyncP.nc"
static inline error_t /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Init__init(void )
#line 78
{
  /* atomic removed: atomic calls only */
  {
    Atm128_TCCR2A_t x;
    Atm128_TCCR2B_t y;

    /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__TimerAsync__setTimer2Asynchronous();
    x.flat = 0;
    x.bits.wgm21 = 1;
    /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__TimerCtrl__setControlA(x.flat);
    y.flat = 0;
    y.bits.cs = 3;
    /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__TimerCtrl__setControlB(y.flat);
    /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Compare__set(/*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__MAXT);
    /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Compare__start();
  }
  /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__setInterrupt();
  return SUCCESS;
}

# 260 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MessageBufferLayerP.nc"
static inline error_t MessageBufferLayerP__SoftwareInit__init(void )
{
  uint8_t i;

  for (i = 0; i < MessageBufferLayerP__RECEIVE_QUEUE_SIZE; ++i) 
    MessageBufferLayerP__receiveQueue[i] = MessageBufferLayerP__receiveQueueData + i;

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

# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t RF230DriverLayerP__SpiResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = Atm128SpiP__Resource__request(0U);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 224 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static inline error_t RF230DriverLayerP__SoftwareInit__init(void )
{

  return RF230DriverLayerP__SpiResource__request();
}

# 45 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__Init__init(void )
#line 45
{
  memset(/*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__resQ, /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__NO_ENTRY, sizeof /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__resQ);
  return SUCCESS;
}

# 214 "/opt/tinyos-2.x/tos/lib/serial/SerialP.nc"
static __inline void SerialP__ackInit(void )
#line 214
{
  SerialP__ackQ.writePtr = SerialP__ackQ.readPtr = 0;
}

#line 205
static __inline void SerialP__rxInit(void )
#line 205
{
  SerialP__rxBuf.writePtr = SerialP__rxBuf.readPtr = 0;
  SerialP__rxState = SerialP__RXSTATE_NOSYNC;
  SerialP__rxByteCnt = 0;
  SerialP__rxProto = 0;
  SerialP__rxSeqno = 0;
  SerialP__rxCRC = 0;
}

#line 193
static __inline void SerialP__txInit(void )
#line 193
{
  uint8_t i;

  /* atomic removed: atomic calls only */
#line 195
  for (i = 0; i < SerialP__TX_BUFFER_COUNT; i++) SerialP__txBuf[i].state = SerialP__BUFFER_AVAILABLE;
  SerialP__txState = SerialP__TXSTATE_IDLE;
  SerialP__txByteCnt = 0;
  SerialP__txProto = 0;
  SerialP__txSeqno = 0;
  SerialP__txCRC = 0;
  SerialP__txPending = FALSE;
  SerialP__txIndex = 0;
}

#line 218
static inline error_t SerialP__Init__init(void )
#line 218
{

  SerialP__txInit();
  SerialP__rxInit();
  SerialP__ackInit();

  return SUCCESS;
}

# 69 "/opt/tinyos-2.x/tos/chips/atm128/Atm128UartP.nc"
static inline error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__Init__init(void )
#line 69
{
  if (PLATFORM_BAUDRATE == 19200UL) {
    /*Atm128Uart0C.UartP*/Atm128UartP__0__m_byte_time = 200;
    }
  else {
#line 72
    if (PLATFORM_BAUDRATE == 57600UL) {
      /*Atm128Uart0C.UartP*/Atm128UartP__0__m_byte_time = 68;
      }
    }
#line 74
  return SUCCESS;
}

# 143 "/opt/tinyos-2.x/tos/platforms/iris/MeasureClockC.nc"
static inline uint16_t MeasureClockC__Atm128Calibrate__baudrateRegister(uint32_t baudrate)
#line 143
{

  return ((uint32_t )MeasureClockC__cycles << 12) / baudrate - 1;
}

# 60 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128Calibrate.nc"
inline static uint16_t HplAtm128UartP__Atm128Calibrate__baudrateRegister(uint32_t baudrate){
#line 60
  unsigned short __nesc_result;
#line 60

#line 60
  __nesc_result = MeasureClockC__Atm128Calibrate__baudrateRegister(baudrate);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 204 "/opt/tinyos-2.x/tos/chips/atm1281/HplAtm128UartP.nc"
static inline error_t HplAtm128UartP__Uart1Init__init(void )
#line 204
{
  Atm128UartMode_t mode;
  Atm128UartStatus_t stts;
  Atm128UartControl_t ctrl;
  uint16_t ubrr1;

  ctrl.bits = (struct Atm128_UCSRB_t ){ .rxcie = 0, .txcie = 0, .rxen = 0, .txen = 0 };
  stts.bits = (struct Atm128_UCSRA_t ){ .u2x = 1 };
  mode.bits = (struct Atm128_UCSRC_t ){ .ucsz = ATM128_UART_DATA_SIZE_8_BITS };

  ubrr1 = HplAtm128UartP__Atm128Calibrate__baudrateRegister(PLATFORM_BAUDRATE);
  * (volatile uint8_t *)0xCC = ubrr1;
  * (volatile uint8_t *)0xCD = ubrr1 >> 8;
  * (volatile uint8_t *)0xC8 = stts.flat;
  * (volatile uint8_t *)0xCA = mode.flat;
  * (volatile uint8_t *)0XC9 = ctrl.flat;

  return SUCCESS;
}

#line 110
static inline error_t HplAtm128UartP__Uart0Init__init(void )
#line 110
{
  Atm128UartMode_t mode;
  Atm128UartStatus_t stts;
  Atm128UartControl_t ctrl;
  uint16_t ubrr0;

  ctrl.bits = (struct Atm128_UCSRB_t ){ .rxcie = 0, .txcie = 0, .rxen = 0, .txen = 0 };
  stts.bits = (struct Atm128_UCSRA_t ){ .u2x = 1 };
  mode.bits = (struct Atm128_UCSRC_t ){ .ucsz = ATM128_UART_DATA_SIZE_8_BITS };

  ubrr0 = HplAtm128UartP__Atm128Calibrate__baudrateRegister(PLATFORM_BAUDRATE);
  * (volatile uint8_t *)0xC4 = ubrr0;
  * (volatile uint8_t *)0xC5 = ubrr0 >> 8;
  * (volatile uint8_t *)0xC0 = stts.flat;
  * (volatile uint8_t *)0xC2 = mode.flat;
  * (volatile uint8_t *)0XC1 = ctrl.flat;

  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = HplAtm128UartP__Uart0Init__init();
#line 51
  __nesc_result = ecombine(__nesc_result, HplAtm128UartP__Uart1Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*Atm128Uart0C.UartP*/Atm128UartP__0__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, SerialP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*Atm128SpiC.Arbiter.Queue*/FcfsResourceQueueC__1__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, RF230DriverLayerP__SoftwareInit__init());
#line 51
  __nesc_result = ecombine(__nesc_result, RandomMlcgC__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, MessageBufferLayerP__SoftwareInit__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, NeighborhoodP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, UniqueLayerP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*RF230RadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__Init__init());
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 131 "RssiBaseC.nc"
static inline void RssiBaseC__Boot__booted(void )
#line 131
{
  RssiBaseC__busy = TRUE;
  RssiBaseC__RadioControl__start();
}

# 328 "../InterceptBase/BaseStationP.nc"
static inline void BaseStationP__Boot__booted(void )
#line 328
{
  uint8_t i;


  for (i = 0; i < BaseStationP__UART_QUEUE_LEN; i++) {
      BaseStationP__uartQueue[i] = &BaseStationP__uartQueueBufs[i];
    }
  BaseStationP__uartIn = BaseStationP__uartOut = 0;
  BaseStationP__uartBusy = FALSE;
  BaseStationP__uartFull = TRUE;


  for (i = 0; i < BaseStationP__RADIO_QUEUE_LEN; i++) {
      BaseStationP__radioQueue[i] = &BaseStationP__radioQueueBufs[i];
    }
  BaseStationP__radioIn = BaseStationP__radioOut = 0;
  BaseStationP__radioBusy = FALSE;
  BaseStationP__radioFull = TRUE;
#line 358
  BaseStationP__dropBlink();
  BaseStationP__RadioControl__start();
  BaseStationP__SerialControl__start();
}

# 49 "/opt/tinyos-2.x/tos/interfaces/Boot.nc"
inline static void RealMainP__Boot__booted(void ){
#line 49
  BaseStationP__Boot__booted();
#line 49
  RssiBaseC__Boot__booted();
#line 49
}
#line 49
# 199 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer2AsyncP.nc"
static inline mcu_power_t HplAtm1281Timer2AsyncP__McuPowerOverride__lowestState(void )
#line 199
{
  uint8_t diff;


  if (* (volatile uint8_t *)0x70 & ((1 << 1) | (1 << 0))) {




      while (* (volatile uint8_t *)0xB6 & (((1 << 4) | (1 << 3)) | (1 << 1))) 
        ;
      diff = * (volatile uint8_t *)0xB3 - * (volatile uint8_t *)0xB2;
      if (diff < EXT_STANDBY_T0_THRESHOLD || 
      * (volatile uint8_t *)0xB2 > 256 - EXT_STANDBY_T0_THRESHOLD) {
        return ATM128_POWER_EXT_STANDBY;
        }
#line 214
      return ATM128_POWER_SAVE;
    }
  else {
      return ATM128_POWER_DOWN;
    }
}

# 54 "/opt/tinyos-2.x/tos/interfaces/McuPowerOverride.nc"
inline static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = HplAtm1281Timer2AsyncP__McuPowerOverride__lowestState();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 88 "/opt/tinyos-2.x/tos/chips/atm1281/McuSleepC.nc"
static inline mcu_power_t McuSleepC__getPowerState(void )
#line 88
{










  if ((
#line 98
  * (volatile uint8_t *)0x6E & ((1 << 1) | (1 << 2)) || 
  * (volatile uint8_t *)0x6F & ((((1 << 5) | (1 << 1)) | (1 << 2)) | (1 << 3))) || 
  * (volatile uint8_t *)0x71 & ((((1 << 5) | (1 << 1)) | (1 << 2)) | (1 << 3))) 
    {
      return ATM128_POWER_IDLE;
    }
  else {
    if (* (volatile uint8_t *)(uint16_t )& * (volatile uint8_t *)(0x2C + 0x20) & (1 << 7)) {
        return ATM128_POWER_IDLE;
      }
    else {
      if (* (volatile uint8_t *)0XC1 & (((1 << 6) | (1 << 7)) | (1 << 5))) {
          return ATM128_POWER_IDLE;
        }
      else {
#line 112
        if (* (volatile uint8_t *)0XC9 & (((1 << 6) | (1 << 7)) | (1 << 5))) {
            return ATM128_POWER_IDLE;
          }
        else {
          if (* (volatile uint8_t *)(uint16_t )& * (volatile uint8_t *)0xBC & (1 << 2)) {
              return ATM128_POWER_IDLE;
            }
          else {
            if (* (volatile uint8_t *)(uint16_t )& * (volatile uint8_t *)0x7A & (1 << 7)) {
                return ATM128_POWER_ADC_NR;
              }
            else {
                return ATM128_POWER_DOWN;
              }
            }
          }
        }
      }
    }
}

# 157 "/opt/tinyos-2.x/tos/chips/atm1281/atm128hardware.h"
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 157
{
  return m1 < m2 ? m1 : m2;
}

# 128 "/opt/tinyos-2.x/tos/chips/atm1281/McuSleepC.nc"
static inline void McuSleepC__McuSleep__sleep(void )
#line 128
{
  uint8_t powerState;

  powerState = mcombine(McuSleepC__getPowerState(), McuSleepC__McuPowerOverride__lowestState());
  * (volatile uint8_t *)(0x33 + 0x20) = ((
  * (volatile uint8_t *)(0x33 + 0x20) & 0xf0) | (1 << 0)) | __extension__ ({
#line 133
    uint16_t __addr16 = (uint16_t )(uint16_t )&McuSleepC__atm128PowerBits[powerState];
#line 133
    uint8_t __result;

#line 133
     __asm ("lpm %0, Z""\n\t" : "=r"(__result) : "z"(__addr16));__result;
  }
  );
#line 134
   __asm volatile ("sei" :  :  : "memory");

   __asm volatile ("sleep" :  :  : "memory");
   __asm volatile ("cli" :  :  : "memory");

  * (volatile uint8_t *)(0x33 + 0x20) &= ~(1 << 0);
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
# 184 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer2AsyncP.nc"
static __inline void HplAtm1281Timer2AsyncP__stabiliseTimer2(void )
#line 184
{
  * (volatile uint8_t *)0xB0 = * (volatile uint8_t *)0xB0;
  while (* (volatile uint8_t *)0xB6 & (1 << 1)) 
    ;
}

# 47 "/opt/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void )
{
}

# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Counter__overflow(void ){
#line 71
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow();
#line 71
}
#line 71
# 105 "/opt/tinyos-2.x/tos/chips/atm1281/atm128hardware.h"
static __inline  void __nesc_enable_interrupt()
#line 105
{
   __asm volatile ("sei" :  :  : "memory");}

# 171 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer2AsyncP.nc"
static inline uint8_t HplAtm1281Timer2AsyncP__Compare__get(void )
#line 171
{
#line 171
  return * (volatile uint8_t *)0xB3;
}

# 39 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
inline static /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Compare__size_type /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Compare__get(void ){
#line 39
  unsigned char __nesc_result;
#line 39

#line 39
  __nesc_result = HplAtm1281Timer2AsyncP__Compare__get();
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 176 "/opt/tinyos-2.x/tos/chips/atm1281/timer/Atm1281AlarmAsyncP.nc"
static inline void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Compare__fired(void )
#line 176
{
  int overflowed;


  /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__base += /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Compare__get() + 1U;
  overflowed = !/*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__base;
  __nesc_enable_interrupt();
  /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__setInterrupt();
  if (overflowed) {
    /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Counter__overflow();
    }
}

# 49 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
inline static void HplAtm1281Timer2AsyncP__Compare__fired(void ){
#line 49
  /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Compare__fired();
#line 49
}
#line 49
# 257 "/opt/tinyos-2.x/tos/chips/atm1281/timer/Atm1281AlarmAsyncP.nc"
static inline void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Timer__overflow(void )
#line 257
{
}

# 61 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
inline static void HplAtm1281Timer2AsyncP__Timer__overflow(void ){
#line 61
  /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Timer__overflow();
#line 61
}
#line 61
# 96 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128Spi.nc"
inline static void Atm128SpiP__Spi__enableInterrupt(bool enabled){
#line 96
  HplAtm128SpiP__SPI__enableInterrupt(enabled);
#line 96
}
#line 96
# 196 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
static inline error_t Atm128SpiP__sendNextPart(void )
#line 196
{
  uint16_t end;
  uint16_t tmpPos;
  uint16_t myLen;
  uint8_t * tx;
  uint8_t * rx;

  /* atomic removed: atomic calls only */
#line 203
  {
    myLen = Atm128SpiP__len;
    tx = Atm128SpiP__txBuffer;
    rx = Atm128SpiP__rxBuffer;
    tmpPos = Atm128SpiP__pos;
    end = Atm128SpiP__pos + Atm128SpiP__SPI_ATOMIC_SIZE;
    end = end > Atm128SpiP__len ? Atm128SpiP__len : end;
  }

  for (; tmpPos < end - 1; tmpPos++) {
      uint8_t val;

#line 214
      if (tx != (void *)0) {
        val = Atm128SpiP__SpiByte__write(tx[tmpPos]);
        }
      else {
#line 217
        val = Atm128SpiP__SpiByte__write(0);
        }
      if (rx != (void *)0) {
          rx[tmpPos] = val;
        }
    }



  Atm128SpiP__Spi__enableInterrupt(TRUE);
  /* atomic removed: atomic calls only */
#line 227
  {
    if (tx != (void *)0) {
      Atm128SpiP__Spi__write(tx[tmpPos]);
      }
    else {
#line 231
      Atm128SpiP__Spi__write(0);
      }
    Atm128SpiP__pos = tmpPos;
  }


  return SUCCESS;
}

#line 299
static inline void Atm128SpiP__Spi__dataReady(uint8_t data)
#line 299
{
  bool again;

  /* atomic removed: atomic calls only */
#line 302
  {
    if (Atm128SpiP__rxBuffer != (void *)0) {
        Atm128SpiP__rxBuffer[Atm128SpiP__pos] = data;
      }

    Atm128SpiP__pos++;
  }
  Atm128SpiP__Spi__enableInterrupt(FALSE);
  /* atomic removed: atomic calls only */
  {
    again = Atm128SpiP__pos < Atm128SpiP__len;
  }

  if (again) {
      Atm128SpiP__sendNextPart();
    }
  else {
      uint8_t discard;
      uint16_t myLen;
      uint8_t * rx;
      uint8_t * tx;

      /* atomic removed: atomic calls only */
#line 324
      {
        myLen = Atm128SpiP__len;
        rx = Atm128SpiP__rxBuffer;
        tx = Atm128SpiP__txBuffer;
        Atm128SpiP__rxBuffer = (void *)0;
        Atm128SpiP__txBuffer = (void *)0;
        Atm128SpiP__len = 0;
        Atm128SpiP__pos = 0;
      }
      discard = Atm128SpiP__Spi__read();

      Atm128SpiP__SpiPacket__sendDone(tx, rx, myLen, SUCCESS);
    }
}

# 92 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128Spi.nc"
inline static void HplAtm128SpiP__SPI__dataReady(uint8_t data){
#line 92
  Atm128SpiP__Spi__dataReady(data);
#line 92
}
#line 92
# 48 "/opt/tinyos-2.x/tos/chips/rf2xx/util/Tasklet.nc"
inline static void RadioAlarmP__Tasklet__schedule(void ){
#line 48
  TaskletC__Tasklet__schedule();
#line 48
}
#line 48
# 53 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarmP.nc"
static inline void RadioAlarmP__Alarm__fired(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      if (RadioAlarmP__state == RadioAlarmP__STATE_WAIT) {
        RadioAlarmP__state = RadioAlarmP__STATE_FIRED;
        }
    }
#line 60
    __nesc_atomic_end(__nesc_atomic); }
  RadioAlarmP__Tasklet__schedule();
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__fired(void ){
#line 67
  RadioAlarmP__Alarm__fired();
#line 67
}
#line 67
# 110 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128AlarmC.nc"
static inline void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Compare__fired(void )
#line 110
{
  /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Compare__stop();
  ;
  __nesc_enable_interrupt();
  /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__fired();
}

# 49 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
inline static void HplAtm1281Timer1P__CompareA__fired(void ){
#line 49
  /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Compare__fired();
#line 49
}
#line 49
# 209 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer1P.nc"
static inline void HplAtm1281Timer1P__CompareB__default__fired(void )
#line 209
{
}

# 49 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
inline static void HplAtm1281Timer1P__CompareB__fired(void ){
#line 49
  HplAtm1281Timer1P__CompareB__default__fired();
#line 49
}
#line 49
# 213 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer1P.nc"
static inline void HplAtm1281Timer1P__CompareC__default__fired(void )
#line 213
{
}

# 49 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
inline static void HplAtm1281Timer1P__CompareC__fired(void ){
#line 49
  HplAtm1281Timer1P__CompareC__default__fired();
#line 49
}
#line 49
# 661 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static inline void RF230DriverLayerP__IRQ__captured(uint16_t time)
{
  for (; 0; ) ;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      RF230DriverLayerP__capturedTime = time;
      RF230DriverLayerP__radioIrq = TRUE;
    }
#line 669
    __nesc_atomic_end(__nesc_atomic); }

  RF230DriverLayerP__Tasklet__schedule();
}

# 50 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
inline static void HplRF230P__IRQ__captured(uint16_t time){
#line 50
  RF230DriverLayerP__IRQ__captured(time);
#line 50
}
#line 50
# 199 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer1P.nc"
static inline uint16_t HplAtm1281Timer1P__Capture__get(void )
#line 199
{
#line 199
  return * (volatile uint16_t *)0x86;
}

# 38 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Capture.nc"
inline static HplRF230P__Capture__size_type HplRF230P__Capture__get(void ){
#line 38
  unsigned short __nesc_result;
#line 38

#line 38
  __nesc_result = HplAtm1281Timer1P__Capture__get();
#line 38

#line 38
  return __nesc_result;
#line 38
}
#line 38
# 53 "/opt/tinyos-2.x/tos/platforms/iris/chips/rf230/HplRF230P.nc"
static inline void HplRF230P__Capture__captured(uint16_t time)
{
  time = HplRF230P__Capture__get();
  HplRF230P__IRQ__captured(time);
}

# 51 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Capture.nc"
inline static void HplAtm1281Timer1P__Capture__captured(HplAtm1281Timer1P__Capture__size_type t){
#line 51
  HplRF230P__Capture__captured(t);
#line 51
}
#line 51
# 117 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128AlarmC.nc"
static inline void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Timer__overflow(void )
#line 117
{
}

# 51 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128TimerInitC.nc"
static inline void /*InitOneP.InitOne*/Atm128TimerInitC__0__Timer__overflow(void )
#line 51
{
}

# 47 "/opt/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void )
{
}

# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static void /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__Counter__overflow(void ){
#line 71
  /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow();
#line 71
}
#line 71
# 122 "/opt/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static inline void /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__CounterFrom__overflow(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__m_upper++;
      if ((/*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__m_upper & /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__OVERFLOW_MASK) == 0) {
        /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__Counter__overflow();
        }
    }
#line 130
    __nesc_atomic_end(__nesc_atomic); }
}

# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static void /*CounterOne16C.NCounter*/Atm128CounterC__0__Counter__overflow(void ){
#line 71
  /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__CounterFrom__overflow();
#line 71
}
#line 71
# 56 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128CounterC.nc"
static inline void /*CounterOne16C.NCounter*/Atm128CounterC__0__Timer__overflow(void )
{
  /*CounterOne16C.NCounter*/Atm128CounterC__0__Counter__overflow();
}

# 61 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
inline static void HplAtm1281Timer1P__Timer__overflow(void ){
#line 61
  /*CounterOne16C.NCounter*/Atm128CounterC__0__Timer__overflow();
#line 61
  /*InitOneP.InitOne*/Atm128TimerInitC__0__Timer__overflow();
#line 61
  /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Timer__overflow();
#line 61
}
#line 61
# 387 "/opt/tinyos-2.x/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFrameComm__dataReceived(uint8_t data)
#line 387
{
  SerialP__rx_state_machine(FALSE, data);
}

# 83 "/opt/tinyos-2.x/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC__SerialFrameComm__dataReceived(uint8_t data){
#line 83
  SerialP__SerialFrameComm__dataReceived(data);
#line 83
}
#line 83
# 384 "/opt/tinyos-2.x/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFrameComm__delimiterReceived(void )
#line 384
{
  SerialP__rx_state_machine(TRUE, 0);
}

# 74 "/opt/tinyos-2.x/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC__SerialFrameComm__delimiterReceived(void ){
#line 74
  SerialP__SerialFrameComm__delimiterReceived();
#line 74
}
#line 74
# 61 "/opt/tinyos-2.x/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC__UartStream__receivedByte(uint8_t data)
#line 61
{






  if (data == HDLC_FLAG_BYTE) {

      HdlcTranslateC__SerialFrameComm__delimiterReceived();
      return;
    }
  else {
#line 73
    if (data == HDLC_CTLESC_BYTE) {

        HdlcTranslateC__state.receiveEscape = 1;
        return;
      }
    else {
#line 78
      if (HdlcTranslateC__state.receiveEscape) {

          HdlcTranslateC__state.receiveEscape = 0;
          data = data ^ 0x20;
        }
      }
    }
#line 83
  HdlcTranslateC__SerialFrameComm__dataReceived(data);
}

# 79 "/opt/tinyos-2.x/tos/interfaces/UartStream.nc"
inline static void /*Atm128Uart0C.UartP*/Atm128UartP__0__UartStream__receivedByte(uint8_t byte){
#line 79
  HdlcTranslateC__UartStream__receivedByte(byte);
#line 79
}
#line 79
# 116 "/opt/tinyos-2.x/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC__UartStream__receiveDone(uint8_t *buf, uint16_t len, error_t error)
#line 116
{
}

# 99 "/opt/tinyos-2.x/tos/interfaces/UartStream.nc"
inline static void /*Atm128Uart0C.UartP*/Atm128UartP__0__UartStream__receiveDone(uint8_t * buf, uint16_t len, error_t error){
#line 99
  HdlcTranslateC__UartStream__receiveDone(buf, len, error);
#line 99
}
#line 99
# 133 "/opt/tinyos-2.x/tos/chips/atm128/Atm128UartP.nc"
static inline void /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__rxDone(uint8_t data)
#line 133
{

  if (/*Atm128Uart0C.UartP*/Atm128UartP__0__m_rx_buf) {
      /*Atm128Uart0C.UartP*/Atm128UartP__0__m_rx_buf[/*Atm128Uart0C.UartP*/Atm128UartP__0__m_rx_pos++] = data;
      if (/*Atm128Uart0C.UartP*/Atm128UartP__0__m_rx_pos >= /*Atm128Uart0C.UartP*/Atm128UartP__0__m_rx_len) {
          uint8_t *buf = /*Atm128Uart0C.UartP*/Atm128UartP__0__m_rx_buf;

          /* atomic removed: atomic calls only */
#line 139
          {
            /*Atm128Uart0C.UartP*/Atm128UartP__0__m_rx_buf = (void *)0;
            if (/*Atm128Uart0C.UartP*/Atm128UartP__0__m_rx_intr != 3) {
                /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__disableRxIntr();
                /*Atm128Uart0C.UartP*/Atm128UartP__0__m_rx_intr = 0;
              }
          }
          /*Atm128Uart0C.UartP*/Atm128UartP__0__UartStream__receiveDone(buf, /*Atm128Uart0C.UartP*/Atm128UartP__0__m_rx_len, SUCCESS);
        }
    }
  else {
      /*Atm128Uart0C.UartP*/Atm128UartP__0__UartStream__receivedByte(data);
    }
}

# 49 "/opt/tinyos-2.x/tos/chips/atm128/HplAtm128Uart.nc"
inline static void HplAtm128UartP__HplUart0__rxDone(uint8_t data){
#line 49
  /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__rxDone(data);
#line 49
}
#line 49
# 391 "/opt/tinyos-2.x/tos/lib/serial/SerialP.nc"
static inline bool SerialP__valid_rx_proto(uint8_t proto)
#line 391
{
  switch (proto) {
      case SERIAL_PROTO_PACKET_ACK: 
        return TRUE;
      case SERIAL_PROTO_ACK: 
        case SERIAL_PROTO_PACKET_NOACK: 
          default: 
            return FALSE;
    }
}

# 192 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__lockCurrentBuffer(void )
#line 192
{
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufOneLocked = 1;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufZeroLocked = 1;
    }
}

#line 188
static inline bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__isCurrentBufferLocked(void )
#line 188
{
  return /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which ? /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufOneLocked : /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufZeroLocked;
}

#line 215
static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket(void )
#line 215
{
  error_t result = SUCCESS;

  /* atomic removed: atomic calls only */
#line 217
  {
    if (!/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__isCurrentBufferLocked()) {


        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__lockCurrentBuffer();
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_BEGIN;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex = 0;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType = TOS_SERIAL_UNKNOWN_ID;
      }
    else {
        result = EBUSY;
      }
  }
  return result;
}

# 51 "/opt/tinyos-2.x/tos/lib/serial/ReceiveBytePacket.nc"
inline static error_t SerialP__ReceiveBytePacket__startPacket(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 309 "/opt/tinyos-2.x/tos/lib/serial/SerialP.nc"
static __inline uint16_t SerialP__rx_current_crc(void )
#line 309
{
  uint16_t crc;
  uint8_t tmp = SerialP__rxBuf.writePtr;

#line 312
  tmp = tmp == 0 ? SerialP__RX_DATA_BUFFER_SIZE : tmp - 1;
  crc = SerialP__rxBuf.buf[tmp] & 0x00ff;
  crc = (crc << 8) & 0xFF00;
  tmp = tmp == 0 ? SerialP__RX_DATA_BUFFER_SIZE : tmp - 1;
  crc |= SerialP__rxBuf.buf[tmp] & 0x00FF;
  return crc;
}

# 69 "/opt/tinyos-2.x/tos/lib/serial/ReceiveBytePacket.nc"
inline static void SerialP__ReceiveBytePacket__endPacket(error_t result){
#line 69
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(result);
#line 69
}
#line 69
# 210 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBufferSwap(void )
#line 210
{
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which ? 0 : 1;
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer = (uint8_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messagePtrs[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which];
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 232 "/opt/tinyos-2.x/tos/lib/serial/SerialP.nc"
static __inline bool SerialP__ack_queue_is_full(void )
#line 232
{
  uint8_t tmp;
#line 233
  uint8_t tmp2;

  /* atomic removed: atomic calls only */
#line 234
  {
    tmp = SerialP__ackQ.writePtr;
    tmp2 = SerialP__ackQ.readPtr;
  }
  if (++tmp > SerialP__ACK_QUEUE_SIZE) {
#line 238
    tmp = 0;
    }
#line 239
  return tmp == tmp2;
}







static __inline void SerialP__ack_queue_push(uint8_t token)
#line 248
{
  if (!SerialP__ack_queue_is_full()) {
      /* atomic removed: atomic calls only */
#line 250
      {
        SerialP__ackQ.buf[SerialP__ackQ.writePtr] = token;
        if (++ SerialP__ackQ.writePtr > SerialP__ACK_QUEUE_SIZE) {
#line 252
          SerialP__ackQ.writePtr = 0;
          }
      }
#line 254
      SerialP__MaybeScheduleTx();
    }
}

# 233 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(uint8_t b)
#line 233
{
  /* atomic removed: atomic calls only */
#line 234
  {
    switch (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state) {
        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_BEGIN: 
          /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_DATA;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(b);
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType = b;
        break;

        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_DATA: 
          if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex < sizeof(message_t )) {
              /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex] = b;
              /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex++;
            }
          else {
            }




        break;

        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_IDLE: 
          default: 
#line 255
            ;
      }
  }
}

# 58 "/opt/tinyos-2.x/tos/lib/serial/ReceiveBytePacket.nc"
inline static void SerialP__ReceiveBytePacket__byteReceived(uint8_t data){
#line 58
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(data);
#line 58
}
#line 58
# 299 "/opt/tinyos-2.x/tos/lib/serial/SerialP.nc"
static __inline uint8_t SerialP__rx_buffer_top(void )
#line 299
{
  uint8_t tmp = SerialP__rxBuf.buf[SerialP__rxBuf.readPtr];

#line 301
  return tmp;
}

#line 303
static __inline uint8_t SerialP__rx_buffer_pop(void )
#line 303
{
  uint8_t tmp = SerialP__rxBuf.buf[SerialP__rxBuf.readPtr];

#line 305
  if (++ SerialP__rxBuf.readPtr > SerialP__RX_DATA_BUFFER_SIZE) {
#line 305
    SerialP__rxBuf.readPtr = 0;
    }
#line 306
  return tmp;
}

#line 295
static __inline void SerialP__rx_buffer_push(uint8_t data)
#line 295
{
  SerialP__rxBuf.buf[SerialP__rxBuf.writePtr] = data;
  if (++ SerialP__rxBuf.writePtr > SerialP__RX_DATA_BUFFER_SIZE) {
#line 297
    SerialP__rxBuf.writePtr = 0;
    }
}

# 55 "/opt/tinyos-2.x/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC__SerialFrameComm__resetReceive(void )
#line 55
{
  HdlcTranslateC__state.receiveEscape = 0;
}

# 68 "/opt/tinyos-2.x/tos/lib/serial/SerialFrameComm.nc"
inline static void SerialP__SerialFrameComm__resetReceive(void ){
#line 68
  HdlcTranslateC__SerialFrameComm__resetReceive();
#line 68
}
#line 68
#line 54
inline static error_t SerialP__SerialFrameComm__putData(uint8_t data){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = HdlcTranslateC__SerialFrameComm__putData(data);
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 513 "/opt/tinyos-2.x/tos/lib/serial/SerialP.nc"
static inline error_t SerialP__SendBytePacket__completeSend(void )
#line 513
{
  bool ret = FAIL;

  /* atomic removed: atomic calls only */
#line 515
  {
    SerialP__txBuf[SerialP__TX_DATA_INDEX].state = SerialP__BUFFER_COMPLETE;
    ret = SUCCESS;
  }
  return ret;
}

# 60 "/opt/tinyos-2.x/tos/lib/serial/SendBytePacket.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__completeSend(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = SerialP__SendBytePacket__completeSend();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 167 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte(void )
#line 167
{
  uint8_t b;
  uint8_t indx;

  /* atomic removed: atomic calls only */
#line 170
  {
    b = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex];
    /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex++;
    indx = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex;
  }
  if (indx > /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendLen) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__completeSend();
      return 0;
    }
  else {
      return b;
    }
}

# 70 "/opt/tinyos-2.x/tos/lib/serial/SendBytePacket.nc"
inline static uint8_t SerialP__SendBytePacket__nextByte(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 642 "/opt/tinyos-2.x/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFrameComm__putDone(void )
#line 642
{
  {
    error_t txResult = SUCCESS;

    switch (SerialP__txState) {

        case SerialP__TXSTATE_PROTO: 

          txResult = SerialP__SerialFrameComm__putData(SerialP__txProto);

        SerialP__txState = SerialP__TXSTATE_INFO;



        SerialP__txCRC = crcByte(SerialP__txCRC, SerialP__txProto);
        break;

        case SerialP__TXSTATE_SEQNO: 
          txResult = SerialP__SerialFrameComm__putData(SerialP__txSeqno);
        SerialP__txState = SerialP__TXSTATE_INFO;
        SerialP__txCRC = crcByte(SerialP__txCRC, SerialP__txSeqno);
        break;

        case SerialP__TXSTATE_INFO: 
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 666
            {
              txResult = SerialP__SerialFrameComm__putData(SerialP__txBuf[SerialP__txIndex].buf);
              SerialP__txCRC = crcByte(SerialP__txCRC, SerialP__txBuf[SerialP__txIndex].buf);
              ++SerialP__txByteCnt;

              if (SerialP__txIndex == SerialP__TX_DATA_INDEX) {
                  uint8_t nextByte;

#line 673
                  nextByte = SerialP__SendBytePacket__nextByte();
                  if (SerialP__txBuf[SerialP__txIndex].state == SerialP__BUFFER_COMPLETE || SerialP__txByteCnt >= SerialP__SERIAL_MTU) {
                      SerialP__txState = SerialP__TXSTATE_FCS1;
                    }
                  else {
                      SerialP__txBuf[SerialP__txIndex].buf = nextByte;
                    }
                }
              else {
                  SerialP__txState = SerialP__TXSTATE_FCS1;
                }
            }
#line 684
            __nesc_atomic_end(__nesc_atomic); }
        break;

        case SerialP__TXSTATE_FCS1: 
          txResult = SerialP__SerialFrameComm__putData(SerialP__txCRC & 0xff);
        SerialP__txState = SerialP__TXSTATE_FCS2;
        break;

        case SerialP__TXSTATE_FCS2: 
          txResult = SerialP__SerialFrameComm__putData((SerialP__txCRC >> 8) & 0xff);
        SerialP__txState = SerialP__TXSTATE_ENDFLAG;
        break;

        case SerialP__TXSTATE_ENDFLAG: 
          txResult = SerialP__SerialFrameComm__putDelimiter();
        SerialP__txState = SerialP__TXSTATE_ENDWAIT;
        break;

        case SerialP__TXSTATE_ENDWAIT: 
          SerialP__txState = SerialP__TXSTATE_FINISH;
        case SerialP__TXSTATE_FINISH: 
          SerialP__MaybeScheduleTx();
        break;
        case SerialP__TXSTATE_ERROR: 
          default: 
            txResult = FAIL;
        break;
      }

    if (txResult != SUCCESS) {
        SerialP__txState = SerialP__TXSTATE_ERROR;
        SerialP__MaybeScheduleTx();
      }
  }
}

# 89 "/opt/tinyos-2.x/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC__SerialFrameComm__putDone(void ){
#line 89
  SerialP__SerialFrameComm__putDone();
#line 89
}
#line 89
# 48 "/opt/tinyos-2.x/tos/interfaces/UartStream.nc"
inline static error_t HdlcTranslateC__UartStream__send(uint8_t * buf, uint16_t len){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = /*Atm128Uart0C.UartP*/Atm128UartP__0__UartStream__send(buf, len);
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 104 "/opt/tinyos-2.x/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC__UartStream__sendDone(uint8_t *buf, uint16_t len, 
error_t error)
#line 105
{
  if (HdlcTranslateC__state.sendEscape) {
      HdlcTranslateC__state.sendEscape = 0;
      HdlcTranslateC__m_data = HdlcTranslateC__txTemp;
      HdlcTranslateC__UartStream__send(&HdlcTranslateC__m_data, 1);
    }
  else {
      HdlcTranslateC__SerialFrameComm__putDone();
    }
}

# 57 "/opt/tinyos-2.x/tos/interfaces/UartStream.nc"
inline static void /*Atm128Uart0C.UartP*/Atm128UartP__0__UartStream__sendDone(uint8_t * buf, uint16_t len, error_t error){
#line 57
  HdlcTranslateC__UartStream__sendDone(buf, len, error);
#line 57
}
#line 57
# 46 "/opt/tinyos-2.x/tos/chips/atm128/HplAtm128Uart.nc"
inline static void /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__tx(uint8_t data){
#line 46
  HplAtm128UartP__HplUart0__tx(data);
#line 46
}
#line 46
# 173 "/opt/tinyos-2.x/tos/chips/atm128/Atm128UartP.nc"
static inline void /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__txDone(void )
#line 173
{

  if (/*Atm128Uart0C.UartP*/Atm128UartP__0__m_tx_pos < /*Atm128Uart0C.UartP*/Atm128UartP__0__m_tx_len) {
      /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__tx(/*Atm128Uart0C.UartP*/Atm128UartP__0__m_tx_buf[/*Atm128Uart0C.UartP*/Atm128UartP__0__m_tx_pos++]);
    }
  else {
      uint8_t *buf = /*Atm128Uart0C.UartP*/Atm128UartP__0__m_tx_buf;

#line 180
      /*Atm128Uart0C.UartP*/Atm128UartP__0__m_tx_buf = (void *)0;
      /*Atm128Uart0C.UartP*/Atm128UartP__0__m_tx_intr = 0;
      /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__disableTxIntr();
      /*Atm128Uart0C.UartP*/Atm128UartP__0__UartStream__sendDone(buf, /*Atm128Uart0C.UartP*/Atm128UartP__0__m_tx_len, SUCCESS);
    }
}

# 47 "/opt/tinyos-2.x/tos/chips/atm128/HplAtm128Uart.nc"
inline static void HplAtm128UartP__HplUart0__txDone(void ){
#line 47
  /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__txDone();
#line 47
}
#line 47
# 300 "/opt/tinyos-2.x/tos/chips/atm1281/HplAtm128UartP.nc"
static inline void HplAtm128UartP__HplUart1__default__rxDone(uint8_t data)
#line 300
{
}

# 49 "/opt/tinyos-2.x/tos/chips/atm128/HplAtm128Uart.nc"
inline static void HplAtm128UartP__HplUart1__rxDone(uint8_t data){
#line 49
  HplAtm128UartP__HplUart1__default__rxDone(data);
#line 49
}
#line 49
# 299 "/opt/tinyos-2.x/tos/chips/atm1281/HplAtm128UartP.nc"
static inline void HplAtm128UartP__HplUart1__default__txDone(void )
#line 299
{
}

# 47 "/opt/tinyos-2.x/tos/chips/atm128/HplAtm128Uart.nc"
inline static void HplAtm128UartP__HplUart1__txDone(void ){
#line 47
  HplAtm128UartP__HplUart1__default__txDone();
#line 47
}
#line 47
# 205 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer3P.nc"
static inline void HplAtm1281Timer3P__CompareA__default__fired(void )
#line 205
{
}

# 49 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
inline static void HplAtm1281Timer3P__CompareA__fired(void ){
#line 49
  HplAtm1281Timer3P__CompareA__default__fired();
#line 49
}
#line 49
# 209 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer3P.nc"
static inline void HplAtm1281Timer3P__CompareB__default__fired(void )
#line 209
{
}

# 49 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
inline static void HplAtm1281Timer3P__CompareB__fired(void ){
#line 49
  HplAtm1281Timer3P__CompareB__default__fired();
#line 49
}
#line 49
# 213 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer3P.nc"
static inline void HplAtm1281Timer3P__CompareC__default__fired(void )
#line 213
{
}

# 49 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Compare.nc"
inline static void HplAtm1281Timer3P__CompareC__fired(void ){
#line 49
  HplAtm1281Timer3P__CompareC__default__fired();
#line 49
}
#line 49
# 217 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer3P.nc"
static inline void HplAtm1281Timer3P__Capture__default__captured(uint16_t time)
#line 217
{
}

# 51 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Capture.nc"
inline static void HplAtm1281Timer3P__Capture__captured(HplAtm1281Timer3P__Capture__size_type t){
#line 51
  HplAtm1281Timer3P__Capture__default__captured(t);
#line 51
}
#line 51
# 70 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer3P.nc"
static inline uint16_t HplAtm1281Timer3P__Timer__get(void )
#line 70
{
#line 70
  return * (volatile uint16_t *)0x94;
}

# 216 "/opt/tinyos-2.x/tos/chips/atm128/Atm128UartP.nc"
static inline void /*Atm128Uart0C.UartP*/Atm128UartP__0__Counter__overflow(void )
#line 216
{
}

# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static void /*CounterMicro32C.Transform32*/TransformCounterC__1__Counter__overflow(void ){
#line 71
  /*Atm128Uart0C.UartP*/Atm128UartP__0__Counter__overflow();
#line 71
}
#line 71
# 122 "/opt/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static inline void /*CounterMicro32C.Transform32*/TransformCounterC__1__CounterFrom__overflow(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMicro32C.Transform32*/TransformCounterC__1__m_upper++;
      if ((/*CounterMicro32C.Transform32*/TransformCounterC__1__m_upper & /*CounterMicro32C.Transform32*/TransformCounterC__1__OVERFLOW_MASK) == 0) {
        /*CounterMicro32C.Transform32*/TransformCounterC__1__Counter__overflow();
        }
    }
#line 130
    __nesc_atomic_end(__nesc_atomic); }
}

# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static void /*CounterThree16C.NCounter*/Atm128CounterC__1__Counter__overflow(void ){
#line 71
  /*CounterMicro32C.Transform32*/TransformCounterC__1__CounterFrom__overflow();
#line 71
}
#line 71
# 56 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128CounterC.nc"
static inline void /*CounterThree16C.NCounter*/Atm128CounterC__1__Timer__overflow(void )
{
  /*CounterThree16C.NCounter*/Atm128CounterC__1__Counter__overflow();
}

# 51 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128TimerInitC.nc"
static inline void /*InitThreeP.InitThree*/Atm128TimerInitC__1__Timer__overflow(void )
#line 51
{
}

# 61 "/opt/tinyos-2.x/tos/chips/atm128/timer/HplAtm128Timer.nc"
inline static void HplAtm1281Timer3P__Timer__overflow(void ){
#line 61
  /*InitThreeP.InitThree*/Atm128TimerInitC__1__Timer__overflow();
#line 61
  /*CounterThree16C.NCounter*/Atm128CounterC__1__Timer__overflow();
#line 61
}
#line 61
# 52 "/opt/tinyos-2.x/tos/system/RealMainP.nc"
  int main(void )
#line 52
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      * (volatile uint8_t *)(0x34 + 0x20) = 0;
#line 60
       __asm volatile ("in __tmp_reg__, __SREG__""\n\t""cli""\n\t""sts %0, %1""\n\t""sts %0, __zero_reg__""\n\t""out __SREG__,__tmp_reg__""\n\t" :  : "M"((uint16_t )& * (volatile uint8_t *)0x60), "r"((uint8_t )((1 << 4) | (1 << 3))) : "r0");;

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
static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0xf7540b38){
#line 64
  switch (arg_0xf7540b38) {
#line 64
    case BaseStationP__uartSendTask:
#line 64
      BaseStationP__uartSendTask__runTask();
#line 64
      break;
#line 64
    case BaseStationP__radioSendTask:
#line 64
      BaseStationP__radioSendTask__runTask();
#line 64
      break;
#line 64
    case /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask:
#line 64
      /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__runTask();
#line 64
      break;
#line 64
    case PacketLinkLayerP__send:
#line 64
      PacketLinkLayerP__send__runTask();
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
    case MessageBufferLayerP__stateDoneTask:
#line 64
      MessageBufferLayerP__stateDoneTask__runTask();
#line 64
      break;
#line 64
    case MessageBufferLayerP__sendTask:
#line 64
      MessageBufferLayerP__sendTask__runTask();
#line 64
      break;
#line 64
    case MessageBufferLayerP__deliverTask:
#line 64
      MessageBufferLayerP__deliverTask__runTask();
#line 64
      break;
#line 64
    case RandomCollisionLayerP__calcNextRandom:
#line 64
      RandomCollisionLayerP__calcNextRandom__runTask();
#line 64
      break;
#line 64
    case Atm128SpiP__zeroTask:
#line 64
      Atm128SpiP__zeroTask__runTask();
#line 64
      break;
#line 64
    case /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__grantedTask:
#line 64
      /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__grantedTask__runTask();
#line 64
      break;
#line 64
    case SerialP__RunTx:
#line 64
      SerialP__RunTx__runTask();
#line 64
      break;
#line 64
    case SerialP__startDoneTask:
#line 64
      SerialP__startDoneTask__runTask();
#line 64
      break;
#line 64
    case SerialP__stopDoneTask:
#line 64
      SerialP__stopDoneTask__runTask();
#line 64
      break;
#line 64
    case SerialP__defaultSerialFlushTask:
#line 64
      SerialP__defaultSerialFlushTask__runTask();
#line 64
      break;
#line 64
    case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone:
#line 64
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask();
#line 64
      break;
#line 64
    case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask:
#line 64
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask();
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
    default:
#line 64
      SchedulerBasicP__TaskBasic__default__runTask(arg_0xf7540b38);
#line 64
      break;
#line 64
    }
#line 64
}
#line 64
# 155 "/opt/tinyos-2.x/tos/system/AMQueueImplP.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(uint8_t last, message_t * msg, error_t err)
#line 155
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg = (void *)0;
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend();
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(last, msg, err);
}

# 63 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
static ieee154_header_t *Ieee154PacketLayerP__getHeader(message_t *msg)
{
  return (void *)msg + Ieee154PacketLayerP__SubPacket__headerLength(msg);
}

# 63 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/ActiveMessageLayerP.nc"
static error_t ActiveMessageLayerP__AMSend__send(am_id_t id, am_addr_t addr, message_t *msg, uint8_t len)
{
  if (len > ActiveMessageLayerP__Packet__maxPayloadLength()) {
    return EINVAL;
    }
  if (ActiveMessageLayerP__Config__checkFrame(msg) != SUCCESS) {
    return FAIL;
    }
  ActiveMessageLayerP__Packet__setPayloadLength(msg, len);
  ActiveMessageLayerP__AMPacket__setSource(msg, ActiveMessageLayerP__AMPacket__address());
  ActiveMessageLayerP__AMPacket__setGroup(msg, ActiveMessageLayerP__AMPacket__localGroup());
  ActiveMessageLayerP__AMPacket__setType(msg, id);
  ActiveMessageLayerP__AMPacket__setDestination(msg, addr);

  ActiveMessageLayerP__SendNotifier__aboutToSend(id, addr, msg);

  return ActiveMessageLayerP__SubSend__send(msg);
}

# 872 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static void RF230DriverLayerP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length)
{
  for (; 0; ) ;
  for (; 0; ) ;


  __nesc_hton_leuint8(RF230DriverLayerP__getHeader(msg)->length.data, length + 2);
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

# 121 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TinyosNetworkLayerC.nc"
static error_t TinyosNetworkLayerC__TinyosSend__send(message_t *msg)
{

  __nesc_hton_leuint8(TinyosNetworkLayerC__getHeader(msg)->network.data, 0x3f);

  return TinyosNetworkLayerC__SubSend__send(msg);
}

#line 116
static network_header_t *TinyosNetworkLayerC__getHeader(message_t *msg)
{
  return (void *)msg + TinyosNetworkLayerC__SubPacket__headerLength(msg);
}

# 126 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
static void Ieee154PacketLayerP__Ieee154PacketLayer__setAckRequired(message_t *msg, bool ack)
{
  unsigned char *__nesc_temp43;
  unsigned char *__nesc_temp42;

#line 128
  if (ack) {
    (__nesc_temp42 = Ieee154PacketLayerP__getHeader(msg)->fcf.data, __nesc_hton_leuint16(__nesc_temp42, __nesc_ntoh_leuint16(__nesc_temp42) | (1 << IEEE154_FCF_ACK_REQ)));
    }
  else {
#line 131
    (__nesc_temp43 = Ieee154PacketLayerP__getHeader(msg)->fcf.data, __nesc_hton_leuint16(__nesc_temp43, __nesc_ntoh_leuint16(__nesc_temp43) & ~ (uint16_t )(1 << IEEE154_FCF_ACK_REQ)));
    }
}

# 209 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MessageBufferLayerP.nc"
static error_t MessageBufferLayerP__Send__send(message_t *msg)
{
  if (MessageBufferLayerP__state != MessageBufferLayerP__STATE_READY) {
    return EBUSY;
    }
  MessageBufferLayerP__txMsg = msg;
  MessageBufferLayerP__state = MessageBufferLayerP__STATE_TX_PENDING;
  MessageBufferLayerP__retries = 0;
  MessageBufferLayerP__sendTask__postTask();

  return SUCCESS;
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

# 97 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static error_t /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__release(uint8_t id)
#line 97
{
  bool released = FALSE;

#line 99
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 99
    {
      if (/*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__state == /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_BUSY && /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId == id) {
          if (/*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Queue__isEmpty() == FALSE) {
              /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId = /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__NO_RES;
              /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__reqResId = /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Queue__dequeue();
              /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__state = /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_GRANTING;
              /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__postTask();
            }
          else {
              /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId = /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__NO_RES;
              /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__state = /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_IDLE;
            }
          released = TRUE;
        }
    }
#line 113
    __nesc_atomic_end(__nesc_atomic); }
  if (released == TRUE) {
      /*RF230RadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__unconfigure(id);
      return SUCCESS;
    }
  return FAIL;
}

# 48 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static void /*HplAtm128GeneralIOC.PortA.Bit0*/HplAtm128GeneralIOPinP__0__IO__toggle(void )
#line 48
{
#line 48
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 48
    * (volatile uint8_t * )34U ^= 1 << 0;
#line 48
    __nesc_atomic_end(__nesc_atomic); }
}

#line 48
static void /*HplAtm128GeneralIOC.PortA.Bit2*/HplAtm128GeneralIOPinP__2__IO__toggle(void )
#line 48
{
#line 48
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 48
    * (volatile uint8_t * )34U ^= 1 << 2;
#line 48
    __nesc_atomic_end(__nesc_atomic); }
}

# 518 "../InterceptBase/BaseStationP.nc"
static void BaseStationP__timedUartSendTask(void )
#line 518
{

  if (BaseStationP__inReset == TRUE) {
#line 520
    return;
    }


  if (BaseStationP__UartTimer__isRunning() == FALSE) {
      BaseStationP__UartTimer__startOneShot(BaseStationP__UART_TIME);
    }
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

# 188 "/opt/tinyos-2.x/tos/chips/atm1281/timer/Atm1281AlarmAsyncP.nc"
static uint32_t /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Counter__get(void )
#line 188
{
  uint32_t now;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {


      uint8_t now8 = /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Timer__get();

      if (((Atm128_TIFR2_t )/*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__TimerCtrl__getInterruptFlag()).bits.ocfa) {


        now = /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__base + /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Compare__get() + 1 + /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Timer__get();
        }
      else {

        now = /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__base + now8;
        }
    }
#line 206
    __nesc_atomic_end(__nesc_atomic); }
#line 206
  return now;
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static void BaseStationP__RadioTimer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(3U, dt);
#line 62
}
#line 62
static void BaseStationP__ResetTimer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(1U, dt);
#line 62
}
#line 62
# 48 "/opt/tinyos-2.x/tos/chips/atm128/pins/HplAtm128GeneralIOPinP.nc"
static void /*HplAtm128GeneralIOC.PortA.Bit1*/HplAtm128GeneralIOPinP__1__IO__toggle(void )
#line 48
{
#line 48
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 48
    * (volatile uint8_t * )34U ^= 1 << 1;
#line 48
    __nesc_atomic_end(__nesc_atomic); }
}

# 347 "/opt/tinyos-2.x/tos/lib/serial/SerialP.nc"
static void SerialP__testOff(void )
#line 347
{
  bool turnOff = FALSE;

#line 349
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 349
    {
      if (SerialP__txState == SerialP__TXSTATE_INACTIVE && 
      SerialP__rxState == SerialP__RXSTATE_INACTIVE) {
          turnOff = TRUE;
        }
    }
#line 354
    __nesc_atomic_end(__nesc_atomic); }
  if (turnOff) {
      SerialP__stopDoneTask__postTask();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 357
        SerialP__offPending = FALSE;
#line 357
        __nesc_atomic_end(__nesc_atomic); }
    }
  else {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 360
        SerialP__offPending = TRUE;
#line 360
        __nesc_atomic_end(__nesc_atomic); }
    }
}

# 86 "/opt/tinyos-2.x/tos/lib/serial/HdlcTranslateC.nc"
static error_t HdlcTranslateC__SerialFrameComm__putDelimiter(void )
#line 86
{
  HdlcTranslateC__state.sendEscape = 0;
  HdlcTranslateC__m_data = HDLC_FLAG_BYTE;
  return HdlcTranslateC__UartStream__send(&HdlcTranslateC__m_data, 1);
}

# 155 "/opt/tinyos-2.x/tos/chips/atm128/Atm128UartP.nc"
static error_t /*Atm128Uart0C.UartP*/Atm128UartP__0__UartStream__send(uint8_t *buf, uint16_t len)
#line 155
{

  if (len == 0) {
    return FAIL;
    }
  else {
#line 159
    if (/*Atm128Uart0C.UartP*/Atm128UartP__0__m_tx_buf) {
      return EBUSY;
      }
    }
#line 162
  /*Atm128Uart0C.UartP*/Atm128UartP__0__m_tx_len = len;
  /*Atm128Uart0C.UartP*/Atm128UartP__0__m_tx_buf = buf;
  /*Atm128Uart0C.UartP*/Atm128UartP__0__m_tx_pos = 0;
  /*Atm128Uart0C.UartP*/Atm128UartP__0__m_tx_intr = 1;
  /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__enableTxIntr();
  /*Atm128Uart0C.UartP*/Atm128UartP__0__HplUart__tx(buf[/*Atm128Uart0C.UartP*/Atm128UartP__0__m_tx_pos++]);

  return SUCCESS;
}

# 187 "/opt/tinyos-2.x/tos/chips/atm1281/HplAtm128UartP.nc"
static void HplAtm128UartP__HplUart0__tx(uint8_t data)
#line 187
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 188
    {
      * (volatile uint8_t *)0XC6 = data;
      * (volatile uint8_t *)0xC0 |= 1 << 6;
    }
#line 191
    __nesc_atomic_end(__nesc_atomic); }
}

# 502 "/opt/tinyos-2.x/tos/lib/serial/SerialP.nc"
static void SerialP__MaybeScheduleTx(void )
#line 502
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 503
    {
      if (SerialP__txPending == 0) {
          if (SerialP__RunTx__postTask() == SUCCESS) {
              SerialP__txPending = 1;
            }
        }
    }
#line 509
    __nesc_atomic_end(__nesc_atomic); }
}

# 356 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
static error_t Atm128SpiP__Resource__release(uint8_t id)
#line 356
{
  error_t error = Atm128SpiP__ResourceArbiter__release(id);

#line 358
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 358
    {
      if (!Atm128SpiP__ArbiterInfo__inUse()) {
          Atm128SpiP__stopSpi();
        }
    }
#line 362
    __nesc_atomic_end(__nesc_atomic); }
  return error;
}

# 124 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static bool /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__ArbiterInfo__inUse(void )
#line 124
{
  /* atomic removed: atomic calls only */
#line 125
  {
    if (/*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__state == /*Atm128SpiC.Arbiter.Arbiter*/SimpleArbiterP__1__RES_IDLE) 
      {
        unsigned char __nesc_temp = 
#line 127
        FALSE;

#line 127
        return __nesc_temp;
      }
  }
#line 129
  return TRUE;
}

# 130 "/opt/tinyos-2.x/tos/chips/atm128/spi/HplAtm128SpiP.nc"
static void HplAtm128SpiP__SPI__enableSpi(bool enabled)
#line 130
{
  if (enabled) {
      * (volatile uint8_t *)(0x2C + 0x20) |= 1 << 6;
      HplAtm128SpiP__Mcu__update();
    }
  else {
      * (volatile uint8_t *)(0x2C + 0x20) &= ~(1 << 6);
      HplAtm128SpiP__Mcu__update();
    }
}

# 101 "/opt/tinyos-2.x/tos/chips/rf2xx/util/TaskletC.nc"
static void TaskletC__Tasklet__schedule(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      if (TaskletC__state != 0) 
        {
          TaskletC__state |= 0x80;
          {
#line 108
            __nesc_atomic_end(__nesc_atomic); 
#line 108
            return;
          }
        }
      TaskletC__state = 1;
    }
#line 112
    __nesc_atomic_end(__nesc_atomic); }

  TaskletC__doit();
}

#line 63
static void TaskletC__doit(void )
{
  for (; ; ) 
    {
      TaskletC__Tasklet__run();

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          if (TaskletC__state == 1) 
            {
              TaskletC__state = 0;
              {
#line 74
                __nesc_atomic_end(__nesc_atomic); 
#line 74
                return;
              }
            }
          for (; 0; ) ;
          TaskletC__state = 1;
        }
#line 79
        __nesc_atomic_end(__nesc_atomic); }
    }
}

# 272 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static bool RF230DriverLayerP__isSpiAcquired(void )
{
  if (RF230DriverLayerP__SpiResource__isOwner()) {
    return TRUE;
    }
  if (RF230DriverLayerP__SpiResource__immediateRequest() == SUCCESS) 
    {
      RF230DriverLayerP__SELN__makeOutput();
      RF230DriverLayerP__SELN__set();

      return TRUE;
    }

  RF230DriverLayerP__SpiResource__request();
  return FALSE;
}

# 106 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
static void Atm128SpiP__startSpi(void )
#line 106
{
  Atm128SpiP__Spi__enableSpi(FALSE);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 108
    {
      Atm128SpiP__Spi__initMaster();
      Atm128SpiP__Spi__enableInterrupt(FALSE);
      Atm128SpiP__Spi__setMasterDoubleSpeed(TRUE);
      Atm128SpiP__Spi__setClockPolarity(FALSE);
      Atm128SpiP__Spi__setClockPhase(FALSE);
      Atm128SpiP__Spi__setClock(0);
      Atm128SpiP__Spi__enableSpi(TRUE);
    }
#line 116
    __nesc_atomic_end(__nesc_atomic); }
  Atm128SpiP__McuPowerState__update();
}

# 115 "/opt/tinyos-2.x/tos/chips/atm128/spi/HplAtm128SpiP.nc"
static void HplAtm128SpiP__SPI__enableInterrupt(bool enabled)
#line 115
{
  if (enabled) {
      * (volatile uint8_t *)(0x2C + 0x20) |= 1 << 7;
      HplAtm128SpiP__Mcu__update();
    }
  else {
      * (volatile uint8_t *)(0x2C + 0x20) &= ~(1 << 7);
      HplAtm128SpiP__Mcu__update();
    }
}

# 347 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
static error_t Atm128SpiP__Resource__request(uint8_t id)
#line 347
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 348
    {
      if (!Atm128SpiP__ArbiterInfo__inUse()) {
          Atm128SpiP__startSpi();
        }
    }
#line 352
    __nesc_atomic_end(__nesc_atomic); }
  return Atm128SpiP__ResourceArbiter__request(id);
}

# 43 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MetadataFlagsLayerC.nc"
static flags_metadata_t *MetadataFlagsLayerC__getMeta(message_t *msg)
{
  return (void *)msg + sizeof(message_t ) - MetadataFlagsLayerC__RadioPacket__metadataLength(msg);
}

# 93 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static rf230_metadata_t *RF230DriverLayerP__getMeta(message_t *msg)
{
  return (void *)msg + sizeof(message_t ) - RF230DriverLayerP__RadioPacket__metadataLength(msg);
}

# 69 "/opt/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__to_size_type /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__Counter__get(void )
{
  /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__to_size_type rv = 0;

#line 72
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__upper_count_type high = /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__m_upper;
      /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__from_size_type low = /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__CounterFrom__get();

#line 76
      if (/*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__CounterFrom__get();
        }
      {
        /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__to_size_type high_to = high;
        /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__to_size_type low_to = low >> /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__LOW_SHIFT_RIGHT;

#line 90
        rv = (high_to << /*LocalTimeMicroC.TransformCounterC*/TransformCounterC__0__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 39 "/opt/tinyos-2.x/tos/chips/rf2xx/util/PacketFlag.nc"
static void TimeStampingLayerP__TimeStampFlag__set(message_t *msg){
#line 39
  MetadataFlagsLayerC__PacketFlag__set(1U, msg);
#line 39
}
#line 39
# 49 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/TimeStampingLayerP.nc"
static timestamp_metadata_t *TimeStampingLayerP__getMeta(message_t *msg)
{
  return (void *)msg + sizeof(message_t ) - TimeStampingLayerP__RadioPacket__metadataLength(msg);
}

# 74 "/opt/tinyos-2.x/tos/chips/atm128/timer/Atm128AlarmC.nc"
static void /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__Alarm__startAt(/*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__timer_size t0, /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__timer_size dt)
#line 74
{






  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__timer_size now;
#line 83
      /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__timer_size elapsed;
#line 83
      /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__timer_size expires;

      ;


      now = /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Timer__get();
      elapsed = now + 3 - t0;
      if (elapsed >= dt) {
        expires = now + 3;
        }
      else {
#line 93
        expires = t0 + dt;
        }



      if (expires == 0) {
        expires = 1;
        }



      /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Compare__set(expires - 1);
      /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Compare__reset();
      /*HplRF230C.AlarmC.NAlarm*/Atm128AlarmC__0__HplAtm128Compare__start();
    }
#line 107
    __nesc_atomic_end(__nesc_atomic); }
}

# 201 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MessageBufferLayerP.nc"
static void MessageBufferLayerP__RadioSend__sendDone(error_t error)
{
  for (; 0; ) ;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 205
    MessageBufferLayerP__txError = error;
#line 205
    __nesc_atomic_end(__nesc_atomic); }
  MessageBufferLayerP__sendTask__postTask();
}

# 88 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static void *RF230DriverLayerP__getPayload(message_t *msg)
{
  return (void *)msg + RF230DriverLayerP__RadioPacket__headerLength(msg);
}

# 194 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/Ieee154PacketLayerP.nc"
static bool Ieee154PacketLayerP__Ieee154PacketLayer__requiresAckReply(message_t *msg)
{
  return Ieee154PacketLayerP__Ieee154PacketLayer__getAckRequired(msg)
   && Ieee154PacketLayerP__Ieee154PacketLayer__isDataFrame(msg)
   && Ieee154PacketLayerP__Ieee154PacketLayer__getDestAddr(msg) == Ieee154PacketLayerP__ActiveMessageAddress__amAddress();
}

# 417 "/opt/tinyos-2.x/tos/chips/rf2xx/rf230/RF230DriverLayerP.nc"
static error_t RF230DriverLayerP__RadioSend__send(message_t *msg)
{
  uint16_t time;
  uint8_t length;
  uint8_t *data;
  uint8_t header;
  uint32_t time32;
  void *timesync;

  if (((RF230DriverLayerP__cmd != RF230DriverLayerP__CMD_NONE || RF230DriverLayerP__state != RF230DriverLayerP__STATE_RX_ON) || !RF230DriverLayerP__isSpiAcquired()) || RF230DriverLayerP__radioIrq) {
    return EBUSY;
    }
  length = (RF230DriverLayerP__PacketTransmitPower__isSet(msg) ? 
  RF230DriverLayerP__PacketTransmitPower__get(msg) : 0) & RF230_TX_PWR_MASK;

  if (length != RF230DriverLayerP__txPower) 
    {
      RF230DriverLayerP__txPower = length;
      RF230DriverLayerP__writeRegister(RF230_PHY_TX_PWR, RF230_TX_AUTO_CRC_ON | RF230DriverLayerP__txPower);
    }


  if (
#line 438
  RF230DriverLayerP__Config__requiresRssiCca(msg)
   && (RF230DriverLayerP__readRegister(RF230_PHY_RSSI) & RF230_RSSI_MASK) > (RF230DriverLayerP__rssiClear + RF230DriverLayerP__rssiBusy) >> 3) {
    return EBUSY;
    }
  RF230DriverLayerP__writeRegister(RF230_TRX_STATE, RF230_PLL_ON);


  time32 = RF230DriverLayerP__LocalTime__get();
  timesync = RF230DriverLayerP__PacketTimeSyncOffset__isSet(msg) ? (void *)msg + RF230DriverLayerP__PacketTimeSyncOffset__get(msg) : 0;


  if ((RF230DriverLayerP__readRegister(RF230_TRX_STATUS) & RF230_TRX_STATUS_MASK) != RF230_PLL_ON) 
    {
      for (; 0; ) ;

      RF230DriverLayerP__state = RF230DriverLayerP__STATE_PLL_ON_2_RX_ON;
      return EBUSY;
    }


  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      RF230DriverLayerP__SLP_TR__set();
      time = RF230DriverLayerP__RadioAlarm__getNow();
    }
#line 462
    __nesc_atomic_end(__nesc_atomic); }
  RF230DriverLayerP__SLP_TR__clr();


  for (; 0; ) ;

  RF230DriverLayerP__SELN__clr();
  RF230DriverLayerP__FastSpiByte__splitWrite(RF230_CMD_FRAME_WRITE);

  data = RF230DriverLayerP__getPayload(msg);
  length = __nesc_ntoh_leuint8(RF230DriverLayerP__getHeader(msg)->length.data);


  RF230DriverLayerP__FastSpiByte__splitReadWrite(length);


  length -= 2;

  header = RF230DriverLayerP__Config__headerPreloadLength();
  if (header > length) {
    header = length;
    }
  length -= header;


  do {
      RF230DriverLayerP__FastSpiByte__splitReadWrite(* data++);
    }
  while (--header != 0);










  time32 += (int16_t )(time + RF230DriverLayerP__TX_SFD_DELAY) - (int16_t )time32;

  if (timesync != 0) {
    __nesc_hton_int32((* (timesync_relative_t *)timesync).data, * (timesync_absolute_t *)timesync - time32);
    }
  while (length-- != 0) 
    RF230DriverLayerP__FastSpiByte__splitReadWrite(* data++);


  RF230DriverLayerP__FastSpiByte__splitRead();
  RF230DriverLayerP__SELN__set();
#line 526
  RF230DriverLayerP__writeRegister(RF230_TRX_STATE, RF230_RX_ON);

  if (timesync != 0) {
    * (timesync_absolute_t *)timesync = __nesc_ntoh_int32((* (timesync_relative_t *)timesync).data) + time32;
    }
  RF230DriverLayerP__PacketTimeStamp__set(msg, time32);
#line 548
  RF230DriverLayerP__state = RF230DriverLayerP__STATE_BUSY_TX_2_RX_ON;
  RF230DriverLayerP__cmd = RF230DriverLayerP__CMD_TRANSMIT;

  return SUCCESS;
}

# 38 "/opt/tinyos-2.x/tos/chips/rf2xx/util/RadioAlarm.nc"
static void RandomCollisionLayerP__RadioAlarm__wait(uint16_t timeout){
#line 38
  RadioAlarmP__RadioAlarm__wait(1U, timeout);
#line 38
}
#line 38
# 73 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/RandomCollisionLayerP.nc"
static uint16_t RandomCollisionLayerP__getBackoff(uint16_t maxBackoff)
{
  uint16_t a;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      a = RandomCollisionLayerP__nextRandom;
      RandomCollisionLayerP__nextRandom += 273;
    }
#line 81
    __nesc_atomic_end(__nesc_atomic); }
  RandomCollisionLayerP__calcNextRandom__postTask();

  return a % maxBackoff + RandomCollisionLayerP__Config__getMinimumBackoff();
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

# 461 "../InterceptBase/BaseStationP.nc"
static message_t *BaseStationP__receive(message_t *msg, void *payload, uint8_t len, am_id_t id)
#line 461
{
  message_t *ret = msg;


  if (BaseStationP__inReset == TRUE) {
#line 465
    return ret;
    }
  if (!BaseStationP__RadioIntercept__forward(id, msg, payload, len)) {
    return ret;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {

      if (!BaseStationP__uartFull) {
          ret = BaseStationP__uartQueue[BaseStationP__uartIn];
          BaseStationP__uartQueue[BaseStationP__uartIn] = msg;

          BaseStationP__uartIn = (BaseStationP__uartIn + 1) % BaseStationP__UART_QUEUE_LEN;

          if (BaseStationP__uartIn == BaseStationP__uartOut) {
            BaseStationP__uartFull = TRUE;
            }
          if (!BaseStationP__uartBusy) {


              BaseStationP__timedUartSendTask();

              BaseStationP__uartBusy = TRUE;
            }
          else {
            }
        }
      else {





          BaseStationP__failBlink();
          BaseStationP__dropBlink();


          BaseStationP__uartFull = TRUE;
          BaseStationP__radioFull = TRUE;
          ++BaseStationP__uartFailCounter;
          BaseStationP__UartTimer__stop();




          BaseStationP__Reset__reset();
        }
    }
#line 512
    __nesc_atomic_end(__nesc_atomic); }


  return ret;
}

# 117 "RssiBaseC.nc"
static void RssiBaseC__sendBlink(void )
#line 117
{

  return;


  if (RssiBaseC__blinkCnSend == 0) {
      RssiBaseC__Leds__led1Toggle();
    }


  RssiBaseC__blinkCnSend = 0;
}

# 88 "/opt/tinyos-2.x/tos/chips/rf2xx/util/TaskletC.nc"
static void TaskletC__Tasklet__resume(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      if (--TaskletC__state != 0x80) {
        {
#line 93
          __nesc_atomic_end(__nesc_atomic); 
#line 93
          return;
        }
        }
#line 95
      TaskletC__state = 1;
    }
#line 96
    __nesc_atomic_end(__nesc_atomic); }

  TaskletC__doit();
}

# 218 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/PacketLinkLayerP.nc"
static void PacketLinkLayerP__signalDone(error_t error)
#line 218
{
  message_t *msg = PacketLinkLayerP__currentSendMsg;

#line 220
  PacketLinkLayerP__currentSendMsg = (void *)0;




  PacketLinkLayerP__DelayTimer__stop();
  PacketLinkLayerP__PacketLink__setRetries(msg, PacketLinkLayerP__totalRetries);
  PacketLinkLayerP__Send__sendDone(msg, error);
}

#line 127
static uint16_t PacketLinkLayerP__PacketLink__getRetryDelay(message_t *msg)
#line 127
{
  return PacketLinkLayerP__getMeta(msg)->retryDelay;
}

# 63 "/opt/tinyos-2.x/tos/chips/rf2xx/layers/MessageBufferLayerP.nc"
static error_t MessageBufferLayerP__SplitControl__start(void )
{
  error_t error;

  MessageBufferLayerP__Tasklet__suspend();

  if (MessageBufferLayerP__state != MessageBufferLayerP__STATE_READY) {
    error = EBUSY;
    }
  else {
#line 72
    error = MessageBufferLayerP__RadioState__turnOn();
    }
  if (error == SUCCESS) {
    MessageBufferLayerP__state = MessageBufferLayerP__STATE_TURN_ON;
    }
  MessageBufferLayerP__Tasklet__resume();

  return error;
}

static error_t MessageBufferLayerP__SplitControl__stop(void )
{
  error_t error;

  MessageBufferLayerP__Tasklet__suspend();

  if (MessageBufferLayerP__state != MessageBufferLayerP__STATE_READY) {
    error = EBUSY;
    }
  else {
#line 91
    error = MessageBufferLayerP__RadioState__turnOff();
    }
  if (error == SUCCESS) {
    MessageBufferLayerP__state = MessageBufferLayerP__STATE_TURN_OFF;
    }
  MessageBufferLayerP__Tasklet__resume();

  return error;
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

# 239 "/opt/tinyos-2.x/tos/chips/atm1281/timer/Atm1281AlarmAsyncP.nc"
static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__startAt(uint32_t nt0, uint32_t ndt)
#line 239
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__set = TRUE;
      /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__t0 = nt0;
      /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__dt = ndt;
    }
#line 245
    __nesc_atomic_end(__nesc_atomic); }
  /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__setInterrupt();
}

#line 117
static void /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__setInterrupt(void )
#line 117
{
  bool fired = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {



      uint8_t interrupt_in = 1 + /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Compare__get() - /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Timer__get();
      uint8_t newOcr2A;
      uint8_t tifr2 = /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__TimerCtrl__getInterruptFlag();

#line 128
      ;
      if ((interrupt_in != 0 && interrupt_in < /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__MINDT) || tifr2 & (1 << 1)) {
          if (interrupt_in < /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__MINDT) {
              ;
            }
          else {
              ;
            }
          {
#line 136
            __nesc_atomic_end(__nesc_atomic); 
#line 136
            return;
          }
        }

      if (!/*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__set) {
          newOcr2A = /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__MAXT;
          ;
        }
      else 
        {
          uint32_t now = /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Counter__get();

#line 147
          ;

          if ((uint32_t )(now - /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__t0) >= /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__dt) 
            {
              /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__set = FALSE;
              fired = TRUE;
              newOcr2A = /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__MAXT;
            }
          else 
            {


              uint32_t alarm_in = /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__t0 + /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__dt - /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__base;

              if (alarm_in > /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__MAXT) {
                newOcr2A = /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__MAXT;
                }
              else {
#line 163
                if ((uint8_t )alarm_in < /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__MINDT) {
                  newOcr2A = /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__MINDT;
                  }
                else {
#line 166
                  newOcr2A = alarm_in;
                  }
                }
            }
        }
#line 169
      newOcr2A--;
      /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__setOcr2A(newOcr2A);
    }
#line 171
    __nesc_atomic_end(__nesc_atomic); }
  if (fired) {
    /*AlarmCounterMilliP.Atm128AlarmAsyncC.Atm1281AlarmAsyncP*/Atm1281AlarmAsyncP__0__Alarm__fired();
    }
}

# 222 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer2AsyncP.nc"
__attribute((signal))   void __vector_13(void )
#line 222
{
  HplAtm1281Timer2AsyncP__stabiliseTimer2();


  HplAtm1281Timer2AsyncP__Compare__fired();
}


__attribute((signal))   void __vector_15(void )
#line 230
{
  HplAtm1281Timer2AsyncP__stabiliseTimer2();

  HplAtm1281Timer2AsyncP__Timer__overflow();
}

# 102 "/opt/tinyos-2.x/tos/chips/atm128/spi/HplAtm128SpiP.nc"
__attribute((signal))   void __vector_24(void )
#line 102
{
  HplAtm128SpiP__SPI__dataReady(HplAtm128SpiP__SPI__read());
}

# 128 "/opt/tinyos-2.x/tos/chips/atm128/spi/Atm128SpiP.nc"
static uint8_t Atm128SpiP__SpiByte__write(uint8_t tx)
#line 128
{





  Atm128SpiP__Spi__enableSpi(TRUE);
  Atm128SpiP__McuPowerState__update();

  Atm128SpiP__Spi__write(tx);
  while (!(* (volatile uint8_t *)(0x2D + 0x20) & 0x80)) ;
  return Atm128SpiP__Spi__read();
}

# 206 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer1P.nc"
__attribute((interrupt))   void __vector_17(void )
#line 206
{
  HplAtm1281Timer1P__CompareA__fired();
}

__attribute((interrupt))   void __vector_18(void )
#line 210
{
  HplAtm1281Timer1P__CompareB__fired();
}

__attribute((interrupt))   void __vector_19(void )
#line 214
{
  HplAtm1281Timer1P__CompareC__fired();
}

__attribute((interrupt))   void __vector_16(void )
#line 218
{
  HplAtm1281Timer1P__Capture__captured(HplAtm1281Timer1P__Timer__get());
}

__attribute((interrupt))   void __vector_20(void )
#line 222
{
  HplAtm1281Timer1P__Timer__overflow();
}

# 194 "/opt/tinyos-2.x/tos/chips/atm1281/HplAtm128UartP.nc"
__attribute((signal))   void __vector_25(void )
#line 194
{
  if ((* (volatile uint8_t *)0xC0 & (1 << 7)) != 0) {
      HplAtm128UartP__HplUart0__rxDone(* (volatile uint8_t *)0XC6);
    }
}

# 402 "/opt/tinyos-2.x/tos/lib/serial/SerialP.nc"
static void SerialP__rx_state_machine(bool isDelimeter, uint8_t data)
#line 402
{

  switch (SerialP__rxState) {

      case SerialP__RXSTATE_NOSYNC: 
        if (isDelimeter) {
            SerialP__rxInit();
            SerialP__rxState = SerialP__RXSTATE_PROTO;
          }
      break;

      case SerialP__RXSTATE_PROTO: 
        if (!isDelimeter) {
            SerialP__rxCRC = crcByte(SerialP__rxCRC, data);
            SerialP__rxState = SerialP__RXSTATE_TOKEN;
            SerialP__rxProto = data;
            if (!SerialP__valid_rx_proto(SerialP__rxProto)) {
              goto nosync;
              }
            if (SerialP__rxProto != SERIAL_PROTO_PACKET_ACK) {
                goto nosync;
              }
            if (SerialP__ReceiveBytePacket__startPacket() != SUCCESS) {
                goto nosync;
              }
          }
      break;

      case SerialP__RXSTATE_TOKEN: 
        if (isDelimeter) {
            goto nosync;
          }
        else {
            SerialP__rxSeqno = data;
            SerialP__rxCRC = crcByte(SerialP__rxCRC, SerialP__rxSeqno);
            SerialP__rxState = SerialP__RXSTATE_INFO;
          }
      break;

      case SerialP__RXSTATE_INFO: 
        if (SerialP__rxByteCnt < SerialP__SERIAL_MTU) {
            if (isDelimeter) {
                if (SerialP__rxByteCnt >= 2) {
                    if (SerialP__rx_current_crc() == SerialP__rxCRC) {
                        SerialP__ReceiveBytePacket__endPacket(SUCCESS);
                        SerialP__ack_queue_push(SerialP__rxSeqno);
                        goto nosync;
                      }
                    else {
                        goto nosync;
                      }
                  }
                else {
                    goto nosync;
                  }
              }
            else {
                if (SerialP__rxByteCnt >= 2) {
                    SerialP__ReceiveBytePacket__byteReceived(SerialP__rx_buffer_top());
                    SerialP__rxCRC = crcByte(SerialP__rxCRC, SerialP__rx_buffer_pop());
                  }
                SerialP__rx_buffer_push(data);
                SerialP__rxByteCnt++;
              }
          }
        else 

          {
            goto nosync;
          }
      break;

      default: 
        goto nosync;
    }
  goto done;

  nosync: 

    SerialP__rxInit();
  SerialP__SerialFrameComm__resetReceive();
  SerialP__ReceiveBytePacket__endPacket(FAIL);
  if (SerialP__offPending) {
      SerialP__rxState = SerialP__RXSTATE_INACTIVE;
      SerialP__testOff();
    }
  else {
    if (isDelimeter) {
        SerialP__rxState = SerialP__RXSTATE_PROTO;
      }
    }
  done: ;
}

# 81 "/opt/tinyos-2.x/tos/chips/atm128/crc.h"
static __attribute((noinline))  uint16_t crcByte(uint16_t oldCrc, uint8_t byte)
{

  uint16_t *table = crcTable;
  uint16_t newCrc;

   __asm ("eor %1,%B3\n"
  "\tlsl %1\n"
  "\tadc %B2, __zero_reg__\n"
  "\tadd %A2, %1\n"
  "\tadc %B2, __zero_reg__\n"
  "\tlpm\n"
  "\tmov %B0, %A3\n"
  "\tmov %A0, r0\n"
  "\tadiw r30,1\n"
  "\tlpm\n"
  "\teor %B0, r0" : 
  "=r"(newCrc), "+r"(byte), "+z"(table) : "r"(oldCrc));
  return newCrc;
}

# 285 "/opt/tinyos-2.x/tos/lib/serial/SerialDispatcherP.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(error_t result)
#line 285
{
  uint8_t postsignalreceive = FALSE;

  /* atomic removed: atomic calls only */
#line 287
  {
    if (!/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending && result == SUCCESS) {
        postsignalreceive = TRUE;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending = TRUE;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskType = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskWhich = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskBuf = (message_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskSize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBufferSwap();
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_IDLE;
      }
    else 
#line 297
      {

        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which);
      }
  }
  if (postsignalreceive) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__postTask();
    }
}

# 200 "/opt/tinyos-2.x/tos/chips/atm1281/HplAtm128UartP.nc"
__attribute((interrupt))   void __vector_27(void )
#line 200
{
  HplAtm128UartP__HplUart0__txDone();
}

# 92 "/opt/tinyos-2.x/tos/lib/serial/HdlcTranslateC.nc"
static error_t HdlcTranslateC__SerialFrameComm__putData(uint8_t data)
#line 92
{
  if (data == HDLC_CTLESC_BYTE || data == HDLC_FLAG_BYTE) {
      HdlcTranslateC__state.sendEscape = 1;
      HdlcTranslateC__txTemp = data ^ 0x20;
      HdlcTranslateC__m_data = HDLC_CTLESC_BYTE;
    }
  else {
      HdlcTranslateC__m_data = data;
    }
  return HdlcTranslateC__UartStream__send(&HdlcTranslateC__m_data, 1);
}

# 288 "/opt/tinyos-2.x/tos/chips/atm1281/HplAtm128UartP.nc"
__attribute((signal))   void __vector_36(void )
#line 288
{
  if ((* (volatile uint8_t *)0xC8 & (1 << 7)) != 0) {
    HplAtm128UartP__HplUart1__rxDone(* (volatile uint8_t *)0XCE);
    }
}

#line 293
__attribute((interrupt))   void __vector_38(void )
#line 293
{
  HplAtm128UartP__HplUart1__txDone();
}

# 206 "/opt/tinyos-2.x/tos/chips/atm1281/timer/HplAtm1281Timer3P.nc"
__attribute((interrupt))   void __vector_32(void )
#line 206
{
  HplAtm1281Timer3P__CompareA__fired();
}

__attribute((interrupt))   void __vector_33(void )
#line 210
{
  HplAtm1281Timer3P__CompareB__fired();
}

__attribute((interrupt))   void __vector_34(void )
#line 214
{
  HplAtm1281Timer3P__CompareC__fired();
}

__attribute((interrupt))   void __vector_31(void )
#line 218
{
  HplAtm1281Timer3P__Capture__captured(HplAtm1281Timer3P__Timer__get());
}

__attribute((interrupt))   void __vector_35(void )
#line 222
{
  HplAtm1281Timer3P__Timer__overflow();
}

