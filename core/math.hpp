#pragma once



#define Q15_MUL(a,b)( ((int32_t)(a) * (b) + (1<<14) ) >> 15)
#define Q15_DIV(a,b)( ((int32_t)(a) / (b) + (1<<14) ) >> 15)
#define Q15_ADD(a,b)( a + b )
#define Q15_ADDSAT(a,b)
#define Q15_SUB(a,b)( a - b )
#define Q15_MOD(a,b)
#define Q15_SAT(a)( (a>0x7FFF)?0x7FFF : ( (a<(-0x8000))? -0x8000 : a) )
#define Q15_TO_FLOAT(a)
#define Q15_FROM_FLOAT(a, scale)

#define ELECFREQ_TO_MECHFREQ(freq, polepairs)(freq/polepairs)
#define MECHFREQ_TO_ELECFREQ(freq, polepairs)((freq*polepairs))
#define HZ_TO_RPM(hz)((hz*60))
#define RPM_TO_HZ(rpm)(rpm/60)


// SI Prefix Conversions
// Nano (10^-9)
#define UNIT_TO_MILLI(u)            ((u) * 1000)   // unit → m
#define UNIT_TO_MICRO(u)            ((u) * 1000000)
#define UNIT_TO_NANO(u)             ((u) * 1000000000)

#define MILLI_TO_UNIT(milli)        ((milli) / 1000)   // milli → unit
#define MICRO_TO_UNIT(micro)        ((micro) / 1000000)   // micro → unit
#define NANO_TO_UNIT(nano)          ((nano) / 1000000000)   // micro → unit

#define NANO_TO_MILLI(nano)         ((nano) / 1000000)

// Custom for your PWM context
#define TICKS_TO_NANOS(ticks, freq_Hz) (((ticks) * 1000000000) / (freq_Hz))
#define TICKS_TO_MICROS(ticks, freq_Hz) (((ticks) * 1000000) / (freq_Hz))
#define NANOS_TO_TICKS(ns, freq_Hz) (((ns) * (freq_Hz)) / 1000000000)
#define MICROS_TO_TICKS(us, freq_Hz) (((us) * (freq_Hz)) / 1000000)


// Generic versions with configurable min/max
#define INCREMENT_ROLL(x, min, max)   do { (x) = ((x) >= (max)) ? (min) : ((x) + 1); } while(0)
#define DECREMENT_ROLL(x, min, max)   do { (x) = ((x) <= (min)) ? (max) : ((x) - 1); } while(0)
