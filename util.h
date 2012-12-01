// ----------------------------------------------------------------------------- 
//
// ----------------------------------------------------------------------------- 

#ifndef __have_util_h__
#define __have_util_h__

// ----------------------------------------------------------------------------- 

#include <stdint.h>

// ----------------------------------------------------------------------------- 

#ifdef __cplusplus
extern "C" {
#endif
  
// ----------------------------------------------------------------------------- 
// work around for older c++ compiler warnings
//   seems to work without using avr-gcc (GCC) 4.6.2 and 4.7.2
#if 0
#ifdef __cplusplus
#include <avr/pgmspace.h> 
 
#ifdef PSTR
# undef PSTR 
#endif

#define __PROGMEM_DS __attribute__(( section(".progmem.data") )) 
#define PSTR(s) (__extension__({ const char * const __c[] __PROGMEM_DS = (s); &__c[0]; }))  
#endif
#endif

// ----------------------------------------------------------------------------- 

#define TRUE  1
#define FALSE 0

// ----------------------------------------------------------------------------- 

#define atomicAssign(dst, src) { cli(); dst = src; sei(); }

#define clampValue(val, lo, hi) if (val > hi) val = hi; if (val < lo) val = lo;

#define minValue(a, b) ((a < b) ? a : b)
#define maxValue(a, b) ((a > b) ? a : b)

// ----------------------------------------------------------------------------- 

typedef union {
  struct {
    uint8_t b0:1;
    uint8_t b1:1;
    uint8_t b2:1;
    uint8_t b3:1;
    uint8_t b4:1;
    uint8_t b5:1;
    uint8_t b6:1;
    uint8_t b7:1;
  };
  uint8_t value;
} __attribute__((packed)) Byte_t;

// ----------------------------------------------------------------------------- 

// fixed-point to ascii
extern void fptoa(uint32_t n, int8_t decimals, int8_t fractions, char *result);

// simple itoa for base10 only, saves 98 bytes compared to itoa(i, a, 10)
extern void itoa10(int32_t n, char *result);

// count digits in integer
uint8_t countDigits(uint32_t n);

// ----------------------------------------------------------------------------- 

#define HEX__(n) 0x##n##LU

#define B8__(x) ((x & 0x0000000FLU) ? 1 : 0) \
+((x & 0x000000F0LU) ?   2 : 0) \
+((x & 0x00000F00LU) ?   4 : 0) \
+((x & 0x0000F000LU) ?   8 : 0) \
+((x & 0x000F0000LU) ?  16 : 0) \
+((x & 0x00F00000LU) ?  32 : 0) \
+((x & 0x0F000000LU) ?  64 : 0) \
+((x & 0xF0000000LU) ? 128 : 0)

#define B8 (d) ((uint8_t)B8__(HEX__(d)))
#define B16(dmsb, dlsb) (((uint16_t)B8(dmsb)<< + B8(dlsb))
#define B32(dmsb, db2, db3, dlsb) (((uint32_t)B8(dmsb)<<24) + ((uint32_t)B8(db2)<<16) + ((uint32_t)B8(db3)<< + B8(dlsb))

/* Sample usage:
  B8(01010101) = 55
  B16(10101010,01010101) = 43605
  B32(10000000,11111111,10101010,01010101) = 2164238933
*/

// ----------------------------------------------------------------------------- 

#ifdef __cplusplus
} // extern "C"
#endif

// ----------------------------------------------------------------------------- 

#endif //__have_util_h__


/* 
  Sample usage for byte structs:

struct st_io {                              
 union {                                
       unsigned char BYTE;   
       struct {    
              unsigned char B0:1;
              unsigned char B1:1; 
              unsigned char B2:1;
              unsigned char B3:1; 
              unsigned char B4:1; 
              unsigned char B5:1; 
              unsigned char B6:1; 
              unsigned char :1;   
              }      BIT; 
       }            PRD;
};     

#define IO      (*(volatile struct st_io    *)0x32) // 0x32 is SFRs address

with empty bits:
typedef union
{
    struct
    {
        unsigned MSEL :15; // 0..14
        unsigned NSEL :8;  // 15..23
        unsigned :1;
        unsigned PLLE :1;  // 24
        unsigned PLLC :1;  // 25
        unsigned PLOCK :1; // 26
        unsigned :6;       // 27..31
    } bits;
    unsigned int reg;
} __attribute__((packed)) PLLStatus_t;

#define PLL_STATUS (*((PLL_Status_t volatile *) 0x12345678))
*/

