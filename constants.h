// ----------------------------------------------------------------------------- 
//
// ----------------------------------------------------------------------------- 

#ifndef __have_constants_h__
#define __have_constants_h__

// ----------------------------------------------------------------------------- 

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------------- 

#include <stdint.h>

// ----------------------------------------------------------------------------- 

extern const char * const DisplayLayout[];
extern const uint8_t DisplayLayoutCount;

// ----------------------------------------------------------------------------- 

typedef enum DisplayUnit_e {
  DisplayVolt    = 0,
  DisplaymVolt   = 1,
  DisplayAmpere  = 2,
  DisplaymAmpere = 3,
} DisplayUnit_t;

extern const char * const DisplayUnit[];
extern const uint8_t DisplayUnitCount;

// ----------------------------------------------------------------------------- 

#ifdef __cplusplus
} // extern "C"
#endif

// ----------------------------------------------------------------------------- 

#endif // __have_constants_h__
