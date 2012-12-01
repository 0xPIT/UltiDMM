// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------

#ifndef __have_fonts_h__
#define __have_fonts_h__

// ----------------------------------------------------------------------------- 

#ifdef __cplusplus
extern "C" {
#endif  

// -----------------------------------------------------------------------------

#include <avr/pgmspace.h>
#include <stdint.h>

// -----------------------------------------------------------------------------
// select desired fonts. (Simply comment out those not needed)

#define EN_SIX_DOT

#define EN_TEN_DOT
# define FONT_TEN_WITH_NUMBERS_ONLY 1

//#define EN_FIVE_DOT
//#define EN_SEVEN_DOT
//#define EN_NINE_DOT
//#define EN_FIFTEEN_DOT
//#define EN_EIGHTEEN_DOT

// -----------------------------------------------------------------------------
// define number labels for the font selections
//
typedef enum
{
#ifdef EN_FIVE_DOT
  FONT_FIVE_DOT,
#endif
  
#ifdef EN_SIX_DOT
  FONT_SIX_DOT,
#endif
  
#ifdef EN_SEVEN_DOT
  FONT_SEVEN_DOT,
#endif
  
#ifdef EN_NINE_DOT
  FONT_NINE_DOT,
#endif
  
#ifdef EN_TEN_DOT
  FONT_TEN_DOT,
#endif
  
#ifdef EN_FIFTEEN_DOT
  FONT_FIFTEEN_DOT,
#endif
  
#ifdef EN_EIGHTEEN_DOT
  FONT_EIGHTEEN_DOT,
#endif
  
  FONT_COUNT
} FONT_BASE;

// -----------------------------------------------------------------------------

struct FONT_DEF 
{ 
  const uint8_t store_width;  // glyph storage width in bytes
  const uint8_t glyph_height; // glyph height for storage
  const uint8_t *glyph_table; // font table start address in memory
  const uint8_t fixed_width;  // fixed width of glyphs. If zero, then use the width table.
  const uint8_t *width_table; // variable width table start adress
  const uint8_t glyph_beg;		// start ascii offset in table
  const uint8_t glyph_end;		// end ascii offset in table
  const uint8_t glyph_def;		// code for undefined glyph code
};

// -----------------------------------------------------------------------------

// font definition tables for the fonts
extern const struct FONT_DEF fonts[FONT_COUNT] PROGMEM;

// glyph bitmap and width tables for the fonts
#ifdef EN_FIVE_DOT
extern const uint8_t five_dot_glyph_table[] PROGMEM;
extern const uint8_t five_dot_width_table[] PROGMEM;
#endif

#ifdef EN_SIX_DOT
extern const uint8_t six_dot_glyph_table[] PROGMEM;
extern const uint8_t six_dot_width_table[] PROGMEM;
#endif

#ifdef EN_SEVEN_DOT
#define DEG_CHAR ('~' + 1)
extern const uint8_t seven_dot_glyph_table[] PROGMEM;
extern const uint8_t seven_dot_width_table[] PROGMEM;
#endif

#ifdef EN_NINE_DOT
extern const uint8_t nine_dot_glyph_table[] PROGMEM;
#endif

#ifdef EN_TEN_DOT
extern const uint8_t ten_dot_glyph_table[] PROGMEM;
#endif

#ifdef EN_FIFTEEN_DOT
extern const uint8_t fifteen_dot_glyph_table[] PROGMEM;
extern const uint8_t fifteen_dot_width_table[] PROGMEM;
#endif

#ifdef EN_EIGHTEEN_DOT
extern const uint8_t eighteen_dot_glyph_table[] PROGMEM;
extern const uint8_t eighteen_dot_width_table[] PROGMEM;
#endif

// ----------------------------------------------------------------------------- 
  
#ifdef __cplusplus
} // extern "C"
#endif

// -----------------------------------------------------------------------------

#endif // __have_fonts_h__
