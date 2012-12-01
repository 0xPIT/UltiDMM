//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include "sed1520.h"
#include "fonts.h"

//-------------------------------------------------------------------------------------------------
// pixel level bit masks for display
// this array is setup to map the order of bits in a byte 
// to the vertical order of bits at the LCD controller
//
// TODO: avoid or PROGMEM
//
const unsigned char l_mask_array[8] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 };

// the LCD display image memory
// buffer arranged so page memory is sequential in RAM
unsigned char l_display_array[LCD_Y_BYTES][LCD_X_BYTES];

//-------------------------------------------------------------------------------------------------
//
void lcd_initports(void)
{
  SED1520_DATA_DDR = 0xFF;

  SED1520_CONTROL_DDR |= (SED1520_E2 | SED1520_E1 | SED1520_RW | SED1520_A0 | SED1520_RES);

  _delay_ms(10);

  SED1520_CONTROL_PORT |= SED1520_RES;
}

//-------------------------------------------------------------------------------------------------
//
void lcd_waitforstatus(unsigned char status, unsigned char controller)
{
  char tmp = 0;
  SED1520_CONTROL_PORT &= ~SED1520_A0; 
  SED1520_CONTROL_PORT |= SED1520_RW; 
  SED1520_DATA_DDR = 0x00;

  do {
    if (controller & 0x01) {
      SED1520_CONTROL_PORT |= SED1520_E1; 
      asm("nop"); asm("nop");
      tmp = SED1520_DATA_PIN; 
      SED1520_CONTROL_PORT &= ~SED1520_E1; 
    }

    if (controller & 0x02) {
      SED1520_CONTROL_PORT |= SED1520_E2; 
      asm("nop"); asm("nop"); 
      tmp = SED1520_DATA_PIN; 
      SED1520_CONTROL_PORT &= ~SED1520_E2; 
    }
  }
  while (tmp & status);

  SED1520_DATA_DDR = 0xFF; 
}

//-------------------------------------------------------------------------------------------------
//
void lcd_writecommand(unsigned char commandToWrite, unsigned char ctrl)
{
  lcd_waitforstatus(0x80, ctrl);

  SED1520_CONTROL_PORT &= ~SED1520_A0;
  SED1520_CONTROL_PORT &= ~SED1520_RW;
  SED1520_DATA_PORT = commandToWrite;

  if (ctrl & 0x01) {
    SED1520_CONTROL_PORT |= SED1520_E2;
    asm("nop"); asm("nop");
    SED1520_CONTROL_PORT &= ~SED1520_E2;
  }

  if (ctrl & 0x02) {
    SED1520_CONTROL_PORT |= SED1520_E1;
    asm("nop"); asm("nop");
    SED1520_CONTROL_PORT &= ~SED1520_E1;
  }
}

//-------------------------------------------------------------------------------------------------
//
void lcd_writedata(unsigned char dataToWrite, uint8_t ctrl)
{
  lcd_waitforstatus(0x80, 1);
  lcd_waitforstatus(0x80, 2);

  SED1520_CONTROL_PORT |= SED1520_A0; 
  SED1520_CONTROL_PORT &= ~SED1520_RW; 
  SED1520_DATA_PORT = dataToWrite; 

    if (ctrl & 0x01) {
      SED1520_CONTROL_PORT |= SED1520_E1;
      asm("nop"); asm("nop");
      SED1520_CONTROL_PORT &= ~SED1520_E1;
    }

    if (ctrl & 0x02) {
      SED1520_CONTROL_PORT |= SED1520_E2;
      asm("nop"); asm("nop");
      SED1520_CONTROL_PORT &= ~SED1520_E2;
    }
}

//-------------------------------------------------------------------------------------------------
//
void lcd_init(void)
{
  lcd_initports();

  lcd_writecommand(LCD_RESET, 1);
  lcd_writecommand(LCD_RESET, 2);

  _delay_ms(50);

  lcd_waitforstatus(0x10, 1);
  lcd_waitforstatus(0x10, 2);

  lcd_writecommand(LCD_DISP_OFF, 1);
  lcd_writecommand(LCD_DISP_OFF, 2);

  lcd_writecommand(LCD_DISP_ON, 1);
  lcd_writecommand(LCD_DISP_ON, 2);

  lcd_writecommand(LCD_SET_LINE | 0, 1);
  lcd_writecommand(LCD_SET_LINE | 0, 2);
}

//-------------------------------------------------------------------------------------------------
// fill buffer and LCD with pattern
//
void lcd_fill(const unsigned char pattern)
{ 
  unsigned char page, col;

  for (page = 0; page < LCD_Y_BYTES; page++) {
    for (col = 0; col < LCD_X_BYTES; col++) {
      l_display_array[page][col] = pattern;
    }
  }

  lcd_update_all();
}

//-------------------------------------------------------------------------------------------------
//
void lcd_erase(void)
{ 
  lcd_fill(0x00);
  lcd_update_all();
}

//-------------------------------------------------------------------------------------------------
// Updates area of the display. Writes data from "framebuffer" 
// RAM to the lcd display controller RAM.
//
// \param   top     top line of area to update.
// \param   bottom  bottom line of area to update.
//
void lcd_update(const unsigned char top, const unsigned char bottom)
{ 
  unsigned char x;
  unsigned char y;
  unsigned char yt;
  unsigned char yb;
  unsigned char *colptr;

  // setup bytes of range
  yb = bottom >> 3;
  yt = top    >> 3;

  for (y = yt; y <= yb; y++) {
    lcd_writecommand(LCD_SET_PAGE + y, 3);
    lcd_writecommand(LCD_SET_COL + 0, 3);

    colptr = &l_display_array[y][0];

    for (x = 0; x < LCD_X_BYTES; x++) { 
      if (x < LCD_X_BYTES / 2) {
        lcd_writedata(*colptr++, 1);
      }
      else {
        lcd_writedata(*colptr++, 2);
      }
    }
  }
}

//-------------------------------------------------------------------------------------------------
//
void lcd_update_all(void)
{
  lcd_update(SCRN_TOP, SCRN_BOTTOM);
}

//-------------------------------------------------------------------------------------------------
// sets/clears/switchs(XOR) dot at (x,y)
//
void lcd_dot(const unsigned char x, const unsigned char y, const unsigned char mode) 
{ 
  unsigned char bitnum, bitmask, yByte;
  unsigned char *pBuffer; /* pointer used for optimisation */

  if ((x > SCRN_RIGHT ) || (y > SCRN_BOTTOM)) {
    return;
  }

  yByte   = y >> 3; 
  bitnum  = y & 0x07;
  bitmask = l_mask_array[bitnum]; // bitmask = ( 1 << (y & 0x07) );
  pBuffer = &(l_display_array[yByte][x]);

  switch (mode) { // paint mode
    case LCD_MODE_SET:
      *pBuffer |= bitmask;
      break;

    case LCD_MODE_CLEAR:
      *pBuffer &= ~bitmask;
      break;

    case LCD_MODE_XOR:
      *pBuffer ^= bitmask;
      break;

    default: 
      break;
  }
}

//-------------------------------------------------------------------------------------------------
//
void lcd_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, const uint8_t mode)
{ 
  uint8_t length, xTmp, yTmp, i;
#ifdef WITH_ANGLED_LINES
  int16_t m;
  uint8_t y, yAlt;
#endif

  if (x1 == x2) { // vertical line
    // x1|y1 must be the upper point
    if (y1 > y2) { 
      xTmp = x1;
      yTmp = y1;
      x1 = x2;
      y1 = y2;
      x2 = xTmp;
      y2 = yTmp;
    }
    length = y2 - y1;

    for (i = 0; i <= length; i++) {
      lcd_dot(x1, y1 + i, mode);
    }
  } 
  else if (y1 == y2) { // horizontal line
    // x1|y1 must be the left point
    if (x1 > x2) {
      xTmp = x1;
      yTmp = y1;
      x1 = x2;
      y1 = y2;
      x2 = xTmp;
      y2 = yTmp;
    }

    length = x2 - x1;
    for (i = 0; i <= length; i++) {
      lcd_dot(x1 + i, y1, mode);
    }   
  } 
#ifdef WITH_ANGLED_LINES
  else {
    // x1 must be smaller than x2
    if (x1 > x2) {
      xTmp = x1;
      yTmp = y1;
      x1 = x2;
      y1 = y2;
      x2 = xTmp;
      y2 = yTmp;
    }

    if ((y2 - y1) >= (x2 - x1) || (y1 - y2) >= (x2 - x1)) { 
      // angle larger or equal 45�
      length = x2 - x1;								// not really the length :)
      m = ((y2 - y1) * 200) / length;
      yAlt = y1;
      for (i = 0; i <= length; i++) { 
        y = ((m * i) / 200) + y1;
        if ((m * i) % 200 >= 100) {
          y++;
        }
        else if ((m * i) % 200 <= -100) {
          y--;
        }

        lcd_line(x1 + i, yAlt, x1 + i, y, mode ); /* wuff wuff recurs. */

        if (length <= (y2 - y1) && y1 < y2) {
          yAlt = y + 1;
        }
        else if (length <= (y1 - y2) && y1 > y2) {
          yAlt = y-1;
        }
        else {
          yAlt = y;
        }
      }
    } 
    else { // angle smaller 45�
      // y1 must be smaller than y2
      if(y1 > y2) {
        xTmp = x1;
        yTmp = y1;
        x1 = x2;
        y1 = y2;
        x2 = xTmp;
        y2 = yTmp;
      }
      length = y2 - y1;
      m = ((x2 - x1) * 200) / length;
      yAlt = x1;
      for(i = 0; i <= length; i++) { 
        y = ((m * i) / 200) + x1;

        if ((m * i) % 200 >= 100) {
          y++;
        }
        else if((m * i) % 200 <= -100) {
          y--;
        }

        lcd_line(yAlt, y1 + i, y, y1 + i, mode); /* wuff */

        if (length <= (x2 - x1) && x1 < x2) {
          yAlt = y + 1;
        }
        else if(length <= (x1 - x2) && x1 > x2) {
          yAlt = y - 1;
        }
        else {
          yAlt = y;
        }
      }
    }
  }
#endif
}

//-------------------------------------------------------------------------------------------------
//
#ifdef WITH_CIRCLE
void lcd_circle(const uint8_t xCenter, const uint8_t yCenter, const uint8_t radius, const uint8_t mode) 
{ 
  int16_t tSwitch, y, x = 0;
  uint8_t d;

  d = yCenter - xCenter;
  y = radius;
  tSwitch = 3 - 2 * radius;

  while (x <= y) { 
    lcd_dot(xCenter + x, yCenter + y, mode);
    lcd_dot(xCenter + x, yCenter - y, mode);

    lcd_dot(xCenter - x, yCenter + y, mode);
    lcd_dot(xCenter - x, yCenter - y, mode);

    lcd_dot(yCenter + y - d, yCenter + x, mode);
    lcd_dot(yCenter + y - d, yCenter - x, mode);

    lcd_dot(yCenter - y - d, yCenter + x, mode);
    lcd_dot(yCenter - y - d, yCenter - x, mode);

    if (tSwitch < 0) {
      tSwitch += (4 * x + 6);
    }
    else {
      tSwitch += (4 * (x - y) + 10);
      y--;
    }
    x++;
  }
}
#endif

//-------------------------------------------------------------------------------------------------
//
void lcd_rect(const uint8_t x, const uint8_t y, uint8_t width, uint8_t height, const uint8_t mode) 
{
  width--;
  height--;
  lcd_line(x, y, x + width, y, mode);					// top
  lcd_line(x, y, x, y + height, mode);					// left
  lcd_line(x, y + height, x + width, y + height, mode);	// bottom
  lcd_line(x + width, y, x + width, y + height, mode);	// right
}

//-------------------------------------------------------------------------------------------------
//
void lcd_box(const uint8_t x, const uint8_t y, uint8_t width, const uint8_t height, const uint8_t mode) 
{
  if (!width) {
    return; 
  }

  width--;

  for (uint8_t i = y; i < y + height; i++) {
    lcd_line(x, i, x + width, i, mode);
  }
}


//-------------------------------------------------------------------------------------------------
// Writes a glyph("letter") to the display at location x,y
// (adapted function from the MJK-code)
//
// column    - x corrdinate of the left part of glyph          
// row       - y coordinate of the top part of glyph       
// width     - size in pixels of the width of the glyph    
// height    - size in pixels of the height of the glyph   
// glyph     - an unsigned char pointer to the glyph pixels 
// to write assumed to be of length "width"
//
void lcd_glyph(uint8_t left, uint8_t top, uint8_t width, uint8_t height, uint8_t *glyph_ptr, uint8_t store_width)
{
  uint8_t bit_pos;
  uint8_t byte_offset;
  uint8_t y_bits;
  uint8_t remaining_bits;
  uint8_t mask;
  uint8_t char_mask;
  uint8_t x;
  uint8_t *glyph_scan;
  uint8_t glyph_offset;

  bit_pos = top & 0x07; // get the bit offset into a byte
  glyph_offset = 0;     // start at left side of the glyph rasters
  char_mask = 0x80;     // initial character glyph mask

  /* '#' charwidth: 8
   0x00, 0x00, 	/  [        ]  /
   0x14, 0x00, 	/  [   * *  ]  /
   0x7E, 0x00, 	/  [ ****** ]  /
   0x28, 0x00, 	/  [  * *   ]  /
   0xFC, 0x00, 	/  [******  ]  /
   0x50, 0x00, 	/  [ * *    ]  /
   0x00, 0x00, 	/  [        ]  /
   0x00, 0x00, 	/  [        ]  /
   */

  for (x = left; x < (left + width); x++) {
    byte_offset = top >> 3;                // get the byte offset into y direction
    y_bits = height;                       // get length in y direction to write
    remaining_bits = 8 - bit_pos;          // number of bits left in byte
    mask = l_mask_array[bit_pos];          // get mask for this bit
    glyph_scan = glyph_ptr + glyph_offset; // point to base of the glyph

    // boundary checking here to account for the possibility of
    // write past the bottom of the screen.

    while ((y_bits) && (byte_offset < LCD_Y_BYTES)) { // while there are bits still to write
      // check if the character pixel is set or not
      if (pgm_read_byte(glyph_scan) & char_mask) {
        l_display_array[byte_offset][x] |= mask;  // set image pixel
      }
      else {
        l_display_array[byte_offset][x] &= ~mask; // clear the image pixel
      }

      if (l_mask_array[0] & 0x80) {
        mask >>= 1;
      }
      else {
        mask <<= 1;
      }

      y_bits--;
      remaining_bits--;
      if (remaining_bits == 0) { // just crossed over a byte boundry, reset byte counts
        remaining_bits = 8;
        byte_offset++;
        mask = l_mask_array[0];
      }

      // bump the glyph scan to next raster
      glyph_scan += store_width;
    }

    // shift over to next glyph bit
    char_mask >>= 1;
    if (char_mask == 0) { // reset for next byte in raster
      char_mask = 0x80;
      glyph_offset++;
    }
  }
}


//-------------------------------------------------------------------------------------------------
// Prints the given string at location x,y in the specified font.
// Prints each character given via calls to lcd_glyph. The entry string
// is null terminated. (adapted function from the MJK-code)
//
// left       coordinate of left start of string.
// top        coordinate of top of string.
// font       font number to use for display (see fonts.h)
// str        text string to display (null-terminated)
//
static void lcd_text_intern(uint8_t left, uint8_t top, uint8_t font, const char * str, uint8_t inprogmem)
{
  uint8_t x = left;
  uint8_t glyph;
  uint8_t width;
  uint8_t height, defaultheight;
  uint8_t store_width;
  uint8_t *glyph_ptr;
  uint8_t *width_table_ptr;
  uint8_t *glyph_table_ptr;
  uint8_t glyph_beg, glyph_end;
  uint8_t fixedwidth;

  defaultheight = pgm_read_byte ( &(fonts[font].glyph_height) );
  store_width = pgm_read_byte ( &(fonts[font].store_width) );
  width_table_ptr = (uint8_t *) pgm_read_word( &(fonts[font].width_table) );
  glyph_table_ptr = (uint8_t *) pgm_read_word( &(fonts[font].glyph_table) );
  glyph_beg  = pgm_read_byte( &(fonts[font].glyph_beg) );
  glyph_end  = pgm_read_byte( &(fonts[font].glyph_end) );
  fixedwidth = pgm_read_byte( &(fonts[font].fixed_width) );

  if (inprogmem) {
    glyph = pgm_read_byte(str);
  }
  else {
    glyph = (uint8_t)*str;
  }

  while (glyph != 0x00) {
    // check to make sure the symbol is a legal one
    // if not then just replace it with the default character
    if ((glyph < glyph_beg) || (glyph > glyph_end)) {
      glyph = pgm_read_byte(&(fonts[font].glyph_def));
    }

    // make zero based index into the font data arrays
    glyph -= glyph_beg;
    if (fixedwidth == 0) {
      width = pgm_read_byte(width_table_ptr + glyph);
    }
    else {
      width = fixedwidth;
    }

    height = defaultheight;
    glyph_ptr = glyph_table_ptr + ((unsigned int)glyph * (unsigned int)store_width * (unsigned int)height);

    /* range check / limit things here */
    if (x > SCRN_RIGHT) {
      x = SCRN_RIGHT;
    }

    if ((x + width) > SCRN_RIGHT + 1) {
      width = SCRN_RIGHT - x + 1;
    }

    if (top > SCRN_BOTTOM) {
      top = SCRN_BOTTOM;
    }

    if ((top + height) > SCRN_BOTTOM + 1) {
      height = SCRN_BOTTOM - top + 1;
    }

    lcd_glyph(x, top, width, height, glyph_ptr, store_width);  // plug symbol into buffer

    x += width; // move right for next character
    str++;      // point to next character in string

    if (inprogmem) {
      glyph = pgm_read_byte(str);
    }
    else {
      glyph = (uint8_t)*str;
    }
  }
}

//-------------------------------------------------------------------------------------------------
// draw string from RAM
//
void lcd_text(uint8_t left, uint8_t top, uint8_t font, const char *str)
{ 
  lcd_text_intern(left, top, font, str, 0);
}

//-------------------------------------------------------------------------------------------------
// draw string from PROGMEM
//
void lcd_text_p(uint8_t left, uint8_t top, uint8_t font, char const * const str)
{ 
  lcd_text_intern(left, top, font, str, 1);
}

//-------------------------------------------------------------------------------------------------
// Draws a bitmap into the Framebuffer.
// Bitmaps are converted from Windows BMP-Format to 
// C-Arrays with the fontgen-tool (see files bmp.h/bmp.c)
//
#ifdef WITH_BITMAP
void lcd_bitmap(const uint8_t left, const uint8_t top, const struct IMG_DEF *img_ptr, const uint8_t mode)
{
  uint8_t width, heigth, h, w, pattern, mask;
  uint8_t* ptable;

  width  = pgm_read_byte( &(img_ptr->width_in_pixels) );
  heigth = pgm_read_byte( &(img_ptr->height_in_pixels) );
  ptable  = (uint8_t*) pgm_read_word( &(img_ptr->char_table) ); 

  for (h = 0; h < heigth; h++) {
    mask = 0x80;
    pattern = pgm_read_byte( ptable );
    ptable++;
    for (w = 0; w < width; w++) {
      if (pattern & mask) {
        lcd_dot(w + left, h + top, mode);
      }

      mask >>= 1;
      if (mask == 0) {
        mask = 0x80;
        pattern = pgm_read_byte(ptable);
        ptable++;
      }
    }
  }
}
#endif // WITH_BITMAP

