// ----------------------------------------------------------------------------- 
//
// ----------------------------------------------------------------------------- 

#include <avr/pgmspace.h>
#include "util.h"

// ----------------------------------------------------------------------------- 
//
uint8_t countDigits(uint32_t n)
{
  uint8_t d = 1;
  switch (n) {
    case  100000000 ... 999999999:  d++;
    case   10000000 ... 99999999:   d++;
    case    1000000 ... 9999999:    d++;
    case     100000 ... 999999:     d++;
    case      10000 ... 99999:      d++;
    case       1000 ... 9999:       d++;
    case        100 ... 999:        d++;
    case         10 ... 99:         d++;
  }
  return d;
}

// -----------------------------------------------------------------------------
// Simple itoa, for a base10 numbers only. (Saves ~98 bytes)
// http://www.mikrocontroller.net/articles/FAQ#Eigene_Umwandlungsfunktionen
//
void itoa10(int32_t n, char *result)
{
  uint32_t u;
  uint16_t i = 0;

  if (n < 0) { // for negative number, prepend '-' and invert
    result[0] = '-';
    result++;    
    u = ((uint32_t) -(n + 1)) + 1;
  }
  else { 
    u = (uint32_t)n;
  }
  
  do {
    result[i++] = '0' + u % 10;
    u /= 10;
  } 
  while (u > 0);
  
  // rotate string bytewise
  for (uint16_t j = 0; j < i / 2; ++j) {
    char tmp = result[j];
    result[j] = result[i - j - 1];
    result[i - j - 1] = tmp;
  }
  result[i] = '\0';
}

// ----------------------------------------------------------------------------- 
//
void fptoa(uint32_t n, int8_t decimals, int8_t fractions, char *result)
{
  int8_t i = 0;
  uint8_t digits = countDigits(n);
  int8_t decimalsrequested = decimals;
  
  if (digits < decimals + fractions) { // clamp decimals
    decimals = digits - fractions;
    if (decimals < 0) decimals = 0;
  }

  if (decimals < decimalsrequested) { // right align & leading zero
    uint8_t indent = decimalsrequested - decimals;
    for (i = 0; i < indent; i++) {
      *result++ = (decimals < 1 && i == indent - 1) ? '0' : ' ';
    }
  }
  
  uint8_t s[digits + 1]; // convert
  for (i = digits - 1; i >= 0; i--) {
    s[i] = '0' + n % 10;
    n /= 10;
  }

  uint8_t outputcount = minValue(digits, decimals + fractions);
  for (i = 0; i < outputcount; i++) { // output
    if (i == decimals) { // insert dot (leading '0' has already been added)
      *result++ = '.';
    }
    if (s[i] >= '0') {
      *result++ = s[i];
    }
  }

  if (digits < fractions) { // fill if too few digits
    for (i = 0; i < fractions - digits; i++) {
      *result++ = ' ';
    }
  }

  *result = 0;
}

// -----------------------------------------------------------------------------

#if 0

void outhex(uint8_t val)
{
  uint8_t code HEX[] = "0123456789ABCDEF";
  uint8_t bl = HEX[val & 0x0F];
  uint8_t bh = HEX[val >> 4];
  putchar( bh );
  putchar( bl );
}

void outint(int val)
{
  uint code TEST[] = { 10, 100, 1000, 10000 };
  uchar d, i;
  bit zero;
  uint uval = val;
  
  if (val < 0) {
    uval = -val;
    putchar('-');
  }
  
  zero = 1;
  i = 4;
  do {
    i--;
    for (d = '0'; uval >= TEST[i]; uval -= TEST[i]) {
      d++;
      zero = 0;
    }
    if (zero == 0) {
      putchar(d);
    }
  } while(i);
    
  putchar((uchar)uval + '0');
}
#endif


#if 0
// ----------------------------------------------------------------------------- 
//
// (c) 2006 Thorsten de Buhr <thorsten@lasertechnix.de>
// Use is free, could run or not  ;-)
// 
// Supports: %i, %s, %c \n, \t
//
// uses #define mprintfOut(c) fputc(c, 0); for actual output
//
int mprintf(char *string, ...)
{
  va_list tag;
  int i = 0;
  int *pointer = 0;
  signed int int_tmp = 0;
    
  va_start(tag, string);
  
  do {
    if (*string == '%') {
      switch (*(++string)) {
        case 'i': {
          int_tmp = va_arg(tag, int);
          
          if(int_tmp < 0) {
            mprintfOut('-');
            int_tmp=-int_tmp;
          }
 
          pointer = (int *)int_tmp;
          for (i = MAX_NUM_DEC; i; i /= 10) {            
            if((int)pointer / i) {
              mprintfOut((int_tmp / i) + 48);
              int_tmp %= i;
            }
          }
        } break;
          
        case 'c': {
          mprintfOut((char)va_arg(tag, int));
        } break;
          
        case 's': {
          pointer = (int *)va_arg(tag, char *);
          
          do {
            mprintfOut(*pointer);
          } while(*pointer++);
        } break;
      }
    } // end if '%'
    
    else if(*string == '\\') {
      switch(*(++string)) {
        case 'n': mprintfOut('\n');
          break;
          
        case 't': mprintfOut('\t');
          break;
      }
    } // end if '\'
    
    else mprintfOut(*string);
    
  } while(*(++string));
  
  va_end(tag);
  
  return 0;
}
#endif
