// ----------------------------------------------------------------------------- 
//
// ----------------------------------------------------------------------------- 

#include <avr/pgmspace.h> 
#include "constants.h"

// ----------------------------------------------------------------------------- 

static const char decimalLayout1[] PROGMEM = "8.888";
static const char decimalLayout2[] PROGMEM = "88.88";
static const char decimalLayout3[] PROGMEM = "888.8";

const char * const DisplayLayout[] PROGMEM = {
  decimalLayout1,
  decimalLayout2,
  decimalLayout3
};

const uint8_t DisplayLayoutCount = 3;

// ----------------------------------------------------------------------------- 

//const char unit_kV[] 	PROGMEM = "kV";
static const char unit_V[]  	PROGMEM = "V ";
static const char unit_mV[] 	PROGMEM = "mV";
//const char unit_uV[] 	PROGMEM = "µV";

//const char unit_kA[] 	PROGMEM = "kA";
static const char unit_A[]  	PROGMEM = "A ";
static const char unit_mA[] 	PROGMEM = "mA";
//const char unit_uA[] 	PROGMEM = "µA";

/*
const char unit_Deg[] 	PROGMEM = "@";
const char unit_DegC[] 	PROGMEM = "@C";
const char unit_DegF[] 	PROGMEM = "@F";

const char unit_F[] 	PROGMEM = "F";
const char unit_uF[] 	PROGMEM = "µF";
const char unit_nF[] 	PROGMEM = "nF";

const char unit_H[] 	PROGMEM = "H";
const char unit_mH[] 	PROGMEM = "mH";
const char unit_uH[] 	PROGMEM = "µH";

const char unit_km[] 	PROGMEM = "km";
const char unit_m[] 	PROGMEM = "m";
const char unit_cm[]	PROGMEM = "cm";
const char unit_mm[] 	PROGMEM = "mm";

const char unit_Hz[] 	PROGMEM = "Hz";
const char unit_kHz[] 	PROGMEM = "kHz";
const char unit_MHz[] 	PROGMEM = "MHz";
const char unit_GHz[] 	PROGMEM = "GHz";
*/

const char * const DisplayUnit[] PROGMEM = {
	unit_V,
	unit_mV,
	unit_A,
	unit_mA
};

const uint8_t DisplayUnitCount = 4;

// ----------------------------------------------------------------------------- 
