// ---------------------------------------------------------------------------- 
// DS18X20-Functions via One-Wire-Bus
//
// (c) Martin Thomas (mthomas(at)rhrk.uni-kl.de)
// Partly based on code from Peter Dannegger and others.
//
// ChangeLog:
// 20041124 - Extended measurements for DS18(S)20 contributed by Carsten Foss (CFO)
// 200502xx - function DS18X20_read_meas_single
// 20050310 - DS18x20 EEPROM functions (can be disabled to save flash-memory)
//            (DS18X20_EEPROMSUPPORT in ds18x20.h)
// 20100625 - removed inner returns, added static function for read scratchpad
//            replaced full-celcius and fractbit method with decicelsius
//            and maxres (degreeCelsius*10e-4) functions, renamed eeprom-functions,
//            delay in recall_e2 replaced by timeout-handling
// 20100714 - ow_command_skip_last_recovery used for parasite-powerd devices so the
//            strong pull-up can be enabled in time even with longer OW recovery times
// 20110209 - fix in DS18X20_format_from_maxres() by Marian Kulesza
// ---------------------------------------------------------------------------- 

#ifndef __have_ds18x20_h__
#define __have_ds18x20_h__

// ---------------------------------------------------------------------------- 

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------- 

#include <stdlib.h>
#include <stdint.h>

// ---------------------------------------------------------------------------- 

#define DS18X20_CHECKCRC          0		// Check CRC disabled(0) or enabled(1)
#define DS18X20_EEPROMSUPPORT     0		// DS18x20 EERPROM support disabled(0) or enabled(1)
#define DS18X20_DECICELSIUS       1		// decicelsius functions disabled(0) or enabled(1)
#define DS18X20_MAX_RESOLUTION    0		// max. resolution functions disabled(0) or enabled(1)
#define DS18X20_VERBOSE           0		// extended output via UART disabled(0) or enabled(1) :

// ---------------------------------------------------------------------------- 
// return values
#define DS18X20_OK                0x00
#define DS18X20_ERROR             0x01
#define DS18X20_START_FAIL        0x02
#define DS18X20_ERROR_CRC         0x03

#define DS18X20_INVALID_DECICELSIUS  2000

#define DS18X20_POWER_PARASITE    0x00
#define DS18X20_POWER_EXTERN      0x01

#define DS18X20_CONVERSION_DONE   0x00
#define DS18X20_CONVERTING        0x01

// ---------------------------------------------------------------------------- 
// DS18X20 specific values (see datasheet)
#define DS18S20_FAMILY_CODE       0x10
#define DS18B20_FAMILY_CODE       0x28
#define DS1822_FAMILY_CODE        0x22

#define DS18X20_CONVERT_T         0x44
#define DS18X20_READ              0xBE
#define DS18X20_WRITE             0x4E
#define DS18X20_EE_WRITE          0x48
#define DS18X20_EE_RECALL         0xB8
#define DS18X20_READ_POWER_SUPPLY 0xB4

#define DS18B20_CONF_REG          4
#define DS18B20_9_BIT             0
#define DS18B20_10_BIT            (1<<5)
#define DS18B20_11_BIT            (1<<6)
#define DS18B20_12_BIT            ((1<<6) | (1<<5))
#define DS18B20_RES_MASK          ((1<<6) | (1<<5))

// ---------------------------------------------------------------------------- 
// undefined bits in LSB if 18B20 != 12bit
#define DS18B20_9_BIT_UNDF        ((1<<0) | (1<<1) | (1<<2))
#define DS18B20_10_BIT_UNDF       ((1<<0) | (1<<1))
#define DS18B20_11_BIT_UNDF       ((1<<0))
#define DS18B20_12_BIT_UNDF       0

// ---------------------------------------------------------------------------- 
// conversion times in milliseconds
#define DS18B20_TCONV_12BIT       750
#define DS18B20_TCONV_11BIT       DS18B20_TCONV_12_BIT / 2
#define DS18B20_TCONV_10BIT       DS18B20_TCONV_12_BIT / 4
#define DS18B20_TCONV_9BIT        DS18B20_TCONV_12_BIT / 8
#define DS18S20_TCONV             DS18B20_TCONV_12_BIT

#define DS18X20_FRACCONV          625	// constant to convert the fraction bits to cel*(10^-4)

#define DS18X20_SP_SIZE           9	// scratchpad size in bytes

// scratchpad size in bytes
// DS18X20 EEPROM-Support
#define DS18X20_WRITE_SCRATCHPAD  0x4E
#define DS18X20_COPY_SCRATCHPAD   0x48
#define DS18X20_RECALL_E2         0xB8
#define DS18X20_COPYSP_DELAY      10 		// ms
#define DS18X20_TH_REG            2
#define DS18X20_TL_REG            3

#define DS18X20_DECIMAL_CHAR      '.'

// ---------------------------------------------------------------------------- 

extern uint8_t DS18X20_find_sensor(uint8_t *diff, uint8_t id[]);
extern uint8_t DS18X20_get_power_status(uint8_t id[]);
extern uint8_t DS18X20_start_meas(uint8_t with_external, uint8_t id[]);

// returns 1 if conversion is in progress, 0 if finished
// not available when parasite powered
extern uint8_t DS18X20_conversion_in_progress(void);

#if DS18X20_DECICELSIUS
extern uint8_t DS18X20_read_decicelsius(uint8_t id[], int16_t *decicelsius);
extern uint8_t DS18X20_read_decicelsius_single(uint8_t familycode, int16_t *decicelsius);
extern uint8_t DS18X20_format_from_decicelsius(int16_t decicelsius, char *s);
#endif


#if DS18X20_MAX_RESOLUTION
// temperature unit for max. resolution is °C * 10e-4
// examples: -250625 -> -25.0625°C, 1250000 -> 125.0000 °C
extern uint8_t DS18X20_read_maxres(uint8_t id[], int32_t *temperaturevalue);
extern uint8_t DS18X20_read_maxres_single(uint8_t familycode, int32_t *temperaturevalue);
extern uint8_t DS18X20_format_from_maxres(int32_t temperaturevalue, char s[], uint8_t n);
#endif


#if DS18X20_EEPROMSUPPORT
// write th, tl and config-register to scratchpad (config ignored on DS18S20)
uint8_t DS18X20_write_scratchpad(uint8_t id[], uint8_t th, uint8_t tl, uint8_t conf);

// read scratchpad into array SP
uint8_t DS18X20_read_scratchpad(uint8_t id[], uint8_t sp[], uint8_t n);

// copy values int scratchpad into DS18x20 eeprom
uint8_t DS18X20_scratchpad_to_eeprom(uint8_t with_power_extern, uint8_t id[]);

// copy values from DS18x20 eeprom into scratchpad
uint8_t DS18X20_eeprom_to_scratchpad(uint8_t id[]);
#endif 


#if DS18X20_VERBOSE
extern void DS18X20_show_id_uart(uint8_t *id, size_t n);
extern uint8_t DS18X20_read_meas_all_verbose(void);
#endif

// ---------------------------------------------------------------------------- 

#ifdef __cplusplus
}
#endif

// ---------------------------------------------------------------------------- 

#endif // __have_ds18x20_h__
