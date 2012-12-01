// ----------------------------------------------------------------------------- 
// ADC Driver with oversampling, persisted calibration & linearization
//
// (c) 2012 karl@pitrich.com
// inspired by AVR120, AVR121 and http://www.mikrocontroller.net/topic/170454 
// ----------------------------------------------------------------------------- 

#ifndef __have_adc_h__
#define __have_adc_h__

// ----------------------------------------------------------------------------- 

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------------- 
  
#include <stdint.h>
#include <avr/io.h>

// ----------------------------------------------------------------------------- 

typedef enum AdcValueType_e {
  AdcReadLinearized = (1<<0),
  AdcReadRaw        = (1<<1)
} AdcValueType_t;

typedef struct adcCalibration_s {
  uint32_t hi;
  uint32_t lo;
} AdcCalibration_t;

// ----------------------------------------------------------------------------- 
// calibration points at runtime

extern AdcCalibration_t adcCalibration;

// ----------------------------------------------------------------------------- 
// measured AVCC at startup in mV

extern uint16_t adcAVcc;

// ----------------------------------------------------------------------------- 

extern void     adcInit(void);
extern void     adcLoadCalibrationData(void);
extern void     adcSaveCalibrationData(void);
extern void 	adcCalculateReferenceCompensationFactor(void);
extern uint32_t adcValue(uint8_t channel, AdcValueType_t valuetype);

// ----------------------------------------------------------------------------- 

#define adcSelectChannel(channel) \
  (ADMUX = (ADMUX & (~((1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0)))) | (channel))

#define adcEnableInternalVref()       (ADMUX  |=  ((1<<REFS1) | (1<<REFS0))) // 1.1V internal Bandgap Reference
#define adcEnableExternalVref()       (ADMUX  &= ~((1<<REFS1) | (1<<REFS0))) // VRef at Pin AREF
#define adcEnableVccVref()            (ADMUX  &= ~(1<<REFS1), ADMUX |= (1<<REFS0)) // AVCC with external capacitor at AREF pin

#define adcStartConversion()          (ADCSRA |=  (1<<ADSC))
#define adcIsConversionFinished()     ((ADCSRA &  (1<<ADIF)) ? TRUE : FALSE)
#define adcIsConversionRunning()      ((ADCSRA | ~(1<<ADIF)) ? TRUE : FALSE)
#define adcWaitForConversion()        while (ADCSRA & (1<<ADSC))

#define adcEnable()                   (ADCSRA |=  (1<<ADEN))
#define adcDisable()                  (ADCSRA &= ~(1<<ADEN))

#define adcEnableInt()                (ADCSRA |=  (1<<ADIE))
#define adcDisableInt()               (ADCSRA &= ~(1<<ADIE))

#define adcClearFlag()                (ADCSRA &=  (1<<ADIF))

// ----------------------------------------------------------------------------- 
  
#ifdef __cplusplus
} // extern "C"
#endif

// ----------------------------------------------------------------------------- 

#endif // __have_adc_h__
