// ----------------------------------------------------------------------------- 
// UltiDMM
// Configurable Panel Meter
//
// Copyright (c) 2012, 2014 karl@pitrich.com
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  1. Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//  2. Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
//  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
//  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
//  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
//  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
//  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// -----------------------------------------------------------------------------

/*
TODO
- Zero-Offset for both channels
*/  

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

// ----------------------------------------------------------------------------- 

#include "constants.h"
#include "util.h"
#include "sed1520.h"
#include "fonts.h"
#include "adc.h"
#include "encoder.h"
#include "menu.h"
#include "crc8.h"

#ifdef WITH_OWSENSORS
# include "onewire.h"
# include "ds18x20.h"
#endif

// ----------------------------------------------------------------------------- 

#ifdef WITH_OWSENSORS
# define MAXSENSORS 2
  uint8_t owScanBus(void);
  uint8_t temperatureSensorID[MAXSENSORS][OW_ROMCODE_SIZE];
  int16_t measuredTemperatures[MAXSENSORS];  
  volatile bool triggerOneWireRead = true;
#endif // WITH_OWSENSORS

// ----------------------------------------------------------------------------- 

#define GPIO_IN   PINC
#define GPIO_OUT  PORTC
#define GPIO_DDR  DDRC
#define GPIO_LIMIT_U_PIN  PC4
#define GPIO_LIMIT_I_PIN  PC3
#define GPIO_DISABLE_PIN  PC2

// ----------------------------------------------------------------------------- 

void displayDelay(uint8_t t);

#define DEFAULT_FONT    FONT_SIX_DOT

#define lcdDEBUG(text) \
  lcd_text_p(1, 1, DEFAULT_FONT, PSTR(text)); displayDelay(5);

// ----------------------------------------------------------------------------- 

int32_t tmp = 0;
char buffer[30];
uint16_t tick = 0;
volatile bool triggerDisplayRefresh = false;
bool isDirty;

Menu::Engine Engine;
RotaryEncoder Encoder;
int8_t  encMovement = 0;
int32_t encAbsoluteValue = 0;

// ----------------------------------------------------------------------------- 
// System Settings
//

typedef struct DisplaySettings_s {
  uint8_t  unit;
  uint8_t  layout;
  uint16_t scale;     // scale % (normal = 100)
  int32_t  limit;     // when to display 'limit' - fixed-point
} DisplaySettings_t;

typedef struct SystemSettings_s {
  DisplaySettings_t Channel[2];
  uint8_t checksum; // always last member
} SystemSettings_t;

namespace Settings {
  enum Channel_e { // used as array index
    ChannelVoltage  = 0,
    ChannelCurrent  = 1,
    ChannelMax
  };

  SystemSettings_t Persisted EEMEM = {
    {
      { DisplayVolt,    1, 100, 350 },
      { DisplaymAmpere, 3, 100, 300 }
    },
    0
  };

  SystemSettings_t Active;
};

// ----------------------------------------------------------------------------- 

void saveSettings(uint8_t block = false) 
{
  //lcd_box(0, 0, 122, 10, LCD_MODE_CLR);
  //lcd_text_p(1, 1, DEFAULT_FONT, PSTR("Saving changes...")); displayDelay(5);

  Settings::Active.checksum = crc8((uint8_t *)&Settings::Active, sizeof(Settings::Active) - sizeof(uint8_t));

  if (block) cli();
  do {} while (!(eeprom_is_ready()));
  eeprom_write_block(&Settings::Active, &Settings::Persisted, sizeof(Settings::Active));
  if (block) sei();
}

// ----------------------------------------------------------------------------- 

void loadSettings(uint8_t block = false)
{
  bool applydefaults = false;

  if (block) cli();
  do {} while (!(eeprom_is_ready()));
  eeprom_read_block(&Settings::Active, &Settings::Persisted, sizeof(Settings::Active));

  uint8_t checksum = crc8((uint8_t *)&Settings::Active, sizeof(Settings::Active) - sizeof(uint8_t));
  if (Settings::Active.checksum != checksum) {
    applydefaults = true;
  }

  if (applydefaults) {
    lcd_text_p(1, 1, DEFAULT_FONT, PSTR("Applying Default Settings...")); displayDelay(20);
    Settings::Active.Channel[Settings::ChannelVoltage] = { DisplayVolt,    1, 100, 350 };
    Settings::Active.Channel[Settings::ChannelCurrent] = { DisplaymAmpere, 1, 100, 300 };
    saveSettings(block);
    lcd_box(0, 0, 122, 10, LCD_MODE_CLR);
  }

  if (block) sei();
}

// ----------------------------------------------------------------------------- 
// set up ticker at timer0 to run every 1ms
//
void initTimer0(void)
{
  TCCR0A |= (1<<WGM01);            // Mode: CTC
  TCCR0B |= (1<<CS01) | (1<<CS00); // prescale 64   // 1024: (1<<CS02) | (1<<CS00)
  TIMSK0 |= (1<<OCIE0A);
  
  // OCRn = (clock / prescaler * desired_time_in_seconds) - 1
  // OCR0A = F_CPU / 64 * 0.001 - 1;
  OCR0A = 124;
}

// ----------------------------------------------------------------------------- 
//
ISR(TIMER0_COMPA_vect)
{  
  if (tick % 64 == 0) {
    triggerDisplayRefresh = true;
  }

#ifdef WITH_OWSENSORS
  if (tick % 512 == 0) {
    triggerOneWireRead = true;
  }
#endif

  Encoder.service();

  tick++;
}

// ----------------------------------------------------------------------------- 
//
namespace State {  
  typedef enum SystemMode_e {
    None      = 0,
    Measure   = (1<<0),
    Settings  = (1<<1),
    Edit      = (1<<2),
    Limiting  = (1<<3),
    Standby   = (1<<4)
  } SystemMode;

  typedef enum EditMode_e {
    EditModeNone   = 0,
    EditModeLayout = (1<<0),
    EditModeUnit   = (1<<1),
    EditModeScale  = (1<<2),
    EditModeCal    = (1<<3)
  } EditMode; 
};

uint8_t systemState = State::Measure;
uint8_t editState   = State::EditModeNone;

// ----------------------------------------------------------------------------- 
//
bool menuExit(const Menu::Action_t action)
{
  // clear lcd at menu area (update done in main loop)
  lcd_box(0, 0, 122, 10, LCD_MODE_CLR);
  
  // reset encoder to use acceleration
  Encoder.setAccelerationEnabled(true); 

  // reset system status
  systemState = State::Measure;
  editState = State::EditModeNone;
  
  Engine.currentItem = &Menu::NullItem;

  return true;
}

// ----------------------------------------------------------------------------- 
//
bool menuRenderLabel(const Menu::Action_t action)
{
  //if (action == menuActionLabel) {
    lcd_box(0, 0, 122, 10, LCD_MODE_CLR);
    lcd_text(1, 1, DEFAULT_FONT, Engine.getLabel(Engine.currentItem));
  //}

  return true;
}

// ----------------------------------------------------------------------------- 
//  ADC calibration
//
bool menuActionCalibrate(const Menu::Action_t action) 
{
  extern const Menu::Item_t miCalibrateLo0, miCalibrateLo1, miCalibrateHi1;
  
  uint8_t channel = Engine.currentItem == &miCalibrateLo1 || Engine.currentItem == &miCalibrateHi1;

  uint32_t *adcCalibrationValue =
    (Engine.currentItem == &miCalibrateLo0 || Engine.currentItem == &miCalibrateLo1)
    ? &adcCalibration[channel].lo
    : &adcCalibration[channel].hi;
  
  if (action == Menu::actionDisplay) {
    if (editState != State::EditModeNone) {
      if (encMovement) {
        atomicAssign(*adcCalibrationValue, (*adcCalibrationValue + encMovement));
      }
    }

    // display calibration value
    itoa10(*adcCalibrationValue, buffer);
    lcd_box( 50, 1, 50, 9, LCD_MODE_CLR);
    lcd_text(50, 1, DEFAULT_FONT, buffer);

    // edit cursor
    if (editState == State::EditModeCal) {
      lcd_box(49, 0, 21, 8, LCD_MODE_XOR);
    }
    lcd_rect(49, 0, 21, 8, (editState == State::EditModeCal) ? LCD_MODE_SET : LCD_MODE_CLR);

    // display current raw adc value
    tmp = adcValue(channel, AdcReadRaw);
    itoa10(tmp, buffer);
    lcd_box( 100, 1, 50, 9, LCD_MODE_CLR);
    lcd_text(100, 1, DEFAULT_FONT, buffer);
  }

  if (action == Menu::actionTrigger) {
    if (editState == State::EditModeNone) { // enter edit mode
      systemState = State::Edit;            // prevent encoder to change menu
      editState = State::EditModeCal; 
    }
  }

  if (action == Menu::actionParent) { // navigating to self->parent
    if (editState != State::EditModeNone) { // leave edit mode, stay on menu item
      adcSaveCalibrationData();
      editState = State::EditModeNone;
      systemState = State::Settings; // release encoder
      return false;
    }
  }

  if (action == Menu::actionLabel) {
    menuRenderLabel(action);
  }

  return true;
}

// ----------------------------------------------------------------------------- 
// display layout & unit
//   saves parameters on exit
//
bool menuActionView(const Menu::Action_t action)
{  
  static int8_t menuValue = 0;

  extern const Menu::Item_t miChannelView0;
  uint8_t channel = (Engine.currentItem == &miChannelView0) ? Settings::ChannelVoltage : Settings::ChannelCurrent;  
  DisplaySettings_t *C = &Settings::Active.Channel[channel];

  // ---------------------------------------------------------------

  if (action == Menu::actionDisplay) {
    if (editState != State::EditModeNone) {
      menuValue += encMovement;
    }

    if (editState == State::EditModeLayout) {
      clampValue(menuValue, 0, DisplayLayoutCount - 1);
      isDirty = true;
      C->layout = menuValue;
    }

    if (editState == State::EditModeUnit) {
      clampValue(menuValue, 0, DisplayUnitCount - 1);
      isDirty = true;
      C->unit = menuValue;
    }
    
    // clear value display
    lcd_box(49, 0, 122, 10, LCD_MODE_CLR);

    // layout
    lcd_text_p(50, 1, DEFAULT_FONT, (char const * const)pgm_read_word(&DisplayLayout[C->layout]));

    // unit
    lcd_text_p(81, 1, DEFAULT_FONT, (char const * const)pgm_read_word(&DisplayUnit[C->unit]));

    // cursor
    if (editState == State::EditModeLayout) {
      lcd_box(49, 0, 23, 8, LCD_MODE_XOR);
    }
    else if (editState == State::EditModeUnit) {
      lcd_box(80, 0, 13, 8, LCD_MODE_XOR);
    }
  }

  // ---------------------------------------------------------------

  if (action == Menu::actionTrigger) { // enter edit mode, then switch between unit and layout
    if (editState == State::EditModeNone) { // enter edit mode
      isDirty = false;
      systemState = State::Edit;            // prevent encoder to change menu
      editState = State::EditModeUnit;      // force swap below
    }

    if (editState == State::EditModeLayout) {
      editState = State::EditModeUnit;
      menuValue = C->unit;
    }
    else if (editState == State::EditModeUnit) {
      editState = State::EditModeLayout;
      menuValue = C->layout;
    }
  }

  // ---------------------------------------------------------------

  if (action == Menu::actionParent) { // navigating to self->parent
    if (editState != State::EditModeNone) { // leave edit mode, stay on menu item
      editState = State::EditModeNone;
      systemState = State::Settings; // release encoder
      if (isDirty) {
        saveSettings(true);
        isDirty = false;
      }
      return false;
    }
  }

  // ---------------------------------------------------------------

  if (action == Menu::actionLabel) {
    menuRenderLabel(action);  
  }

  return true;
}

// ----------------------------------------------------------------------------- 
// scale factor
//
bool menuActionScale(const Menu::Action_t action)
{  
  extern const Menu::Item_t miChScale0;
  uint8_t channel = (Engine.currentItem == &miChScale0) ? Settings::ChannelVoltage : Settings::ChannelCurrent;  
  DisplaySettings_t *C = &Settings::Active.Channel[channel];

  // ---------------------------------------------------------------

  if (action == Menu::actionDisplay) {
    if (editState == State::EditModeScale) {
      clampValue(encAbsoluteValue, 10, 500); // allow 10 - 500% scale factor
      if (encAbsoluteValue != C->scale) {
        C->scale = encAbsoluteValue;
        isDirty = true;
      }
    }

    lcd_box(49, 0, 122, 10, LCD_MODE_CLR);

    itoa10(C->scale, buffer);
    lcd_text(50, 1, DEFAULT_FONT, buffer);  
    // TODO: add '%'

    if (editState == State::EditModeScale) {
      lcd_box(49, 0, 30, 8, LCD_MODE_XOR);  // invert dirty value
    }
  }

  // ---------------------------------------------------------------

  if (action == Menu::actionTrigger) {
    if (editState != State::EditModeScale) { // enter edit mode
      systemState = State::Edit;             // prevent encoder to change menu
      editState = State::EditModeScale;
      isDirty = false;
      encAbsoluteValue = C->scale;
      Encoder.setAccelerationEnabled(true);
    }
  }

  // ---------------------------------------------------------------

  if (action == Menu::actionParent) {
    if (editState == State::EditModeScale) { // leave edit mode, stay on menu item
      editState = State::EditModeNone;
      systemState = State::Settings; // release encoder
      Encoder.setAccelerationEnabled(false);

      if (isDirty) {
        saveSettings(true);
        isDirty = false;
      }

      return false;
    }
  }

  // ---------------------------------------------------------------

  if (action == Menu::actionLabel) {
    menuRenderLabel(action);  
  }

  return true;
}


// ----------------------------------------------------------------------------- 
// menuItem Arguments: Name, Label, Next, Previous, Parent, Child, Callback
//

MenuItem(miExit, "", Menu::NullItem, Menu::NullItem, Menu::NullItem, Menu::NullItem, menuExit);

MenuItem(miSettings, "Settings >", Menu::NullItem, Menu::NullItem, miExit, miChannel0, menuRenderLabel);

  MenuItem(miChannel0, "Channel 0 >", miChannel1, Menu::NullItem, miSettings, miCalibrateLo0, menuRenderLabel);
    MenuItem(miCalibrateLo0, "Calibrate Lo", miCalibrateHi0, Menu::NullItem, miChannel0, Menu::NullItem, menuActionCalibrate);
    MenuItem(miCalibrateHi0, "Calibrate Hi", miChannelView0, miCalibrateLo0, miChannel0, Menu::NullItem, menuActionCalibrate);
    MenuItem(miChannelView0, "View",         miChScale0,     miCalibrateHi0, miChannel0, Menu::NullItem, menuActionView);
    MenuItem(miChScale0,     "Scale",        Menu::NullItem, miChannelView0, miChannel0, Menu::NullItem, menuActionScale);

  MenuItem(miChannel1, "Channel 1 >", miTempShutdown, miChannel0, miSettings, miCalibrateLo1, menuRenderLabel);
    MenuItem(miCalibrateLo1, "Calibrate Lo", miCalibrateHi1,  Menu::NullItem, miChannel1, Menu::NullItem, menuActionCalibrate);
    MenuItem(miCalibrateHi1, "Calibrate Hi", miChannelView1,  miCalibrateLo1, miChannel1, Menu::NullItem, menuActionCalibrate);
    MenuItem(miChannelView1, "View",         miChScale1,      miCalibrateHi1, miChannel1, Menu::NullItem, menuActionView);
    MenuItem(miChScale1,     "Scale",        Menu::NullItem,  miChannelView1, miChannel1, Menu::NullItem, menuActionScale);

  // not yet implemented
  MenuItem(miTempShutdown, "@C Shutdown", miTempFanStart, miChannel1,     miSettings, Menu::NullItem, menuRenderLabel);
  MenuItem(miTempFanStart, "@C FanStart", Menu::NullItem, miTempShutdown, miSettings, Menu::NullItem, menuRenderLabel);

// ---------------------------------------------------------------------------- 
//
int __attribute__((naked)) main(void)
{
  uint32_t V = 0;
  uint32_t C = 0;
  uint32_t smoothV = 0;
  uint32_t smoothC = 0;
  DisplaySettings_t *S;

  lcd_init();
  lcd_erase();

#ifdef WITH_OWSENSORS
  uint8_t temperatureSensorCount = owScanBus();
#endif // WITH_OWSENSORS

  Encoder.init();
  adcInit();
  initTimer0();

  // configure IO ports for hw-limiting
  GPIO_DDR &= ~(1 << GPIO_LIMIT_U_PIN) | ~(1 << GPIO_LIMIT_I_PIN);
  GPIO_DDR |= (1 << GPIO_DISABLE_PIN);
  GPIO_OUT |= (1 << GPIO_LIMIT_U_PIN) |  (1 << GPIO_LIMIT_I_PIN);

  // enable interrupts
  sei();
	
  // reset menu
  menuExit(Menu::actionDisplay);

  // load eeprom settings
  loadSettings();

  lcd_erase();

  // ---------------------------------------------------------------
  //
  while (1) {

    // -------------------------------------------------------------
    // handle encoder movement
    //
    encMovement = Encoder.getStep();
    if (encMovement) {
      encAbsoluteValue += encMovement;

      #if 0 // display encoder value in bottom right corner
      itoa10(encAbsoluteValue, buffer);
      lcd_box(110, 26, 121, 32, LCD_MODE_CLR);
      lcd_text(110, 26, DEFAULT_FONT, buffer);
      #endif

      if (systemState == State::Settings) { // navigate only while in settings menu
        Engine.navigate((encMovement > 0) ? Engine.getNext() : Engine.getPrev());
      }
    }

    // -------------------------------------------------------------
    // handle encoder button
    //
    switch (Encoder.getButton()) {
      case State::Clicked:
        if (systemState != State::Measure) {
          Engine.invoke();
        }
        break;

      case State::DoubleClicked:
        if (systemState == State::Measure) {
          //
          // Standby functionality
          //
          if (systemState != State::Standby) {
            GPIO_OUT |= (1<<GPIO_DISABLE_PIN);
            systemState = State::Standby;
            lcd_box(0, 0, 122, 10, LCD_MODE_CLR);
            // TODO: Center text
            lcd_text_p(1, 1, DEFAULT_FONT, PSTR("- Standby Mode -"));    
          }
          else {
            GPIO_OUT &= ~(1<<GPIO_DISABLE_PIN);
            lcd_box(0, 0, 122, 10, LCD_MODE_CLR);
            systemState = State::Measure;
          }         
        }
        else { 
          Engine.navigate(Engine.getParent());
        }
        break;

      case State::Held:
        if (systemState == State::Measure) { 
          systemState = State::Settings;
          Encoder.setAccelerationEnabled(false);
          Engine.navigate(&miSettings);
        }
        break;

      case State::Released:
        break;

      default:
        break;
    }

    // -------------------------------------------------------------
    // refresh display of currently active menu
    //
    if (Engine.currentItem != &Menu::NullItem) {
      Engine.executeCallbackAction(Menu::actionDisplay);      
    }

// -------------------------------------------------------------
// main screen (always visible)
//

    // -------------------------------------------------------------
    // smoothing: IIR Filter
    //
    #define IIR_SHIFT 2
    V = ((V << IIR_SHIFT) - V + adcValue(Settings::ChannelVoltage, AdcReadLinearized)) >> IIR_SHIFT;
    C = ((C << IIR_SHIFT) - C + adcValue(Settings::ChannelCurrent, AdcReadLinearized)) >> IIR_SHIFT;

    // -------------------------------------------------------------
    // smoothing: exponential moving average with window
    //
    // weighting factor W = 0 .. 1
    // current average = (current sensor value * W) + (last average * (1 - W))
    // avg = val * 0.1 + avg * (1 - 0.1);
    //
    #define EXP_SHIFT   5
    #define EXP_WEIGHT 15
    #define EXP_SCALE   4
    #define EXP_WINDOW (2 ^ EXP_SHIFT)
    #define EXP_RC     (2 ^ EXP_SHIFT / 2) // Rounding correction: add 0,5 == 2 ^ SHIFT / 2

    smoothV = (V << (EXP_SHIFT - EXP_SCALE)) + ((smoothV * EXP_WEIGHT) >> EXP_SCALE);
    smoothC = (C << (EXP_SHIFT - EXP_SCALE)) + ((smoothC * EXP_WEIGHT) >> EXP_SCALE);

    uint32_t sv = (smoothV + EXP_RC) >> EXP_SHIFT;
    if (abs(sv - V) < EXP_WINDOW) {
      V = sv;
    }

    uint32_t sc = (smoothC + EXP_RC) >> EXP_SHIFT;
    if (abs(sc - C) < EXP_WINDOW) {
      C = sc;
    }

    // -------------------------------------------------------------
    // display VOLTAGE
    //
    if (triggerDisplayRefresh) {
      S = &Settings::Active.Channel[Settings::ChannelVoltage];
      fptoa(V, S->layout + 1, 4 - S->layout, buffer);
      buffer[5] = 0; // strip last digit
      lcd_text(1, 10, FONT_TEN_DOT, buffer);
      lcd_text_p(9 * 5 + 1, 14, FONT_SIX_DOT, (const char *)pgm_read_word(&DisplayUnit[S->unit]));

      // hw limit pin / display CV mode
      bool Vlimit = GPIO_IN & (1<<GPIO_LIMIT_U_PIN) || systemState == State::Standby;
      lcd_box(9 * 5 + 1, 10, 10, 2, (Vlimit) ? LCD_MODE_SET : LCD_MODE_CLR);
    }

    // -------------------------------------------------------------
    // display CURRENT
    //
    if (triggerDisplayRefresh) {
      S = &Settings::Active.Channel[Settings::ChannelCurrent];
      fptoa(C, S->layout + 1, 4 - S->layout, buffer);
      buffer[5] = 0; // strip last digit
      lcd_text(  1 + 9 * 5 + 1 + 16, 10, FONT_TEN_DOT, buffer);
      lcd_text_p(1 + 9 * 5 + 1 + 16 + 9 * 5 + 1, 14, FONT_SIX_DOT, (const char *)pgm_read_word(&DisplayUnit[S->unit]));

      // hw limit pin / display CC mode
      bool Climit = GPIO_IN & (1<<GPIO_LIMIT_I_PIN) || systemState == State::Standby;
      lcd_box(1 + 9 * 5 + 1 + 16 + 9 * 5 + 1, 10, 10, 2, (Climit) ? LCD_MODE_SET : LCD_MODE_CLR);
    }
    
    // -------------------------------------------------------------
    // display POWER
    //
    uint32_t P = V * C / 100000; // TODO: calculate scale factor depending on layout + unit
    fptoa(P, 3, 2, buffer);
    buffer[5] = 'w';
    buffer[6] = ' ';
    buffer[7] = 0;
    lcd_box( 76, 26, 30, 6, LCD_MODE_CLR);
    lcd_text(76, 26, FONT_SIX_DOT, buffer);

#ifdef WITH_OWSENSORS
    // -------------------------------------------------------------
    // display TEMPERATURE
    //
    if (temperatureSensorCount > 0) {
      uint8_t i = 0;
      if (triggerOneWireRead) {
        DS18X20_start_meas(DS18X20_POWER_PARASITE, NULL);
        for (i = 1; i <= temperatureSensorCount; i--) {
          DS18X20_read_decicelsius(&temperatureSensorID[i][0], &measuredTemperatures[i]);
        }
        triggerOneWireRead = false;
      }
      else if (triggerDisplayRefresh) {
        for (i = 1; i <= temperatureSensorCount; i--) {
          DS18X20_format_from_decicelsius(measuredTemperatures[i], buffer);
          uint8_t p = strlen(buffer);
          buffer[p++] = '@'; buffer[p++] = 'C'; buffer[p++] = 0;

          // display
          uint8_t x = (i == 0) ? 1 : 38;
          lcd_box( x, 26, 37, 6, LCD_MODE_CLR);
          lcd_text(x, 26, DEFAULT_FONT, buffer);
        }
      }
    }
#endif

    // -------------------------------------------------------------

    triggerDisplayRefresh = false;

    // -------------------------------------------------------------

    lcd_update_all();
  }

  return 0;
}

// ----------------------------------------------------------------------------- 
//

#ifdef WITH_OWSENSORS
uint8_t owScanBus(void)
{
  uint8_t i;
  uint8_t id[OW_ROMCODE_SIZE];
  uint8_t diff = OW_SEARCH_FIRST;
  uint8_t temperatureSensorCount = 0;
  
  ow_reset();

  while (diff != OW_LAST_DEVICE && temperatureSensorCount < MAXSENSORS) {
    DS18X20_find_sensor(&diff, &id[0]);

    if (diff == OW_PRESENCE_ERR) {
      lcd_text_p(1, 1, DEFAULT_FONT, PSTR("No Sensors found.")); displayDelay(10);
      break;
    }

    if (diff == OW_DATA_ERR) {
      lcd_text_p(1, 1, DEFAULT_FONT, PSTR("1Wire Bus error.")); displayDelay(10);
      break;
    }

    for (i = 0; i < OW_ROMCODE_SIZE; i++) {
      temperatureSensorID[temperatureSensorCount][i] = id[i];
    }

    temperatureSensorCount++;
  }

  return temperatureSensorCount;
}
#endif


// ----------------------------------------------------------------------------- 
// update display (framebuffer to LCD-RAM and delay for t*100 ms
//
void displayDelay(uint8_t t)
{
  lcd_update_all();
  for (uint8_t i = 0; i < t; i++) { 
    _delay_ms(100);
  }
}

// ----------------------------------------------------------------------------- 
// ----------------------------------------------------------------------------- 
// ----------------------------------------------------------------------------- 

/*
Calculating the scale Factor: 4V instead of 5V = old/new*100 = 5/4*100=125
    //int32_t k = ( (5000 * (adcScale / 4096)) / (4000 * (adcScale / 4096))) * 100;
    //int32_t k = ( (50 * (adcScale )) / (40 * (adcScale )) ) * 10 * 10;
-> results in:
    int32_t k = 125;

-> usage:
    int32_t result;
    result  = value * k;
    result /= 100;

    k / 100 * neuer Wert  = VRef in mV  -- 125/100*4000 = 5000

    VRef / k * 100 = neuen Wert in mV   -- 5000/125*100 = 4000


    1 / 125 * 10000 = 80%

    10000 / 80 = 125 -> 125 in eeprom, 80 auf display

*/



/*
uint8_t const  c;              c ist konstant und ist ein uint8_t

const uint8_t  c;              ist genau dasselbe, da const schon
                               ganz links steht. c ist ein uint8_t und
                               dieser uint8_t ist konstant.

uint8_t const * c;             c ist ein Pointer. Und zwar ein Pointer
                               auf etwas das konstant ist. Und dieses
                               etwas ist ein uint8_t

uint8_t * const c;             c ist konstant. c ist ein konstanter
                               Pointer. Und dieser Pointer zeigt
                               auf einen uint8_t. Dieser uint8_t
                               ist nicht konstant, sondern kann
                               über den Pointer verändert werden.

uint8_t const * const c;       c ist konstant. c ist ein konstanter
                               Pointer. Und dieser Pointer zeigt auf
                               etwas, was selbst wieder konstant ist.
                               Und dieses seinerseits konstante ist
                               ein uint8_t

const uint8_t * const c;       Ist genau dasselbe, wie die Version
                               zuvor.
*/
