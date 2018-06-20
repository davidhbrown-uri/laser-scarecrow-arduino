/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

*/

/**
   The SettingsObserver checks the extern currentSettings for changes against
   its own private cached _settings looking for differences. When found,
   the relevant *Class*::applySettings(&currentSettings) is called.

   In terms of the Observer design pattern, this is more like the Subject part
   of the pattern, but calling this class the SettingSubject seemed less
   meaningful. Also, the settings themselves are just an enhanced struct;
   it's not their responsibility to update the controllers. Maybe it's more of
   a Mediator? Well, whatever. It's not intended to be a model of OOP style.

   June 16 2018: feat/14_save_settings
   With a bit of reluctance, I'm also making this class responsible for saving
   and loading settings. I already have more tabs in the Arduino IDE than can fit
   on a 4k monitor!

   Addr (dec)  Name     Data
   0-3         id       "LS18" [76,83,49,56] file identifier / magic cookie -- won't be read if it doesn't match
   4-5         size     (uint16_t) sizeof(_settings) -- won't be read if doesn't match
   6-9         crc      CRC32::calculate(_settings,sizeof(_settings))
   10-10+size  data

   @todo for future: add ability to store settings at other points in the EEPROM, maybe in RAM on a RTC

   Created by David H. Brown, June 13, 2018
   for branch feat/8_Command
   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

*/
#ifndef SettingsObserver_h
#define SettingsObserver_h
#include "Arduino.h"
#include "config.h"
#include "ServoController.h"
#include "AmbientLightSensor.h"
#include "Interrupt.h"

// https://github.com/bakercp/CRC32
#include <CRC32.h>

#include "Settings.h"
extern Settings currentSettings;

class SettingsObserver {
  private:
    static Settings _settings;
    static bool _unsaved;
    static unsigned long _lastChangedMillis;
  public:
    //static only; no constructor
    static void init();
    static void process();
    static bool save(Settings *settings_ptr); // save current to storage - return true if successful
    static bool checkHeader(Settings *settings_ptr); // true if storage header is valid for this settings
    static bool verifyStoredSettings(); // true if stored checksum and data agree (ignores id, size)
    static bool load(Settings *settings_ptr); // load current from storage - return true if successful
    static uint32_t settingsCRC(Settings *settings_ptr); // calculate CRC from individual bytes of settings
#ifdef DEBUG_SERIAL
#ifdef DEBUG_SETTINGSOBSERVER
    static void viewSettings(Settings *settings_ptr);
    static void viewEEPROM();
#endif
#endif
};
#endif
