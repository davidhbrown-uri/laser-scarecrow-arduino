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

#include "Settings.h"
extern Settings currentSettings;

class SettingsObserver {
  private:
    static Settings _settings;
  public:
  //static only; no constructor
    static void init();
    static void process();
};
#endif
