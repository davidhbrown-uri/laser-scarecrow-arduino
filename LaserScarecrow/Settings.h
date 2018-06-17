/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */
 
/**
   A Settings stores the behavior settings of the Laser Scarecrow
   It really isn't much more than a struct, so its properties are public
   Created by David H. Brown, February 25, 2018

   In classes that need to access the current settings, put the following near the top:
  #include "Settings.h"
  extern Settings currentSettings;

  ...and include a method such as: (not going for OOP interface, but maybe some day we'd want an array of everything to be get settings applied?)
    static void applySettings(Settings *settings);

  ... then the class's init() method can include the call:
    applySettings(& currentSettings);

  ...here's an example applySettings method:
  void AmbientLightSensor::applySettings(Settings *settings)
  {
  _threshold = settings->light_sensor_threshold;
  }

*/
#ifndef Settings_h
#define Settings_h
#include "Arduino.h"
#include "config.h"

// if/when settings are added, increment 
// the SETTINGS_VERSION, but as it will be
// stored in a single byte, if we ever reach
// 256, roll over to 0.
#define SETTINGS_VERSION 1
class Settings {
  public:
    Settings();
    void init();
    //#ifdef DEBUG_SERIAL
    //#ifdef DEBUG_SETTINGS
    void printToStream(Stream *st);
    //#endif
    //#endif
    //properties
    int stepper_randomsteps_min; //done but no control implemented
    int stepper_randomsteps_max; //done but no control implemented
    int stepper_stepsWhileSeeking; //done but no control implemented
    int light_sensor_threshold; //done but no control implemented
    byte cycle_mode; //depends on RTC feature not yet implemented
    unsigned int rtc_wake; //depends on RTC feature not yet implemented
    unsigned int rtc_sleep; //depends on RTC feature not yet implemented
    int stepper_target; // not implemented
    int interrupt_frequency; // done, including knob control
    int servo_min; // done, including knob control
    int servo_max; // done, including knob control; changed name/concept from servo_range... while the knob controls range, it makes sense for the setting to be a point, not a delta
    int servo_hold_time; // done but no control implemented
}; //class Settings
#endif

