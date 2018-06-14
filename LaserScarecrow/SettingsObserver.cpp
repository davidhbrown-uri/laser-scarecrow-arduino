/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */

 /** @see SettingsObserver.h */
 #include "SettingsObserver.h"

Settings SettingsObserver::_settings;
 void SettingsObserver::init()
{ 
   _settings=currentSettings; // shallow copy needed
}
void SettingsObserver::process()
{
  bool foundChanges = false;

/* future: StepperController doesn't have applySettings yet as there are no controls
  // Check Stepper
  if ( _settings.stepper_randomsteps_min != currentSettings.stepper_randomsteps_min
    || _settings.stepper_randomsteps_max != currentSettings.stepper_randomsteps_max
    || _settings.stepper_stepsWhileSeeking != currentSettings.stepper_stepsWhileSeeking)
    {
      StepperController::applySettings(&currentSettings);
      foundChanges=true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_SETTINGSOBSERVER
Serial.println(F("\r\nSettingsObserver found changes in Stepper settings"));
#endif
#endif
    }
*/

      // Check Light sensor
  if ( _settings.light_sensor_threshold != currentSettings.light_sensor_threshold)
    {
      AmbientLightSensor::applySettings(&currentSettings);
      foundChanges=true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_SETTINGSOBSERVER
Serial.println(F("\r\nSettingsObserver found changes in Ambient Light Sensor settings"));
#endif
#endif
    }


//    byte cycle_mode; //depends on RTC feature not yet implemented
//    word rtc_wake; //depends on RTC feature not yet implemented
//    word rtc_sleep; //depends on RTC feature not yet implemented
//    int stepper_target; // not implemented

      
  // Check Light sensor
  if ( _settings.interrupt_frequency != currentSettings.interrupt_frequency)
    {
      Interrupt::applySettings(&currentSettings);
      foundChanges=true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_SETTINGSOBSERVER
Serial.println(F("\r\nSettingsObserver found changes in Interrupt settings"));
#endif
#endif
    }

  // Check Servo
  if ( _settings.servo_min != currentSettings.servo_min
    || _settings.servo_max != currentSettings.servo_max
    || _settings.servo_hold_time != currentSettings.servo_hold_time)
    {
      ServoController::applySettings(&currentSettings);
      foundChanges=true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_SETTINGSOBSERVER
Serial.println(F("\r\nSettingsObserver found changes in ServoController settings"));
#endif
#endif
    }

    if(foundChanges)
    {
      _settings = currentSettings; // shallow copy needed
    }
}

