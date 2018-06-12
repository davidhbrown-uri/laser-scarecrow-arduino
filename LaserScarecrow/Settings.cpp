/**
   @see Settings.h

*/
#include "config.h"
#include "Settings.h"
Settings::Settings()
{
  init();
}
void Settings::init() {
  stepper_randomsteps_min = STEPPER_RANDOMSTEPS_MIN;
  stepper_randomsteps_max = STEPPER_RANDOMSTEPS_MAX;
  stepper_stepsWhileSeeking = STEPPER_STEPS_WHILE_SEEKING;
  cycle_mode = 0; //use light sensor
  light_sensor_threshold = AMBIENTLIGHTSENSOR_DEFAULT_THRESHOLD;
  // selection of defaults based on twilight extremes in June and July 2018 from:
  // https://www.sunrisesunset.com/USA/RhodeIsland/
  rtc_wake = 278; // 4:38 AM... 4h * 60 + 38m = 278 minutes
  rtc_sleep = 1258; // 8:58pm... 20h * 60 + 58m = 1258 minutes
  interrupt_frequency = 100;
  stepper_target = 0;
  servo_min = SERVO_PULSE_USABLE_MIN;
  servo_max = SERVO_PULSE_USABLE_MAX;
  servo_hold_time = SERVO_HOLD_TIME_MS;
}
#ifdef DEBUG_SERIAL
#ifdef DEBUG_SETTINGS
void Settings::printToStream(Stream *st)
{
  st->println();
  st->println(F(">> Settings:"));
  st->print(F("  > Interrupt Frequency = "));
  st->println(interrupt_frequency);
  st->print(F("  > Servo min/max = "));
  st->print(servo_min);
  st->print('/');
  st->println(servo_max);
  st->println();  
}
#endif
#endif
