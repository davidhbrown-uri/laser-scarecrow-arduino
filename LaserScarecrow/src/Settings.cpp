/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */
 
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
  light_sensor_threshold = AMBIENTLIGHTSENSOR_DEFAULT_THRESHOLD;
  interrupt_frequency = INTERRUPT_FREQUENCY_DEFAULT;
  servo_min = SERVO_PULSE_USABLE_MIN;
  servo_max = SERVO_PULSE_USABLE_MAX;
  servo_hold_time = SERVO_HOLD_TIME_MS;
}
#ifdef DEBUG_SERIAL
#ifdef DEBUG_SETTINGS
void Settings::printToStream(Stream *st)
{
  st->println();//if previous output
  //header
  st->println(F(">>> Settings:"));
  //knob equivalents:
  st->print(F(" >> Speed (interrupt frequency Hz) = "));
  st->println(interrupt_frequency);
  st->print(F(" >> Servo min/max (pulse width) = "));
  st->print(servo_min);
  st->print('/');
  st->println(servo_max);
  // other settings
  st->print(F("  > Servo hold time (ms) = "));
  st->println(servo_hold_time);
  st->print(F(" >> Stepper random movement min/max (micro-steps) = "));
  st->print(stepper_randomsteps_min);
  st->print('/');
  st->println(stepper_randomsteps_max);
  st->print(F("  > Stepper steps while seeking (full steps) = "));
  st->println(stepper_stepsWhileSeeking);
  st->print(F("  > Light sensor threshold (10-bit DAC value) = "));
  st->println(light_sensor_threshold);
  st->println();  
}
#endif
#endif
