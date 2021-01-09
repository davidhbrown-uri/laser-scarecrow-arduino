/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */

#include "config.h"
#ifndef _STEPPERCONTROLLER_h
#define _STEPPERCONTROLLER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "Settings.h"
extern Settings currentSettings;

class StepperController
{
protected:
  static volatile int _stepsToStep;
  static volatile bool _sleeping;
  static inline void _wake();
  static void _buildTable();
  // TIMER 3 is a 16-bit timer with a prescaler
  static void _initTimer();
  static bool _direction;
  static const uint16_t _timer_top_accel_table_length = (STEPPER_MICROSEC_STEP_DELAY_MAX - STEPPER_MICROSEC_STEP_DELAY_MIN) / STEPPER_MICROSEC_ACCEL_STEP;
  static uint16_t _timer_top_accel_table[(STEPPER_MICROSEC_STEP_DELAY_MAX - STEPPER_MICROSEC_STEP_DELAY_MIN) / STEPPER_MICROSEC_ACCEL_STEP];
  // can reduce timer_top_accel_table_max_index to limit maximum speed.
  static uint16_t _timer_top_accel_table_max_index;
  volatile unsigned long timer_steps_remaining = 0L;
  volatile unsigned long timer_steps_taken = 0L;
  volatile uint16_t timer_top_accel_table_current_index = 0;

public:
  static void init();
  static void update();
  static void stop();
  static void runFullstep();
  static void runHalfstep();
  static void addStepsToStep(int steps);
  static void setStepsToStep(int steps);
  static void setStepsToStepRandom(int minSteps, int maxSteps);
  static int getStepsToStep();
  static void setSpeedLimitPercent(int percent);
  static int getSpeedLimitPercent();
  static void applySettings(Settings *settings);
};

#endif
