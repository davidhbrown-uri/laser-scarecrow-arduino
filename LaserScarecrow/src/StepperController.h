/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */
#pragma once

#include "Arduino.h"
#include "config.h"
#include "Settings.h"
extern Settings currentSettings;

class StepperController
{
private:
  static uint8_t _event;
  static int _stepsRequested;
  static uint8_t _state;
  static uint8_t _statePrevious;
  static uint8_t _stateReturn;
  static unsigned long _waitBeganAtMS;
  static unsigned long _waitDuration;
  static bool _direction;
  static void _buildTable();
  static void _initTimer();

protected:
  static const uint16_t _isr_top_accel_table_length = (STEPPER_MICROSEC_STEP_DELAY_MAX - STEPPER_MICROSEC_STEP_DELAY_MIN) / STEPPER_MICROSEC_ACCEL_STEP;
  static uint16_t _isr_top_accel_table[(STEPPER_MICROSEC_STEP_DELAY_MAX - STEPPER_MICROSEC_STEP_DELAY_MIN) / STEPPER_MICROSEC_ACCEL_STEP];
  // can reduce timer_top_accel_table_max_index to limit maximum speed.
  static uint16_t _isr_top_accel_table_max_index;
  static volatile unsigned long _isr_steps_remaining;
  static volatile unsigned long _isr_steps_taken;
  static volatile uint16_t _isr_top_accel_table_current_index;

public:
  static void init();
  /**
   * @brief call in main loop to update StepperController state
   * 
   */
  static void update();
  /**
   * @brief stop as quickly as possible, using deceleration
   * 
   */
  static void move_stop();
  /**
   * @brief Stop, then disable (sleep) the stepper driver
   * 
   */
  static void turn_off();
  /**
   * @brief Move the stepper forwards (positive value) or backwards (negative value)
   * 
   * @param steps the number of microsteps to take
   */
  static void move(int steps);
  /**
   * @brief Move the stepper in the current direction 
   * 
   * @param steps the additional microsteps to take
   */
  static void move_extend(int steps);
  static void setSpeedLimitPercent(int percent);
  static int getSpeedLimitPercent();
  static void applySettings(Settings *settings);
  static bool is_stopped();
  static void isr();
};
