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
  uint8_t _event;
  int _stepsRequested;
  uint8_t _state;
  uint8_t _statePrevious;
  uint8_t _stateReturn;
  unsigned long _waitBeganAtMS;
  unsigned long _waitDuration;
  bool _direction;
  void _buildTable();
  void _initTimer();

protected:
  const uint16_t _isr_top_accel_table_length = (STEPPER_MICROSEC_STEP_DELAY_MAX - STEPPER_MICROSEC_STEP_DELAY_MIN) / STEPPER_MICROSEC_ACCEL_STEP;
  uint16_t _isr_top_accel_table[(STEPPER_MICROSEC_STEP_DELAY_MAX - STEPPER_MICROSEC_STEP_DELAY_MIN) / STEPPER_MICROSEC_ACCEL_STEP];
  // can reduce timer_top_accel_table_max_index to limit maximum speed.
  uint16_t _isr_top_accel_table_max_index;
  volatile unsigned long _isr_steps_remaining;
  volatile unsigned long _isr_steps_taken;
  volatile uint16_t _isr_top_accel_table_current_index;

public:
  void init();
  /**
   * @brief call in main loop to update StepperController state
   * 
   */
  void update();
  /**
   * @brief stop as quickly as possible, using deceleration
   * 
   */
  void move_stop();
  /**
   * @brief Stop, then disable (sleep) the stepper driver
   * 
   */
  void turn_off();
  /**
   * @brief Move the stepper forwards (positive value) or backwards (negative value)
   * 
   * @param steps the number of microsteps to take
   */
  void move(int steps);
  /**
   * @brief Move the stepper in the current direction 
   * 
   * @param steps the additional microsteps to take
   */
  void move_extend(int steps);
  void setSpeedLimitPercent(int percent);
  int getSpeedLimitPercent();
  void applySettings(Settings *settings);
  bool is_stopped();
  static void isr(StepperController *stepper_contoller);
};
