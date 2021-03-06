/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */
 
#include "ServoController.h"
#include "config.h"
#include <Servo.h>

#define SERVO_RUNNING_STATE_SEEKING 1
#define SERVO_RUNNING_STATE_HOLDING 2
#define SERVO_RUNNING_STATE_MANUAL 3

Servo ServoController::_servo;
bool ServoController::_running;
int ServoController::_pulse;
int ServoController::_pulseTarget;
int ServoController::_pulseRangeMin;
int ServoController::_pulseRangeMax;
unsigned long ServoController::_millisBeginHold;
unsigned int ServoController::_holdTimeMillis;
unsigned int ServoController::_runningState;

void ServoController::init() {
  Servo _servo;
  
  applySettings(& currentSettings);

  _pulse = (_pulseRangeMin + _pulseRangeMax ) / 2;
  _pulseTarget = _pulse;
  _running = false;
  _millisBeginHold=0L;
  _runningState = SERVO_RUNNING_STATE_SEEKING;
}
void ServoController::update()
{
  if (_running)
  {
    switch (_runningState)
    {
      case SERVO_RUNNING_STATE_SEEKING:
        _move();
        if (_pulse == _pulseTarget)
        {
          _runningState = SERVO_RUNNING_STATE_HOLDING;
          _millisBeginHold = millis();
          _servo.detach();
        }
        break;
      case SERVO_RUNNING_STATE_MANUAL: //manual never holds so never picks a new value
        _move();
          //added manual hold timeout to fix issue #28
         if ((millis() - _millisBeginHold) > SERVO_MANUAL_HOLD_MS)
        {
          //exit manual hold
          _runningState = SERVO_RUNNING_STATE_SEEKING;
        }
        break;
      case SERVO_RUNNING_STATE_HOLDING:
        if ((millis() - _millisBeginHold) > _holdTimeMillis)
        {
          //pick new target:
          _runningState = SERVO_RUNNING_STATE_SEEKING;
          _pulseTarget = random(_pulseRangeMin, _pulseRangeMax);
          _servo.attach(SERVO_PIN_PULSE);
        }
        break;
      default: //shouldn't happen, but just in case...
        init();
    }

  } // if running
}
void ServoController::_move() {
  if (_pulse != _pulseTarget && (millis()-_millisBeginHold > SERVO_CHANGE_TIME_MS)) {
    // how far to go?
      int delta = _pulseTarget - _pulse;
    // limit to how far we can go in one step
      delta = max(-SERVO_PULSE_DELTA, delta);
      delta = min(SERVO_PULSE_DELTA, delta);
      _pulse += delta;
      _servo.writeMicroseconds(_pulse);
    _millisBeginHold = millis();
  }
}
void ServoController::stop()
{
  _servo.detach();
  _running = false;
  _runningState = SERVO_RUNNING_STATE_HOLDING;
}
void ServoController::run()
{
  _servo.attach(SERVO_PIN_PULSE);
  _servo.writeMicroseconds(_pulse);
  _running = true;
  _runningState = SERVO_RUNNING_STATE_HOLDING;
}

void ServoController::runManually()
{
  run();
  _runningState = SERVO_RUNNING_STATE_MANUAL;
}

void ServoController::setPulseRange(int low, int high)
{
  _pulseRangeMin = max(low,SERVO_PULSE_SAFETY_MIN);
  _pulseRangeMax = min(high,SERVO_PULSE_SAFETY_MAX);
}

void ServoController::setHoldTime(int ms)
{
  _holdTimeMillis = max(0,ms);
}

bool ServoController::isRunning() {
  return _running;
}

int ServoController::getPulse() {
  return _pulse;
}
int ServoController::getPulseTarget() {
  return _pulseTarget;
}
void ServoController::setPulseTarget(int pulse) {
  _pulseTarget = constrain(pulse, SERVO_PULSE_SAFETY_MIN, SERVO_PULSE_SAFETY_MAX);
  //added manual hold timeout to fix issue #28
  _millisBeginHold = millis();
  _runningState=SERVO_RUNNING_STATE_MANUAL;//this alone was causing issue #28; entered manual and never left.
}
int ServoController::getPulseRangeMin() {
  return _pulseRangeMin;
}
int ServoController::getPulseRangeMax() {
  return _pulseRangeMax;
}
void ServoController::applySettings(Settings *settings)
{
  setPulseRange(settings->servo_min, settings->servo_max);
  setHoldTime(settings->servo_hold_time);
}
