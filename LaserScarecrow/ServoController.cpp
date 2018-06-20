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
volatile bool ServoController::_running;
volatile int ServoController::_pulse;
volatile int ServoController::_pulseTarget;
volatile int ServoController::_pulseRangeMin;
volatile int ServoController::_pulseRangeMax;
unsigned long ServoController::_millisBeginHold;
unsigned int ServoController::_holdTimeMillis;
unsigned int ServoController::_runningState;

void ServoController::init() {
  Servo _servo;
  
  applySettings(& currentSettings);

  _pulse = (_pulseRangeMin + _pulseRangeMax ) / 2;
  _pulseTarget = _pulse;
  _running = false;
  _runningState = SERVO_RUNNING_STATE_SEEKING;
}
void ServoController::update()
{
  if (_running)
  {
    switch (_runningState)
    {
      case SERVO_RUNNING_STATE_SEEKING:
        if (_pulse == _pulseTarget)
        {
          _runningState = SERVO_RUNNING_STATE_HOLDING;
          _millisBeginHold = millis();
          _servo.detach();
        }
        //if, not else: we want to fall through to this
      case SERVO_RUNNING_STATE_MANUAL: //manual never holds so never picks a new value
        if (_pulse != _pulseTarget)
        {
          int delta = _pulseTarget - _pulse;
          delta = max(-SERVO_PULSE_DELTA_LIMIT, delta);
          delta = min(SERVO_PULSE_DELTA_LIMIT, delta);
          _pulse += delta;
          _servo.writeMicroseconds(_pulse);
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
        _runningState = SERVO_RUNNING_STATE_HOLDING;
    }

  } // if running
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
  _runningState=SERVO_RUNNING_STATE_MANUAL;
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
