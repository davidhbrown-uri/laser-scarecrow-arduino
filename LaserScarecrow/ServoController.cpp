#include "ServoController.h"
#include "config.h"
#include <Servo.h>

#define SERVO_RUNNING_STATE_SEEKING 1
#define SERVO_RUNNING_STATE_HOLDING 2

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

  _pulse = (SERVO_PULSE_USABLE_MIN + SERVO_PULSE_USABLE_MAX ) / 2;
  _pulseTarget = _pulse;
  _pulseRangeMin = _pulse;
  _pulseRangeMax = _pulse;
  _running = false;
  _runningState = SERVO_RUNNING_STATE_SEEKING;
  setHoldTime(SERVO_HOLD_TIME_MS);
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
        _runningState = SERVO_RUNNING_STATE_SEEKING;
    }

  } // if running
}
void ServoController::stop()
{
  _servo.detach();
  _running = false;
}
void ServoController::run()
{
  _servo.attach(SERVO_PIN_PULSE);
  //_servo.write(_angle);
  _servo.writeMicroseconds(_pulse);
  _running = true;
}

void ServoController::setPulseRange(int low, int high)
{
  _pulseRangeMin = low;
  _pulseRangeMax = high;
  if (_pulseRangeMin < SERVO_PULSE_SAFETY_MIN) _pulseRangeMin = SERVO_PULSE_SAFETY_MIN;
  if (_pulseRangeMax > SERVO_PULSE_SAFETY_MAX) _pulseRangeMax = SERVO_PULSE_SAFETY_MAX;
}

void ServoController::setHoldTime(unsigned int ms)
{
  _holdTimeMillis = ms;
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
int ServoController::getPulseRangeMin() {
  return _pulseRangeMin;
}
int ServoController::getPulseRangeMax() {
  return _pulseRangeMax;
}

