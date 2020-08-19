/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */
 
#include "LaserController.h"
#include "config.h"

bool LaserController::_requestedOn;
bool LaserController::_isOn;
bool LaserController::_isCoolingdown;
unsigned long LaserController::_lastRun_millis;
bool LaserController::_lastRun_isOn;
unsigned long LaserController::_accumulatedOnTime;
unsigned long LaserController::_accumulatedOffTime;
unsigned long LaserController::_dutyCycleMaxRuntime;
unsigned long LaserController::_dutyCycleMinCooldown;


void LaserController::init()
{
#ifdef DEBUG_SERIAL
#ifdef DEBUG_LASERCONTROLLER
  Serial.print(F("\r\nInitializing laser controller on pin "));
  Serial.println(LASER_PIN_POWER);
#endif
#endif
  pinMode(LASER_PIN_POWER, OUTPUT);
  turnOff();
  _accumulatedOnTime = 0;
  setDutyCycleMaxRuntime(LASERCONTROLLER_DEFAULTDUTYCYCLERUNTIME);
  setDutyCycleMinCooldown(LASERCONTROLLER_DEFAULTDUTYCYCLECOOLDOWN);
  _lastRun_isOn = false;
  _lastRun_millis = millis();
}

void LaserController::setDutyCycleMaxRuntime(unsigned long millis)
{
  _dutyCycleMaxRuntime = millis;
}

void LaserController::setDutyCycleMinCooldown(unsigned long millis)
{
  _dutyCycleMinCooldown = millis;
}

void LaserController::update()
{
  // STEP 1: Timekeeping
  unsigned long elapsed = millis() - _lastRun_millis;
  // add the elapsed time to the appropriate counter.
  // I'm presuming that it will never hit the 50-day wraparound, but if your
  // application suggests otherwise, better add some code to detect that.
  if (_lastRun_isOn) {
    _accumulatedOnTime += elapsed;
  }
  else {
    _accumulatedOffTime += elapsed;
  }

  // STEP 2: Enter or leave cooldown
  if (_isOn && (_accumulatedOnTime > _dutyCycleMaxRuntime)) _enterCooldown();
  if (_isCoolingdown && (_accumulatedOffTime > _dutyCycleMinCooldown)) _leaveCooldown();

  // STEP 3: Figure out what to do next and prepare
  bool nextIsOn = _requestedOn && !_isCoolingdown;

  // If we're currently off but will be turning on,
  // subtract the time off from the accumulated on time
  if (!_isOn && nextIsOn) {
    //not using max(on-off,0) because subtraction of unsigned longs won't be < 0
    if (_accumulatedOnTime > _accumulatedOffTime) {
      _accumulatedOnTime -= _accumulatedOffTime; //maybe scale some day?
    }
    else {
      _accumulatedOnTime = 0;
    }
  }

  // STEP 4: do it and save state from this run;
  if (nextIsOn) {
    if (!_isOn) __on();
  }
  else {
    if (_isOn) __off();
  }
  _lastRun_isOn = nextIsOn;
  _lastRun_millis = millis();
}

void LaserController::doInterrupt()
{
#ifdef LASER_TOGGLE_WITH_INTERRUPT
  if (_isOn)
  {
    digitalWrite(LASER_PIN_POWER, !digitalRead(LASER_PIN_POWER));
  }
#endif
}

void LaserController::turnOn()
{
#ifdef DEBUG_SERIAL
#ifdef DEBUG_LASERCONTROLLER
  if (!_requestedOn) {
    Serial.println(F("\r\nLaser controller received turnOn request"));
  }
#endif
#endif
  _requestedOn = true;
}

void LaserController::turnOff()
{
#ifdef DEBUG_SERIAL
#ifdef DEBUG_LASERCONTROLLER
  if (_requestedOn) {
    Serial.println(F("\r\nLaser controller received turnOff request"));
  }
#endif
#endif
  _requestedOn = false;
}

bool LaserController::isOn()
{
  return _isOn;
}

bool LaserController::isCoolingDown()
{
  return _isCoolingdown;
}

void LaserController::_enterCooldown()
{
  __off();
  _isCoolingdown = true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_LASERCONTROLLER
  Serial.println(F("\r\nLaser controller entered cooldown"));
#endif
#endif
}

/*
  Reset accumulatedOnTime to zero
*/
void LaserController::_leaveCooldown()
{
  _isCoolingdown = false;
  _accumulatedOnTime = 0;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_LASERCONTROLLER
  Serial.println(F("\r\nLaser controller left cooldown"));
#endif
#endif
}

void LaserController::__on()
{
  digitalWrite(LASER_PIN_POWER, HIGH);
  _accumulatedOffTime = 0;
  _isOn = true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_LASERCONTROLLER
  Serial.println(F("\r\nLaser controller switched on"));
#endif
#endif
}

void LaserController::__off()
{
  digitalWrite(LASER_PIN_POWER, LOW);
  _isOn = false;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_LASERCONTROLLER
  Serial.println(F("\r\nLaser controller switched off"));
#endif
#endif
}

#ifdef DEBUG_LASER_SERIAL
long LaserController::getAccumulatedOnTime() {
  return _accumulatedOnTime;
}
#endif
