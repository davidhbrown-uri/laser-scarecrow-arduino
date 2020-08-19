/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

*/

#include "IrReflectanceSensor.h"
#include "config.h"

int IrReflectanceSensor::_presentThreshold;
bool IrReflectanceSensor::_disabled;

void IrReflectanceSensor::init()
{
  _presentThreshold = IR_REFLECTANCE_DEFAULT_PRESENT;
  pinMode(IR_REFLECTANCE_PIN, INPUT);
}

void IrReflectanceSensor::setDisabled(bool disabled) {
  _disabled = disabled;
}
void IrReflectanceSensor::setPresentThreshold(int value) {
  _presentThreshold = value;
}

bool IrReflectanceSensor::isPresent() {
  
#ifdef IR_REFLECTANCE_INVERT
  return !_disabled && read() <= _presentThreshold;
#else
  return !_disabled && read() >= _presentThreshold;
#endif

}
bool IrReflectanceSensor::isDisabled() {
  return _disabled;
}

int IrReflectanceSensor::read()
{
  return analogRead(IR_REFLECTANCE_PIN);
}
