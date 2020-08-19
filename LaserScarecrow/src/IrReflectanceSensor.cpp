/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

*/

#include "IrReflectanceSensor.h"
#include "config.h"

//int IrReflectanceSensor::_presentThreshold;
int IrReflectanceSensor::_lowerThreshold, IrReflectanceSensor::_upperThreshold;
bool IrReflectanceSensor::_disabled, IrReflectanceSensor::_present;

void IrReflectanceSensor::init()
{
//  _presentThreshold = IR_REFLECTANCE_DEFAULT_PRESENT;
  _lowerThreshold = IR_REFLECTANCE_DEFAULT_PRESENT;
  _upperThreshold = IR_REFLECTANCE_DEFAULT_PRESENT;
  _present = false;
  pinMode(IR_REFLECTANCE_PIN, INPUT);
}

void IrReflectanceSensor::setDisabled(bool disabled) {
  _disabled = disabled;
}
/*
void IrReflectanceSensor::setPresentThreshold(int value) {
  _presentThreshold = value;
}
*/

void IrReflectanceSensor::setThreshold(int lower, int upper) {
  _lowerThreshold = lower;
  _upperThreshold = upper;
}

bool IrReflectanceSensor::isPresent() {
  if (_disabled) return false;
  int value = read();
  if (_present && value >= _upperThreshold) 
  {
    _present = false;
  }
  else if (!_present && value <= _lowerThreshold)
  {
    _present = true;
  }
  return _present ^ IR_REFLECTANCE_INVERT;
}
bool IrReflectanceSensor::isDisabled() {
  return _disabled;
}

int IrReflectanceSensor::read()
{
  return analogRead(IR_REFLECTANCE_PIN);
}

int IrReflectanceSensor::readAverage(int count)
{
  long sum = 0L;
  for(int i=0; i<count; i++)
  {
    sum+=IrReflectanceSensor::read();
  }
  sum /= count;
  return (int) sum; 
}
