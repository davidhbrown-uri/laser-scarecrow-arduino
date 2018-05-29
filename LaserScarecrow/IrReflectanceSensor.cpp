#include "IrReflectanceSensor.h"
#include "config.h"

int IrReflectanceSensor::_presentThreshold;
int IrReflectanceSensor::_absentThreshold;

void IrReflectanceSensor::init()
{
  _presentThreshold = IR_REFLECTANCE_DEFAULT_PRESENT;
  _absentThreshold = IR_REFLECTANCE_DEFAULT_ABSENT;
  pinMode(IR_REFLECTANCE_PIN, INPUT);
}

void IrReflectanceSensor::setAbsentThreshold(int value) {
  _absentThreshold = value;
}
void IrReflectanceSensor::setPresentThreshold(int value) {
  _presentThreshold = value;
}
bool IrReflectanceSensor::isAbsent() {
  return read() < _absentThreshold;
}
bool IrReflectanceSensor::isPresent() {
  return read() > _presentThreshold;
}

int IrReflectanceSensor::read()
{
  return analogRead(IR_REFLECTANCE_PIN);
}


