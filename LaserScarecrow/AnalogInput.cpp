/*
   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino
*/

#include "AnalogInput.h"
AnalogInput::AnalogInput(byte pin)
{
  AnalogInput(pin, ANALOGINPUT_DEFAULT_READINGS);
}
AnalogInput::AnalogInput(byte pin, byte numReadings)
{
  AnalogInput(pin, numReadings, ANALOGINPUT_DEFAULT_MS);
}
AnalogInput::AnalogInput(byte pin, byte numReadings, unsigned int ms)
{
  AnalogInput(pin, numReadings, ms, ANALOGINPUT_DEFAULT_CHANGE_THRESHOLD);
}
AnalogInput::AnalogInput(byte pin, byte numReadings, unsigned int ms, int changeThreshold)
{
  _pin = pin;
  _numReadings = constrain(numReadings, 0, ANALOGINPUT_READING_ARRAY_SIZE);
  _changeThreshold = max(0, changeThreshold);
  _msBetweenReads = max(0, ms);
  pinMode(_pin, INPUT);
  _value = analogRead(_pin);
  _lastReadMillis = millis();
  for (_readIndex = 0; _readIndex < _numReadings; _readIndex++)
  {
    _readings[_readIndex] = _value;
  }
  _readIndex = 0;
  _hasNewValue = false;
}
void AnalogInput::process()
{
  if (millis() - _lastReadMillis >= _msBetweenReads)
  {
    _readings[_readIndex++] = analogRead(_pin);
    _readIndex %= _numReadings;
    _lastReadMillis = millis();
    /* 
     *  _lastReadMillis = millis();//caution: possible timing drift probably don't care, but...
     * @todo consider alternate approach of _lastReadMillis += _msBetweenReads; 
     * Could this cause a race condition if the loop took a lot more time
     * than _msBetweenReads ? @link https://playground.arduino.cc/Code/TimingRollover
     */
    unsigned long sum = 0L;
    for (byte i = 0; i < _numReadings; i++)
    {
      sum += (unsigned long) _readings[i];
    }
    int average = (int)(sum / _numReadings);
    if (abs(average - _value) > _changeThreshold)
    {
      _value = average;
      _hasNewValue = true;
    }// if change
  } // if time for a reading
}
bool AnalogInput::hasNewValue()
{
  return _hasNewValue;
}
int AnalogInput::getValue()
{
  return _value;
}
void AnalogInput::acknowledgeNewValue()
{
  _hasNewValue = false;
}

