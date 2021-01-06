/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */

#include "AmbientLightSensor.h"
#include "config.h"

int AmbientLightSensor::_threshold;
int AmbientLightSensor::_readings[AMBIENTLIGHTSENSOR_READINGS_TO_AVERAGE];
int AmbientLightSensor::_readingsIndex = 0;
int AmbientLightSensor::_average;
bool AmbientLightSensor::_average_is_complete = false;
unsigned long AmbientLightSensor::_lastReadingMillis;
int AmbientLightSensor::_minimum_reading, AmbientLightSensor::_maximum_reading;

void AmbientLightSensor::init()
{
  applySettings(&currentSettings);
  pinMode(AMBIENTLIGHTSENSOR_PIN, INPUT);
  for (int i = 0; i < AMBIENTLIGHTSENSOR_READINGS_TO_AVERAGE; i++)
  {
    _readings[i] = analogRead(AMBIENTLIGHTSENSOR_PIN);
  }
  _readingsIndex = 0;
  _lastReadingMillis = millis();
  _average_is_complete = false;
  _minimum_reading = 9999;
  _maximum_reading = -1;
}

void AmbientLightSensor::update()
{
  if (millis() - _lastReadingMillis > AMBIENTLIGHTSENSOR_READ_INTERVAL_MS)
  {
    _readings[_readingsIndex] = analogRead(AMBIENTLIGHTSENSOR_PIN);
    _readingsIndex++;
    _readingsIndex = _readingsIndex % AMBIENTLIGHTSENSOR_READINGS_TO_AVERAGE;
    if (_readingsIndex == 0) // wrapped around; did complete set of readings
    {
      _average_is_complete = true;
    }
    unsigned long sum = 0L;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_LIGHTSENSOR
    Serial.print(F("\r\nAmbient light readings:"));
#endif
#endif
    for (int i = 0; i < (_average_is_complete ? AMBIENTLIGHTSENSOR_READINGS_TO_AVERAGE : _readingsIndex); i++)
    {
#ifdef DEBUG_SERIAL
#ifdef DEBUG_LIGHTSENSOR
      Serial.print(F(" "));
      if (i == (_readingsIndex + AMBIENTLIGHTSENSOR_READINGS_TO_AVERAGE - 1) % AMBIENTLIGHTSENSOR_READINGS_TO_AVERAGE)
      {
        Serial.print(F("*"));
      }
      Serial.print(_readings[i]);
#endif
#endif
      sum += (unsigned long)_readings[i];
    }
    _average = sum / (_average_is_complete ? AMBIENTLIGHTSENSOR_READINGS_TO_AVERAGE : _readingsIndex);
    if (_average < _minimum_reading)
    {
      _minimum_reading = _average;
    }
    if (_average > _maximum_reading)
    {
      _maximum_reading = _average;
    }
    _lastReadingMillis = millis();
#ifdef DEBUG_SERIAL
#ifdef DEBUG_LIGHTSENSOR
    Serial.print(F("; average="));
    Serial.print(_average);
    Serial.print(F(" ("));
    Serial.print(_minimum_reading);
    Serial.print(F(" - "));
    Serial.print(_maximum_reading);
    Serial.println(F(")"));
#endif
#endif
  }
}
void AmbientLightSensor::setThreshold(int value)
{
  _threshold = value;
}

bool AmbientLightSensor::isDark()
{
  return _average < _threshold;
}
bool AmbientLightSensor::isLight()
{
  return _average > _threshold;
}
int AmbientLightSensor::read()
{
  return _average;
}
int AmbientLightSensor::minimum_reading()
{
  return _minimum_reading;
}
int AmbientLightSensor::maximum_reading()
{
  return _maximum_reading;
}
void AmbientLightSensor::applySettings(Settings *settings)
{
  _threshold = settings->light_sensor_threshold;
}
