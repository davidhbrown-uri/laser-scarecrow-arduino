/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */
 
#include "AmbientLightSensor.h"
#include "config.h"

int AmbientLightSensor::_threshold;
int AmbientLightSensor::_readings[AMBIENTLIGHTSENSOR_READINGS_TO_AVERAGE];
int AmbientLightSensor::_readingsIndex;
int AmbientLightSensor::_average;
unsigned long AmbientLightSensor::_lastReadingMillis;

void AmbientLightSensor::init()
{
  applySettings(& currentSettings);
  pinMode(AMBIENTLIGHTSENSOR_PIN, INPUT);
  for (int i = 0; i < AMBIENTLIGHTSENSOR_READINGS_TO_AVERAGE; i++)
  {
    _readings[i] = analogRead(AMBIENTLIGHTSENSOR_PIN);
  }
  _readingsIndex = 0;
  _lastReadingMillis = millis();
}

void AmbientLightSensor::update()
{
  if (millis() - _lastReadingMillis > AMBIENTLIGHTSENSOR_READ_INTERVAL_MS)
  {
    _readingsIndex = ++_readingsIndex % AMBIENTLIGHTSENSOR_READINGS_TO_AVERAGE;
    _readings[_readingsIndex] = analogRead(AMBIENTLIGHTSENSOR_PIN);
    unsigned long sum = 0L;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_LIGHTSENSOR
    Serial.print(F("\r\nAmbient light readings:"));
#endif
#endif
    for (int i = 0; i < AMBIENTLIGHTSENSOR_READINGS_TO_AVERAGE; i++)
    {
#ifdef DEBUG_SERIAL
#ifdef DEBUG_LIGHTSENSOR
      Serial.print(F(" "));
      Serial.print(_readings[i]);
#endif
#endif
      sum += (unsigned long) _readings[i];
    }
    _average = sum / AMBIENTLIGHTSENSOR_READINGS_TO_AVERAGE;
    _lastReadingMillis = millis();
#ifdef DEBUG_SERIAL
#ifdef DEBUG_LIGHTSENSOR
    Serial.print(F("; average="));
    Serial.print(_average);
    Serial.print(F("; readingsIndex: "));
    Serial.print(_readingsIndex);
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
void AmbientLightSensor::applySettings(Settings *settings)
{
  _threshold = settings->light_sensor_threshold;
}

