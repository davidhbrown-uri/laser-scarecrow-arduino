/*
  Design used small CdS photocells with a resistance of around
  2.4k in room light (~170 lux), a bit under 100 ohms directly
  under a lamp, and over 140k in the shadow of a cupped hand.

  A 8.2k resistor between the analog pin and ground provided
  a good range of readings, 760 in room light, 160 in hand's
  shadow, and 1000 under the lamp.

  Twilight generally reads similar resistance to room light.
*/
#ifndef AmbientLightSensor_h
#define AmbientLightSensor_h
#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

class AmbientLightSensor
{
    static int _threshold, _average, _readingsIndex;
    static int _readings[];
    static unsigned long _lastReadingMillis;
  public:
    static void init();
    static void update();
    static int read();
    static void setThreshold(int value);
    static bool isDark();
    static bool isLight();
};
#endif
