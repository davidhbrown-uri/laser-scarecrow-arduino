/*
  Uses the TCRT5000 IR emitter/detector pair.
  See http://blog.huntgang.com/2014/06/17/arduino-tcrt5000-build-ir-sensor/
  100-ohm resistor limiting 5V current before the anode of the emitter (blue)
  10k-ohm resistor between 5V and the collector of the detector (black)

  Low values = no tape; high IR reflectance from white bucket
  High values = tape is absorbing IR
*/
#ifndef IR_REFLECTANCE_SENSOR_h
#define IR_REFLECTANCE_SENSOR_h
#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif


class IrReflectanceSensor
{
    static int _presentThreshold;
    static int _absentThreshold;
  public:
    static void init();
    static void setPresentThreshold(int value);
    static void setAbsentThreshold(int value);
    static bool isPresent();
    static bool isAbsent();
    static int read();
};
#endif // not defined
