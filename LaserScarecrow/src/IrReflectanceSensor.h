/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */
 
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
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


class IrReflectanceSensor
{
//    static int _presentThreshold;
    static int _lowerThreshold;
    static int _upperThreshold;
    static bool _disabled;
    static bool _present;
  public:
    static void init();
//    static void setPresentThreshold(int value);
    static void setThreshold(int lower, int upper);
    static void setDisabled(bool disabled);
    static bool isPresent();
    static bool isDisabled();
    static int read();
    static int readAverage(int count);
};
#endif // not defined
