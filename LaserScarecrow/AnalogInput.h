/*
   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino
 */

#ifndef AnalogInput_h
#define AnalogInput_h
#include "Arduino.h"

#define ANALOGINPUT_DEFAULT_READINGS 3
#define ANALOGINPUT_DEFAULT_MS 100
#define ANALOGINPUT_DEFAULT_CHANGE_THRESHOLD 51
//5% of full range 
#define ANALOGINPUT_READING_ARRAY_SIZE 8
class AnalogInput
{
  public:
    void begin(byte pin);
    void begin(byte pin, byte numReadings);
    void begin(byte pin, byte numReadings, unsigned int ms);
    void begin(byte pin, byte numReadings, unsigned int ms, int threshold);
    void process(); // possibly read the pin if enough time has elapsed 
    bool hasNewValue();
    void acknowledgeNewValue();
    int getValue();

  private:
    byte _pin;
    byte _numReadings;
    int _readings[ANALOGINPUT_READING_ARRAY_SIZE];
    int _changeThreshold;
    unsigned int _msBetweenReads;
    unsigned long _lastReadMillis;
    bool _hasNewValue;
    int _value;
    byte _readIndex;
};

#endif
