/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

*/

#ifndef _SERVOCONTROLLER_h
#define _SERVOCONTROLLER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Servo.h>
#include "Settings.h"
extern Settings currentSettings;

class ServoController
{
  protected:
    static Servo _servo;
    static bool _running;
    static int _pulse;
    static int _pulseTarget;
    static int _pulseRangeMin;
    static int _pulseRangeMax;
    static unsigned long _millisBeginHold;
    static unsigned int _holdTimeMillis;
    static unsigned int _runningState;
    static void _move();
  public:
    static void init();
    static void update();
    static void stop();
    static void run();
    static void runManually();
    static bool isRunning();
    static void setPulseRange(int low, int high);
    static void setHoldTime(int ms);
    static void setPulseTarget(int pulse);
    static int getPulse();
    static int getPulseTarget();
    static int getPulseRangeMin();
    static int getPulseRangeMax();
    static void applySettings(Settings *settings);
};

#endif
