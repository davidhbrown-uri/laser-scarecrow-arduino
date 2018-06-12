#ifndef _SERVOCONTROLLER_h
#define _SERVOCONTROLLER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
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
    static volatile bool _running;
    static volatile int _pulse;
    static volatile int _pulseTarget;
    static volatile int _pulseRangeMin;
    static volatile int _pulseRangeMax;
    static unsigned long _millisBeginHold;
    static unsigned int _holdTimeMillis;
  static unsigned int _runningState; public:
    static void init();
    static void update();
    static void stop();
    static void run();
    static bool isRunning();
    static void setPulseRange(int low, int high);
    static void setHoldTime(unsigned int ms);
    static int getPulse();
    static int getPulseTarget();
    static int getPulseRangeMin();
    static int getPulseRangeMax();
    static void applySettings(Settings *settings);
};

#endif

