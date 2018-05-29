#ifndef _INTERRUPT_h
#define _INTERRUPT_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <Servo.h>

class Interrupt
{
    static int _frequency;
  public:
    static void init();
    static void setFrequency(int hz);
    static int getFrequency();
};

#endif

