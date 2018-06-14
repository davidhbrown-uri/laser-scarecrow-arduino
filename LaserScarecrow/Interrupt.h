/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */
 
#ifndef _INTERRUPT_h
#define _INTERRUPT_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <Servo.h>
#include "Settings.h"
extern Settings currentSettings;

class Interrupt
{
    static int _frequency;
  public:
    static void init();
    static void setFrequency(int hz);
    static int getFrequency();
    static void applySettings(Settings *settings);
};

#endif

