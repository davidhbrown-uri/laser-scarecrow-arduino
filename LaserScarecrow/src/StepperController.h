/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */
 
#include "config.h"
#ifndef _STEPPERCONTROLLER_h
#define _STEPPERCONTROLLER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class StepperController
{
  protected:
    static volatile int _stepsToStep;
    static volatile bool _sleeping;
    static inline void _wake();
  public:
    static void init();
    static void update();
    static void stop();
    static void runFullstep();
    static void runHalfstep();
    static void setStepsToStep(int steps);
    static void setStepsToStepRandom(int minSteps, int maxSteps);
    static int getStepsToStep();
};

#endif