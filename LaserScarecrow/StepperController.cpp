/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */
 
#include "StepperController.h"
#include "config.h"

volatile int StepperController::_stepsToStep;
volatile bool StepperController::_sleeping;

void StepperController::init()
{
  //set up i/o based on defines in config.h:
  pinMode(STEPPER_PIN_SLEEP, OUTPUT);
  pinMode(STEPPER_PIN_STEP, OUTPUT);
  pinMode(STEPPER_PIN_DIR, OUTPUT);
  pinMode(STEPPER_PIN_MICROSTEP, OUTPUT);
  digitalWrite(STEPPER_PIN_SLEEP, LOW);
  digitalWrite(STEPPER_PIN_STEP, LOW);
  digitalWrite(STEPPER_PIN_DIR, LOW);
  digitalWrite(STEPPER_PIN_MICROSTEP, LOW);
  stop();
  setStepsToStep(0);
}
void StepperController::stop()
{
  digitalWrite(STEPPER_PIN_SLEEP, LOW);
  _sleeping = true;
}
inline void StepperController::_wake()
{
  /*
     From A4988 datasheet:
     "When emerging from Sleep mode, in order
     to allow the charge pump to stabilize, provide a delay of 1 ms
     before issuing a Step command."
  */
  if (_sleeping)
  {
    digitalWrite(STEPPER_PIN_SLEEP, HIGH);
    delay(1);
    _sleeping = false;
  }
}
void StepperController::runFullstep()
{
  _wake();
  digitalWrite(STEPPER_PIN_MICROSTEP, LOW);
}
void StepperController::runHalfstep()
{
  _wake();
  digitalWrite(STEPPER_PIN_MICROSTEP, HIGH);
}


void StepperController::update()
{
  if (!_sleeping && _stepsToStep != 0) // supposed to be moving
  {
    if (_stepsToStep > 0) //turning "forwards"
    {
      digitalWrite(STEPPER_PIN_DIR, HIGH);
      _stepsToStep--;
    }
    else // turning "backwards"
    {
      digitalWrite(STEPPER_PIN_DIR, LOW);
      _stepsToStep++;
    }
    // do a step by toggling
    digitalWrite(STEPPER_PIN_STEP, HIGH);
    digitalWrite(STEPPER_PIN_STEP, LOW);
  }// if moving

}

void StepperController::setStepsToStepRandom(int minSteps, int maxSteps)
{
  setStepsToStep(random(minSteps, maxSteps));
#ifdef DEBUG_SERIAL
#ifdef DEBUG_STEPPER
  Serial.print(F("\r\nStepperController::setStepsToStepRandom stepsToStep="));
  Serial.print(getStepsToStep());
  Serial.print(F(" ("));
  Serial.print(minSteps);
  Serial.print(F("..."));
  Serial.print(maxSteps);
  Serial.println(F(")"));
#endif
#endif
}

int StepperController::getStepsToStep()
{
  return _stepsToStep;
}

void StepperController::setStepsToStep(int steps)
{
  _stepsToStep = steps;
}

