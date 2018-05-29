#include "config.h"
#include "Interrupt.h"
#include "StepperController.h"
#include "ServoController.h"
#include "LaserController.h"
#include "AmbientLightSensor.h"
#include "IrReflectanceSensor.h"
#include "LaserController.h"

// definitions for finite state machine
#define STATE_INIT 0
#define STATE_ACTIVE 1
#define STATE_SEEKING 2
#define STATE_DARK 3
#define STATE_COOLDOWN 4
#define STATE_POWERON 255
byte stateCurrent, statePrevious;

unsigned long loopLedLastChangeMillis = 0L;
unsigned long loopProcessLastMillis = 0L;
unsigned long loopLastSerial = 0L;
int seekingRotationLimit;

#ifdef DEBUG_AIMCONTROLLER
#ifdef DEBUG_SERIAL
unsigned long aimStatusCycle = LOOP_SERIAL_OUTPUT_RATE / 80L;
unsigned long loopLastAimStatus = 0L;
#endif
#endif
void setup() {
  // put your setup code here, to run once:
  stateCurrent = STATE_POWERON;
#ifdef DEBUG_SERIAL
  Serial.begin(DEBUG_SERIAL_DATARATE);
  Serial.println();
  delay(1000);
  for (int i = 3; i >= 0; i--) {
    Serial.print(F("Laser Scarecrow Startup in "));
    Serial.print(i);
    Serial.println(F("..."));
    delay(1000);
  }
#endif // DEBUG_SERIAL

  /*************************
     INITIALIZE I/O pins
  */

  // Knobs
  pinMode(KNOB1_PIN, INPUT);
  pinMode(KNOB2_PIN, INPUT);
  pinMode(KNOB3_PIN, INPUT);

  pinMode(LED1_PIN, OUTPUT);

  statePrevious = stateCurrent;
  stateCurrent = STATE_INIT;
}

void loop() {
  // put your main code here, to run repeatedly:
#ifdef DEBUG_SERIAL
  bool outputSerialDebug = (millis() - loopLastSerial > DEBUG_SERIAL_OUTPUT_INTERVAL_MS);
  if (outputSerialDebug) loopLastSerial = millis();
#endif // DEBUG_SERIAL
  /*
    Simple Finite State Machine: arrange each switch case as follows...
    case STATE_WHATEVER:
      if(stateCurrent!=statePrevious)
      {
        statePrevious=stateCurrent;
        // entry code here:
      }
    //...then write the do/update code for the state,
    //changing stateCurrent if the state should exit, then
    //finish each switch case with...
      if(stateCurrent!=statePrevious)
      {
        // exit code here:
      }
    break;
  */
  switch (stateCurrent) {

    case STATE_INIT:
      if (stateCurrent != statePrevious) {
        statePrevious = stateCurrent;
        //enter code:
#ifdef DEBUG_SERIAL
        Serial.println(F("\r\n[[Entering INIT State]]"));
#endif // DEBUG_SERIAL
        // pending LedController
        pinMode(LED1_PIN, OUTPUT);
        pinMode(LED2_PIN, OUTPUT);
        digitalWrite(LED1_PIN, LED1_INVERT);
        digitalWrite(LED2_PIN, LED2_INVERT);
        LaserController::init();
        ServoController::init();
        StepperController::init();
        IrReflectanceSensor::init();
        AmbientLightSensor::init();
        Interrupt::init();
        setInterruptSpeed();
        findReflectanceThreshold();
      }
      //update:
      stateCurrent = STATE_DARK;
      if (stateCurrent != statePrevious) {
        //exit code:
      }
      break;

    case STATE_ACTIVE:
      if (stateCurrent != statePrevious) {
        statePrevious = stateCurrent;
        //enter code:
#ifdef DEBUG_SERIAL
        Serial.println(F("\r\n[[Entering ACTIVE State]]"));
#endif // DEBUG_SERIAL
        LaserController::turnOn();
        ServoController::run();
        StepperController::runHalfstep();
        StepperController::setStepsToStepRandom(STEPPER_RANDOMSTEPS_MIN, STEPPER_RANDOMSTEPS_MAX);
      }
      //update:
      // check for transition events (later checks have priority)
      if (IrReflectanceSensor::isPresent()) stateCurrent = STATE_SEEKING;
      if (LaserController::isCoolingDown()) stateCurrent = STATE_COOLDOWN;
      if (AmbientLightSensor::isDark()) stateCurrent = STATE_DARK;
      // do our things:
      setInterruptSpeed();
      if (StepperController::getStepsToStep() == 0) StepperController::setStepsToStepRandom(STEPPER_RANDOMSTEPS_MIN, STEPPER_RANDOMSTEPS_MAX);
      setServoRange();
      if (stateCurrent != statePrevious) {
        //exit code:
      }
      break;
    case STATE_SEEKING:
      if (stateCurrent != statePrevious) {
        statePrevious = stateCurrent;
#ifdef DEBUG_SERIAL
        Serial.println(F("\r\n[[Entering SEEKING State]]"));
#endif // DEBUG_SERIAL
        LaserController::turnOff();
        ServoController::stop();
        StepperController::runFullstep();
        StepperController::setStepsToStep(STEPPER_FULLSTEPS_PER_ROTATION);
        seekingRotationLimit = SEEKING_ROTATION_LIMIT;
      }
      //update:
      //check for transition
      if (IrReflectanceSensor::isAbsent()) stateCurrent = STATE_ACTIVE;
      //do our things:
      if (StepperController::getStepsToStep() == 0)
      {
        StepperController::setStepsToStep(STEPPER_FULLSTEPS_PER_ROTATION);
        seekingRotationLimit--;
      }
      if (seekingRotationLimit == 0)
      {
        stateCurrent = STATE_INIT;
      }
      setInterruptSpeed();
      if (stateCurrent != statePrevious) {
        //exit code:
      }
      break;
    case STATE_DARK:
      if (stateCurrent != statePrevious) {
        statePrevious = stateCurrent;
        //enter code:
#ifdef DEBUG_SERIAL
        Serial.println(F("\r\n[[Entering DARK State]]"));
#endif // DEBUG_SERIAL
        LaserController::turnOff();
        ServoController::stop();
        StepperController::stop();
        // might save a bit of power, but mostly wanting a slow blink rate:
        Interrupt::setFrequency(1);
      }
      //update:
      if (AmbientLightSensor::isLight())
      {
        stateCurrent = STATE_SEEKING;
      }
      if (stateCurrent != statePrevious) {
        //exit code:
        // return the interrupt speed back to user-defined
        setInterruptSpeed();
      }
      break;
    case STATE_COOLDOWN:
      if (stateCurrent != statePrevious) {
        statePrevious = stateCurrent;
        //enter code:
#ifdef DEBUG_SERIAL
        Serial.println(F("\r\n[[Entering COOLDOWN State]]"));
#endif // DEBUG_SERIAL
        LaserController::turnOff(); // it may already have done this itself.
        //servo detach
        ServoController::stop();
        StepperController::stop();
        // might save a bit of power, but mostly wanting a moderate blink rate:
        Interrupt::setFrequency(3);
      }
      //update:
      if (!LaserController::isCoolingDown())
      {
        stateCurrent = STATE_ACTIVE;
      }
      if (stateCurrent != statePrevious) {
        //exit code:
        // return the interrupt speed back to user-defined
        setInterruptSpeed();
      }
      break;
  }
  // timing-imprecise tasks:
  LaserController::update();
  AmbientLightSensor::update();
  digitalWrite(LED2_PIN, IrReflectanceSensor::isPresent() ^ LED2_INVERT);

#ifdef DEBUG_SERIAL
  if (outputSerialDebug)
  {
    Serial.println();
    Serial.print(SOFTWARE_VERSION);
#ifdef DEBUG_REFLECTANCE
    Serial.print(F("IR raw="));
    Serial.print(IrReflectanceSensor::read());
    Serial.print(IrReflectanceSensor::isPresent() ? F(" Present;") : F(" not present;"));
    Serial.print(IrReflectanceSensor::isAbsent() ? F(" absent") : F(" not absent"));
#endif
#ifdef DEBUG_AMBIENT
    Serial.print(F("Ambient light average="));
    Serial.print(AmbientLightSensor::read());
    Serial.print(AmbientLightSensor::isLight() ? F(" (Light)") : F(" (Dark)"));
#endif
#ifdef DEBUG_STEPPER
    Serial.print("\r\nStepperController::getStepsToStep = ");
    Serial.print(StepperController::getStepsToStep());
#endif
#ifdef DEBUG_SERVO
    Serial.print("\r\nServoController::pulse=");
    Serial.print(ServoController::getPulse());
    Serial.print("; target=");
    Serial.print(ServoController::getPulseTarget());
    Serial.print("; rangeMin=");
    Serial.print(ServoController::getPulseRangeMin());
    Serial.print("; rangeMax=");
    Serial.print(ServoController::getPulseRangeMax());
#endif
#ifdef DEBUG_KNOBS
    Serial.print("\r\nKNOB1=");
    Serial.print(analogRead(KNOB1_PIN));
    Serial.print("; KNOB2=");
    Serial.print(analogRead(KNOB2_PIN));
    Serial.print("; KNOB3=");
    Serial.print(analogRead(KNOB3_PIN));
#endif
    Serial.println();
  } //output serial debug
#endif
} // main arduino loop


/*********************
   INTERRUPT HANDLERS
*/

inline void doInterrupt()
{
  digitalWrite(LED1_PIN, !digitalRead(LED1_PIN));
  ServoController::update();
  StepperController::update();
#ifdef LASER_TOGGLE_WITH_INTERRUPT
  LaserController::doInterrupt();
#endif
}
#ifdef INTERRUPTS_ATmega328P_T2
#define INTERRUPT_VECT TIMER2_COMPA_vect
ISR(TIMER2_COMPA_vect)
{
  doInterrupt();
}
#endif
#ifdef INTERRUPTS_ATmega32U4_T1
ISR(TIMER1_COMPA_vect)
{
  doInterrupt();
}
#endif
#ifdef INTERRUPTS_ATmega32U4_T3
ISR(TIMER3_COMPA_vect)
{
  doInterrupt();
}
#endif

/*****************
   Helper functions
   Put things here that require access to multuple modules
*/
void setInterruptSpeed()
{
  //don't do this constantly or it really messes up the timing!
  int newFequency = 12 + analogRead(KNOB1_PIN) / 8;
  if (abs(Interrupt::getFrequency() - newFequency) > 4)
  {
    Interrupt::setFrequency(newFequency);
  }
}
void setServoRange()
{
  int knob2 = analogRead(KNOB2_PIN);
  int knob3 = analogRead(KNOB3_PIN);

  int pulseLow = map(knob2, 0, 1023, SERVO_PULSE_USABLE_MIN, SERVO_PULSE_USABLE_MAX);
  int pulseHigh = map(knob3, 0, 1023, pulseLow, SERVO_PULSE_USABLE_MAX);
  ServoController::setPulseRange(pulseLow, pulseHigh);
}

// Version 1.1 attempts to find bimodal peaks and extents:
void findReflectanceThreshold() {
  int bin, value, highest = 0, lowest = 1023;
  int peakOneBin, peakOneBinMax, peakOneBinMin;
  int peakTwoBin, peakTwoBinMax, peakTwoBinMin;
  int troughBin, troughBinMin, troughBinMax;
#ifdef DEBUG_REFLECTANCE
  const char hexcode[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
#endif
  bool error = false;
#define REFLECTANCE_BINCOUNT 32
  int bins[REFLECTANCE_BINCOUNT];
  for (int i = 0; i < REFLECTANCE_BINCOUNT; bins[i++] = 0);
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_INIT_READINGS
  Serial.println(F("\r\nFinding reflectance threshold:"));
#endif
#endif
  for (int i = 0; i < IR_REFLECTANCE_READINGS; i++)
  {
    delay(1);
    value = IrReflectanceSensor::read();;
    if (value > highest) {
      highest = value;
    }
    if (value < lowest) {
      lowest = value;
    }
  }
  if (highest - lowest > (2 * IR_REFLECTANCE_MINIMUM_CONTRAST))
  {
    error = true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    Serial.println(F("Reflectance cannot be used: readings too variable without movement (failed sensor?)"));
    Serial.print(F("Low-High = ")); Serial.print(lowest); Serial.print(F("-")); Serial.println(highest);
#endif
#endif
  }

  if (! error)
  {
    highest = 0; lowest = 1023;
    StepperController::setStepsToStep(0);
    StepperController::runHalfstep();
    for (int i = 0; i < IR_REFLECTANCE_READINGS; i++) {
      value = IrReflectanceSensor::read();
      if (value > highest) {
        highest = value;
      }
      if (value < lowest) {
        lowest = value;
      }
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
#ifdef DEBUG_REFLECTANCE_INIT_READINGS
      Serial.println(value);
#else
      Serial.print(hexcode[value / 64]);
#endif
#endif
#endif
      bin = value / (1024 / REFLECTANCE_BINCOUNT);
      //smooth the histogram by adding 2 to this bin and 1 to adjacent bins (if they exist)
      bins[bin] += 2;
      if (bin >= 1) bins[bin - 1]++;
      if (bin + 1 < REFLECTANCE_BINCOUNT) bins[bin + 1]++;
      // move to next reading
      StepperController::setStepsToStep(IR_REFLECTANCE_STEPS_PER_READ);
      while (StepperController::getStepsToStep() != 0) {
        ;
      }
    }
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    Serial.println();
    Serial.print(F("Reflectance readings range from "));
    Serial.print(lowest);
    Serial.print(F(" to "));
    Serial.print(highest);
    Serial.println(F("."));
    Serial.println(F("Bins:"));
    for (int i = 0; i < REFLECTANCE_BINCOUNT; i++)
    {
      Serial.print(i);
      Serial.print(F(": "));
      Serial.println(bins[i]);
    }
#endif
#endif
    // find highest peak
    highest = -1;
    for (int i = 0; i < REFLECTANCE_BINCOUNT; i++)
    {
      if (bins[i] > highest) {
        highest = bins[i];
        peakOneBin = i;
      }
    }
    // find extent of first peak
    peakOneBinMax = peakOneBin;
    while (peakOneBinMax + 1 < REFLECTANCE_BINCOUNT && bins[peakOneBinMax + 1] < bins[peakOneBinMax]) peakOneBinMax++;
    peakOneBinMin = peakOneBin;
    while (peakOneBinMin > 0 && bins[peakOneBinMin - 1] < bins[peakOneBinMin]) peakOneBinMin--;
    if (peakOneBinMin == 0 && peakOneBinMax + 1 == REFLECTANCE_BINCOUNT)
    {
      error = true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
      Serial.println(F("Reflectance histogram peak covers full range; cannot sense tape."));
#endif
#endif
    }
    if (highest - lowest < IR_REFLECTANCE_MINIMUM_CONTRAST)
    {
      error = true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
      Serial.println(F("Reflectance cannot be used: insufficient contrast!"));
#endif
#endif
    }
  }// if !error
  if (!error) {
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    Serial.println();
    Serial.print(F("Peak One: "));
    Serial.print(peakOneBinMin);
    Serial.print(F("-"));
    Serial.print(peakOneBinMax);
    Serial.print(F(" @ "));
    Serial.println(peakOneBin);
#endif
#endif
    // find peak in remaining range
    highest = -1; peakTwoBin = -1;
    for (int i = 0; i < REFLECTANCE_BINCOUNT; i++)
    {
      if (i < peakOneBinMin || i > peakOneBinMax) // skip peakOne's range
      {
        if (bins[i] > highest) {
          highest = bins[i];
          peakTwoBin = i;
        }
      } // skip peakOne
    } // for i in bins
    // find extent of second peak
    peakTwoBinMax = peakTwoBin;
    while (peakTwoBinMax + 1 < REFLECTANCE_BINCOUNT && bins[peakTwoBinMax + 1] < bins[peakTwoBinMax]) peakTwoBinMax++;
    peakTwoBinMin = peakTwoBin;
    while (peakTwoBinMin > 0 && bins[peakTwoBinMin - 1] < bins[peakTwoBinMin]) peakTwoBinMin--;
    if (bins[peakTwoBin] == 0)
    {
      error = true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
      Serial.println(F("No observations outside range of first peak; cannot sense tape."));
#endif
#endif
    }
  } // !error
  if (!error)
  {
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    Serial.print(F("Peak Two: "));
    Serial.print(peakTwoBinMin);
    Serial.print(F("-"));
    Serial.print(peakTwoBinMax);
    Serial.print(F(" @ "));
    Serial.println(peakTwoBin);
#endif
#endif
    //find trough between peaks
    lowest = bins[peakOneBin];
    if (peakOneBin < peakTwoBin)
    {
      troughBinMin = peakOneBinMax;
      troughBinMax = peakTwoBinMin;
    }
    else
    {
      troughBinMin = peakTwoBinMax;
      troughBinMax = peakOneBinMin;
    }
    troughBin = troughBinMin;
    for (int i = troughBinMin; i <= troughBinMax; i++)
    {
      if (bins[i] < lowest)
      {
        troughBin = i;
        lowest = bins[i];
      }
    }
    //find extent of trough
    troughBinMax = troughBin;
    while (troughBinMax + 1 < REFLECTANCE_BINCOUNT && bins[troughBinMax + 1] <= bins[troughBinMax]) troughBinMax++;
    troughBinMin = troughBin;
    while (troughBinMin > 0 && bins[troughBinMin - 1] <= bins[troughBinMin]) troughBinMin--;

    value = ((troughBinMin + troughBinMax + 1) * (1024 / REFLECTANCE_BINCOUNT)) / 2;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    Serial.print(F("Trough: "));
    Serial.print(troughBinMin);
    Serial.print(F("-"));
    Serial.print(troughBinMax);
    Serial.print(F(" @ "));
    Serial.println(troughBin);
    Serial.print(F("Reflectance threshold = "));
    Serial.println(value);
#endif
#endif
    IrReflectanceSensor::setPresentThreshold(value);
    IrReflectanceSensor::setAbsentThreshold(value);
  }//!error
  else
  {
    IrReflectanceSensor::setPresentThreshold(IR_REFLECTANCE_DO_NOT_USE_THRESHOLD);
    IrReflectanceSensor::setAbsentThreshold(IR_REFLECTANCE_DO_NOT_USE_THRESHOLD);
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    Serial.println(F("Laser will be enabled full circle!"));
#endif
#endif
  }
} // findReflectanceThreshold


