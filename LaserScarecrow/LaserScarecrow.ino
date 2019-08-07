/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

*/
#include "config.h"
#include "Settings.h"
#include "Interrupt.h"
#include "StepperController.h"
#include "ServoController.h"
#include "LaserController.h"
#include "AmbientLightSensor.h"
#include "IrReflectanceSensor.h"
#include "LaserController.h"
#include "AnalogInput.h"
#include "Command.h"
#include "CommandProcessor.h"
#include "SettingsObserver.h"
#include <Wire.h>
#include <uRTCLib.h>



// definitions for finite state machine
// STATE_CONTINUE can be used to indicate that the current state should be continued
#define STATE_CONTINUE 0
#define STATE_POWERON 10
#define STATE_INIT 20
#define STATE_INIT_REFLECTANCE 36
#define STATE_ACTIVE 30
#define STATE_SEEKING 40
#define STATE_DARK 50
#define STATE_COOLDOWN 60
#define STATE_MANUAL 99


/*****************
   GLOBALS
*/
bool serialCanWrite=false;
byte stateCurrent, statePrevious;
bool stateManual=false;
unsigned long manualLaserPulseMillis;
byte manualLaserPulseMask;

unsigned long stateInitReflectanceMillis;

unsigned long loopLedLastChangeMillis = 0L;
unsigned long loopProcessLastMillis = 0L;
unsigned long loopLastSerial = 0L;
int seekingRotationLimitCountdown;

#ifdef DEBUG_AIMCONTROLLER
#ifdef DEBUG_SERIAL
unsigned long aimStatusCycle = LOOP_SERIAL_OUTPUT_RATE / 80L;
unsigned long loopLastAimStatus = 0L;
#endif
#endif

unsigned long rtc_last_refresh_millis = 0UL;
bool rtc_is_running = false;
uint8_t rtc_last_second;

bool bt_connected = false;
/*
      In classes that need to access the current settings,
      put the following near the top:

  #include "Settings.h"
  extern Settings currentSettings;

*/
Settings currentSettings;
uRTCLib rtc(RTC_WIRE_RTC_ADDRESS, RTC_WIRE_EE_ADDRESS);
Command uCommand, btCommand;
CommandProcessor uProcessor, btProcessor;

AnalogInput knobSpeed, knobAngleMin, knobAngleRange;

/*************************************
   SETUP
*/

void setup() {
  // put your setup code here, to run once:

  /*************************
     INITIALIZE I/O
  */

  // Knobs
  knobSpeed.begin(KNOB1_PIN, 5, 50, INTERRUPT_FREQUENCY_KNOB_CHANGE_THRESHOLD);
  knobAngleMin.begin(KNOB2_PIN, 5, 50, SERVO_PULSE_KNOB_CHANGE_THRESHOLD);
  knobAngleRange.begin(KNOB3_PIN, 5, 50, SERVO_PULSE_KNOB_CHANGE_THRESHOLD);

  // pending LedController
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LED1_INVERT);
  digitalWrite(LED2_PIN, LED2_INVERT);

  // Wire and RTC
  Wire.begin();

  pinMode(BT_PIN_STATE, INPUT);

  /*********************
     Settings, Configuration, and command processors
  */
  currentSettings.init();
  SettingsObserver::init();

#ifdef COMMAND_PROCESSOR_ENABLE_USB
  COMMAND_PROCESSOR_STREAM_USB.begin(COMMAND_PROCESSOR_DATARATE_USB);
  uCommand.init();
  uProcessor.setCommand(&uCommand);
  uProcessor.setSettings(&currentSettings);
  //future:  uProcessor.setConfiguration(&configuration);
  //future:  uProcessor.setRTC(&rtc);
  uProcessor.setStream(& COMMAND_PROCESSOR_STREAM_USB);
#endif
#ifdef COMMAND_PROCESSOR_ENABLE_BLUETOOTH
  pinMode(BT_PIN_STATE, INPUT);// Should be high when connected, low when not
  pinMode(BT_PIN_RXD, INPUT); // is this going to be handled by Serial1.begin?
  pinMode(BT_PIN_TXD, OUTPUT);// is this going to be handled by Serial1.begin?
  COMMAND_PROCESSOR_STREAM_BLUETOOTH.begin(COMMAND_PROCESSOR_DATARATE_BLUETOOTH);
  btCommand.init();
  btProcessor.setCommand(&btCommand);
  btProcessor.setSettings(&currentSettings);
  //future:  btProcessor.setConfiguration(&configuration);
  //future:  btProcessor.setRTC(&rtc);
  btProcessor.setStream(& COMMAND_PROCESSOR_STREAM_BLUETOOTH);
#ifdef DEBUG_BLUETOOTH
  COMMAND_PROCESSOR_STREAM_BLUETOOTH.println(SOFTWARE_VERSION);
#endif
#endif
  /********************************
     Initialize state machine for loop
  */
  stateCurrent = STATE_POWERON;
  statePrevious = stateCurrent;
  stateManual = false;
} // end setup

/****************
   LOOP
*/

void loop() { // put your main code here, to run repeatedly:

  /////// BEFORE any STATE

#ifdef DEBUG_SERIAL
  serialCanWrite = Serial && Serial.availableForWrite() > 16;
#endif
  /// Check for settings changes:
#ifdef COMMAND_PROCESSOR_ENABLE_USB
  uProcessor.process();
#endif
#ifdef COMMAND_PROCESSOR_ENABLE_BLUETOOTH
  btProcessor.process();
#endif
  checkServoKnobs();
  checkSpeedKnob();
  SettingsObserver::process(); // apply any changes to settings

  // if the Bluetooth state has changed, enter or leave the manual control state
  if (digitalRead(BT_PIN_STATE))
  {
    if (!bt_connected) {
      stateManual = true;
    }
    bt_connected = true;
  }
  else {
    if (bt_connected) {
      stateManual = false;
    }
    bt_connected = false;
  }
  /*
    Simple Finite State Machine: arrange each switch case as follows...
    case STATE_WHATEVER:
      if(stateCurrent!=statePrevious)
      {
        statePrevious=stateCurrent;
        // entry code here:
        // must include setting the interrupt (or applying settings) if stepper/servo/laser is of any concern
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
    /*********************
      POWERON
    *********************/
    case STATE_POWERON:
      currentSettings.init();
      StepperController::init();
      StepperController::stop();
#ifdef DEBUG_SERIAL
      Serial.begin(DEBUG_SERIAL_DATARATE);
      if (serialCanWrite) Serial.println();
      for (int i = DEBUG_SERIAL_COUNTDOWN_SECONDS; i >= 0; i--) {
        serialCanWrite = Serial && Serial.availableForWrite() > 16;
        if(serialCanWrite) {
          Serial.print(F("Laser Scarecrow Startup in "));
          Serial.print(i);
          Serial.println(F("..."));
        }
        delay(1000);
      }
#endif // DEBUG_SERIAL
      stateCurrent = STATE_INIT;
      break;

    /*********************
      INIT
    *********************/
    case STATE_INIT:
      if (stateCurrent != statePrevious) {
        statePrevious = stateCurrent;
        // onEnter code:
#ifdef DEBUG_SERIAL
        if (serialCanWrite) Serial.println(F("\r\n[[Entering INIT State]]"));
#endif 
        if (SettingsObserver::load(&currentSettings))
        {
#ifdef DEBUG_SERIAL
          if (serialCanWrite) Serial.println(F("Loaded settings from storage."));
#endif
          ;
        }
        else
        {
#ifdef DEBUG_SERIAL
          if (serialCanWrite) Serial.println(F("Could not load settings from storage."));
#endif
          ;
        }
        LaserController::init();
        ServoController::init();
        StepperController::init();
        IrReflectanceSensor::init();
        AmbientLightSensor::init();
        Interrupt::init();
  // check if clock is running
  rtc.refresh();
  rtc_last_second = rtc.second();
  delay(1200); 
  rtc.refresh();
  rtc_is_running = (rtc_last_second != rtc.second());
#ifdef DEBUG_SERIAL
#ifdef DEBUG_RTC
  if (serialCanWrite) {
    Serial.print(F("RTC at startup is "));
    Serial.println(rtc_is_running ? F("running") : F("STOPPED"));
  }
#endif
#endif      
}// finish enter code
      //update:
      stateCurrent = STATE_INIT_REFLECTANCE;
      if (stateCurrent != statePrevious) {
        //exit code:
      }
      break;

    /*********************
     INIT_REFLECTANCE
     *********************/
     case STATE_INIT_REFLECTANCE:
      if (stateCurrent != statePrevious) {
        statePrevious = stateCurrent;
        //enter code:
#ifdef DEBUG_SERIAL
        if (serialCanWrite) Serial.println(F("\r\n[[Entering INIT_REFLECTANCE State]]"));
#endif 
        LaserController::turnOff();
        LaserController::update();// because this operation isn't looping
        ServoController::stop();
        StepperController::stop();
        Interrupt::applySettings(& currentSettings);
        //laser is on here
      }//end enter code
      // do/ :
      findReflectanceThreshold();
      stateCurrent = STATE_DARK;
      if (stateCurrent != statePrevious) {
        //exit code:
        stateInitReflectanceMillis = millis();
      }
      break;
    /*********************
      ACTIVE
    *********************/
    case STATE_ACTIVE:
      if (stateCurrent != statePrevious) {
        statePrevious = stateCurrent;
        //enter code:
#ifdef DEBUG_SERIAL
        if (serialCanWrite) Serial.println(F("\r\n[[Entering ACTIVE State]]"));
#endif // DEBUG_SERIAL
        Interrupt::applySettings(& currentSettings);
        LaserController::turnOn();
        ServoController::run();
        StepperController::runHalfstep();
        StepperController::setStepsToStepRandom(
          currentSettings.stepper_randomsteps_min,
          currentSettings.stepper_randomsteps_max);
      }
      //update:
      // check for transition events (later checks have priority)
      if (millis()-stateInitReflectanceMillis > IR_REFLECTANCE_RECALIBRATE_MS) {
        stateInitReflectanceMillis=millis();
        stateCurrent = STATE_INIT_REFLECTANCE;
      }
      if (IrReflectanceSensor::isPresent()) stateCurrent = STATE_SEEKING;
      if (LaserController::isCoolingDown()) stateCurrent = STATE_COOLDOWN;

      // do not go dark if BT is connected, issue #37
      if (!bt_connected && rtc_is_running && currentSettings.rtc_control)
      {
        if (!rtcIsWakePeriod()) stateCurrent = STATE_DARK;
      }
      else
      {
        if (!bt_connected && AmbientLightSensor::isDark()) stateCurrent = STATE_DARK;
      }
      // do our things:
      if (StepperController::getStepsToStep() == 0) StepperController::setStepsToStepRandom(STEPPER_RANDOMSTEPS_MIN, STEPPER_RANDOMSTEPS_MAX);
      if (stateManual) {
        stateCurrent = STATE_MANUAL;
      }
      if (stateCurrent != statePrevious) {
        //exit code:
      }
      break;
    /*********************
      SEEKING
    *********************/
    case STATE_SEEKING:
      if (stateCurrent != statePrevious) { // onEntry
        statePrevious = stateCurrent;
#ifdef DEBUG_SERIAL
        if (serialCanWrite) Serial.println(F("\r\n[[Entering SEEKING State]]"));
#endif // DEBUG_SERIAL
        Interrupt::applySettings(& currentSettings);
        LaserController::turnOff();
        ServoController::stop();
        StepperController::runFullstep();
        StepperController::setStepsToStep(currentSettings.stepper_stepsWhileSeeking);
        seekingRotationLimitCountdown = IR_REFLECTANCE_SEEKING_ROTATION_LIMIT * STEPPER_FULLSTEPS_PER_ROTATION / currentSettings.stepper_stepsWhileSeeking;
      }
      //update:
      //check for transition
      if (!IrReflectanceSensor::isPresent())
      {
        stateCurrent = STATE_ACTIVE;
        // force some forward movement to avoid lots of chattering at trailing edge of tape
        // as of version 2.1.1, stepper_randomsteps_max should be half the width of the smallest usable span
        StepperController::setStepsToStep(random(currentSettings.stepper_randomsteps_max,currentSettings.stepper_randomsteps_max*1.5));
      }
      //do our things:
      if (StepperController::getStepsToStep() == 0)
      {
        StepperController::setStepsToStep(currentSettings.stepper_stepsWhileSeeking);
        seekingRotationLimitCountdown--;
      }
      if (seekingRotationLimitCountdown == 0)
      {
        stateCurrent = STATE_INIT_REFLECTANCE;
      }
      if (stateManual) {
        stateCurrent = STATE_MANUAL;
      }
      if (stateCurrent != statePrevious) {
        //exit code:
      }
      break;
    /*********************
      DARK
    *********************/
    case STATE_DARK:
      if (stateCurrent != statePrevious) {
        statePrevious = stateCurrent;
        //enter code:
#ifdef DEBUG_SERIAL
        if (serialCanWrite) Serial.println(F("\r\n[[Entering DARK State]]"));
#endif // DEBUG_SERIAL
        LaserController::turnOff();
        ServoController::stop();
        StepperController::stop();
        // might save a bit of power, but mostly wanting a slow blink rate:
        Interrupt::setFrequency(1);
      }
      //update:
      if (rtc_is_running && currentSettings.rtc_control)
      {
        if (rtcIsWakePeriod()) stateCurrent = STATE_SEEKING;
      }
      else
      {
        if (AmbientLightSensor::isLight()) stateCurrent = STATE_SEEKING;
      }
      if (stateManual) {
        stateCurrent = STATE_MANUAL;
      }
      if (stateCurrent != statePrevious) {
        //exit code:
      }
      break;
    /*********************
      COOLDOWN
    *********************/
    case STATE_COOLDOWN:
      if (stateCurrent != statePrevious) {
        statePrevious = stateCurrent;
        //enter code:
#ifdef DEBUG_SERIAL
        if (serialCanWrite) Serial.println(F("\r\n[[Entering COOLDOWN State]]"));
#endif // DEBUG_SERIAL
        LaserController::turnOff(); // it may already have done this itself.
        //servo detach
        ServoController::stop();
        StepperController::runHalfstep();
        StepperController::setStepsToStep(0);
        // might save a bit of power, but mostly wanting a moderate blink rate:
        Interrupt::setFrequency(3);
      }
      //update:
      if (!LaserController::isCoolingDown())
      {
        stateCurrent = STATE_ACTIVE;
      }
      if (stateManual) {
        stateCurrent = STATE_MANUAL;
      }
      if (stateCurrent != statePrevious) {
        //exit code:
      }
      break;
      /**************************
       * MANUAL
       ************************/
    case STATE_MANUAL:
      if (stateCurrent != statePrevious) {
        statePrevious = stateCurrent;
        //enter code:
#ifdef DEBUG_SERIAL
        if (serialCanWrite) Serial.println(F("\r\n[[Entering MANUAL State]]"));
#endif // DEBUG_SERIAL
        Interrupt::applySettings(& currentSettings);
        LaserController::turnOff();
        StepperController::runHalfstep();
        StepperController::setStepsToStep(0);
        ServoController::runManually();
        manualLaserPulseMillis=millis();
      } // enter code
      // manual behaviors mostly done by command processor
      // here, we only need to pulse the laser depending on whether tape is sensed
      if(millis()-manualLaserPulseMillis > STATE_MANUAL_LASER_PULSE_MS) {
        manualLaserPulseMask = manualLaserPulseMask==0 ? STATE_MANUAL_LASER_PULSE_PATTERN : manualLaserPulseMask >> 1;
        manualLaserPulseMillis = millis();
        if(IrReflectanceSensor::isPresent()) {
          if(manualLaserPulseMask & B00000001 == B00000001) {
            LaserController::turnOn();
          } else {
            LaserController::turnOff();
          }
        } else { // no reflectance
          if(manualLaserPulseMask & B00000001 == B00000001) {
            LaserController::turnOff();
          } else {
            LaserController::turnOn();
          }          
        } // else no reflectance
      } // manual mode laser pulse
      
      if (!stateManual) {
        stateCurrent = STATE_ACTIVE;
        // don't want to go to POWERON or INIT; unstored settings would be overwritten.
      }
      if (stateCurrent != statePrevious) {
        //exit code:
        ServoController::applySettings(& currentSettings);
      }
      break;
    default:
#ifdef DEBUG_SERIAL
      if (serialCanWrite) Serial.println(F("\r\n[[Unknown State requested]]"));
#endif // DEBUG_SERIAL
      stateCurrent = statePrevious; //or INIT? POWERON?
  } // switch STATE

  ///////// AFTER ANY STATE:

  // timing-imprecise tasks:
  LaserController::update();
  AmbientLightSensor::update();
  if (millis() - rtc_last_refresh_millis > RTC_REFRESH_MILLIS)
  {
    rtc.refresh();
    rtc_last_refresh_millis = millis();
    rtc_is_running = ( rtc.second() != rtc_last_second);
    rtc_last_second = rtc.second();
  }
  digitalWrite(LED2_PIN, IrReflectanceSensor::isPresent() ^ LED2_INVERT);

  ////////// DEBUG OUTPUT:

#ifdef DEBUG_SERIAL
  bool outputSerialDebug = (millis() - loopLastSerial > DEBUG_SERIAL_OUTPUT_INTERVAL_MS);
  if (outputSerialDebug && Serial)
  {
    loopLastSerial = millis();
    Serial.println();
    Serial.print(SOFTWARE_VERSION);
#ifdef DEBUG_SETTINGS_VERBOSE
    currentSettings.printToStream(&Serial);
#endif
#ifdef DEBUG_REFLECTANCE
    Serial.print(F("\r\nIR raw="));
    Serial.print(IrReflectanceSensor::read());
    if(IrReflectanceSensor::isDisabled()) {
      Serial.println(F(" (disabled)"));
    } else
    {
      Serial.println(IrReflectanceSensor::isPresent() ? F(" (tape present)") : F(" (no tape)"));
    }
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
    Serial.print(F("\r\nServoController::pulse="));
    Serial.print(ServoController::getPulse());
    Serial.print(F("; target="));
    Serial.print(ServoController::getPulseTarget());
    Serial.print(F("; rangeMin="));
    Serial.print(ServoController::getPulseRangeMin());
    Serial.print(F("; rangeMax="));
    Serial.print(ServoController::getPulseRangeMax());
#endif
#ifdef DEBUG_KNOBS
    Serial.print(F("\r\nKNOB1="));
    Serial.print(analogRead(KNOB1_PIN));
    Serial.print(F("; KNOB2="));
    Serial.print(analogRead(KNOB2_PIN));
    Serial.print(F("; KNOB3="));
    Serial.print(analogRead(KNOB3_PIN));
#endif
#ifdef DEBUG_RTC
    rtc.refresh();

//    Serial.print(F("\r\nRTC DateTime: "));
//    Serial.print(rtc.year());
//    Serial.print('/');
//    Serial.print(rtc.month());
//    Serial.print('/');
//    Serial.print(rtc.day());

//    Serial.print(' ');
// really we care only about the time:
    Serial.print(F("\r\nRTC time: "));
    Serial.print(rtc.hour());
    Serial.print(':');
    Serial.print(rtc.minute());
    Serial.print(':');
    Serial.print(rtc.second());
    Serial.print(F("\r\n  wake/sleep cycle mode = "));
    Serial.print(currentSettings.rtc_control ? F("RTC") : F("sensor"));
    Serial.print(F("; if RTC control, wake at "));
    Serial.print(currentSettings.rtc_wake / 60);
    Serial.print(':');
    Serial.print(currentSettings.rtc_wake % 60);
    Serial.print(F(" and sleep at "));
    Serial.print(currentSettings.rtc_sleep / 60);
    Serial.print(':');
    Serial.println(currentSettings.rtc_sleep % 60);
#endif
#ifdef DEBUG_BLUETOOTH
    Serial.print(F("\r\nBluetooth state: "));
    Serial.println(digitalRead(BT_PIN_STATE));
#endif

    Serial.println();
  } //output serial debug
#endif
} // END main Arduino loop


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

/** considers cycle mode and RTC
    True if RTC settings indicate it's a wake period

*/
bool rtcIsWakePeriod()
{
  int timeInt = rtc.hour();
  if (timeInt < 0 || timeInt > 23) // invalid values indicate a problem with the RTC, so let's be awake
  {
    return true;
  }
  timeInt *= 60;
  timeInt += rtc.minute();
  return (timeInt > currentSettings.rtc_wake) != (timeInt > currentSettings.rtc_sleep) != (currentSettings.rtc_wake > currentSettings.rtc_sleep);
}

void checkSpeedKnob()
{
  knobSpeed.process();
  if (knobSpeed.hasNewValue())
  {
    currentSettings.interrupt_frequency = map(knobSpeed.getValue(), 0, 1023, INTERRUPT_FREQUENCY_MIN, INTERRUPT_FREQUENCY_MAX);
    knobSpeed.acknowledgeNewValue();
    // Interrupt::applySettings(& currentSettings); // won't need once we have the SettingsObserver
#ifdef DEBUG_SERIAL
#ifdef DEBUG_INTERRUPT_FREQUENCY
    Serial.print(F("\n\r+++Changing frequency via knob. New freq = "));
    Serial.print(currentSettings.interrupt_frequency);
#endif
#endif
  }
}
void checkServoKnobs()
{
  knobAngleMin.process();
  knobAngleRange.process();
  if (knobAngleMin.hasNewValue() || knobAngleRange.hasNewValue())
  {
    knobAngleMin.acknowledgeNewValue();
    knobAngleRange.acknowledgeNewValue();
    int pulseLow = map(knobAngleMin.getValue(), 0, 1023, SERVO_PULSE_USABLE_MIN, SERVO_PULSE_USABLE_MAX);
    int pulseHigh = map(knobAngleRange.getValue(), 0, 1023, pulseLow, SERVO_PULSE_USABLE_MAX);
    currentSettings.servo_min = pulseLow;
    currentSettings.servo_max = pulseHigh;
    //let SettingsObserver do this: ServoController::applySettings(& currentSettings);
  }
}

// Version 1.1 attempts to find bimodal peaks and extents:
void findReflectanceThreshold() {
  int bin, value, highest = 0, lowest = 1023, maxCount=-1;
  int peakOneBin, peakOneBinMax, peakOneBinMin;
  int peakTwoBin, peakTwoBinMax, peakTwoBinMin;
  int troughBinMin, troughBinMax;
#ifdef DEBUG_REFLECTANCE
  const char hexcode[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
#endif
  bool error = false;
#define REFLECTANCE_BINCOUNT 32
  int bins[REFLECTANCE_BINCOUNT];
  for (int i = 0; i < REFLECTANCE_BINCOUNT; bins[i++] = 0);
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_INIT_READINGS
  if (serialCanWrite) Serial.println(F("\r\nFinding reflectance threshold:"));
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
  if ((highest - lowest) > IR_REFLECTANCE_RANGE_REQUIRED)
  {
    error = true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    if (serialCanWrite) {
      Serial.println(F("Reflectance cannot be used: readings too variable without movement (failed sensor?)"));
      Serial.print(F("Low-High = ")); Serial.print(lowest); Serial.print(F("-")); Serial.println(highest);
    }
#endif
#endif
  }
  // FIND RANGE FULL STEPS
  if (! error)
  {
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
      if (serialCanWrite) Serial.println(F("Full-step rotation to find reflectance range..."));
#endif
#endif
    highest = 0; lowest = 1023;
    StepperController::setStepsToStep(0);
    StepperController::runFullstep();

    for (int rotations = 0; rotations < IR_REFLECTANCE_SEEKING_ROTATION_LIMIT && (highest - lowest) <  IR_REFLECTANCE_RANGE_REQUIRED; rotations++) {
    for (int i = 0; i < STEPPER_FULLSTEPS_PER_ROTATION / 4; i++) {
      value = IrReflectanceSensor::read();
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
#ifdef DEBUG_REFLECTANCE_INIT_READINGS
      if (serialCanWrite) Serial.println(value);
#else
      if (serialCanWrite) Serial.print(hexcode[value / 64]);
#endif
#endif
#endif
      if (value > highest) {
        highest = value;
      }
      if (value < lowest) {
        lowest = value;
      }
      StepperController::setStepsToStep(-4);// move backwards to make this behavior easier to see/understand
      while (StepperController::getStepsToStep() != 0) { ; }
    } // steps while fullstepping around
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    if (serialCanWrite) { Serial.print(F("Reflectance range finding completed rotation "));
    Serial.println(rotations+1);
    }
#endif
#endif
    } // rotations
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
if(serialCanWrite){
#ifndef DEBUG_REFLECTANCE_INIT_READINGS
      Serial.println();
#endif
    Serial.print(F("Reflectance readings range from "));
    Serial.print(lowest);
    Serial.print(F(" to "));
    Serial.print(highest);
    Serial.println(F("."));
}
#endif
#endif
    if (highest - lowest < IR_REFLECTANCE_RANGE_REQUIRED)
    {
      error = true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
      if (serialCanWrite) Serial.println(F("Reflectance cannot be used: insufficient contrast!"));
#endif
#endif
    }
  }// if !error

  // BUILD HISTOGRAM MICROSTEPPING
  if (! error)
  {
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
      if (serialCanWrite) Serial.println(F("Microstep rotation to build reflectance histogram..."));
#endif
#endif
    StepperController::setStepsToStep(0);
    StepperController::runHalfstep();
    for (int i = 0; i < IR_REFLECTANCE_READINGS; i++) {
      value = IrReflectanceSensor::read();
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
#ifdef DEBUG_REFLECTANCE_INIT_READINGS
      if (serialCanWrite) Serial.println(value);
#else
      if (serialCanWrite) Serial.print(hexcode[value / 64]);
#endif
#endif
#endif
      bin = (value - lowest) / ((highest-lowest)/ REFLECTANCE_BINCOUNT); // issue #18 - consider only active range?
//      bin = value / (1024 / REFLECTANCE_BINCOUNT);
      bin = constrain(bin,0,REFLECTANCE_BINCOUNT-1);
      //smooth the histogram by adding 2 to this bin and 1 to adjacent bins (if they exist)
      bins[bin] += 2;
      if (bin >= 1) bins[bin - 1]++;
      if (bin + 1 < REFLECTANCE_BINCOUNT) bins[bin + 1]++;
      // move to next reading
      StepperController::setStepsToStep(IR_REFLECTANCE_STEPS_PER_READ);
      while (StepperController::getStepsToStep() != 0) { ; }//wait for it
    }
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
if(serialCanWrite) {
    Serial.println();
    Serial.println(F("Bins:"));
    for (int i = 0; i < REFLECTANCE_BINCOUNT; i++)
    {
      Serial.print(i);
      Serial.print(F(" ("));
      Serial.print(i*((highest-lowest)/ REFLECTANCE_BINCOUNT) + lowest);
      Serial.print(F("): "));
      Serial.println(bins[i]);
    }
}
#endif
#endif
    // find highest peak
    maxCount = -1;
    for (int i = 0; i < REFLECTANCE_BINCOUNT; i++)
    {
      if (bins[i] > maxCount) {
        maxCount = bins[i];
        peakOneBin = i;
      }
    }
    // find extent of first peak
    // consider a bin to be part of a peak if its count is > 1/3 of the peak-side neighbor.
    peakOneBinMax = peakOneBin;
    while (peakOneBinMax + 1 < REFLECTANCE_BINCOUNT && bins[peakOneBinMax + 1] > (1*bins[peakOneBinMax])/3 ) { peakOneBinMax++; }
    peakOneBinMin = peakOneBin;
    while (peakOneBinMin > 0 && bins[peakOneBinMin - 1] > (1*bins[peakOneBinMin])/3) { peakOneBinMin--; }
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
if(serialCanWrite){
    Serial.println();
    Serial.print(F("Peak One: from bins"));
    Serial.print(peakOneBinMin);
    Serial.print(F("-"));
    Serial.print(peakOneBinMax);
    Serial.print(F(" with peak @ "));
    Serial.println(peakOneBin);
}
#endif
#endif
    if (peakOneBinMin == 0 && peakOneBinMax + 1 == REFLECTANCE_BINCOUNT)
    {
      error = true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
      if (serialCanWrite) Serial.println(F("Reflectance histogram peak covers full range; cannot sense tape."));
#endif
#endif
    }

  }// if !error
  if (!error) {
    // find peak in remaining range
    maxCount = -1; 
    for (int i = 0; i < REFLECTANCE_BINCOUNT; i++)
    {
      if (i < peakOneBinMin || i > peakOneBinMax) // skip peakOne's range
      {
        if (bins[i] > maxCount) {
          maxCount = bins[i];
          peakTwoBin = i;
        }
      } // skip peakOne
    } // for i in bins
    // find extent of second peak
    // consider a bin to be part of a peak if its count is > 1/3 of the peak-side neighbor.
    peakTwoBinMax = peakTwoBin;
    while (peakTwoBinMax + 1 < REFLECTANCE_BINCOUNT && bins[peakTwoBinMax + 1] > (1*bins[peakTwoBinMax])/3) peakTwoBinMax++;
    peakTwoBinMin = peakTwoBin;
    while (peakTwoBinMin > 0 && bins[peakTwoBinMin - 1] > (1*bins[peakTwoBinMin])/3) peakTwoBinMin--;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
if(serialCanWrite) {
    Serial.print(F("Peak Two: covers bins "));
    Serial.print(peakTwoBinMin);
    Serial.print(F("-"));
    Serial.print(peakTwoBinMax);
    Serial.print(F(" with peak at @ "));
    Serial.println(peakTwoBin);
}
#endif
#endif
    if (bins[peakTwoBin] == 0)
    {
      error = true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
      if (serialCanWrite) Serial.println(F("No observations outside range of first peak; cannot sense tape."));
#endif
#endif
    }
  } // !error
  if (!error)
  {
    //find trough between peaks
    if (peakOneBin < peakTwoBin)
    {
      troughBinMin = peakOneBinMax+1;
      troughBinMax = peakTwoBinMin-1;
    }
    else
    {
      troughBinMin = peakTwoBinMax+1;
      troughBinMax = peakOneBinMin-1;
    }
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
if(serialCanWrite) {
    Serial.print(F("Trough: bins "));
    Serial.print(troughBinMin);
    Serial.print(F("-"));
    Serial.println(troughBinMax);
}
#endif
#endif
    if (troughBinMax <= troughBinMin) {
      error=true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
if(serialCanWrite) {
    Serial.print(F("Peaks overlap; cannot reliably separate high from low reflectance."));
}
#endif
#endif
    } else { // can calculate threshold between peaks
    //was middle of the trough: value = reflectanceMin + ((troughBinMin + troughBinMax + 1) * (reflectanceRange / REFLECTANCE_BINCOUNT)) / 2;
    //let's try 20% (1/5) into the trough:
    value = lowest + troughBinMin * ((highest-lowest) / REFLECTANCE_BINCOUNT) + (((troughBinMax) * ((highest-lowest) / REFLECTANCE_BINCOUNT))/5);
    IrReflectanceSensor::setPresentThreshold(value);
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
if(serialCanWrite) {
    Serial.print(F("Reflectance threshold = "));
    Serial.println(value);
}
#endif
#endif
    }//else peaks do not overlap
  }//!error
  IrReflectanceSensor::setDisabled(error);
  if (!error) {
    // find shortest usable span (issue #32)
      int span = findShortestUsableSpan();
      if(span>0) {
        currentSettings.stepper_randomsteps_max = span / 2; // half the smallest span forwards
        currentSettings.stepper_randomsteps_min = span * -5 / 7; // 70% of that backwards
      }
  }//!error 
  if(error) {
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    if (serialCanWrite) {Serial.println(F("Laser will be enabled full circle!"));}
#endif
#endif
  }
} // findReflectanceThreshold

// attempts to find the shortest usable span to better set random stepping
// if returned value <=0, then do not set stepping range; unable to determine appropriate value
// See github issue #23
int findShortestUsableSpan() {
  // MATCH_STEPS determined experimentally to help ignore jitter in readings
#define MATCH_STEPS 10
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    if(serialCanWrite) { Serial.println(F("Looking for shortest usable span")); }
#endif
#endif
    int stepsInCircle = STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPPING_DIVISOR;
    int shortestSpan = STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPPING_DIVISOR; //full circle
    bool currentStepIsPresent, previousStepIsPresent = IrReflectanceSensor::isPresent();
    //first, find some tape, moving forward
    long stepsStepped = 0L;
    int matchingStepCount = 0;
    while(matchingStepCount < MATCH_STEPS && stepsStepped < STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPPING_DIVISOR) {
      StepperController::setStepsToStep(1);
      while (StepperController::getStepsToStep() != 0) { ; }//wait for it
      stepsStepped++;
      currentStepIsPresent=IrReflectanceSensor::isPresent();
      if(currentStepIsPresent && previousStepIsPresent) { matchingStepCount++; } else { matchingStepCount=0; }
      previousStepIsPresent=currentStepIsPresent;
    }
    if (stepsStepped > stepsInCircle) return -1; //could not find tape
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    if(serialCanWrite) { Serial.println(F("Found tape")); }
#endif
#endif
    //second, find the end of the tape, moving backward (to make behavior easier to recognize)
    stepsStepped = 0L;
    matchingStepCount = 0;
    while(matchingStepCount < MATCH_STEPS && stepsStepped < stepsInCircle) {
      StepperController::setStepsToStep(-1);
      while (StepperController::getStepsToStep() != 0) { ; }//wait for it
      stepsStepped++;
      currentStepIsPresent=IrReflectanceSensor::isPresent();
      if(!currentStepIsPresent && !previousStepIsPresent) { matchingStepCount++; } else { matchingStepCount=0; }
      previousStepIsPresent=currentStepIsPresent;
    }
    if (stepsStepped > stepsInCircle) return -2; //could not find end of tape
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    if(serialCanWrite) { Serial.println(F("Found end of tape")); }
#endif
#endif
   bool inUsableSpan=true;
   int spanLength = matchingStepCount;// we already saw this many usable steps
   //cover the rest of the circle, still moving backward
   matchingStepCount = 0;
   for(int i = spanLength; i < stepsInCircle; i++) {
      StepperController::setStepsToStep(-1);
      while (StepperController::getStepsToStep() != 0) { ; }//wait for it
      currentStepIsPresent=IrReflectanceSensor::isPresent();
      if(currentStepIsPresent == previousStepIsPresent) { matchingStepCount++; } else {matchingStepCount = 0; }
      previousStepIsPresent=currentStepIsPresent;
      if(inUsableSpan) {
        if(currentStepIsPresent && matchingStepCount >= MATCH_STEPS) { // back to tape
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    if(serialCanWrite) { Serial.print(F("Found end of span with length = ")); Serial.println(spanLength); }
#endif
#endif
      if(spanLength>2*MATCH_STEPS) { // span counts toward shortest unless very short
        shortestSpan=min(shortestSpan,spanLength);
      }
      inUsableSpan=false;
      } // found tape
      else {
        spanLength++;
      }// still in usable span
   } // if inUsableSpan
   else { // on tape
    if(!currentStepIsPresent && matchingStepCount >= 2) { // back to span
      spanLength=matchingStepCount;
      inUsableSpan=true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    if(serialCanWrite) { Serial.print(F("Found beginning of another span")); Serial.println(spanLength); }
#endif
#endif    
      } // if we found another span
    } // else not in span
   }//for steps around the circle
  if(inUsableSpan) { // shouldn't happen at end of circle, but maybe?
    shortestSpan=min(shortestSpan,spanLength);
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    if(serialCanWrite) { Serial.print(F("Ended on span with length = ")); Serial.println(spanLength); }
#endif
#endif
  }
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    if(serialCanWrite) { Serial.print(F("Shortest meaningful span is = ")); Serial.println(shortestSpan); }
#endif
#endif
  return shortestSpan;
} // function findShortestUsableSpan
