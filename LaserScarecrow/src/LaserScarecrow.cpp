/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

*/
#include <Arduino.h>
#include "config.h"
#include "StepperController.h"
#include "Settings.h"
#include "ServoController.h"
#include "LaserController.h"
#include "AmbientLightSensor.h"
#include "IrReflectanceSensor.h"
#include "IrThreshold.h"
#include "LaserController.h"
#include "AnalogInput.h"
#include "Command.h"
#include "CommandProcessor.h"
#include "SettingsObserver.h"
#include "Led.h"

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

// forward definition of functions found below
void checkSpeedKnob();
void checkServoKnobs();
void findReflectanceThreshold();
int findShortestUsableSpan();
void do_pre_laser_rotation();

/*****************
   GLOBALS
*/
bool serialCanWrite = false;
byte stateCurrent, statePrevious;
bool stateManual = false;
unsigned long manualLaserPulseMillis;
byte manualLaserPulseMask;

unsigned long stateInitReflectanceMillis;

unsigned long loopLedLastChangeMillis = 0L;
unsigned long loopProcessLastMillis = 0L;
unsigned long loopLastSerial = 0L;
unsigned long seekingMicrostepsLimit = IR_REFLECTANCE_SEEKING_ROTATION_LIMIT * STEPPER_FULLSTEPS_PER_ROTATION * STEPPER_MICROSTEPPING_DIVISOR;

#ifdef DEBUG_AIMCONTROLLER
#ifdef DEBUG_SERIAL
unsigned long aimStatusCycle = LOOP_SERIAL_OUTPUT_RATE / 80L;
unsigned long loopLastAimStatus = 0L;
#endif
#endif

bool bt_connected = false;
/*
      In classes that need to access the current settings,
      put the following near the top:

  #include "Settings.h"
  extern Settings currentSettings;

*/
Settings currentSettings;
StepperController stepper_controller;
Command uCommand, btCommand;
CommandProcessor uProcessor, btProcessor;

AnalogInput knobSpeed, knobAngleMin, knobAngleRange;

Led led1(LED1_PIN, LED1_INVERT), led2(LED2_PIN, LED2_INVERT);

#ifdef DEBUG_SERIAL
#ifdef DEBUG_LOOP_TIME
unsigned long loopCount = 0L;
#endif
#endif
/*************************************
   SETUP
*/

void setup()
{
  // put your setup code here, to run once:

  /*************************
     INITIALIZE I/O
  */

  int unused_pins[UNUSED_PIN_COUNT] = UNUSED_PINS;
  for (int i = 0; i < UNUSED_PIN_COUNT; i++)
  {
    pinMode(unused_pins[i], INPUT_PULLUP);
  }

  // Knobs
  knobSpeed.begin(KNOB1_PIN, 5, 50, STEPPER_SPEED_KNOB_CHANGE_THRESHOLD);
  knobAngleMin.begin(KNOB2_PIN, 5, 50, SERVO_PULSE_KNOB_CHANGE_THRESHOLD);
  knobAngleRange.begin(KNOB3_PIN, 5, 50, SERVO_PULSE_KNOB_CHANGE_THRESHOLD);

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
  uProcessor.setStream(&COMMAND_PROCESSOR_STREAM_USB);
#endif
#ifndef COMMAND_PROCESSOR_ENABLE_USB
#ifdef DEBUG_SERIAL
  Serial.begin(COMMAND_PROCESSOR_DATARATE_USB);
#endif
#endif

#ifdef COMMAND_PROCESSOR_ENABLE_BLUETOOTH
  pinMode(BT_PIN_STATE, INPUT); // Should be high when connected, low when not
  pinMode(BT_PIN_RXD, INPUT);   // is this going to be handled by Serial1.begin?
  pinMode(BT_PIN_TXD, OUTPUT);  // is this going to be handled by Serial1.begin?
  COMMAND_PROCESSOR_STREAM_BLUETOOTH.begin(COMMAND_PROCESSOR_DATARATE_BLUETOOTH);
  btCommand.init();
  btProcessor.setCommand(&btCommand);
  btProcessor.setSettings(&currentSettings);
  //future:  btProcessor.setConfiguration(&configuration);
  btProcessor.setStream(&COMMAND_PROCESSOR_STREAM_BLUETOOTH);
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

void loop()
{ // put your main code here, to run repeatedly:

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
    if (!bt_connected)
    {
      stateManual = true;
    }
    bt_connected = true;
  }
  else
  {
    if (bt_connected)
    {
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
  switch (stateCurrent)
  {
  /*********************
      POWERON
    *********************/
  case STATE_POWERON:
    currentSettings.init();
    stepper_controller.init();
    led1.on();
    led2.on();
// Serial was possibly initialized by the USB command processor
#ifdef DEBUG_SERIAL
    if (serialCanWrite)
      Serial.println();
    for (int i = DEBUG_SERIAL_COUNTDOWN_SECONDS; i >= 0; i--)
    {
      serialCanWrite = Serial && Serial.availableForWrite() > 16;
      if (serialCanWrite)
      {
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
    if (stateCurrent != statePrevious)
    {
      statePrevious = stateCurrent;
      // onEnter code:
      led1.off();
      led2.off();
#ifdef DEBUG_SERIAL
      if (serialCanWrite)
        Serial.println(F("\r\n[[Entering INIT State]]"));
#endif
      if (SettingsObserver::load(&currentSettings))
      {
#ifdef DEBUG_SERIAL
        if (serialCanWrite)
          Serial.println(F("Loaded settings from storage."));
#endif
        ;
      }
      else
      {
#ifdef DEBUG_SERIAL
        if (serialCanWrite)
          Serial.println(F("Could not load settings from storage."));
#endif
        ;
      }
      LaserController::init();
      ServoController::init();
      stepper_controller.init();
      IrReflectanceSensor::init();
      AmbientLightSensor::init();

    } // finish /enter code
    //update:
    stateCurrent = STATE_INIT_REFLECTANCE;
    if (stateCurrent != statePrevious)
    {
      //exit code:
      // This is part of our implementation of a pre-emission warning delay per CFR 21 J 1040.10(f)(5)(iii)
      do_pre_laser_rotation();
    }
    break;

    /*********************
     INIT_REFLECTANCE
     *********************/
  case STATE_INIT_REFLECTANCE:
    if (stateCurrent != statePrevious)
    {
      statePrevious = stateCurrent;
      //enter code:
      led1.off();
      led2.on();
#ifdef DEBUG_SERIAL
      if (serialCanWrite)
        Serial.println(F("\r\n[[Entering INIT_REFLECTANCE State]]"));
#endif
      LaserController::turnOff();
      LaserController::update(); // because this operation isn't looping
      ServoController::stop();
      stepper_controller.move_stop();
      stepper_controller.applySettings(&currentSettings);
      //laser is on here
    } //end enter code
    // do/ :
    IrThreshold::setReflectanceThreshold();
    stateCurrent = IrReflectanceSensor::isPresent() ? STATE_SEEKING : STATE_ACTIVE;
    if (stateCurrent != statePrevious)
    {
      //exit code:
      stateInitReflectanceMillis = millis();
      led2.off();
    }
    break;
  /*********************
      ACTIVE
    *********************/
  case STATE_ACTIVE:
    if (stateCurrent != statePrevious)
    {
      statePrevious = stateCurrent;
      //enter code:
      led1.flicker();
      led2.off();
#ifdef DEBUG_SERIAL
      if (serialCanWrite)
        Serial.println(F("\r\n[[Entering ACTIVE State]]"));
#endif // DEBUG_SERIAL
      stepper_controller.applySettings(&currentSettings);
      LaserController::turnOn();
      ServoController::run();
    } // end /enter behavior
    //update:
    // check for transition events (later checks have priority)
    if (millis() - stateInitReflectanceMillis > IR_REFLECTANCE_RECALIBRATE_MS)
    {
      /** @todo set stateInitReflectanceMillis in STATE_INIT_REFLECTANCE, not here */
      stateInitReflectanceMillis = millis();
      stateCurrent = STATE_INIT_REFLECTANCE;
    }
    if (IrReflectanceSensor::isPresent())
    {
      stateCurrent = STATE_SEEKING;
    }
    if (LaserController::isCoolingDown())
    {
      stateCurrent = STATE_COOLDOWN;
    }

    // do not go dark if BT is connected, issue #37
    if (!bt_connected && AmbientLightSensor::isDark())
    {
      stateCurrent = STATE_DARK;
    }
    // do our things:
    if (stepper_controller.is_stopped())
    {
      stepper_controller.move(
          random(currentSettings.stepper_randomsteps_min, currentSettings.stepper_randomsteps_max) *
          (random(0, 100) < STEPPER_TRAVEL_REVERSE_PERCENT ? -1 : 1));
    }
    ServoController::update();

    if (stateManual)
    {
      stateCurrent = STATE_MANUAL;
    }
    if (stateCurrent != statePrevious)
    {
      //exit code:
    }
    break;
  /*********************
      SEEKING
    *********************/
  case STATE_SEEKING:
    if (stateCurrent != statePrevious)
    { // onEntry
      statePrevious = stateCurrent;
      led1.flicker();
      led2.on();
#ifdef DEBUG_SERIAL
      if (serialCanWrite)
        Serial.println(F("\r\n[[Entering SEEKING State]]"));
#endif // DEBUG_SERIAL
      stepper_controller.runFullSpeed();
      LaserController::turnOff();
      ServoController::stop();
      if (!stepper_controller.is_stopped())
      {
        stepper_controller.move_extend(currentSettings.stepper_stepsWhileSeeking);
      }
    }
    //do/
    //check for transition
    if (!IrReflectanceSensor::isPresent())
    {
      stateCurrent = STATE_ACTIVE;
      // force some additional movement to avoid lots of chattering at trailing edge of tape
      // as of version 2.1.1, stepper_randomsteps_max should be half the width of the smallest usable span
      // ... but it's still getting hung up on the edge of tape, so let's go at least 1/4 of the smallest usable span out.
      //      stepper_controller.move_extend(random(currentSettings.stepper_randomsteps_max / 2, currentSettings.stepper_randomsteps_max));
    }
    //do our things:
    if (stepper_controller.is_stopped())
    {
      stepper_controller.move_extend(currentSettings.stepper_stepsWhileSeeking);
    }
    if (stepper_controller.get_steps_taken_this_move() > seekingMicrostepsLimit)
    {
#ifdef DEBUG_SERIAL
      if (serialCanWrite)
        Serial.print(F("!!! Seeking Rotation limit: "));
      Serial.print(stepper_controller.get_steps_taken_this_move());
      Serial.print(F(" steps this move exceeds "));
      Serial.println(seekingMicrostepsLimit);
#endif // DEBUG_SERIAL
      stateCurrent = STATE_INIT_REFLECTANCE;
    }
    if (stateManual)
    {
      stateCurrent = STATE_MANUAL;
    }
    if (stateCurrent != statePrevious)
    {
      stepper_controller.runSetSpeed();
    }
    break;
  /*********************
      DARK
    *********************/
  case STATE_DARK:
    if (stateCurrent != statePrevious)
    {
      statePrevious = stateCurrent;
      led1.blink(500, 9500);
      led2.off();
      //enter code:
#ifdef DEBUG_SERIAL
      if (serialCanWrite)
        Serial.println(F("\r\n[[Entering DARK State]]"));
#endif // DEBUG_SERIAL
      LaserController::turnOff();
      ServoController::stop();
      stepper_controller.turn_off();
      // TODO: slow blink rate?
    }
    //update:
    if (AmbientLightSensor::isLight())
    {
      stateCurrent = STATE_SEEKING;
    }
    if (stateManual)
    {
      stateCurrent = STATE_MANUAL;
    }
    if (stateCurrent != statePrevious)
    {
      //exit code:
      // This is part of our implementation of a pre-emission warning delay per CFR 21 J 1040.10(f)(5)(iii)
      do_pre_laser_rotation();
    }
    break;
  /*********************
      COOLDOWN
    *********************/
  case STATE_COOLDOWN:
    if (stateCurrent != statePrevious)
    {
      statePrevious = stateCurrent;
      led1.off();
      led2.on();
      led1.blink(700, 4300);
      led2.blink(200, 4800);
      //enter code:
#ifdef DEBUG_SERIAL
      if (serialCanWrite)
        Serial.println(F("\r\n[[Entering COOLDOWN State]]"));
#endif                            // DEBUG_SERIAL
      LaserController::turnOff(); // it may already have done this itself.
      //servo detach
      ServoController::stop();
      stepper_controller.turn_off();
      // TODO: moderate blink rate:
    }
    //update:
    if (!LaserController::isCoolingDown())
    {
      stateCurrent = STATE_ACTIVE;
    }
    if (stateManual)
    {
      stateCurrent = STATE_MANUAL;
    }
    if (stateCurrent != statePrevious)
    {
      //exit code:
      // This is part of our implementation of a pre-emission warning delay per CFR 21 J 1040.10(f)(5)(iii)
      do_pre_laser_rotation();
    }
    break;
    /**************************
       * MANUAL
       ************************/
  case STATE_MANUAL:
    if (stateCurrent != statePrevious)
    {
      statePrevious = stateCurrent;
      led1.blink(600, 400);
      //enter code:
#ifdef DEBUG_SERIAL
      if (serialCanWrite)
        Serial.println(F("\r\n[[Entering MANUAL State]]"));
#endif // DEBUG_SERIAL
      stepper_controller.applySettings(&currentSettings);
      LaserController::turnOff();
      stepper_controller.move_stop();
      ServoController::runManually();
      manualLaserPulseMillis = millis();
    } // enter code
    // manual behaviors mostly done by command processor
    // here, we only need to pulse the laser depending on whether tape is sensed
    if (millis() - manualLaserPulseMillis > STATE_MANUAL_LASER_PULSE_MS)
    {
      manualLaserPulseMask = manualLaserPulseMask == 0 ? STATE_MANUAL_LASER_PULSE_PATTERN : manualLaserPulseMask >> 1;
      manualLaserPulseMillis = millis();
      if (IrReflectanceSensor::isPresent())
      {
        led2.blink(1, 1);
        if ((manualLaserPulseMask & B00000001) == B00000001)
        {
          LaserController::turnOn();
        }
        else
        {
          LaserController::turnOff();
        }
      }
      else
      { // no reflectance
        led2.off();
        if ((manualLaserPulseMask & B00000001) == B00000001)
        {
          LaserController::turnOff();
        }
        else
        {
          LaserController::turnOn();
        }
      } // else no reflectance
    }   // manual mode laser pulse

    if (!stateManual)
    {
      stateCurrent = STATE_ACTIVE;
      // don't want to go to POWERON or INIT; unstored settings would be overwritten.
    }
    if (stateCurrent != statePrevious)
    {
      //exit code:
      ServoController::applySettings(&currentSettings);
    }
    break;
  default:
#ifdef DEBUG_SERIAL
    if (serialCanWrite)
      Serial.println(F("\r\n[[Unknown State requested]]"));
#endif                            // DEBUG_SERIAL
    stateCurrent = statePrevious; //or INIT? POWERON?
  }                               // switch STATE

  ///////// AFTER ANY STATE:

  // timing-imprecise tasks:
  LaserController::update();
  AmbientLightSensor::update();
  stepper_controller.update();
  led1.update();
  led2.update();

  ////////// DEBUG OUTPUT:

#ifdef DEBUG_SERIAL
#ifdef DEBUG_LOOP_TIME
  loopCount++;
#endif
  long loopsTotalDuration = millis() - loopLastSerial;
  bool outputSerialDebug = (loopsTotalDuration > DEBUG_SERIAL_OUTPUT_INTERVAL_MS);
  if (outputSerialDebug && Serial)
  {
    loopLastSerial = millis();
    Serial.println();
#ifdef DEBUG_LOOP_TIME
    Serial.print(F("Average loop duration (ms): "));
    Serial.print(loopsTotalDuration / loopCount);
    Serial.print(F("  ("));
    Serial.print(loopsTotalDuration);
    Serial.print(F(" / "));
    Serial.println(loopCount);
    loopCount = 0L;
#endif
    Serial.println(SOFTWARE_VERSION);
#ifdef DEBUG_SETTINGS_VERBOSE
    currentSettings.printToStream(&Serial);
#endif
#ifdef DEBUG_REFLECTANCE
    Serial.print(F("\r\nIR raw="));
    Serial.print(IrReflectanceSensor::read());
    if (IrReflectanceSensor::isDisabled())
    {
      Serial.println(F(" (disabled)"));
    }
    else
    {
      Serial.println(IrReflectanceSensor::isPresent() ? F(" (tape present)") : F(" (no tape)"));
    }
#endif
#ifdef DEBUG_AMBIENT
    Serial.print(F("Ambient light average="));
    Serial.print(AmbientLightSensor::read());
    Serial.print(AmbientLightSensor::isLight() ? F(" (Light)") : F(" (Dark)"));
#endif
#ifdef DEBUG_STEPPER_CONTROLLER
    if (stepper_controller.is_stopped())
    {
      Serial.println(F("StepperController is stopped"));
    }
    else
    {
      Serial.print(F("StepperController steps this move = "));
      Serial.println(stepper_controller.get_steps_taken_this_move());
    }
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

ISR(TIMER3_COMPA_vect)
{
  StepperController::isr(&stepper_controller);
#ifdef LASER_TOGGLE_WITH_INTERRUPT
  LaserController::doInterrupt();
#endif
}

/*****************
   Helper functions
   Put things here that require access to multiple modules
*/

void checkSpeedKnob()
{
  knobSpeed.process();
  if (knobSpeed.hasNewValue())
  {
    currentSettings.stepper_speed_limit_percent = map(knobSpeed.getValue(), 0, 1023, 0, 100);
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

/**
 * @brief Warning rotation before laser is enabled
 * 
 * This is part of our implementation of a pre-emission warning delay per CFR 21 J 1040.10(f)(5)(iii)
 * 
 */
void do_pre_laser_rotation()
{
  led1.flicker();
  led2.flicker();
  stepper_controller.runFullSpeed();
  for (int i = 2; i <= 4; i++)
  {
    stepper_controller.move((i / 2) * STEPPER_FULLSTEPS_PER_ROTATION * STEPPER_MICROSTEPPING_DIVISOR);
    while (!stepper_controller.is_stopped())
    {
      stepper_controller.update();
      led1.update();
      led2.update();
    }
  }
  // the long number of steps taken was messing up the seek rotation limit; this should reset it to a reasonable value
  stepper_controller.move(1);
  stepper_controller.runSetSpeed();
}