/*
   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */

#pragma once

#define SOFTWARE_VERSION F("Version 2.6.0 - August 19 2020 release for 2020 kits")

/*******************
   VERSION HISTORY
 *******************
  2.6.0 - 2020-08-19: Move IR tape threshold scan to class IrThreshol; streamline initialization with single read pass to memory

  2.5.1 - 2020-08: tweak tape sensor threshold to be in middle of trough instead of 20%; tweaks to seeking behavior
  
  2.5.0 - configure for 2020 kits (no RTC, set unused pins to pullup, direct drive speed boost)

  2.3.2 - increase speed for direct drive
  
  2.3.1 - remove RTC support. Never going to be used vs. ambient light; don't need the battery-backed RAM, either (using EEPROM)

  2.3.0 - better support for Bluetooth control and tape sensor. 
  
  2.1.1 beta - issue #32 attempt to find minimum useful span and set random steps accordingly
  2.1.0.alpha - issue #18 - rework light sensor threshold setting; recheck periodically -- fix is not just software but also requires hardware change from 10k to 4.7k resistor in tape sensor.

  2.0.3 - address issue #34 - bugfixes to setting of minimum angle (oversights in earlier code)
  
  2.0.1 - fix issue #28 - servo motion stops when angle adjusted via knob; added manual servo hold timeout
  2.0.0a - disabled LASER_TOGGLE_WITH_INTERRUPT
  2.0.0 - Tweaked a few minor things

  2.0_rc2 - June 20, 2018 - release candidate 2

  2.0_rc1 - release candidate 1 - still had a bug in inverted tape sensor

  1.4_1 - June 2018 - branch feat/1_invert_tape_sensor (via #define in config.ini)
  
  1.4_0 - June 2018 - branch feat/16_bt_manual add manual control state triggered by BT connection
  
  1.3_1 - June 2018 - branch develop - adjust servo settings for Futaba 1307S
  
  1.3_0 - June 2018 - branch feat/7_rtc control of sleep/wake via RTC

  1.2_9 - June 2018 - branch feat/14_save_settings automatic save and load of settings from EEPROM

  1.2_8 - June 2018 - branch feat/8_Command
  . allow commands from Serial (USB) and Serial1 (Bluetooth)
    to change settings

  1.2_4 - June 2018 - branch feat/4_Settings
  . Gather #defines used for Settings object
    and categories others to find more easily
  . rename STEPPER_PIN_HALFSTEPPING STEPPER_PIN_MICROSTEP
    because microstep pins can be selected with jumpers on 2018 boards

  1.1.1 - May 28 2018
  . Minor clean-up for github initial release

  1.1.0 - August 11 2017
  . Complete rewrite of tape reflectance threshold calculation.
    Now saves a histogram of readings, finds two peaks and then a trough between them
    Threshold is set to midpoint of trough.
  . Gentle servo: reduced travel per interrupt; turn offf for 500ms after each goal reached
  . Gentle laser: alternate on/off with interrupt
  . Gentle servo/laser tweaks should improve part lifespan and also reduce power consumption.

  1.0.0 - July 2017 production version
  . knobs control interrupt rate, minimum servo angle, servo angle range
  . if tape is sensed, turns off laser (had been reversed)
  . light cutoff set to 350 (about 30 lux)


  unnumbered - 2017 preproduction
  . knobs control interrupt rate, minimum servo angle, servo angle range
  . no useful software support for bluetooth or clock/ram modules

  unnumbered - 2016 prototype
  . used Pro Mini
*/


/****************************
 * Default behavior configuration
 */

// overall speed:
#define INTERRUPT_FREQUENCY_DEFAULT 200
#define STEPPER_RANDOMSTEPS_MIN -50
#define STEPPER_RANDOMSTEPS_MAX 80
#define STEPPER_STEPS_WHILE_SEEKING 17
/* Ambient lux levels measured by ColorMunki -> threshold values
    Per https://en.wikipedia.org/wiki/Lux dark limit of twilight is 3.4 lux
     3 lux => 28~30  // office during thunderstorm 3pm
    16 lux => 155 // office thunderstorm; overhead lights lowest
    19 lux => 183 // lights up some
    24 lux => 219 // lights up some more
    // measured via HoldPeak HP-881C
    // when Rebecca said it was about time for the birds to be done
    130 lux => 270
*/
#define AMBIENTLIGHTSENSOR_DEFAULT_THRESHOLD 250
// Values for the servo that wiggles the laser up and down
// The angle set will range from LOW to HIGH+WIGGLE
// A new angle will be set at random every SERVO_HOLD_TIME_MS ms
#define SERVO_HOLD_TIME_MS 500
// wiggle was 6
#define SERVO_ANGLE_WIGGLE 3
// speed must not be > 255 - SERVO_ANGLE_HIGH or possible byte overflow error
#define SERVO_ANGLE_SPEED 20
#define SERVO_ANGLE_DWELL_TIME 2500L

/****************************
 * Hardware Configuration
 */
// duty cycle 30min on, 5min off (example from 50mW wide-beam green Laser Module)
#define LASER_DUTYCYCLERUNTIME 1800000
#define LASER_DUTYCYCLECOOLDOWN 300000
// Pulse time units are in µs; ranges should be determined for each model of servo used
// (Angle is too imprecise)
// 2017: MG90s purchased in 2017: control range apx 700µs to 2300µs
// 2017: 1ms to 2ms resulted in 90-degree movement
// 2017: Mounted horn on left because lower values rotate further clockwise.
// 2017: Mounted arm on horn so that 1ms points down and 2ms points back across the servo case.
// 2018: Futaba 1307S: control range apx 700µs to 2300µs
// 2018: Horn mounted on right; minimum angle = flat across top of crop; 
// 2018:   increasing / maximum angle points down into canopy
// 2018: Futaba S1307: useful range apx 560µs to 2050µs; movemen
// 2018: Futaba S1307: movement stops (before physical limits) at 550µs, 2400µs
#define SERVO_PULSE_SAFETY_MIN 550
#define SERVO_PULSE_SAFETY_MAX 2400
#define SERVO_PULSE_USABLE_MIN 560
#define SERVO_PULSE_USABLE_MAX 2050
#define SERVO_PULSE_DELTA_LIMIT 5

// based on the stepper motor:
#define STEPPER_FULLSTEPS_PER_ROTATION 200
// these adjust the rate of updates
#define SERVO_POSTSCALE_MASK 0x0f
#define SERVO_POSTSCALE_MASK_SEEKING 0xFF
// had been 0x3f
#define STEPPER_POSTSCALE_MASK 0x00
// had been 0x1f; changed to 0x02 for belt drive; 0x00 for direct drive?
#define STEPPER_POSTSCALE_MASK_SEEKING 0x00
#define STEPPER_MICROSTEPPING_DIVISOR 2
// IR Reflectance readings are done during microstepping, so readings * step per read should equal 400 (200 if done at full speed)
// in testing black tape on white bucket, mid-range reads correlate to distance from sensor: 1% @1cm; 5% @2cm; 8% @3cm
// 2020-08: now requires 2 bytes RAM for each reading; must be divisor of (STEPPER_FULLSTEPS_PER_ROTATION * STEPPER_MICROSTEPPING_DIVISOR)
#define IR_REFLECTANCE_READINGS 200
#define IR_REFLECTANCE_STEPS_PER_READ (STEPPER_FULLSTEPS_PER_ROTATION * STEPPER_MICROSTEPPING_DIVISOR / IR_REFLECTANCE_READINGS)
//these 2017 values for black tape on white bucket:
#define IR_REFLECTANCE_MID_READ_LIMIT 10
#define IR_REFLECTANCE_DEFAULT_PRESENT 550
// minimum difference in raw readings required to use reflectance:
#define IR_REFLECTANCE_RANGE_REQUIRED 100
// how often to recalibrate the refelctance readings (ms: 3600000 = 1h production; 60000 = 1min testing)
#define IR_REFLECTANCE_RECALIBRATE_MS 3600000
// Default checks for reflective tape, i.e., aluminum on black bucket.
// Set true to check for black tape on white bucket instead:
#define IR_REFLECTANCE_INVERT false
// how far through the trough from low readings to high readings should the threshold be set? (2020-08-14)
// experimentally, 72 seems to target an area in the trough with very few readings
// #define IR_REFLECTANCE_THRESHOLD_TROUGH_PERCENT 72
// better idea: upper and lower threshold; must cross far side to switch
#define IR_REFLECTANCE_UPPER_THRESHOLD_TROUGH_PERCENT 75
#define IR_REFLECTANCE_LOWER_THRESHOLD_TROUGH_PERCENT 25
// how many samples to average when finding the initial thresholds
#define IR_REFLECTANCE_SCANNING_AVERAGING 3
// Ignore minimum usable span below quarter circle
#define IR_REFLECTANCE_MINIMUM_USABLE_SPAN (STEPPER_FULLSTEPS_PER_ROTATION * STEPPER_MICROSTEPPING_DIVISOR / 4)
//when seeking for the tape, the system will make
//at most this many full rotations; if
//tape is not found within that time,
//it will ignore reflectance and operate full-circle.
#define IR_REFLECTANCE_SEEKING_ROTATION_LIMIT 2
//how often the RTC should refresh
#define RTC_REFRESH_MILLIS 2000


/***************************
 * Miscellaneous behavior defines
 * ...could be Settings, but would probably be confusing
 */
#define AMBIENTLIGHTSENSOR_READ_INTERVAL_MS 2000
#define LOOP_PROCESS_RATE 50
#define LOOP_SERIAL_OUTPUT_RATE 2000
#define LOOPLED_RATE 1500
//an out-of-range threshold value to ensure IR reflectance never is used
//DEPRECATED (use setDisabled() instead): #define IR_REFLECTANCE_DO_NOT_USE_THRESHOLD 1111
// flash or steady
//#define LASER_TOGGLE_WITH_INTERRUPT
#define INTERRUPT_FREQUENCY_MIN 20
#define INTERRUPT_FREQUENCY_MAX 150
// these thresholds will be raw values when using AnalogInput
#define INTERRUPT_FREQUENCY_KNOB_CHANGE_THRESHOLD 20
#define SERVO_PULSE_KNOB_CHANGE_THRESHOLD 20
//for testing, save settings 30000 ms (30 seconds) after last change; change to 100000 (100 seconds; we're telling them 2 minutes) for release
//that long a delay was obnoxious. Changed to 15 seconds.
#define SETTINGSOBSERVER_SAVE_AFTER_MS 15000
//exit servo manual hold after this time elapsed; probably convenient to have same as save time to indicate when the settings are saved
#define SERVO_MANUAL_HOLD_MS 15000
//laser will pulse at apx this rate in manual mode, pattern b10100000 if laser is blocked by reflectance or b01011111 if not.
#define STATE_MANUAL_LASER_PULSE_MS 150
// pattern will be right-shifted every pulse and reset when 0, so its loop length is its highest 1-bit.
#define STATE_MANUAL_LASER_PULSE_PATTERN 160

/*************
 * Software configuration
 */
// an array has to be allocated based on AMBIENTLIGHTSENSOR_READINGS_TO_AVERAGE, so maybe not a setting?
#define AMBIENTLIGHTSENSOR_READINGS_TO_AVERAGE 8
#define INTERRUPT_FREQUENCY_KNOB_READINGS_TO_AVERAGE 5
#define COMMAND_PROCESSOR_ENABLE_USB
#define COMMAND_PROCESSOR_STREAM_USB Serial
#define COMMAND_PROCESSOR_DATARATE_USB 115200
#define COMMAND_PROCESSOR_ENABLE_BLUETOOTH
#define COMMAND_PROCESSOR_STREAM_BLUETOOTH Serial1
#define COMMAND_PROCESSOR_DATARATE_BLUETOOTH 38400

/***********************
 * Arduino / pin assignments
 *
 ********************/

#define UNUSED_PIN_COUNT 4
#define UNUSED_PINS {2, 3, 4, 15}

// use 8-bit timer 2 for ATmega328 (UNO, Pro Mini)
// # define INTERRUPTS_ATmega328P_T2
// use timer 1 or 3 for ATmega32u4 (Leonardo, Micro)
// actually might need to use timer 3 to avoid conflict with servo library on Timer 1
// # define INTERRUPTS_ATmega32U4_T1
#define INTERRUPTS_ATmega32U4_T3

// for Arduino Pro Micro (Sparkfun design)

#define AMBIENTLIGHTSENSOR_PIN A0
#define KNOB1_PIN A1
#define KNOB2_PIN A2
#define KNOB3_PIN A3

#define IR_REFLECTANCE_PIN A10


// Bluetooth is on Serial1 at pins 0/1
// also have to initialize the pins in setup:
#define BT_PIN_RXD 1
#define BT_PIN_TXD 0
#define BT_PIN_STATE 5

//The stepper must be controlled via an Allegro A4988
//driver such as https://www.pololu.com/product/1182
//(Don't forget the 100uF electrolytic between VMot and its ground!)
#define STEPPER_PIN_MICROSTEP 6
#define STEPPER_PIN_SLEEP 7
#define STEPPER_PIN_STEP 8
#define STEPPER_PIN_DIR 9

#define LASER_PIN_POWER 14

#define SERVO_PIN_PULSE 16


//on the cheap knockoffs, the LEDs are illuminated when the pin is low
#define LED1_PIN LED_BUILTIN_RX
#define LED1_INVERT true
#define LED2_PIN LED_BUILTIN_TX
#define LED2_INVERT true
 
 /***************************
 * DEBUG Flags
 */
#define DEBUG_SERIAL
// serial debug data rate will use COMMAND_PROCESSOR_DATARATE_USB
#define DEBUG_SERIAL_OUTPUT_INTERVAL_MS 10000
// increase somewhat while debugging to make serial connection easier; 4 for production
#define DEBUG_SERIAL_COUNTDOWN_SECONDS 3
//#define DEBUG_LOOP_TIME
//#define DEBUG_SERVO
//#define DEBUG_KNOBS
//#define DEBUG_LIGHTSENSOR
//#define DEBUG_REFLECTANCE
#define DEBUG_REFLECTANCE_THRESHOLD
//#define DEBUG_SETTINGS
//#define DEBUG_SETTINGS_VERBOSE
//#define DEBUG_SETTINGSOBSERVER

//#define DEBUG_STEPPER
//#define DEBUG_STEPPER_STEPS
//#define DEBUG_LASERCONTROLLER
//#define DEBUG_LASER_DUTY_CYCLE
//#define DEBUG_INTERRUPT_FREQUENCY
//#define DEBUG_BLUETOOTH
