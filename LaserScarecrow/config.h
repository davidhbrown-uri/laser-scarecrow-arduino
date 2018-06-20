/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */

#pragma once

#define SOFTWARE_VERSION F("Version 1.4_0 - dev - feat/16_bt_manual")

/*******************
   VERSION HISTORY
 *******************

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

#define STEPPER_RANDOMSTEPS_MIN -70
#define STEPPER_RANDOMSTEPS_MAX 100
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

#define STEPPER_FULLSTEPS_PER_ROTATION 200
// these adjust the rate of updates
#define SERVO_POSTSCALE_MASK 0x0f
#define SERVO_POSTSCALE_MASK_SEEKING 0xFF
// had been 0x3f
#define STEPPER_POSTSCALE_MASK 0x02
// had been 0x1f
#define STEPPER_POSTSCALE_MASK_SEEKING 0x00
#define STEPPER_STEPS_PER_REVOLUTION 200
#define STEPPER_MICROSTEPPING_DIVISOR 2
// IR Reflectance readings are done during microstepping, so readings * step per read should equal 400 (200 if done at full speed)
//in testing black tape on white bucket, mid-range reads correlate to distance from sensor: 1% @1cm; 5% @2cm; 8% @3cm
#define IR_REFLECTANCE_READINGS 100
#define IR_REFLECTANCE_STEPS_PER_READ STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPPING_DIVISOR / IR_REFLECTANCE_READINGS
//these 2017 values for black tape on white bucket:
#define IR_REFLECTANCE_MID_READ_LIMIT 10
#define IR_REFLECTANCE_DEFAULT_PRESENT 550
#define IR_REFLECTANCE_DEFAULT_ABSENT 400
#define IR_REFLECTANCE_MINIMUM_CONTRAST 16
//when seeking for the tape, the system will make
//at most this many full rotations; if
//tape is not found within that time,
//it will ignore reflectance and operate full-circle.
#define SEEKING_ROTATION_LIMIT 2
// to use reflective tape on a non-reflective (black) bucket (e.g., 2018 kits),
// invert the logic of the isPresent, isAbsent tests (leave actual values alone):
#define IR_REFLECTANCE_INVERT

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
#define IR_REFLECTANCE_DO_NOT_USE_THRESHOLD 1111
// flash or steady
#define LASER_TOGGLE_WITH_INTERRUPT
#define INTERRUPT_FREQUENCY_MIN 20
#define INTERRUPT_FREQUENCY_MAX 150
// these thresholds will be raw values when using AnalogInput
#define INTERRUPT_FREQUENCY_KNOB_CHANGE_THRESHOLD 20
#define SERVO_PULSE_KNOB_CHANGE_THRESHOLD 20
//for testing, save settings 30000 ms (30 seconds) after last change; change to 300000 (5 minutes) for release
#define SETTINGSOBSERVER_SAVE_AFTER_MS 30000
#define STATE_MANUAL_LASER_OFF_DELAY_MS 15000

/*************
 * Software configuration
 */
// an array has to be allocated based on AMBIENTLIGHTSENSOR_READINGS_TO_AVERAGE, so maybe not a setting?
#define AMBIENTLIGHTSENSOR_READINGS_TO_AVERAGE 8
#define INTERRUPT_FREQUENCY_KNOB_READINGS_TO_AVERAGE 5
#define COMMAND_PROCESSOR_ENABLE_USB
#define COMMAND_PROCESSOR_STREAM_USB Serial
#define COMMAND_PROCESSOR_DATARATE_USB 57600
#define COMMAND_PROCESSOR_ENABLE_BLUETOOTH
#define COMMAND_PROCESSOR_STREAM_BLUETOOTH Serial1
#define COMMAND_PROCESSOR_DATARATE_BLUETOOTH 38400

/***********************
 * Arduino / pin assignments
 *
 ********************/

// use 8-bit timer 2 for ATmega328 (UNO, Pro Mini)
// # define INTERRUPTS_ATmega328P_T2
// use timer 1 or 3 for ATmega32u4 (Leonardo, Micro)
// actually might need to use timer 3 to avoid conflict with servo library on Timer 1
// # define INTERRUPTS_ATmega32U4_T1
#define INTERRUPTS_ATmega32U4_T3

// Pin assignments valid April 2017 through at least June 2018
// for Arduino Pro Micro (Sparkfun design)

#define AMBIENTLIGHTSENSOR_PIN A0
#define KNOB1_PIN A1
#define KNOB2_PIN A2
#define KNOB3_PIN A3

#define IR_REFLECTANCE_PIN A10

#define RTC_PIN_SDA 2
#define RTC_PIN_SCL 3

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

#define RTC_WIRE_RTC_ADDRESS 0x68
#define RTC_WIRE_EE_ADDRESS 0x57
/***************************
 * DEBUG Flags
 */
#define DEBUG_SERIAL
#define DEBUG_SERIAL_DATARATE 57600
#define DEBUG_SERIAL_OUTPUT_INTERVAL_MS 4000
#define DEBUG_SERIAL_COUNTDOWN_SECONDS 4
//#define DEBUG_SERVO
//#define DEBUG_KNOBS
//#define DEBUG_LIGHTSENSOR
#define DEBUG_REFLECTANCE
//#define DEBUG_REFLECTANCE_INIT_READINGS
//#define DEBUG_SETTINGS
//#define DEBUG_SETTINGS_VERBOSE
//#define DEBUG_SETTINGSOBSERVER

//#define DEBUG_STEPPER
//#define DEBUG_STEPPER_STEPS
//#define DEBUG_LASERCONTROLLER
//#define DEBUG_LASER_DUTY_CYCLE
//#define DEBUG_INTERRUPT_FREQUENCY
//#define DEBUG_RTC
//#define DEBUG_BLUETOOTH
