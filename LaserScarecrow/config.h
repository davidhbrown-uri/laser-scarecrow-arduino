#pragma once

#define SOFTWARE_VERSION F("Version 1.1.1 - slow servo, flicker laser, bimodal threshold")

/*******************
   VERSION HISTORY
 *******************

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

// use 8-bit timer 2 for ATmega328 (UNO, Pro Mini)
// # define INTERRUPTS_ATmega328P_T2
// use timer 1 or 3 for ATmega32u4 (Leonardo, Micro)
// actually might need to use timer 3 to avoid conflict with servo library on Timer 1
// # define INTERRUPTS_ATmega32U4_T1
#define INTERRUPTS_ATmega32U4_T3

// Pin assignments April 2017 for
// Arduino Pro Micro (Sparkfun design)

#define RTC_PIN_SDA 2
#define RTC_PIN_SCL 3

#define KNOB1_PIN A1
#define KNOB2_PIN A2
#define KNOB3_PIN A3

#define LASER_PIN_POWER 14


//on the cheap knockoffs, the LEDs are illuminated when the pin is low
#define LED1_PIN LED_BUILTIN_RX
#define LED1_INVERT true
#define LED2_PIN LED_BUILTIN_TX
#define LED2_INVERT true

#define LOOP_PROCESS_RATE 50
#define LOOP_SERIAL_OUTPUT_RATE 2000
#define LOOPLED_RATE 1500

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
#define AMBIENTLIGHTSENSOR_PIN A0
#define AMBIENTLIGHTSENSOR_READ_INTERVAL_MS 2000
#define AMBIENTLIGHTSENSOR_READINGS_TO_AVERAGE 8

// duty cycle 30min on, 5min off (example from 50mW wide-beam green Laser Module)
#define LASER_DUTYCYCLERUNTIME 1800000
#define LASER_DUTYCYCLECOOLDOWN 300000
#define LASER_TOGGLE_WITH_INTERRUPT

//The stepper must be controlled via an Allegro A4988
//driver such as https://www.pololu.com/product/1182
//(Don't forget the 100uF electrolytic between VMot and its ground!)

#define STEPPER_PIN_HALFSTEPPING 6
#define STEPPER_PIN_SLEEP 7
#define STEPPER_PIN_STEP 8
#define STEPPER_PIN_DIR 9

// had been 0x3f
#define STEPPER_POSTSCALE_MASK 0x02
// had been 0x1f
#define STEPPER_POSTSCALE_MASK_SEEKING 0x00
#define STEPPER_RANDOMSTEPS_MIN -70
#define STEPPER_RANDOMSTEPS_MAX 100
#define STEPPER_SEEKSTEPS 15
#define STEPPER_FULLSTEPS_PER_ROTATION 200

// Values for the servo that wiggles the laser up and down
// The angle set will range from LOW to HIGH+WIGGLE
// A new angle will be set at random every DWELL_TIME ms
#define SERVO_PIN_PULSE 16
// the ms ranges should be determined for each model of servo used
// MG90s purchased in 2017: control range apx 0.7ms (700µs) to 2.3ms
// 1ms to 2ms resulted in 90-degree movement
// Mount horn on left because lower values rotate further clockwise.
// Mount arm on horn so that 1ms points down and 2ms points back across the servo case.
// Pulse time units are in µs
#define SERVO_PULSE_SAFETY_MIN 700
#define SERVO_PULSE_SAFETY_MAX 2300
#define SERVO_PULSE_USABLE_MIN 1000
#define SERVO_PULSE_USABLE_MAX 2000
#define SERVO_PULSE_DELTA_LIMIT 5

#define SERVO_HOLD_TIME_MS 500

//#define SERVO_ANGLE_LOW_LIMIT 90
//#define SERVO_ANGLE_HIGH_LIMIT 170
// wiggle was 6
#define SERVO_ANGLE_WIGGLE 3
// speed must not be > 255 - SERVO_ANGLE_HIGH or possible byte overflow error
#define SERVO_ANGLE_SPEED 20
#define SERVO_ANGLE_DWELL_TIME 2500L
#define SERVO_POSTSCALE_MASK 0x0f
#define SERVO_POSTSCALE_MASK_SEEKING 0xFF

#define IR_REFLECTANCE_MINIMUM_CONTRAST 16
// IR Reflectance readings are done at half speed, so reading * step per read should equal 400 (200 if done at full speed)
#define IR_REFLECTANCE_READINGS 100
#define IR_REFLECTANCE_STEPS_PER_READ 4
//in testing, mid-range reads correlate to distance from sensor: 1% @1cm; 5% @2cm; 8% @3cm
//
#define IR_REFLECTANCE_MID_READ_LIMIT 10
#define IR_REFLECTANCE_DEFAULT_PRESENT 550
#define IR_REFLECTANCE_DEFAULT_ABSENT 400
#define IR_REFLECTANCE_PIN A10
//an out-of-range threshold value to ensure IR reflectance never is used
#define IR_REFLECTANCE_DO_NOT_USE_THRESHOLD 1111

//when seeking, the system will make
//at most this many full rotations; if
//tape is not found within that time,
//it will reenter initialization.
#define SEEKING_ROTATION_LIMIT 2

#define DEBUG_SERIAL
#define DEBUG_SERIAL_DATARATE 57600
#define DEBUG_SERIAL_OUTPUT_INTERVAL_MS 2000
#define DEBUG_SERVO
#define DEBUG_KNOBS
#define DEBUG_LIGHTSENSOR
#define DEBUG_REFLECTANCE
//#define DEBUG_REFLECTANCE_INIT_READINGS

#define DEBUG_STEPPER
//#define DEBUG_STEPPER_STEPS
#define DEBUG_LASERCONTROLLER
//#define DEBUG_LASER_DUTY_CYCLE
#define DEBUG_INTERRUPT_FREQUENCY
