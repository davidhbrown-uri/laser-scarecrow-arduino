/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */
 
// LaserController.h

/*
  Do not attempt to drive a laser diode module directly from an arduino/atmega pin output!
  Instead, use something like a TIP120/TIP121/TIP122 switching transistor to handle
  the high current required.
*/

/*
  CRITICAL ASSUMPTION: the diode's duty cycle off time is less than on time and
  so we can count time spent off against time spent on. E.g., if
  the laser is on 40 seconds, then off 10 seconds before being turned
  back on, we can start the duty cycle clock back at 30 seconds
  without endangering the laser diode.
*/
#pragma once
#include "Arduino.h"
#include "config.h"
// for testing, a rapid duty cycle: 20sec on, 30sec off
#ifdef DEBUG_LASER_DUTY_CYCLE
#define LASERCONTROLLER_DEFAULTDUTYCYCLERUNTIME 20000
#define LASERCONTROLLER_DEFAULTDUTYCYCLECOOLDOWN 30000
#else
// duty cycle 30min on, 5min off (example from 50mW wide-beam green Laser Module)
#define LASERCONTROLLER_DEFAULTDUTYCYCLERUNTIME 1800000
#define LASERCONTROLLER_DEFAULTDUTYCYCLECOOLDOWN 300000
#endif // !DEBUG_LASER_DUTY_CYCLE


class LaserController {
  public:
    static void init();
    static void setDutyCycleMaxRuntime(unsigned long millis);
    static void setDutyCycleMinCooldown(unsigned long millis);
    static void update();
    static void turnOn();
    static void turnOff();
    static bool isOn();
    static bool isCoolingDown();
    static void doInterrupt();
#ifdef DEBUG_LASER_SERIAL
    static long getAccumulatedOnTime();
#endif

  private:
    static void _enterCooldown();
    static void _leaveCooldown();
    static void __on();
    static void __off();
    static bool _requestedOn;
    static bool _isOn;
    static bool _isCoolingdown;
    static unsigned long _lastRun_millis;
    static bool _lastRun_isOn;
    static unsigned long _accumulatedOnTime;
    static unsigned long _accumulatedOffTime;
    static unsigned long _dutyCycleMaxRuntime;
    static unsigned long _dutyCycleMinCooldown;
};
