/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */
#pragma once

#include "Arduino.h"
#include "config.h"

#define LED_MODE_ON 1
#define LED_MODE_OFF 0
#define LED_MODE_BLINK 2
#define LED_MODE_EXTERNAL 99
#define LED_FLICKER_MS 50
class Led
{
    private:
        byte _mode;
        int _pin;
        bool _inverted;
        bool _on;
        unsigned long _ms_blink_on;
        unsigned long _ms_blink_off;
        unsigned long _ms_blink_last;
        void _turn_on();
        void _turn_off();
        void _turn(bool is_on);
    public:
    /**
     * @brief Construct a new Led object
     * 
     * @param pin digitial I/O pin to use; will be set to OUTPUT
     * @param invert true if writing LOW turns the LED on
     */
        Led(int pin, bool invert);
        /**
         * @brief Call this every time through the loop() to allow blinking
         * 
         */
        void update();
        /**
         * @brief Turn LED on
         * Caution: Do not do this for the USB RX LED or you'll have to use a reset button to reprogram
         * 
         */
        void on();
        /**
         * @brief Turn LED off
         * 
         */
        void off();
        /**
         * @brief Blink LED rapidly
         * 
         */
        void flicker();
        /**
         * @brief Blink LED on and off a specified ms each
         * Starts with whichever state the LED is on when blink() is called
         * Caution: avoid long ms_on times for USB RX LED or reprogram will require reset button press
         * 
         * @param ms_on time on in milliseconds
         * @param ms_off time off in milliseconds
         */
        void blink(unsigned long ms_on, unsigned long ms_off);
        /**
         * @brief Don't control this LED
         * 
         */
        void external();
};