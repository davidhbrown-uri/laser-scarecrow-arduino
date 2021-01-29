#pragma once

#include "Arduino.h"
#include "config.h"

#define LED_MODE_ON 1
#define LED_MODE_OFF 0
#define LED_MODE_BLINK 2
class Led
{
    private:
        byte _mode;
        int _pin;
        bool _inverted;
        bool _on;
        unsigned long _ms_blink_on;
        unsigned long _ms_blink_off;
        void _turn_on();
        void _turn_off();
    public:
        Led(int pin, bool invert);
        void update();
        void on();
        void off();
        void blink(unsigned long ms_on, unsigned long ms_off);
};