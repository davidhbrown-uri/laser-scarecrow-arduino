/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */
#include "Led.h"

Led::Led(int pin, bool invert)
{
    pinMode(pin, OUTPUT);
    _inverted = invert;
    _pin = pin;
    _mode = LED_MODE_OFF;
    _turn_off();
}
void Led::_turn_on()
{
    digitalWrite(_pin, _inverted ? LOW : HIGH);
    _on = true;
}
void Led::_turn(bool is_on)
{
    if (is_on)
    {
        _turn_on();
    }
    else 
    {
        _turn_off();
    }
}
void Led::_turn_off()
{
    digitalWrite(_pin, _inverted ? HIGH : LOW);
    _on = false;
}
void Led::on()
{
    _mode = LED_MODE_ON;
    _turn_on();
}
void Led::off()
{
    _mode = LED_MODE_OFF;
    _turn_off();
}
void Led::flicker()
{
    blink(LED_FLICKER_MS, LED_FLICKER_MS);
}
void Led::blink(unsigned long ms_on, unsigned long ms_off)
{
    _ms_blink_on = ms_on;
    _ms_blink_off = ms_off;
    _ms_blink_last = millis();
    _mode = LED_MODE_BLINK;
}
void Led::external()
{
    _mode = LED_MODE_EXTERNAL;
}
void Led::update()
{
    switch (_mode)
    {
        case LED_MODE_OFF:
        _turn_off();
        break;
        case LED_MODE_ON:
        _turn_on();
        break;
        case LED_MODE_BLINK:
        {
            unsigned long elapsed = millis() - _ms_blink_last;
        if (elapsed > (_on ? _ms_blink_on : _ms_blink_off))
        {
            _ms_blink_last = millis();
            _turn(!_on);
        }
        }
        break;
        case LED_MODE_EXTERNAL:
        ; // do nothing
        break;
        default:
        ;
    }
}