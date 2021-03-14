# laser-scarecrow-arduino
For Dr. Rebecca Brown's research testing the effectiveness of laser light in preventing bird damage in sweet corn

Please look for information about this project and related resources at https://sites.google.com/view/urilaserscarecrow/home and  http://digitalcommons.uri.edu/riaes_bulletin/

Laser devices are subject to regulation at national, regional/state, and local levels. You must research and comply with laws that apply to your situation. We cannot and will not provide legal advice.

# Installing
Unfortunately, I have been unable to find a reliable tool for flashing a .hex file to the ATMega 32U4 microcontroller. 

This software was developed using PlatformIO within Visual Studio Code. Both are free to use but require some setup. 

### Configure Third-party libraries
In addition to standard Arduino libraries (Wire, Servo, Serial, EEPROM), this project uses the following:

- uRTCLib - https://github.com/Naguissa/uRTCLib (GPL v3)
- CRC32 - https://github.com/bakercp/CRC32 (MIT License)
