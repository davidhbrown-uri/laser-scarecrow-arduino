# laser-scarecrow-arduino
For Dr. Rebecca Brown's research testing the effectiveness of laser light in preventing bird damage in sweet corn

Please look for information about this project and related resources at http://digitalcommons.uri.edu/riaes_bulletin/

The v2 2018 kit release is what we used in the summer of 2018.
It is configured for black buckets, with aluminum tape marking any area that should be skipped.
Three trimpots control the overall speed, minimum angle, and angle range.

## Sparkfun Board Library
I selected the Sparkfun Pro Micro 5V/16MHz board for this project. You will need to install their board libraries from https://github.com/sparkfun/Arduino_Boards

Make certain you select the 5V part both when purchasing (or it won't work for this) and when programming (or you might brick it).

## Third-party libraries
In addition to standard Arduino libraries (Wire, Servo, Serial, EEPROM), this project uses the following:

- uRTCLib - https://github.com/Naguissa/uRTCLib (GPL v3)
- CRC32 - https://github.com/bakercp/CRC32 (MIT License)
