# laser-scarecrow-arduino
For Dr. Rebecca Brown's research testing the effectiveness of laser light in preventing bird damage in sweet corn

Please look for information about this project and related resources at http://digitalcommons.uri.edu/riaes_bulletin/

The v2 2018 kit release is what we used in the summer of 2018.
It is configured for black buckets, with aluminum tape marking any area that should be skipped.
Three trimpots control the overall speed, minimum angle, and angle range.

# Installing
Unfortunately, I have been unable to find a reliable tool for flashing a .hex file to the ATMega 32U4 microcontroller. So, you will need to download the complete code and libraries, open it in the Arduino IDE, and compile and upload it from there. The upside is you'll be able to tweak settings in config.h if you wish.

## Set up your system
You need to do these steps only once.

### Install the Arduino IDE
You will need to download and install the Arduino IDE (not the web editor) from https://www.arduino.cc/en/Main/Software

### Configure the Sparkfun Board Library
Go to the Arduino IDE Preferences (in the file menu) and add https://raw.githubusercontent.com/sparkfun/Arduino_Boards/master/IDE_Board_Manager/package_sparkfun_index.json to the Additional Boards Manager URLs field (it will probably be empty). OK that dialog, then from Tools > Board… > Board Manager, find “SparkFun AVR Boards by SparkFun Electronics” and install that.  Now you can select Tools > Board > **SparkFun Pro Micro** and Tools > Processor > **ATMega32U4 (5V, 16MHz)**. 

You must have the correct combination of board and processor selected or it is possible to put the module into an unusable state.

### Configure Third-party libraries
In addition to standard Arduino libraries (Wire, Servo, Serial, EEPROM), this project uses the following:

- uRTCLib - https://github.com/Naguissa/uRTCLib (GPL v3)
- CRC32 - https://github.com/bakercp/CRC32 (MIT License)

To make these available in the Arduino IDE, select Sketch > Include Library > Manage Libraries… Search for **uRTCLib** by Naguissa; install that and also install **CRC32** by Christopher Baker. 

### Get the code
From this https://github.com/davidhbrown-uri/laser-scarecrow-arduino page, click the “Clone or download” link to get a .zip file which includes all the code for this project. Expand the .zip archive to a convenient location on your computer. If the code changes (likely, but infrequently), you will repeat this step.

## Program the Arduino
Start here if you've already set up your system as described above.

### Open LaserScarecrow.ino
The zip file you downloaded and expanded includes a LaserScarecrow folder. In here you'll find the LaserScarecrow.ino file. Opening that file should start the Arduino IDE, or start the Arduino IDE and choose File > Open to find it that way.

### Check your board and existing portx
Triple-check that you have the SparkFun Pro Micro selected as your board and the 5V processor selected (to avoid damaging the Arduino). Select Tools > Board > **SparkFun Pro Micro** and Tools > Processor > **ATMega32U4 (5V, 16MHz)**. 

Before connecting the Arduino, look at the Tools > Port menu to see what ports are already listed (maybe even write them down); you can ignore them. 

### Connect, select port, upload
Now connect your Arduino module to the computer with a standard USB micro cable. You should see a new port appear in the Tools > Port menu. Select this port. 

Finally, from the Sketch menu select Upload. If all goes well, the sketch should compile and be uploaded to the module. 

At this point, you can unplug the Arduino. If you have multiple devices to program, this connect, select (should be same port number if you use the same USB port), upload step can be repeated.
Some
