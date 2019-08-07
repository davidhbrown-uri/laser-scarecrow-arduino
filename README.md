# laser-scarecrow-arduino
For Dr. Rebecca Brown's research testing the effectiveness of laser light in preventing bird damage in sweet corn

Please look for information about this project and related resources at https://sites.google.com/view/urilaserscarecrow/home and  http://digitalcommons.uri.edu/riaes_bulletin/

The v2 2018 kit release is what we used in the summer of 2018.
It is configured for black buckets, with aluminum tape marking any area that should be skipped.
Three trimpots control the overall speed, minimum angle, and angle range.
Version 2.3 includes improvements for 

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

At this point, you can unplug the Arduino. If you have multiple devices to program, this connect-select-upload sequence can be repeated. (If you use the same USB port, additional devices should show up on the same port number.)

# Operation
This section describes the basics of how the Scarecrow behaves. 
Description of the tape sensor functions presumes a black bucket (low reflectance normally) with aluminum tape (high reflectance) where the laser should not shine. If you have a white bucket and wish to use the tape sensor, you should apply low-reflectance tape (e.g., a black Duck or Gorilla tape) where you do wish the laser to shine. You may also need to use aluminum tape to ensure areas are skipped.
## Automatic behavior
### Power-on self-check
When the scarecrow is connected to power, you will hear and perhaps see a bit of a jump as the stepper and servo motors engage. This is followed by a pause of a few seconds in case a USB Serial connection were needed for debugging.
### Tape sensor calibration
This process occurs at power-on and repeats after approximately every hour (to better adapt to changing light levels) when the scarecrow is active. The laser is off during this process.
1. The scarecrow fairly quickly reads the tape sensor about 100x to see whether the reading is stable. If it is not, the tape sensor is either missing, disconnected, or damaged; the tape sensor will be ignored and normal operation will begin.
2. The arm rotates _backwards_ one turn at full speed to find the range of minimum and maximum reflectance values. If the range of values is too small, a second rotation will be tried. If the final range of values is too small, the tape sensor will be ignored and normal operation will begin.
3. The arm rotates _forwards_ one turn at normal (microstep) speed one turn to more carefully map the range of values (internally, it calculates a histogram) to ensure there is sufficient separation between high and low values. If there is not, the tape sensor will be ignored and normal operation will begin.
4. The arm rotates _backwards_ at microstep speed to find the beginning of the next usable span (not blocked off by reflective tape). It then continues backwards one full turn to find size of the smallest usable span. This value is used to set the size of the random motion.
## Normal (daylight) operation
### Random rotation and angle
The scarecrow selects a random number of microsteps to move, either forwards or backwards, but the forward range is larger than backwards, so the general trend of motion is forward. Once the scarecrow has taken that number of steps, a new random number of steps is selected. 
Similarly, the servo will move toward a randomly selected angle. Once that angle is reached, a new target angle will be selected.
The speed setting controls how often these steps are taken. 
#### Laser Cooldown
Once the laser has accumulated 30min of on-time, it needs to cool down for 5 minutes. During this time, the step LED will blink on-and-off at a rate between 1 and 2 times per second.
#### Tape sensor recalibrate
The tape sensor will recalibrate approximately every hour (see description above). The time this takes is dependent on the scarecrow's speed, but is typically about 30 seconds.
#### Skipping tape
If the tape sensor calibration completed successfully, whenever the scarecrow detects reflective tape, it will move forward at full speed until tape is no longer sensed. A new, forward-only movement step will be selected and normal operation continues.
## Night mode
When the light sensor detects low levels of light (averaged over several seconds), it will turn off the laser, stepper motor, and servo until light levels are back up over the preset threshold. 
## Control knobs
Three knobs are available on the circuit board to adjust the scarecrow's behavior. 
 -  The first knob controls the overall speed: how much time elapses between steps as the stepper and servo complete their movements.
 - The second knob sets the minimum angle of the servo
 - The third knob determines how much of the range beyond that minimum will be used. (Basically, it sets the maximum angle, but if we used it to set the maximum angle directly, you could easily ask for a maximum that was less than the minim. That would be more confusing than this explanation.)

Changes to the settings are saved after about 15 seconds, so give it a while after you get it where you want it before disconnecting power.
## Bluetooth control
An alpha-quality Android app is available by request. (Apple's iOS devices don't support Bluetooth version 3.) "Alpha" quality means lots of bugs and missing features.
When the Bluetooth interface is connected...
- night mode is never active
- laser rotation pauses (the servo keeps going though)
- the laser pulses in a pattern depending on whether it would normally be shining where it's pointed (each - or O is 150ms):
	- mostly on if it would be shining, like -O-OOOOO, or
	- mostly off, like O-O----- when it is in an area where it normally wouldn't shine (e.g., reflective tape is sensed.) 

Controls in the app allow you to rotate the arm, adjust the speed and angle settings, and resume normal rotation/skipping behavior.
