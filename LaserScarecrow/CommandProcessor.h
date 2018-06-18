/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */

/**
   A command processor aggregates a Command and a Configuration, handling things
   that need to know about both so they can each focus on their own
   responsibilities.

   Created by David H. Brown, February 25, 2018
   License TBD
   Part of the URI Laser Scarecrow project
*/
#ifndef CommandProcessor_h
#define CommandProcessor_h
#include "Arduino.h"
#include "Settings.h"
#include "Command.h"
#include "AmbientLightSensor.h"
#include <uRTCLib.h>
extern uRTCLib rtc;
extern bool rtc_is_running;

// until we do have a configuration object, we'll need access to values defined in:
#include "config.h"

#define CPCODE_Hello 0
#define CPCODE_InterruptRate 101
#define CPCODE_StepperTarget 111
#define CPCODE_RtcControl 201
#define CPCODE_LightSensorRead 210
#define CPCODE_LightThrehold 221
#define CPCODE_RtcYmd 251
#define CPCODE_RtcHms 252
#define CPCODE_RtcRunning 259
#define CPCODE_RtcWake 261
#define CPCODE_RtcSleep 262
#define CPCODE_ServoMinimum 131
#define CPCODE_ServoRange 132


enum CPSTATUS {
  CPSTATUS_Ready,
  CPSTATUS_Done,
  CPSTATUS_InvalidLetter,
  CPSTATUS_InvalidCode,
  CPSTATUS_InvalidParameter,
  CPSTATUS_ProcessFailed
};

class CommandProcessor
{
  public:
    CommandProcessor();
    void setCommand(Command *cd);
    void setStream(Stream *st);
    void setSettings(Settings *stgs);
// future:    void setConfiguration(Configuration *cf);
// future:    void setRTC(MockRTC *rtc);
    void process();
    static byte getHoursFromTimeWord(word time);
    static byte getMinutesFromTimeWord(word time);
    static word getTimeWord(byte hours, byte minutes);
    CPSTATUS status;
  private:
    void processOK();
    void processError(CPSTATUS cperr);
    void finishProcess();
    Stream* stream;
    Command* command;
    Settings* settings;
// future:    Configuration* configuration;
// future:    MockRTC* rtc;
    bool verbose;
};// class CommandProcessor
#endif
