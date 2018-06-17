/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */
 
/**
   @see CommandProcessor.h
*/
#include "CommandProcessor.h"
CommandProcessor::CommandProcessor()
{
  status = CPSTATUS_Ready;
  verbose = false;
}
void CommandProcessor::setCommand(Command *cd)
{
  command = cd;
}
void CommandProcessor::setSettings(Settings *stgs)
{
  settings = stgs;
}
/* Future:
void CommandProcessor::setConfiguration(Configuration *cf)
{
  configuration = cf;
}
*/
void CommandProcessor::setStream(Stream *st)
{
  stream = st;
  stream->println(F("Laser Scarecrow Command Processor Ready!"));
}
/*
void CommandProcessor::setRTC(MockRTC *mockRTC)
{
  rtc=mockRTC;
}
*/
void CommandProcessor::process()
{
  if (stream->available() && command->isWritable()) {
    command->write(stream->read());
  }
  if (command->state == CSTATE_OK)
  {
    switch (command->letter) {
      case 'L':
        command->printCommandToStream(stream);
        stream->print(' ');
        switch (command->code) {
          case CPCODE_Hello:
            processOK();
            break;          
          case CPCODE_CycleMode:
            stream->println(settings->cycle_mode);
            processOK();
            break;
          case CPCODE_LightThrehold:
            stream->println(settings->light_sensor_threshold);
            processOK();
            break;
          case CPCODE_InterruptRate:
            stream->println(settings->interrupt_frequency);
            processOK();
            break;
          case CPCODE_StepperTarget:
            stream->println(settings->stepper_target);
            processOK();
            break;
          case CPCODE_ServoMinimum:
            stream->println(settings->servo_min);
            processOK();
            break;
          case CPCODE_ServoRange:
            stream->println(settings->servo_max-settings->servo_min);
            processOK();
            break;
/*
          case CPCODE_RtcHms:
             stream->print(rtc->getHours()); stream->print(' ');
             stream->print(rtc->getMinutes()); stream->print(' ');
             stream->println(rtc->getSeconds());
             processOK();
             break;
          case CPCODE_RtcWake:
             stream->print(getHoursFromTimeWord(settings->rtc_wake)); stream->print(' ');
             stream->println(getMinutesFromTimeWord(settings->rtc_wake));
             processOK();
             break;
          case CPCODE_RtcSleep:
             stream->print(getHoursFromTimeWord(settings->rtc_sleep)); stream->print(' ');
             stream->println(getMinutesFromTimeWord(settings->rtc_sleep));
             processOK();
             break;
*/
          default:
            processError(CPSTATUS_InvalidCode);
        } // switch code
        break; // case L (look / report)
      case 'S':
        switch (command->code) {
          case CPCODE_CycleMode: // cycle_mode
            if (command->parameterCount == 1 && command->parameter[0] < 4)
            {
              settings->cycle_mode = command->parameter[0];
              processOK();
            } else
            {
              processError(CPSTATUS_InvalidParameter);
            }
            break; // case 201
          case CPCODE_LightThrehold:
            if (command->parameterCount == 1 && command->parameter[0] < 1024)
            {
              settings->light_sensor_threshold = command->parameter[0];
              processOK();
            } else
            {
              processError(CPSTATUS_InvalidParameter);
            }
            break; //CPCODE_LightThrehold
          case CPCODE_InterruptRate:
            if (command->parameterCount == 1 && command->parameter[0] < 1024)
            {
              /* @todo get limits from configuration */
              /* @todo rename Rate to Frequency */
              settings->interrupt_frequency = constrain(command->parameter[0],INTERRUPT_FREQUENCY_MIN,INTERRUPT_FREQUENCY_MAX);
              processOK();
            } else
            {
              processError(CPSTATUS_InvalidParameter);
            }
            break; //CPCODE_InterruptRate
          case CPCODE_StepperTarget:
            if (command->parameterCount == 1)
            {
              settings->stepper_target = command->parameter[0] % 360;
              processOK();
            } else 
            {
              processError(CPSTATUS_InvalidParameter);
            }
            break; //CPCODE_StepperTarget
          case CPCODE_ServoMinimum:
            if (command->parameterCount == 1 && command->parameter[0] < 1024)
            {
              /* @todo get servo limits from configuration */
              settings->servo_min = constrain(command->parameter[0], SERVO_PULSE_USABLE_MIN, SERVO_PULSE_USABLE_MAX);
              processOK();
            } else
            {
              processError(CPSTATUS_InvalidParameter);
            }
            break; //CPCODE_ServoMinimum
          case CPCODE_ServoRange:
            if (command->parameterCount == 1 && command->parameter[0] < 1024)
            {
              /* @todo get SERVO_ANGLE_HIGH_LIMIT from configuration */
              settings->servo_max = map(command->parameter[0],0,1023,settings->servo_min,SERVO_PULSE_USABLE_MAX);
              processOK();
            } else
            {
              processError(CPSTATUS_InvalidParameter);
            }
            break; //CPCODE_ServoRange
/*            
          case CPCODE_RtcHms:
            if (command->parameterCount > 1 && command->parameterCount < 4) {
              int hours = command->parameter[0] % 24;
              int minutes = command->parameter[1] % 60;
              int seconds = (command->parameterCount == 3) ? command->parameter[2] % 60 : 0; 
              stream->print(hours);stream->print(':');
              stream->print(minutes);stream->print(':');
              stream->println(seconds);
              rtc->setTime(hours, minutes, seconds);
              processOK();
            } else 
            {
              processError(CPSTATUS_InvalidParameter);
            }
            break;
          case CPCODE_RtcWake:
          case CPCODE_RtcSleep:
            if(command->parameterCount == 2 && 
              command->parameter[0]>=0 && command->parameter[0]<24 && 
              command->parameter[1]>=0 && command->parameter[0]<60) {
                if(command->code==CPCODE_RtcWake) {
                  settings->rtc_wake = getTimeWord(command->parameter[0],command->parameter[1]);
                } else {
                  settings->rtc_sleep = getTimeWord(command->parameter[0],command->parameter[1]);
                }
                processOK();
              } // if parameters valid
             else 
            {
              processError(CPSTATUS_InvalidParameter);
            }
             break;
*/
          default:
            processError(CPSTATUS_InvalidCode);
        } // switch code
        break; // case S (set)
      case 'X':
        switch (command->code) {
          case 0:
            stream->println(F("Laser Scarecrow Debug Available!"));
            break;
          case 1: //verbosity
            verbose = (command->parameterCount > 0 && command->parameter[0] > 0);
            break;
/*            
          case CPCODE_RtcHms:
            stream->print(F("System millis = ")); stream->println(millis());
            stream->print(F("MockRTC offset = ")); stream->println(rtc->offset);
            stream->print(F("MockRTC millisSinceMidnight = ")); stream->println(rtc->getMillisSinceMidnight());
            break;
*/            
        }// switch code
        // for any debug code, call it good.
        processOK();
        break;//debug codes 'X'
      default:
        processError(CPSTATUS_InvalidLetter);
    }
  }// if CSTATE_OK
  if (command->state == CSTATE_ERROR)
  {
    //not our error; we just need to wrap things up
    processOK();
  }
}
void CommandProcessor::processOK() {
  status = CPSTATUS_Done;
  finishProcess();
}
void CommandProcessor::processError(CPSTATUS cperr) {
  status = cperr;
  if (verbose)
  {
    stream->print(F("Command Processor Error Code "));
    stream->println(status);
  }
  command->processError();
  finishProcess();
}
void CommandProcessor::finishProcess()
{
  if (verbose) {
    command->printVerboseToStream(stream);
  } else
  {
    command->printStatusToStream(stream);
  }
  command->init();
  status = CPSTATUS_Ready;
}

byte CommandProcessor::getHoursFromTimeWord(word time){
  word hours = time / 60;
  return hours > 255 ? 255 : hours;
}
byte CommandProcessor::getMinutesFromTimeWord(word time){
  return time % 60;
}
word CommandProcessor::getTimeWord(byte hours, byte minutes)
{
  return (word) (60L*(long)hours+(long)minutes);
}
