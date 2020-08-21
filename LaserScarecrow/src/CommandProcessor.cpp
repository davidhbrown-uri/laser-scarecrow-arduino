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
  //stream->println(F("Laser Scarecrow Command Processor Ready!"));
  //this was silly; we don't know there's anyone listening yet.
}

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
            stream->print(SOFTWARE_VERSION);
            stream->print(' ');
            processOK();
            break;
          case CPCODE_StateCurrent:
            stream->println(stateCurrent);
            processOK();
            break;
          case CPCODE_StateManual:
            stream->println(stateManual ? 1 : 0);
            processOK();
            break;
          case CPCODE_InterruptRate:
            stream->println(settings->interrupt_frequency);
            processOK();
            break;
          /*
            case CPCODE_StepperTargetAngle:
            stream->println(settings->stepper_target);//NOT a setting; current state
            processOK();
            break;
          */
          case CPCODE_StepperTargetMicrosteps:
            stream->println(StepperController::getStepsToStep());
            processOK();
            break;
          case CPCODE_ServoMinimum:
            stream->println(settings->servo_min);
            processOK();
            break;
          case CPCODE_ServoRange:
            stream->println(settings->servo_max - settings->servo_min);
            processOK();
            break;
          case CPCODE_TapeSensed:
            stream->println(IrReflectanceSensor::isPresent() ? 0 : 1);
            processOK();
            break;
          case CPCODE_TapeSensor:
            stream->println(IrReflectanceSensor::read());
            processOK();
            break;
          case CPCODE_LightSensorRead:
            stream->println(AmbientLightSensor::read());
            processOK();
            break;
          case CPCODE_LightSensorReadMin:
            stream->println(AmbientLightSensor::minimum_reading());
            processOK();
            break;
          case CPCODE_LightSensorReadMax:
            stream->println(AmbientLightSensor::maximum_reading());
            processOK();
            break;
          case CPCODE_LightThrehold:
            stream->println(settings->light_sensor_threshold);
            processOK();
            break;
          default:
            processError(CPSTATUS_InvalidCode);
        } // switch code
        break; // case L (look / report)
      case 'S':
        switch (command->code) {
          case CPCODE_StateManual: // request manual control
            if (command->parameterCount == 1 && command->parameter[0] < 2)
            {
              stateManual = command->parameter[0] > 0;
              processOK();
            } else
            {
              processError(CPSTATUS_InvalidParameter);
            }
            break;
          case CPCODE_StepperMicrostepPositive: // move this many steps forward
            if (command->parameterCount == 1) // check that param0 is in integer range?
            {
              StepperController::setStepsToStep((int) command->parameter[0]);
              processOK();
            }
            else
            {
              processError(CPSTATUS_InvalidParameter);
            }
            break;
          case CPCODE_StepperMicrostepNegative: // move this many steps backward
            if (command->parameterCount == 1) // check that param0 is in integer range?
            {
              StepperController::setStepsToStep(0 - (int) command->parameter[0]);
              processOK();
            }
            else
            {
              processError(CPSTATUS_InvalidParameter);
            }
            break;
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
              settings->interrupt_frequency = map(command->parameter[0], 0, 1023, INTERRUPT_FREQUENCY_MIN, INTERRUPT_FREQUENCY_MAX);
              processOK();
            } else
            {
              processError(CPSTATUS_InvalidParameter);
            }
            break; //CPCODE_InterruptRate
          /*
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
          */
          case CPCODE_ServoMinimum:
            if (command->parameterCount == 1 && command->parameter[0] < 1024)
            {
              /* @todo get servo limits from configuration */
              // update v2.1... use map instead of constrain; also attempt to reset range (avoid inversion)
              int currentRange = map(settings->servo_max, settings->servo_min, SERVO_PULSE_USABLE_MAX, 0, 1023);
              settings->servo_min = map(command->parameter[0], 0, 1023, SERVO_PULSE_USABLE_MIN, SERVO_PULSE_USABLE_MAX);
              settings->servo_max = map(currentRange, 0, 1023, settings->servo_min, SERVO_PULSE_USABLE_MAX);
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
              settings->servo_max = map(command->parameter[0], 0, 1023, settings->servo_min, SERVO_PULSE_USABLE_MAX);
              processOK();
            } else
            {
              processError(CPSTATUS_InvalidParameter);
            }
            break; //CPCODE_ServoRange
          case CPCODE_ServoTarget:
            if (command->parameterCount == 1 && command->parameter[0] < 1024)
            {
              ServoController::setPulseTarget( map(command->parameter[0], 0, 100, 
                  ServoController::getPulseRangeMin(), ServoController::getPulseRangeMax()));
              processOK();
            } else
            {
              processError(CPSTATUS_InvalidParameter);
            }
            break;

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
                        stream->print(F("MockRTC offset = ")); stream->println(rtc.offset);
                        stream->print(F("MockRTC millisSinceMidnight = ")); stream->println(rtc.getMillisSinceMidnight());
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
