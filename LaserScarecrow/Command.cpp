/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */
 
/**
   @see Command.h
*/
#include "Command.h"
#define SPACE ' '
#define TAB '\t'
#define NEWLINE '\n'
#define RETURN '\r'
#define DECIMAL_PLACE_SHIFT 10
#define ASCII_ZERO 48
//'0'==48, '9'==57
#define ASCII_CASE_SHIFT 32
//'A'==65, 'a'==97

Command::Command() {
  init();
}
void Command::init()
{
  state = CSTATE_INIT;
  errorCode = CERROR_NoError;
  code = 0;
  letter = '_';
  parameterCount = 0;
  for (int i = 0; i < COMMAND_PARAMETER_LIMIT; i++)
  {
    parameter[i] = 0;
  }
  isParsed = false;

}//::init()

// it may only be a coincidence at this point in development that the logic of
// isWritable is the inverse of isReadyToProcess; I don't think we should make
// them depend on each other or omit one unless we get incredibly desparate for bytes

bool Command::isWritable() {
  return !(state == CSTATE_ERROR || state == CSTATE_OK); //these states should NOT be written
}
bool Command::isReadyToProcess() {
  return (state == CSTATE_ERROR || state == CSTATE_OK); //these states can be processed
}

void Command::write(int inByte)
{
  char c = (char) inByte;
  switch (state) {
    case CSTATE_ERROR_FLUSHING:
      //consume any remaining characters
      if (c == NEWLINE) {
        isParsed = true;
        state = CSTATE_ERROR;
      }
      break;

    case CSTATE_INIT:
      //@TODO: start timeout timer
      if (isLowerCase(c))
      {
        c = c - ASCII_CASE_SHIFT; //convert to uppercase
      }
      if (isUpperCase(c)) {
        letter = c;
        state = CSTATE_GET_CODE;
      } else if (c == NEWLINE)
      { //EOL before letter, ignore whole line; no command received
        //@TODO cancel timeout timer
        state = CSTATE_INIT;
      } else if (isWhitespace(c))
      {
        ;//do nothing to ignore leading whitespace
      }  else { //something unrecognized
        state = CSTATE_ERROR_FLUSHING;
        errorCode = CERROR_InvalidCharacter;
      }
      break; //CSTATE_INIT

    case CSTATE_GET_CODE:
      if (isDigit(c)) {
        word last = code;
        code = code * DECIMAL_PLACE_SHIFT + (c - ASCII_ZERO);
        if (code < last) //overflow
        {
          state = CSTATE_ERROR_FLUSHING;
          errorCode = CERROR_MalformedCode;
        }
      } else if (c == NEWLINE)
      {
        isParsed = true;
        state = CSTATE_OK;
      } else if (c == RETURN)
      {
        ;//ignore
      } else if (isWhitespace(c)) //other than \r or \n
      {
        state = CSTATE_PARAM_POSSIBLE;
      }  else {
        state = CSTATE_ERROR_FLUSHING;
        errorCode = CERROR_InvalidCharacter;
      }
      break; //CSTATE_GET_CODE

    case CSTATE_PARAM_POSSIBLE:
      if (isDigit(c)) {
        if (parameterCount < COMMAND_PARAMETER_LIMIT)
        {
          parameter[parameterCount] = c - ASCII_ZERO;
          state = CSTATE_GET_PARAM;
        } else // got a digit but too many parameters
        {
          state = CSTATE_ERROR_FLUSHING;
          errorCode = CERROR_MalformedParameter;
        }
      }
      else if (c == NEWLINE)
      {
        state = CSTATE_OK;
      }
      else if (isWhitespace(c))
      {
        ;//ignore
      }
      else {
        state = CSTATE_ERROR_FLUSHING;
        errorCode = CERROR_InvalidCharacter;
      }
      break; // CSTATE_PARAM_POSSIBLE

    case CSTATE_GET_PARAM:
      if (isDigit(c)) {
        word last = parameter[parameterCount];
        parameter[parameterCount] = parameter[parameterCount] * DECIMAL_PLACE_SHIFT + (c - ASCII_ZERO);
        if (parameter[parameterCount] < last) //overflow
        {
          state = CSTATE_ERROR_FLUSHING;
          errorCode = CERROR_MalformedParameter;
        }
      } else if (c == NEWLINE)
      {
        parameterCount++;
        state = CSTATE_OK;
        isParsed = true;
      } else if (c == RETURN)
      {
        ;//ignore
      } else if (isWhitespace(c)) //other than \r or \n
      {
        parameterCount++;
        state = CSTATE_PARAM_POSSIBLE;
      }  else {
        state = CSTATE_ERROR_FLUSHING;
        errorCode = CERROR_InvalidCharacter;
      }
      break; // CSTATE_GET_PARAM

    case CSTATE_OK:
    case CSTATE_ERROR:
      ;//tempting to assert an error as characters can't be processed in this state, but no point on an Arduino
      break;

  } //switch state
}//::write()

void Command::processError()
{
  errorCode = CERROR_CouldNotProcess;
  state = CSTATE_ERROR;
}


/**
   Caution: Makes no attempt to check that the command is ready/valid
*/
void Command::printCommandToStream(Stream *stream_p)
{
  stream_p->print(letter);
  stream_p->print(code);
  for (int i = 0; i < parameterCount; i++)
  {
    stream_p->print(' ');
    stream_p->print(parameter[i]);
  }
}
/**
   Caution: Makes no attempt to check that the command is ready/valid
*/
void Command::printStatusToStream(Stream *stream_p)
{
  switch (state) {
    case CSTATE_OK:
      stream_p->println(F("ok"));
      break;
    case CSTATE_ERROR:
    case CSTATE_ERROR_FLUSHING:
      stream_p->print(F("error("));
      stream_p->print(errorCode);
      stream_p->println(')');
      break;
    default:
      stream_p->print(F("other("));
      stream_p->print(state);
      stream_p->print(')');
  }
}

void Command::printVerboseToStream(Stream *stream_p)
{
  stream_p->print(F("Command: "));
  printCommandToStream(stream_p);
  stream_p->println();
  stream_p->print(F("Status: "));
  printStatusToStream(stream_p);
  stream_p->println();
}


