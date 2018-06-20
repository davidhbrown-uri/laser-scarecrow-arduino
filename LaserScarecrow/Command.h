/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */
 
/*
   Command.h - library for processing a text command
   Created by David H. Brown, February 19, 2018
   License TBD
   Part of the URI Laser Scarecrow project
   @todo: decide how to deal with a timeout
*/
#ifndef Command_h
#define Command_h

#include "Arduino.h"

#define COMMAND_PARAMETER_LIMIT 4
enum CSTATE {
  CSTATE_OK,
  CSTATE_ERROR,
  CSTATE_ERROR_FLUSHING,
  CSTATE_INIT,
  CSTATE_NEED_LETTER,
  CSTATE_GET_CODE,
  CSTATE_PARAM_POSSIBLE,
  CSTATE_GET_PARAM
};

enum CERROR {
  CERROR_NoError,
  CERROR_Unknown,
  CERROR_Timeout,
  CERROR_InvalidCharacter,
  CERROR_MalformedCode,
  CERROR_MalformedParameter,
  CERROR_CouldNotProcess
};

/*
   Memory space is tight; I am choosing to ignore encapsulation, especially when reading,
   and treat the Command somewhat like a struct, accessing and manipulating
   its data from outside. We'll see if I regret that! -DHB 2018-02-25
*/
class Command
{
  public:
    Command();
    /**
       Set defaults and get ready to read
    */
    void init();
    /**
       Transmit the next character for parsing
    */
    void write(int inByte);
    bool isWritable();
    bool isReadyToProcess();
    CERROR errorCode;
    void printCommandToStream(Stream *stream_p);
    void printStatusToStream(Stream *stream_p);
    void printVerboseToStream(Stream *stream_p);

    /**
       Call if command could not processed and an error occurred
       errorCode will be set to CERROR_CouldNotProcess and
       state is set to CSTATE_ERROR
    */
    void processError();

    CSTATE state;
    char letter;
    word code;
    byte parameterCount;
    word parameter[COMMAND_PARAMETER_LIMIT];
    bool isParsed; // the command has been read successfully or errored
    bool isProcessed; // the command has been dealt with by outside code
};

#endif


