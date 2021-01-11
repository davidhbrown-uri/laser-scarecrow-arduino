/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */

#include "StepperController.h"
#include "config.h"

#define STEPPER_CONTROLLER_STATE_OFF 0
#define STEPPER_CONTROLLER_STATE_RUNNING 1
#define STEPPER_CONTROLLER_STATE_WAITING 11
#define STEPPER_CONTROLLER_STATE_STOPPING 12

#define STEPPER_CONTROLLER_EVENT_NONE 0
#define STEPPER_CONTROLLER_EVENT_MOVE 1
#define STEPPER_CONTROLLER_EVENT_TURN_OFF 2

void StepperController::init()
{
  // init fields
  _isr_top_accel_table_max_index = _isr_top_accel_table_length - 1;
  _isr_steps_remaining = 0L;
  _isr_steps_taken = 0L;
  _isr_top_accel_table_current_index = 0;
  _state = STEPPER_CONTROLLER_STATE_OFF;
  _statePrevious = STEPPER_CONTROLLER_STATE_OFF;
  _stateReturn = STEPPER_CONTROLLER_STATE_OFF;
  _event = STEPPER_CONTROLLER_EVENT_NONE;
  _stepsRequested = 0;
  _direction = true;
  _waitBeganAtMS = 0L;
  _waitDuration = 0L;
  //set up i/o based on defines in config.h:
  pinMode(STEPPER_PIN_SLEEP, OUTPUT);
  pinMode(STEPPER_PIN_STEP, OUTPUT);
  pinMode(STEPPER_PIN_DIR, OUTPUT);
  pinMode(STEPPER_PIN_MICROSTEP, OUTPUT);
  digitalWrite(STEPPER_PIN_SLEEP, LOW);
  digitalWrite(STEPPER_PIN_STEP, LOW);
  digitalWrite(STEPPER_PIN_DIR, LOW);
  digitalWrite(STEPPER_PIN_MICROSTEP, LOW);
  _buildTable();
  applySettings(&currentSettings);
  _initTimer();
}

void StepperController::update()
{
  switch (_state)
  {
  case STEPPER_CONTROLLER_STATE_OFF:
    // enter/
    if (_state != _statePrevious)
    {
      digitalWrite(STEPPER_PIN_SLEEP, LOW);
      _statePrevious = _state;
    } // end enter/
    // do/
    if (_event == STEPPER_CONTROLLER_EVENT_MOVE)
    {
      _state = STEPPER_CONTROLLER_STATE_RUNNING;
    } // do/ event move
    // exit
    if (_state != _statePrevious)
    {
      digitalWrite(STEPPER_PIN_SLEEP, HIGH);
      digitalWrite(STEPPER_PIN_MICROSTEP, HIGH);
      delay(1);
    } // end exit/
    break;
  case STEPPER_CONTROLLER_STATE_RUNNING:
    // enter/
    if (_state != _statePrevious)
    {
      _statePrevious = _state;
    } //end enter/
    // do/
    switch (_event)
    {
    case STEPPER_CONTROLLER_EVENT_MOVE:
      if (_direction != (_stepsRequested >= 0)) // need to change direction
      {
        if (_isr_steps_remaining == 0) // have to finish previous move
        {
          _waitDuration = STEPPER_MILLISEC_DIRECTION_PAUSE;
          _state = STEPPER_CONTROLLER_STATE_WAITING;
          _direction = _stepsRequested >= 0;
          digitalWrite(STEPPER_PIN_DIR, _direction ? HIGH : LOW);
        }
      }    // do direction change
      else // we can do the move
      {
        _isr_steps_remaining += abs(_stepsRequested);
        _stepsRequested = 0; // clear so we can tell if we've stopped
        _event = STEPPER_CONTROLLER_EVENT_NONE;
      } // end else
      break;
    case STEPPER_CONTROLLER_EVENT_TURN_OFF:
      if (_isr_steps_remaining == 0)
      {
        _state = STEPPER_CONTROLLER_STATE_OFF;
        _event = STEPPER_CONTROLLER_EVENT_NONE;
      }
      break;
    case STEPPER_CONTROLLER_EVENT_NONE:; // nothing to process
      break;
    default:; //no event?
    }         //end switch on event
    // end do/
    // exit/
    if (_state != _statePrevious) // exit/ ?
    {
    }
    break; // end RUNNING
  case STEPPER_CONTROLLER_STATE_WAITING:
    // enter/
    if (_state != _statePrevious)
    {
      _waitBeganAtMS = millis();
      _stateReturn = _statePrevious;
      _statePrevious = _state;
    }
    // do
    if (millis() - _waitBeganAtMS > _waitDuration)
    {
      _state = _stateReturn;
    }
    // exit
    if (_state != _statePrevious) // exit/ ?
    {
      ;
    }
    break;  // end WAITING
  default:; // unknown state is a problem
  }         // end switch on _state
}

void StepperController::move_stop()
{
  _isr_steps_remaining = min(_isr_top_accel_table_max_index, _isr_steps_remaining);
  _event = STEPPER_CONTROLLER_EVENT_NONE;
}

void StepperController::turn_off()
{
  StepperController::move_stop();
  _event = STEPPER_CONTROLLER_EVENT_TURN_OFF;
}

bool StepperController::is_stopped()
{
  return (_stepsRequested == 0 && _isr_steps_remaining == 0);
}

void StepperController::move(int steps)
{
  _stepsRequested = steps;
  _event = STEPPER_CONTROLLER_EVENT_MOVE;
}

void StepperController::move_extend(int steps)
{
  _stepsRequested = abs(steps) * (_direction ? 1 : -1);
  _event = STEPPER_CONTROLLER_EVENT_MOVE;
}

void StepperController::setSpeedLimitPercent(int percent)
{
  _isr_top_accel_table_max_index = min(_isr_top_accel_table_length - 1, max(0,
                                                                            map(percent, 0, 100, 0, _isr_top_accel_table_length - 1)));
}

int StepperController::getSpeedLimitPercent()
{
  return map(_isr_top_accel_table_max_index, 0, _isr_top_accel_table_length - 1, 0, 100);
}

void StepperController::applySettings(Settings *settings)
{
  setSpeedLimitPercent(settings->stepper_speed_limit_percent);
}

void StepperController::_initTimer()
{
  noInterrupts(); // disable all interrupts
  // TCCR3A bits 7:COM1A1,6:COM1A0; 5:COM1B1,4:COM1B0; 3:COM1C1,2L:COM1C0
  // COMpare output modes (nonPWM) are 00 - normal; OC1x disconnected,
  // 01 toggle OC1x on compare match; 10 OC1x low on compare match; 11 = OC1x high on match
  // TCCT1A bits 1:WGM11, 0: WGM10 combine with bits in TCCR1B
  TCCR3A = 0; // ensure defaults
  // TCCR3B bits 7:ICNC1, 6: ICES1, 5:-, 4:WGM33,3:WGM32, 3:CS32,1:CS31,0:CS30

  TCCR3B = 0; // ensure defaults
  TCNT3 = 0L; // initialize counter to 0
  // OCR3A = compare match register A; set to (16MHz/prescale/desired frequency) - 1
  OCR3A = (16000000L / 64L / 61L) - 1L; // 2nd-to-last term is frequency
  TCCR3B |= (1 << WGM32);               // // Waveform Generation Mode mode => CTC Top=OCR1A
  // CSn2/CSn1/CSn0 = Clock Select. Set to 000=>Stop; 001=>clk/1, 010=>clk/8,
  //                                011=>clock/64, 100=>clk/256; 101 => clk/1024
  TCCR3B |= (0 << CS32); // prescale (CS22 bit)
  TCCR3B |= (1 << CS31); // prescale (CS21 bit)
  TCCR3B |= (1 << CS30); // prescale (CS20 bit)
  // TIMING NOTE: if a 16 prescale were available, then 1 tick would be 1μs;
  //    we need timings around 2ms-7ms, or 2000-7000μs.
  //    Using the 64 prescale, 1 tick is 4μs, so our TOP should range between 500 (fast) and 1750 (slow).
  // Timer/Counter1 Interrupt Mask Register
  // bit 5:Input capture interrupt enable; bit 0: Timer/Counter1 Overflow Interrupt Enable
  // bit 3/2/1: Output Compare C/B/A Match Interrupt Enable
  TIMSK3 |= (1 << OCIE3A); // enable timer compare interrupt
  interrupts();            // enable all interrupts
}

void StepperController::_buildTable()
{
  float f_delay_range = (float)STEPPER_MICROSEC_STEP_DELAY_MAX - (float)STEPPER_MICROSEC_STEP_DELAY_MIN;
  for (unsigned int i = 0; i < StepperController::_isr_top_accel_table_length; i++)
  {
    float p = (float)(i + 1) / (float)StepperController::_isr_top_accel_table_length; // how far are we through the range 0.0-1.0?
    if (p < 0.5)                                                                      // ease-in quadratic, then linear halfway through
    {
      p = p * p * 2.0;
    }
    StepperController::_isr_top_accel_table[i] = (STEPPER_MICROSEC_STEP_DELAY_MAX - (int)(p * p * f_delay_range)) / STEPPER_TIMER_MICROSEC_PER_TICK;
  }
  // timer_top_accel_table_max_index should be read from a setting; must be < timer_top_accel_table_length
  StepperController::_isr_top_accel_table_max_index = StepperController::_isr_top_accel_table_length - 1;
  if (StepperController::_isr_top_accel_table_max_index >= StepperController::_isr_top_accel_table_length)
  {
    StepperController::_isr_top_accel_table_max_index = StepperController::_isr_top_accel_table_length - 1;
  }
}

inline void StepperController::isr()
{
  if (_isr_steps_remaining > 0)
  {
    digitalWrite(STEPPER_PIN_STEP, HIGH);
    delayMicroseconds(STEPPER_PULSE_MICROSECONDS);
    digitalWrite(STEPPER_PIN_STEP, LOW);
    _isr_steps_taken++;
    _isr_steps_remaining--;
    // accelerate if before midpoint
    if (_isr_steps_taken < _isr_steps_remaining &&
        _isr_top_accel_table_current_index < _isr_top_accel_table_max_index)
    {
      _isr_top_accel_table_current_index++;
    }
    // decelerate if isr_steps_remaining < timer_top_accel_table_current_index and isr_steps_taken > timer_steps_midpoint
    else if (
        _isr_steps_remaining < _isr_top_accel_table_current_index && _isr_steps_taken > _isr_steps_remaining && _isr_top_accel_table_current_index > 0)
    {
      _isr_top_accel_table_current_index--;
    }

    // OCR3A = compare match register A is TOP; set to prescale = 64, so 4μs per tick on 16MHz clock
    OCR3A = _isr_top_accel_table[_isr_top_accel_table_current_index];
  } // if stepping
}