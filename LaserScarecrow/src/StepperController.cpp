/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

 */

#include "StepperController.h"
#include "config.h"

volatile int StepperController::_stepsToStep;
volatile bool StepperController::_sleeping;

void StepperController::init()
{
  //set up i/o based on defines in config.h:
  pinMode(STEPPER_PIN_SLEEP, OUTPUT);
  pinMode(STEPPER_PIN_STEP, OUTPUT);
  pinMode(STEPPER_PIN_DIR, OUTPUT);
  pinMode(STEPPER_PIN_MICROSTEP, OUTPUT);
  digitalWrite(STEPPER_PIN_SLEEP, LOW);
  digitalWrite(STEPPER_PIN_STEP, LOW);
  digitalWrite(STEPPER_PIN_DIR, LOW);
  digitalWrite(STEPPER_PIN_MICROSTEP, LOW);
  _direction = true;
  _buildTable();
  stop();
  setStepsToStep(0);
  applySettings(&currentSettings);
  _initTimer();
}
void StepperController::stop()
{
  digitalWrite(STEPPER_PIN_SLEEP, LOW);
  _sleeping = true;
}
inline void StepperController::_wake()
{
  /*
     From A4988 datasheet:
     "When emerging from Sleep mode, in order
     to allow the charge pump to stabilize, provide a delay of 1 ms
     before issuing a Step command."
  */
  if (_sleeping)
  {
    digitalWrite(STEPPER_PIN_SLEEP, HIGH);
    delay(1);
    _sleeping = false;
  }
}
void StepperController::runFullstep()
{
  _wake();
  digitalWrite(STEPPER_PIN_MICROSTEP, LOW);
}
void StepperController::runHalfstep()
{
  _wake();
  digitalWrite(STEPPER_PIN_MICROSTEP, HIGH);
}

void StepperController::update()
{
  if (!_sleeping && _stepsToStep != 0) // supposed to be moving
  {
    if (_stepsToStep > 0) //turning "forwards"
    {
      digitalWrite(STEPPER_PIN_DIR, HIGH);
      _stepsToStep--;
    }
    else // turning "backwards"
    {
      digitalWrite(STEPPER_PIN_DIR, LOW);
      _stepsToStep++;
    }
    // do a step by toggling
    digitalWrite(STEPPER_PIN_STEP, HIGH);
    digitalWrite(STEPPER_PIN_STEP, LOW);
  } // if moving
}

void StepperController::setStepsToStepRandom(int minSteps, int maxSteps)
{
  setStepsToStep(random(minSteps, maxSteps));
#ifdef DEBUG_SERIAL
#ifdef DEBUG_STEPPER
  Serial.print(F("\r\nStepperController::setStepsToStepRandom stepsToStep="));
  Serial.print(getStepsToStep());
  Serial.print(F(" ("));
  Serial.print(minSteps);
  Serial.print(F("..."));
  Serial.print(maxSteps);
  Serial.println(F(")"));
#endif
#endif
}

int StepperController::getStepsToStep()
{
  return _stepsToStep;
}

void StepperController::setStepsToStep(int steps)
{
  _stepsToStep = steps;
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
  for (unsigned int i = 0; i < StepperController::_timer_top_accel_table_length; i++)
  {
    float p = (float)(i + 1) / (float)StepperController::_timer_top_accel_table_length;  // how far are we through the range 0.0-1.0?
    if(p<0.5) // ease-in quadratic, then linear halfway through
    {
      p = p*p*2.0;
    }
    StepperController::_timer_top_accel_table[i] = (STEPPER_MICROSEC_STEP_DELAY_MAX - (int)(p*p * f_delay_range)) / STEPPER_TIMER_MICROSEC_PER_TICK;
  }
  // timer_top_accel_table_max_index should be read from a setting; must be < timer_top_accel_table_length
  StepperController::_timer_top_accel_table_max_index = StepperController::_timer_top_accel_table_length - 1;
  if (StepperController::_timer_top_accel_table_max_index >= StepperController::_timer_top_accel_table_length)
  {
    StepperController::_timer_top_accel_table_max_index = StepperController::_timer_top_accel_table_length - 1;
  }
}

void StepperController::setSpeedLimitPercent(int percent)
{
  _timer_top_accel_table_max_index = min(_timer_top_accel_table_length-1, max(0,  
  map(percent, 0, 100, 0, _timer_top_accel_table_length-1)));
}

int StepperController::getSpeedLimitPercent()
{
  return map(_timer_top_accel_table_max_index, 0, _timer_top_accel_table_length-1, 0, 100);
}

void StepperController::applySettings(Settings *settings)
{
  setSpeedLimitPercent(settings->stepper_speed_limit_percent);
}