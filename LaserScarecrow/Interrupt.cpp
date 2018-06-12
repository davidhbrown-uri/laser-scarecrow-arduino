#include "Interrupt.h"
#include "config.h"

int Interrupt::_frequency;

void Interrupt::init()
{
  applySettings(&currentSettings);
  // initialize timer; define one INTERRUPTS_* timer (not multiple!) in config.h
#ifdef INTERRUPTS_ATmega328P_T2
#ifdef DEBUG_SERIAL
  Serial.println(F("\r\nSetting up Timer 2"));
#endif
  noInterrupts();           // disable all interrupts
  // TCCR2B - Timer/Counter Control Register A (Compare Output Mode, Waveform Generation Mode)
  // bit 7/6: COM2A1/0; bit 5/4: COM2B1/0; bit 3/2:-; bit 1/0: WGM21/0
  TCCR2A = 0; // ensure defaults
  // TCCR2B - Timer/Counter Control Register B (Force Output Compare, Waveform Mode, Clock Scale)
  // bit 7/6: Force Output Compare A/B; bit 5/4:-; bit 3: WGM22; bit 2/1/0: CS22/CS21/CS20
  TCCR2B = 0; // ensure defaults
  // TCNT2 - Timer/Counter Register (8-bit)
  TCNT2 = 0;
  // OCR2A - Output Compare Register A. Interrupt will happen when counter reaches this value
  // Set to: 16000000 / 1024 / desired frequency - 1; minimum usable frequency is about 61Hz.
  OCR2A = 255;  // 61.27 Hz
  TCCR2A |= (1 << WGM21);   // Waveform Generation Mode 2 = CTC TOP=OCRA; WGM2:2/1/0 = 010
  TCCR2B |= (1 << CS22);    // prescale (CS22 alone is /64)
  TCCR2B |= (1 << CS21);    // prescale (CS22 & CS21 is /256)
  TCCR2B |= (1 << CS20);    // prescale (all of CS22&CS21&CS20 = /1024)
  TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt
#endif
  // we probably don't want T1 because it controls the servo
#ifdef INTERRUPTS_ATmega32U4_T1
#ifdef DEBUG_SERIAL
  Serial.println(F("\r\nSetting up Timer 1"));
#endif
  noInterrupts();           // disable all interrupts
  // TCCR1A bits 7:COM1A1,6:COM1A0; 5:COM1B1,4:COM1B0; 3:COM1C1,2L:COM1C0
  // COMpare output modes (nonPWM) are 00 - normal; OC1x disconnected,
  // 01 toggle OC1x on compare match; 10 OC1x low on compare match; 11 = OC1x high on match
  // TCCT1A bits 1:WGM11, 0: WGM10 combine with bits in TCCR1B
  TCCR1A = 0; // ensure defaults
  // TCCR1B bits 7:ICNC1, 6: ICES1, 5:-, 4:WGM13,3:WGM12, 3:CS12,1:CS11,0:CS10
  // CS=Clock Select, table 14-6 on page 133. CSn2/CSn1/CSn0: 000=stopped;
  // 001=no prescale, 010=/8, 011=/64, 100=/256, 101=/1024. (110/111=external clock)
  TCCR1B = 0; // ensure defaults
  TCNT1 = 0L; // initialize counter to 0
  // OCR1A = compare match register A; set to (16MHz/prescale/desired frequency) - 1
  OCR1A = 16000000L / 256L / 61L - 1L;  // 2nd-to-last term is frequency
  TCCR1B |= (1 << WGM12);   // // Waveform Generation Mode mode => CTC Top=OCR1A
  // CSn2/CSn1/CSn0 = Clock Select. Set to 000=>Stop; 001=>clk/1, 010=>clk/8,
  //                                011=>clock/64, 100=>clk/256; 101 => clk/1024
  TCCR1B |= (1 << CS12);    // prescale (CS22 bit)
  TCCR1B |= (0 << CS11);    // prescale (CS21 bit)
  TCCR1B |= (0 << CS10);    // prescale (CS20 bit)
  // Timer/Counter1 Interrupt Mask Register
  // bit 5:Input capture interrupt enable; bit 0: Timer/Counter1 Overflow Interrupt Enable
  // bit 3/2/1: Output Compare C/B/A Match Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
#endif
#ifdef INTERRUPTS_ATmega32U4_T3
#ifdef DEBUG_SERIAL
  Serial.println(F("\r\nSetting up Timer 3"));
#endif
  noInterrupts();           // disable all interrupts
  // TCCR3A bits 7:COM1A1,6:COM1A0; 5:COM1B1,4:COM1B0; 3:COM1C1,2L:COM1C0
  // COMpare output modes (nonPWM) are 00 - normal; OC1x disconnected,
  // 01 toggle OC1x on compare match; 10 OC1x low on compare match; 11 = OC1x high on match
  // TCCT1A bits 1:WGM11, 0: WGM10 combine with bits in TCCR1B
  TCCR3A = 0; // ensure defaults
  // TCCR1B bits 7:ICNC1, 6: ICES1, 5:-, 4:WGM13,3:WGM12, 3:CS12,1:CS11,0:CS10
  // CS=Clock Select, table 14-6 on page 133. CSn2/CSn1/CSn0: 000=stopped;
  // 001=no prescale, 010=/8, 011=/64, 100=/256, 101=/1024. (110/111=external clock)
  TCCR3B = 0; // ensure defaults
  TCNT3 = 0L; // initialize counter to 0
  // OCR1A = compare match register A; set to (16MHz/prescale/desired frequency) - 1
  OCR3A = (16000000L / 256L / 61L) - 1L;  // 2nd-to-last term is frequency
  TCCR3B |= (1 << WGM32);   // // Waveform Generation Mode mode => CTC Top=OCR1A
  // CSn2/CSn1/CSn0 = Clock Select. Set to 000=>Stop; 001=>clk/1, 010=>clk/8,
  //                                011=>clock/64, 100=>clk/256; 101 => clk/1024
  TCCR3B |= (1 << CS32);    // prescale (CS22 bit)stepper motor s
  TCCR3B |= (0 << CS31);    // prescale (CS21 bit)
  TCCR3B |= (0 << CS30);    // prescale (CS20 bit)
  // Timer/Counter1 Interrupt Mask Register
  // bit 5:Input capture interrupt enable; bit 0: Timer/Counter1 Overflow Interrupt Enable
  // bit 3/2/1: Output Compare C/B/A Match Interrupt Enable
  TIMSK3 |= (1 << OCIE3A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
#endif
#ifdef DEBUG_SERIAL
  Serial.print(F("\r\ninitalizeInterruptTimer -- finished timer set up"));
#endif
}

void Interrupt::setFrequency(int frequency)
{
  long scaled = 16000000L / 256L / ((long) frequency) - 1L;

  noInterrupts();           // disable all interrupts
#ifdef INTERRUPTS_ATmega328P_T2
  // OCR2A - Output Compare Register A. Interrupt will happen when counter reaches this value
  // Set to: 16000000 / 1024 / desired frequency - 1; minimum usable frequency is about 61Hz.
  scaled = scaled / 4L;
  if (scaled > 255L) scaled = 255L;
  OCR2A = (byte) scaled;
#endif
  // we probably don't want T1 because it controls the servo
#ifdef INTERRUPTS_ATmega32U4_T10
  // OCR1A = compare match register A; set to (16MHz/prescale/desired frequency) - 1
  OCR1A = scaled;  // 2nd-to-last term is frequency
#endif
#ifdef INTERRUPTS_ATmega32U4_T3
  OCR3A = scaled;  // 2nd-to-last term is frequency
#endif
  _frequency = frequency;
  interrupts();             // enable all interrupts
#ifdef DEBUG_SERIAL
#ifdef DEBUG_INTERRUPT_FREQUENCY
  Serial.print(F("\r\nInterrupt::setInterruptFrequency to "));
  Serial.print(frequency);
  Serial.print(F("Hz: output compare register = "));
  Serial.print(scaled);
#endif
#endif
}
int Interrupt::getFrequency()
{
  return _frequency;
}
void Interrupt::applySettings(Settings *settings)
{
  setFrequency(settings->interrupt_frequency);
}

