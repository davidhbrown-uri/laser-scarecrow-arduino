#include "IrThreshold.h"
#define REFLECTANCE_BINCOUNT 32

uint16_t IrThreshold::reflectance_readings[IR_REFLECTANCE_READINGS];
uint8_t IrThreshold::reflectance_bins[REFLECTANCE_BINCOUNT];
uint16_t IrThreshold::reflectance_minimum = 1023, IrThreshold::reflectance_maximum = 0, IrThreshold::reflectance_range = 0;
uint16_t IrThreshold::lower_threshold = 0, IrThreshold::upper_threshold = 1023;
uint16_t IrThreshold::span_shortest_absent = STEPPER_FULLSTEPS_PER_ROTATION * STEPPER_MICROSTEPPING_DIVISOR, IrThreshold::span_longest_present = 0;

bool IrThreshold::reflectanceVariesWithoutMovement()
{
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_THRESHOLD
  if (serialCanWrite)
    Serial.println(F("\r\nChecking reflectance range with no movement..."));
#endif
#endif
  int highest = 0, lowest = 1023;
  for (int i = 0; i < IR_REFLECTANCE_READINGS; i++)
  {
    delay(1);
    int value = IrReflectanceSensor::read();
    ;
    if (value > highest)
    {
      highest = value;
    }
    if (value < lowest)
    {
      lowest = value;
    }
  }
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_THRESHOLD
  if (serialCanWrite)
  {
    Serial.print(F("Range = "));
    Serial.print(highest - lowest);
    Serial.print(F(": "));
    Serial.print(lowest);
    Serial.print(F("-"));
    Serial.println(highest);
  }
#endif
#endif

  if ((highest - lowest) > IR_REFLECTANCE_RANGE_REQUIRED)
  {
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_THRESHOLD
    if (serialCanWrite)
    {
      Serial.println(F("Reflectance cannot be used: readings too variable without movement (failed sensor?)"));
    }
#endif
#endif
    return true;
  }
  return false;
}

/**
 * @brief make one rotation, recording average reflectance at each step. 
 * Also sets reflectance_minimum, reflectance_maximum, and reflectance_range
 * 
 */
void IrThreshold::scanReflectance()
{
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_THRESHOLD
  if (serialCanWrite)
    Serial.println(F("\r\nScanning for reflectance (scaled 0x0-0xF):"));
#endif
#endif
  StepperController::move_stop();
  while (!StepperController::is_stopped())
  {
    ;
  }
  for (int i = 0; i < IR_REFLECTANCE_READINGS; i++)
  {
    // move backwards to make this behavior easier to see/understand
    StepperController::move(-IR_REFLECTANCE_STEPS_PER_READ);
    while (!StepperController::is_stopped())
    {
      ;
    }
    reflectance_readings[i] = IrReflectanceSensor::readAverage(IR_REFLECTANCE_SCANNING_AVERAGING);
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_THRESHOLD
    if (serialCanWrite)
      Serial.print(reflectance_readings[i] / 64, HEX);
#endif
#endif
    if (reflectance_minimum > reflectance_readings[i])
    {
      reflectance_minimum = reflectance_readings[i];
    }
    if (reflectance_maximum < reflectance_readings[i])
    {
      reflectance_maximum = reflectance_readings[i];
    }
  }
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_THRESHOLD
  if (serialCanWrite)
  {
    Serial.println(); // finish display individual readings
    Serial.print(F("Reflectance readings range from "));
    Serial.print(reflectance_minimum);
    Serial.print(F(" to "));
    Serial.print(reflectance_maximum);
    Serial.println(F("."));
  }
#endif
#endif
  reflectance_range = reflectance_maximum - reflectance_minimum;
}

void IrThreshold::buildReflectanceHistogram()
{
  uint16_t bin_width = reflectance_range / REFLECTANCE_BINCOUNT;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_THRESHOLD
  if (serialCanWrite)
    Serial.println(F("\r\nBuilding reflectance histogram:"));
#endif
#endif
  // initialize the bins
  for (int i = 0; i < REFLECTANCE_BINCOUNT; reflectance_bins[i++] = 0)
    ;
  for (int i = 0; i < IR_REFLECTANCE_READINGS; i++)
  {
    // normalize each value so reflectance_minimum = 0 and find its histogram bin
    int bin = (reflectance_readings[i] - reflectance_minimum) / bin_width;
    bin = constrain(bin, 0, REFLECTANCE_BINCOUNT - 1); // just to be sure!
    reflectance_bins[bin]++;
    /*
      //smooth the histogram by adding 2 to this bin and 1 to adjacent bins (if they exist)
        reflectance_bins[bin] += 2;
        if (bin >= 1) reflectance_bins[bin - 1]++;
        if (bin + 1 < REFLECTANCE_BINCOUNT) reflectance_bins[bin + 1]++;
        */
  }
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_THRESHOLD
  if (serialCanWrite)
  {
    for (int i = 0; i < REFLECTANCE_BINCOUNT; i++)
    {
      Serial.print(F("#"));
      Serial.print(i);
      Serial.print(F(": "));
      Serial.print(i * bin_width + reflectance_minimum);
      Serial.print(F(" - "));
      Serial.print(i * bin_width + bin_width - 1 + reflectance_minimum);
      Serial.print(F(" = "));
      Serial.println(reflectance_bins[i]);
    }
  }
#endif
#endif
}
// Version 1.1 attempts to find bimodal peaks and extents:
void IrThreshold::setReflectanceThreshold()
{
  int maxCount = -1;
  uint8_t peakOneBin, peakOneBinMax, peakOneBinMin;
  uint8_t peakTwoBin, peakTwoBinMax, peakTwoBinMin;
  uint8_t troughBinMin, troughBinMax;
  bool error = false;

  error = reflectanceVariesWithoutMovement();
  if (!error)
  {
    scanReflectance();

    if (reflectance_range < IR_REFLECTANCE_RANGE_REQUIRED)
    {
      error = true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_THRESHOLD
      if (serialCanWrite)
      {
        Serial.println(F("Reflectance cannot be used: insufficient contrast."));
      }
#endif
#endif
    }
  } // if !error

  if (!error)
  {
    buildReflectanceHistogram();
    // find highest peak
    maxCount = -1;
    for (int i = 0; i < REFLECTANCE_BINCOUNT; i++)
    {
      if (reflectance_bins[i] > maxCount)
      {
        maxCount = reflectance_bins[i];
        peakOneBin = i;
      }
    }
    // find extent of first peak
    // consider a bin to be part of a peak if its count is > 1/3 of the peak-side neighbor.
    // changed to 2/3 2020-08-18 because no longer smoothing histogram sides
    peakOneBinMax = peakOneBin;
    while (peakOneBinMax + 1 < REFLECTANCE_BINCOUNT && reflectance_bins[peakOneBinMax + 1] > (2 * reflectance_bins[peakOneBinMax]) / 3)
    {
      peakOneBinMax++;
    }
    peakOneBinMin = peakOneBin;
    while (peakOneBinMin > 0 && reflectance_bins[peakOneBinMin - 1] > (1 * reflectance_bins[peakOneBinMin]) / 3)
    {
      peakOneBinMin--;
    }
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_THRESHOLD
    if (serialCanWrite)
    {
      Serial.println();
      Serial.print(F("Peak One: from bins"));
      Serial.print(peakOneBinMin);
      Serial.print(F("-"));
      Serial.print(peakOneBinMax);
      Serial.print(F(" with peak @ "));
      Serial.println(peakOneBin);
    }
#endif
#endif
    if (peakOneBinMin == 0 && peakOneBinMax + 1 == REFLECTANCE_BINCOUNT)
    {
      error = true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_THRESHOLD
      if (serialCanWrite)
        Serial.println(F("Reflectance histogram peak covers full range; cannot sense tape."));
#endif
#endif
    }

  } // if !error
  if (!error)
  {
    // find peak in remaining range
    maxCount = -1;
    for (int i = 0; i < REFLECTANCE_BINCOUNT; i++)
    {
      if (i < peakOneBinMin || i > peakOneBinMax) // skip peakOne's range
      {
        if (reflectance_bins[i] > maxCount)
        {
          maxCount = reflectance_bins[i];
          peakTwoBin = i;
        }
      } // skip peakOne
    }   // for i in bins
    // find extent of second peak
    // consider a bin to be part of a peak if its count is > 1/3 of the peak-side neighbor.
    // changed to 2/3 since we're no longer smoothing 2020-08-18
    peakTwoBinMax = peakTwoBin;
    while (peakTwoBinMax + 1 < REFLECTANCE_BINCOUNT && reflectance_bins[peakTwoBinMax + 1] > (2 * reflectance_bins[peakTwoBinMax]) / 3)
      peakTwoBinMax++;
    peakTwoBinMin = peakTwoBin;
    while (peakTwoBinMin > 0 && reflectance_bins[peakTwoBinMin - 1] > (1 * reflectance_bins[peakTwoBinMin]) / 3)
      peakTwoBinMin--;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_THRESHOLD
    if (serialCanWrite)
    {
      Serial.print(F("Peak Two: covers bins "));
      Serial.print(peakTwoBinMin);
      Serial.print(F("-"));
      Serial.print(peakTwoBinMax);
      Serial.print(F(" with peak at @ "));
      Serial.println(peakTwoBin);
    }
#endif
#endif
    if (reflectance_bins[peakTwoBin] == 0)
    {
      error = true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_THRESHOLD
      if (serialCanWrite)
        Serial.println(F("No observations outside range of first peak; cannot sense tape."));
#endif
#endif
    }
  } // !error
  if (!error)
  {
    //find trough between peaks
    if (peakOneBin < peakTwoBin)
    {
      troughBinMin = peakOneBinMax + 1;
      troughBinMax = peakTwoBinMin - 1;
    }
    else
    {
      troughBinMin = peakTwoBinMax + 1;
      troughBinMax = peakOneBinMin - 1;
    }
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_THRESHOLD
    if (serialCanWrite)
    {
      Serial.print(F("Trough: bins "));
      Serial.print(troughBinMin);
      Serial.print(F("-"));
      Serial.println(troughBinMax);
    }
#endif
#endif
    if (troughBinMax <= troughBinMin)
    {
      error = true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_THRESHOLD
      if (serialCanWrite)
      {
        Serial.print(F("Peaks overlap; cannot reliably separate high from low reflectance."));
      }
#endif
#endif
    }
    else
    { // can calculate threshold between peaks
      //middle of the trough: value = reflectanceMin + ((troughBinMin + troughBinMax + 1) * (reflectanceRange / REFLECTANCE_BINCOUNT)) / 2;
      // trying to compensate for changing ambient IR, let's try 20% (1/5) into the trough:
      //value = lowest + troughBinMin * ((highest-lowest) / REFLECTANCE_BINCOUNT) + (((troughBinMax) * ((highest-lowest) / REFLECTANCE_BINCOUNT))/5);
      // back to middle with revised tape sensor design? (Summer '20)
      uint16_t trough_min = reflectance_minimum + troughBinMin * (reflectance_range / REFLECTANCE_BINCOUNT);
      uint16_t trough_max = reflectance_minimum + (troughBinMax + 1) * (reflectance_range / REFLECTANCE_BINCOUNT) - 1;
      lower_threshold = trough_min + (IR_REFLECTANCE_LOWER_THRESHOLD_TROUGH_PERCENT * (trough_max - trough_min) / 100);
      upper_threshold = trough_min + (IR_REFLECTANCE_UPPER_THRESHOLD_TROUGH_PERCENT * (trough_max - trough_min) / 100);
      IrReflectanceSensor::setThreshold(lower_threshold, upper_threshold);
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_THRESHOLD
      if (serialCanWrite)
      {
        Serial.print(F("Trough minimum reflectance = "));
        Serial.println(trough_min);
        Serial.print(F("Trough maximum reflectance = "));
        Serial.println(trough_max);
        Serial.print(F("Reflectance thresholds = "));
        Serial.print(lower_threshold);
        Serial.print(F(" - "));
        Serial.println(upper_threshold);
      }
#endif
#endif
    } //else peaks do not overlap
  }   //!error
  IrReflectanceSensor::setDisabled(error);
  if (!error)
  {
    // find shortest usable span (issue #32)
    findShortestAbsentSpan();
    findLongestPresentSpan();
    // when the laser is active, step randomly from...
    currentSettings.stepper_randomsteps_max = ((int)span_shortest_absent) * 2 / 3;              // two thirds the smallest no-tape span forwards
    currentSettings.stepper_randomsteps_min = currentSettings.stepper_randomsteps_max * -5 / 7; // 70% of that backwards
    // when seeking / skipping tape...
    currentSettings.stepper_stepsWhileSeeking = max(
                                                    STEPPER_FULLSTEPS_PER_ROTATION * STEPPER_MICROSTEPPING_DIVISOR / 8, // step at least 1/8 rotation
                                                    ((int)span_longest_present) * 2 / 3)                                // or up to 2/3 the longest detected span of tape
                                                / STEPPER_MICROSTEPPING_DIVISOR;                                        // seeking is done with whole steps
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_THRESHOLD
    if (serialCanWrite)
    {
      Serial.print(F("Random steps will range from "));
      Serial.print(currentSettings.stepper_randomsteps_min);
      Serial.print(F(" to "));
      Serial.println(currentSettings.stepper_randomsteps_max);
      Serial.print("FULL Steps while seeking usable span: ");
      Serial.println(currentSettings.stepper_stepsWhileSeeking);
    }
#endif
#endif
  } //!error
  if (error)
  {
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_THRESHOLD
    if (serialCanWrite)
    {
      Serial.println(F("Laser will be enabled full circle!"));
    }
#endif
#endif
  }
} // findReflectanceThreshold

void IrThreshold::findLongestPresentSpan()
{
  // Reflectance is present when reading is below lower threshold
  // but if IR_REFLECTANCE_INVERT, then we want readings above upper threshold
  uint16_t steps = 0;
  span_longest_present = 0;
  bool in_span = false;
  for (int i = 0; i < IR_REFLECTANCE_READINGS * 2; i++)
  {
    uint16_t value = reflectance_readings[i % IR_REFLECTANCE_READINGS];
    bool present = IR_REFLECTANCE_INVERT ? value >= upper_threshold : value <= lower_threshold;
    if (present)
    {
      if (in_span)
      {
        steps += IR_REFLECTANCE_STEPS_PER_READ;
      }
      else
      {
        steps = IR_REFLECTANCE_STEPS_PER_READ;
        in_span = true;
      }
      if (steps > span_longest_present)
      {
        span_longest_present = min(steps, (STEPPER_FULLSTEPS_PER_ROTATION * STEPPER_MICROSTEPPING_DIVISOR) - IR_REFLECTANCE_MINIMUM_USABLE_SPAN);
      }
    }
    else
    {
      in_span = false;
    }
  }
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_THRESHOLD
  if (serialCanWrite)
  {
    Serial.print("Longest tape-present span: ");
    Serial.println(span_longest_present);
  }
#endif
#endif
}

void IrThreshold::findShortestAbsentSpan()
{
  // Reflectance is absent when reading is above upper threshold
  // but if IR_REFLECTANCE_INVERT, then we want readings below lower threshold
  uint16_t steps = 0;
  // start with full circle and work down from there
  span_shortest_absent = STEPPER_FULLSTEPS_PER_ROTATION * STEPPER_MICROSTEPPING_DIVISOR;
  bool in_span = false;
  for (int i = 0; i < IR_REFLECTANCE_READINGS * 2; i++)
  {
    uint16_t value = reflectance_readings[i % IR_REFLECTANCE_READINGS];
    bool present = IR_REFLECTANCE_INVERT ? value <= lower_threshold : value >= upper_threshold;
    if (present)
    {
      if (in_span)
      {
        if (steps < span_shortest_absent)
        {
          span_shortest_absent = max(steps, IR_REFLECTANCE_MINIMUM_USABLE_SPAN);
        }
      }
      in_span = false;
    }
    else
    {
      if (in_span)
      {
        steps += IR_REFLECTANCE_STEPS_PER_READ;
      }
      else
      {
        in_span = true;
        steps = IR_REFLECTANCE_STEPS_PER_READ;
      }
    }
  }
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE_INIT_READINGS
  if (serialCanWrite)
  {
    Serial.print("Shortest tape-absent span: ");
    Serial.println(span_shortest_absent);
  }
#endif
#endif
}

/* Old version before we were storing a decent set of readings in memory
// attempts to find the shortest usable span to better set random stepping
// if returned value <=0, then do not set stepping range; unable to determine appropriate value
// See github issue #23
int findShortestUsableSpan() {
  // MATCH_STEPS determined experimentally to help ignore jitter in readings
#define MATCH_STEPS 10
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    if(serialCanWrite) { Serial.println(F("Looking for shortest usable span")); }
#endif
#endif
    int stepsInCircle = STEPPER_FULLSTEPS_PER_ROTATION * STEPPER_MICROSTEPPING_DIVISOR;
    int shortestSpan = STEPPER_FULLSTEPS_PER_ROTATION * STEPPER_MICROSTEPPING_DIVISOR; //full circle
    bool currentStepIsPresent, previousStepIsPresent = IrReflectanceSensor::isPresent();
    //first, find some tape, moving forward
    long stepsStepped = 0L;
    int matchingStepCount = 0;
    while(matchingStepCount < MATCH_STEPS && stepsStepped < STEPPER_FULLSTEPS_PER_ROTATION * STEPPER_MICROSTEPPING_DIVISOR) {
      StepperController::setStepsToStep(1);
      while (StepperController::getStepsToStep() != 0) { ; }//wait for it
      stepsStepped++;
      currentStepIsPresent=IrReflectanceSensor::isPresent();
      if(currentStepIsPresent && previousStepIsPresent) { matchingStepCount++; } else { matchingStepCount=0; }
      previousStepIsPresent=currentStepIsPresent;
    }
    if (stepsStepped > stepsInCircle) return -1; //could not find tape
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    if(serialCanWrite) { Serial.println(F("Found tape")); }
#endif
#endif
    //second, find the end of the tape, moving backward (to make behavior easier to recognize)
    stepsStepped = 0L;
    matchingStepCount = 0;
    while(matchingStepCount < MATCH_STEPS && stepsStepped < stepsInCircle) {
      StepperController::setStepsToStep(-1);
      while (StepperController::getStepsToStep() != 0) { ; }//wait for it
      stepsStepped++;
      currentStepIsPresent=IrReflectanceSensor::isPresent();
      if(!currentStepIsPresent && !previousStepIsPresent) { matchingStepCount++; } else { matchingStepCount=0; }
      previousStepIsPresent=currentStepIsPresent;
    }
    if (stepsStepped > stepsInCircle) return -2; //could not find end of tape
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    if(serialCanWrite) { Serial.println(F("Found end of tape")); }
#endif
#endif
   bool inUsableSpan=true;
   int spanLength = matchingStepCount;// we already saw this many usable steps
   //cover the rest of the circle, still moving backward
   matchingStepCount = 0;
   for(int i = spanLength; i < stepsInCircle; i++) {
      StepperController::setStepsToStep(-1);
      while (StepperController::getStepsToStep() != 0) { ; }//wait for it
      currentStepIsPresent=IrReflectanceSensor::isPresent();
      if(currentStepIsPresent == previousStepIsPresent) { matchingStepCount++; } else {matchingStepCount = 0; }
      previousStepIsPresent=currentStepIsPresent;
      if(inUsableSpan) {
        if(currentStepIsPresent && matchingStepCount >= MATCH_STEPS) { // back to tape
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    if(serialCanWrite) { Serial.print(F("Found end of span with length = ")); Serial.println(spanLength); }
#endif
#endif
      if(spanLength>2*MATCH_STEPS) { // span counts toward shortest unless very short
        shortestSpan=min(shortestSpan,spanLength);
      }
      inUsableSpan=false;
      } // found tape
      else {
        spanLength++;
      }// still in usable span
   } // if inUsableSpan
   else { // on tape
    if(!currentStepIsPresent && matchingStepCount >= 2) { // back to span
      spanLength=matchingStepCount;
      inUsableSpan=true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    if(serialCanWrite) { Serial.print(F("Found beginning of another span")); Serial.println(spanLength); }
#endif
#endif    
      } // if we found another span
    } // else not in span
   }//for steps around the circle
  if(inUsableSpan) { // shouldn't happen at end of circle, but maybe?
    shortestSpan=min(shortestSpan,spanLength);
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    if(serialCanWrite) { Serial.print(F("Ended on span with length = ")); Serial.println(spanLength); }
#endif
#endif
  }
#ifdef DEBUG_SERIAL
#ifdef DEBUG_REFLECTANCE
    if(serialCanWrite) { Serial.print(F("Shortest meaningful span is = ")); Serial.println(shortestSpan); }
#endif
#endif
  return max(shortestSpan, IR_REFLECTANCE_MINIMUM_USABLE_SPAN);
} // function findShortestUsableSpan
*/