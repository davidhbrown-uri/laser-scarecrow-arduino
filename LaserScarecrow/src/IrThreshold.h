#pragma once

#include <Arduino.h>
#include "config.h"
#include "IrReflectanceSensor.h"
#include "StepperController.h"
#include "Settings.h"

extern Settings currentSettings;
extern bool serialCanWrite;

class IrThreshold {
    static uint16_t reflectance_readings[];
    static uint8_t reflectance_bins[];
    static uint16_t reflectance_minimum;
    static uint16_t reflectance_maximum;
    static uint16_t reflectance_range;
    static uint16_t lower_threshold;
    static uint16_t upper_threshold;
    static uint16_t span_shortest_absent;
    static uint16_t span_longest_present;
    static void findShortestAbsentSpan();
    static void findLongestPresentSpan();
    static bool reflectanceVariesWithoutMovement();
    static void scanReflectance();
    static void buildReflectanceHistogram();
    public:
    static void setReflectanceThreshold();
};