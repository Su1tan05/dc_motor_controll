/*
 * LowpassFilter.cpp
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 */

#include "LowpassFilter.h"

using namespace std;

/**
 * Creates a LowpassFilter object with a default corner frequency of 1000 [rad/s].
 */
LowpassFilter::LowpassFilter() {
    
    period = 1.0f;
    frequency = 1000.0f;
    
    a11 = (1.0f+frequency*period)*exp(-frequency*period);
    a12 = period*exp(-frequency*period);
    a21 = -frequency*frequency*period*exp(-frequency*period);
    a22 = (1.0f-frequency*period)*exp(-frequency*period);
    b1 = (1.0f-(1.0f+frequency*period)*exp(-frequency*period))/frequency/frequency;
    b2 = period*exp(-frequency*period);
    
    x1 = 0.0f;
    x2 = 0.0f;
}

/**
 * Deletes the LowpassFilter object.
 */
LowpassFilter::~LowpassFilter() {}

/**
 * Resets the filtered value to zero.
 */
void LowpassFilter::reset() {
    
    x1 = 0.0f;
    x2 = 0.0f;
}

/**
 * Resets the filtered value to a given value.
 * @param value the value to reset the filter to.
 */
void LowpassFilter::reset(float value) {
    
    x1 = value/frequency/frequency;
    x2 = (x1-a11*x1-b1*value)/a12;
}

/**
 * Sets the sampling period of the filter.
 * This is typically the sampling period of the realtime thread of a controller that uses this filter.
 * @param the sampling period, given in [s].
 */
void LowpassFilter::setPeriod(float period) {
    
    this->period = period;
    
    a11 = (1.0f+frequency*period)*exp(-frequency*period);
    a12 = period*exp(-frequency*period);
    a21 = -frequency*frequency*period*exp(-frequency*period);
    a22 = (1.0f-frequency*period)*exp(-frequency*period);
    b1 = (1.0f-(1.0f+frequency*period)*exp(-frequency*period))/frequency/frequency;
    b2 = period*exp(-frequency*period);
}

/**
 * Sets the corner frequency of this filter.
 * @param frequency the corner frequency of the filter in [rad/s].
 */
void LowpassFilter::setFrequency(float frequency) {
    
    this->frequency = frequency;
    
    a11 = (1.0f+frequency*period)*exp(-frequency*period);
    a12 = period*exp(-frequency*period);
    a21 = -frequency*frequency*period*exp(-frequency*period);
    a22 = (1.0f-frequency*period)*exp(-frequency*period);
    b1 = (1.0f-(1.0f+frequency*period)*exp(-frequency*period))/frequency/frequency;
    b2 = period*exp(-frequency*period);
}

/**
 * Gets the current corner frequency of this filter.
 * @return the current corner frequency in [rad/s].
 */
float LowpassFilter::getFrequency() {
    
    return frequency;
}

/**
 * Filters a value.
 * @param value the original unfiltered value.
 * @return the filtered value.
 */
float LowpassFilter::filter(float value) {

    float x1old = x1;
    float x2old = x2;
    
    x1 = a11*x1old+a12*x2old+b1*value;
    x2 = a21*x1old+a22*x2old+b2*value;
    
    return frequency*frequency*x1;
}