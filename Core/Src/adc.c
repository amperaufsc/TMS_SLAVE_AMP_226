/**
 * @file adc.c
 * @brief Signal processing logic for NTC thermistor data.
 *
 * Processing Pipeline:
 * Raw ADC -> Median Filter (3 samples) -> IIR Filter (alpha=1/8)
 *         -> Voltage Conversion -> Thermal Unit Conversion
 *
 * Created on: Sep 3, 2025
 * Author: Guilherme Lettmann
 */

#include "math.h"
#include "stdlib.h"
#include "adc.h"
#include "stdio.h"

/* ======================== Filtering Buffers =========================== */
/** @brief IIR filter accumulator for each channel */
uint16_t filteredAdcBuffer[numberOfThermistors] = {0};
/** @brief Circular buffer for 3-tap median window */
uint16_t medianBuffer[numberOfThermistors][3] = {0};
/** @brief Circular index for the median window buffer */
uint8_t medianIndex[numberOfThermistors];

/**
 * @brief Sorting network for 3 values.
 * @param a, b, c Input values to be sorted.
 * @return The median value of the three inputs.
 */
uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
	/* Sorting network: sorts 3 elements using exactly 3 comparisons */
	if(a > b){
		uint16_t t = a; a = b; b = t;
	}
	if(b > c){
		uint16_t t = b; b = c; c = t;
	}
	if(a > b){
		uint16_t t = a; a = b; b = t;
	}
	return b; // Median is now in b
}

/**
 * @brief Applies a 3-tap median filter to a specific channel.
 * @param newReading The latest raw ADC sample.
 * @param index The index of the thermistor channel.
 * @return The filtered value after median processing.
 */
uint16_t applyMedianFilter(uint16_t newReading, uint8_t index)
{
	/* Insert new reading into the circular buffer */
	medianBuffer[index][medianIndex[index]] = newReading;

	/* Access last 3 samples */
	uint16_t a = medianBuffer[index][0];
	uint16_t b = medianBuffer[index][1];
	uint16_t c = medianBuffer[index][2];

	/* Advance circular pointer (0 -> 1 -> 2 -> 0) */
	medianIndex[index]++;
	if(medianIndex[index] >= 3) {
		medianIndex[index] = 0;
	}

	return median3(a,b,c);
}

uint16_t applyIIRFilter(uint16_t newReading, uint8_t index){
	/* Implementation uses bit-shifting for efficiency (alpha = 1/8)
	 * Formula: y[n] = y[n-1] + (x[n] - y[n-1]) / 8 */
	filteredAdcBuffer[index] += (newReading - filteredAdcBuffer[index]) >> 3;
	return filteredAdcBuffer[index];
}

float convertBitsToVoltage(uint16_t rawAdcVal){
	return (rawAdcVal*vcc)/adcResolution;
}

float convertVoltageToTemperature(float voltage){
	/* Apply calibrated 4th order polynomial for thermal conversion */
	return C0 + C1 * voltage + C2 * pow(voltage, 2) + C3 * pow(voltage, 3) + C4 *pow(voltage, 4);
}

thermStatus checkThermistorConnection(uint16_t rawAdcVal){
	/* Extreme values outside operational range indicate physical faults */
	if (rawAdcVal <= shortCircuitThreshold) {
		return THERM_SHORT;
	}
	if (rawAdcVal >= openCircuitThreshhold) {
		return THERM_OPEN;
	}
	return OK;
}

void initTemperatureFilters(uint16_t rawAdcBuffer[])
{
    for(int i = 0; i < numberOfThermistors; i++)
    {
        /* Seed IIR accumulator with initial real value */
        filteredAdcBuffer[i] = rawAdcBuffer[i];

        /* Seed all 3 median window positions to prevent startup spikes */
        for(int j = 0; j < 3; j++) {
            medianBuffer[i][j] = rawAdcBuffer[i];
        }

        medianIndex[i] = 0;
    }
}
