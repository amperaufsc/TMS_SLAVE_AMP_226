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
/** @brief Circular buffer for N-tap median window (see MEDIAN_WINDOW_LEN in adc.h) */
uint16_t medianBuffer[numberOfThermistors][MEDIAN_WINDOW_LEN] = {0};
/** @brief Circular index for the median window buffer */
uint8_t medianIndex[numberOfThermistors];

/**
 * @brief Sorting network for 5 values — returns the median.
 *        Uses 9 comparisons (Knuth's optimal 5-element network).
 *        Operates on a local copy so the caller's buffer is preserved.
 */
static uint16_t median5(uint16_t v0, uint16_t v1, uint16_t v2, uint16_t v3, uint16_t v4)
{
	uint16_t a = v0, b = v1, c = v2, d = v3, e = v4;
	#define SWAP_IF(x,y) do { if((x) > (y)) { uint16_t _t = (x); (x) = (y); (y) = _t; } } while(0)
	SWAP_IF(a, b);
	SWAP_IF(c, d);
	SWAP_IF(a, c);
	SWAP_IF(b, d);
	SWAP_IF(b, c);
	SWAP_IF(a, e);
	SWAP_IF(a, c);
	SWAP_IF(b, e);
	SWAP_IF(b, c);
	#undef SWAP_IF
	return c;
}

/**
 * @brief 3-element median (kept for callers that want a cheaper filter).
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
 * @brief Applies an N-tap (MEDIAN_WINDOW_LEN) median filter to a specific channel.
 *        5-tap kills EMI bursts up to 2 consecutive bad samples (3-tap only kills 1).
 */
uint16_t applyMedianFilter(uint16_t newReading, uint8_t index)
{
	/* Insert new reading into the circular buffer */
	medianBuffer[index][medianIndex[index]] = newReading;

	/* Advance circular pointer */
	medianIndex[index]++;
	if(medianIndex[index] >= MEDIAN_WINDOW_LEN) {
		medianIndex[index] = 0;
	}

	return median5(medianBuffer[index][0],
	               medianBuffer[index][1],
	               medianBuffer[index][2],
	               medianBuffer[index][3],
	               medianBuffer[index][4]);
}

uint16_t applyIIRFilter(uint16_t newReading, uint8_t index){
	/* y[n] = y[n-1] + (x[n] - y[n-1]) / 8  (alpha = 1/8)
	 * Cálculo da diff em int32 + divisão signed evita o shift-on-signed UB e
	 * deixa a intenção explícita (compilador gera o mesmo ASR no ARM). */
	int32_t diff = (int32_t)newReading - (int32_t)filteredAdcBuffer[index];
	filteredAdcBuffer[index] = (uint16_t)((int32_t)filteredAdcBuffer[index] + (diff / 8));
	return filteredAdcBuffer[index];
}

float convertBitsToVoltage(uint16_t rawAdcVal){
	return ((float)rawAdcVal * vcc) / (float)adcResolution;
}

float convertVoltageToTemperature(float voltage){
#ifdef USE_FITTING_CURVE
	/* Polinômio T(V) de 4ª ordem via Horner — 3 mul + 4 add, sem chamadas a pow().
	 * T(V) = C0 + V·(C1 + V·(C2 + V·(C3 + V·C4))) */
	return C0 + voltage * (C1 + voltage * (C2 + voltage * (C3 + voltage * C4)));
#endif

#ifdef USE_STEINHART_HART
	/* Equação Beta: ADC voltage → Resistência → Temperatura */
	if (voltage <= 0.0f || voltage >= vcc) return -999.0f;

	/* R_ntc = R_pullup × V / (Vcc − V)  [divisor de tensão com NTC pull-down] */
	float R_ntc = R_PULLUP * voltage / (vcc - voltage);

	/* 1/T = 1/T0 + (1/β) × ln(R/R0) */
	float tempK = 1.0f / ( (1.0f / NTC_T0) + (1.0f / NTC_BETA) * logf(R_ntc / NTC_R0) );

	return tempK - 273.15f;  /* Kelvin → Celsius */
#endif
}

thermStatus checkThermistorConnection(uint16_t rawAdcVal){
	/* Extreme values outside operational range indicate physical faults */
	if (rawAdcVal <= shortCircuitThreshold) {
		return THERM_SHORT;
	}
	if (rawAdcVal >= openCircuitThreshold) {
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

        /* Seed all median window positions to prevent startup spikes */
        for(int j = 0; j < MEDIAN_WINDOW_LEN; j++) {
            medianBuffer[i][j] = rawAdcBuffer[i];
        }

        medianIndex[i] = 0;
    }
}
