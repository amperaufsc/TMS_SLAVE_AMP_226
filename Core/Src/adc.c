/*
 * adc.c
 *
 *  Created on: Sep 3, 2025
 *      Author: Guilherme Lettmann
 */
#include "math.h"
#include "stdlib.h"
#include "adc.h"

float convertBitsToVoltage(uint16_t rawAdcVal){
	return (rawAdcVal*vcc)/adcResolution;
}

float convertVoltageToTemperature(float voltage){
	return C0 + C1 * voltage + C2 * pow(voltage, 2) + C3 * pow(voltage, 3) + C4 *pow(voltage, 4);
}

