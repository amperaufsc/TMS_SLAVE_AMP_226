/*
 * adc.h
 *
 *  Created on: Sep 3, 2025
 *      Author: Guilherme Lettmann
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"
#include "cmsis_os.h"

#define numberOfThermistors 16
#define windowSize 10
#define adcResolution 4095
#define VREF 3.3
#define shortCircuitThreshold 50
#define openCircuitThreshhold (adcResolution-50)

/*T(V) - coefficients*/
#define C4 (6.03)
#define C3 (-40.82)
#define C2 (105.64)
#define C1 (-156.45)
#define C0 (134.25)

typedef enum {
    OK = 0,
    THERM_SHORT,
    THERM_OPEN,
} thermStatus;

float convertBitsToVoltage(uint16_t rawAdcVal);
float convertVoltageToTemperature(float voltage);
void initializeHistory();
void applyMovingAverageFilter(float rawReadings[]);
thermStatus checkThermistorConnection(uint16_t rawAdcVal);
#endif /* INC_ADC_H_ */
