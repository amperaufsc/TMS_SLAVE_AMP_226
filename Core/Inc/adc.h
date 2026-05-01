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
#define adcResolution 4095
#define vcc 3.3
#define shortCircuitThreshold 100
#define openCircuitThreshhold (adcResolution-100)

/*T(V) - coefficients  MF52 NTC 10K B3950*/
#define C4 (3.477407)
#define C3 (-33.005072)
#define C2 (104.735668)
#define C1 (-161.078689)
#define C0 (128.920577)

typedef enum {
    OK = 0,
    THERM_SHORT,
    THERM_OPEN,
} thermStatus;

uint16_t median3(uint16_t a, uint16_t b, uint16_t c);
uint16_t applyMedianFilter(uint16_t newReading, uint8_t index);
uint16_t applyIIRFilter(uint16_t newReading, uint8_t index);
float convertBitsToVoltage(uint16_t rawAdcVal);
float convertVoltageToTemperature(float voltage);
thermStatus checkThermistorConnection(uint16_t rawAdcVal);
void initTemperatureFilters(uint16_t rawAdcBuffer[]);

#endif /* INC_ADC_H_ */
