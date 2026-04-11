/*
 * adc.h — Interface do módulo de leitura e processamento dos termistores
 *
 *  Created on: Sep 3, 2025
 *      Author: Guilherme Lettmann
 *
 *  Este módulo implementa toda a cadeia de aquisição de temperatura:
 *  ADC bruto → Filtro Mediana → Filtro IIR → Conversão Bits→Volts→°C
 *
 *  O pipeline de filtragem é:
 *    1. median3() / applyMedianFilter() → Remove spikes de ruído EMI
 *    2. applyIIRFilter()                → Suaviza oscilações residuais
 *    3. convertBitsToVoltage()          → 0-4095 → 0-3.3V
 *    4. convertVoltageToTemperature()   → Polinômio calibrado T(V)
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"
#include "cmsis_os.h"

/* ========================= Constantes do ADC ========================= */
#define numberOfThermistors 16             // Quantidade de sensores NTC no acumulador
#define adcResolution 4095                 // Resolução máxima do ADC de 12 bits (2^12 - 1)
#define vcc 3.3                            // Tensão de referência do ADC (Volts)
#define shortCircuitThreshold 100          // ADC < 100 = curto-circuito no termistor
#define openCircuitThreshhold (adcResolution-100)  // ADC > 3995 = circuito aberto (cabo rompido)

/* ============ Coeficientes do polinômio T(V) de 4ª ordem ============= */
/* Calibrados empiricamente para o modelo de NTC utilizado no acumulador.
 * T(V) = C0 + C1·V + C2·V² + C3·V³ + C4·V⁴                           */
#define C4 (6.03)
#define C3 (-40.82)
#define C2 (105.64)
#define C1 (-156.45)
#define C0 (134.25)

/* ================== Status de integridade do sensor ================== */
typedef enum {
    OK = 0,        // Leitura dentro da faixa válida
    THERM_SHORT,   // Curto-circuito detectado (ADC muito baixo)
    THERM_OPEN,    // Circuito aberto detectado (ADC muito alto)
} thermStatus;

/* ====================== Protótipos de funções ======================== */
uint16_t median3(uint16_t a, uint16_t b, uint16_t c);
uint16_t applyMedianFilter(uint16_t newReading, uint8_t index);
uint16_t applyIIRFilter(uint16_t newReading, uint8_t index);
float convertBitsToVoltage(uint16_t rawAdcVal);
float convertVoltageToTemperature(float voltage);
thermStatus checkThermistorConnection(uint16_t rawAdcVal);
void initTemperatureFilters(uint16_t rawAdcBuffer[]);

#endif /* INC_ADC_H_ */
