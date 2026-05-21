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
#define vcc 3.3f                           // Tensão de referência do ADC (Volts) — 'f' evita promoção a double
#define shortCircuitThreshold 100          // ADC < 100 = curto-circuito no termistor
#define openCircuitThreshold (adcResolution-100)   // ADC > 3995 = circuito aberto (cabo rompido)
/* Median filter window length — escolha ímpar; 5 é mais resiliente a bursts EMI que 3 */
#define MEDIAN_WINDOW_LEN 5

/* ============ Método de conversão de temperatura ===================== */
/* Descomente EXATAMENTE UM dos defines abaixo:
 *   USE_FITTING_CURVE   → Polinômio T(V) de 4ª ordem (calibrado empiricamente)
 *   USE_STEINHART_HART  → Equação Beta do NTC (baseada no datasheet)           */
//#define USE_FITTING_CURVE
#define USE_STEINHART_HART

#ifdef USE_FITTING_CURVE
/* ============ Coeficientes do polinômio T(V) de 4ª ordem ============= */
/* Calibrados empiricamente para o modelo MF52 NTC 10K B3950.
 * T(V) = C0 + C1·V + C2·V² + C3·V³ + C4·V⁴                           */
#define C4 (3.477407)
#define C3 (-33.005072)
#define C2 (104.735668)
#define C1 (-161.078689)
#define C0 (128.920577)
#endif

#ifdef USE_STEINHART_HART
/* ============ Parâmetros do NTC (equação Beta) ======================== */
/* Circuito: Vcc ── R_PULLUP ──┬── NTC ── GND
 *                             ADC_pin
 * 1/T = 1/T0 + (1/β) × ln(R/R0)                                       */
#define NTC_BETA       3950.0f       // B-value do MF52 NTC 10K B3950 (K)
#define NTC_R0         10000.0f      // Resistência a 25°C (Ohms)
#define NTC_T0         298.15f       // 25°C em Kelvin
#define R_PULLUP       10000.0f      // Resistor de pull-up do divisor (Ohms)
#endif

#if !defined(USE_FITTING_CURVE) && !defined(USE_STEINHART_HART)
#error "Defina USE_FITTING_CURVE ou USE_STEINHART_HART em adc.h"
#endif

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
