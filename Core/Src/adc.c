/*
 * adc.c — Processamento de sinais analógicos dos termistores NTC
 *
 *  Created on: Sep 3, 2025
 *      Author: Guilherme Lettmann
 *
 *  Pipeline de processamento (executado para cada um dos 16 canais):
 *    ADC bruto → Filtro Mediana (3 amostras) → Filtro IIR (α=1/8)
 *              → Conversão Bits→Volts → Conversão Volts→°C
 */
#include "math.h"
#include "stdlib.h"
#include "adc.h"
#include "stdio.h"

/* ======================== Buffers de filtragem ======================== */
/* filteredAdcBuffer: Acumulador do filtro IIR para cada canal.
 * medianBuffer:      Janela circular de 3 amostras para o filtro de mediana.
 * medianIndex:       Ponteiro circular indicando a posição atual no buffer. */
uint16_t filteredAdcBuffer[numberOfThermistors] = {0}, medianBuffer[numberOfThermistors][3] = {0};
uint8_t medianIndex[numberOfThermistors];

/**
 * @brief  Retorna a mediana de 3 valores usando sorting network (3 comparações).
 *
 * Elimina picos espúrios (spikes) de ruído eletromagnético sem
 * introduzir atraso significativo. Tempo de execução constante O(1).
 *
 * @param  a, b, c: Três amostras a serem comparadas
 * @retval O valor do meio (mediana) dos três
 */
uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
	/* Sorting network: ordena 3 elementos com exatamente 3 swaps */
	if(a > b){
		uint16_t t = a;
		a = b;
		b = t;
	}

	if(b > c){
		uint16_t t = b;
		b = c;
		c = t;
	}

	if(a > b){
		uint16_t t = a;
		a = b;
		b = t;
	}

	return b; // b agora contém o valor do meio
}

/**
 * @brief  Aplica filtro de mediana de janela 3 sobre o canal especificado.
 *
 * Mantém um buffer circular de 3 amostras por canal. A cada nova leitura,
 * insere no buffer e retorna a mediana das 3 últimas amostras.
 *
 * @param  newReading: Nova leitura bruta do ADC (0–4095)
 * @param  index:      Índice do canal (0–15, identifica qual termistor)
 * @retval Valor filtrado pela mediana
 */
uint16_t applyMedianFilter(uint16_t newReading, uint8_t index)
{
	/* Insere a nova leitura na posição atual do buffer circular */
	medianBuffer[index][medianIndex[index]] = newReading;

	/* Lê as 3 últimas amostras armazenadas */
	uint16_t a = medianBuffer[index][0];
	uint16_t b = medianBuffer[index][1];
	uint16_t c = medianBuffer[index][2];

	/* Avança o ponteiro circular (0 → 1 → 2 → 0 → ...) */
	medianIndex[index]++;
	if(medianIndex[index] >= 3)
		medianIndex[index] = 0;

	return median3(a,b,c);
}

/**
 * @brief  Aplica filtro IIR (passa-baixa) de 1ª ordem com α = 1/8.
 *
 * Implementado com shift de bits (>> 3) para evitar operações de ponto
 * flutuante. Suaviza as oscilações residuais que o filtro de mediana
 * não captura completamente.
 *
 * Fórmula: y[n] = y[n-1] + (x[n] - y[n-1]) / 8
 *
 * @param  newReading: Valor já filtrado pela mediana
 * @param  index:      Índice do canal (0–15)
 * @retval Valor suavizado pelo IIR
 */
uint16_t applyIIRFilter(uint16_t newReading, uint8_t index){
	filteredAdcBuffer[index] += (newReading - filteredAdcBuffer[index]) >> 3;
	return filteredAdcBuffer[index];
}

/**
 * @brief  Converte valor bruto do ADC (0–4095) para tensão real (0–3.3V).
 *
 * @param  rawAdcVal: Valor bruto do ADC de 12 bits
 * @retval Tensão em Volts (float)
 */
float convertBitsToVoltage(uint16_t rawAdcVal){
	return (rawAdcVal*vcc)/adcResolution;
}

/**
 * @brief  Converte tensão em temperatura (°C) usando polinômio de 4ª ordem.
 *
 * T(V) = C0 + C1·V + C2·V² + C3·V³ + C4·V⁴
 * Coeficientes calibrados para o modelo de NTC utilizado no acumulador.
 *
 * @param  voltage: Tensão lida no divisor de tensão do termistor (Volts)
 * @retval Temperatura em graus Celsius (float)
 */
float convertVoltageToTemperature(float voltage){
	return C0 + C1 * voltage + C2 * pow(voltage, 2) + C3 * pow(voltage, 3) + C4 *pow(voltage, 4);
}

/**
 * @brief  Diagnóstico de integridade do sensor NTC.
 *
 * Verifica se o valor do ADC está dentro da faixa operacional esperada.
 * Valores extremos indicam falha física no circuito do termistor.
 *
 * @param  rawAdcVal: Valor bruto do ADC de 12 bits
 * @retval OK          → Sensor funcionando normalmente
 *         THERM_SHORT → Curto-circuito (ADC < 100)
 *         THERM_OPEN  → Circuito aberto / cabo rompido (ADC > 3995)
 */
thermStatus checkThermistorConnection(uint16_t rawAdcVal){
	if (rawAdcVal <= shortCircuitThreshold){
		return THERM_SHORT;
	}
	if (rawAdcVal >= openCircuitThreshhold){
		return THERM_OPEN;
	}
	return OK;
}

/**
 * @brief  Inicializa os buffers dos filtros com a primeira leitura real do ADC.
 *
 * Deve ser chamada apenas UMA VEZ, na primeira iteração do loop de leitura.
 * Preenche o buffer circular da mediana com o valor inicial e seta o
 * acumulador do filtro IIR, evitando um transitório de aquecimento longo
 * no startup onde o filtro partiria de zero.
 *
 * @param  rawAdcBuffer: Array com a primeira leitura DMA dos 16 canais
 */
void initTemperatureFilters(uint16_t rawAdcBuffer[])
{
    for(int i = 0; i < numberOfThermistors; i++)
    {
        /* Seta o acumulador do IIR com o valor real do ADC */
        filteredAdcBuffer[i] = rawAdcBuffer[i];

        /* Preenche todas as 3 posições da mediana com o mesmo valor inicial */
        for(int j = 0; j < 3; j++)
        {
            medianBuffer[i][j] = rawAdcBuffer[i];
        }

        /* Reseta o ponteiro circular */
        medianIndex[i] = 0;
    }
}
