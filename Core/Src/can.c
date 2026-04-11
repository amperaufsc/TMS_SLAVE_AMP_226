/*
 * can.c
 *
 *  Created on: Sep 4, 2025
 *      Author: Guilherme Lettmann
 */
#include "can.h"
#include "string.h"
#include "main.h"
#include "adc.h"

extern FDCAN_HandleTypeDef hfdcan1;
uint8_t FDCAN1TxData[8];
FDCAN_TxHeaderTypeDef FDCAN1TxHeader;

static uint8_t canConsecutiveFailures = 0;

/**
 * @brief  Envia um único quadro CAN com retry e atualiza o contador de falhas.
 * @param  identifier: ID CAN do quadro (11 bits standard)
 * @param  data: Ponteiro para os 8 bytes de payload
 * @retval CAN_TX_OK     se o quadro foi aceito pelo hardware
 *         CAN_TX_FAIL   se esgotou as tentativas para este quadro
 *         CAN_TX_FATAL  se o limiar de falhas consecutivas foi atingido (Shutdown)
 */
static CAN_TxStatus_t sendSingleFrame(uint16_t identifier, uint8_t *data)
{
	if (hfdcan1.State != HAL_FDCAN_STATE_BUSY)
	{
		canConsecutiveFailures++;
		if (canConsecutiveFailures >= CAN_TX_FAULT_THRESHOLD)
		{
			return CAN_TX_FATAL;
		}
		return CAN_TX_FAIL;
	}

	FDCAN1TxHeader.Identifier = identifier;
	FDCAN1TxHeader.DataLength = FDCAN_DLC_BYTES_8;

	uint8_t retry = 0;
	while (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1TxHeader, data) != HAL_OK)
	{
		osDelay(1);
		if (++retry >= CAN_TX_RETRY_MAX)
		{
			canConsecutiveFailures++;
			if (canConsecutiveFailures >= CAN_TX_FAULT_THRESHOLD)
			{
				return CAN_TX_FATAL;
			}
			return CAN_TX_FAIL; // Descarta este quadro, mas sistema continua
		}
	}

	canConsecutiveFailures = 0;
	return CAN_TX_OK;
}

/**
 * @brief  Envia as 16 temperaturas para a Master em quadros CAN sequenciais.
 *
 * Cada quadro carrega 2 floats (8 bytes), totalizando 8 quadros por burst.
 * Usa memcpy para evitar violação de strict aliasing (undefined behavior).
 *
 * @param  buffer: Array de floats com as temperaturas convertidas
 * @param  baseID: ID base do burst (ex: 0x010 para Slave 1)
 * @retval CAN_TX_OK    se todos os quadros foram enviados
 *         CAN_TX_FAIL  se pelo menos um quadro foi descartado
 *         CAN_TX_FATAL se o limiar de falhas foi atingido (chamador deve acionar Shutdown)
 */
CAN_TxStatus_t sendTemperatureToMaster(float buffer[], uint16_t baseID)
{
	uint8_t frames = (numberOfThermistors + 1) / 2;
	CAN_TxStatus_t worstResult = CAN_TX_OK;

	for (uint8_t frame = 0; frame < frames; frame++)
	{
		uint8_t i = frame * 2;

		/* Monta o payload com memcpy (seguro contra strict aliasing) */
		float temp1 = buffer[i];
		float temp2 = (i + 1 < numberOfThermistors) ? buffer[i + 1] : buffer[i];
		memcpy(&FDCAN1TxData[0], &temp1, sizeof(float));
		memcpy(&FDCAN1TxData[4], &temp2, sizeof(float));

		CAN_TxStatus_t result = sendSingleFrame(baseID + frame, FDCAN1TxData);

		if (result == CAN_TX_FATAL)
		{
			Error_Handler(); // Aciona Shutdown imediatamente
			return CAN_TX_FATAL; // Nunca chega aqui (Error_Handler trava), mas satisfaz o compilador
		}

		if (result == CAN_TX_FAIL && worstResult == CAN_TX_OK)
		{
			worstResult = CAN_TX_FAIL; // Registra que pelo menos um quadro falhou
		}
	}

	return worstResult;
}

/**
 * @brief  Envia um quadro de erro de leitura de termistor para a Master.
 *
 * O ID é selecionado em tempo de compilação pelo define slaveN.
 * Usa o mesmo mecanismo de retry da sendSingleFrame.
 */
void sendReadingErrorInfoIntoCAN(void)
{
#if defined(slave1)
	uint16_t errorId = idSlave1ThermistorError;
#elif defined(slave2)
	uint16_t errorId = idSlave2ThermistorError;
#elif defined(slave3)
	uint16_t errorId = idSlave3ThermistorError;
#elif defined(slave4)
	uint16_t errorId = idSlave4ThermistorError;
#else
	#error "Nenhum slave definido! Defina slave1, slave2, slave3 ou slave4 em can.h"
#endif

	memset(FDCAN1TxData, 0, sizeof(FDCAN1TxData));
	FDCAN1TxData[0] = 67; // Código de erro de termistor

	CAN_TxStatus_t result = sendSingleFrame(errorId, FDCAN1TxData);
	if (result == CAN_TX_FATAL)
	{
		Error_Handler();
	}
}
