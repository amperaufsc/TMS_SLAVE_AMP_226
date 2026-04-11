/*
 * can.c — Módulo de comunicação CAN (FDCAN) para transmissão de dados
 *
 *  Created on: Sep 4, 2025
 *      Author: Guilherme Lettmann
 *
 *  Este módulo gerencia toda a transmissão CAN do Slave para a Master:
 *  - Envio periódico dos 16 valores de temperatura (8 quadros CAN)
 *  - Envio de alarmes de falha de termistor
 *  - Mecanismo de retry com detecção de falha crítica (Shutdown/SDC)
 *
 *  Arquitetura de segurança:
 *    - Cada quadro CAN tem até CAN_TX_RETRY_MAX tentativas de envio (≈20ms)
 *    - Se um quadro falha, o sistema descarta e segue para o próximo
 *    - Se CAN_TX_FAULT_THRESHOLD falhas CONSECUTIVAS ocorrem sem nenhum
 *      sucesso intermediário, o sistema entra em Error_Handler → Shutdown
 *    - Qualquer envio bem-sucedido zera o contador de falhas
 */
#include "can.h"
#include "string.h"
#include "main.h"
#include "adc.h"

/* ==================== Variáveis globais de TX ======================== */
extern FDCAN_HandleTypeDef hfdcan1;  // Handle do periférico FDCAN1 (gerado pelo CubeMX)
uint8_t FDCAN1TxData[8];             // Buffer de payload para transmissão (8 bytes = CAN clássico)
FDCAN_TxHeaderTypeDef FDCAN1TxHeader; // Header de transmissão (ID, DLC, formato, etc.)

/* Contador persistente de falhas consecutivas de transmissão.
 * Static no escopo do arquivo: visível apenas dentro de can.c,
 * compartilhado entre sendTemperatureToMaster e sendReadingErrorInfoIntoCAN.
 * Qualquer envio bem-sucedido zera este contador automaticamente. */
static uint8_t canConsecutiveFailures = 0;

/* ==================== Funções internas (static) ====================== */

/**
 * @brief  Envia um único quadro CAN com mecanismo de retry e detecção de falha.
 *
 * Esta é a função-base usada por todas as rotinas de transmissão.
 * Centraliza a lógica de retry e o contador de falhas num único local.
 *
 * Fluxo:
 *   1. Verifica se o periférico FDCAN está operacional (HAL_FDCAN_STATE_BUSY)
 *   2. Configura o ID e DLC no header global
 *   3. Tenta inserir a mensagem na fila de TX do hardware (FIFO)
 *   4. Se a fila estiver cheia, espera 1ms e tenta novamente (até CAN_TX_RETRY_MAX vezes)
 *   5. Se todas as tentativas falharem, incrementa o contador de falhas
 *   6. Se o contador atingir CAN_TX_FAULT_THRESHOLD, retorna CAN_TX_FATAL
 *
 * @param  identifier: ID CAN do quadro (11 bits, Standard ID)
 * @param  data:       Ponteiro para os 8 bytes de payload a enviar
 * @retval CAN_TX_OK     → Quadro aceito pelo hardware com sucesso
 *         CAN_TX_FAIL   → Esgotou tentativas para este quadro (descartado)
 *         CAN_TX_FATAL  → Limiar de falhas consecutivas atingido (rede inoperante)
 */
static CAN_TxStatus_t sendSingleFrame(uint16_t identifier, uint8_t *data)
{
	/* Passo 1: Verificar se o periférico FDCAN está em estado operacional.
	 * Após HAL_FDCAN_Start(), o estado deve ser HAL_FDCAN_STATE_BUSY.
	 * Se estiver em outro estado (erro de init, bus-off), não tenta enviar. */
	if (hfdcan1.State != HAL_FDCAN_STATE_BUSY)
	{
		canConsecutiveFailures++;
		if (canConsecutiveFailures >= CAN_TX_FAULT_THRESHOLD)
		{
			return CAN_TX_FATAL;
		}
		return CAN_TX_FAIL;
	}

	/* Passo 2: Configura o header do quadro CAN.
	 * Os campos FDFormat, IdType e BitRateSwitch já foram configurados
	 * uma vez durante MX_FDCAN1_Init() e persistem no header global. */
	FDCAN1TxHeader.Identifier = identifier;
	FDCAN1TxHeader.DataLength = FDCAN_DLC_BYTES_8;

	/* Passo 3: Tenta inserir o quadro na fila de TX do hardware.
	 * HAL_FDCAN_AddMessageToTxFifoQ retorna HAL_OK se a mensagem
	 * foi aceita pelo módulo FDCAN. Retorna HAL_ERROR se a fila
	 * de TX está cheia (todas as 3 mailboxes ocupadas). */
	uint8_t retry = 0;
	while (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1TxHeader, data) != HAL_OK)
	{
		osDelay(1);  // Cede 1ms ao kernel FreeRTOS para processar outras tasks
		if (++retry >= CAN_TX_RETRY_MAX)
		{
			/* Esgotou as tentativas: a fila de TX não liberou espaço */
			canConsecutiveFailures++;
			if (canConsecutiveFailures >= CAN_TX_FAULT_THRESHOLD)
			{
				return CAN_TX_FATAL;  // Rede completamente inoperante
			}
			return CAN_TX_FAIL;  // Descarta este quadro, mas sistema continua
		}
	}

	/* Passo 4: Envio bem-sucedido — zera contador de falhas.
	 * Isso garante que glitches isolados (ruído EMI, colisão momentânea)
	 * não acumulem até atingir o limiar de Shutdown. */
	canConsecutiveFailures = 0;
	return CAN_TX_OK;
}

/* ==================== Funções públicas ================================ */

/**
 * @brief  Envia as 16 temperaturas para a Master em quadros CAN sequenciais.
 *
 * Cada quadro carrega 2 floats (8 bytes), totalizando 8 quadros por burst.
 * Os IDs são sequenciais: baseID+0, baseID+1, ..., baseID+7.
 *
 * Exemplo para Slave 1 (baseID = 0x010):
 *   Frame 0 → ID 0x010 → [temp[0], temp[1]]
 *   Frame 1 → ID 0x011 → [temp[2], temp[3]]
 *   ...
 *   Frame 7 → ID 0x017 → [temp[14], temp[15]]
 *
 * Usa memcpy para copiar os floats para o buffer de TX, evitando violação
 * de strict aliasing (cast float* sobre uint8_t* é undefined behavior no C
 * e pode gerar código incorreto em otimizações -O2/-O3 do GCC).
 *
 * @param  buffer: Array de floats com as temperaturas convertidas (16 valores)
 * @param  baseID: ID base do burst (definido em can.h, ex: idSlave1Burst0)
 * @retval CAN_TX_OK    → Todos os 8 quadros foram enviados com sucesso
 *         CAN_TX_FAIL  → Pelo menos um quadro foi descartado
 *         CAN_TX_FATAL → Limiar de falhas atingido (Error_Handler é chamado internamente)
 */
CAN_TxStatus_t sendTemperatureToMaster(float buffer[], uint16_t baseID)
{
	/* Calcula o número de quadros: 16 termistores / 2 por quadro = 8 quadros */
	uint8_t frames = (numberOfThermistors + 1) / 2;
	CAN_TxStatus_t worstResult = CAN_TX_OK;  // Rastreia o pior resultado do burst

	for (uint8_t frame = 0; frame < frames; frame++)
	{
		uint8_t i = frame * 2;  // Índice do primeiro termistor deste quadro

		/* Monta o payload com memcpy (seguro contra strict aliasing) */
		float temp1 = buffer[i];
		float temp2 = (i + 1 < numberOfThermistors) ? buffer[i + 1] : buffer[i];
		memcpy(&FDCAN1TxData[0], &temp1, sizeof(float));  // Bytes 0-3: temperatura 1
		memcpy(&FDCAN1TxData[4], &temp2, sizeof(float));  // Bytes 4-7: temperatura 2

		/* Envia o quadro usando a função comum de retry */
		CAN_TxStatus_t result = sendSingleFrame(baseID + frame, FDCAN1TxData);

		if (result == CAN_TX_FATAL)
		{
			Error_Handler();  // Aciona Shutdown (GPIO PC8) e trava o processador
			return CAN_TX_FATAL;  // Nunca chega aqui, mas satisfaz o compilador
		}

		/* Registra se pelo menos um quadro falhou neste burst */
		if (result == CAN_TX_FAIL && worstResult == CAN_TX_OK)
		{
			worstResult = CAN_TX_FAIL;
		}
	}

	return worstResult;
}

/**
 * @brief  Envia um quadro CAN de erro de termistor para a Master.
 *
 * Chamada quando checkThermistorConnection() detecta um circuito aberto
 * ou curto-circuito em algum dos 16 sensores NTC.
 *
 * O ID do quadro é selecionado em tempo de compilação pelo define slaveN
 * (configurado em can.h). Usa a mesma lógica de retry e contagem de falhas
 * da sendSingleFrame, mantendo o mecanismo de Shutdown unificado.
 *
 * Payload: Byte 0 = 67 (código de erro de termistor), Bytes 1-7 = 0x00
 */
void sendReadingErrorInfoIntoCAN(void)
{
	/* Seleciona o ID de erro correspondente ao slave ativo */
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

	/* Monta payload: zera tudo e seta o código de erro no byte 0 */
	memset(FDCAN1TxData, 0, sizeof(FDCAN1TxData));
	FDCAN1TxData[0] = 67;

	/* Envia usando o mesmo mecanismo de retry */
	CAN_TxStatus_t result = sendSingleFrame(errorId, FDCAN1TxData);
	if (result == CAN_TX_FATAL)
	{
		Error_Handler();  // Rede CAN totalmente inoperante → Shutdown
	}
}
