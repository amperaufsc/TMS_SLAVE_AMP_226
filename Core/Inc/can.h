/*
 * can.h — Interface do módulo de comunicação CAN (FDCAN)
 *
 *  Created on: Sep 4, 2025
 *      Author: Guilherme Lettmann
 *
 *  Este header centraliza todas as definições da rede CAN do TMS:
 *  - Seleção de identidade do slave (qual placa física é esta)
 *  - Mapa de IDs CAN para burst de temperaturas e erros
 *  - Parâmetros de robustez (retry, timeout de falha)
 *  - Tipos de retorno para funções de transmissão
 *  - Struct para enfileiramento de mensagens recebidas (FreeRTOS Queue)
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "main.h"
#include "adc.h"

/* ======== Capacidade da fila de recepção CAN (FreeRTOS Queue) ======== */
#define CAN_RX_QUEUE_SIZE 16   // Suporta até 16 mensagens enfileiradas simultaneamente

/* ================ Seleção de identidade do Slave ===================== */
/* IMPORTANTE: Descomente apenas UM define por placa física.
 * Cada placa Slave possui IDs CAN únicos para evitar colisões no barramento. */
#define slave1
//#define slave2
//#define slave3
//#define slave4

/* =================== Flag de modo de teste =========================== */
/* Quando definido, habilita a drenagem da Message Queue de RX na thread
 * xSendCAN para validar a recepção em modo Internal Loopback do FDCAN.
 * Remover este define para operação normal no carro. */
#define testLoopback

/* ==================== Mapa de IDs CAN ================================ */
/* ID de mensagem da Master (resumo geral do sistema TMS) */
#define idMaster 0x00A

/* IDs base dos bursts de temperatura de cada Slave.
 * Cada Slave envia 8 quadros sequenciais (2 floats por quadro = 16 termistores).
 * Ex: Slave 1 usa IDs 0x010, 0x011, 0x012, ..., 0x017                  */
#define idSlave1Burst0 0x010
#define idSlave2Burst0 0x020
#define idSlave3Burst0 0x030
#define idSlave4Burst0 0x040

/* IDs de erro de leitura de termistor (curto-circuito ou circuito aberto) */
#define idSlave1ThermistorError 0x050
#define idSlave2ThermistorError 0x051
#define idSlave3ThermistorError 0x052
#define idSlave4ThermistorError 0x053

/* ================ Parâmetros de robustez do envio CAN ================ */
#define CAN_TX_RETRY_MAX      20   // Tentativas por quadro antes de desistir (≈20ms)
#define CAN_TX_FAULT_THRESHOLD 10  // Falhas consecutivas antes de acionar Shutdown (SDC)

/* ============= Status de retorno das funções de transmissão ========== */
typedef enum {
	CAN_TX_OK = 0,       // Todos os quadros enviados com sucesso
	CAN_TX_FAIL,         // Falha pontual (quadro descartado, sistema continua operando)
	CAN_TX_FATAL         // Falha crítica: rede CAN inoperante, Shutdown será acionado
} CAN_TxStatus_t;

/* ====================== Protótipos de funções ======================== */
CAN_TxStatus_t sendTemperatureToMaster(float buffer[], uint16_t baseID);
void sendReadingErrorInfoIntoCAN(void);

/* ============= Struct para fila de recepção CAN (Queue) ============== */
/* Encapsula um quadro CAN completo (header + 8 bytes de dados).
 * Usada pela ISR (HAL_FDCAN_RxFifo0Callback) para empurrar mensagens
 * recebidas para a fila do FreeRTOS sem usar variáveis globais. */
typedef struct {
	FDCAN_RxHeaderTypeDef header;   // Header com ID, DLC, timestamp, etc.
	uint8_t data[8];                // Payload de dados (até 8 bytes no CAN clássico)
} CAN_RxMsg_t;

/* Handle da fila de recepção (definida em main.c, acessível globalmente) */
extern osMessageQueueId_t canRxQueueHandle;

#endif /* INC_CAN_H_ */
