/*
 * can.h
 *
 *  Created on: Sep 4, 2025
 *      Author: Guilherme Lettmann
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "main.h"
#include "adc.h"

#define CAN_RX_QUEUE_SIZE 16

#define slave1
//#define slave2
//#define slave3
//#define slave4

#define testLoopback

#define idMaster 0x00A

#define idSlave1Burst0 0x010

#define idSlave2Burst0 0x020

#define idSlave3Burst0 0x030

#define idSlave4Burst0 0x040

#define idSlave1ThermistorError 0x050
#define idSlave2ThermistorError 0x051
#define idSlave3ThermistorError 0x052
#define idSlave4ThermistorError 0x053

/* Parâmetros de robustez do envio CAN */
#define CAN_TX_RETRY_MAX      20   // Tentativas por quadro antes de desistir (20ms)
#define CAN_TX_FAULT_THRESHOLD 10  // Falhas consecutivas antes de acionar Shutdown

/* Status de retorno das funções de transmissão CAN */
typedef enum {
	CAN_TX_OK = 0,
	CAN_TX_FAIL,        // Falha pontual (quadro descartado, mas sistema segue)
	CAN_TX_FATAL         // Falha crítica: rede inoperante, Shutdown acionado
} CAN_TxStatus_t;

CAN_TxStatus_t sendTemperatureToMaster(float buffer[], uint16_t baseID);
void sendReadingErrorInfoIntoCAN(void);

typedef struct {
	FDCAN_RxHeaderTypeDef header;
	uint8_t data[8];
} CAN_RxMsg_t;

extern osMessageQueueId_t canRxQueueHandle;

#endif /* INC_CAN_H_ */
