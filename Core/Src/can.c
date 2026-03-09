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

void sendTemperatureToMaster(float buffer[])
{
	for(int i = 0; i < numberOfThermistors; i += 2)
	{
		FDCAN1TxHeader.DataLength = FDCAN_DLC_BYTES_8;
		FDCAN1TxHeader.Identifier = idSlave1Burst0 + (i/2);

		memcpy(&FDCAN1TxData[0], &buffer[i], sizeof(float));
		memcpy(&FDCAN1TxData[4], &buffer[i+1], sizeof(float));

		uint8_t retry = 0;

		while(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1TxHeader,  FDCAN1TxData) != HAL_OK){
			retry++;
			if (retry >= 20){
				Error_Handler();
			}
		}
	}
}

void sendReadingErrorInfoIntoCAN(void){

#ifdef slave1
	FDCAN1TxHeader.Identifier = idSlave1ThermistorError;
#endif
#ifdef slave2
	FDCAN1TxHeader.Identifier = idSlave2ThermistorError;
#endif
#ifdef slave3
	FDCAN1TxHeader.Identifier = idSlave3ThermistorError;
#endif
#ifdef slave4
	FDCAN1TxHeader.Identifier = idSlave4ThermistorError;
#endif

	FDCAN1TxData[0] = 67;
	while(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1TxHeader,  FDCAN1TxData) != HAL_OK){
		static int retry = 0;
		retry++;
		if (retry >= 20){
			Error_Handler();
		}
	}
}
