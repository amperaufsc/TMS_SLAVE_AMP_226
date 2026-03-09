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

void sendTemperatureToMaster(float buffer[],uint16_t baseID){

	float *canPayload = (float*)FDCAN1TxData;

	uint8_t frames = (numberOfThermistors + 1) / 2;

	for(uint8_t frame = 0; frame < frames; frame++)
	{
		uint8_t i = frame * 2;

		FDCAN1TxHeader.DataLength = FDCAN_DLC_BYTES_8;
		FDCAN1TxHeader.Identifier = baseID + frame;

		canPayload[0] = buffer[i];
		canPayload[1] = (i + 1 < numberOfThermistors) ? buffer[i+1] : buffer[i];

		uint8_t retry = 0;

		while(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1TxHeader, FDCAN1TxData) != HAL_OK)
		{
			if(++retry >= 20)
			{
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
