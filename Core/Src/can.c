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

void sendTemperatureToMaster0(float buffer[]){

	FDCAN1TxHeader.DataLength = 8;

#ifdef slave1
	FDCAN1TxHeader.Identifier = idSlave1Burst0;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave2
	FDCAN1TxHeader.Identifier = idSlave2Burst0;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave3
	FDCAN1TxHeader.Identifier = idSlave3Burst0;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave4
	FDCAN1TxHeader.Identifier = idSlave4Burst0;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));

#endif

	while(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1TxHeader,  FDCAN1TxData)){
		int count = 0;
		count++;
		if (count >= 20){
			Error_Handler();
		}
	}
}

void sendTemperatureToMaster1(float buffer[]){

	FDCAN1TxHeader.DataLength = 8;

#ifdef slave1
	FDCAN1TxHeader.Identifier = idSlave1Burst1;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave2
	FDCAN1TxHeader.Identifier = idSlave2Burst2;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave3
	FDCAN1TxHeader.Identifier = idSlave3Burst3;
	memcpy(&FDCAN1TxData[0], &buffer[0], size3f(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave4
	FDCAN1TxHeader.Identifier = idSlave4Burst4;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));

#endif

	while(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1TxHeader,  FDCAN1TxData)){
		int count = 0;
		count++;
		if (count >= 20){
			Error_Handler();
		}
	}
}

void sendTemperatureToMaster2(float buffer[]){

	FDCAN1TxHeader.DataLength = 8;

#ifdef slave1
	FDCAN1TxHeader.Identifier = idSlave1Burst2;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave2
	FDCAN1TxHeader.Identifier = idSlave2Burst2;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave3
	FDCAN1TxHeader.Identifier = idSlave3Burst2;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave4
	FDCAN1TxHeader.Identifier = idSlave4Burst2;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));

#endif

	while(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1TxHeader,  FDCAN1TxData)){
		int count = 0;
		count++;
		if (count >= 20){
			Error_Handler();
		}
	}
}

void sendTemperatureToMaster3(float buffer[]){

	FDCAN1TxHeader.DataLength = 8;

#ifdef slave1
	FDCAN1TxHeader.Identifier = idSlave1Burst3;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave2
	FDCAN1TxHeader.Identifier = idSlave2Burst3;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave3
	FDCAN1TxHeader.Identifier = idSlave3Burst3;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave4
	FDCAN1TxHeader.Identifier = idSlave4Burst3;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));

#endif

	while(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1TxHeader,  FDCAN1TxData)){
		int count = 0;
		count++;
		if (count >= 20){
			Error_Handler();
		}
	}
}

void sendTemperatureToMaster4(float buffer[]){

	FDCAN1TxHeader.DataLength = 8;

#ifdef slave1
	FDCAN1TxHeader.Identifier = idSlave1Burst4;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave2
	FDCAN1TxHeader.Identifier = idSlave2Burst4;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave3
	FDCAN1TxHeader.Identifier = idSlave3Burst4;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave4
	FDCAN1TxHeader.Identifier = idSlave4Burst4;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));

#endif

	while(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1TxHeader,  FDCAN1TxData)){
		int count = 0;
		count++;
		if (count >= 20){
			Error_Handler();
		}
	}
}

void sendTemperatureToMaster5(float buffer[]){

	FDCAN1TxHeader.DataLength = 8;

#ifdef slave1
	FDCAN1TxHeader.Identifier = idSlave1Burst5;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave2
	FDCAN1TxHeader.Identifier = idSlave2Burst5;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave3
	FDCAN1TxHeader.Identifier = idSlave3Burst5;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave4
	FDCAN1TxHeader.Identifier = idSlave4Burst5;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));

#endif

	while(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1TxHeader,  FDCAN1TxData)){
		int count = 0;
		count++;
		if (count >= 20){
			Error_Handler();
		}
	}
}

void sendTemperatureToMaster6(float buffer[]){

	FDCAN1TxHeader.DataLength = 8;

#ifdef slave1
	FDCAN1TxHeader.Identifier = idSlave1Burst6;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave2
	FDCAN1TxHeader.Identifier = idSlave2Burst6;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave3
	FDCAN1TxHeader.Identifier = idSlave3Burst6;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave4
	FDCAN1TxHeader.Identifier = idSlave4Burst6;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));

#endif

	while(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1TxHeader,  FDCAN1TxData)){
		int count = 0;
		count++;
		if (count >= 20){
			Error_Handler();
		}
	}
}

void sendTemperatureToMaster7(float buffer[]){

	FDCAN1TxHeader.DataLength = 8;

#ifdef slave1
	FDCAN1TxHeader.Identifier = idSlave1Burst7;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave2
	FDCAN1TxHeader.Identifier = idSlave2Burst7;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave3
	FDCAN1TxHeader.Identifier = idSlave3Burst7;
	memcpy(&FDCAN1TxData[0], &buffer[0], size7of(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));
#endif

#ifdef slave4
	FDCAN1TxHeader.Identifier = idSlave4Burst7;
	memcpy(&FDCAN1TxData[0], &buffer[0], sizeof(float));
	memcpy(&FDCAN1TxData[4], &buffer[1], sizeof(float));

#endif

	while(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1TxHeader,  FDCAN1TxData)){
		int count = 0;
		count++;
		if (count >= 20){
			Error_Handler();
		}
	}
}
