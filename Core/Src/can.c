/*
 * can.c
 *
 *  Created on: Sep 4, 2025
 *      Author: Guilherme Lettmann
 */
#include "can.h"

uint8_t FDCAN1TxData[8];
FDCAN_TxHeaderTypeDef FDCAN1TxHeader;

void sendTemperatureToMaster(float buffer){
	FDCAN1TxHeader.DataLength = 8;
#ifdef slave1
	FDCAN1TxHeader.Identifier = idSlave1;
#endif
#ifdef slave2
	FDCAN1TxHeader.Identifier = idSlave2;
#endif
#ifdef slave3
	FDCAN1TxHeader.Identifier = idSlave3;
#endif
#ifdef slave4
	FDCAN1TxHeader.Identifier = idSlave4;
#endif

}

