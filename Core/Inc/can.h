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


#define slave1
//#define slave2
//#define slave3
//#define slave4

#define idMaster 0x00A

#define idSlave1Burst0 0x010

#define idSlave2Burst0 0x020

#define idSlave3Burst0 0x030

#define idSlave4Burst0 0x040

#define idSlave1ThermistorError 0x050
#define idSlave2ThermistorError 0x051
#define idSlave3ThermistorError 0x052
#define idSlave4ThermistorError 0x053

void sendTemperatureToMaster(float buffer[], uint16_t baseID);
void sendReadingErrorInfoIntoCAN(void);

#endif /* INC_CAN_H_ */
