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
#define idSlave1 0x001
#define idSlave2 0x002
#define idSlave3 0x003
#define idSlave4 0x004

void sendTemperatureToMaster(float buffer);

#endif /* INC_CAN_H_ */
