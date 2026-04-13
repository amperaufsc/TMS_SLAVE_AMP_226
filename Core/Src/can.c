/**
 * @file can.c
 * @brief CAN (FDCAN) communication module for data transmission to Master.
 *
 * This module manages all outgoing CAN traffic from the Slave:
 * - Periodic transmission of 16 temperature values (8-frame burst).
 * - Transmission of thermistor fault alarms.
 * - Robust retry mechanism with safety-critical failure detection (SDC Shutdown).
 *
 * Safety Architecture:
 * - Each frame has up to CAN_TX_RETRY_MAX attempts (~20ms).
 * - If a frame fails after retries, it is dropped to prevent blocking the RTOS.
 * - If CAN_TX_FAULT_THRESHOLD consecutive frames fail, the system enters 
 *   Error_Handler (triggering SDC Shutdown).
 * - Any successful transmission resets the failure counter.
 *
 * Created on: Sep 4, 2025
 * Author: Guilherme Lettmann
 */

#include "can.h"
#include "string.h"
#include "main.h"
#include "adc.h"

/* ================== External Globals from main.c ===================== */
extern FDCAN_HandleTypeDef hfdcan1;
/** @brief Buffer for outcoming 8-byte payload */
uint8_t FDCAN1TxData[8];
/** @brief Header for transmission (ID, DLC, etc.) */
FDCAN_TxHeaderTypeDef FDCAN1TxHeader;

/** @brief Persistent counter for consecutive TX failures across all channels */
static uint8_t canConsecutiveFailures = 0;

/* ==================== Internal Static Functions ====================== */

/**
 * @brief  Sends a single CAN frame with retry logic and safety monitoring.
 *
 * This is the core transmission function for all slave routines.
 * 
 * Flow:
 * 1. Checks if peripheral is operational.
 * 2. Attempts to add message to hardware FIFO.
 * 3. On full FIFO, waits 1ms and retries (up to CAN_TX_RETRY_MAX).
 * 4. Tracks 'canConsecutiveFailures' for network health monitoring.
 *
 * @param  identifier: Standard CAN ID (11 bits).
 * @param  data: Pointer to 8-byte payload.
 * @return CAN_TX_OK, CAN_TX_FAIL (one frame lost), or CAN_TX_FATAL (Shutdown).
 */
static CAN_TxStatus_t sendSingleFrame(uint16_t identifier, uint8_t *data)
{
	/* Peripheral health check */
	if (hfdcan1.State != HAL_FDCAN_STATE_BUSY)
	{
		canConsecutiveFailures++;
		if (canConsecutiveFailures >= CAN_TX_FAULT_THRESHOLD) {
			return CAN_TX_FATAL;
		}
		return CAN_TX_FAIL;
	}

	/* Setup Header */
	FDCAN1TxHeader.Identifier = identifier;
	FDCAN1TxHeader.DataLength = FDCAN_DLC_BYTES_8;

	uint8_t retry = 0;
	/* Attempt to queue message in hardware FIFO */
	while (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1TxHeader, data) != HAL_OK)
	{
		osDelay(1); // Yield 1ms to other RTOS tasks while waiting for FIFO space
		if (++retry >= CAN_TX_RETRY_MAX)
		{
			/* Network likely congested or disconnected */
			canConsecutiveFailures++;
			if (canConsecutiveFailures >= CAN_TX_FAULT_THRESHOLD) {
				return CAN_TX_FATAL;
			}
			return CAN_TX_FAIL;
		}
	}

	/* Transmission successful: reset safety counter */
	canConsecutiveFailures = 0;
	return CAN_TX_OK;
}

/* ==================== Public Functions =============================== */

CAN_TxStatus_t sendTemperatureToMaster(float buffer[], uint16_t baseID)
{
	/* 16 thermistors / 2 values per frame = 8 frames total */
	uint8_t frames = (numberOfThermistors + 1) / 2;
	CAN_TxStatus_t worstResult = CAN_TX_OK;

	for (uint8_t frame = 0; frame < frames; frame++)
	{
		uint8_t i = frame * 2;

		/* Pack 2 floats into the 8-byte payload.
		 * Use memcpy to follow strict aliasing rules for safety. */
		float temp1 = buffer[i];
		float temp2 = (i + 1 < numberOfThermistors) ? buffer[i + 1] : buffer[i];
		memcpy(&FDCAN1TxData[0], &temp1, sizeof(float)); // Bytes 0-3
		memcpy(&FDCAN1TxData[4], &temp2, sizeof(float)); // Bytes 4-7

		/* IDs are sequential within the burst: baseID, baseID+1, etc. */
		CAN_TxStatus_t result = sendSingleFrame(baseID + frame, FDCAN1TxData);

		if (result == CAN_TX_FATAL)
		{
			Error_Handler(); // Trigger SDC Shutdown and halt core
			return CAN_TX_FATAL;
		}

		/* Record if any single frame was dropped during the burst */
		if (result == CAN_TX_FAIL && worstResult == CAN_TX_OK) {
			worstResult = CAN_TX_FAIL;
		}
	}

	return worstResult;
}

void sendReadingErrorInfoIntoCAN(void)
{
	/* Select pre-configured error ID based on slave board identity */
#if defined(slave1)
	uint16_t errorId = idSlave1ThermistorError;
#elif defined(slave2)
	uint16_t errorId = idSlave2ThermistorError;
#elif defined(slave3)
	uint16_t errorId = idSlave3ThermistorError;
#elif defined(slave4)
	uint16_t errorId = idSlave4ThermistorError;
#else
	#error "No slave defined! Define slave1, slave2, slave3 or slave4 in can.h"
#endif

	/* Pack error code 67 into byte 0 */
	memset(FDCAN1TxData, 0, sizeof(FDCAN1TxData));
	FDCAN1TxData[0] = 67;

	CAN_TxStatus_t result = sendSingleFrame(errorId, FDCAN1TxData);
	if (result == CAN_TX_FATAL) {
		Error_Handler(); // Fatal network error triggers SDC Shutdown
	}
}
