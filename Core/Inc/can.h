/**
 * @file can.h
 * @brief Configuration and constants for CAN communication from Slave to
 * Master.
 *
 * This header centralizes all TMS network definitions from the slave's
 * perspective:
 * - Slave identity selection (which hardware board is this).
 * - CAN ID mapping for temperature bursts and error frames.
 * - Robustness parameters (retry count, failure thresholds).
 * - Status types for transmission functions.
 * - Message structure for the RX queue.
 *
 * Created on: Sep 4, 2025
 * Author: Guilherme Lettmann
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "adc.h"
#include "main.h"

/* ================== CAN RX Message Queue Capacity ================== */
/** @brief Maximum number of messages queued simultaneously for processing */
#define CAN_RX_QUEUE_SIZE 16

/* ================ Slave Identity Selection =========================== */
/**
 * IMPORTANT: Uncomment exactly ONE define per physical board.
 * Each Slave board must have a unique ID to avoid bus collisions.
 */
#define slave1
// #define slave2
// #define slave3
// #define slave4

/* ================== Test & Debug Flags =============================== */
/**
 * @brief Enable to validate FDCAN internal loopback reception.
 * Drain the RX message queue inside the xSendCAN thread during testing.
 * Remove this define for production deployment in the vehicle.
 */
// #define testLoopback

/* ================== CAN Identifier Map =============================== */
/** @brief Identifier for messages sent by the Master (System Status) */
#define idMaster 0x00A

/**
 * @brief Base Identifiers for Temperature Bursts.
 * Each slave sends 8 sequential frames (2 floats per frame = 16 thermistors).
 * e.g., Slave 1 uses IDs 0x010, 0x011, 0x012, ..., 0x017.
 */
#define idSlave1Burst0 0x010
#define idSlave2Burst0 0x020
#define idSlave3Burst0 0x030
#define idSlave4Burst0 0x040

/** @brief Identifiers for Thermistor Disconnection faults (Open/Short circuit)
 */
#define idSlave1ThermistorError 0x050
#define idSlave2ThermistorError 0x051
#define idSlave3ThermistorError 0x052
#define idSlave4ThermistorError 0x053

/* ================== Transmission Robustness Parameters ================ */
/** @brief Maximum retries per frame before dropping it (~20ms wait) */
#define CAN_TX_RETRY_MAX 20
/** @brief Consecutive failed frames before triggering SDC Shutdown */
#define CAN_TX_FAULT_THRESHOLD 10

/* ================== Transmission Status Types ======================== */
typedef enum {
  CAN_TX_OK = 0, /**< Frame(s) successfully accepted by hardware */
  CAN_TX_FAIL,   /**< Single frame dropped (system remains active) */
  CAN_TX_FATAL   /**< Safety threshold reached: SDC Shutdown triggered */
} CAN_TxStatus_t;

/* ================== Function Prototypes ============================== */

/**
 * @brief Sends all 16 smoothed temperature values to the Master.
 * @param buffer Array of 16 processed float temperatures.
 * @param baseID Base CAN ID for the transmission burst.
 * @return CAN_TxStatus_t Status of the transmission burst.
 */
CAN_TxStatus_t sendTemperatureToMaster(float buffer[], uint16_t baseID);

/**
 * @brief Broadcasts a critical thermistor reading error to the Master.
 */
void sendReadingErrorInfoIntoCAN(void);

/* ================== CAN RX Message structure for Queuing ============= */
typedef struct {
  FDCAN_RxHeaderTypeDef header; /**< HAL Header (ID, DLC, etc.) */
  uint8_t data[8];              /**< 8-byte data payload */
} CAN_RxMsg_t;

/** @brief Handle for the global RX Message Queue (defined in main.c) */
extern osMessageQueueId_t canRxQueueHandle;

#endif /* INC_CAN_H_ */
