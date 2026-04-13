# TMS Slave Unit - Firmware Documentation

This repository contains the firmware for the **TMS Slave Unit** (based on STM32G4). Each unit is responsible for monitoring 16 NTC thermistors and reporting the processed data to the Master unit over the CAN bus.

## 1. System Architecture

The Slave firmware operates on **FreeRTOS** and focuses on high-precision signal acquisition and robust data reporting.

### Core Tasks
| Task Name | Priority | Frequency | Purpose |
| :--- | :--- | :--- | :--- |
| `xReadTemp` | Normal | 10 Hz | Manages the DSP pipeline: high-speed ADC sampling, filtering, and sensor health diagnostics. |
| `xSendCAN` | Above Normal | 10 Hz | Handles the CAN reporting protocol. Bursts 16 temperature values in 8 frames. |

## 2. Thermal Acquisition Pipeline

To ensure reliable readings in high-EMI environments (like near an inverter), the firmware implements a 4-step Digital Signal Processing (DSP) chain:
1. **Median Filter (Window 3)**: A sorting-network-based filter that eliminates transient voltage spikes (impulse noise).
2. **IIR Low-Pass Filter (α=1/8)**: A recursive filter that smooths the residual noise floor.
3. **Rational Conversion**: ADC counts are mapped to voltage (0-3.3V).
4. **Thermal Model**: A calibrated 4th-order polynomial `T(V)` converts voltage to Celsius.

## 3. Communication Protocol (Burst Protocol)

Slaves interact with the Master via a **Burst Transmission** on CAN1:
- **Identifier**: Standard CAN ID (11-bit).
- **Payload**: 8 bytes (Classic CAN).
- **Structure**: Each burst contains 8 frames. Each frame carries two `float` values (4 bytes each), covering all 16 thermistors.
- **Reporting Cycle**: 100ms (10Hz).

## 4. Safety and Reliability

### Local Diagnostics
The Slave monitors its own sensor integrity:
- **Open Circuit**: Detected if ADC > 3995 (indicates a broken wire).
- **Short Circuit**: Detected if ADC < 100.
- When a fault is detected, the Slave sends a critical error frame to the Master and halts to trigger the system-wide SDC shutdown.

### Network Robustness
The same robust TX mechanism from the Master is used here:
- **Retries**: Up to 20 attempts per frame.
- **Failure Threshold**: If 10 consecutive frames fail to be accepted by the network, the Slave triggers a local `Error_Handler` to safely trip the SDC.

## 5. Configuration (Board Identity)

Multiple slaves share the same bus. The board identity (Slave 1, 2, 3, or 4) is selected in `can.h` before compilation. This selection determines the unique CAN IDs used for reporting temperatures and faults.
