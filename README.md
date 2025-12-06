# 🚁 Fiddlesticks: Rust Mini Flight Controller

This project is a firmware implementation for a palm-sized quadcopter drone, built using **Rust** for embedded systems and targeting the **STM32F4** microcontroller family. 

The primary goal is to build a flight control system from the ground up to learn the underlying principles of IMU data processing, sensor fusion, and PID control.

## ⚙️ Hardware Overview

| Component | Description |
| :--- | :--- |
| **Microcontroller** | STM32F4 (F401/F411) |
| **IMU** | MPU9250/6500 (SPI) |
| **Actuators** | 4x DC Motors (PWM Control) |
| **Communication** | USB-CDC (Calibration/Debug) |
| **Radio** | NRF24L01 (Planned) |

## 🛠️ Current Status

The system is currently capable of hardware initialization, sensor reading, and robust on-device calibration.

* **Communication:** Interrupt-driven USB-CDC serial interface with a packet queue.
* **Calibration:** Implements a state-machine driven **6-point Accelerometer Calibration** algorithm. It calculates the Bias Vector ($b$) and Inverse Sensitivity Matrix ($S^{-1}$) on-device using `nalgebra`.
* **Drivers:** Custom SPI driver for MPU9250/6500.

## ✅ TODO List

The following tasks are prioritized to achieve stable flight:

- [x] **Hardware Initialization** (Clocks, GPIO, PWM, SPI, USB)
- [x] **MPU Driver** (Basic register reading/writing)
- [x] **USB Communication** (Packet handling & buffering)
- [x] **Accelerometer Calibration** (6-point algorithm)
- [ ] **Gyro Calibration** (Bias calculation)
- [ ] **Sensor Fusion** (Implement Madgwick Filter for orientation estimation)
- [ ] **RC Receiver** (NRF24L01 driver & logic)
- [ ] **PID Controller** (Roll, Pitch, Yaw stabilization)
- [ ] **Motor Mixing** (Map PID output to motor PWM duty cycles)

## 🚀 Build & Run

Ensure you have the nightly toolchain and `probe-rs` installed.

```bash
# Flash and run
cargo run --release
