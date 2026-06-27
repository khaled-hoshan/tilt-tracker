# 🧭 Tilt Tracker: Real-Time IMU Orientation Dashboard

A full-stack embedded systems project that streams real-time orientation data from an STM32 microcontroller to a live 3D browser dashboard. 

This project demonstrates **hardware interfacing, sensor fusion mathematics, and real-time data pipelines** by visually comparing two different tilt-calculation methods side-by-side: raw accelerometer trigonometry vs. a Madgwick quaternion filter.

---

## 🚀 Project Overview

* **Microcontroller:** STM32F401CEU6 (Black Pill)
* **Sensor:** MPU6050 (6-axis IMU: Accelerometer + Gyroscope)
* **Firmware:** C++ / Arduino framework via PlatformIO
* **Backend:** Python (FastAPI, PySerial, WebSockets)
* **Frontend:** HTML5, Vanilla JS, Three.js (WebGL)

### The Pipeline
1. **Sense:** The STM32 reads raw accelerometer and gyroscope data from the MPU6050 via I2C at 400kHz.
2. **Compute:** The firmware calculates orientation using two simultaneous methods (Trig & Madgwick) on a non-blocking 100Hz loop.
3. **Transmit:** Data is serialized into JSON and sent over a USB CDC virtual serial port (115200 baud).
4. **Route:** A Python backend reads the serial stream and broadcasts it via WebSockets.
5. **Render:** A Three.js frontend receives the WebSocket data and visually rotates two 3D models in near real-time.

---

## 🛠️ Hardware Setup

**Components Needed:**
* 1x STM32F401CEU6 "Black Pill" board
* 1x MPU6050 (GY-521) IMU module
* 4x Female-to-Female jumper wires
* 1x USB-C data cable

**Wiring Diagram:**

| MPU6050 Pin | STM32 Pin | Description |
| :--- | :--- | :--- |
| **VCC** | **3V3** | 3.3V Power Supply |
| **GND** | **GND** | Common Ground |
| **SCL** | **PB6** | I2C Clock |
| **SDA** | **PB7** | I2C Data |
| **AD0** | *Unconnected* | Defaults to 0x68 I2C address |

> **Note:** Always power the GY-521 module using the **3V3** pin on the STM32 for this project to ensure logic levels remain safe and consistent.

---

## 🧮 Two Methods of Orientation Computation

A core goal of this project is demonstrating *why* sensor fusion is necessary in embedded systems. The dashboard shows both methods running simultaneously:

### 1. Direct Trigonometry (Accelerometer Only)
* **How it works:** Uses `atan2()` on the gravity vector to calculate roll and pitch.
* **Pros:** Mathematically simple, stateless, no long-term drift.
* **Cons:** Highly susceptible to vibration and linear acceleration. Fails completely when the sensor is moving. Cannot measure yaw (without a magnetometer).

### 2. Madgwick Filter (Sensor Fusion)
* **How it works:** Integrates gyroscope data for fast, smooth rotation tracking, while using accelerometer data to slowly correct gyroscope drift via a gradient descent algorithm (quaternion-based).
* **Pros:** Smooth, responsive, immune to short-term vibrations. Internal state avoids gimbal lock.
* **Cons:** Computationally heavier. Yaw will still drift slowly over time due to the lack of a magnetometer reference.

---

## 💻 Installation & Quick Start

### 1. Flash the Firmware
You will need [PlatformIO](https://platformio.org/) installed (via VS Code or CLI).
```bash
# Clone the repository
git clone [https://github.com/khaled-hoshan/tilt-tracker.git](https://github.com/khaled-hoshan/tilt-tracker.git)
cd tilt-tracker/firmware

# Compile and upload to the STM32 via USB (DFU mode)
pio run --target upload