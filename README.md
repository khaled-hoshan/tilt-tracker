# 🧭 Tilt Tracker: Real-Time IMU Orientation Dashboard

A full-stack embedded systems project that streams real-time orientation data from an STM32 microcontroller to a live 3D browser dashboard.

This project demonstrates **hardware interfacing, sensor fusion mathematics, and real-time data pipelines** by visually comparing two different tilt-calculation methods side-by-side: raw accelerometer trigonometry vs. a Madgwick quaternion filter.

---

## 🚀 Project Overview

- **Microcontroller:** STM32F401CEU6 (Black Pill)
- **Sensor:** MPU6050 (6-axis IMU: Accelerometer + Gyroscope)
- **Firmware:** C++ / Arduino framework via PlatformIO
- **Backend:** Python (FastAPI, PySerial, WebSockets)
- **Frontend:** HTML5, Vanilla JavaScript, Three.js (WebGL)

### The Pipeline

1. **Sense:** The STM32 reads raw accelerometer and gyroscope data from the MPU6050 via I²C at 400 kHz.
2. **Compute:** The firmware calculates orientation using two simultaneous methods (Trigonometry & Madgwick Filter) on a non-blocking 100 Hz loop.
3. **Transmit:** Data is serialized into JSON and sent over a USB CDC virtual serial port (115200 baud).
4. **Route:** A Python backend reads the serial stream and broadcasts it through WebSockets.
5. **Render:** A Three.js frontend receives the WebSocket data and rotates two 3D models in near real time.

---

## 🛠️ Hardware Setup

### Wiring Diagram

| MPU6050 Pin | STM32 Pin | Description |
|-------------|-----------|-------------|
| **VCC** | **3V3** | 3.3 V Power Supply |
| **GND** | **GND** | Common Ground |
| **SCL** | **PB6** | I²C Clock |
| **SDA** | **PB7** | I²C Data |
| **AD0** | *Unconnected* | Defaults to I²C address `0x68` |

> **Note:** Always power the GY-521 (MPU6050) module from the STM32's **3V3** pin to ensure compatible logic voltage levels.

---

## 🧮 Two Methods of Orientation Computation

A primary objective of this project is demonstrating **why sensor fusion is necessary** for reliable orientation estimation.

### 1. Direct Trigonometry (Accelerometer Only)

**How it works**

Uses `atan2()` on the gravity vector to compute roll and pitch directly from accelerometer measurements.

**Advantages**

- Simple mathematics
- No accumulated drift over time

**Limitations**

- Sensitive to movement and vibration
- Cannot determine yaw without an external heading reference (magnetometer)

---

### 2. Madgwick Filter (Sensor Fusion)

**How it works**

Integrates gyroscope measurements for smooth, responsive orientation updates while continuously correcting drift using accelerometer data through a quaternion-based gradient descent algorithm.

**Advantages**

- Smooth and responsive motion tracking
- More resistant to short-term vibrations
- Quaternion representation avoids gimbal lock

**Limitations**

- More computationally intensive
- Yaw slowly drifts over time without a magnetometer

---

## 💻 Installation & Quick Start

### 1. Clone the Repository

```bash
git clone https://github.com/khaled-hoshan/tilt-tracker.git
cd tilt-tracker
```

---

### 2. Flash the Firmware

Install PlatformIO, then run:

```bash
cd firmware
pio run --target upload
```

---

### 3. Start the Backend

Python 3.8 or newer is recommended.

```bash
cd ../backend

python -m venv venv

# Linux/macOS
source venv/bin/activate

# Windows
# venv\Scripts\activate

pip install pyserial fastapi uvicorn websockets

uvicorn main:app --reload --port 8000
```

---

### 4. Open the Dashboard

Open your browser and navigate to:

```
http://localhost:8000
```

Ensure the STM32 is connected via USB. The backend will automatically detect the STMicroelectronics USB CDC device and begin streaming orientation data to the dashboard.

---

## 🛑 Error Handling & LED Status

The STM32 firmware uses the onboard LED (PC13) to indicate system status.

| LED State | Meaning |
|-----------|---------|
| **Off** | Initialization in progress |
| **Solid On** | System operating normally and streaming data |
| **Blinking** | Fatal error (e.g., MPU6050 communication failure or I²C error) |

---

## 📂 Project Structure

```text
tilt-tracker/
├── firmware/          # STM32 firmware (PlatformIO)
├── backend/           # FastAPI + Serial + WebSocket server
├── frontend/          # HTML, JavaScript, Three.js dashboard
├── README.md
└── LICENSE
```

---

## 🎯 Features

- Real-time IMU visualization
- Side-by-side comparison of two orientation estimation methods
- Non-blocking embedded firmware running at 100 Hz
- USB serial communication using JSON packets
- FastAPI backend with WebSocket broadcasting
- Interactive Three.js 3D visualization
- Automatic serial device detection
- Onboard LED diagnostic status indicator

---

## 📄 License

This project was developed as part of a fourth-year Computer Engineering course. You are welcome to use, modify, and fork it for educational and personal projects.