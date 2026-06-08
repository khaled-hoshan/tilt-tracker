#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

/* ── I2C Address ─────────────────────────────────────────────────────
 * The AD0 pin on the MPU6050 module is pulled LOW by default,
 * which sets the 7-bit I2C address to 0x68.
 * Wire library uses 7-bit addresses directly.                        */
#define MPU6050_ADDR             0x68

/* ── Internal Register Addresses ────────────────────────────────────
 * These are the memory-mapped registers inside the MPU6050 chip.
 * We write to them to configure the sensor,
 * and read from them to get measurements.                            */
#define MPU6050_REG_PWR_MGMT_1   0x6B   // Power: wake up / sleep
#define MPU6050_REG_ACCEL_CONFIG 0x1C   // Accelerometer range setting
#define MPU6050_REG_GYRO_CONFIG  0x1B   // Gyroscope range setting
#define MPU6050_REG_ACCEL_XOUT_H 0x3B   // Start of 14-byte data block
#define MPU6050_REG_WHO_AM_I     0x75   // Identity check register

/* ── Conversion Factors ──────────────────────────────────────────────
 * The sensor outputs raw 16-bit integers. These factors convert them
 * to physical units:
 * ±2g  range → 16384 raw units per g (1g = 9.81 m/s²)
 * ±250°/s range → 131 raw units per degree/second                   */
#define ACCEL_SENSITIVITY        16384.0f
#define GYRO_SENSITIVITY           131.0f

/* ── Sensor Reading Container ────────────────────────────────────────
 * Holds one complete reading from the MPU6050 in physical units.
 * ax/ay/az are in g (multiples of gravitational acceleration).
 * gx/gy/gz are in degrees per second.                               */
typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
} MPU6050_Data;

bool MPU6050_Init();
bool MPU6050_Read(MPU6050_Data *data);

#endif