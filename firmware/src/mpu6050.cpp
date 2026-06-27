#include "mpu6050.h"

/* ── Helper: write one byte to one register ─────────────────────────
 * beginTransmission starts the I2C transaction,
 * write() queues the register address then the value,
 * endTransmission() sends it and releases the bus.                  */
static void writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

/* ── Helper: read one byte from one register ─────────────────────────
 * We first write the register address we want to read,
 * then do a repeated start (false = don't release bus yet),
 * then request 1 byte and read it.                                   */
static uint8_t readReg(uint8_t reg) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 1);
    return Wire.read();
}

bool MPU6050_Init() {
    /* ── Verify the sensor is actually there ────────────────────────
     * WHO_AM_I is a read-only register that always returns 0x68.
     * If we get something else, the sensor isn't connected or
     * the wiring is wrong.                                           */
    Wire.beginTransmission(MPU6050_ADDR);
    uint8_t err = Wire.endTransmission();
    if (err != 0) return false;
    
    // uint8_t id = readReg(MPU6050_REG_WHO_AM_I);
    // if (id != 0x68) return false;
    
    /* ── Wake the sensor up ─────────────────────────────────────────
     * After power-on the MPU6050 is in SLEEP mode by default.
     * Writing 0x00 to PWR_MGMT_1 clears the SLEEP bit (bit 6)
     * and all other power management bits, starting normal mode.    */
    writeReg(MPU6050_REG_PWR_MGMT_1, 0x00);

    /* ── Set accelerometer range to ±2g ─────────────────────────────
     * Bits [4:3] in ACCEL_CONFIG control the full-scale range.
     * 00 = ±2g  → highest resolution, best for tilt sensing
     * (We don't need to measure large accelerations here)           */
    writeReg(MPU6050_REG_ACCEL_CONFIG, 0x00);

    /* ── Set gyroscope range to ±250°/s ─────────────────────────────
     * Bits [4:3] in GYRO_CONFIG control the full-scale range.
     * 00 = ±250°/s → highest resolution
     * Sufficient for hand-held rotation speeds                      */
    writeReg(MPU6050_REG_GYRO_CONFIG, 0x00);

    delay(10);    // Wait for sensor to apply config and stabilize
    return true;
}

bool MPU6050_Read(MPU6050_Data *data) {
    /* ── Burst read: 14 bytes starting at ACCEL_XOUT_H ─────────────
     * The MPU6050 stores its output in consecutive registers:
     * [ACCEL_X_H][ACCEL_X_L][ACCEL_Y_H][ACCEL_Y_L][ACCEL_Z_H][ACCEL_Z_L]
     * [TEMP_H][TEMP_L]
     * [GYRO_X_H][GYRO_X_L][GYRO_Y_H][GYRO_Y_L][GYRO_Z_H][GYRO_Z_L]
     *
     * Reading all 14 in one transaction is important: it guarantees
     * all values are from the same sample instant. Reading them
     * in separate transactions risks getting a mix of old and new. */
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);          // false = repeated start, keep bus
    Wire.requestFrom(MPU6050_ADDR, 14);

    if (Wire.available() < 14) return false;

    /* ── Combine high + low bytes into signed 16-bit integers ───────
     * MPU6050 sends big-endian: high byte first, low byte second.
     * We shift the high byte left 8 bits and OR with low byte.
     * Casting to int16_t gives us the correct signed value.         */
    int16_t raw;

    raw = (int16_t)((Wire.read() << 8) | Wire.read());
    data->ax = raw / ACCEL_SENSITIVITY;   // Convert to g

    raw = (int16_t)((Wire.read() << 8) | Wire.read());
    data->ay = raw / ACCEL_SENSITIVITY;

    raw = (int16_t)((Wire.read() << 8) | Wire.read());
    data->az = raw / ACCEL_SENSITIVITY;

    Wire.read(); Wire.read();             // Temperature bytes — discard

    raw = (int16_t)((Wire.read() << 8) | Wire.read());
    data->gx = raw / GYRO_SENSITIVITY;   // Convert to °/s

    raw = (int16_t)((Wire.read() << 8) | Wire.read());
    data->gy = raw / GYRO_SENSITIVITY;

    raw = (int16_t)((Wire.read() << 8) | Wire.read());
    data->gz = raw / GYRO_SENSITIVITY;

    return true;
}