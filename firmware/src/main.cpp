#include <Arduino.h>
#include <Wire.h>
#include "mpu6050.h"
#include "madgwick.h"

/* ══════════════════════════════════════════════════════════════════════
 * STATE MACHINE
 * ══════════════════════════════════════════════════════════════════════
 * The firmware runs as a state machine with 3 states:
 *
 *  ┌─────────┐    sensor OK    ┌─────────┐
 *  │  INIT   │ ──────────────► │ RUNNING │
 *  └─────────┘                 └─────────┘
 *       │         sensor fail       │  read fail
 *       └──────────────────────►┌───▼─────┐
 *                               │  ERROR  │
 *                               └─────────┘
 * ══════════════════════════════════════════════════════════════════════ */
typedef enum {
    STATE_INIT,       // One-time startup: init hardware
    STATE_RUNNING,    // Normal operation: read, compute, transmit
    STATE_ERROR       // Unrecoverable failure: blink LED
} SystemState;

static SystemState state = STATE_INIT;

/* ── Global objects ────────────────────────────────────────────────── */
MPU6050_Data sensor;
EulerAngles  madgwick_angles;
uint32_t     lastTick = 0;
float        dt       = 0.01f;    // Time step in seconds (updated each loop)

void setup() {
    /* ════════════════════════════════════════
     * STATE: INIT
     * ════════════════════════════════════════ */

    /* Built-in LED on Black Pill is PC13, active LOW */
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, HIGH);    // LED off

    /* USB CDC serial port — what the laptop reads.
     * 115200 baud is fast enough for our 100Hz JSON packets.        */
    Serial.begin(115200);

    /* Wait up to 2 seconds for the serial port to be opened.
     * If nothing connects we continue anyway — sensor still runs.   */
    delay(2000);

    /* I2C bus on Black Pill hardware I2C1 pins:
     * PB6 = SCL (clock), PB7 = SDA (data)
     * 400kHz = "Fast Mode" — reduces I2C read time                  */
    Wire.setSCL(PB6);
    Wire.setSDA(PB7);
    Wire.begin();
    Wire.setClock(400000);

    /* Try to initialize the MPU6050.
     * If it fails, the sensor is not connected or wired wrongly.    */
    if (!MPU6050_Init()) {
        state = STATE_ERROR;
        Serial.println("{\"error\":\"MPU6050 not found. Check wiring.\"}");
        return;
    }

    /* Initialize Madgwick filter.
     * Beta = 0.1 — good balance for a hand-held device.
     * Increase to 0.3 if the filter feels sluggish.                 */
    Madgwick_Init(0.1f);

    /* Short delay: lets the first few sensor readings stabilize
     * before we start the filter (avoids garbage initial angles)    */
    delay(200);

    lastTick = millis();
    state = STATE_RUNNING;
    Serial.println("{\"status\":\"ready\"}");
    digitalWrite(PC13, LOW);     // LED on = system running
}

void loop() {

    /* ════════════════════════════════════════
     * STATE: ERROR
     * Blink LED rapidly, do nothing else
     * ════════════════════════════════════════ */
    if (state == STATE_ERROR) {
        digitalWrite(PC13, !digitalRead(PC13));
        delay(150);
        return;
    }

    /* ════════════════════════════════════════
     * STATE: RUNNING
     * ════════════════════════════════════════ */

    /* ── Compute exact elapsed time ─────────────────────────────────
     * Using real elapsed time (not a fixed delay) makes the Madgwick
     * filter more accurate because dt is always correct even if the
     * I2C read took slightly longer than expected.                   */
    uint32_t now = millis();
    dt = (now - lastTick) / 1000.0f;    // ms → seconds
    lastTick = now;

    /* ── Read raw sensor data ────────────────────────────────────────
     * sensor.ax/ay/az = acceleration in g
     * sensor.gx/gy/gz = rotation in degrees/second                  */
    /*if (!MPU6050_Read(&sensor)) {
        state = STATE_ERROR;
        Serial.println("{\"error\":\"MPU6050 read failed.\"}");
        return;
    }*/
    uint8_t retries = 0;
    while (!MPU6050_Read(&sensor)) {
    retries++;
    if (retries >= 3) {
        // Try to re-init the sensor before declaring error
        if (!MPU6050_Init()) {
            state = STATE_ERROR;
            Serial.println("{\"error\":\"MPU6050 read failed.\"}");
            return;
        }
        retries = 0;
        }
        delay(5);
    }
    /* ══════════════════════════════════════════════════════════════
     * METHOD 1: DIRECT TRIGONOMETRY
     * ══════════════════════════════════════════════════════════════
     * Uses ONLY the accelerometer.
     * The accelerometer measures the gravity vector projected onto
     * each axis. When the board is still, this is pure gravity.
     * We use atan2 (4-quadrant arctangent) to find the angle:
     *
     * Roll:
     *   The board tilts left/right. ay and az both change.
     *   atan2(ay, az) gives the angle of that tilt.
     *
     * Pitch:
     *   The board tilts forward/back. ax changes.
     *   We compare ax against the magnitude of the other two axes.
     *
     * 57.2957795 = 180/π, converts radians to degrees.
     *
     * LIMITATION: when the board moves, the accelerometer picks up
     * the motion acceleration too, not just gravity. This makes
     * the angles noisy and wrong during movement.                   */
    float trig_roll  = atan2f(sensor.ay, sensor.az) * 57.2957795f;

    float trig_pitch = atan2f(-sensor.ax,
                               sqrtf(sensor.ay * sensor.ay +
                                     sensor.az * sensor.az))
                       * 57.2957795f;

    /* ══════════════════════════════════════════════════════════════
     * METHOD 2: MADGWICK FILTER
     * ══════════════════════════════════════════════════════════════
     * Uses BOTH accelerometer + gyroscope.
     * The gyroscope measures rotation rate directly — it's accurate
     * during motion but drifts over time.
     * The filter combines both: gyroscope handles fast movement,
     * accelerometer corrects long-term drift.
     * Result: smooth, accurate angles even during motion.          */
    Madgwick_Update(
        sensor.gx, sensor.gy, sensor.gz,
        sensor.ax, sensor.ay, sensor.az,
        dt
    );
    madgwick_angles = Madgwick_GetEuler();

    /* ── Transmit both results as one JSON line ──────────────────────
     * Format: {"tr":roll,"tp":pitch,"mr":roll,"mp":pitch,"my":yaw}
     *   t_ prefix = trig method
     *   m_ prefix = madgwick method
     *
     * Newline \n is the packet delimiter — the Python backend
     * calls readline() which reads exactly one line at a time.      */
    Serial.print("{\"tr\":");  Serial.print(trig_roll,           2);
    Serial.print(",\"tp\":"); Serial.print(trig_pitch,          2);
    Serial.print(",\"mr\":"); Serial.print(madgwick_angles.roll, 2);
    Serial.print(",\"mp\":"); Serial.print(madgwick_angles.pitch,2);
    Serial.print(",\"my\":"); Serial.print(madgwick_angles.yaw,  2);
    Serial.println("}");

    /* 10ms delay → ~100Hz loop rate */
    delay(10);
}