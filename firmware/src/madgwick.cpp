#include "madgwick.h"
#include <math.h>

/* ── Filter Gain (Beta) ──────────────────────────────────────────────
 * Beta controls the balance between the two sensors:
 *
 * How it works: the gyroscope is very accurate for short-term motion
 * (it directly measures rotation speed) but it DRIFTS over time
 * because small errors accumulate when we integrate it.
 * The accelerometer doesn't drift but it's NOISY during motion
 * because any vibration or movement looks like a changed gravity direction.
 *
 * Beta = how much we let the accelerometer correct the gyroscope:
 * - High beta (0.5+) → trusts accelerometer more → responsive but jittery
 * - Low beta (0.01)  → trusts gyroscope more → smooth but slow to correct
 * - 0.1 is a good balance for a hand-held device                    */
static float beta = 0.1f;

/* ── Current Orientation ─────────────────────────────────────────────
 * Stored as a quaternion. Updated every time Madgwick_Update() runs.
 * Starts as identity quaternion (q0=1, rest=0) meaning "no rotation" */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

void Madgwick_Init(float b) {
    beta = b;
    /* Reset to identity quaternion = start with no assumed rotation */
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
}

void Madgwick_Update(float gx, float gy, float gz,
                     float ax, float ay, float az, float dt) {

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot0, qDot1, qDot2, qDot3;
    float _2q0, _2q1, _2q2, _2q3;
    float _4q0, _4q1, _4q2;
    float _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    /* ── Step 1: Convert gyroscope from °/s to radians/s ────────────
     * All trig functions work in radians. π/180 ≈ 0.017453293       */
    gx *= 0.017453293f;
    gy *= 0.017453293f;
    gz *= 0.017453293f;

    /* ── Step 2: Gyroscope integration (predict new orientation) ────
     * This is the quaternion kinematics equation:
     *   dq/dt = 0.5 × q ⊗ [0, gx, gy, gz]
     * where ⊗ is quaternion multiplication.
     * This says: "given current orientation q and rotation rate g,
     * how fast is the orientation changing right now?"               */
    qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    /* ── Step 3: Accelerometer correction ───────────────────────────
     * We only use the accelerometer when it gives a valid reading.
     * During free-fall or high vibration, its magnitude would be
     * far from 1g, meaning it's not measuring gravity reliably.      */
    float accelMag = sqrtf(ax*ax + ay*ay + az*az);
    if (accelMag > 0.0f) {

        /* Normalize: we only care about the DIRECTION of gravity,
         * not its magnitude. Dividing by magnitude gives unit vector */
        recipNorm = 1.0f / accelMag;
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* Precompute terms used multiple times below */
        _2q0 = 2.0f * q0; _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0; _4q1 = 4.0f * q1; _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1; _8q2 = 8.0f * q2;
        q0q0 = q0*q0; q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3;

        /* Gradient descent step:
         * This computes "how wrong is our current quaternion
         * compared to what the accelerometer says gravity should be?"
         * s0-s3 is the direction to move the quaternion to fix that. */
        s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
        s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay
             - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
        s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay
             - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
        s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;

        /* Normalize the gradient step */
        recipNorm = 1.0f / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm; s1 *= recipNorm;
        s2 *= recipNorm; s3 *= recipNorm;

        /* Subtract the correction from the gyroscope prediction.
         * Beta scales how strongly we apply the correction.          */
        qDot0 -= beta * s0;
        qDot1 -= beta * s1;
        qDot2 -= beta * s2;
        qDot3 -= beta * s3;
    }

    /* ── Step 4: Integrate — apply the change over time step dt ─────
     * q_new = q_old + (dq/dt) × dt
     * This is simple Euler integration of the quaternion             */
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    /* ── Step 5: Re-normalize ────────────────────────────────────────
     * A valid rotation quaternion must have magnitude = 1.
     * Floating point math during integration slowly breaks this,
     * so we normalize back to unit length after every update.        */
    recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm;
    q2 *= recipNorm; q3 *= recipNorm;
}

EulerAngles Madgwick_GetEuler() {
    EulerAngles e;
    float sinp;

    /* ── Convert quaternion → Euler angles ──────────────────────────
     * These formulas come directly from rotation matrix derivation.
     * 57.2957795 = 180/π, converts radians to degrees.              */

    /* Roll: rotation around X axis */
    e.roll = atan2f(2.0f*(q0*q1 + q2*q3),
                    1.0f - 2.0f*(q1*q1 + q2*q2)) * 57.2957795f;

    /* Pitch: rotation around Y axis
     * We clamp to ±90° to handle gimbal lock at the poles           */
    sinp = 2.0f * (q0*q2 - q3*q1);
    e.pitch = (fabsf(sinp) >= 1.0f)
              ? copysignf(90.0f, sinp)
              : asinf(sinp) * 57.2957795f;

    /* Yaw: rotation around Z axis */
    e.yaw = atan2f(2.0f*(q0*q3 + q1*q2),
                   1.0f - 2.0f*(q2*q2 + q3*q3)) * 57.2957795f;
    return e;
}

Quaternion Madgwick_GetQuaternion() {
    return {q0, q1, q2, q3};
}