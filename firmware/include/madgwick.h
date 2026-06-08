#ifndef MADGWICK_H
#define MADGWICK_H

/* ── Euler Angles ────────────────────────────────────────────────────
 * The three angles that describe orientation in 3D space:
 *
 * Roll  — rotation around the X axis (tilt left ↔ right)
 * Pitch — rotation around the Y axis (tilt forward ↔ backward)
 * Yaw   — rotation around the Z axis (rotate left ↔ right, like a compass)
 *
 * All values in degrees.                                             */
typedef struct {
    float roll;
    float pitch;
    float yaw;
} EulerAngles;

/* ── Quaternion ──────────────────────────────────────────────────────
 * A quaternion is a 4-component number (q0, q1, q2, q3) used to
 * represent 3D rotation without the "gimbal lock" problem that
 * Euler angles suffer from at certain orientations.
 * The filter works entirely in quaternion space internally,
 * then we convert to Euler only at the end for display.             */
typedef struct {
    float q0, q1, q2, q3;
} Quaternion;

void        Madgwick_Init(float beta);
void        Madgwick_Update(float gx, float gy, float gz,
                            float ax, float ay, float az,
                            float dt);
EulerAngles Madgwick_GetEuler();
Quaternion  Madgwick_GetQuaternion();

#endif