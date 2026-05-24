#ifndef MPUDTO_H
#define MPUDTO_H

// -----------------------------------------------------------------------------
// mpuDTO — Data Transfer Object for IMU samples
// -----------------------------------------------------------------------------
// This struct is intentionally kept minimal and IMU-only. It is meant to be
// passed between the IMU task and any consumer (control loop, telemetry,
// logger). It must NOT carry actuator state (motors, servos, ESCs) — those
// belong in their own DTO in the consuming project (e.g. the drone firmware).
//
// Depends on the abstract `IMUSensor` interface rather than any concrete chip
// type so that swapping MPU6050 / MPU9250 / ICM-* does not ripple through
// every file that handles a sample.
// -----------------------------------------------------------------------------

#include "imu_sensor.h"

struct mpuDTO {
    IMUSensor::Vector3     accel;        // g
    IMUSensor::Vector3     gyro;         // deg/s
    IMUSensor::Vector3     mag;          // uT (zero on sensors without mag)
    IMUSensor::Orientation orientation;  // roll/pitch/yaw, deg
    IMUSensor::Quaternion  quaternion;   // identity on sensors without quaternion path
    float                  temperature;  // degC
};

#endif
