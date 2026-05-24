#ifndef MPUDTO_H
#define MPUDTO_H

// -----------------------------------------------------------------------------
// mpuDTO — Data Transfer Object for IMU samples
// -----------------------------------------------------------------------------
// This struct is intentionally kept minimal and IMU-only. It is meant to be
// passed between the IMU task and any consumer (control loop, telemetry,
// logger). It must NOT carry actuator state (motors, servos, ESCs) — those
// belong in their own DTO in the consuming project (e.g. the drone firmware).
// -----------------------------------------------------------------------------

#include "mpu9250.h"

struct mpuDTO {
    MPU9250::Vector3     accel;        // g
    MPU9250::Vector3     gyro;         // deg/s
    MPU9250::Vector3     mag;          // uT
    MPU9250::Orientation orientation;  // roll/pitch/yaw, deg
    float                temperature;  // degC
};

#endif
