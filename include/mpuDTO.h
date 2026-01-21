#ifndef MPUDTO_H
#define MPUDTO_H

#include "mpu9250.h"

#define NUM_MOTORS 4

struct mpuDTO {
        MPU9250::Vector3 accel;
        MPU9250::Vector3 gyro;
        MPU9250::Vector3 mag;
        MPU9250::Orientation orientation;

        float motorSpeeds[NUM_MOTORS];
    };

#endif