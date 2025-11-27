#ifndef MPUDTO_H
#define MPUDTO_H

#include "mpu9250.h"

struct mpuDTO {
        MPU9250::Vector3 accel;
        MPU9250::Vector3 gyro;
        MPU9250::Vector3 mag;
        MPU9250::Orientation orientation;
    };

#endif