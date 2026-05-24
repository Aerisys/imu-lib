#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include "esp_err.h"

// -----------------------------------------------------------------------------
// IMUSensor — abstract interface common to every IMU implementation in this
// library (MPU6050, MPU9250, and future ICM-* chips).
// -----------------------------------------------------------------------------
// Rationale: the drone firmware should be able to swap sensor types without
// rewriting the control loop. Anything that varies wildly between chips
// (init config, magnetometer presence, persistence backend, advanced tuning
// knobs) stays on the concrete class; this interface exposes only what every
// IMU we plan to support can answer.
//
// Usage in a consumer project:
//
//   #include "mpu9250.h"
//   #include "imu_sensor.h"
//
//   static MPU9250 mpu;                  // concrete type for init/tuning
//   IMUSensor* imu = &mpu;               // polymorphic for the control loop
//
//   mpu.init(bus);                       // sensor-specific
//   mpu.setMahonyGains(1.0f, 0.0f);      // MPU9250-only
//   imu->startSensorTask();              // polymorphic
//
//   while (1) {
//       auto q = imu->getQuaternion();   // polymorphic, control loop input
//       // ... PID work ...
//   }
//
// Note on types and enums: every nested type and enum below is exposed in
// the public scope of IMUSensor, so concrete derived classes inherit them
// without redeclaration. `MPU9250::Vector3`, `MPU9250::CALIBRATED`,
// `MPU9250::MAHONY` therefore keep working unchanged.
// -----------------------------------------------------------------------------

class IMUSensor
{
public:
    // ---------------- Shared value types ------------------------------------
    struct Vector3
    {
        float x;
        float y;
        float z;
    };

    struct Orientation
    {
        float roll;
        float pitch;
        float yaw;
    };

    // Unit quaternion, body-to-world rotation. w + xi + yj + zk convention.
    struct Quaternion
    {
        float w;
        float x;
        float y;
        float z;
    };

    enum CalibrationStatus
    {
        NOT_CALIBRATED,
        CALIBRATING,
        CALIBRATED
    };

    // Minimum common filter set. Implementations are free to add private
    // variants (e.g. quaternion-only Mahony) for their own internal use, but
    // anything reachable through this interface must map to one of these.
    enum FilterMode
    {
        COMPLEMENTARY,
        MAHONY
    };

    // ---------------- Lifecycle ---------------------------------------------
    virtual ~IMUSensor() = default;

    // Triggers the calibration sequence. Returns ESP_OK if the calibration
    // task was successfully created (calibration itself runs asynchronously
    // in the background — poll `getCalibrationStatus()` for completion).
    virtual esp_err_t calibrate() = 0;

    // Spawns the FreeRTOS task that reads the sensor at the configured rate
    // (interrupt-driven or polling, depending on the concrete impl).
    virtual esp_err_t startSensorTask() = 0;

    virtual CalibrationStatus getCalibrationStatus() = 0;

    // ---------------- Configuration ------------------------------------------
    virtual esp_err_t setFilterMode(FilterMode mode) = 0;
    virtual esp_err_t setInvertAxis(bool invertX, bool invertY, bool invertZ) = 0;
    virtual esp_err_t setSwitchRollPitch(bool swap) = 0;

    // ---------------- Data getters ------------------------------------------
    virtual Vector3     getAccel()       = 0;          // g
    virtual Vector3     getGyro()        = 0;          // deg/s
    virtual Orientation getOrientation() = 0;          // deg
    virtual float       getTemperature() = 0;          // degC
    virtual bool        isSensorHealthy()= 0;

    // Magnetometer (uT). Default implementation returns {0,0,0} for sensors
    // without a magnetometer (MPU6050). Override in chips that have one.
    virtual Vector3 getMag() { return {0.0f, 0.0f, 0.0f}; }

    // Body-to-world quaternion. Default returns identity for implementations
    // that don't run a quaternion-based filter. Drone control loops should
    // prefer this over `getOrientation()` to avoid gimbal lock at +/- 90 deg
    // pitch during loops, flips, or sustained inverted flight.
    virtual Quaternion getQuaternion() { return {1.0f, 0.0f, 0.0f, 0.0f}; }
};

#endif // IMU_SENSOR_H
