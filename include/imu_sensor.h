#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include "esp_err.h"
#include <stdint.h>

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

    // Atomic snapshot of every value the IMU exposes in one consistent set.
    // Concrete implementations should publish this in one shot so a consumer
    // PID loop sees coherent accel/gyro/orientation taken at the same moment
    // (not three different iterations stitched together by individual getters).
    //
    // `timestampUs` is the `esp_timer_get_time()` value captured by the
    // sensor task when the I2C burst completed; consumers can compute their
    // own `dt = (current.timestampUs - previous.timestampUs) * 1e-6` and
    // avoid polling FreeRTOS ticks at ms resolution.
    struct SampleBundle
    {
        Vector3     accel;
        Vector3     gyro;
        Vector3     mag;
        Orientation orientation;
        Quaternion  quaternion;
        float       temperature;
        uint64_t    timestampUs;
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

    // -------------------------------------------------------------------------
    // Atomic snapshot + new-sample signalling
    // -------------------------------------------------------------------------
    // The drone PID loop typically wants (a) a coherent set of values taken
    // at the same instant, and (b) to be woken precisely when a fresh sample
    // is ready — not poll at a guessed period.
    //
    // The default `getSnapshot()` below gathers fields through the individual
    // getters; this is observably correct but the fields can come from two
    // different sensor iterations. Concrete implementations should override
    // with a lock-free atomic publish (e.g. seqlock) so a PID loop reading
    // accel and gyro sees the same source iteration.
    //
    // The default `waitForNewSample()` returns ESP_ERR_NOT_SUPPORTED; the
    // MPU9250 override blocks on an internal binary semaphore that the
    // sensor task gives on each publish.
    virtual SampleBundle getSnapshot()
    {
        SampleBundle bundle = {};
        bundle.accel       = getAccel();
        bundle.gyro        = getGyro();
        bundle.mag         = getMag();
        bundle.orientation = getOrientation();
        bundle.quaternion  = getQuaternion();
        bundle.temperature = getTemperature();
        bundle.timestampUs = 0;
        return bundle;
    }

    virtual esp_err_t waitForNewSample(uint32_t /*timeoutMs*/)
    {
        return ESP_ERR_NOT_SUPPORTED;
    }
};

#endif // IMU_SENSOR_H
