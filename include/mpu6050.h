#ifndef MPU6050_H
#define MPU6050_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "imu_sensor.h"
#include <math.h>
#include <string.h>
#include <cfloat>

using std::max;
using std::min;

// MPU6050 Registers
#define MPU6050_ADDR 0x68
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_WHO_AM_I 0x75

// Constants
#define RAD_TO_DEG 57.2957795f
#define DEG_TO_RAD 0.0174532925f
#define GRAVITY 9.80665f
#define CALIBRATION_SAMPLES 1000
#define FILTER_ALPHA 0.96f
#define MAHONY_KP 0.5f
#define MAHONY_KI 0.1f

#define TAG_MPU6050 "MPU6050"
#define LOG_LEVEL ESP_LOG_INFO

// MPU6050 implements the common IMUSensor interface. Value types and the
// FilterMode / CalibrationStatus enums are inherited from IMUSensor — the
// `MPU6050::Vector3`, `MPU6050::CALIBRATED` etc. spellings still work via
// public inheritance scope rules.
//
// The previous public MAHONY_QUAT mode was an MPU6050-only extension and
// did not fit the polymorphic interface; it is now kept internally if the
// implementation chooses to use it, but is no longer part of the API.
class MPU6050 : public IMUSensor {
public:

    // Optional initialization parameters (see MPU9250::Config for rationale).
    // The MPU6050 has no magnetometer, so only the MPU address is configurable.
    // Default SCL clock is 400 kHz (Fast Mode), supported by the MPU6050.
    struct Config
    {
        uint8_t  mpuAddr    = 0x68;
        uint32_t sclSpeedHz = 400000;
    };

    MPU6050();
    ~MPU6050();

    // The bus is owned by the CALLER; this class only adds itself as a device.
    // See MPU9250::init for the full rationale and a usage example.
    esp_err_t init(i2c_master_bus_handle_t busHandle, const Config& config = {});

    // ---- IMUSensor interface implementation --------------------------------
    esp_err_t calibrate() override;
    esp_err_t setFilterMode(FilterMode mode) override;
    esp_err_t startSensorTask() override;

    Orientation       getOrientation()      override;
    Vector3           getAccel()            override;
    Vector3           getGyro()             override;
    // getMag() and getQuaternion() use the IMUSensor defaults (zero / identity)
    // — the MPU6050 has no magnetometer and does not maintain a public
    // quaternion in its current implementation.
    float             getTemperature()      override;
    bool              isSensorHealthy()     override;
    CalibrationStatus getCalibrationStatus() override { return calibStatus; }

    // Axis-orientation knobs required by the interface. The MPU6050 path does
    // not yet apply them to its sample pipeline — they are accepted and
    // stored so the call is observably valid (returns ESP_OK), but have no
    // runtime effect today. Implement properly in mpu6050.cpp when needed,
    // or use MPU9250 if axis orientation correction is required.
    esp_err_t setInvertAxis(bool invertX, bool invertY, bool invertZ) override;
    esp_err_t setSwitchRollPitch(bool swap) override;

private:
    esp_err_t writeRegister(uint8_t reg, uint8_t data);
    esp_err_t readRegisters(uint8_t reg, uint8_t length, uint8_t *data);

    float readAccel(uint8_t axisOffset);
    float readGyro(uint8_t axisOffset);
    void readAllSensors();

    static void sensorTask(void *arg);
    void processMeasurements(float dt);
    void updateComplementaryFilter(float dt);
    void updateMahonyFilter(float dt);
    void updateMahonyQuat(float dt);
    // computeAnglesFromAccel(): removed — was declared but never defined nor
    // called; the filter paths derive roll/pitch directly from accel inside
    // updateComplementaryFilter / updateMahonyFilter.

    void performCalibration();
    void resetCalibration();

    // I2C bus owned by the caller; the device handle is created in init().
    i2c_master_bus_handle_t busHandle;
    i2c_master_dev_handle_t mpuDev;

    TaskHandle_t taskHandle;
    SemaphoreHandle_t dataMutex;
    uint32_t lastProcessTime;

    Vector3 accel;
    Vector3 gyro;
    float temperature;
    Orientation orientation;

    Vector3 gyroIntegrated;
    Vector3 mahonyIntegralError;

    Vector3 accelOffset;
    Vector3 gyroOffset;
    CalibrationStatus calibStatus;

    uint32_t errorCount;
    uint32_t successCount;

    FilterMode filterMode;

    // Quaternion for Mahony filter
    float q0, q1, q2, q3; // Quaternion components
};

#endif // MPU6050_H
