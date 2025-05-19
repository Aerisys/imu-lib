#ifndef MPU6050_H
#define MPU6050_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c.h"
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

class MPU6050 {
public:
    struct Orientation {
        float roll;
        float pitch;
        float yaw;
    };

    struct Vector3 {
        float x;
        float y;
        float z;
    };

    enum CalibrationStatus {
        NOT_CALIBRATED,
        CALIBRATING,
        CALIBRATED
    };

    enum FilterMode {
        COMPLEMENTARY,
        MAHONY,
        MAHONY_QUAT
    };

    MPU6050();
    ~MPU6050();

    esp_err_t init(i2c_port_t i2cPort, uint8_t sdaPin, uint8_t sclPin);
    esp_err_t calibrate();
    esp_err_t setFilterMode(FilterMode mode);
    esp_err_t startSensorTask();

    Orientation getOrientation();
    Vector3 getAccel();
    Vector3 getGyro();
    float getTemperature();
    bool isSensorHealthy();
    CalibrationStatus getCalibrationStatus() { return calibStatus; }

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
    void computeAnglesFromAccel();

    void performCalibration();
    void resetCalibration();

    i2c_port_t i2cPort;
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
