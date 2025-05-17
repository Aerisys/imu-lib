#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include <math.h>
#include <string.h>
#include <cfloat>

using std::max;
using std::min;

// MPU9250 Registers
#define MPU9250_ADDR 0x68
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_GYRO_XOUT_H 0x43
#define MPU9250_EXT_SENS_DATA_00 0x49
#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_USER_CTRL 0x6A
#define MPU9250_INT_PIN_CFG 0x37
#define MPU9250_WHO_AM_I 0x75

// AK8963 (Magnetometer) Registers
#define AK8963_ADDR 0x0C
#define AK8963_WHO_AM_I 0x00
#define AK8963_ST1 0x02
#define AK8963_HXL 0x03
#define AK8963_CNTL1 0x0A
#define AK8963_ASAX 0x10

// Constants
#define RAD_TO_DEG 57.2957795f
#define DEG_TO_RAD 0.0174532925f
#define GRAVITY 9.80665f
#define CALIBRATION_SAMPLES 1000
#define FILTER_ALPHA 0.96f // Complementary filter constant
#define MAHONY_KP 0.5f     // Mahony filter proportional gain
#define MAHONY_KI 0.1f     // Mahony filter integral gain

// Debugging
#define TAG_MPU "MPU9250"
#define LOG_LEVEL ESP_LOG_INFO

class MPU9250
{
public:
    struct Orientation
    {
        float roll;
        float pitch;
        float yaw;
    };

    struct Vector3
    {
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

    MPU9250();
    ~MPU9250();

    esp_err_t init(i2c_port_t i2cPort, uint8_t sdaPin, uint8_t sclPin);
    esp_err_t startSensorTask();
    esp_err_t calibrate();
    Orientation getOrientation();
    Vector3 getAccel();
    Vector3 getGyro();
    Vector3 getMag();
    float getTemperature();
    bool isSensorHealthy();
    CalibrationStatus getCalibrationStatus() { return calibStatus; }

private:
    // I2C Communication
    esp_err_t writeRegister(uint8_t addr, uint8_t reg, uint8_t data);
    esp_err_t readRegisters(uint8_t addr, uint8_t reg, uint8_t length, uint8_t *data);

    // Sensor Reading
    float readAccel(uint8_t axisOffset);
    float readGyro(uint8_t axisOffset);
    float readMag(uint8_t axisOffset);
    void readAllSensors();

    // Sensor Processing
    static void sensorTask(void *arg);
    void processMeasurements(float dt);
    void updateComplementaryFilter(float dt);
    void updateMahonyFilter(float dt);
    void computeAnglesFromAccel();
    float computeHeadingFromMag();

    // Calibration
    void performCalibration();
    void resetCalibration();

    // Variables
    i2c_port_t i2cPort;
    TaskHandle_t taskHandle;
    SemaphoreHandle_t dataMutex;
    uint32_t lastProcessTime;

    // Sensor Data
    Vector3 accel;
    Vector3 gyro;
    Vector3 mag;
    float temperature;

    // Computed Orientation
    Orientation orientation;

    // Filter State
    Vector3 gyroIntegrated;
    Vector3 mahonyIntegralError;
    float magHeading;

    // Calibration
    Vector3 accelOffset;
    Vector3 gyroOffset;
    Vector3 magOffset;
    Vector3 magScale;
    CalibrationStatus calibStatus;

    // Health monitoring
    uint32_t errorCount;
    uint32_t successCount;
    bool magAvailable;
    uint8_t magAdjustValues[3];

    // Filter mode
    enum FilterMode
    {
        COMPLEMENTARY,
        MAHONY
    } filterMode;
};
