#ifndef MPU9250_H
#define MPU9250_H

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
#define TAG_MPU9250 "MPU9250"
#define LOG_LEVEL ESP_LOG_INFO

// --------------------------------------------------------------------------------
// MPU9250 Class
// --------------------------------------------------------------------------------
// This class handles the initialization, calibration, and data retrieval from the MPU9250
// and AK8963 sensors (Magnetometer in MPU9250). It also provides methods for orientation calculation and sensor health monitoring.
//
// The class uses FreeRTOS for task management and synchronization.
// It supports both complementary and Mahony filter algorithms for orientation estimation.
// The class also includes calibration routines for accelerometer, gyroscope, and magnetometer.
// The calibration process is performed in the background and can be monitored through the `getCalibrationStatus` method.
// The class provides methods to retrieve the current orientation, accelerometer, gyroscope, and magnetometer readings.
// The orientation is represented as roll, pitch, and yaw angles in degrees.
// The accelerometer, gyroscope, and magnetometer readings are represented as 3D vectors.
// The class also includes a health monitoring feature that tracks the number of successful and failed sensor readings.
// The health status can be checked using the `isSensorHealthy` method.
// The class is designed to be used with ESP-IDF framework and requires the I2C driver for communication with the sensors.
// TO USE THIS CLASS:
// 1. Include the header file in your project.
// 2. Create an instance of the MPU9250 class.
// 3. Call the `init` method to initialize the I2C communication and sensors.
// 4. Start the sensor task using the `startSensorTask` method.
// 5. Optionally, call the `calibrate` method to perform calibration.
// 6. Use the `getOrientation`, `getAccel`, `getGyro`, and `getMag` methods to retrieve sensor data.
// 7. Monitor the calibration status and health status using `getCalibrationStatus` and `isSensorHealthy` methods.
// 8. Stop the sensor task and clean up resources when done.
//
// Example usage:
// extern "C" void app_main() {
//     static MPU9250 imu;

//     esp_err_t err = imu.init(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22);  // Set appropriate SDA/SCL pins
//     if (err != ESP_OK) {
//         ESP_LOGE("APP", "Failed to initialize MPU9250");
//         return;
//     }

//     ESP_LOGI("APP", "MPU9250 initialized");

//     err = imu.calibrate();
//     if (err != ESP_OK) {
//         ESP_LOGE("APP", "Failed to start calibration");
//         return;
//     }

//     err = imu.startSensorTask();  // Assuming this creates a FreeRTOS task for reading sensor data
//     if (err != ESP_OK) {
//         ESP_LOGE("APP", "Failed to start sensor task");
//         return;
//     }
// }
// --------------------------------------------------------------------------------
class MPU9250
{
public:
    // Sensor Data Structures
    // These structures are used to represent the sensor data and orientation.

    // The `Orientation` structure contains roll, pitch, and yaw angles in degrees.
    struct Orientation
    {
        float roll;
        float pitch;
        float yaw;
    };

    // The `Vector3` structure represents a 3D vector with x, y, and z components.
    // It is used to represent accelerometer, gyroscope, and magnetometer readings.
    struct Vector3
    {
        float x;
        float y;
        float z;
    };

    // The `CalibrationStatus` enum represents the calibration status of the sensor.
    // It can be one of the following values:
    // - NOT_CALIBRATED: The sensor is not calibrated.
    // - CALIBRATING: The sensor is currently being calibrated.
    // - CALIBRATED: The sensor has been calibrated successfully.
    enum CalibrationStatus
    {
        NOT_CALIBRATED,
        CALIBRATING,
        CALIBRATED
    };

    // The `FilterMode` enum represents the filter mode used for orientation calculation.
    // It can be one of the following values:
    // - COMPLEMENTARY: Use the complementary filter for orientation estimation.
    // - MAHONY: Use the Mahony filter for orientation estimation.
    // The complementary filter combines accelerometer and gyroscope data to estimate orientation.
    // The Mahony filter uses a quaternion-based approach to estimate orientation and includes an integral feedback loop.
    // The choice of filter mode can affect the performance and accuracy of the orientation estimation.
    // The complementary filter is simpler and faster, while the Mahony filter provides better performance in dynamic conditions.
    // The filter mode can be set using the `setFilterMode` method.
    enum FilterMode
    {
        COMPLEMENTARY,
        MAHONY
    };

    // Constructor and Destructor
    MPU9250();
    ~MPU9250();

    // Public Methods
    // These methods are used to initialize the sensor, start the sensor task, perform calibration,
    // retrieve sensor data, and check the sensor health status.

    // The `init` method initializes the I2C communication and configures the sensor.
    // It takes the I2C port number, SDA pin number, and SCL pin number as parameters.
    // It returns an ESP error code indicating the success or failure of the initialization.
    esp_err_t init(i2c_port_t i2cPort, uint8_t sdaPin, uint8_t sclPin);

    // The `calibrate` method performs calibration of the accelerometer, gyroscope, and magnetometer.
    // It collects a specified number of samples and computes the offsets for each sensor.
    // The calibration process is performed in the background and can take some time.
    // The method returns an ESP error code indicating the success or failure of the calibration.
    // The calibration status can be monitored using the `getCalibrationStatus` method.
    esp_err_t calibrate();

    // The `setFilterMode` method sets the filter mode for orientation calculation.
    // It takes a `FilterMode` enum value as a parameter and sets the filter mode accordingly.
    // The filter mode can be set to either COMPLEMENTARY or MAHONY.
    // The choice of filter mode can affect the performance and accuracy of the orientation estimation.
    // The method returns an ESP error code indicating the success or failure of the operation.
    esp_err_t setFilterMode(FilterMode mode);

    // The `startSensorTask` method starts the sensor task, which continuously reads sensor data
    // and processes it in the background. It returns an ESP error code indicating the success or failure.
    // The task runs in a FreeRTOS environment and uses a mutex for synchronization.
    // The task is responsible for reading sensor data, applying filters, and updating the orientation.
    // The task also handles calibration and health monitoring.
    // The task runs at a specified frequency, which can be adjusted in the implementation.
    esp_err_t startSensorTask();

    // The `getOrientation` method retrieves the current orientation of the sensor.
    // It returns an `Orientation` structure containing the roll, pitch, and yaw angles in degrees.
    // The angles are computed using the complementary or Mahony filter, depending on the filter mode.
    // The roll angle represents the rotation around the x-axis, the pitch angle represents the rotation around the y-axis,
    // and the yaw angle represents the rotation around the z-axis.
    Orientation getOrientation();

    // The `getAccel`, `getGyro`, and `getMag` methods retrieve the current accelerometer, gyroscope, and magnetometer readings.
    // They return `Vector3` structures containing the x, y, and z components of the respective sensor data.
    // The accelerometer readings are in g (gravitational units), the gyroscope readings are in degrees per second,
    // and the magnetometer readings are in microteslas (µT).
    Vector3 getAccel();
    Vector3 getGyro();
    Vector3 getMag();

    // The `getTemperature` method retrieves the current temperature reading from the sensor.
    // It returns the temperature in degrees Celsius.
    // The temperature is measured by the internal temperature sensor of the MPU9250.
    // The temperature reading can be used for compensation in some applications.
    // The temperature is not used for orientation calculation but can be useful for monitoring the sensor's operating conditions.
    float getTemperature();

    // The `isSensorHealthy` method checks the health status of the sensor.
    // It returns true if the sensor is healthy and able to provide valid readings,
    // and false if there are issues with the sensor communication or data retrieval.
    // The health status is monitored by counting the number of successful and failed sensor readings.
    // The method can be used to detect sensor malfunctions or communication errors.
    // The health status can be used to trigger error handling or fallback mechanisms in the application.
    // The method can also be used to check if the sensor is ready for use after initialization or calibration.
    bool isSensorHealthy();

    // The `getCalibrationStatus` method retrieves the current calibration status of the sensor.
    // It returns a `CalibrationStatus` enum value indicating whether the sensor is not calibrated,
    // currently being calibrated, or has been calibrated successfully.
    CalibrationStatus getCalibrationStatus() { return calibStatus; }

    // The `setInvertAxis` method sets the axis inversion for the sensor readings.
    // It takes three boolean parameters indicating whether to invert the x, y, and z axes respectively.    
    // Inverting an axis means that the readings from that axis will be negated.
    // This can be useful for correcting the orientation of the sensor in certain applications.
    // The method returns an ESP error code indicating the success or failure of the operation.
    esp_err_t setInvertAxis(bool invertX, bool invertY, bool invertZ);

    // The `setSwitchRollPitch` method sets whether to switch the roll and pitch axes.
    // It takes a boolean parameter indicating whether to switch the axes.
    // Switching the roll and pitch axes means that the readings from these axes will be exchanged.
    // This can be useful for correcting the orientation of the sensor in certain applications.
    // The method returns an ESP error code indicating the success or failure of the operation.
    esp_err_t setSwitchRollPitch(bool switchRollPitch);
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
    FilterMode filterMode;

    // Quaternion state for Mahony filter
    struct Quaternion { float w, x, y, z; } q;

    // Inverted axis
    struct InvertAxis
    {
        int8_t x;
        int8_t y;
        int8_t z;
    } invertAxis = {1, 1, 1};

    // Switch roll and pitch
    bool switchRollPitch;
};
#endif // MPU9250_H