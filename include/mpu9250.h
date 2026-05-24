#ifndef MPU9250_H
#define MPU9250_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "imu_sensor.h"
#include <atomic>
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
#define CALIBRATION_SAMPLES     1000  // 10 s at 100 Hz, sensor immobile
#define MAG_CALIBRATION_SAMPLES 3000  // 30 s at 100 Hz, sensor rotated
#define FILTER_ALPHA 0.96f // Complementary filter constant
// Default Mahony AHRS gains. These can be overridden at runtime through
// setMahonyGains(). Ki is intentionally 0.0 by default: on a drone, a
// non-zero Ki causes integral windup during aggressive maneuvers (rapid
// roll/pitch, prolonged accel != 1g), which can latch a bias and tilt
// the estimated attitude after the maneuver. Only enable Ki (typ. < 0.005)
// if observed gyro bias drift in flight justifies it.
#define MAHONY_KP 0.5f
#define MAHONY_KI 0.0f

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
class MPU9250 : public IMUSensor
{
public:
    // Value types (Vector3, Orientation, Quaternion) and enums
    // (CalibrationStatus, FilterMode) are inherited from IMUSensor. Existing
    // code referencing `MPU9250::Vector3`, `MPU9250::CALIBRATED`, etc. keeps
    // working unchanged thanks to public inheritance scope rules.

    // The `Config` struct groups all the optional initialization parameters
    // for the sensor. It is passed to the `init` method.
    //
    // - mpuAddr   : I2C address of the MPU9250 itself. 0x68 (AD0=GND) or 0x69 (AD0=VCC).
    // - magAddr   : I2C address of the AK8963 magnetometer. Fixed at 0x0C on MPU9250.
    // - sclSpeedHz: SCL clock used for both the MPU and the AK8963 device handles.
    //               400 kHz (Fast Mode) by default — required to sustain the
    //               1 kHz interrupt-driven loop. The MPU9250 and AK8963 are
    //               both rated up to 400 kHz per their respective datasheets.
    //               Lower it to 100 kHz (Standard Mode) only if running on a
    //               shared bus with a slower peripheral or with long PCB
    //               traces that would not meet Fast Mode rise/fall times.
    // - intPin    : GPIO connected to the MPU9250 INT pin. When set, the sensor
    //               task is woken by the DATA_READY interrupt instead of polling
    //               at a fixed period, which dramatically reduces jitter and
    //               lets the task track the actual sample rate (1 kHz with the
    //               default DLPF settings). Set to GPIO_NUM_NC (-1) to keep the
    //               legacy polling behaviour (~100 Hz) for boards where the INT
    //               pin is not wired.
    //               NOTE: to reach 1 kHz wakeup granularity reliably, the
    //               application's FreeRTOS tick should be set to 1000 Hz
    //               (CONFIG_FREERTOS_HZ=1000 in sdkconfig). With INT-driven
    //               wakeups the tick rate only bounds the timeout fallback,
    //               not the nominal loop, so a lower tick still works but the
    //               watchdog/fallback resolution is coarser.
    struct Config
    {
        uint8_t    mpuAddr    = 0x68;
        uint8_t    magAddr    = 0x0C;
        uint32_t   sclSpeedHz = 400000;        // Fast Mode (see notes above)
        gpio_num_t intPin     = GPIO_NUM_NC;

        Config() : mpuAddr(0x68), magAddr(0x0C), sclSpeedHz(400000), intPin(GPIO_NUM_NC) {}
    };

    // Constructor and Destructor
    MPU9250();
    ~MPU9250();

    // Public Methods
    // These methods are used to initialize the sensor, start the sensor task, perform calibration,
    // retrieve sensor data, and check the sensor health status.

    // The `init` method configures the MPU9250 (and AK8963 magnetometer if present)
    // on an I2C bus that is OWNED AND ALREADY INITIALIZED BY THE CALLER.
    //
    // The library does NOT install the I2C driver itself: this lets multiple
    // devices (baro, OSD, ...) share the same bus from the consumer project,
    // and stays compatible with the new ESP-IDF v5 i2c_master driver.
    //
    // Typical usage:
    //   i2c_master_bus_config_t busCfg = {
    //       .i2c_port = I2C_NUM_0,
    //       .sda_io_num = GPIO_NUM_21,
    //       .scl_io_num = GPIO_NUM_22,
    //       .clk_source = I2C_CLK_SRC_DEFAULT,
    //       .glitch_ignore_cnt = 7,
    //       .flags = { .enable_internal_pullup = true },
    //   };
    //   i2c_master_bus_handle_t bus;
    //   ESP_ERROR_CHECK(i2c_new_master_bus(&busCfg, &bus));
    //   imu.init(bus);
    //
    // The destructor removes only the MPU/AK8963 device handles from the bus;
    // the bus itself remains owned by the caller.
    esp_err_t init(i2c_master_bus_handle_t busHandle, const Config& config = Config());

    // -------------------------------------------------------------------------
    // calibrate() — fulfils the IMUSensor interface contract by performing
    // ONLY the gyro+accel calibration (alias of calibrateGyroAccel below).
    //
    // CHANGE vs. v1.0: previously this method also attempted a magnetometer
    // calibration in the same 10-second window, while telling the user to
    // "keep the sensor still" — instructions impossible to satisfy together
    // (mag calibration needs the sensor rotated through every axis). The
    // mag calibration is now a separate explicit call (`calibrateMag()`).
    // -------------------------------------------------------------------------
    esp_err_t calibrate() override;

    // -------------------------------------------------------------------------
    // calibrateGyroAccel — captures the static gyro bias and the accelerometer
    // 1g reference. The sensor must stay IMMOBILE for ~10 seconds (1000 samples
    // at 100 Hz internal). At the end:
    //   - gyroOffset, accelOffset, gyroCalibTemp are updated
    //   - the calibration is auto-persisted to NVS
    //   - calibStatus transitions NOT_CALIBRATED -> CALIBRATING -> CALIBRATED
    //
    // This is the sensible default to run at first boot (or whenever
    // `getCalibrationStatus() != CALIBRATED` after `init()` loaded NVS).
    // It runs asynchronously in its own task; poll `getCalibrationStatus()`.
    // -------------------------------------------------------------------------
    esp_err_t calibrateGyroAccel();

    // -------------------------------------------------------------------------
    // calibrateMag — captures the hard-iron (offset) and soft-iron (scale)
    // corrections of the AK8963 magnetometer. The sensor must be ROTATED
    // through every orientation (figure-8 motion on all three axes) for ~30
    // seconds (3000 samples at 100 Hz internal). At the end:
    //   - magOffset and magScale are updated
    //   - the calibration is auto-persisted to NVS
    //   - calibStatus is NOT changed (mag-only calibration does not affect
    //     gyro/accel readiness)
    //
    // This is typically done ONCE during initial drone setup, not at every
    // boot. Returns ESP_ERR_NOT_SUPPORTED if no magnetometer was detected
    // at init(). Runs asynchronously in its own task.
    // -------------------------------------------------------------------------
    esp_err_t calibrateMag();

    // The `setFilterMode` method sets the filter mode for orientation calculation.
    // It takes a `FilterMode` enum value as a parameter and sets the filter mode accordingly.
    // The filter mode can be set to either COMPLEMENTARY or MAHONY.
    // The choice of filter mode can affect the performance and accuracy of the orientation estimation.
    // The method returns an ESP error code indicating the success or failure of the operation.
    esp_err_t setFilterMode(FilterMode mode) override;

    // The `startSensorTask` method starts the sensor task, which continuously reads sensor data
    // and processes it in the background. It returns an ESP error code indicating the success or failure.
    // The task runs in a FreeRTOS environment and uses a mutex for synchronization.
    // The task is responsible for reading sensor data, applying filters, and updating the orientation.
    // The task also handles calibration and health monitoring.
    // The task runs at a specified frequency, which can be adjusted in the implementation.
    esp_err_t startSensorTask() override;

    // The `getOrientation` method retrieves the current orientation of the sensor.
    // It returns an `Orientation` structure containing the roll, pitch, and yaw angles in degrees.
    // The angles are computed using the complementary or Mahony filter, depending on the filter mode.
    // The roll angle represents the rotation around the x-axis, the pitch angle represents the rotation around the y-axis,
    // and the yaw angle represents the rotation around the z-axis.
    Orientation getOrientation() override;

    // The `getAccel`, `getGyro`, and `getMag` methods retrieve the current accelerometer, gyroscope, and magnetometer readings.
    // They return `Vector3` structures containing the x, y, and z components of the respective sensor data.
    // The accelerometer readings are in g (gravitational units), the gyroscope readings are in degrees per second,
    // and the magnetometer readings are in microteslas (µT).
    Vector3 getAccel() override;
    Vector3 getGyro() override;
    Vector3 getMag() override;

    // Returns the latest unit quaternion produced by the Mahony filter
    // (body-to-world rotation). Drone control loops should use this in
    // preference to `getOrientation()`: it avoids gimbal lock at +/- 90 deg
    // pitch (loops, flips, inverted flight) and is what a quaternion-based
    // attitude controller consumes directly.
    //
    // If the COMPLEMENTARY filter is selected, the returned quaternion is
    // the last one written by the Mahony path (not continuously updated);
    // call setFilterMode(MAHONY) for a live quaternion.
    Quaternion getQuaternion() override;

    // Set the Mahony AHRS proportional / integral gains at runtime.
    // Defaults are 0.5 (Kp) and 0.0 (Ki). Typical drone tuning ranges:
    //   - Kp: 0.5 ... 2.0   (higher = more accel/mag trust, more noise)
    //   - Ki: 0.0 ... 0.005 (small; non-zero only if gyro bias drift is
    //                       observed in flight. Aggressive maneuvers can
    //                       cause Ki integral windup, so keep it small.)
    // Returns ESP_OK on success, ESP_ERR_INVALID_ARG on negative gains.
    esp_err_t setMahonyGains(float kp, float ki);

    // The `getTemperature` method retrieves the current temperature reading from the sensor.
    // It returns the temperature in degrees Celsius.
    // The temperature is measured by the internal temperature sensor of the MPU9250.
    // The temperature reading can be used for compensation in some applications.
    // The temperature is not used for orientation calculation but can be useful for monitoring the sensor's operating conditions.
    float getTemperature() override;

    // The `isSensorHealthy` method checks the health status of the sensor.
    // It returns true if the sensor is healthy and able to provide valid readings,
    // and false if there are issues with the sensor communication or data retrieval.
    // The health status is monitored by counting the number of successful and failed sensor readings.
    // The method can be used to detect sensor malfunctions or communication errors.
    // The health status can be used to trigger error handling or fallback mechanisms in the application.
    // The method can also be used to check if the sensor is ready for use after initialization or calibration.
    bool isSensorHealthy() override;

    // The `getCalibrationStatus` method retrieves the current calibration status of the sensor.
    // It returns a `CalibrationStatus` enum value indicating whether the sensor is not calibrated,
    // currently being calibrated, or has been calibrated successfully.
    CalibrationStatus getCalibrationStatus() override { return calibStatus; }

    // The `setInvertAxis` method sets the axis inversion for the sensor readings.
    // It takes three boolean parameters indicating whether to invert the x, y, and z axes respectively.
    // Inverting an axis means that the readings from that axis will be negated.
    // This can be useful for correcting the orientation of the sensor in certain applications.
    // The method returns an ESP error code indicating the success or failure of the operation.
    esp_err_t setInvertAxis(bool invertX, bool invertY, bool invertZ) override;

    // The `setSwitchRollPitch` method sets whether to switch the roll and pitch axes.
    // It takes a boolean parameter indicating whether to switch the axes.
    // Switching the roll and pitch axes means that the readings from these axes will be exchanged.
    // This can be useful for correcting the orientation of the sensor in certain applications.
    // The method returns an ESP error code indicating the success or failure of the operation.
    esp_err_t setSwitchRollPitch(bool switchRollPitch) override;

    // -------------------------------------------------------------------------
    // Atomic snapshot + new-sample signalling — overrides of IMUSensor.
    // -------------------------------------------------------------------------
    // `getSnapshot()` reads a coherent set via a seqlock published by the
    // sensor task (no mutex on the reader path). Worst-case it loops a few
    // times if the reader was preempted exactly during a writer publish.
    //
    // `waitForNewSample()` blocks the calling task on an internal binary
    // semaphore that the sensor task signals on every publish. Because the
    // semaphore is binary (not counting), a slow consumer never accumulates
    // backlog: it always wakes up to the most recent sample.
    SampleBundle getSnapshot() override;
    esp_err_t    waitForNewSample(uint32_t timeoutMs) override;

    // -------------------------------------------------------------------------
    // Gyro notch filter (motor-vibration suppression)
    // -------------------------------------------------------------------------
    // Drones typically vibrate at the motor fundamental frequency (commonly
    // 100-400 Hz for small quads, up to ~600 Hz for racers). The MPU9250's
    // on-chip DLPF (set to ~41 Hz) suppresses higher harmonics but the
    // fundamental still leaks into the gyro stream. Without rejection, the
    // PID chases the vibration -> motor heating, oscillation, poor
    // attitude hold.
    //
    // This installs a biquad notch (IIR, Direct Form II Transposed) on each
    // gyro axis. Use `setGyroNotch` with the measured motor frequency at
    // hover (e.g. RPM / 60 for a brushed motor, RPM * poles/2 / 60 for a
    // BLDC, or measure with a vibration log). Bandwidth typically 30-80 Hz.
    //
    //   sampleRateHz must match what the sensor task is actually running at:
    //     1000 Hz when DATA_READY interrupt is wired (default Config),
    //     ~100 Hz in polling fallback (Config::intPin = GPIO_NUM_NC).
    //
    // `clearGyroNotch()` disables the filter (passthrough) and resets state.
    // The filter is disabled by default — behaviour is unchanged until called.
    esp_err_t setGyroNotch(float centerHz, float bandwidthHz, float sampleRateHz = 1000.0f);
    esp_err_t clearGyroNotch();

    // -------------------------------------------------------------------------
    // Calibration persistence (NVS)
    // -------------------------------------------------------------------------
    // The consumer project is responsible for initialising NVS once at startup
    // (`nvs_flash_init()`). The library only opens/closes a namespace.
    //
    // `loadCalibration()` is called automatically at the end of `init()`; if
    // a valid blob is found, the sensor comes up already calibrated and skips
    // the 10-second immobility window. Returns ESP_ERR_NOT_FOUND if no calib
    // is stored, ESP_ERR_NVS_NOT_INITIALIZED if the consumer never initialised
    // NVS, or another esp_err_t on I/O failure.
    //
    // `saveCalibration()` is called automatically at the end of both
    // `calibrateGyroAccel()` and `calibrateMag()` once new offsets have been
    // computed. You only need to call it manually after
    // `setGyroTempCompCoeff()` if you want the new coefficient persisted
    // without re-running a full calibration.
    //
    // `clearStoredCalibration()` erases the blob. Useful to force a fresh
    // calibration on the next boot (e.g. after swapping the sensor module).
    esp_err_t saveCalibration();
    esp_err_t loadCalibration();
    esp_err_t clearStoredCalibration();

    // -------------------------------------------------------------------------
    // Per-axis gyro bias temperature compensation coefficient (deg/s per degC).
    // -------------------------------------------------------------------------
    // Effective offset = gyroOffset + coeff * (currentTemp - gyroCalibTemp).
    //
    // MPU9250 gyro bias typically drifts ~0.01 deg/s per degC, with the sign
    // and magnitude varying per chip. To measure for a specific device:
    //   1. Calibrate cold (right after power-on, e.g. 25 degC), note bias1.
    //   2. Let the device self-heat for several minutes, then sample raw
    //      gyro again at ~50 degC, note bias2.
    //   3. coeff = (bias2 - bias1) / (T2 - T1) for each axis.
    //
    // Default {0, 0, 0} = compensation disabled (no behaviour change). Once
    // a coefficient is set, the next saveCalibration() persists it alongside
    // the static offsets.
    esp_err_t setGyroTempCompCoeff(Vector3 coeff);
private:
    // I2C Communication
    // The `dev` handle selects which device on the bus is addressed
    // (mpuDev for the MPU9250 itself, magDev for the AK8963 magnetometer).
    esp_err_t writeRegister(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t data);
    esp_err_t readRegisters(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t length, uint8_t *data);

    // Sensor Reading (single-axis helpers kept for the calibration path).
    // The hot-loop in sensorTask uses an inline 14-byte burst read for
    // throughput; these helpers are only used during the (slow) calibration
    // routine where readability matters more than per-read overhead.
    float readAccel(uint8_t axisOffset);
    float readGyro(uint8_t axisOffset);
    float readMag(uint8_t axisOffset);

    // Sensor Processing
    static void sensorTask(void *arg);

    // Static ISR handler for the MPU9250 DATA_READY pin. Receives `this` via
    // the GPIO ISR user-arg, then notifies the sensor task with
    // vTaskNotifyGiveFromISR. Kept in IRAM so it is callable when the flash
    // cache is disabled.
    static void IRAM_ATTR dataReadyIsr(void *arg);
    void processMeasurements(float dt);
    void updateComplementaryFilter(float dt);
    void updateMahonyFilter(float dt);
    float computeHeadingFromMag();

    // Calibration — split into two independent paths since gyro/accel and
    // mag have contradictory user instructions (immobile vs. rotated).
    void performGyroAccelCalibration();
    void performMagCalibration();
    void resetGyroAccelOffsets();
    void resetMagOffsets();

    // Variables
    // I2C bus and device handles. The bus is owned by the CALLER (not freed
    // by this class). The two device handles are created in `init` and
    // released in the destructor via `i2c_master_bus_rm_device`.
    i2c_master_bus_handle_t busHandle;
    i2c_master_dev_handle_t mpuDev;
    i2c_master_dev_handle_t magDev;

    // GPIO pin wired to the MPU9250 INT (DATA_READY) line. GPIO_NUM_NC means
    // the polling fallback is used in the sensor task.
    gpio_num_t intPin;

    TaskHandle_t taskHandle;
    // dataMutex still guards INTERNAL mutations (calibration writes,
    // setters). It is NEVER taken on the consumer read path: that goes
    // through the seqlock-backed `publishedBundle` below.
    SemaphoreHandle_t dataMutex;
    // Binary semaphore given by the sensor task on every successful publish.
    // Consumers block on this via `waitForNewSample()`. Binary (not
    // counting) so a slow consumer never builds up a backlog.
    SemaphoreHandle_t sampleSem;

    // ---- Seqlock-protected published snapshot ------------------------------
    // The writer (sensor task) bumps `bundleSeq` once before and once after
    // writing `publishedBundle`. Readers loop until they see two equal even
    // values around their copy. One writer + N readers, lock-free reads.
    SampleBundle              publishedBundle;
    std::atomic<uint32_t>     bundleSeq;

    // ---- Gyro notch biquad (Direct Form II Transposed) ---------------------
    // Coefficients are recomputed in setGyroNotch() and stored here. The
    // sensor task reads them at the top of each iteration into a local copy
    // (so a mid-flight setGyroNotch never tears a single sample). State (z1,
    // z2) is per-axis and only touched by the sensor task.
    struct NotchCoeffs { float b0, b1, b2, a1, a2; };
    struct NotchState  { float z1, z2; };
    NotchCoeffs                gyroNotchCoeffs;
    NotchState                 gyroNotchState[3];
    std::atomic<bool>          gyroNotchEnabled;

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
    // Temperature (degC) recorded at the moment gyroOffset was captured.
    // Used by the runtime compensation: see `setGyroTempCompCoeff`.
    float   gyroCalibTemp;
    // Per-axis linear coefficient applied in the sensor task to compensate
    // gyro bias drift with temperature. {0,0,0} = no compensation.
    Vector3 gyroTempCompCoeff;
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

    // Quaternion state for Mahony filter (body-to-world rotation).
    // Type definition lives in the public section above.
    Quaternion q;

    // Mahony AHRS gains. Initialised to MAHONY_KP / MAHONY_KI defined at the
    // top of this header; can be retuned at runtime via setMahonyGains().
    float mahonyKp;
    float mahonyKi;

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