#include "mpu9250.h"
#include "nvs.h"

// -----------------------------------------------------------------------------
// Calibration persistence
// -----------------------------------------------------------------------------
// All offsets / scales / per-chip mag fuse values are stored as a single
// versioned blob under the namespace below. Versioning lets us evolve the
// layout without silently feeding garbage to a new firmware.
//
// The consumer project owns NVS init (`nvs_flash_init()`); this library only
// opens/closes a namespace. If the consumer never initialised NVS, the
// load/save paths log a warning and return ESP_ERR_NVS_NOT_INITIALIZED
// without crashing the rest of the IMU bring-up.
// -----------------------------------------------------------------------------
#define MPU9250_NVS_NAMESPACE "imu_lib"
#define MPU9250_NVS_KEY       "calib"
// v2 ajoute qHomeInverse + homeIsSet (cf. setHome / clearHome).
// Les blobs v1 plus anciens sont rejetés au load (force recalibration).
#define MPU9250_CALIB_VERSION 3

namespace
{
    // v2 layout. v1 (sans qHomeInverse) est rejeté au load -> force
    // recalibration. Pour bumper à v3, étendre ici puis incrémenter
    // MPU9250_CALIB_VERSION et le static_assert.
    struct CalibBlob
    {
        uint8_t version;
        uint8_t magAvailable; // mirrors detection at calibration time
        uint8_t homeIsSet;    // v2: 0/1 — drives whether qHomeInverse is applied
        uint8_t padding;
        float   accelOffset[3];
        float   gyroOffset[3];
        float   gyroCalibTemp;
        float   gyroTempCompCoeff[3];
        float   magOffset[3];
        float   magScale[3];
        uint8_t magAdjustValues[3];
        uint8_t padding2;
        float   qHomeInverse[4]; // v2: roll/pitch-only mount offset (conjugated)
    };
    static_assert(sizeof(CalibBlob) == 4 + 12 + 12 + 4 + 12 + 12 + 12 + 4 + 16,
                  "CalibBlob size unexpected — review NVS layout before bumping version");

    // Conversion Quaternion -> Euler (Degrés)
    inline IMUSensor::Orientation quatToEuler(const IMUSensor::Quaternion& q, bool switchRollPitch)
    {
        IMUSensor::Orientation euler;
        if (switchRollPitch)
        {
            euler.pitch = atan2f(2.0f*(q.w*q.x + q.y*q.z), 1.0f - 2.0f*(q.x*q.x + q.y*q.y)) * RAD_TO_DEG;
            euler.roll  = asinf( 2.0f*(q.w*q.y - q.z*q.x)) * RAD_TO_DEG;
        } 
        else 
        {
            euler.roll  = atan2f(2.0f*(q.w*q.x + q.y*q.z), 1.0f - 2.0f*(q.x*q.x + q.y*q.y)) * RAD_TO_DEG;
            euler.pitch = asinf( 2.0f*(q.w*q.y - q.z*q.x)) * RAD_TO_DEG;
        }

        euler.yaw = atan2f(2.0f*(q.w*q.z + q.x*q.y), 1.0f - 2.0f*(q.y*q.y + q.z*q.z)) * RAD_TO_DEG;

        // Normalisation [0, 360)
        if (euler.yaw < 0.0f) euler.yaw += 360.0f;
        else if (euler.yaw >= 360.0f) euler.yaw -= 360.0f;

        return euler;
    }
}

// I2C transaction timeout in ms used for register reads/writes.
// Generous enough for clock stretching on slow buses; tight enough that
// the sensor task does not hang on a wire fault.
#define MPU9250_I2C_TIMEOUT_MS 100

// Enable the per-loop profiler block in sensorTask. Off by default to avoid
// polluting the production log stream and to remove the small per-iteration
// overhead of esp_timer_get_time() calls. Override from build:
//   PlatformIO: build_flags = -DMPU9250_PROFILER=1
//   ESP-IDF   : idf_component_register(... PRIV_REQUIRES ... PRIV_REQ_CFG)
#ifndef MPU9250_PROFILER
#define MPU9250_PROFILER 0
#endif

// -----------------------------------------------------------------------------
// File-local quaternion helpers (used by setHome / Mahony output composition).
// Kept here to avoid polluting the public Quaternion type in imu_sensor.h with
// member operators that not every consumer needs.
// -----------------------------------------------------------------------------
namespace
{
    using Q = IMUSensor::Quaternion;

    // Hamilton product: r = a * b
    inline Q quatMul(const Q& a, const Q& b)
    {
        return {
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
        };
    }

    // Conjugate (= inverse for unit quaternion): q* = (w, -x, -y, -z)
    inline Q quatConj(const Q& q)
    {
        return { q.w, -q.x, -q.y, -q.z };
    }
}

// Implementation
MPU9250::MPU9250()
    : busHandle(nullptr),
      mpuDev(nullptr),
      magDev(nullptr),
      intPin(GPIO_NUM_NC),
      taskCoreId(tskNO_AFFINITY),
      taskPriority(5),
      taskStackSize(4096),
      taskHandle(nullptr),
      dataMutex(nullptr),
      sampleSem(nullptr),
      publishedBundle{},
      bundleSeq(0),
      gyroNotchCoeffs{1.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // passthrough
      gyroNotchEnabled(false),
      accel{0, 0, 0},
      gyro{0, 0, 0},
      mag{0, 0, 0},
      temperature(0),
      orientation{0, 0, 0},
      gyroIntegrated{0, 0, 0},
      mahonyIntegralError{0, 0, 0},
      magHeading(0),
      accelOffset{0, 0, 0},
      gyroOffset{0, 0, 0},
      gyroCalibTemp(25.0f),               // reasonable cold-start default
      gyroTempCompCoeff{0.0f, 0.0f, 0.0f}, // {0,0,0} = compensation disabled
      magOffset{0, 0, 0},
      magScale{1.0f, 1.0f, 1.0f},
      calibStatus(NOT_CALIBRATED),
      errorCount(0),
      successCount(0),
      magAvailable(false),
      filterMode(COMPLEMENTARY),
      q{1.0f, 0.0f, 0.0f, 0.0f}, // Identity quaternion (no rotation)
      mahonyKp(MAHONY_KP),
      mahonyKi(MAHONY_KI),
      mahonyBoostKp(10.0f),
      mahonyBoostDurationMs(3000),
      mahonyBoostUntilUs(0),               // 0 = boost not currently active
      qHomeInverse{1.0f, 0.0f, 0.0f, 0.0f}, // identity = no offset
      homeIsSet(false),
      switchRollPitch(false)
{
    memset(magAdjustValues, 0, sizeof(magAdjustValues));
    memset(gyroNotchState, 0, sizeof(gyroNotchState));
    dataMutex = xSemaphoreCreateMutex();
    // Binary semaphore is created "empty" — first waitForNewSample() blocks
    // until the sensor task gives it after the first publish.
    sampleSem = xSemaphoreCreateBinary();
}

MPU9250::~MPU9250()
{
    // Detach the GPIO ISR BEFORE deleting the task: otherwise an interrupt
    // could fire and try to notify a freed task handle.
    if (intPin != GPIO_NUM_NC)
    {
        gpio_isr_handler_remove(intPin);
        // Best-effort: disable the DATA_READY interrupt source on the MPU
        // itself so it stops driving the line after we're gone.
        if (mpuDev != nullptr)
            writeRegister(mpuDev, 0x38, 0x00); // INT_ENABLE = 0
        intPin = GPIO_NUM_NC;
    }

    if (taskHandle != nullptr)
    {
        vTaskDelete(taskHandle);
        taskHandle = nullptr;
    }

    // Remove device handles from the bus. We do NOT delete the bus itself:
    // it is owned by the caller and may have other devices attached.
    if (mpuDev != nullptr)
    {
        i2c_master_bus_rm_device(mpuDev);
        mpuDev = nullptr;
    }
    if (magDev != nullptr)
    {
        i2c_master_bus_rm_device(magDev);
        magDev = nullptr;
    }

    if (dataMutex != nullptr)
    {
        vSemaphoreDelete(dataMutex);
        dataMutex = nullptr;
    }
    if (sampleSem != nullptr)
    {
        // Best-effort: wake any blocked consumer before destroying the
        // semaphore so it exits waitForNewSample() rather than blocking on
        // a freed handle. The consumer is expected to be stopped by this
        // point — this is just a safety net.
        xSemaphoreGive(sampleSem);
        vSemaphoreDelete(sampleSem);
        sampleSem = nullptr;
    }
}

esp_err_t MPU9250::init(i2c_master_bus_handle_t bus, const Config& config)
{
    if (bus == nullptr)
    {
        ESP_LOGE(TAG_MPU9250, "init: bus handle is null");
        return ESP_ERR_INVALID_ARG;
    }

    busHandle             = bus;
    intPin                = config.intPin;
    taskCoreId            = config.taskCoreId;
    taskPriority          = config.taskPriority;
    taskStackSize         = config.taskStackSize;
    mahonyBoostKp         = config.mahonyBoostKp;
    mahonyBoostDurationMs = config.mahonyBoostDurationMs;

    // Attach the MPU9250 itself as a device on the bus.
    i2c_device_config_t mpuCfg = {};
    mpuCfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    mpuCfg.device_address  = config.mpuAddr;
    mpuCfg.scl_speed_hz    = config.sclSpeedHz;

    esp_err_t err = i2c_master_bus_add_device(busHandle, &mpuCfg, &mpuDev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_MPU9250, "Failed to add MPU device (0x%02x): %d", config.mpuAddr, err);
        return err;
    }

    // Check MPU9250 identity
    uint8_t whoami = 0;
    err = readRegisters(mpuDev, MPU9250_WHO_AM_I, 1, &whoami);
    if (err != ESP_OK || whoami != 0x71)
    {
        ESP_LOGE(TAG_MPU9250, "MPU9250 not found, WHO_AM_I = 0x%02x (err=%d)", whoami, err);
        return ESP_FAIL;
    }

    // Reset device
    err = writeRegister(mpuDev, MPU9250_PWR_MGMT_1, 0x80);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MPU9250, "Failed to reset MPU9250");
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    // Wake up device
    err = writeRegister(mpuDev, MPU9250_PWR_MGMT_1, 0x01);
    if (err != ESP_OK){
        ESP_LOGE(TAG_MPU9250, "Failed to wake up MPU9250");
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Sample Rate Divider: Sample Rate = Internal_Sample_Rate / (1 + SMPLRT_DIV).
    // With DLPF enabled (CONFIG below uses DLPF_CFG != 0 and != 7), Internal
    // Sample Rate is fixed at 1 kHz, so writing 0 here makes the MPU produce
    // one fresh accel+gyro sample every 1 ms. The DATA_READY interrupt then
    // pulses at 1 kHz, which is the rate consumed by the sensor task when
    // `Config::intPin` is wired.
    err = writeRegister(mpuDev, 0x19, 0x00); // SMPLRT_DIV = 0 -> 1 kHz
    if (err != ESP_OK)
        return err;

    // Configure gyro and accel
    err = writeRegister(mpuDev, 0x1A, 0x03); // CONFIG: DLPF_CFG = 3
    if (err != ESP_OK)
        return err;

    err = writeRegister(mpuDev, 0x1B, 0x10); // GYRO_CONFIG: ±1000 dps
    if (err != ESP_OK)
        return err;

    err = writeRegister(mpuDev, 0x1C, 0x08); // ACCEL_CONFIG: ±4g
    if (err != ESP_OK)
        return err;

    err = writeRegister(mpuDev, 0x1D, 0x03); // ACCEL_CONFIG2: DLPF_CFG = 3
    if (err != ESP_OK)
        return err;

    // Configure interrupt pin (INT_PIN_CFG register, datasheet table 21):
    //   bit 7 ACTL              = 0  -> active-high
    //   bit 6 OPEN              = 0  -> push-pull
    //   bit 5 LATCH_INT_EN      = 0  -> 50 us pulse (NOT latched). With
    //                                   LATCH=1, the INT line stayed high
    //                                   forever after the first sample
    //                                   unless we read INT_STATUS (0x3A),
    //                                   so the GPIO POSEDGE ISR fired
    //                                   exactly once and the sensor task
    //                                   fell back to the 20 ms timeout.
    //   bit 4 INT_ANYRD_2CLEAR  = 1  -> interrupt is cleared by any read,
    //                                   which our 14-byte burst already
    //                                   does on every iteration.
    //   bit 1 BYPASS_EN         = 1  -> AK8963 reachable as a separate
    //                                   device on the same I2C bus.
    //   => 0b0001 0010 = 0x12
    err = writeRegister(mpuDev, MPU9250_INT_PIN_CFG, 0x12);
    if (err != ESP_OK)
        return err;

    vTaskDelay(pdMS_TO_TICKS(10));

    // Attach the AK8963 magnetometer as a second device on the same bus.
    // Once attached, presence is verified via WHO_AM_I; if missing, the
    // handle is released and the rest of the lib works without magnetometer.
    i2c_device_config_t magCfg = {};
    magCfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    magCfg.device_address  = config.magAddr;
    magCfg.scl_speed_hz    = config.sclSpeedHz;

    err = i2c_master_bus_add_device(busHandle, &magCfg, &magDev);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG_MPU9250, "Failed to add AK8963 device (0x%02x): %d — continuing without mag", config.magAddr, err);
        magDev = nullptr;
        magAvailable = false;
        return ESP_OK;
    }

    uint8_t magWhoami = 0;
    err = readRegisters(magDev, AK8963_WHO_AM_I, 1, &magWhoami);
    if (err == ESP_OK && magWhoami == 0x48)
    {
        ESP_LOGI(TAG_MPU9250, "AK8963 magnetometer found");
        magAvailable = true;

        // Reset AK8963
        err = writeRegister(magDev, AK8963_CNTL1, 0x00); // Power down
        if (err != ESP_OK)
            return err;
        vTaskDelay(pdMS_TO_TICKS(10));

        // Read adjustment values
        err = writeRegister(magDev, AK8963_CNTL1, 0x0F); // Fuse ROM access mode
        if (err != ESP_OK)
            return err;
        vTaskDelay(pdMS_TO_TICKS(10));

        err = readRegisters(magDev, AK8963_ASAX, 3, magAdjustValues);
        if (err != ESP_OK)
            return err;

        // Set to continuous mode 2 (100Hz)
        err = writeRegister(magDev, AK8963_CNTL1, 0x16);
        if (err != ESP_OK)
            return err;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    else
    {
        ESP_LOGW(TAG_MPU9250, "AK8963 magnetometer not found (whoami=0x%02x)", magWhoami);
        magAvailable = false;
        i2c_master_bus_rm_device(magDev);
        magDev = nullptr;
    }

    // Try to load a previously stored calibration. Best-effort: if NVS is not
    // initialised by the consumer, or if no blob exists yet, we silently fall
    // through with calibStatus == NOT_CALIBRATED. The user can then call
    // calibrate() to produce and persist a fresh set of offsets.
    esp_err_t loadErr = loadCalibration();
    if (loadErr == ESP_OK)
    {
        ESP_LOGI(TAG_MPU9250, "Loaded calibration from NVS — skipping cold calibration");
    }
    else if (loadErr == ESP_ERR_NOT_FOUND)
    {
        ESP_LOGI(TAG_MPU9250, "No stored calibration — call calibrate() to create one");
    }
    else
    {
        ESP_LOGW(TAG_MPU9250, "loadCalibration failed (err=%d) — continuing without persisted offsets", loadErr);
    }

    return ESP_OK;
}

esp_err_t MPU9250::writeRegister(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t data)
{
    if (dev == nullptr)
        return ESP_ERR_INVALID_STATE;

    const uint8_t buffer[2] = {reg, data};
    esp_err_t ret = i2c_master_transmit(dev, buffer, sizeof(buffer), MPU9250_I2C_TIMEOUT_MS);

    if (ret != ESP_OK)
    {
        errorCount++;
        ESP_LOGD(TAG_MPU9250, "Write register failed: reg=0x%02x, err=%d", reg, ret);
    }
    else
    {
        successCount++;
    }

    return ret;
}

esp_err_t MPU9250::readRegisters(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
    if (dev == nullptr || data == nullptr || length == 0)
        return ESP_ERR_INVALID_ARG;

    // Combined write-register-address + read in a single transaction so a
    // repeated-start is emitted on the bus (required by the MPU register map).
    esp_err_t ret = i2c_master_transmit_receive(dev, &reg, 1, data, length, MPU9250_I2C_TIMEOUT_MS);

    if (ret != ESP_OK)
    {
        errorCount++;
        ESP_LOGD(TAG_MPU9250, "Read registers failed: reg=0x%02x, len=%d, err=%d", reg, length, ret);
    }
    else
    {
        successCount++;
    }

    return ret;
}

float MPU9250::readAccel(uint8_t axisOffset)
{
    uint8_t rawData[2] = {0};
    if (readRegisters(mpuDev, MPU9250_ACCEL_XOUT_H + axisOffset, 2, rawData) == ESP_OK)
    {
        int16_t value = (((int16_t)rawData[0]) << 8) | rawData[1];
        // Scale for ±4g range
        return (float)value / 8192.0f; // 16bit / 4g = 8192 LSB/g
    }
    return 0.0f;
}

float MPU9250::readGyro(uint8_t axisOffset)
{
    uint8_t rawData[2] = {0};
    if (readRegisters(mpuDev, MPU9250_GYRO_XOUT_H + axisOffset, 2, rawData) == ESP_OK)
    {
        int16_t value = (((int16_t)rawData[0]) << 8) | rawData[1];
        // Scale for ±1000 dps range
        return (float)value / 32.8f; // 16bit / 1000dps = 32.8 LSB/°/s
    }
    return 0.0f;
}

float MPU9250::readMag(uint8_t axisOffset)
{
    if (!magAvailable)
        return 0.0f;

    // Check data ready
    uint8_t st1;
    if (readRegisters(magDev, AK8963_ST1, 1, &st1) != ESP_OK || !(st1 & 0x01))
    {
        return 0.0f;
    }

    uint8_t rawData[2] = {0};
    if (readRegisters(magDev, AK8963_HXL + axisOffset, 2, rawData) == ESP_OK)
    {
        int16_t value = (((int16_t)rawData[1]) << 8) | rawData[0]; // Little endian

        // Apply factory calibration
        float adjust = (((float)magAdjustValues[axisOffset / 2] - 128.0f) / 256.0f + 1.0f);
        // Scale for 16-bit mode (4912 uT full scale)
        return (float)value * 0.15f * adjust; // 16bit / 4912uT = 0.15 LSB/uT
    }
    return 0.0f;
}

// NOTE: `readAllSensors()` and `computeAnglesFromAccel()` were removed in the
// MPU9250 refactor — the hot loop in `sensorTask` does an inline 14-byte
// burst read (much faster than 3 single-axis I2C transactions per sensor),
// and the Mahony / complementary filters compute attitude from accel + gyro
// + mag directly. Neither helper was on any code path.

float MPU9250::computeHeadingFromMag()
{
    if (!magAvailable)
        return 0.0f;

    // Tilt-compensated magnetic heading
    float cosRoll = cosf(orientation.roll * DEG_TO_RAD);
    float sinRoll = sinf(orientation.roll * DEG_TO_RAD);
    float cosPitch = cosf(orientation.pitch * DEG_TO_RAD);
    float sinPitch = sinf(orientation.pitch * DEG_TO_RAD);

    // Tilt-compensated magnetic field components
    float magX = mag.x * cosPitch + mag.y * sinRoll * sinPitch + mag.z * cosRoll * sinPitch;
    float magY = mag.y * cosRoll - mag.z * sinRoll;

    float heading = atan2f(-magY, magX) * RAD_TO_DEG;

    // Normalize to 0-360
    while (heading < 0)
        heading += 360.0f;
    while (heading >= 360.0f)
        heading -= 360.0f;

    return heading;
}

void MPU9250::updateComplementaryFilter(float dt)
{
    // Integrate gyro rates
    gyroIntegrated.x += gyro.x * dt;
    gyroIntegrated.y += gyro.y * dt;
    gyroIntegrated.z += gyro.z * dt;

    // Compute accelerometer angles
    float rollAcc = atan2f(accel.y, accel.z) * RAD_TO_DEG;
    float pitchAcc = atan2f(-accel.x, sqrtf(accel.y * accel.y + accel.z * accel.z)) * RAD_TO_DEG;

    // Complementary filter
    if (switchRollPitch)
    {
        orientation.roll = FILTER_ALPHA * (orientation.roll + gyro.y * dt) + (1.0f - FILTER_ALPHA) * rollAcc;
        orientation.pitch = FILTER_ALPHA * (orientation.pitch + gyro.x * dt) + (1.0f - FILTER_ALPHA) * pitchAcc;
    }
    else
    {
        orientation.roll = FILTER_ALPHA * (orientation.roll + gyro.x * dt) + (1.0f - FILTER_ALPHA) * rollAcc;
        orientation.pitch = FILTER_ALPHA * (orientation.pitch + gyro.y * dt) + (1.0f - FILTER_ALPHA) * pitchAcc;
    }

    // Update yaw from gyro or magnetometer
    if (magAvailable)
    {
        float heading = computeHeadingFromMag();
        // Only use mag data when sensor is stable (low acceleration)
        float accelMagnitude = sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
        if (accelMagnitude > 0.85f && accelMagnitude < 1.15f)
        {
            orientation.yaw = FILTER_ALPHA * (orientation.yaw + gyro.z * dt) + (1.0f - FILTER_ALPHA) * heading;
        }
        else
        {
            orientation.yaw += gyro.z * dt;
        }
    }
    else
    {
        orientation.yaw += gyro.z * dt;
    }

    // Normalize yaw to 0-360
    while (orientation.yaw < 0)
        orientation.yaw += 360.0f;
    while (orientation.yaw >= 360.0f)
        orientation.yaw -= 360.0f;
}

void MPU9250::updateMahonyFilter(float dt)
{
    // -------------------------------------------------------------------------
    // Mahony AHRS — 9-DOF when magnetometer is valid, 6-DOF fallback otherwise.
    // -------------------------------------------------------------------------
    // Reference: R. Mahony, T. Hamel, J.-M. Pflimlin, "Nonlinear Complementary
    // Filters on the Special Orthogonal Group", IEEE T-AC 2008. The numerical
    // recipe below follows the canonical x-io Technologies implementation
    // (Madgwick/Mahony AHRS) with one error vector built from BOTH the
    // accelerometer (gravity reference) AND the magnetometer (horizontal
    // magnetic-field reference). Without the mag term the yaw is only
    // integrated from the gyro and drifts unbounded — that was the previous
    // bug. With the mag term, yaw is locked to magnetic north.
    // -------------------------------------------------------------------------

    // Convert gyro to rad/s
    float gx = gyro.x * DEG_TO_RAD;
    float gy = gyro.y * DEG_TO_RAD;
    float gz = gyro.z * DEG_TO_RAD;

    // Unpack quaternion state into locals (cheaper than repeated field access)
    float q0 = q.w;
    float q1 = q.x;
    float q2 = q.y;
    float q3 = q.z;

    // -- Normalize accelerometer (must have non-zero magnitude) ----------------
    float accNorm = sqrtf(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z);
    if (accNorm == 0.0f) return; // gravity reference unusable -> skip update
    float invAccNorm = 1.0f / accNorm;
    float ax = accel.x * invAccNorm;
    float ay = accel.y * invAccNorm;
    float az = accel.z * invAccNorm;

    // -- Estimated direction of gravity in body frame --------------------------
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // -- Error from accelerometer (cross product gravity reference) ------------
    float ex = (ay*vz - az*vy);
    float ey = (az*vx - ax*vz);
    float ez = (ax*vy - ay*vx);

    // -- Add magnetometer error if available and valid -------------------------
    // We compute the world-frame magnetic field expected from the current
    // attitude, then form the cross product with the measured mag direction.
    // When the magnetometer is missing or returns a zero vector (e.g. during
    // calibration or hardware fault), we fall back to 6-DOF behaviour.
    if (magAvailable)
    {
        float magNorm = sqrtf(mag.x*mag.x + mag.y*mag.y + mag.z*mag.z);
        if (magNorm > 0.0f)
        {
            float invMagNorm = 1.0f / magNorm;
            float mx = mag.x * invMagNorm;
            float my = mag.y * invMagNorm;
            float mz = mag.z * invMagNorm;

            // Project measured magnetic field into the world frame (h = q * m * q*)
            float hx = 2.0f * (mx*(0.5f - q2*q2 - q3*q3) + my*(q1*q2 - q0*q3) + mz*(q1*q3 + q0*q2));
            float hy = 2.0f * (mx*(q1*q2 + q0*q3) + my*(0.5f - q1*q1 - q3*q3) + mz*(q2*q3 - q0*q1));
            float hz = 2.0f * (mx*(q1*q3 - q0*q2) + my*(q2*q3 + q0*q1) + mz*(0.5f - q1*q1 - q2*q2));

            // Reference magnetic field: project onto X-Z plane of the world frame
            // (horizontal component = bx, vertical = bz). This is what the field
            // SHOULD look like in world coordinates given the current attitude.
            float bx = sqrtf(hx*hx + hy*hy);
            float bz = hz;

            // Estimated direction of magnetic field in body frame
            float wx = 2.0f * (bx*(0.5f - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2));
            float wy = 2.0f * (bx*(q1*q2 - q0*q3) + bz*(q0*q1 + q2*q3));
            float wz = 2.0f * (bx*(q0*q2 + q1*q3) + bz*(0.5f - q1*q1 - q2*q2));

            // Cross product (measured x estimated) — adds yaw correction
            ex += (my*wz - mz*wy);
            ey += (mz*wx - mx*wz);
            ez += (mx*wy - my*wx);
        }
    }

    // -- Integral feedback (compensates slow gyro bias drift) ------------------
    if (mahonyKi > 0.0f) {
        mahonyIntegralError.x += ex * mahonyKi * dt;
        mahonyIntegralError.y += ey * mahonyKi * dt;
        mahonyIntegralError.z += ez * mahonyKi * dt;
        gx += mahonyIntegralError.x;
        gy += mahonyIntegralError.y;
        gz += mahonyIntegralError.z;
    } else {
        // Prevent windup if Ki is later re-enabled after being zero
        mahonyIntegralError = {0.0f, 0.0f, 0.0f};
    }

    // -- Proportional feedback -------------------------------------------------
    // Fast-init boost: temporarily raise Kp at boot (and after
    // calibrateGyroAccel) so the filter converges from identity to the
    // actual attitude in well under 1 s instead of the 5-10 s observed
    // with nominal Kp + any residual gyro bias.
    float effectiveKp = mahonyKp;
    if (mahonyBoostUntilUs > 0) {
        if (esp_timer_get_time() < mahonyBoostUntilUs) {
            effectiveKp = mahonyBoostKp;
        } else {
            mahonyBoostUntilUs = 0;
            ESP_LOGI(TAG_MPU9250, "Mahony boost ended, Kp restored to %.2f", mahonyKp);
        }
    }

    gx += effectiveKp * ex;
    gy += effectiveKp * ey;
    gz += effectiveKp * ez;

    // -- Integrate quaternion derivative ---------------------------------------
    float half_dt = 0.5f * dt;
    float qa = q0;
    float qb = q1;
    float qc = q2;
    float qd = q3;

    q0 += (-qb * gx - qc * gy - qd * gz) * half_dt;
    q1 += ( qa * gx + qc * gz - qd * gy) * half_dt;
    q2 += ( qa * gy - qb * gz + qd * gx) * half_dt;
    q3 += ( qa * gz + qb * gy - qc * gx) * half_dt;

    // -- Normalize quaternion --------------------------------------------------
    float qNorm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (qNorm == 0.0f) return; // numerical pathology, leave previous q intact
    float invQNorm = 1.0f / qNorm;
    q.w = q0 * invQNorm;
    q.x = q1 * invQNorm;
    q.y = q2 * invQNorm;
    q.z = q3 * invQNorm;

    // Convert quaternion to Euler angles (deg)
    orientation = quatToEuler(q, switchRollPitch);
}

void MPU9250::processMeasurements(float dt)
{
    // Process based on selected filter
    if (filterMode == COMPLEMENTARY)
    {
        updateComplementaryFilter(dt);
    }
    else
    {
        updateMahonyFilter(dt);
    }
}

void MPU9250::sensorTask(void *arg)
{
    MPU9250 *sensor = static_cast<MPU9250 *>(arg);
    int64_t lastTime = esp_timer_get_time();

    // Magnetometer poll throttle.
    //
    // The AK8963 runs at 100 Hz internally (continuous mode 2 configured in
    // init()), i.e. one fresh sample every 10 ms. Polling its DRDY bit at
    // 1 kHz wastes ~390 us of I2C per iteration on stale data, which pushes
    // the per-loop budget above the 1 ms sample period and forces the task
    // down to ~870 Hz effective.
    //
    // We instead attempt one full mag read every 9 ms (slightly faster than
    // the producer so we never miss a sample). At 1 kHz iteration rate this
    // is ~1 attempt every 9 iterations; at the 100 Hz polling fallback,
    // it's roughly every iteration — both behave correctly without changes.
    //
    // Storing as a function-local (not a class member) so multiple MPU9250
    // instances each get their own counter, and we avoid any shared state.
    int64_t nextMagReadUs = 0; // 0 ensures the first iteration reads

#if MPU9250_PROFILER
    // Per-loop timing accumulators. Compiled out by default; enable via
    // -DMPU9250_PROFILER=1 in the consumer build flags when investigating
    // jitter or I2C budget on a new airframe.
    uint32_t loopCount     = 0;
    int64_t  totalI2cTime  = 0;
    int64_t  totalMathTime = 0;
    int64_t  maxI2cTime    = 0;
#endif

    ESP_LOGI(TAG_MPU9250, "Sensor task started");

    while (true)
    {
        // Calculate time delta
        int64_t now = esp_timer_get_time();
        float dt = (now - lastTime) / 1000000.0f;
        lastTime = now;

        // No division by 0
        if (dt <= 0.0f) dt = 0.01f;

        // Read sensor data — inline burst for the hot path (one I2C transaction
        // covers the 14 contiguous accel+temp+gyro bytes, vs. 7 separate ones
        // if we used readAccel/readGyro helpers).
        Vector3 loc_accel = {0.0f, 0.0f, 0.0f};
        Vector3 loc_gyro  = {0.0f, 0.0f, 0.0f};
        Vector3 loc_mag   = {0.0f, 0.0f, 0.0f};
        float loc_temp = 0.0f;
        bool newDataReady = false;
        // Tracks whether THIS iteration produced a fresh magnetometer sample.
        // The AK8963 runs at 100 Hz while the MPU is read at 1 kHz: only one
        // in ten iterations gets new mag data. Without this flag we used to
        // overwrite the shared `sensor->mag` with zeros on every non-fresh
        // iteration, which broke the Mahony 9-DOF correction completely.
        bool magNewSample = false;

#if MPU9250_PROFILER
        int64_t startI2c = esp_timer_get_time();
#endif

        // (14 octets : 6 Accel, 2 Temp, 6 Gyro)
        // Registre de départ : 0x3B (ACCEL_XOUT_H)
        uint8_t buffer[14];
        if (sensor->readRegisters(sensor->mpuDev, 0x3B, 14, buffer) == ESP_OK)
        {
            int16_t ax = (buffer[0] << 8) | buffer[1];
            int16_t ay = (buffer[2] << 8) | buffer[3];
            int16_t az = (buffer[4] << 8) | buffer[5];
            int16_t temp = (buffer[6] << 8) | buffer[7];
            int16_t gx = (buffer[8] << 8) | buffer[9];
            int16_t gy = (buffer[10] << 8) | buffer[11];
            int16_t gz = (buffer[12] << 8) | buffer[13];

            // Applied offsets and invert
            loc_accel.x = sensor->invertAxis.x * ((ax / 8192.0f) - sensor->accelOffset.x);
            loc_accel.y = sensor->invertAxis.y * ((ay / 8192.0f) - sensor->accelOffset.y);
            loc_accel.z = sensor->invertAxis.z * ((az / 8192.0f) - sensor->accelOffset.z);

            loc_temp = (temp / 333.87f) + 21.0f;

            // Temperature-compensated gyro offset: shift the static bias by
            // the per-axis linear coefficient times the temperature delta vs
            // the calibration point. coeff is {0,0,0} by default so this is
            // a no-op until the user calls setGyroTempCompCoeff().
            const float tempDelta = loc_temp - sensor->gyroCalibTemp;
            const float gxOff = sensor->gyroOffset.x + sensor->gyroTempCompCoeff.x * tempDelta;
            const float gyOff = sensor->gyroOffset.y + sensor->gyroTempCompCoeff.y * tempDelta;
            const float gzOff = sensor->gyroOffset.z + sensor->gyroTempCompCoeff.z * tempDelta;

            loc_gyro.x = sensor->invertAxis.x * ((gx / 32.8f) - gxOff);
            loc_gyro.y = sensor->invertAxis.y * ((gy / 32.8f) - gyOff);
            loc_gyro.z = sensor->invertAxis.z * ((gz / 32.8f) - gzOff);

            // Gyro notch filter (motor-vibration suppression).
            // Disabled by default — no-op until setGyroNotch() is called.
            // We snapshot the atomic flag once per iteration, then apply a
            // Direct Form II Transposed biquad per axis. State updates here
            // are exclusive to this task, so no locking is needed.
            if (sensor->gyroNotchEnabled.load(std::memory_order_acquire))
            {
                const NotchCoeffs c = sensor->gyroNotchCoeffs; // local copy
                float in, out;

                in  = loc_gyro.x;
                out = c.b0 * in + sensor->gyroNotchState[0].z1;
                sensor->gyroNotchState[0].z1 = c.b1 * in - c.a1 * out + sensor->gyroNotchState[0].z2;
                sensor->gyroNotchState[0].z2 = c.b2 * in - c.a2 * out;
                loc_gyro.x = out;

                in  = loc_gyro.y;
                out = c.b0 * in + sensor->gyroNotchState[1].z1;
                sensor->gyroNotchState[1].z1 = c.b1 * in - c.a1 * out + sensor->gyroNotchState[1].z2;
                sensor->gyroNotchState[1].z2 = c.b2 * in - c.a2 * out;
                loc_gyro.y = out;

                in  = loc_gyro.z;
                out = c.b0 * in + sensor->gyroNotchState[2].z1;
                sensor->gyroNotchState[2].z1 = c.b1 * in - c.a1 * out + sensor->gyroNotchState[2].z2;
                sensor->gyroNotchState[2].z2 = c.b2 * in - c.a2 * out;
                loc_gyro.z = out;
            }

            newDataReady = true;
        }

        // Magnetometer — throttled to ~111 Hz to match the AK8963 internal
        // sample rate (100 Hz). Without this throttle we burned ~390 us of
        // I2C every iteration polling for stale data, blowing the 1 ms
        // budget needed to sustain the 1 kHz INT-driven loop. See the
        // declaration of `nextMagReadUs` above the loop for the rationale.
        if (sensor->magAvailable && now >= nextMagReadUs)
        {
            // Schedule the next attempt before doing the I2C so a one-off
            // bus error doesn't cause back-to-back retries on the next iter.
            nextMagReadUs = now + 9000;

            uint8_t st1;
            // If data ready
            if (sensor->readRegisters(sensor->magDev, 0x02, 1, &st1) == ESP_OK && (st1 & 0x01))
            {
                uint8_t magBuf[7];
                // 7 octets : 6 pour XYZ + 1 octet final state (ST2)
                if (sensor->readRegisters(sensor->magDev, 0x03, 7, magBuf) == ESP_OK)
                {
                    // Check overflow mag
                    if (!(magBuf[6] & 0x08))
                    {
                        int16_t mx = (magBuf[1] << 8) | magBuf[0];
                        int16_t my = (magBuf[3] << 8) | magBuf[2];
                        int16_t mz = (magBuf[5] << 8) | magBuf[4];

                        float adjX = (((float)sensor->magAdjustValues[0] - 128.0f) / 256.0f + 1.0f);
                        float adjY = (((float)sensor->magAdjustValues[1] - 128.0f) / 256.0f + 1.0f);
                        float adjZ = (((float)sensor->magAdjustValues[2] - 128.0f) / 256.0f + 1.0f);

                        loc_mag.x = sensor->invertAxis.x * (((mx * 0.15f * adjX) - sensor->magOffset.x) * sensor->magScale.x);
                        loc_mag.y = sensor->invertAxis.y * (((my * 0.15f * adjY) - sensor->magOffset.y) * sensor->magScale.y);
                        loc_mag.z = sensor->invertAxis.z * (((mz * 0.15f * adjZ) - sensor->magOffset.z) * sensor->magScale.z);
                        magNewSample = true;
                    }
                }
            }
        }

#if MPU9250_PROFILER
        int64_t endI2c = esp_timer_get_time();
        int64_t currentI2cTime = endI2c - startI2c;
        totalI2cTime += currentI2cTime;
        if (currentI2cTime > maxI2cTime) maxI2cTime = currentI2cTime;

        int64_t startMath = esp_timer_get_time();
#endif

        if (newDataReady)
        {
            if (xSemaphoreTake(sensor->dataMutex, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                sensor->accel = loc_accel;
                sensor->gyro = loc_gyro;
                sensor->temperature = loc_temp;

                // Only update the shared mag when this iteration produced a
                // fresh sample; otherwise keep the last good value so the
                // Mahony filter does not see a (0,0,0) field 9/10 of the time.
                if (sensor->magAvailable && magNewSample) {
                    sensor->mag = loc_mag;
                }

                // Process measurements if not calibrating
                if (sensor->calibStatus != CALIBRATING)
                {
                    sensor->processMeasurements(dt);
                }

                xSemaphoreGive(sensor->dataMutex);
            }

            // -----------------------------------------------------------------
            // Atomic publish via seqlock + new-sample signal.
            //
            // We do this OUTSIDE the dataMutex on purpose:
            //   - The reader path (getSnapshot) never takes dataMutex; it
            //     loops on bundleSeq instead, so mutex contention against
            //     the reader is zero.
            //   - The seqlock writer protocol guarantees readers either
            //     copy the previous coherent bundle or the new one — never
            //     a half-written mix.
            //   - The semaphore give is a single FreeRTOS call (~few us);
            //     no need to hold any lock around it.
            // -----------------------------------------------------------------
            // Step 1: bump seq to odd -> readers know "writer in progress"
            uint32_t s = sensor->bundleSeq.load(std::memory_order_relaxed);
            sensor->bundleSeq.store(s + 1, std::memory_order_release);

            // Step 2: write the bundle (small, takes well under 1 us)
            sensor->publishedBundle.accel       = sensor->accel;
            sensor->publishedBundle.gyro        = sensor->gyro;
            sensor->publishedBundle.mag         = sensor->mag;
            sensor->publishedBundle.temperature = sensor->temperature;
            sensor->publishedBundle.timestampUs = (uint64_t)now;

            if (sensor->homeIsSet)
            {
                // q_publié = q_brut * q_offset
                Quaternion qPub = quatMul(sensor->q, sensor->qHomeInverse);
                sensor->publishedBundle.quaternion  = qPub;
                sensor->publishedBundle.orientation = quatToEuler(qPub, sensor->switchRollPitch);
            }
            else
            {
                // Pas d'offset, on publie les valeurs brutes
                sensor->publishedBundle.quaternion  = sensor->q;
                sensor->publishedBundle.orientation = sensor->orientation;
            }

            // Step 3: bump seq to even -> readers know the bundle is coherent
            sensor->bundleSeq.store(s + 2, std::memory_order_release);

            // Step 4: wake any task blocked in waitForNewSample(). Binary
            // sem: if the consumer hasn't taken yet, this is a no-op.
            xSemaphoreGive(sensor->sampleSem);
        }

#if MPU9250_PROFILER
        totalMathTime += (esp_timer_get_time() - startMath);

        loopCount++;
        // Logs once per `kProfilerWindow` loops. At 1 kHz INT-driven that
        // is roughly every 0.5 s; at ~100 Hz polling fallback every 5 s.
        constexpr uint32_t kProfilerWindow = 500;
        if (loopCount >= kProfilerWindow)
        {
            ESP_LOGI("PROFILER", "--- Profiling sur %lu boucles ---", (unsigned long)kProfilerWindow);
            ESP_LOGI("PROFILER", "I2C  (Moy)  : %lld us", totalI2cTime / kProfilerWindow);
            ESP_LOGI("PROFILER", "I2C  (Max)  : %lld us", maxI2cTime);
            ESP_LOGI("PROFILER", "Math (Moy)  : %lld us", totalMathTime / kProfilerWindow);

            loopCount = 0;
            totalI2cTime = 0;
            totalMathTime = 0;
            maxI2cTime = 0;
        }
#endif

        // -------------------------------------------------------------------
        // Wait for the next sample.
        //
        // INT-driven mode (intPin set):
        //   xTaskNotifyWait blocks until the GPIO ISR fires
        //   vTaskNotifyGiveFromISR. A 20 ms timeout acts as a watchdog: if
        //   the interrupt stops (cable yanked, sensor hung), the loop still
        //   runs at 50 Hz minimum so health monitoring keeps working.
        //
        // Polling mode (intPin == GPIO_NUM_NC):
        //   Legacy ~100 Hz behaviour, kept so the lib still works on boards
        //   where the INT line is not wired.
        // -------------------------------------------------------------------
        if (sensor->intPin != GPIO_NUM_NC)
        {
            // ulTaskNotifyTake clears the notification count on exit so each
            // ISR pulse corresponds to one loop iteration (no backlog).
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz polling fallback
        }
    }
}

void IRAM_ATTR MPU9250::dataReadyIsr(void *arg)
{
    // Wakes the sensor task on every MPU9250 sample. Kept extremely small
    // (a single direct-to-task notification) to minimise ISR latency at
    // 1 kHz. Any work — I2C reads, math — must happen in the task context.
    MPU9250 *sensor = static_cast<MPU9250 *>(arg);
    if (sensor->taskHandle == nullptr)
        return;

    BaseType_t higherWoken = pdFALSE;
    vTaskNotifyGiveFromISR(sensor->taskHandle, &higherWoken);
    if (higherWoken == pdTRUE)
        portYIELD_FROM_ISR();
}

esp_err_t MPU9250::startSensorTask()
{
    // xTaskCreatePinnedToCore accepts `tskNO_AFFINITY` as the core ID,
    // which is equivalent to the legacy `xTaskCreate` behaviour (the
    // scheduler may run the task on either core). Using the pinned API
    // unconditionally lets us cover both "no pinning" and "pinned to a
    // specific core" with a single call path.
    BaseType_t ret = xTaskCreatePinnedToCore(
        sensorTask,
        "mpu9250_task",
        taskStackSize,
        this,
        taskPriority,
        &taskHandle,
        taskCoreId);

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG_MPU9250, "Failed to create sensor task (prio=%u, core=%d, stack=%u)",
                 (unsigned)taskPriority, (int)taskCoreId, (unsigned)taskStackSize);
        return ESP_FAIL;
    }

    if (taskCoreId == tskNO_AFFINITY)
        ESP_LOGI(TAG_MPU9250, "Sensor task created (prio=%u, stack=%u, core=any)",
                 (unsigned)taskPriority, (unsigned)taskStackSize);
    else
        ESP_LOGI(TAG_MPU9250, "Sensor task created (prio=%u, stack=%u, pinned to core %d)",
                 (unsigned)taskPriority, (unsigned)taskStackSize, (int)taskCoreId);

    // If the user provided an INT pin, wire it up to drive the sensor task.
    // The order matters:
    //   1. Task created first (so taskHandle is valid when the ISR fires).
    //   2. GPIO + ISR installed.
    //   3. MPU's INT_ENABLE bit set last, which is what actually starts
    //      generating pulses on the pin.
    if (intPin != GPIO_NUM_NC)
    {
        gpio_config_t io = {};
        io.intr_type    = GPIO_INTR_POSEDGE;            // active-high pulse (INT_PIN_CFG ACTL=0)
        io.pin_bit_mask = (1ULL << intPin);
        io.mode         = GPIO_MODE_INPUT;
        io.pull_down_en = GPIO_PULLDOWN_ENABLE;          // line idles low between pulses
        io.pull_up_en   = GPIO_PULLUP_DISABLE;
        esp_err_t err = gpio_config(&io);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_MPU9250, "gpio_config(intPin=%d) failed: %d", intPin, err);
            return err;
        }

        // The ISR service is per-app; ignore ESP_ERR_INVALID_STATE which
        // simply means another component already installed it.
        err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        {
            ESP_LOGE(TAG_MPU9250, "gpio_install_isr_service failed: %d", err);
            return err;
        }

        err = gpio_isr_handler_add(intPin, &MPU9250::dataReadyIsr, this);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_MPU9250, "gpio_isr_handler_add failed: %d", err);
            return err;
        }

        // Enable RAW_DATA_RDY interrupt source on the MPU. After this write
        // the INT pin will start pulsing at the sample rate (1 kHz with the
        // SMPLRT_DIV/DLPF settings from init()).
        err = writeRegister(mpuDev, 0x38, 0x01); // INT_ENABLE = RAW_DATA_RDY_EN
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_MPU9250, "Failed to enable DATA_READY interrupt: %d", err);
            return err;
        }

        ESP_LOGI(TAG_MPU9250, "DATA_READY interrupt enabled on GPIO %d", intPin);
    }
    else
    {
        ESP_LOGW(TAG_MPU9250, "No INT pin configured — falling back to polling (~100 Hz)");
    }

    // Auto-trigger the Mahony fast-init boost so the filter converges from
    // identity to the actual attitude in well under 1 s. No-op if
    // mahonyBoostDurationMs == 0 (boost disabled).
    triggerMahonyBoost();

    return ESP_OK;
}

esp_err_t MPU9250::calibrate()
{
    // IMUSensor interface contract: this is the default "safe" calibration
    // every drone bring-up needs. It only handles gyro+accel — the mag
    // calibration requires user rotation and is a separate explicit call.
    return calibrateGyroAccel();
}

esp_err_t MPU9250::calibrateGyroAccel()
{
    if (calibStatus == CALIBRATING)
    {
        ESP_LOGW(TAG_MPU9250, "Calibration already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    resetGyroAccelOffsets();
    calibStatus = CALIBRATING;

    // Background task — performGyroAccelCalibration takes ~10 s and we do
    // not want to block the caller.
    TaskHandle_t calibTaskHandle = NULL;
    BaseType_t ret = xTaskCreate(
        [](void *arg)
        {
            MPU9250 *sensor = static_cast<MPU9250 *>(arg);
            sensor->performGyroAccelCalibration();
            vTaskDelete(NULL);
        },
        "mpu9250_ga_cal",
        4096,
        this,
        5,
        &calibTaskHandle);

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG_MPU9250, "Failed to create gyro/accel calibration task");
        calibStatus = NOT_CALIBRATED;
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t MPU9250::calibrateMag()
{
    if (!magAvailable)
    {
        ESP_LOGW(TAG_MPU9250, "calibrateMag: no magnetometer detected at init");
        return ESP_ERR_NOT_SUPPORTED;
    }
    // calibStatus is intentionally NOT mutated here: the gyro+accel state
    // remains whatever it was. A mag-only calibration is an orthogonal
    // operation; consumers waiting on getCalibrationStatus() see no change.

    resetMagOffsets();

    TaskHandle_t calibTaskHandle = NULL;
    BaseType_t ret = xTaskCreate(
        [](void *arg)
        {
            MPU9250 *sensor = static_cast<MPU9250 *>(arg);
            sensor->performMagCalibration();
            vTaskDelete(NULL);
        },
        "mpu9250_mag_cal",
        4096,
        this,
        5,
        &calibTaskHandle);

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG_MPU9250, "Failed to create mag calibration task");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void MPU9250::resetGyroAccelOffsets()
{
    ESP_LOGI(TAG_MPU9250, "Resetting gyro/accel calibration values");
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        accelOffset   = {0, 0, 0};
        gyroOffset    = {0, 0, 0};
        gyroCalibTemp = 25.0f;
        // Note: gyroTempCompCoeff is preserved across re-runs (user-supplied,
        // expensive to remeasure).
        xSemaphoreGive(dataMutex);
    }
}

void MPU9250::resetMagOffsets()
{
    ESP_LOGI(TAG_MPU9250, "Resetting magnetometer calibration values");
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        magOffset = {0, 0, 0};
        magScale  = {1.0f, 1.0f, 1.0f};
        xSemaphoreGive(dataMutex);
    }
}

// -----------------------------------------------------------------------------
// performGyroAccelCalibration — sensor IMMOBILE for 10 seconds.
// -----------------------------------------------------------------------------
void MPU9250::performGyroAccelCalibration()
{
    ESP_LOGI(TAG_MPU9250, "Gyro/Accel calibration: keep the sensor still on a flat surface...");
    vTaskDelay(pdMS_TO_TICKS(1000)); // Let sensor settle thermally

    Vector3 accelSum = {0, 0, 0};
    Vector3 gyroSum  = {0, 0, 0};

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        accelSum.x += invertAxis.x * readAccel(0);
        accelSum.y += invertAxis.y * readAccel(2);
        accelSum.z += invertAxis.z * readAccel(4);
        gyroSum.x  += invertAxis.x * readGyro(0);
        gyroSum.y  += invertAxis.y * readGyro(2);
        gyroSum.z  += invertAxis.z * readGyro(4);

        if (i % 100 == 0)
        {
            ESP_LOGI(TAG_MPU9250, "Gyro/Accel calibration progress: %d%%", i * 100 / CALIBRATION_SAMPLES);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Capture temperature once at the end (chip thermally stable by now).
    float calibTemp = 25.0f;
    {
        uint8_t rawData[2] = {0};
        if (readRegisters(mpuDev, 0x41, 2, rawData) == ESP_OK)
        {
            int16_t tempRaw = (((int16_t)rawData[0]) << 8) | rawData[1];
            calibTemp = (float)tempRaw / 333.87f + 21.0f;
        }
    }

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        gyroOffset.x  = gyroSum.x / CALIBRATION_SAMPLES;
        gyroOffset.y  = gyroSum.y / CALIBRATION_SAMPLES;
        gyroOffset.z  = gyroSum.z / CALIBRATION_SAMPLES;
        gyroCalibTemp = calibTemp;
        // gyroTempCompCoeff preserved (user-supplied via setGyroTempCompCoeff).

        accelOffset.x = accelSum.x / CALIBRATION_SAMPLES;
        accelOffset.y = accelSum.y / CALIBRATION_SAMPLES;
        accelOffset.z = (accelSum.z / CALIBRATION_SAMPLES) - 1.0f; // subtract 1g gravity

        // Reset filter integrators so they don't carry pre-calibration error
        gyroIntegrated      = {0, 0, 0};
        mahonyIntegralError = {0, 0, 0};

        // calibStatus advances to CALIBRATED here even if mag has never been
        // calibrated: gyro+accel is the minimum to safely produce attitude.
        // The mag stays at its previous offsets/scales (zero/unity if never
        // calibrated), in which case the Mahony 9-DOF falls back to 6-DOF
        // naturally (zero mag vector skipped, see updateMahonyFilter).
        calibStatus = CALIBRATED;

        xSemaphoreGive(dataMutex);
    }

    ESP_LOGI(TAG_MPU9250, "Gyro/Accel calibration complete");
    ESP_LOGI(TAG_MPU9250, "  Accel offsets: %.3f, %.3f, %.3f", accelOffset.x, accelOffset.y, accelOffset.z);
    ESP_LOGI(TAG_MPU9250, "  Gyro offsets:  %.3f, %.3f, %.3f (calibTemp=%.1f degC)",
             gyroOffset.x, gyroOffset.y, gyroOffset.z, gyroCalibTemp);

    // Re-arm the Mahony fast-init boost: the new gyro/accel offsets have
    // shifted the filter's expected gravity reference, so a brief high-Kp
    // window helps the orientation estimate re-converge without lag.
    triggerMahonyBoost();

    esp_err_t saveErr = saveCalibration();
    if (saveErr != ESP_OK)
        ESP_LOGW(TAG_MPU9250, "saveCalibration failed (err=%d) — RAM-only", saveErr);
}

// -----------------------------------------------------------------------------
// performMagCalibration — sensor ROTATED in figure-8 for 30 seconds.
// -----------------------------------------------------------------------------
void MPU9250::performMagCalibration()
{
    if (!magAvailable)
    {
        ESP_LOGW(TAG_MPU9250, "Mag calibration requested but no magnetometer detected — skipping");
        return;
    }

    ESP_LOGI(TAG_MPU9250, "Mag calibration: rotate the sensor through ALL orientations in a figure-8");
    ESP_LOGI(TAG_MPU9250, "  (slow, sustained motion for ~30 seconds — cover roll, pitch and yaw)");
    vTaskDelay(pdMS_TO_TICKS(2000)); // give the user time to start moving

    Vector3 magMin = { FLT_MAX,  FLT_MAX,  FLT_MAX};
    Vector3 magMax = {-FLT_MAX, -FLT_MAX, -FLT_MAX};

    for (int i = 0; i < MAG_CALIBRATION_SAMPLES; i++)
    {
        float mx = invertAxis.x * readMag(0);
        float my = invertAxis.y * readMag(2);
        float mz = invertAxis.z * readMag(4);

        // Skip the (0,0,0) sentinel from readMag when DATA_READY is not set
        if (mx != 0.0f || my != 0.0f || mz != 0.0f)
        {
            magMin.x = min(magMin.x, mx);
            magMin.y = min(magMin.y, my);
            magMin.z = min(magMin.z, mz);

            magMax.x = max(magMax.x, mx);
            magMax.y = max(magMax.y, my);
            magMax.z = max(magMax.z, mz);
        }

        if (i % 300 == 0)
        {
            ESP_LOGI(TAG_MPU9250, "Mag calibration progress: %d%%", i * 100 / MAG_CALIBRATION_SAMPLES);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Sanity: if min == max on any axis the user did not actually rotate.
    // Refuse to apply a degenerate calibration that would zero out a scale.
    const float spanX = magMax.x - magMin.x;
    const float spanY = magMax.y - magMin.y;
    const float spanZ = magMax.z - magMin.z;
    if (spanX <= 0.0f || spanY <= 0.0f || spanZ <= 0.0f)
    {
        ESP_LOGE(TAG_MPU9250, "Mag calibration failed: zero span on at least one axis (rotate harder next time)");
        return;
    }

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        // Hard-iron offset = midpoint of each axis (DC bias from nearby ferro).
        magOffset.x = (magMax.x + magMin.x) / 2.0f;
        magOffset.y = (magMax.y + magMin.y) / 2.0f;
        magOffset.z = (magMax.z + magMin.z) / 2.0f;

        // Soft-iron scale = per-axis span normalized to the widest axis.
        // The widest axis becomes 1.0, narrower axes get >1.0 to stretch
        // their range and approximate a spherical field response.
        const float magDelta = max(max(spanX, spanY), spanZ);
        magScale.x = spanX / magDelta;
        magScale.y = spanY / magDelta;
        magScale.z = spanZ / magDelta;
        xSemaphoreGive(dataMutex);
    }

    ESP_LOGI(TAG_MPU9250, "Mag calibration complete");
    ESP_LOGI(TAG_MPU9250, "  Mag offsets: %.3f, %.3f, %.3f", magOffset.x, magOffset.y, magOffset.z);
    ESP_LOGI(TAG_MPU9250, "  Mag scale:   %.3f, %.3f, %.3f", magScale.x, magScale.y, magScale.z);

    esp_err_t saveErr = saveCalibration();
    if (saveErr != ESP_OK)
        ESP_LOGW(TAG_MPU9250, "saveCalibration failed (err=%d) — RAM-only", saveErr);
}

// Individual getters now go through the lock-free snapshot path so the
// consumer never blocks on dataMutex (which the sensor task holds during
// the I2C burst + math, up to ~800 us). The trade-off is a small constant
// cost per call: each one copies the whole bundle even though the consumer
// only needs one field. If the consumer reads several fields, prefer
// `getSnapshot()` directly to do a single seqlock read.

MPU9250::Orientation MPU9250::getOrientation()
{
    return getSnapshot().orientation;
}

MPU9250::Vector3 MPU9250::getAccel()
{
    return getSnapshot().accel;
}

MPU9250::Vector3 MPU9250::getGyro()
{
    return getSnapshot().gyro;
}

MPU9250::Vector3 MPU9250::getMag()
{
    return getSnapshot().mag;
}

MPU9250::Quaternion MPU9250::getQuaternion()
{
    // Returned in body-to-world convention. Always a unit quaternion as long
    // as the Mahony path has run at least once since boot; otherwise it is
    // the identity {1,0,0,0} initialised by the constructor.
    return getSnapshot().quaternion;
}

esp_err_t MPU9250::setMahonyGains(float kp, float ki)
{
    if (kp < 0.0f || ki < 0.0f)
    {
        ESP_LOGE(TAG_MPU9250, "setMahonyGains: gains must be non-negative");
        return ESP_ERR_INVALID_ARG;
    }
    // No mutex: these are written atomically on ESP32 (4-byte aligned float
    // store) and read by the sensor task on the next iteration. Worst case
    // is one extra iteration with mixed gains, which is harmless.
    mahonyKp = kp;
    mahonyKi = ki;
    // Reset integral state when gains change to avoid a sudden kick from the
    // accumulated error multiplied by the new Ki.
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        mahonyIntegralError = {0.0f, 0.0f, 0.0f};
        xSemaphoreGive(dataMutex);
    }
    return ESP_OK;
}

esp_err_t MPU9250::setMahonyBoost(float boostKp, uint32_t durationMs)
{
    if (boostKp < 0.0f)
    {
        ESP_LOGE(TAG_MPU9250, "setMahonyBoost: boostKp must be non-negative");
        return ESP_ERR_INVALID_ARG;
    }
    // Clamp boostKp to >= nominal mahonyKp (boosting downward makes no sense).
    if (boostKp < mahonyKp)
        boostKp = mahonyKp;

    // 4-byte aligned float / uint32 stores are atomic on ESP32 — no mutex.
    mahonyBoostKp         = boostKp;
    mahonyBoostDurationMs = durationMs;

    ESP_LOGI(TAG_MPU9250, "Mahony boost configured: Kp=%.2f for %u ms",
             boostKp, (unsigned)durationMs);
    return ESP_OK;
}

esp_err_t MPU9250::triggerMahonyBoost()
{
    if (mahonyBoostDurationMs == 0)
    {
        // Boost disabled — silent no-op so callers (startSensorTask,
        // performGyroAccelCalibration) can always invoke without
        // conditioning on Config.
        return ESP_OK;
    }
    // Arm the boost window: effective Kp = mahonyBoostKp until this deadline.
    mahonyBoostUntilUs = esp_timer_get_time()
                       + (int64_t)mahonyBoostDurationMs * 1000;
    ESP_LOGI(TAG_MPU9250, "Mahony boost armed: Kp=%.2f for %u ms",
             mahonyBoostKp, (unsigned)mahonyBoostDurationMs);
    return ESP_OK;
}

float MPU9250::getTemperature()
{
    return getSnapshot().temperature;
}

bool MPU9250::isSensorHealthy()
{
    if (errorCount > 100 && successCount < errorCount)
    {
        return false;
    }
    return true;
}

esp_err_t MPU9250::setFilterMode(FilterMode mode)
{
    if (mode == COMPLEMENTARY || mode == MAHONY)
    {
        filterMode = mode;
    }
    else
    {
        ESP_LOGE(TAG_MPU9250, "Invalid filter mode");
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

esp_err_t MPU9250::setInvertAxis(bool invertX, bool invertY, bool invertZ)
{

    invertAxis.x = invertX ? -1 : 1;
    invertAxis.y = invertY ? -1 : 1;
    invertAxis.z = invertZ ? -1 : 1;
    return ESP_OK;
}

esp_err_t MPU9250::setSwitchRollPitch(bool switchRollPitch)
{
    this->switchRollPitch = switchRollPitch;
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// NVS-backed calibration persistence
// -----------------------------------------------------------------------------

esp_err_t MPU9250::saveCalibration()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(MPU9250_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_MPU9250, "saveCalibration: nvs_open failed (%d). Did the app call nvs_flash_init()?", err);
        return err;
    }

    CalibBlob blob = {};
    blob.version       = MPU9250_CALIB_VERSION;
    blob.magAvailable  = magAvailable ? 1 : 0;

    // Snapshot under the data mutex so we never persist a half-written set.
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        blob.accelOffset[0] = accelOffset.x;
        blob.accelOffset[1] = accelOffset.y;
        blob.accelOffset[2] = accelOffset.z;
        blob.gyroOffset[0]  = gyroOffset.x;
        blob.gyroOffset[1]  = gyroOffset.y;
        blob.gyroOffset[2]  = gyroOffset.z;
        blob.gyroCalibTemp  = gyroCalibTemp;
        blob.gyroTempCompCoeff[0] = gyroTempCompCoeff.x;
        blob.gyroTempCompCoeff[1] = gyroTempCompCoeff.y;
        blob.gyroTempCompCoeff[2] = gyroTempCompCoeff.z;
        blob.magOffset[0]   = magOffset.x;
        blob.magOffset[1]   = magOffset.y;
        blob.magOffset[2]   = magOffset.z;
        blob.magScale[0]    = magScale.x;
        blob.magScale[1]    = magScale.y;
        blob.magScale[2]    = magScale.z;
        blob.magAdjustValues[0] = magAdjustValues[0];
        blob.magAdjustValues[1] = magAdjustValues[1];
        blob.magAdjustValues[2] = magAdjustValues[2];
        // v2: home offset (mount tilt compensation, roll/pitch only).
        blob.homeIsSet = homeIsSet ? 1 : 0;
        blob.qHomeInverse[0] = qHomeInverse.w;
        blob.qHomeInverse[1] = qHomeInverse.x;
        blob.qHomeInverse[2] = qHomeInverse.y;
        blob.qHomeInverse[3] = qHomeInverse.z;
        xSemaphoreGive(dataMutex);
    }

    err = nvs_set_blob(handle, MPU9250_NVS_KEY, &blob, sizeof(blob));
    if (err == ESP_OK)
        err = nvs_commit(handle);
    nvs_close(handle);

    if (err != ESP_OK)
        ESP_LOGE(TAG_MPU9250, "saveCalibration: write failed (%d)", err);
    return err;
}

esp_err_t MPU9250::loadCalibration()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(MPU9250_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND)
        return ESP_ERR_NOT_FOUND; // namespace never created -> no calib
    if (err != ESP_OK)
        return err;

    CalibBlob blob = {};
    size_t length = sizeof(blob);
    err = nvs_get_blob(handle, MPU9250_NVS_KEY, &blob, &length);
    nvs_close(handle);

    if (err == ESP_ERR_NVS_NOT_FOUND)
        return ESP_ERR_NOT_FOUND;
    if (err != ESP_OK)
        return err;

    if (length != sizeof(blob))
    {
        ESP_LOGW(TAG_MPU9250, "loadCalibration: blob size mismatch (got %u, expected %u) — ignoring", (unsigned)length, (unsigned)sizeof(blob));
        return ESP_ERR_INVALID_SIZE;
    }
    if (blob.version != MPU9250_CALIB_VERSION)
    {
        ESP_LOGW(TAG_MPU9250, "loadCalibration: stored version %u != %u — ignoring stale blob", blob.version, MPU9250_CALIB_VERSION);
        return ESP_ERR_INVALID_VERSION;
    }

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        accelOffset       = {blob.accelOffset[0], blob.accelOffset[1], blob.accelOffset[2]};
        gyroOffset        = {blob.gyroOffset[0],  blob.gyroOffset[1],  blob.gyroOffset[2]};
        gyroCalibTemp     = blob.gyroCalibTemp;
        gyroTempCompCoeff = {blob.gyroTempCompCoeff[0], blob.gyroTempCompCoeff[1], blob.gyroTempCompCoeff[2]};
        magOffset         = {blob.magOffset[0],   blob.magOffset[1],   blob.magOffset[2]};
        magScale          = {blob.magScale[0],    blob.magScale[1],    blob.magScale[2]};
        // Note: magAdjustValues are read from the AK8963 fuse ROM in init(),
        // so they will be re-loaded from hardware regardless of what we
        // stored. The blob value is kept for diagnostic purposes only.
        // v2: home offset (mount tilt compensation). If the blob has a
        // home set, apply it; otherwise stay at identity (no compensation).
        homeIsSet    = (blob.homeIsSet != 0);
        qHomeInverse = { blob.qHomeInverse[0], blob.qHomeInverse[1],
                         blob.qHomeInverse[2], blob.qHomeInverse[3] };
        // Safety: if the stored quaternion is garbage (e.g. all zeros from
        // a botched first save), fall back to identity rather than wreck
        // the orientation output.
        const float qn = qHomeInverse.w * qHomeInverse.w + qHomeInverse.x * qHomeInverse.x
                       + qHomeInverse.y * qHomeInverse.y + qHomeInverse.z * qHomeInverse.z;
        if (qn < 0.5f || qn > 1.5f) {
            ESP_LOGW(TAG_MPU9250, "loadCalibration: invalid qHomeInverse (norm^2=%.3f), falling back to identity", qn);
            qHomeInverse = { 1.0f, 0.0f, 0.0f, 0.0f };
            homeIsSet    = false;
        }
        calibStatus = CALIBRATED;
        xSemaphoreGive(dataMutex);
    }

    return ESP_OK;
}

esp_err_t MPU9250::clearStoredCalibration()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(MPU9250_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return err;
    err = nvs_erase_key(handle, MPU9250_NVS_KEY);
    if (err == ESP_OK)
        err = nvs_commit(handle);
    nvs_close(handle);
    return (err == ESP_ERR_NVS_NOT_FOUND) ? ESP_OK : err;
}

esp_err_t MPU9250::setGyroTempCompCoeff(Vector3 coeff)
{
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        gyroTempCompCoeff = coeff;
        xSemaphoreGive(dataMutex);
    }
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Lock-free snapshot + sample signalling
// -----------------------------------------------------------------------------

MPU9250::SampleBundle MPU9250::getSnapshot()
{
    // Seqlock read protocol:
    //   1. Load seq. If odd, writer is mid-publish: retry.
    //   2. Copy the bundle.
    //   3. Load seq again. If it changed, our copy was sliced: retry.
    //
    // Worst-case bound: writer cycle is 1 ms and writes the bundle in <1 us,
    // so the retry window per iteration is tiny. taskYIELD() between
    // attempts so we don't starve the writer if it happens to be on the
    // same core at a lower priority.
    SampleBundle local;
    uint32_t s1, s2;
    do
    {
        s1 = bundleSeq.load(std::memory_order_acquire);
        if (s1 & 1u)
        {
            // Writer in progress — yield and retry.
            taskYIELD();
            s2 = s1 - 1; // FIX: Initialize s2 to force a mismatch and retry
            continue;
        }
        local = publishedBundle;
        s2 = bundleSeq.load(std::memory_order_acquire);
    } while (s1 != s2);
    return local;
}

esp_err_t MPU9250::waitForNewSample(uint32_t timeoutMs)
{
    if (sampleSem == nullptr)
        return ESP_ERR_INVALID_STATE;

    // Binary semaphore: returns immediately if a sample has been published
    // since the last take (or this is the first call), otherwise blocks up
    // to timeoutMs. xSemaphoreTake -> pdTRUE on signal, pdFALSE on timeout.
    const TickType_t ticks = (timeoutMs == 0)
                                 ? portMAX_DELAY
                                 : pdMS_TO_TICKS(timeoutMs);
    return (xSemaphoreTake(sampleSem, ticks) == pdTRUE) ? ESP_OK : ESP_ERR_TIMEOUT;
}

// -----------------------------------------------------------------------------
// Gyro notch biquad — coefficient computation
// -----------------------------------------------------------------------------
// Standard notch from Robert Bristow-Johnson's "Audio EQ Cookbook":
//
//   w0    = 2*pi*f0/fs
//   alpha = sin(w0) / (2*Q)         with Q = f0 / bandwidth
//
//   b0 = 1
//   b1 = -2*cos(w0)
//   b2 = 1
//   a0 = 1 + alpha
//   a1 = -2*cos(w0)
//   a2 = 1 - alpha
//
// Then everything divided by a0 so a0 = 1 in the runtime form (DF-II-T).
// -----------------------------------------------------------------------------

esp_err_t MPU9250::setGyroNotch(float centerHz, float bandwidthHz, float sampleRateHz)
{
    if (centerHz <= 0.0f || bandwidthHz <= 0.0f || sampleRateHz <= 0.0f)
    {
        ESP_LOGE(TAG_MPU9250, "setGyroNotch: parameters must be positive");
        return ESP_ERR_INVALID_ARG;
    }
    // Nyquist sanity: a notch above fs/2 is meaningless. We clamp slightly
    // below half-rate so the digital prewarp is well-conditioned.
    if (centerHz >= 0.49f * sampleRateHz)
    {
        ESP_LOGE(TAG_MPU9250, "setGyroNotch: centerHz (%.1f) too close to Nyquist (%.1f)", centerHz, 0.5f * sampleRateHz);
        return ESP_ERR_INVALID_ARG;
    }

    const float w0       = 2.0f * (float)M_PI * centerHz / sampleRateHz;
    const float cosW0    = cosf(w0);
    const float sinW0    = sinf(w0);
    const float Q        = centerHz / bandwidthHz;
    const float alpha    = sinW0 / (2.0f * Q);

    const float a0       = 1.0f + alpha;
    const float invA0    = 1.0f / a0;

    NotchCoeffs c;
    c.b0 = 1.0f          * invA0;
    c.b1 = (-2.0f * cosW0) * invA0;
    c.b2 = 1.0f          * invA0;
    c.a1 = (-2.0f * cosW0) * invA0;
    c.a2 = (1.0f - alpha) * invA0;

    // Install under the data mutex so the sensor task does not splice a new
    // numerator with the old denominator in a half-applied update. Reset the
    // per-axis state to avoid a transient pop when the response changes.
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        gyroNotchCoeffs = c;
        memset(gyroNotchState, 0, sizeof(gyroNotchState));
        xSemaphoreGive(dataMutex);
    }
    gyroNotchEnabled.store(true, std::memory_order_release);

    ESP_LOGI(TAG_MPU9250, "Gyro notch enabled: f0=%.1f Hz, BW=%.1f Hz, fs=%.1f Hz (Q=%.2f)",
             centerHz, bandwidthHz, sampleRateHz, Q);
    return ESP_OK;
}

esp_err_t MPU9250::clearGyroNotch()
{
    gyroNotchEnabled.store(false, std::memory_order_release);
    // Reset state so a future re-enable starts from a clean slate.
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        memset(gyroNotchState, 0, sizeof(gyroNotchState));
        xSemaphoreGive(dataMutex);
    }
    ESP_LOGI(TAG_MPU9250, "Gyro notch disabled");
    return ESP_OK;
}

esp_err_t MPU9250::setHome()
{
    // Le setHome dépend du quaternion absolu maintenu par le filtre de Mahony
    if (filterMode != MAHONY)
    {
        ESP_LOGE(TAG_MPU9250, "setHome requires MAHONY filter mode to be active");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        // 1. Récupération du cap (yaw) actuel en radians
        float yawRad = orientation.yaw * DEG_TO_RAD;

        // 2. Création d'un quaternion de lacet pur représentant l'attitude "à plat" idéale
        float halfYaw = yawRad * 0.5f;
        Quaternion qYaw = { cosf(halfYaw), 0.0f, 0.0f, sinf(halfYaw) };

        // 3. Calcul de l'offset : qHomeInverse = (q_brut)* * q_yaw
        qHomeInverse = quatMul(quatConj(q), qYaw);
        homeIsSet = true;

        xSemaphoreGive(dataMutex);

        ESP_LOGI(TAG_MPU9250, "Home set successfully. Yaw preserved at %.1f deg", orientation.yaw);
        
        // Sauvegarde automatique en NVS
        return saveCalibration();
    }
    
    return ESP_ERR_TIMEOUT;
}

esp_err_t MPU9250::clearHome()
{
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        qHomeInverse = {1.0f, 0.0f, 0.0f, 0.0f}; // Identité
        homeIsSet = false;
        xSemaphoreGive(dataMutex);

        ESP_LOGI(TAG_MPU9250, "Home offset cleared");
        
        // Sauvegarde automatique en NVS
        return saveCalibration();
    }
    return ESP_ERR_TIMEOUT;
}

MPU9250::Quaternion MPU9250::getAbsoluteQuaternion()
{
    SampleBundle bundle = getSnapshot();
    
    // Si un home est défini, le bundle contient le quaternion corrigé (q_pub).
    // Pour retrouver l'original : q_brut = q_pub * (qHomeInverse)*
    if (homeIsSet)
    {
        return quatMul(bundle.quaternion, quatConj(qHomeInverse));
    }
    
    return bundle.quaternion;
}

MPU9250::Orientation MPU9250::getAbsoluteOrientation()
{
    // On réutilise la conversion d'Euler sur le quaternion absolu
    return quatToEuler(getAbsoluteQuaternion(), switchRollPitch);
}