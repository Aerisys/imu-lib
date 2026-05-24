#include "mpu9250.h"

// I2C transaction timeout in ms used for register reads/writes.
// Generous enough for clock stretching on slow buses; tight enough that
// the sensor task does not hang on a wire fault.
#define MPU9250_I2C_TIMEOUT_MS 100

// Implementation
MPU9250::MPU9250()
    : busHandle(nullptr),
      mpuDev(nullptr),
      magDev(nullptr),
      intPin(GPIO_NUM_NC),
      taskHandle(nullptr),
      dataMutex(nullptr),
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
      magOffset{0, 0, 0},
      magScale{1.0f, 1.0f, 1.0f},
      calibStatus(NOT_CALIBRATED),
      errorCount(0),
      successCount(0),
      magAvailable(false),
      filterMode(COMPLEMENTARY),
      q{1.0f, 0.0f, 0.0f, 0.0f} // Mahony quaternion init
{
    memset(magAdjustValues, 0, sizeof(magAdjustValues));
    dataMutex = xSemaphoreCreateMutex();
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
}

esp_err_t MPU9250::init(i2c_master_bus_handle_t bus, const Config& config)
{
    if (bus == nullptr)
    {
        ESP_LOGE(TAG_MPU9250, "init: bus handle is null");
        return ESP_ERR_INVALID_ARG;
    }

    busHandle = bus;
    intPin = config.intPin;

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

    // Configure interrupt pin: BYPASS_EN=1 so the AK8963 is reachable as a
    // separate device on the same I2C bus (not through the MPU's I2C master).
    err = writeRegister(mpuDev, MPU9250_INT_PIN_CFG, 0x22);
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

void MPU9250::readAllSensors()
{
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        // Read accelerometer
        accel.x = invertAxis.x * (readAccel(0) - accelOffset.x);
        accel.y = invertAxis.y * (readAccel(2) - accelOffset.y);
        accel.z = invertAxis.z * (readAccel(4) - accelOffset.z);

        // Read gyroscope
        gyro.x = invertAxis.x * (readGyro(0) - gyroOffset.x);
        gyro.y = invertAxis.y * (readGyro(2) - gyroOffset.y);
        gyro.z = invertAxis.z * (readGyro(4) - gyroOffset.z);

        // Read magnetometer if available
        if (magAvailable)
        {
            mag.x = invertAxis.x * ((readMag(0) - magOffset.x) * magScale.x);
            mag.y = invertAxis.y * ((readMag(2) - magOffset.y) * magScale.y);
            mag.z = invertAxis.z * ((readMag(4) - magOffset.z) * magScale.z);
        }

        // Read temperature
        uint8_t rawData[2] = {0};
        if (readRegisters(mpuDev, 0x41, 2, rawData) == ESP_OK)
        {
            int16_t tempRaw = (((int16_t)rawData[0]) << 8) | rawData[1];
            temperature = (float)tempRaw / 333.87f + 21.0f;
        }

        xSemaphoreGive(dataMutex);
    }
}

void MPU9250::computeAnglesFromAccel()
{
    // Calculate roll/pitch from accelerometer
    float rollAcc = atan2f(accel.y, accel.z) * RAD_TO_DEG;
    float pitchAcc = atan2f(-accel.x, sqrtf(accel.y * accel.y + accel.z * accel.z)) * RAD_TO_DEG;

    // Store in temporary variables for filter update
    orientation.roll = rollAcc;
    orientation.pitch = pitchAcc;
}

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
    // Convert gyro to rad/s
    float gx = gyro.x * DEG_TO_RAD;
    float gy = gyro.y * DEG_TO_RAD;
    float gz = gyro.z * DEG_TO_RAD;

    // Unpack quaternion state
    float q0 = q.w;
    float q1 = q.x;
    float q2 = q.y;
    float q3 = q.z;

    // Normalize accelerometer measurement
    float norm = sqrtf(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z);
    if (norm == 0.0f) return; // avoid division by zero
    norm = 1.0f / norm;
    float ax = accel.x * norm;
    float ay = accel.y * norm;
    float az = accel.z * norm;

    // Estimated direction of gravity
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // Error is cross product between measured and estimated gravity
    float ex = (ay*vz - az*vy);
    float ey = (az*vx - ax*vz);
    float ez = (ax*vy - ay*vx);

    // Integral feedback
    if (MAHONY_KI > 0.0f) {
        mahonyIntegralError.x += ex * MAHONY_KI * dt;
        mahonyIntegralError.y += ey * MAHONY_KI * dt;
        mahonyIntegralError.z += ez * MAHONY_KI * dt;
        gx += mahonyIntegralError.x;
        gy += mahonyIntegralError.y;
        gz += mahonyIntegralError.z;
    }

    // Proportional feedback
    gx += MAHONY_KP * ex;
    gy += MAHONY_KP * ey;
    gz += MAHONY_KP * ez;

    // Integrate rate of change of quaternion
    float half_dt = 0.5f * dt;
    float qa = q0;
    float qb = q1;
    float qc = q2;
    float qd = q3;

    q0 += (-qb * gx - qc * gy - qd * gz) * half_dt;
    q1 += ( qa * gx + qc * gz - qd * gy) * half_dt;
    q2 += ( qa * gy - qb * gz + qd * gx) * half_dt;
    q3 += ( qa * gz + qb * gy - qc * gx) * half_dt;

    // Normalize quaternion
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    norm = 1.0f / norm;
    q.w = q0 * norm;
    q.x = q1 * norm;
    q.y = q2 * norm;
    q.z = q3 * norm;

    // Convert quaternion to Euler angles (deg)
    if (switchRollPitch)
    {
        orientation.pitch  = atan2f(2.0f*(q.w*q.x + q.y*q.z), 1.0f - 2.0f*(q.x*q.x + q.y*q.y)) * RAD_TO_DEG;
        orientation.roll = asinf( 2.0f*(q.w*q.y - q.z*q.x)) * RAD_TO_DEG;
    } else {
        orientation.roll  = atan2f(2.0f*(q.w*q.x + q.y*q.z), 1.0f - 2.0f*(q.x*q.x + q.y*q.y)) * RAD_TO_DEG;
        orientation.pitch = asinf( 2.0f*(q.w*q.y - q.z*q.x)) * RAD_TO_DEG;
    }
    
    orientation.yaw   = atan2f(2.0f*(q.w*q.z + q.x*q.y), 1.0f - 2.0f*(q.y*q.y + q.z*q.z)) * RAD_TO_DEG;

    // Keep yaw in [0,360)
    if (orientation.yaw < 0) orientation.yaw += 360.0f;
    else if (orientation.yaw >= 360.0f) orientation.yaw -= 360.0f;
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

    uint32_t loopCount = 0;
    int64_t totalI2cTime = 0;
    int64_t totalMathTime = 0;
    int64_t maxI2cTime = 0; // Pour capter le pire scénario (jitter)

    ESP_LOGI(TAG_MPU9250, "Sensor task started");

    while (true)
    {
        // Calculate time delta
        int64_t now = esp_timer_get_time();
        float dt = (now - lastTime) / 1000000.0f;
        lastTime = now;

        // No division by 0
        if (dt <= 0.0f) dt = 0.01f;

        // Read sensor data
        // sensor->readAllSensors();
        Vector3 loc_accel = {0.0f, 0.0f, 0.0f};
        Vector3 loc_gyro  = {0.0f, 0.0f, 0.0f};
        Vector3 loc_mag   = {0.0f, 0.0f, 0.0f};
        float loc_temp = 0.0f;
        bool newDataReady = false;

        int64_t startI2c = esp_timer_get_time();

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

            loc_gyro.x = sensor->invertAxis.x * ((gx / 32.8f) - sensor->gyroOffset.x);
            loc_gyro.y = sensor->invertAxis.y * ((gy / 32.8f) - sensor->gyroOffset.y);
            loc_gyro.z = sensor->invertAxis.z * ((gz / 32.8f) - sensor->gyroOffset.z);

            newDataReady = true;
        }

        // Magnetometer
        if (sensor->magAvailable)
        {
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
                    }
                }
            }
        }

        int64_t endI2c = esp_timer_get_time();
        int64_t currentI2cTime = endI2c - startI2c;
        totalI2cTime += currentI2cTime;
        if (currentI2cTime > maxI2cTime) maxI2cTime = currentI2cTime;

        int64_t startMath = esp_timer_get_time();

        if (newDataReady)
        {
            if (xSemaphoreTake(sensor->dataMutex, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                sensor->accel = loc_accel;
                sensor->gyro = loc_gyro;
                sensor->temperature = loc_temp;

                if (sensor->magAvailable) {
                    sensor->mag = loc_mag;
                }

                // Process measurements if not calibrating
                if (sensor->calibStatus != CALIBRATING)
                {
                    sensor->processMeasurements(dt);
                }

                xSemaphoreGive(sensor->dataMutex);
            }
        }

        totalMathTime += (esp_timer_get_time() - startMath);

        loopCount++;
        // The profiler logs once per `kProfilerWindow` loops. With the
        // INT-driven path running at 1 kHz this stays close to a 1-second
        // cadence; with the polling fallback (~100 Hz) it logs every 5 s.
        // Point 6 of the refactor plan will gate this block behind a build
        // flag — for now it stays on to validate the rate change.
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
    BaseType_t ret = xTaskCreate(
        sensorTask,
        "mpu9250_task",
        4096,
        this,
        5,
        &taskHandle);

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG_MPU9250, "Failed to create sensor task");
        return ESP_FAIL;
    }

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

    return ESP_OK;
}

esp_err_t MPU9250::calibrate()
{
    if (calibStatus == CALIBRATING)
    {
        ESP_LOGW(TAG_MPU9250, "Calibration already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    // Reset calibration values
    resetCalibration();

    // Start calibration in another task to avoid blocking
    calibStatus = CALIBRATING;

    // Create a task for calibration
    TaskHandle_t calibTaskHandle = NULL;
    BaseType_t ret = xTaskCreate(
        [](void *arg)
        {
            MPU9250 *sensor = static_cast<MPU9250 *>(arg);
            sensor->performCalibration();
            vTaskDelete(NULL);
        },
        "mpu9250_calib",
        4096,
        this,
        5,
        &calibTaskHandle);

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG_MPU9250, "Failed to create calibration task");
        calibStatus = NOT_CALIBRATED;
        return ESP_FAIL;
    }

    return ESP_OK;
}

void MPU9250::resetCalibration()
{
    ESP_LOGI(TAG_MPU9250, "Resetting calibration values");
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        accelOffset = {0, 0, 0};
        gyroOffset = {0, 0, 0};
        magOffset = {0, 0, 0};
        magScale = {1.0f, 1.0f, 1.0f};
        xSemaphoreGive(dataMutex);
        ESP_LOGI(TAG_MPU9250, "Calibration values reset");
    }
}

void MPU9250::performCalibration()
{
    ESP_LOGI(TAG_MPU9250, "Starting calibration, keep the sensor still...");
    vTaskDelay(pdMS_TO_TICKS(1000)); // Let sensor stabilize

    Vector3 accelSum = {0, 0, 0};
    Vector3 gyroSum = {0, 0, 0};
    Vector3 magMin = {FLT_MAX, FLT_MAX, FLT_MAX};
    Vector3 magMax = {FLT_MIN, FLT_MIN, FLT_MIN};

    // Collect samples
    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        // Read raw values without applying offsets
        float ax = invertAxis.x * readAccel(0);
        float ay = invertAxis.y * readAccel(2);
        float az = invertAxis.z * readAccel(4);
        float gx = invertAxis.x * readGyro(0);
        float gy = invertAxis.y * readGyro(2);
        float gz = invertAxis.z * readGyro(4);

        accelSum.x += ax;
        accelSum.y += ay;
        accelSum.z += az;
        gyroSum.x += gx;
        gyroSum.y += gy;
        gyroSum.z += gz;

        // For magnetometer, record min/max values while rotating the sensor
        if (magAvailable)
        {
            float mx = invertAxis.x * readMag(0);
            float my = invertAxis.y * readMag(2);
            float mz = invertAxis.z * readMag(4);

            magMin.x = min(magMin.x, mx);
            magMin.y = min(magMin.y, my);
            magMin.z = min(magMin.z, mz);

            magMax.x = max(magMax.x, mx);
            magMax.y = max(magMax.y, my);
            magMax.z = max(magMax.z, mz);
        }

        if (i % 100 == 0)
        {
            ESP_LOGI(TAG_MPU9250, "Calibration progress: %d%%", i * 100 / CALIBRATION_SAMPLES);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        // Calculate average offsets
        gyroOffset.x = gyroSum.x / CALIBRATION_SAMPLES;
        gyroOffset.y = gyroSum.y / CALIBRATION_SAMPLES;
        gyroOffset.z = gyroSum.z / CALIBRATION_SAMPLES;

        // For accelerometer, Z-axis should read 1g (gravity)
        accelOffset.x = accelSum.x / CALIBRATION_SAMPLES;
        accelOffset.y = accelSum.y / CALIBRATION_SAMPLES;
        accelOffset.z = (accelSum.z / CALIBRATION_SAMPLES) - 1.0f; // Subtract gravity

        // For magnetometer, compute hard-iron (offset) and soft-iron (scale) corrections
        if (magAvailable)
        {
            magOffset.x = (magMax.x + magMin.x) / 2.0f;
            magOffset.y = (magMax.y + magMin.y) / 2.0f;
            magOffset.z = (magMax.z + magMin.z) / 2.0f;

            float magDelta = max(max(magMax.x - magMin.x, magMax.y - magMin.y), magMax.z - magMin.z);

            if (magDelta > 0)
            {
                magScale.x = (magMax.x - magMin.x) / magDelta;
                magScale.y = (magMax.y - magMin.y) / magDelta;
                magScale.z = (magMax.z - magMin.z) / magDelta;
            }
        }

        // Reset integrated values
        gyroIntegrated = {0, 0, 0};
        mahonyIntegralError = {0, 0, 0};

        // Update calibration status
        calibStatus = CALIBRATED;

        xSemaphoreGive(dataMutex);
    }

    ESP_LOGI(TAG_MPU9250, "Calibration complete");

    // Log calibration results
    ESP_LOGI(TAG_MPU9250, "Accel offsets: %.3f, %.3f, %.3f", accelOffset.x, accelOffset.y, accelOffset.z);
    ESP_LOGI(TAG_MPU9250, "Gyro offsets: %.3f, %.3f, %.3f", gyroOffset.x, gyroOffset.y, gyroOffset.z);
    if (magAvailable)
    {
        ESP_LOGI(TAG_MPU9250, "Mag offsets: %.3f, %.3f, %.3f", magOffset.x, magOffset.y, magOffset.z);
        ESP_LOGI(TAG_MPU9250, "Mag scale: %.3f, %.3f, %.3f", magScale.x, magScale.y, magScale.z);
    }
}

MPU9250::Orientation MPU9250::getOrientation()
{
    Orientation result;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        result = orientation;
        xSemaphoreGive(dataMutex);
    }
    return result;
}

MPU9250::Vector3 MPU9250::getAccel()
{
    Vector3 result;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        result = accel;
        xSemaphoreGive(dataMutex);
    }
    return result;
}

MPU9250::Vector3 MPU9250::getGyro()
{
    Vector3 result;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        result = gyro;
        xSemaphoreGive(dataMutex);
    }
    return result;
}

MPU9250::Vector3 MPU9250::getMag()
{
    Vector3 result;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        result = mag;
        xSemaphoreGive(dataMutex);
    }
    return result;
}

float MPU9250::getTemperature()
{
    float result = 0;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        result = temperature;
        xSemaphoreGive(dataMutex);
    }
    return result;
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
