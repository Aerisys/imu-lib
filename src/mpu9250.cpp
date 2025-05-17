#include <mpu9250.h>

// Implementation
MPU9250::MPU9250()
    : i2cPort(I2C_NUM_0),
      taskHandle(nullptr),
      dataMutex(nullptr),
      lastProcessTime(0),
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
      filterMode(COMPLEMENTARY)
{

    memset(magAdjustValues, 0, sizeof(magAdjustValues));
    dataMutex = xSemaphoreCreateMutex();
}

MPU9250::~MPU9250()
{
    if (taskHandle != nullptr)
    {
        vTaskDelete(taskHandle);
    }

    if (dataMutex != nullptr)
    {
        vSemaphoreDelete(dataMutex);
    }
}

esp_err_t MPU9250::init(i2c_port_t port, uint8_t sdaPin, uint8_t sclPin)
{
    i2cPort = port;

    // Configure I2C
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sdaPin;
    conf.scl_io_num = sclPin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    conf.clk_flags = 0;

    esp_err_t err = i2c_param_config(i2cPort, &conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_MPU, "I2C config failed");
        return err;
    }

    err = i2c_driver_install(i2cPort, conf.mode, 0, 0, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_MPU, "I2C driver install failed");
        return err;
    }

    // Check MPU9250 identity
    uint8_t whoami = 0;
    err = readRegisters(MPU9250_ADDR, MPU9250_WHO_AM_I, 1, &whoami);
    if (err != ESP_OK || whoami != 0x71)
    {
        ESP_LOGE(TAG_MPU, "MPU9250 not found, WHO_AM_I = 0x%02x", whoami);
        ESP_LOGE(TAG_MPU, "error: %d", err);
        return ESP_FAIL;
    }

    // Reset device
    err = writeRegister(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x80);
    if (err != ESP_OK)
        return err;
    vTaskDelay(pdMS_TO_TICKS(100));

    // Wake up device
    err = writeRegister(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x01);
    if (err != ESP_OK)
        return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    // Configure gyro and accel
    err = writeRegister(MPU9250_ADDR, 0x1A, 0x03); // CONFIG: DLPF_CFG = 3
    if (err != ESP_OK)
        return err;

    err = writeRegister(MPU9250_ADDR, 0x1B, 0x10); // GYRO_CONFIG: ±1000 dps
    if (err != ESP_OK)
        return err;

    err = writeRegister(MPU9250_ADDR, 0x1C, 0x08); // ACCEL_CONFIG: ±4g
    if (err != ESP_OK)
        return err;

    err = writeRegister(MPU9250_ADDR, 0x1D, 0x03); // ACCEL_CONFIG2: DLPF_CFG = 3
    if (err != ESP_OK)
        return err;

    // Configure interrupt pin
    err = writeRegister(MPU9250_ADDR, MPU9250_INT_PIN_CFG, 0x22); // INT_PIN_CFG: BYPASS_EN=1, LATCH_INT_EN=0
    if (err != ESP_OK)
        return err;

    vTaskDelay(pdMS_TO_TICKS(10));

    // Check magnetometer
    uint8_t magWhoami = 0;
    err = readRegisters(AK8963_ADDR, AK8963_WHO_AM_I, 1, &magWhoami);
    if (err == ESP_OK && magWhoami == 0x48)
    {
        ESP_LOGI(TAG_MPU, "AK8963 magnetometer found");
        magAvailable = true;

        // Reset AK8963
        err = writeRegister(AK8963_ADDR, AK8963_CNTL1, 0x00); // Power down
        if (err != ESP_OK)
            return err;
        vTaskDelay(pdMS_TO_TICKS(10));

        // Read adjustment values
        err = writeRegister(AK8963_ADDR, AK8963_CNTL1, 0x0F); // Fuse ROM access mode
        if (err != ESP_OK)
            return err;
        vTaskDelay(pdMS_TO_TICKS(10));

        err = readRegisters(AK8963_ADDR, AK8963_ASAX, 3, magAdjustValues);
        if (err != ESP_OK)
            return err;

        // Set to continuous mode 2 (100Hz)
        err = writeRegister(AK8963_ADDR, AK8963_CNTL1, 0x16);
        if (err != ESP_OK)
            return err;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    else
    {
        ESP_LOGW(TAG_MPU, "AK8963 magnetometer not found");
        magAvailable = false;
    }

    return ESP_OK;
}

esp_err_t MPU9250::writeRegister(uint8_t addr, uint8_t reg, uint8_t data)
{
    uint8_t buffer[2] = {reg, data};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buffer, 2, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2cPort, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        errorCount++;
        ESP_LOGD(TAG_MPU, "Write register failed: device=0x%02x, reg=0x%02x, err=%d", addr, reg, ret);
    }
    else
    {
        successCount++;
    }

    return ret;
}

esp_err_t MPU9250::readRegisters(uint8_t addr, uint8_t reg, uint8_t length, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);

    if (length > 1)
    {
        i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2cPort, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        errorCount++;
        ESP_LOGD(TAG_MPU, "Read registers failed: device=0x%02x, reg=0x%02x, len=%d, err=%d", addr, reg, length, ret);
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
    if (readRegisters(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H + axisOffset, 2, rawData) == ESP_OK)
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
    if (readRegisters(MPU9250_ADDR, MPU9250_GYRO_XOUT_H + axisOffset, 2, rawData) == ESP_OK)
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
    if (readRegisters(AK8963_ADDR, AK8963_ST1, 1, &st1) != ESP_OK || !(st1 & 0x01))
    {
        return 0.0f;
    }

    uint8_t rawData[2] = {0};
    if (readRegisters(AK8963_ADDR, AK8963_HXL + axisOffset, 2, rawData) == ESP_OK)
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
        accel.x = readAccel(0) - accelOffset.x;
        accel.y = readAccel(2) - accelOffset.y;
        accel.z = readAccel(4) - accelOffset.z;

        // Read gyroscope
        gyro.x = readGyro(0) - gyroOffset.x;
        gyro.y = readGyro(2) - gyroOffset.y;
        gyro.z = readGyro(4) - gyroOffset.z;

        // Read magnetometer if available
        if (magAvailable)
        {
            mag.x = (readMag(0) - magOffset.x) * magScale.x;
            mag.y = (readMag(2) - magOffset.y) * magScale.y;
            mag.z = (readMag(4) - magOffset.z) * magScale.z;
        }

        // Read temperature
        uint8_t rawData[2] = {0};
        if (readRegisters(MPU9250_ADDR, 0x41, 2, rawData) == ESP_OK)
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
    orientation.roll = FILTER_ALPHA * (orientation.roll + gyro.x * dt) + (1.0f - FILTER_ALPHA) * rollAcc;
    orientation.pitch = FILTER_ALPHA * (orientation.pitch + gyro.y * dt) + (1.0f - FILTER_ALPHA) * pitchAcc;

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

    // Extract orientation quaternion components
    float q0 = 1.0f; // Initialize quaternion if not already done
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;

    // Convert current euler angles to quaternion
    float cr = cosf(orientation.roll * 0.5f * DEG_TO_RAD);
    float sr = sinf(orientation.roll * 0.5f * DEG_TO_RAD);
    float cp = cosf(orientation.pitch * 0.5f * DEG_TO_RAD);
    float sp = sinf(orientation.pitch * 0.5f * DEG_TO_RAD);
    float cy = cosf(orientation.yaw * 0.5f * DEG_TO_RAD);
    float sy = sinf(orientation.yaw * 0.5f * DEG_TO_RAD);

    q0 = cr * cp * cy + sr * sp * sy;
    q1 = sr * cp * cy - cr * sp * sy;
    q2 = cr * sp * cy + sr * cp * sy;
    q3 = cr * cp * sy - sr * sp * cy;

    // Normalize accelerometer measurement
    float recipNorm = 1.0f / sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    if (isfinite(recipNorm))
    {
        float ax = accel.x * recipNorm;
        float ay = accel.y * recipNorm;
        float az = accel.z * recipNorm;

        // Estimated direction of gravity from quaternion
        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // Error is cross product between estimated and measured direction of gravity
        float ex = ay * vz - az * vy;
        float ey = az * vx - ax * vz;
        float ez = ax * vy - ay * vx;

        // Apply feedback terms
        if (MAHONY_KI > 0.0f)
        {
            mahonyIntegralError.x += ex * MAHONY_KI * dt;
            mahonyIntegralError.y += ey * MAHONY_KI * dt;
            mahonyIntegralError.z += ez * MAHONY_KI * dt;

            // Apply integral feedback
            gx += mahonyIntegralError.x;
            gy += mahonyIntegralError.y;
            gz += mahonyIntegralError.z;
        }

        // Apply proportional feedback
        gx += ex * MAHONY_KP;
        gy += ey * MAHONY_KP;
        gz += ez * MAHONY_KP;
    }

    // Integrate rate of change of quaternion
    gx *= 0.5f;
    gy *= 0.5f;
    gz *= 0.5f;

    float qa = q0;
    float qb = q1;
    float qc = q2;
    float qd = q3;

    q0 += (-qb * gx - qc * gy - qd * gz) * dt;
    q1 += (qa * gx + qc * gz - qd * gy) * dt;
    q2 += (qa * gy - qb * gz + qd * gx) * dt;
    q3 += (qa * gz + qb * gy - qc * gx) * dt;

    // Normalize quaternion
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Convert quaternion to Euler angles
    orientation.roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
    orientation.pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * RAD_TO_DEG;
    orientation.yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;

    // Normalize yaw to 0-360
    while (orientation.yaw < 0)
        orientation.yaw += 360.0f;
    while (orientation.yaw >= 360.0f)
        orientation.yaw -= 360.0f;
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
    sensor->lastProcessTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

    ESP_LOGI(TAG_MPU, "Sensor task started");
    while (true)
    {
        // Calculate time delta
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float dt = (now - sensor->lastProcessTime) / 1000.0f;
        if (dt <= 0.0f)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        sensor->lastProcessTime = now;

        // Read sensor data
        sensor->readAllSensors();

        // Process measurements if not calibrating
        if (sensor->calibStatus != CALIBRATING)
        {
            if (xSemaphoreTake(sensor->dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                sensor->processMeasurements(dt);
                xSemaphoreGive(sensor->dataMutex);
                ESP_LOGI(TAG_MPU, "Orientation: Roll=%.2f, Pitch=%.2f, Yaw=%.2f", sensor->orientation.roll, sensor->orientation.pitch, sensor->orientation.yaw);
            }
        }

        // Wait for next sample
        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
    }
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
        ESP_LOGE(TAG_MPU, "Failed to create sensor task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t MPU9250::calibrate()
{
    if (calibStatus == CALIBRATING)
    {
        ESP_LOGW(TAG_MPU, "Calibration already in progress");
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
        ESP_LOGE(TAG_MPU, "Failed to create calibration task");
        calibStatus = NOT_CALIBRATED;
        return ESP_FAIL;
    }

    return ESP_OK;
}

void MPU9250::resetCalibration()
{
    ESP_LOGI(TAG_MPU, "Resetting calibration values");
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        accelOffset = {0, 0, 0};
        gyroOffset = {0, 0, 0};
        magOffset = {0, 0, 0};
        magScale = {1.0f, 1.0f, 1.0f};
        xSemaphoreGive(dataMutex);
        ESP_LOGI(TAG_MPU, "Calibration values reset");
    }
}

void MPU9250::performCalibration()
{
    ESP_LOGI(TAG_MPU, "Starting calibration, keep the sensor still...");
    vTaskDelay(pdMS_TO_TICKS(1000)); // Let sensor stabilize

    Vector3 accelSum = {0, 0, 0};
    Vector3 gyroSum = {0, 0, 0};
    Vector3 magMin = {FLT_MAX, FLT_MAX, FLT_MAX};
    Vector3 magMax = {FLT_MIN, FLT_MIN, FLT_MIN};

    // Collect samples
    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        // Read raw values without applying offsets
        float ax = readAccel(0);
        float ay = readAccel(2);
        float az = readAccel(4);
        float gx = readGyro(0);
        float gy = readGyro(2);
        float gz = readGyro(4);

        accelSum.x += ax;
        accelSum.y += ay;
        accelSum.z += az;
        gyroSum.x += gx;
        gyroSum.y += gy;
        gyroSum.z += gz;

        // For magnetometer, record min/max values while rotating the sensor
        if (magAvailable)
        {
            float mx = readMag(0);
            float my = readMag(2);
            float mz = readMag(4);

            magMin.x = min(magMin.x, mx);
            magMin.y = min(magMin.y, my);
            magMin.z = min(magMin.z, mz);

            magMax.x = max(magMax.x, mx);
            magMax.y = max(magMax.y, my);
            magMax.z = max(magMax.z, mz);
        }

        if (i % 100 == 0)
        {
            ESP_LOGI(TAG_MPU, "Calibration progress: %d%%", i * 100 / CALIBRATION_SAMPLES);
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

    ESP_LOGI(TAG_MPU, "Calibration complete");

    // Log calibration results
    ESP_LOGI(TAG_MPU, "Accel offsets: %.3f, %.3f, %.3f", accelOffset.x, accelOffset.y, accelOffset.z);
    ESP_LOGI(TAG_MPU, "Gyro offsets: %.3f, %.3f, %.3f", gyroOffset.x, gyroOffset.y, gyroOffset.z);
    if (magAvailable)
    {
        ESP_LOGI(TAG_MPU, "Mag offsets: %.3f, %.3f, %.3f", magOffset.x, magOffset.y, magOffset.z);
        ESP_LOGI(TAG_MPU, "Mag scale: %.3f, %.3f, %.3f", magScale.x, magScale.y, magScale.z);
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
        ESP_LOGE(TAG_MPU, "Invalid filter mode");
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}