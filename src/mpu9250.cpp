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
      filterMode(COMPLEMENTARY),
      q{1.0f, 0.0f, 0.0f, 0.0f} // Mahony quaternion init
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
    esp_err_t err;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sdaPin;
    conf.scl_io_num = sclPin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    conf.clk_flags = 0;

    err = i2c_param_config(i2cPort, &conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_MPU9250, "I2C config failed");
        return err;
    }

    // Initialize I2C driver
    err = i2c_driver_install(i2cPort, conf.mode, 0, 0, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_MPU9250, "I2C driver install failed");
        return err;
    }

    // Check MPU9250 identity
    uint8_t whoami = 0;
    err = readRegisters(MPU9250_ADDR, MPU9250_WHO_AM_I, 1, &whoami);
    if (err != ESP_OK || whoami != 0x71)
    {
        ESP_LOGE(TAG_MPU9250, "MPU9250 not found, WHO_AM_I = 0x%02x", whoami);
        ESP_LOGE(TAG_MPU9250, "error: %d", err);
        return ESP_FAIL;
    }

    // Reset device
    err = writeRegister(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x80);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_MPU9250, "Failed to reset MPU9250");
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    // Wake up device
    err = writeRegister(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x01);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_MPU9250, "Failed to wake up MPU9250");
        return err;
    }
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
        ESP_LOGI(TAG_MPU9250, "AK8963 magnetometer found");
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
        ESP_LOGW(TAG_MPU9250, "AK8963 magnetometer not found");
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
        ESP_LOGD(TAG_MPU9250, "Write register failed: device=0x%02x, reg=0x%02x, err=%d", addr, reg, ret);
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
        ESP_LOGD(TAG_MPU9250, "Read registers failed: device=0x%02x, reg=0x%02x, len=%d, err=%d", addr, reg, length, ret);
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

// Helper function for constraining values
inline float constrain(float val, float min, float max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

// Fast inverse square root implementation (Quake III algorithm)
inline float fastInvSqrt(float x)
{
    float halfx = 0.5f * x;
    int i = *(int *)&x;
    i = 0x5f3759df - (i >> 1);
    x = *(float *)&i;
    x = x * (1.5f - halfx * x * x); // One Newton iteration
    return x;
}

void MPU9250::updateMahonyFilter(float dt)
{
    // Early exit if dt is too small to avoid numerical instability
    if (dt < 1e-6f)
        return;

    // Convert gyro to rad/s
    float gx = (gyro.x) * DEG_TO_RAD;
    float gy = (gyro.y) * DEG_TO_RAD;
    float gz = (gyro.z) * DEG_TO_RAD;

    // Unpack quaternion state
    float q0 = q.w;
    float q1 = q.x;
    float q2 = q.y;
    float q3 = q.z;

    // Normalize accelerometer measurement
    float ax = accel.x;
    float ay = accel.y;
    float az = accel.z;

    float accNormSq = ax * ax + ay * ay + az * az;
    if (accNormSq < 1e-12f)
        return; // Avoid division by very small number
    float invAccNorm = fastInvSqrt(accNormSq);
    ax *= invAccNorm;
    ay *= invAccNorm;
    az *= invAccNorm;

    // Estimated direction of gravity using quaternion rotation
    // Optimized calculation without explicitly computing the rotation matrix
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    float vx = 2.0f * (q1q3 - q0q2);
    float vy = 2.0f * (q0q1 + q2q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // Error is cross product between measured and estimated gravity
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    // Apply integral feedback with anti-windup
    const float maxIntegralError = 0.1f; // Prevent unbounded growth

    if (MAHONY_KI > 0.0f)
    {
        mahonyIntegralError.x += ex * MAHONY_KI * dt;
        mahonyIntegralError.y += ey * MAHONY_KI * dt;
        mahonyIntegralError.z += ez * MAHONY_KI * dt;

        // Apply anti-windup to integral term
        mahonyIntegralError.x = constrain(mahonyIntegralError.x, -maxIntegralError, maxIntegralError);
        mahonyIntegralError.y = constrain(mahonyIntegralError.y, -maxIntegralError, maxIntegralError);
        mahonyIntegralError.z = constrain(mahonyIntegralError.z, -maxIntegralError, maxIntegralError);

        // Apply integral correction to gyro measurements
        gx += mahonyIntegralError.x;
        gy += mahonyIntegralError.y;
        gz += mahonyIntegralError.z;
    }

    // Apply proportional feedback
    gx += MAHONY_KP * ex;
    gy += MAHONY_KP * ey;
    gz += MAHONY_KP * ez;

    // Integrate quaternion rate using more efficient integration
    // Pre-calculate half_dt to avoid repeated multiplication
    float half_dt = 0.5f * dt;

    // First-order quaternion integration
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * half_dt;
    q1 += (q0 * gx + q2 * gz - q3 * gy) * half_dt;
    q2 += (q0 * gy - q1 * gz + q3 * gx) * half_dt;
    q3 += (q0 * gz + q1 * gy - q2 * gx) * half_dt;

    // Normalize quaternion - using fast inverse square root would be even faster
    float qNormSq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    if (qNormSq < 1e-12f)
    {
        // Handle degenerate case
        q.w = 1.0f;
        q.x = q.y = q.z = 0.0f;
    }
    else
    {
        // Normal case
        float invQNorm = fastInvSqrt(qNormSq);
        q.w = q0 * invQNorm;
        q.x = q1 * invQNorm;
        q.y = q2 * invQNorm;
        q.z = q3 * invQNorm;
    }

    // Convert quaternion to Euler angles (deg)
    // Use quaternion to Euler conversion with atan2 for better numerical stability
    if (switchRollPitch)
    {
        orientation.pitch = atan2f(2.0f * (q.w * q.x + q.y * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y)) * RAD_TO_DEG;
        orientation.roll = asinf(constrain(2.0f * (q.w * q.y - q.z * q.x), -1.0f, 1.0f)) * RAD_TO_DEG;
    }
    else
    {
        orientation.roll = atan2f(2.0f * (q.w * q.x + q.y * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y)) * RAD_TO_DEG;
        orientation.pitch = asinf(constrain(2.0f * (q.w * q.y - q.z * q.x), -1.0f, 1.0f)) * RAD_TO_DEG;
    }

    orientation.yaw = atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z)) * RAD_TO_DEG;

    // Normalize yaw to [0,360) range
    orientation.yaw = fmodf(orientation.yaw + 360.0f, 360.0f);
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

    ESP_LOGI(TAG_MPU9250, "Sensor task started");
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
                ESP_LOGI(TAG_MPU9250, "Orientation: Roll=%.2f, Pitch=%.2f, Yaw=%.2f", sensor->orientation.roll, sensor->orientation.pitch, sensor->orientation.yaw);
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
        ESP_LOGE(TAG_MPU9250, "Failed to create sensor task");
        return ESP_FAIL;
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
    switchRollPitch = switchRollPitch;
    return ESP_OK;
}