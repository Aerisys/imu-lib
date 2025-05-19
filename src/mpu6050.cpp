#include "mpu6050.h"

// Implementation
MPU6050::MPU6050()
    : i2cPort(I2C_NUM_0),
      taskHandle(nullptr),
      dataMutex(nullptr),
      lastProcessTime(0),
      accel{0, 0, 0},
      gyro{0, 0, 0},
      temperature(0),
      orientation{0, 0, 0},
      gyroIntegrated{0, 0, 0},
      mahonyIntegralError{0, 0, 0},
      accelOffset{0, 0, 0},
      gyroOffset{0, 0, 0},
      calibStatus(NOT_CALIBRATED),
      errorCount(0),
      successCount(0),
      filterMode(COMPLEMENTARY),
      q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f)
{
    dataMutex = xSemaphoreCreateMutex();
}

MPU6050::~MPU6050()
{
    if (taskHandle)
        vTaskDelete(taskHandle);
    if (dataMutex)
        vSemaphoreDelete(dataMutex);
}

esp_err_t MPU6050::init(i2c_port_t port, uint8_t sdaPin, uint8_t sclPin)
{
    i2cPort = port;
    // I2C configuration
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sdaPin;
    conf.scl_io_num = sclPin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;

    ESP_RETURN_ON_ERROR(i2c_param_config(i2cPort, &conf), TAG_MPU6050, "I2C config failed");
    ESP_RETURN_ON_ERROR(i2c_driver_install(i2cPort, conf.mode, 0, 0, 0), TAG_MPU6050, "I2C driver install failed");

    // Check device ID
    uint8_t whoami = 0;
    ESP_RETURN_ON_ERROR(readRegisters(MPU6050_WHO_AM_I, 1, &whoami), TAG_MPU6050, "Failed to read WHO_AM_I");
    if (whoami != 0x68)
    {
        ESP_LOGE(TAG_MPU6050, "MPU6050 not found (0x%02X)", whoami);
        return ESP_FAIL;
    }

    // Reset & wakeup
    ESP_RETURN_ON_ERROR(writeRegister(MPU6050_PWR_MGMT_1, 0x80), TAG_MPU6050, "Reset failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(writeRegister(MPU6050_PWR_MGMT_1, 0x01), TAG_MPU6050, "Wake failed");
    vTaskDelay(pdMS_TO_TICKS(10));

    // Configure filters and ranges
    ESP_RETURN_ON_ERROR(writeRegister(0x1A, 0x03), TAG_MPU6050, "Config DLPF failed");
    ESP_RETURN_ON_ERROR(writeRegister(0x1B, 0x10), TAG_MPU6050, "Gyro config failed");
    ESP_RETURN_ON_ERROR(writeRegister(0x1C, 0x08), TAG_MPU6050, "Accel config failed");
    ESP_RETURN_ON_ERROR(writeRegister(0x1D, 0x03), TAG_MPU6050, "Accel DLPF failed");

    readAllSensors();
    // Compute initial orientation from accel
    float roll0  = atan2f(accel.y, accel.z);
    float pitch0 = atan2f(-accel.x, sqrtf(accel.y*accel.y + accel.z*accel.z));
    orientation.roll  = roll0  * RAD_TO_DEG;
    orientation.pitch = pitch0 * RAD_TO_DEG;
    orientation.yaw   = 0.0f;

    // Initialize quaternion to match that orientation
    // yaw=0 => q0=cos(roll/2)*cos(pitch/2), q1=sin(roll/2)*cos(pitch/2), q2=cos(roll/2)*sin(pitch/2), q3=0
    float cr = cosf(roll0/2),  sr = sinf(roll0/2);
    float cp = cosf(pitch0/2), sp = sinf(pitch0/2);
    q0 = cr*cp;
    q1 = sr*cp;
    q2 = cr*sp;
    q3 = 0.0f;

    // Zero integral error
    mahonyIntegralError = {0, 0, 0};

    return ESP_OK;
}

esp_err_t MPU6050::writeRegister(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2cPort, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK)
        successCount++;
    else
        errorCount++;
    return ret;
}

esp_err_t MPU6050::readRegisters(uint8_t reg, uint8_t length, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    if (length > 1)
        i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2cPort, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK)
        successCount++;
    else
        errorCount++;
    return ret;
}

void MPU6050::readAllSensors()
{
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) != pdTRUE)
        return;

    uint8_t buf[14] = {0};
    if (readRegisters(MPU6050_ACCEL_XOUT_H, 14, buf) == ESP_OK)
    {
        int16_t ax = (buf[0] << 8) | buf[1];
        int16_t ay = (buf[2] << 8) | buf[3];
        int16_t az = (buf[4] << 8) | buf[5];
        int16_t tempRaw = (buf[6] << 8) | buf[7];
        int16_t gx = (buf[8] << 8) | buf[9];
        int16_t gy = (buf[10] << 8) | buf[11];
        int16_t gz = (buf[12] << 8) | buf[13];

        accel.x = (float)ax / 8192.0f - accelOffset.x;
        accel.y = (float)ay / 8192.0f - accelOffset.y;
        accel.z = (float)az / 8192.0f - accelOffset.z;
        temperature = (float)tempRaw / 333.87f + 21.0f;
        gyro.x = (float)gx / 32.8f - gyroOffset.x;
        gyro.y = (float)gy / 32.8f - gyroOffset.y;
        gyro.z = (float)gz / 32.8f - gyroOffset.z;
    }

    xSemaphoreGive(dataMutex);
}

float MPU6050::readAccel(uint8_t axisOffset)
{
    uint8_t buf[2] = {0};
    if (readRegisters(MPU6050_ACCEL_XOUT_H + axisOffset, 2, buf) == ESP_OK)
    {
        int16_t ax = (buf[0] << 8) | buf[1];
        accel.x = (float)ax / 8192.0f - accelOffset.x;
    }
    return accel.x;
}

float MPU6050::readGyro(uint8_t axisOffset)
{
    uint8_t buf[2] = {0};
    if (readRegisters(MPU6050_GYRO_XOUT_H + axisOffset, 2, buf) == ESP_OK)
    {
        int16_t gx = (buf[0] << 8) | buf[1];
        gyro.x = (float)gx / 32.8f - gyroOffset.x;
    }
    return gyro.x;
}


void MPU6050::updateComplementaryFilter(float dt)
{
    gyroIntegrated.x += gyro.x * dt;
    gyroIntegrated.y += gyro.y * dt;
    gyroIntegrated.z += gyro.z * dt;

    float rollAcc = atan2f(accel.y, accel.z) * RAD_TO_DEG;
    float pitchAcc = atan2f(-accel.x, sqrtf(accel.y * accel.y + accel.z * accel.z)) * RAD_TO_DEG;

    orientation.roll = FILTER_ALPHA * (orientation.roll + gyro.x * dt) + (1.0f - FILTER_ALPHA) * rollAcc;
    orientation.pitch = FILTER_ALPHA * (orientation.pitch + gyro.y * dt) + (1.0f - FILTER_ALPHA) * pitchAcc;
    orientation.yaw += gyro.z * dt;
    while (orientation.yaw < 0)
        orientation.yaw += 360;
    while (orientation.yaw >= 360)
        orientation.yaw -= 360;
}

void MPU6050::updateMahonyFilter(float dt)
{
    // Simplified: only use accel feedback, skip mag
    float gx = gyro.x * DEG_TO_RAD;
    float gy = gyro.y * DEG_TO_RAD;
    float gz = gyro.z * DEG_TO_RAD;

    // Compute accel normalized
    float norm = sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    if (norm > 0)
    {
        accel.x /= norm;
        accel.y /= norm;
        accel.z /= norm;
    }

    // Compute error and apply PI
    float ex = (accel.y * sinf(orientation.roll * DEG_TO_RAD) - accel.z * cosf(orientation.roll * DEG_TO_RAD));
    float ey = (accel.z * sinf(orientation.pitch * DEG_TO_RAD) - accel.x * cosf(orientation.pitch * DEG_TO_RAD));
    if (MAHONY_KI > 0)
    {
        mahonyIntegralError.x += ex * MAHONY_KI * dt;
        mahonyIntegralError.y += ey * MAHONY_KI * dt;
        gx += mahonyIntegralError.x;
        gy += mahonyIntegralError.y;
    }
    gx += ex * MAHONY_KP;
    gy += ey * MAHONY_KP;

    // Integrate
    orientation.roll += gx * dt * RAD_TO_DEG;
    orientation.pitch += gy * dt * RAD_TO_DEG;
    orientation.yaw += gz * dt * RAD_TO_DEG;

    while (orientation.yaw < 0)
        orientation.yaw += 360;
    while (orientation.yaw >= 360)
        orientation.yaw -= 360;
}

void MPU6050::updateMahonyQuat(float dt)
{
    // Convert gyro to rad/sec
    float gx = gyro.x * DEG_TO_RAD;
    float gy = gyro.y * DEG_TO_RAD;
    float gz = gyro.z * DEG_TO_RAD;

    // Normalize accel
    float norm = sqrtf(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z);
    if (norm == 0.0f) return;
    accel.x /= norm; accel.y /= norm; accel.z /= norm;

    // Estimated gravity direction
    float vx = 2*(q1*q3 - q0*q2);
    float vy = 2*(q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // Error = cross(acc_meas, acc_est)
    float ex = (accel.y * vz - accel.z * vy);
    float ey = (accel.z * vx - accel.x * vz);
    float ez = (accel.x * vy - accel.y * vx);

    // Integral term
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

    // Integrate quaternion
    float qa=q0, qb=q1, qc=q2;
    q0 += (-qb*gx - qc*gy - q3*gz) * (0.5f*dt);
    q1 += ( qa*gx + qc*gz - q3*gy) * (0.5f*dt);
    q2 += ( qa*gy - qb*gz + q3*gx) * (0.5f*dt);
    q3 += ( qa*gz + qb*gy - qc*gx) * (0.5f*dt);

    // Normalize quaternion
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0/=norm; q1/=norm; q2/=norm; q3/=norm;

    // Extract Euler angles
    orientation.roll  = atan2f(2*(q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3) * RAD_TO_DEG;
    orientation.pitch = -asinf(2*(q1*q3 - q0*q2)) * RAD_TO_DEG;
    orientation.yaw   = atan2f(2*(q0*q3 + q1*q2), q0*q0 + q1*q1 - q2*q2 - q3*q3) * RAD_TO_DEG;
    if (orientation.yaw < 0) orientation.yaw += 360;
}

void MPU6050::processMeasurements(float dt)
{
    if (filterMode == COMPLEMENTARY)
        updateComplementaryFilter(dt);
    else if (filterMode == MAHONY)
        updateMahonyFilter(dt);
    else if (filterMode == MAHONY_QUAT)
        updateMahonyQuat(dt);
    else
        ESP_LOGE(TAG_MPU6050, "Unknown filter mode");
}

void MPU6050::sensorTask(void *arg)
{
    MPU6050 *sensor = static_cast<MPU6050 *>(arg);
    sensor->lastProcessTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    while (true)
    {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float dt = (now - sensor->lastProcessTime) / 1000.0f;
        if (dt > 0)
        {
            sensor->lastProcessTime = now;
            sensor->readAllSensors();
            if (sensor->calibStatus != CALIBRATING)
            {
                if (xSemaphoreTake(sensor->dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    sensor->processMeasurements(dt);
                    xSemaphoreGive(sensor->dataMutex);
                    ESP_LOGI(TAG_MPU6050, "Orientation: Roll: %.2f, Pitch: %.2f, Yaw: %.2f", sensor->orientation.roll, sensor->orientation.pitch, sensor->orientation.yaw);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t MPU6050::startSensorTask()
{
    if (xTaskCreate(sensorTask, "mpu6050_task", 4096, this, 5, &taskHandle) != pdPASS)
    {
        ESP_LOGE(TAG_MPU6050, "Task create failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

MPU6050::Orientation MPU6050::getOrientation()
{
    Orientation o;
    xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10));
    o = orientation;
    xSemaphoreGive(dataMutex);
    return o;
}

MPU6050::Vector3 MPU6050::getAccel()
{
    Vector3 v;
    xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10));
    v = accel;
    xSemaphoreGive(dataMutex);
    return v;
}

MPU6050::Vector3 MPU6050::getGyro()
{
    Vector3 v;
    xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10));
    v = gyro;
    xSemaphoreGive(dataMutex);
    return v;
}

float MPU6050::getTemperature()
{
    float t;
    xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10));
    t = temperature;
    xSemaphoreGive(dataMutex);
    return t;
}

bool MPU6050::isSensorHealthy() { return !(errorCount > 100 && successCount < errorCount); }

esp_err_t MPU6050::setFilterMode(FilterMode mode)
{
    filterMode = mode;
    return ESP_OK;
}

esp_err_t MPU6050::calibrate()
{
    if (calibStatus == CALIBRATING)
    {
        ESP_LOGW(TAG_MPU6050, "Calibration already in progress");
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
            MPU6050 *sensor = static_cast<MPU6050 *>(arg);
            sensor->performCalibration();
            vTaskDelete(NULL);
        },
        "MPU6050_calib",
        4096,
        this,
        5,
        &calibTaskHandle);

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG_MPU6050, "Failed to create calibration task");
        calibStatus = NOT_CALIBRATED;
        return ESP_FAIL;
    }

    return ESP_OK;
}

void MPU6050::resetCalibration()
{
    ESP_LOGI(TAG_MPU6050, "Resetting calibration values");
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        accelOffset = {0, 0, 0};
        gyroOffset = {0, 0, 0};
        xSemaphoreGive(dataMutex);
        ESP_LOGI(TAG_MPU6050, "Calibration values reset");
    }
}

void MPU6050::performCalibration()
{
    ESP_LOGI(TAG_MPU6050, "Starting calibration, keep the sensor still...");
    vTaskDelay(pdMS_TO_TICKS(1000)); // Let sensor stabilize

    Vector3 accelSum = {0, 0, 0};
    Vector3 gyroSum = {0, 0, 0};

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

        // Accumulate values
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            accelSum.x += ax;
            accelSum.y += ay;
            accelSum.z += az;
            gyroSum.x += gx;
            gyroSum.y += gy;
            gyroSum.z += gz;
            xSemaphoreGive(dataMutex);
        }

        if (i % 100 == 0)
        {
            ESP_LOGI(TAG_MPU6050, "Calibration progress: %d%%", i * 100 / CALIBRATION_SAMPLES);
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

        // Reset integrated values
        gyroIntegrated = {0, 0, 0};
        mahonyIntegralError = {0, 0, 0};

        // Update calibration status
        calibStatus = CALIBRATED;

        xSemaphoreGive(dataMutex);
    }

    ESP_LOGI(TAG_MPU6050, "Calibration complete");

    // Log calibration results
    ESP_LOGI(TAG_MPU6050, "Accel offsets: %.3f, %.3f, %.3f", accelOffset.x, accelOffset.y, accelOffset.z);
    ESP_LOGI(TAG_MPU6050, "Gyro offsets: %.3f, %.3f, %.3f", gyroOffset.x, gyroOffset.y, gyroOffset.z);
}

