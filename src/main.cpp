#include <mpu9250.h>
#include <mpu6050.h>

extern "C" void app_main()
{
    static MPU6050 imu;

    esp_err_t err = imu.init(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22); // Set appropriate SDA/SCL pins
    if (err != ESP_OK)
    {
        ESP_LOGE("APP", "Failed to initialize MPU6050");
        return;
    }
    ESP_LOGI("APP", "MPU6050 initialized");

    err = imu.calibrate();
    if (err != ESP_OK)
    {
        ESP_LOGE("APP", "Failed to start calibration");
        return;
    }
    ESP_LOGI("APP", "Calibration started");

    err = imu.setFilterMode(MPU6050::MAHONY_QUAT);
    if (err != ESP_OK)
    {
        ESP_LOGE("APP", "Failed to set filter mode");
        return;
    }
    ESP_LOGI("APP", "Filter mode set");

    err = imu.startSensorTask();
    if (err != ESP_OK)
    {
        ESP_LOGE("APP", "Failed to start sensor task");
        return;
    }
    ESP_LOGI("APP", "Sensor task started");
}

// extern "C" void app_main()
// {
//     static MPU9250 imu;

//     esp_err_t err = imu.init(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22); // Set appropriate SDA/SCL pins
//     if (err != ESP_OK)
//     {
//         ESP_LOGE("APP", "Failed to initialize MPU9250");
//         return;
//     }

//     ESP_LOGI("APP", "MPU9250 initialized");

//     err = imu.calibrate();
//     if (err != ESP_OK)
//     {
//         ESP_LOGE("APP", "Failed to start calibration");
//         return;
//     }

//     err = imu.startSensorTask(); // Assuming this creates a FreeRTOS task for reading sensor data
//     if (err != ESP_OK)
//     {
//         ESP_LOGE("APP", "Failed to start sensor task");
//         return;
//     }
// }
