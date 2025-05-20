#include <mpu9250.h>
#include <mpu6050.h>

extern "C" void app_main()
{
    static MPU9250 imu;
    // static MPU6050 imu; // Uncomment this line to use MPU6050 instead of MPU9250


    esp_err_t err = imu.init(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22); // Set appropriate SDA/SCL pins
    if (err != ESP_OK)
    {
        ESP_LOGE("APP", "Failed to initialize MPU9250");
        ESP_LOGE("APP", "Error code: %d", err);
        return;
    }
    ESP_LOGI("APP", "MPU9250 initialized");

    err = imu.calibrate();
    if (err != ESP_OK)
    {
        ESP_LOGE("APP", "Failed to start calibration");
        return;
    }
    ESP_LOGI("APP", "Calibration started");

    err = imu.setFilterMode(MPU9250::MAHONY);
    if (err != ESP_OK)
    {
        ESP_LOGE("APP", "Failed to set filter mode");
        return;
    }
    ESP_LOGI("APP", "Filter mode set to Mahony");

    err = imu.startSensorTask(); // Assuming this creates a FreeRTOS task for reading sensor data
    if (err != ESP_OK)
    {
        ESP_LOGE("APP", "Failed to start sensor task");
        return;
    }
}
