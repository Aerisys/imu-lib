#include <mpu9250.h>

extern "C" void app_main()
{
    static MPU9250 imu;

    esp_err_t err = imu.init(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22); // Set appropriate SDA/SCL pins
    if (err != ESP_OK)
    {
        ESP_LOGE("APP", "Failed to initialize MPU9250");
        return;
    }

    ESP_LOGI("APP", "MPU9250 initialized");

    err = imu.calibrate();
    if (err != ESP_OK)
    {
        ESP_LOGE("APP", "Failed to start calibration");
        return;
    }

    err = imu.startSensorTask(); // Assuming this creates a FreeRTOS task for reading sensor data
    if (err != ESP_OK)
    {
        ESP_LOGE("APP", "Failed to start sensor task");
        return;
    }
}
