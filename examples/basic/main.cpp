// -----------------------------------------------------------------------------
// imu-lib — Basic standalone example
// -----------------------------------------------------------------------------
// This file is NOT part of the library itself. It exists only to allow building
// and flashing the library as a standalone ESP-IDF / PlatformIO project for
// manual testing.
//
// When `imu-lib` is consumed as a dependency by another project (e.g. the
// drone firmware), this file is NOT compiled: PlatformIO's library manager
// only pulls `src/` and `include/`, and ESP-IDF projects depending on the
// library never include `examples/` in their component sources.
// -----------------------------------------------------------------------------

#include <mpu9250.h>

extern "C" void app_main()
{
    static MPU9250 imu;

    esp_err_t err = imu.init(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22); // SDA / SCL
    if (err != ESP_OK)
    {
        ESP_LOGE("APP", "Failed to initialize MPU9250 (err=%d)", err);
        return;
    }

    imu.setSwitchRollPitch(true);
    ESP_LOGI("APP", "MPU9250 initialized");

    err = imu.calibrate();
    if (err != ESP_OK)
    {
        ESP_LOGE("APP", "Failed to start calibration");
        return;
    }
    ESP_LOGI("APP", "Calibration started");

    imu.setFilterMode(MPU9250::MAHONY);

    err = imu.startSensorTask();
    if (err != ESP_OK)
    {
        ESP_LOGE("APP", "Failed to start sensor task");
        return;
    }
}
