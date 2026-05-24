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
//
// The bus is created and OWNED by the application here (not by the library)
// so that several devices (IMU, baro, OSD, ...) can share the same I2C port
// in the consumer project.
// -----------------------------------------------------------------------------

#include <mpu9250.h>
#include "driver/i2c_master.h"

extern "C" void app_main()
{
    // 1. Create the I2C bus once for the whole application.
    //
    //    The bus itself has no clock — each device handle sets its own SCL
    //    speed when added (see `MPU9250::Config::sclSpeedHz`, default 400 kHz).
    //    On a real drone, prefer external 2.2 kOhm pull-ups on SDA/SCL over
    //    the ESP32 internal ones for Fast Mode signal integrity.
    i2c_master_bus_config_t busCfg = {};
    busCfg.i2c_port           = I2C_NUM_0;
    busCfg.sda_io_num         = GPIO_NUM_21;
    busCfg.scl_io_num         = GPIO_NUM_22;
    busCfg.clk_source         = I2C_CLK_SRC_DEFAULT;
    busCfg.glitch_ignore_cnt  = 7;
    busCfg.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t bus = nullptr;
    esp_err_t err = i2c_new_master_bus(&busCfg, &bus);
    if (err != ESP_OK)
    {
        ESP_LOGE("APP", "Failed to create I2C bus (err=%d)", err);
        return;
    }

    // 2. Initialize the IMU on that bus. The library attaches its own device
    //    handles to the bus but does not own the bus itself.
    //
    //    The MPU9250 INT pin is wired to GPIO 19 here: this enables the
    //    interrupt-driven sample loop (1 kHz at default DLPF settings).
    //    Set `intPin = GPIO_NUM_NC` to keep the legacy 100 Hz polling
    //    behaviour when the INT line is not connected.
    static MPU9250 imu;

    MPU9250::Config cfg;
    cfg.intPin = GPIO_NUM_19;
    err = imu.init(bus, cfg);
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
    // Optional: tune Mahony gains for this airframe.
    //   Kp around 1.0 gives a faster correction (more responsive but noisier);
    //   Ki kept at 0 to avoid integral windup during aggressive maneuvers.
    imu.setMahonyGains(1.0f, 0.0f);

    err = imu.startSensorTask();
    if (err != ESP_OK)
    {
        ESP_LOGE("APP", "Failed to start sensor task");
        return;
    }

    // 3. Consume the quaternion from the control loop. On a drone, prefer the
    //    quaternion over Euler angles to avoid gimbal lock during loops/flips.
    while (true)
    {
        MPU9250::Quaternion q = imu.getQuaternion();
        ESP_LOGI("APP", "q = [w=%.3f x=%.3f y=%.3f z=%.3f]", q.w, q.x, q.y, q.z);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
