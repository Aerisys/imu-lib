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
#include "imu_sensor.h"
#include "driver/i2c_master.h"
#include "nvs_flash.h"

extern "C" void app_main()
{
    // 0. Initialise NVS so the IMU library can persist its calibration.
    //    Recovering from a corrupted partition by erasing it is a standard
    //    ESP-IDF pattern — do this once at the very top of the app.
    esp_err_t nvsErr = nvs_flash_init();
    if (nvsErr == ESP_ERR_NVS_NO_FREE_PAGES || nvsErr == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

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
    // Optional: pin the internal sensor task to a specific core.
    //   tskNO_AFFINITY (default) lets the scheduler migrate the task — fine
    //   for bench testing without Wi-Fi.
    //   On a drone with Wi-Fi/BLE running, prefer APP_CPU_NUM (= 1) and pin
    //   the IDF networking stack to PRO_CPU_NUM via sdkconfig:
    //     CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_0=y
    //     CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0=y
    //
    //   cfg.taskCoreId   = APP_CPU_NUM;
    //   cfg.taskPriority = 5;
    err = imu.init(bus, cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE("APP", "Failed to initialize MPU9250 (err=%d)", err);
        return;
    }

    imu.setSwitchRollPitch(true);
    ESP_LOGI("APP", "MPU9250 initialized");

    // Calibration is auto-loaded from NVS by init(). Only run a fresh
    // gyro+accel calibration if nothing was found on disk — otherwise we'd
    // uselessly burn 10 s of immobility on every boot.
    //
    // The mag calibration is NOT done automatically: it requires the user
    // to physically rotate the sensor through every orientation for 30 s.
    // Trigger `imu.calibrateMag()` from a dedicated boot mode (e.g. a long
    // button press at power-on, or via the drone configurator) when you
    // actually want to recalibrate it.
    if (imu.getCalibrationStatus() != MPU9250::CALIBRATED)
    {
        err = imu.calibrateGyroAccel();
        if (err != ESP_OK)
        {
            ESP_LOGE("APP", "Failed to start gyro/accel calibration");
            return;
        }
        ESP_LOGI("APP", "Gyro/Accel calibration started (~10 s, keep still)");
    }
    else
    {
        ESP_LOGI("APP", "Calibration loaded from NVS, skipping cold calibration");
    }

    imu.setFilterMode(MPU9250::MAHONY);
    // Optional: tune Mahony gains for this airframe.
    //   Kp around 1.0 gives a faster correction (more responsive but noisier);
    //   Ki kept at 0 to avoid integral windup during aggressive maneuvers.
    imu.setMahonyGains(1.0f, 0.0f);

    // Optional: install a gyro notch filter on the motor fundamental.
    //
    // Measure the motor frequency with a vibration log at hover (e.g. log
    // raw gyro on one axis for a few seconds and FFT it). For a 5" quad on
    // 2306 motors hovering at ~50% throttle, the fundamental is often in
    // the 150-250 Hz range. Bandwidth 40-80 Hz is a reasonable starting
    // point — wider = more rejection but more phase lag in band.
    //
    // Comment this out until you have measured your airframe.
    //
    //   imu.setGyroNotch(200.0f /*centerHz*/, 60.0f /*bwHz*/, 1000.0f /*sampleRateHz*/);

    err = imu.startSensorTask();
    if (err != ESP_OK)
    {
        ESP_LOGE("APP", "Failed to start sensor task");
        return;
    }

    // 3. Consume samples from the control loop.
    //
    //    We go through the abstract IMUSensor interface so this loop does
    //    not need to know which concrete chip is wired in (swap the type
    //    above to a future ICM-* and this stays unchanged).
    //
    //    Two improvements vs. the old "poll every 500 ms" pattern:
    //      - waitForNewSample() blocks until the sensor task publishes a
    //        fresh bundle, so the loop runs exactly in lockstep with the
    //        1 kHz sample rate instead of guessing a period.
    //      - getSnapshot() returns every value as a coherent set taken at
    //        the same instant (one seqlock read), so accel/gyro/quaternion
    //        all come from the same iteration — important for PID stability.
    IMUSensor* sensor = &imu;

    uint32_t logEvery = 0;
    while (true)
    {
        if (sensor->waitForNewSample(100) != ESP_OK)
        {
            ESP_LOGW("APP", "No IMU sample in 100 ms — sensor stuck?");
            continue;
        }

        IMUSensor::SampleBundle s = sensor->getSnapshot();

        // ---- Drone control loop would run its PID here, using s.gyro,
        //      s.quaternion, etc. The loop is now timed BY the IMU.

        // Throttle log output (otherwise 1 kHz of ESP_LOGI floods the UART).
        if (++logEvery >= 1000)
        {
            logEvery = 0;
            ESP_LOGI("APP",
                     "q=[%.3f %.3f %.3f %.3f]  gyro=[%.1f %.1f %.1f] deg/s  T=%.1fC",
                     s.quaternion.w, s.quaternion.x, s.quaternion.y, s.quaternion.z,
                     s.gyro.x, s.gyro.y, s.gyro.z,
                     s.temperature);
            ESP_LOGI("APP", "roll=%.1f pitch=%.1f yaw=%.1f", 
                s.orientation.roll, s.orientation.pitch, s.orientation.yaw);
        }
    }
}
