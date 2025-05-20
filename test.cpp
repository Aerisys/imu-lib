#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_NUM I2C_NUM_0

void try_mpu9250() {
    ESP_LOGI("MPU9250", "Trying to communicate with MPU9250...");

    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Start condition
    i2c_master_start(cmd);

    // Write device address with write bit
    i2c_master_write_byte(cmd, (0x68 << 1) | I2C_MASTER_WRITE, true);

    // Write register address (WHO_AM_I = 0x75)
    i2c_master_write_byte(cmd, 0x75, true);

    // Repeated start condition
    i2c_master_start(cmd);

    // Write device address with read bit
    i2c_master_write_byte(cmd, (0x68 << 1) | I2C_MASTER_READ, true);

    // Read one byte with NACK (end of reading)
    i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);

    // Stop condition
    i2c_master_stop(cmd);

    // Execute I2C commands
    esp_err_t espRc = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);

    i2c_cmd_link_delete(cmd);

    if (espRc == ESP_OK) {
        ESP_LOGI("MPU9250", "MPU9250 WHO_AM_I: 0x%02X", data);
    } else {
        ESP_LOGE("MPU9250", "Failed to communicate with MPU9250, error = %d", espRc);
    }
}


void i2c_scan() {
    esp_err_t espRc;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_21;
    conf.scl_io_num = GPIO_NUM_22;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    conf.clk_flags = 0;
    
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    for (int addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        espRc = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (espRc == ESP_OK) {
            ESP_LOGI("SCAN", "Found device at 0x%02X", addr);

            // Optionally, you can read a byte from the device to confirm it's responsive
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
            uint8_t data;
            i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
            i2c_master_stop(cmd);
            espRc = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);
            if (espRc == ESP_OK) {
                ESP_LOGI("SCAN", "Device at 0x%02X responded with data: 0x%02X", addr, data);
                try_mpu9250(); // Call the function to try to communicate with the MPU9250
            } else {
                ESP_LOGE("SCAN", "Failed to read from device at 0x%02X", addr);
            }
        }
    }
}


extern "C" void app_main() {
    ESP_LOGI("I2C", "Starting I2C scan...");
    i2c_scan();
    ESP_LOGI("I2C", "Scan complete.");
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}