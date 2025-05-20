#pragma once
#include "driver/i2c.h"

class I2CDevice {
public:
    I2CDevice(i2c_port_t port, gpio_num_t sda, gpio_num_t scl);
    esp_err_t writeRegister(uint8_t addr, uint8_t reg, uint8_t data);
    esp_err_t readRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf);
private:
    i2c_port_t port_;
};