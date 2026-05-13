#pragma once

#include "driver/gpio.h"

namespace hal_ds18b20 {

struct Reading {
    bool valid = false;
    float celsius = 0.0f;
};

bool wait_for_conversion(gpio_num_t pin, uint32_t timeout_ms);
bool start_conversion(gpio_num_t pin);
Reading read_temperature(gpio_num_t pin);

} // namespace hal_ds18b20
