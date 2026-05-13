#pragma once

#include <cstdint>

struct WindReading {
    uint8_t id = 0;
    float speed_ms = 0.0f;
    uint16_t direction_deg = 0;
};
