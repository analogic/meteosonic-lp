#include "components/hal_ds18b20/ds18b20.h"

#include <stddef.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace hal_ds18b20 {

static const char *TAG = "ds18b20";
static constexpr uint8_t kOneWireCmdSkipRom = 0xCC;
static constexpr uint8_t kDs18b20CmdConvertT = 0x44;
static constexpr uint8_t kDs18b20CmdReadScratchpad = 0xBE;
static constexpr uint32_t kConversionPollIntervalMs = 20;

static inline void onewire_drive_low(gpio_num_t pin)
{
    gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(pin, 0);
}

static inline void onewire_release(gpio_num_t pin)
{
    gpio_set_direction(pin, GPIO_MODE_INPUT);
}

static bool onewire_reset(gpio_num_t pin)
{
    onewire_drive_low(pin);
    esp_rom_delay_us(480);
    onewire_release(pin);
    esp_rom_delay_us(70);
    const bool present = gpio_get_level(pin) == 0;
    esp_rom_delay_us(410);
    return present;
}

static void onewire_write_bit(gpio_num_t pin, bool bit)
{
    onewire_drive_low(pin);
    if (bit) {
        esp_rom_delay_us(6);
        onewire_release(pin);
        esp_rom_delay_us(64);
        return;
    }

    esp_rom_delay_us(60);
    onewire_release(pin);
    esp_rom_delay_us(10);
}

static bool onewire_read_bit(gpio_num_t pin)
{
    onewire_drive_low(pin);
    esp_rom_delay_us(6);
    onewire_release(pin);
    esp_rom_delay_us(9);
    const bool bit = gpio_get_level(pin) != 0;
    esp_rom_delay_us(55);
    return bit;
}

static void onewire_write_byte(gpio_num_t pin, uint8_t value)
{
    for (int bit = 0; bit < 8; bit++) {
        onewire_write_bit(pin, (value & (1U << bit)) != 0);
    }
}

static uint8_t onewire_read_byte(gpio_num_t pin)
{
    uint8_t value = 0;
    for (int bit = 0; bit < 8; bit++) {
        if (onewire_read_bit(pin)) {
            value |= static_cast<uint8_t>(1U << bit);
        }
    }
    return value;
}

static uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for (int bit = 0; bit < 8; bit++) {
            const uint8_t mix = static_cast<uint8_t>((crc ^ inbyte) & 0x01U);
            crc >>= 1;
            if (mix != 0) {
                crc ^= 0x8CU;
            }
            inbyte >>= 1;
        }
    }
    return crc;
}

bool start_conversion(gpio_num_t pin)
{
    if (!onewire_reset(pin)) {
        return false;
    }

    onewire_write_byte(pin, kOneWireCmdSkipRom);
    onewire_write_byte(pin, kDs18b20CmdConvertT);
    return true;
}

bool wait_for_conversion(gpio_num_t pin, uint32_t timeout_ms)
{
    const int64_t deadline_us = esp_timer_get_time() + (static_cast<int64_t>(timeout_ms) * 1000);

    while (esp_timer_get_time() < deadline_us) {
        if (onewire_read_bit(pin)) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(kConversionPollIntervalMs));
    }

    return onewire_read_bit(pin);
}

Reading read_temperature(gpio_num_t pin)
{
    Reading reading{};
    uint8_t scratchpad[9];

    if (!onewire_reset(pin)) {
        ESP_LOGW(TAG, "DS18B20 not detected on GPIO %d", (int)pin);
        return reading;
    }

    onewire_write_byte(pin, kOneWireCmdSkipRom);
    onewire_write_byte(pin, kDs18b20CmdReadScratchpad);
    for (size_t i = 0; i < sizeof(scratchpad); i++) {
        scratchpad[i] = onewire_read_byte(pin);
    }

    if (crc8(scratchpad, sizeof(scratchpad)) != 0) {
        ESP_LOGW(TAG, "DS18B20 scratchpad CRC mismatch");
        return reading;
    }

    const int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(scratchpad[1]) << 8) | scratchpad[0]);
    reading.celsius = static_cast<float>(raw) / 16.0f;
    reading.valid = true;
    return reading;
}

} // namespace hal_ds18b20
