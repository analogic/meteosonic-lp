#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"
#include "common/wind_reading.h"
#include "components/hal_ds18b20/ds18b20.h"
#include "components/hal_ulp_sdi_wind/ulp_sdi_wind.h"
#include "components/nbiot/modem.h"

static const char *TAG = "lp_main";
static constexpr size_t kWindStatsMaxSamples = 120;
static constexpr size_t kWindRollingWindowSamples = 3;
static constexpr double kPi = 3.14159265358979323846;
static constexpr gpio_num_t kDs18b20Pin = GPIO_NUM_18;
static constexpr uint32_t kDs18b20ConversionTimeoutMs = 750;
#if !defined(LOW_BATTERY_ENTER_MV) || !defined(LOW_BATTERY_RECOVER_MV) || !defined(LOW_BATTERY_CHECK_INTERVAL_SEC)
#error "LOW_BATTERY_* thresholds must be defined in platformio.ini build_flags"
#endif
static constexpr uint16_t kLowBatteryEnterMv = LOW_BATTERY_ENTER_MV;
static constexpr uint16_t kLowBatteryRecoverMv = LOW_BATTERY_RECOVER_MV;
static constexpr uint32_t kLowBatteryCheckIntervalSec = LOW_BATTERY_CHECK_INTERVAL_SEC;

static i2c_master_bus_handle_t i2c_bus_0 = NULL;
static XPowersPMU pmu;
static bool pmu_ready = false;
static RTC_DATA_ATTR bool rtc_power_cut_sleep_active = false;
static RTC_DATA_ATTR bool rtc_low_battery_guard_active = false;

static bool woke_from_deep_sleep(void)
{
    return esp_reset_reason() == ESP_RST_DEEPSLEEP;
}

static esp_err_t i2c_bus_init(void)
{
    i2c_master_bus_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    cfg.i2c_port = I2C_NUM_0;
    cfg.sda_io_num = static_cast<gpio_num_t>(I2C_SDA_0);
    cfg.scl_io_num = static_cast<gpio_num_t>(I2C_SCL_0);
    cfg.glitch_ignore_cnt = 7;

    esp_err_t ret = i2c_new_master_bus(&cfg, &i2c_bus_0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

static esp_err_t pmu_init(void)
{
    if (!pmu.begin(i2c_bus_0, AXP2101_SLAVE_ADDRESS)) {
        ESP_LOGE(TAG, "PMU init failed: AXP2101 not responding");
        return ESP_FAIL;
    }
    pmu_ready = true;
    return ESP_OK;
}

static void pmu_prepare_modem_power(void)
{
    if (!pmu_ready) {
        return;
    }

    pmu.disableDC3LowVoltageTurnOff();
    pmu.setDC3Voltage(3000);
    pmu.enableDC3(); // MODEM

    pmu.disableDC5LowVoltageTurnOff();
    pmu.setDC5Voltage(3300);
    pmu.enableDC5(); // ANEMOMETER

    pmu.setBLDO1Voltage(3300);
    pmu.enableBLDO1(); // level conversion

    // GPS only
    //pmu.setBLDO2Voltage(3300);
    //pmu.enableBLDO2();
}

static void pmu_prepare_battery_measurement(void)
{
    if (!pmu_ready) {
        return;
    }

    pmu.setBLDO1Voltage(3300);
    pmu.enableBLDO1(); // keep I2C level conversion/pull-ups stable while reading PMU ADC
    pmu.enableBattDetection();
    pmu.enableBattVoltageMeasure();
}

static uint16_t read_battery_mv(void)
{
    if (!pmu_ready) {
        return 0;
    }
    pmu_prepare_battery_measurement();
    return pmu.getBattVoltage();
}

static inline void set_pin_input(gpio_num_t pin)
{
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(pin, GPIO_FLOATING);
}

static inline void set_pin_input_pulldown(gpio_num_t pin)
{
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY);
}

static void release_pmu_i2c(void)
{
    if (pmu_ready) {
        pmu.deinit();
        pmu_ready = false;
    }
    if (i2c_bus_0 != NULL) {
        esp_err_t err = i2c_del_master_bus(i2c_bus_0);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "i2c_del_master_bus failed: %s", esp_err_to_name(err));
        }
        i2c_bus_0 = NULL;
    }
}

static void prepare_extreme_sleep_gpio(void)
{
    set_pin_input(GPIO_NUM_4);
    set_pin_input(GPIO_NUM_5);
    set_pin_input(GPIO_NUM_7);
    set_pin_input_pulldown(GPIO_NUM_15);
    set_pin_input_pulldown(GPIO_NUM_38);
    set_pin_input_pulldown(GPIO_NUM_39);
    set_pin_input_pulldown(GPIO_NUM_40);
    set_pin_input_pulldown(GPIO_NUM_41);
    set_pin_input(GPIO_NUM_42);
    set_pin_input_pulldown(GPIO_NUM_45);
    set_pin_input_pulldown(GPIO_NUM_46);
    set_pin_input_pulldown(GPIO_NUM_47);
    set_pin_input_pulldown(GPIO_NUM_48);
}

static void pmu_prepare_for_sleep(void)
{
    if (!pmu_ready) {
        return;
    }

    pmu.disableTemperatureMeasure();
    pmu.disableBattDetection();
    pmu.disableVbusVoltageMeasure();
    pmu.enableBattVoltageMeasure();
    pmu.disableSystemVoltageMeasure();

    pmu.disableDC2();
    pmu.enableDC3(); // MODEM
    pmu.disableDC4();
    pmu.enableDC5();

    pmu.disableALDO1();
    pmu.disableALDO2();
    pmu.disableALDO3();
    pmu.disableALDO4();
    pmu.enableBLDO1();
    pmu.disableBLDO2();
    pmu.disableCPUSLDO();
    pmu.disableDLDO1();
    pmu.disableDLDO2();

    pmu.clearIrqStatus();
    pmu.setChargingLedMode(XPOWERS_CHG_LED_OFF);
}

static void pmu_prepare_for_low_battery_sleep(void)
{
    if (!pmu_ready) {
        return;
    }

    pmu.disableTemperatureMeasure();
    pmu.enableBattDetection();
    pmu.enableBattVoltageMeasure();
    pmu.disableVbusVoltageMeasure();
    pmu.disableSystemVoltageMeasure();

    pmu.disableDC2();
    pmu.disableDC3(); // MODEM off
    pmu.disableDC4();
    pmu.disableDC5(); // ANEMOMETER off

    pmu.disableALDO1();
    pmu.disableALDO2();
    pmu.disableALDO3();
    pmu.disableALDO4();
    pmu.disableBLDO1(); // level conversion off
    pmu.disableBLDO2();
    pmu.disableCPUSLDO();
    pmu.disableDLDO1();
    pmu.disableDLDO2();

    pmu.clearIrqStatus();
    pmu.setChargingLedMode(XPOWERS_CHG_LED_OFF);
}

static modem_config_t make_modem_cfg(void)
{
    modem_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    cfg.uart_num = UART_NUM_1;
    cfg.tx_pin = MODEM_TXD_PIN;
    cfg.rx_pin = MODEM_RXD_PIN;
    cfg.baud_rate = 115200;

    cfg.reset_method = MODEM_RESET_GPIO_PULSE;
    cfg.power_gpio = MODEM_PWRKEY_PIN;
    cfg.power_pulse_ms = MODEM_PWRKEY_PULSE_MS;

    cfg.apn = MODEM_APN;
    cfg.coap_server = COAP_SERVER;
    cfg.coap_port = COAP_PORT;
    cfg.coap_path = COAP_PATH;

    return cfg;
}

struct WindStats {
    size_t sample_count = 0;
    bool has_average = false;
    bool has_rolling_3s = false;
    float speed_avg_ms = 0.0f;
    float gust_3s_ms = 0.0f;
    float min_3s_ms = 0.0f;
    float direction_avg_deg = 0.0f;
    float direction_yamartino_sigma_deg = 0.0f;
};

static float normalize_degrees(double degrees)
{
    while (degrees < 0.0) {
        degrees += 360.0;
    }
    while (degrees >= 360.0) {
        degrees -= 360.0;
    }
    return static_cast<float>(degrees);
}

static WindStats calculate_wind_stats(const std::vector<WindReading> &samples)
{
    WindStats stats{};
    stats.sample_count = samples.size();
    if (samples.empty()) {
        return stats;
    }

    double speed_sum = 0.0;
    double sin_sum = 0.0;
    double cos_sum = 0.0;

    for (const WindReading &sample : samples) {
        speed_sum += sample.speed_ms;

        const double radians = static_cast<double>(sample.direction_deg) * kPi / 180.0;
        sin_sum += std::sin(radians);
        cos_sum += std::cos(radians);
    }

    const double inv_count = 1.0 / static_cast<double>(samples.size());
    stats.speed_avg_ms = static_cast<float>(speed_sum * inv_count);
    stats.direction_avg_deg = normalize_degrees(std::atan2(sin_sum, cos_sum) * 180.0 / kPi);

    const double sin_avg = sin_sum * inv_count;
    const double cos_avg = cos_sum * inv_count;
    const double resultant = std::min(1.0, std::sqrt((sin_avg * sin_avg) + (cos_avg * cos_avg)));
    const double epsilon = std::sqrt(std::max(0.0, 1.0 - resultant));
    const double sigma_rad = std::asin(epsilon) *
                             (1.0 + ((2.0 / std::sqrt(3.0)) - 1.0) * epsilon * epsilon * epsilon);
    stats.direction_yamartino_sigma_deg = static_cast<float>(sigma_rad * 180.0 / kPi);
    stats.has_average = true;

    if (samples.size() >= kWindRollingWindowSamples) {
        bool initialized = false;
        for (size_t i = 0; i + kWindRollingWindowSamples <= samples.size(); i++) {
            double window_sum = 0.0;
            for (size_t j = 0; j < kWindRollingWindowSamples; j++) {
                window_sum += samples[i + j].speed_ms;
            }
            const float window_avg = static_cast<float>(window_sum / kWindRollingWindowSamples);
            if (!initialized) {
                stats.gust_3s_ms = window_avg;
                stats.min_3s_ms = window_avg;
                initialized = true;
            } else {
                stats.gust_3s_ms = std::max(stats.gust_3s_ms, window_avg);
                stats.min_3s_ms = std::min(stats.min_3s_ms, window_avg);
            }
        }
        stats.has_rolling_3s = true;
    }

    return stats;
}

static WindStats read_wind_stats_snapshot()
{
    std::vector<WindReading> samples;
    ulp_sdi_wind::read_recent_samples(samples, kWindStatsMaxSamples);
    return calculate_wind_stats(samples);
}

static void format_json_float_or_null(char *dst, size_t dst_len, bool valid, float value)
{
    if (!valid) {
        snprintf(dst, dst_len, "null");
        return;
    }
    snprintf(dst, dst_len, "%.2f", static_cast<double>(value));
}

static void format_json_int_or_null(char *dst, size_t dst_len, bool valid, float value)
{
    if (!valid) {
        snprintf(dst, dst_len, "null");
        return;
    }
    snprintf(dst, dst_len, "%ld", static_cast<long>(lroundf(value)));
}

static void print_ulp_buffer_state(const char *context)
{
    ulp_sdi_wind::tick(0);

    const ulp_sdi_wind::DebugStatus status = ulp_sdi_wind::debug_status();

    ESP_LOGI(TAG,
             "%s: ULP initialized=%d, phase=%" PRIu32 ", seq=%" PRIu32
             ", sample_seq=%" PRIu32 ", samples=%" PRIu32 ", parse=%" PRIu32,
             context,
             status.initialized ? 1 : 0,
             status.phase,
             status.sequence,
             status.sample_sequence,
             status.sample_count,
             status.parse_status);

    ESP_LOGI(TAG,
             "%s: M status=%" PRIu32 ", len=%" PRIu32 ", bytes=%" PRIu32
             ", D0 status=%" PRIu32 ", len=%" PRIu32 ", bytes=%" PRIu32,
             context,
             status.m_status,
             status.m_rx_len,
             status.m_byte_count,
             status.d_status,
             status.d_rx_len,
             status.d_byte_count);
}

static bool start_or_attach_ulp(bool from_wakeup)
{
    if (from_wakeup) {
        if (ulp_sdi_wind::attach_shared_state()) {
            ESP_LOGI(TAG, "Attached to running ULP SDI-12 reader");
            return true;
        }
        ESP_LOGW(TAG, "No ULP samples found after wake, starting ULP SDI-12 reader");
    }

    return ulp_sdi_wind::init();
}

static void run_modem_network_cycle(bool from_wakeup)
{
    ESP_LOGI(TAG, "Main task stack free before modem cycle: %u bytes",
             (unsigned int)uxTaskGetStackHighWaterMark(NULL));

    const modem_config_t modem_cfg = make_modem_cfg();
    const bool ds18b20_conversion_started = hal_ds18b20::start_conversion(kDs18b20Pin);

    modem_driver_start(&modem_cfg);

    bool coap_needs_init = true;
    if (from_wakeup) {
        modem_resume_fast();
        coap_needs_init = false;
        ESP_LOGI(TAG, "Wake path: optimistic publish using existing modem state");
    }

    if (coap_needs_init) {
        modem_exit_sleep();
        modem_init_modem();
    }

    const WindStats wind_stats = read_wind_stats_snapshot();
    hal_ds18b20::Reading ds18b20{};
    if (ds18b20_conversion_started) {
        if (hal_ds18b20::wait_for_conversion(kDs18b20Pin, kDs18b20ConversionTimeoutMs)) {
            ds18b20 = hal_ds18b20::read_temperature(kDs18b20Pin);
        } else {
            ESP_LOGW(TAG, "DS18B20 conversion timed out");
        }
    } else {
        ESP_LOGW(TAG, "DS18B20 conversion could not be started");
    }

    char speed_avg[16];
    char gust_3s[16];
    char min_3s[16];
    char direction_avg[16];
    char direction_sigma[16];
    char ds18b20_temp[16];
    format_json_float_or_null(speed_avg, sizeof(speed_avg), wind_stats.has_average, wind_stats.speed_avg_ms);
    format_json_float_or_null(gust_3s, sizeof(gust_3s), wind_stats.has_rolling_3s, wind_stats.gust_3s_ms);
    format_json_float_or_null(min_3s, sizeof(min_3s), wind_stats.has_rolling_3s, wind_stats.min_3s_ms);
    format_json_int_or_null(direction_avg,
                            sizeof(direction_avg),
                            wind_stats.has_average,
                            wind_stats.direction_avg_deg);
    format_json_float_or_null(direction_sigma,
                              sizeof(direction_sigma),
                              wind_stats.has_average,
                              wind_stats.direction_yamartino_sigma_deg);
    format_json_float_or_null(ds18b20_temp, sizeof(ds18b20_temp), ds18b20.valid, ds18b20.celsius);

    char payload[320];
    const unsigned int batt_mv = pmu_ready ? (unsigned int)pmu.getBattVoltage() : 0;
    const int payload_len = snprintf(payload,
                                     sizeof(payload),
                                     "{\"b\":%u,"
                                     "\"t\":%s,"
                                     "\"w\":%s,"
                                     "\"g\":%s,"
                                     "\"m\":%s,"
                                     "\"d\":%s}",
                                     batt_mv,
                                     ds18b20_temp,
                                     speed_avg,
                                     gust_3s,
                                     min_3s,
                                     direction_avg);
    if (payload_len < 0 || payload_len >= static_cast<int>(sizeof(payload))) {
        ESP_LOGW(TAG, "JSON payload truncated");
    }
    ESP_LOGI(TAG, "CoAP payload: %s", payload);

    bool published = false;
    modem_set_coap_reinit_allowed(coap_needs_init);
    modem_set_publish_pdp_check_enabled(coap_needs_init);
    if (!coap_needs_init || modem_init_coap()) {
        published = modem_publish_coap(modem_cfg.coap_path,
                                       payload,
                                       COAP_PUBLISH_TIMEOUT_SEC);
    }

    if (!published && !coap_needs_init) {
        ESP_LOGW(TAG, "Optimistic wake publish failed, running full modem init");
        modem_exit_sleep();
        modem_init_modem();
        modem_set_coap_reinit_allowed(true);
        modem_set_publish_pdp_check_enabled(true);
        if (modem_init_coap()) {
            published = modem_publish_coap(modem_cfg.coap_path,
                                           payload,
                                           COAP_PUBLISH_TIMEOUT_SEC);
        }
    }

    ESP_LOGI(TAG, "CoAP publish %s", published ? "OK" : "FAILED");

    if (!modem_enter_sleep()) {
        ESP_LOGW(TAG, "DTR modem sleep request failed");
    }

    ESP_LOGI(TAG, "Main task stack free after modem cycle: %u bytes",
             (unsigned int)uxTaskGetStackHighWaterMark(NULL));
}

static void enter_deep_sleep(uint32_t interval_sec, bool keep_rtc_periph_on)
{
    ESP_ERROR_CHECK(esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL));
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup((uint64_t)interval_sec * 1000000ULL));
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,
                        keep_rtc_periph_on ? ESP_PD_OPTION_ON : ESP_PD_OPTION_OFF);
    esp_deep_sleep_start();
}

static void enter_power_cut_sleep(uint32_t interval_sec)
{
    rtc_power_cut_sleep_active = true;
    ulp_sdi_wind::stop();
    modem_driver_stop();
    pmu_prepare_for_low_battery_sleep();
    vTaskDelay(pdMS_TO_TICKS(20));
    release_pmu_i2c();
    prepare_extreme_sleep_gpio();
    fflush(stdout);
    enter_deep_sleep(interval_sec, false);
}

static void enter_low_battery_sleep(uint16_t batt_mv)
{
    ESP_LOGW(TAG,
             "Battery low (%u mV < %u mV), entering low-power guard for %" PRIu32 " s",
             batt_mv,
             (unsigned int)kLowBatteryEnterMv,
             kLowBatteryCheckIntervalSec);
    rtc_low_battery_guard_active = true;
    enter_power_cut_sleep(kLowBatteryCheckIntervalSec);
}

extern "C" void app_main(void)
{
    const bool from_wakeup = woke_from_deep_sleep();
    const bool from_power_cut_sleep = from_wakeup && rtc_power_cut_sleep_active;
    const bool from_low_battery_guard = from_wakeup && rtc_low_battery_guard_active;
    ESP_LOGI(TAG,
             "Boot start: reason=%d, path=%s%s",
             (int)esp_reset_reason(),
             from_wakeup ? "wake" : "cold",
             from_low_battery_guard ? ", low-battery guard" : "");

    if (i2c_bus_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed");
    }
    if (pmu_init() != ESP_OK) {
        ESP_LOGE(TAG, "PMU init failed");
    }

    const uint16_t startup_batt_mv = read_battery_mv();
    ESP_LOGI(TAG, "Battery voltage: %u mV", (unsigned int)startup_batt_mv);
    if (startup_batt_mv > 0) {
        const uint16_t threshold_mv = from_low_battery_guard ? kLowBatteryRecoverMv : kLowBatteryEnterMv;
        if (startup_batt_mv < threshold_mv) {
            ESP_LOGW(TAG,
                     "Battery below %u mV threshold, staying in guard mode",
                     (unsigned int)threshold_mv);
            enter_low_battery_sleep(startup_batt_mv);
        }
    } else {
        ESP_LOGW(TAG, "Battery voltage unavailable, continuing normal cycle");
    }
    rtc_low_battery_guard_active = false;
    rtc_power_cut_sleep_active = false;

    pmu_prepare_modem_power();

    if (!start_or_attach_ulp(from_wakeup && !from_power_cut_sleep)) {
        ESP_LOGE(TAG, "ULP init failed");
    } else {
        print_ulp_buffer_state(from_wakeup ? "Wake ULP buffer" : "Boot ULP buffer");
    }

    run_modem_network_cycle(from_wakeup && !from_power_cut_sleep);
    const uint16_t sleep_batt_mv = read_battery_mv();
    if (sleep_batt_mv > 0 && sleep_batt_mv < kLowBatteryEnterMv) {
        enter_low_battery_sleep(sleep_batt_mv);
    }
    pmu_prepare_for_sleep();

    vTaskDelay(pdMS_TO_TICKS(20));
    enter_deep_sleep(MODEM_WAKE_INTERVAL_SEC, true);
}
