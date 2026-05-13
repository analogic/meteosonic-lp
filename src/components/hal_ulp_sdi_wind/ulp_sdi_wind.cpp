#include "components/hal_ulp_sdi_wind/ulp_sdi_wind.h"

#include <inttypes.h>
#include <algorithm>

#include "esp_clk_tree.h"
#include "esp_err.h"
#include "esp_log.h"
#include "ulp_main.h"
#include "ulp_common.h"
#include "ulp_riscv.h"

namespace ulp_sdi_wind {
namespace {

const char *TAG = "ulp_sdi_wind";

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

constexpr uint32_t ULP_CYCLES_PER_US_Q10_DEFAULT = 17U * 1024U + 512U;
constexpr uint32_t ULP_CLOCK_SCALE_NUM = 985;
constexpr uint32_t ULP_CLOCK_SCALE_DEN = 1000;
constexpr uint32_t ULP_STUCK_TIMEOUT_MS = 3000;
constexpr uint32_t RTC_RECALIBRATE_PERIOD_MS = 60000;
constexpr size_t SDI_SAMPLE_BUF_LEN = 120;
constexpr uint32_t ULP_WAKEUP_PERIOD_US = 1000000;
constexpr size_t SNAPSHOT_RETRY_COUNT = 5;

uint32_t g_recalibrate_ms = 0;
uint32_t g_unchanged_ms = 0;
uint32_t g_progress_sequence = 0;
bool g_initialized = false;

inline uint32_t ulp_read32(const uint32_t &value)
{
    return *reinterpret_cast<volatile const uint32_t *>(&value);
}

uint32_t get_ulp_cycles_per_us_q10()
{
    uint32_t rtc_fast_hz = 0;
    esp_err_t err = esp_clk_tree_src_get_freq_hz(
        SOC_MOD_CLK_RTC_FAST,
        ESP_CLK_TREE_SRC_FREQ_PRECISION_EXACT,
        &rtc_fast_hz);
    if (err != ESP_OK || rtc_fast_hz == 0) {
        ESP_LOGW(TAG, "RTC_FAST calibration failed, using default timing");
        return ULP_CYCLES_PER_US_Q10_DEFAULT;
    }

    uint64_t q10 = ((uint64_t)rtc_fast_hz << 10) / 1000000ULL;
    q10 = (q10 * ULP_CLOCK_SCALE_NUM) / ULP_CLOCK_SCALE_DEN;
    ESP_LOGI(TAG,
             "RTC_FAST calibrated to %" PRIu32 " Hz, scale=%" PRIu32 "/%" PRIu32,
             rtc_fast_hz,
             ULP_CLOCK_SCALE_NUM,
             ULP_CLOCK_SCALE_DEN);
    return static_cast<uint32_t>(q10);
}

void refresh_timing(bool verbose)
{
    uint32_t calibrated = get_ulp_cycles_per_us_q10();
    uint32_t previous = ulp_read32(ulp_sdi_cycles_per_us_q10);

    ulp_sdi_cycles_per_us_q10 = calibrated;
    if (verbose || previous != calibrated) {
        ESP_LOGI(TAG, "ULP timing q10=%" PRIu32, calibrated);
    }
}

}  // namespace

bool init()
{
    ulp_riscv_reset();

    ulp_sdi_status = 0;
    ulp_sdi_sequence = 0;
    ulp_sdi_phase = 0;
    ulp_sdi_parse_status = 0;
    ulp_sdi_report_sequence = 0;
    ulp_sdi_report_parse_status = 0;
    ulp_sdi_report_direction = 0;
    ulp_sdi_report_strength_centi = 0;
    ulp_sdi_report_sample_count = 0;
    ulp_sdi_m_status = 0;
    ulp_sdi_m_rx_len = 0;
    ulp_sdi_d_status = 0;
    ulp_sdi_d_rx_len = 0;
    ulp_sdi_report_m_timeout_count = 0;
    ulp_sdi_report_m_false_start_count = 0;
    ulp_sdi_report_m_byte_count = 0;
    ulp_sdi_report_d_timeout_count = 0;
    ulp_sdi_report_d_false_start_count = 0;
    ulp_sdi_report_d_byte_count = 0;

    esp_err_t err = ulp_riscv_load_binary(
        ulp_main_bin_start,
        ulp_main_bin_end - ulp_main_bin_start);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ULP binary load failed: %s", esp_err_to_name(err));
        return false;
    }

    refresh_timing(true);
    err = ulp_set_wakeup_period(0, ULP_WAKEUP_PERIOD_US);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ULP timer setup failed: %s", esp_err_to_name(err));
        return false;
    }

    err = ulp_riscv_run();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ULP start failed: %s", esp_err_to_name(err));
        return false;
    }

    g_recalibrate_ms = 0;
    g_unchanged_ms = 0;
    g_progress_sequence = ulp_read32(ulp_sdi_report_sequence);
    g_initialized = true;
    ESP_LOGI(TAG, "ULP SDI-12 reader started");
    return true;
}

bool attach_shared_state()
{
    const uint32_t sequence = ulp_read32(ulp_sdi_report_sequence);
    g_recalibrate_ms = 0;
    g_unchanged_ms = 0;
    g_progress_sequence = sequence;
    g_initialized = true;
    return sequence != 0;
}

void stop()
{
    ulp_riscv_halt();
    g_recalibrate_ms = 0;
    g_unchanged_ms = 0;
    g_progress_sequence = 0;
    g_initialized = false;
    ESP_LOGI(TAG, "ULP SDI-12 reader stopped");
}

void tick(uint32_t elapsed_ms)
{
    if (!g_initialized) {
        return;
    }

    g_recalibrate_ms += elapsed_ms;
    if (g_recalibrate_ms >= RTC_RECALIBRATE_PERIOD_MS) {
        refresh_timing(false);
        g_recalibrate_ms = 0;
    }

    uint32_t sequence = ulp_read32(ulp_sdi_report_sequence);
    if (sequence != g_progress_sequence) {
        g_progress_sequence = sequence;
        g_unchanged_ms = 0;
    } else {
        g_unchanged_ms += elapsed_ms;
    }
}

bool read_latest(WindReading& out)
{
    std::vector<WindReading> samples;
    if (read_recent_samples(samples, 1) == 0) {
        return false;
    }
    out = samples.back();
    return true;
}

size_t read_recent_samples(std::vector<WindReading>& out, size_t max_samples)
{
    out.clear();
    if (!g_initialized) {
        return 0;
    }

    if (max_samples == 0) {
        return 0;
    }

    for (size_t attempt = 0; attempt < SNAPSHOT_RETRY_COUNT; attempt++) {
        const uint32_t seq_before = ulp_read32(ulp_sdi_sample_sequence);
        if ((seq_before & 1U) != 0) {
            continue;
        }

        const uint32_t count = ulp_read32(ulp_sdi_sample_count);
        const uint32_t head = ulp_read32(ulp_sdi_sample_head);
        if (count == 0) {
            return 0;
        }

        const uint32_t take = std::min<uint32_t>(
            static_cast<uint32_t>(max_samples),
            std::min<uint32_t>(count, SDI_SAMPLE_BUF_LEN));

        const uint32_t start = (head + SDI_SAMPLE_BUF_LEN - take) % SDI_SAMPLE_BUF_LEN;
        out.reserve(take);
        for (uint32_t i = 0; i < take; i++) {
            const uint32_t idx = (start + i) % SDI_SAMPLE_BUF_LEN;
            WindReading reading{};
            reading.id = 0;
            reading.direction_deg = static_cast<uint16_t>(ulp_read32(ulp_sdi_direction_samples[idx]));
            reading.speed_ms = static_cast<float>(ulp_read32(ulp_sdi_strength_centi_samples[idx])) / 100.0f;
            out.push_back(reading);
        }

        const uint32_t seq_after = ulp_read32(ulp_sdi_sample_sequence);
        if (seq_before == seq_after && (seq_after & 1U) == 0) {
            return out.size();
        }
        out.clear();
    }

    return 0;
}

bool check_stuck()
{
    if (!g_initialized) {
        return false;
    }
    if (g_unchanged_ms < ULP_STUCK_TIMEOUT_MS) {
        return false;
    }
    g_unchanged_ms = 0;
    return true;
}

uint32_t phase()
{
    return g_initialized ? ulp_read32(ulp_sdi_phase) : 0;
}

DebugStatus debug_status()
{
    DebugStatus status{};
    status.initialized = g_initialized;
    if (!g_initialized) {
        return status;
    }

    status.phase = ulp_read32(ulp_sdi_phase);
    status.sequence = ulp_read32(ulp_sdi_report_sequence);
    status.sample_sequence = ulp_read32(ulp_sdi_report_sample_sequence);
    status.parse_status = ulp_read32(ulp_sdi_report_parse_status);
    status.sample_count = ulp_read32(ulp_sdi_sample_count);
    status.m_status = ulp_read32(ulp_sdi_report_m_status);
    status.m_rx_len = ulp_read32(ulp_sdi_report_m_rx_len);
    status.m_timeout_count = ulp_read32(ulp_sdi_report_m_timeout_count);
    status.m_false_start_count = ulp_read32(ulp_sdi_report_m_false_start_count);
    status.m_byte_count = ulp_read32(ulp_sdi_report_m_byte_count);
    status.d_status = ulp_read32(ulp_sdi_report_d_status);
    status.d_rx_len = ulp_read32(ulp_sdi_report_d_rx_len);
    status.d_timeout_count = ulp_read32(ulp_sdi_report_d_timeout_count);
    status.d_false_start_count = ulp_read32(ulp_sdi_report_d_false_start_count);
    status.d_byte_count = ulp_read32(ulp_sdi_report_d_byte_count);
    const size_t m_preview_len = std::min<size_t>(sizeof(status.m_preview), static_cast<size_t>(status.m_rx_len));
    const size_t d_preview_len = std::min<size_t>(sizeof(status.d_preview), static_cast<size_t>(status.d_rx_len));
    for (size_t i = 0; i < m_preview_len; i++) {
        status.m_preview[i] = static_cast<uint8_t>(ulp_read32(ulp_sdi_m_rx_buf[i]));
    }
    for (size_t i = 0; i < d_preview_len; i++) {
        status.d_preview[i] = static_cast<uint8_t>(ulp_read32(ulp_sdi_d_rx_buf[i]));
    }
    status.last_direction = ulp_read32(ulp_sdi_report_direction);
    status.last_strength_centi = ulp_read32(ulp_sdi_report_strength_centi);
    return status;
}

}  // namespace ulp_sdi_wind
