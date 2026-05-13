#pragma once

#include <cstdint>
#include <vector>

#include "common/wind_reading.h"

namespace ulp_sdi_wind {

struct DebugStatus {
  bool initialized = false;
  uint32_t phase = 0;
  uint32_t sequence = 0;
  uint32_t sample_sequence = 0;
  uint32_t parse_status = 0;
  uint32_t sample_count = 0;
  uint32_t m_status = 0;
  uint32_t m_rx_len = 0;
  uint32_t m_timeout_count = 0;
  uint32_t m_false_start_count = 0;
  uint32_t m_byte_count = 0;
  uint32_t d_status = 0;
  uint32_t d_rx_len = 0;
  uint32_t d_timeout_count = 0;
  uint32_t d_false_start_count = 0;
  uint32_t d_byte_count = 0;
  uint8_t m_preview[32] = {};
  uint8_t d_preview[32] = {};
  uint32_t last_direction = 0;
  uint32_t last_strength_centi = 0;
};

bool init();
bool attach_shared_state();
void stop();
void tick(uint32_t elapsed_ms);
bool read_latest(WindReading& out);
size_t read_recent_samples(std::vector<WindReading>& out, size_t max_samples);
bool check_stuck();
uint32_t phase();
DebugStatus debug_status();

}  // namespace ulp_sdi_wind
