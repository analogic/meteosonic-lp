#ifndef ULP_SDI_H
#define ULP_SDI_H

#include <stdint.h>

#define SDI_RX_BUF_LEN 32
#define SDI_SAMPLE_BUF_LEN 120

enum {
    SDI_STATUS_IDLE = 0,
    SDI_STATUS_OK = 1,
    SDI_STATUS_TIMEOUT = 2,
    SDI_STATUS_FRAMING = 3,
};

extern volatile uint32_t sdi_cycles_per_us_q10;
extern volatile uint32_t sdi_phase;
extern volatile uint32_t sdi_recv_debug_timeout_count;
extern volatile uint32_t sdi_recv_debug_false_start_count;
extern volatile uint32_t sdi_recv_debug_byte_count;

void sdi_init(void);
void sdi_refresh_timing(void);
void sdi_send_cmd(const uint8_t *cmd, int cmd_len);
int sdi_recv_line(volatile uint32_t *buf, volatile uint32_t *len);
void sdi_set_bus_idle(void);
void sdi_wait_for_next_cycle(uint32_t loop_start);

#endif
