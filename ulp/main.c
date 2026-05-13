#include <stdint.h>

#include "ulp_riscv_utils.h"

#include "sdi.h"

volatile uint32_t sdi_status = SDI_STATUS_IDLE;
volatile uint32_t sdi_sequence = 0;
volatile uint32_t sdi_phase = 0;
volatile uint32_t sdi_parse_status = SDI_STATUS_IDLE;
volatile uint32_t sdi_report_sequence = 0;
volatile uint32_t sdi_report_parse_status = SDI_STATUS_IDLE;
volatile uint32_t sdi_report_direction = 0;
volatile uint32_t sdi_report_strength_centi = 0;
volatile uint32_t sdi_report_sample_count = 0;
volatile uint32_t sdi_report_m_status = SDI_STATUS_IDLE;
volatile uint32_t sdi_report_m_rx_len = 0;
volatile uint32_t sdi_report_m_timeout_count = 0;
volatile uint32_t sdi_report_m_false_start_count = 0;
volatile uint32_t sdi_report_m_byte_count = 0;
volatile uint32_t sdi_report_d_status = SDI_STATUS_IDLE;
volatile uint32_t sdi_report_d_rx_len = 0;
volatile uint32_t sdi_report_d_timeout_count = 0;
volatile uint32_t sdi_report_d_false_start_count = 0;
volatile uint32_t sdi_report_d_byte_count = 0;
volatile uint32_t sdi_last_direction = 0;
volatile uint32_t sdi_last_strength_centi = 0;
volatile uint32_t sdi_sample_sequence = 0;
volatile uint32_t sdi_report_sample_sequence = 0;
volatile uint32_t sdi_sample_head = 0;
volatile uint32_t sdi_sample_count = 0;
volatile uint32_t sdi_direction_samples[SDI_SAMPLE_BUF_LEN];
volatile uint32_t sdi_strength_centi_samples[SDI_SAMPLE_BUF_LEN];
volatile uint32_t sdi_m_status = SDI_STATUS_IDLE;
volatile uint32_t sdi_m_rx_len = 0;
volatile uint32_t sdi_m_rx_buf[SDI_RX_BUF_LEN];
volatile uint32_t sdi_d_status = SDI_STATUS_IDLE;
volatile uint32_t sdi_d_rx_len = 0;
volatile uint32_t sdi_d_rx_buf[SDI_RX_BUF_LEN];

static const uint8_t cmd_m[] = { '0', 'M', '!' };
static const uint8_t cmd_d0[] = { '0', 'D', '0', '!' };

static inline int is_digit(uint32_t ch)
{
    return ch >= '0' && ch <= '9';
}

static int parse_uint(volatile uint32_t *buf, int start, int end, uint32_t *value)
{
    uint32_t parsed = 0;

    if (start >= end) {
        return 0;
    }

    for (int i = start; i < end; i++) {
        if (!is_digit(buf[i])) {
            return 0;
        }
        parsed = parsed * 10U + (buf[i] - '0');
    }

    *value = parsed;
    return 1;
}

static int parse_fixed_centi(volatile uint32_t *buf, int start, int end, uint32_t *value)
{
    uint32_t whole = 0;
    uint32_t frac = 0;
    uint32_t frac_digits = 0;
    int seen_dot = 0;

    if (start >= end) {
        return 0;
    }

    for (int i = start; i < end; i++) {
        uint32_t ch = buf[i];
        if (ch == '.') {
            if (seen_dot) {
                return 0;
            }
            seen_dot = 1;
            continue;
        }
        if (!is_digit(ch)) {
            return 0;
        }
        if (!seen_dot) {
            whole = whole * 10U + (ch - '0');
        } else if (frac_digits < 2) {
            frac = frac * 10U + (ch - '0');
            frac_digits++;
        }
    }

    if (frac_digits == 1) {
        frac *= 10U;
    }

    *value = whole * 100U + frac;
    return 1;
}

static void push_sample(uint32_t direction, uint32_t strength_centi)
{
    uint32_t head = sdi_sample_head;

    sdi_sample_sequence++;
    sdi_direction_samples[head] = direction;
    sdi_strength_centi_samples[head] = strength_centi;
    sdi_last_direction = direction;
    sdi_last_strength_centi = strength_centi;
    sdi_sample_head = (head + 1U) % SDI_SAMPLE_BUF_LEN;
    if (sdi_sample_count < SDI_SAMPLE_BUF_LEN) {
        sdi_sample_count++;
    }
    sdi_sample_sequence++;
}

static int parse_d0_fields(uint32_t *direction, uint32_t *strength_centi)
{
    int start = 0;
    int field_index = 0;
    int len = (int)sdi_d_rx_len;
    int have_direction = 0;
    int have_strength = 0;

    for (int i = 0; i <= len; i++) {
        if (i < len && sdi_d_rx_buf[i] != '+') {
            continue;
        }

        if (field_index == 1) {
            have_direction = parse_uint(sdi_d_rx_buf, start, i, direction);
        } else if (field_index == 2) {
            have_strength = parse_fixed_centi(sdi_d_rx_buf, start, i, strength_centi);
            break;
        }

        field_index++;
        start = i + 1;
    }

    if (!have_direction || !have_strength) {
        return 0;
    }
    return 1;
}

static void init_state(void)
{
    for (int i = 0; i < SDI_SAMPLE_BUF_LEN; i++) {
        sdi_direction_samples[i] = 0;
        sdi_strength_centi_samples[i] = 0;
    }
    sdi_parse_status = SDI_STATUS_IDLE;
    sdi_report_sequence = 0;
    sdi_report_parse_status = SDI_STATUS_IDLE;
    sdi_report_direction = 0;
    sdi_report_strength_centi = 0;
    sdi_report_sample_count = 0;
    sdi_report_m_status = SDI_STATUS_IDLE;
    sdi_report_m_rx_len = 0;
    sdi_report_m_timeout_count = 0;
    sdi_report_m_false_start_count = 0;
    sdi_report_m_byte_count = 0;
    sdi_report_d_status = SDI_STATUS_IDLE;
    sdi_report_d_rx_len = 0;
    sdi_report_d_timeout_count = 0;
    sdi_report_d_false_start_count = 0;
    sdi_report_d_byte_count = 0;
    sdi_last_direction = 0;
    sdi_last_strength_centi = 0;
    sdi_sample_sequence = 0;
    sdi_report_sample_sequence = 0;
    sdi_sample_head = 0;
    sdi_sample_count = 0;
}

static void reset_cycle_state(void)
{
    sdi_status = SDI_STATUS_IDLE;
    sdi_parse_status = SDI_STATUS_IDLE;
    sdi_m_status = SDI_STATUS_IDLE;
    sdi_m_rx_len = 0;
    sdi_d_status = SDI_STATUS_IDLE;
    sdi_d_rx_len = 0;
}

static void send_receive(const uint8_t *cmd, int cmd_len,
                         volatile uint32_t *rx_buf, volatile uint32_t *rx_len,
                         volatile uint32_t *status, uint32_t send_phase, uint32_t recv_phase)
{
    sdi_phase = send_phase;
    sdi_send_cmd(cmd, cmd_len);

    sdi_phase = recv_phase;
    if (sdi_recv_line(rx_buf, rx_len) > 0) {
        *status = SDI_STATUS_OK;
    } else {
        *status = SDI_STATUS_TIMEOUT;
    }

    if (send_phase == 5) {
        sdi_report_m_timeout_count = sdi_recv_debug_timeout_count;
        sdi_report_m_false_start_count = sdi_recv_debug_false_start_count;
        sdi_report_m_byte_count = sdi_recv_debug_byte_count;
    } else if (send_phase == 7) {
        sdi_report_d_timeout_count = sdi_recv_debug_timeout_count;
        sdi_report_d_false_start_count = sdi_recv_debug_false_start_count;
        sdi_report_d_byte_count = sdi_recv_debug_byte_count;
    }
}

static void finish_cycle(void)
{
    uint32_t direction = 0;
    uint32_t strength_centi = 0;

    if (sdi_d_status == SDI_STATUS_OK) {
        sdi_parse_status = parse_d0_fields(&direction, &strength_centi) ? SDI_STATUS_OK : SDI_STATUS_FRAMING;
        if (sdi_parse_status != SDI_STATUS_OK) {
            direction = 0;
            strength_centi = 0;
        }
    } else {
        direction = 0;
        strength_centi = 0;
    }

    push_sample(direction, strength_centi);

    sdi_status = ((sdi_m_status == SDI_STATUS_OK) && (sdi_d_status == SDI_STATUS_OK))
        ? SDI_STATUS_OK
        : SDI_STATUS_TIMEOUT;

    sdi_report_parse_status = sdi_parse_status;
    sdi_report_direction = sdi_last_direction;
    sdi_report_strength_centi = sdi_last_strength_centi;
    sdi_report_sample_sequence = sdi_sample_sequence;
    sdi_report_sample_count = sdi_sample_count;
    sdi_report_m_status = sdi_m_status;
    sdi_report_m_rx_len = sdi_m_rx_len;
    sdi_report_d_status = sdi_d_status;
    sdi_report_d_rx_len = sdi_d_rx_len;
    sdi_phase = 9;
    sdi_sequence++;
    sdi_report_sequence = sdi_sequence;
    sdi_set_bus_idle();
    sdi_phase = 10;
}

static void run_cycle(void)
{
    reset_cycle_state();
    sdi_phase = 4;

    send_receive(cmd_m, sizeof(cmd_m),
                 sdi_m_rx_buf, &sdi_m_rx_len, &sdi_m_status,
                 5, 6);

    send_receive(cmd_d0, sizeof(cmd_d0),
                 sdi_d_rx_buf, &sdi_d_rx_len, &sdi_d_status,
                 7, 8);

    finish_cycle();
}

int main(void)
{
    sdi_phase = 1;
    sdi_init();
    init_state();
    sdi_phase = 2;
    sdi_refresh_timing();
    sdi_phase = 3;

    while (1) {
        uint32_t loop_start = ULP_RISCV_GET_CCOUNT();
        sdi_refresh_timing();
        run_cycle();
        sdi_wait_for_next_cycle(loop_start);
    }
}
