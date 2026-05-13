#include "sdi.h"

#include "ulp_riscv_gpio.h"
#include "ulp_riscv_utils.h"

#define TX_GPIO 1
#define OE_GPIO 2
#define RX_GPIO 16

#define BREAK_US 12000
#define MARK_US 8333
#define BIT_US 833
#define RESPONSE_TIMEOUT_MS 150

#define CYCLES_PER_US_Q10_DEFAULT ((uint32_t)(ULP_RISCV_CYCLES_PER_US * 1024.0f))

volatile uint32_t sdi_cycles_per_us_q10 = CYCLES_PER_US_Q10_DEFAULT;
volatile uint32_t sdi_recv_debug_timeout_count = 0;
volatile uint32_t sdi_recv_debug_false_start_count = 0;
volatile uint32_t sdi_recv_debug_byte_count = 0;

static uint32_t sdi_initialized = 0;
static uint32_t g_cycles_per_us_q10 = CYCLES_PER_US_Q10_DEFAULT;
static uint32_t g_break_cycles = 0;
static uint32_t g_mark_cycles = 0;
static uint32_t g_bit_cycles = 0;
static uint32_t g_quarter_bit_cycles = 0;
static uint32_t g_half_bit_cycles = 0;
static uint32_t g_sample_span_cycles = 0;
static uint32_t g_response_timeout_cycles = 0;
static uint32_t g_loop_period_cycles = 0;
static uint32_t g_applied_cycles_per_us_q10 = 0;

static inline uint32_t cycles_from_us(uint32_t us)
{
    return (uint32_t)(((uint64_t)us * g_cycles_per_us_q10 + 512U) >> 10);
}

static inline void clear_rx_buffer(volatile uint32_t *buf, volatile uint32_t *len)
{
    for (int i = 0; i < SDI_RX_BUF_LEN; i++) {
        buf[i] = 0;
    }
    *len = 0;
}

static inline void bus_drive(int high)
{
    ulp_riscv_gpio_output_level(TX_GPIO, high);
    ulp_riscv_gpio_output_level(OE_GPIO, 0);
}

static inline void rx_mode(void)
{
    ulp_riscv_gpio_output_level(OE_GPIO, 1);
}

static inline int rx_level(void)
{
    return ulp_riscv_gpio_get_level(RX_GPIO);
}

/* TX levels are electrically inverted vs SDI-12 spec for this hardware */
static void sdi_send_byte(uint8_t ch)
{
    int ones = 0;

    bus_drive(1);
    ulp_riscv_delay_cycles(g_bit_cycles);

    for (int i = 0; i < 7; i++) {
        int bit = !((ch >> i) & 1);
        if (bit) {
            ones++;
        }
        bus_drive(bit);
        ulp_riscv_delay_cycles(g_bit_cycles);
    }

    bus_drive((ones & 1) ? 1 : 0);
    ulp_riscv_delay_cycles(g_bit_cycles);

    bus_drive(0);
    ulp_riscv_delay_cycles(g_bit_cycles);
}

static int sdi_recv_byte(uint8_t *out, uint32_t timeout_cycles)
{
    uint32_t start_wait = ULP_RISCV_GET_CCOUNT();
    while (rx_level() != 0) {
        if ((ULP_RISCV_GET_CCOUNT() - start_wait) > timeout_cycles) {
            return 0;
        }
    }

    uint32_t t0 = ULP_RISCV_GET_CCOUNT();
    uint32_t sample_deadline = t0 + timeout_cycles;
    uint8_t ch = 0;

    while ((ULP_RISCV_GET_CCOUNT() - t0) < g_quarter_bit_cycles) {
        if ((int32_t)(ULP_RISCV_GET_CCOUNT() - sample_deadline) > 0) {
            return 0;
        }
    }
    if (rx_level() != 0) {
        return -1;
    }

    ULP_RISCV_ENTER_CRITICAL();

    for (int i = 0; i < 7; i++) {
        uint32_t center = g_bit_cycles + g_half_bit_cycles + ((uint32_t)i * g_bit_cycles);

        while ((ULP_RISCV_GET_CCOUNT() - t0) < (center - g_sample_span_cycles)) {
            if ((int32_t)(ULP_RISCV_GET_CCOUNT() - sample_deadline) > 0) {
                ULP_RISCV_EXIT_CRITICAL();
                return 0;
            }
        }

        int ones = rx_level();

        while ((ULP_RISCV_GET_CCOUNT() - t0) < center) {
            if ((int32_t)(ULP_RISCV_GET_CCOUNT() - sample_deadline) > 0) {
                ULP_RISCV_EXIT_CRITICAL();
                return 0;
            }
        }
        ones += rx_level();

        while ((ULP_RISCV_GET_CCOUNT() - t0) < (center + g_sample_span_cycles)) {
            if ((int32_t)(ULP_RISCV_GET_CCOUNT() - sample_deadline) > 0) {
                ULP_RISCV_EXIT_CRITICAL();
                return 0;
            }
        }
        ones += rx_level();

        if (ones >= 2) {
            ch |= (1U << i);
        }
    }

    while ((ULP_RISCV_GET_CCOUNT() - t0) < ((uint32_t)10U * g_bit_cycles)) {
        if ((int32_t)(ULP_RISCV_GET_CCOUNT() - sample_deadline) > 0) {
            ULP_RISCV_EXIT_CRITICAL();
            return 0;
        }
    }
    ULP_RISCV_EXIT_CRITICAL();

    *out = ch;
    return 1;
}

void sdi_init(void)
{
    if (sdi_initialized) {
        return;
    }

    ulp_riscv_gpio_init(TX_GPIO);
    ulp_riscv_gpio_output_enable(TX_GPIO);
    ulp_riscv_gpio_input_disable(TX_GPIO);

    ulp_riscv_gpio_init(OE_GPIO);
    ulp_riscv_gpio_output_enable(OE_GPIO);
    ulp_riscv_gpio_input_disable(OE_GPIO);

    ulp_riscv_gpio_init(RX_GPIO);
    ulp_riscv_gpio_output_disable(RX_GPIO);
    ulp_riscv_gpio_input_enable(RX_GPIO);
    ulp_riscv_gpio_pullup(RX_GPIO);
    ulp_riscv_gpio_pulldown_disable(RX_GPIO);

    bus_drive(1);
    sdi_initialized = 1;
}

void sdi_refresh_timing(void)
{
    uint32_t q10 = sdi_cycles_per_us_q10;
    if (q10 == 0) {
        q10 = CYCLES_PER_US_Q10_DEFAULT;
        sdi_cycles_per_us_q10 = q10;
    }
    if (q10 == g_applied_cycles_per_us_q10) {
        return;
    }

    g_cycles_per_us_q10 = q10;
    g_break_cycles = cycles_from_us(BREAK_US);
    g_mark_cycles = cycles_from_us(MARK_US);
    g_bit_cycles = cycles_from_us(BIT_US);
    g_quarter_bit_cycles = g_bit_cycles / 4;
    g_half_bit_cycles = g_bit_cycles / 2;
    g_sample_span_cycles = g_bit_cycles / 6;
    g_response_timeout_cycles = cycles_from_us(RESPONSE_TIMEOUT_MS * 1000U);
    g_loop_period_cycles = cycles_from_us(1000000U);
    g_applied_cycles_per_us_q10 = q10;
}

void sdi_send_cmd(const uint8_t *cmd, int cmd_len)
{
    bus_drive(1);
    ulp_riscv_delay_cycles(g_bit_cycles);
    bus_drive(0);
    ulp_riscv_delay_cycles(g_break_cycles);
    bus_drive(1);
    ulp_riscv_delay_cycles(g_mark_cycles);
    bus_drive(0);
    ulp_riscv_delay_cycles(g_bit_cycles);

    for (int i = 0; i < cmd_len; i++) {
        sdi_send_byte(cmd[i]);
    }

    rx_mode();
}

int sdi_recv_line(volatile uint32_t *buf, volatile uint32_t *len)
{
    uint32_t idle_start = ULP_RISCV_GET_CCOUNT();
    int n = 0;

    clear_rx_buffer(buf, len);
    sdi_recv_debug_timeout_count = 0;
    sdi_recv_debug_false_start_count = 0;
    sdi_recv_debug_byte_count = 0;

    while (n < (SDI_RX_BUF_LEN - 1)) {
        uint32_t now = ULP_RISCV_GET_CCOUNT();
        uint32_t waited_cycles = now - idle_start;
        if (waited_cycles > g_response_timeout_cycles) {
            break;
        }

        uint8_t b = 0;
        int rc = sdi_recv_byte(&b, g_response_timeout_cycles - waited_cycles);
        if (rc == 0) {
            sdi_recv_debug_timeout_count++;
            break;
        }
        if (rc < 0) {
            sdi_recv_debug_false_start_count++;
            continue;
        }

        buf[n++] = b;
        sdi_recv_debug_byte_count++;
        idle_start = ULP_RISCV_GET_CCOUNT();
        if (n >= 2 && buf[n - 2] == '\r' && buf[n - 1] == '\n') {
            break;
        }
    }

    while (n > 0 && buf[n - 1] <= ' ') {
        buf[--n] = 0;
    }

    *len = (uint32_t)n;
    return n;
}

void sdi_set_bus_idle(void)
{
    bus_drive(1);
}

void sdi_wait_for_next_cycle(uint32_t loop_start)
{
    while ((ULP_RISCV_GET_CCOUNT() - loop_start) < g_loop_period_cycles) {
    }
}
