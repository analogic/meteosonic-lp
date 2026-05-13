// Simple synchronous modem driver for SIM7080G.

#include "modem.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define TAG "MODEM"

#define CMD_DEFAULT_TIMEOUT_MS 2000
#define CMD_QUEUE_LEN 24
#define LINE_MAX_LEN 256
#define INIT_TIMEOUT_MS 180000
#define DTR_SETTLE_MS 10

typedef struct {
    char line[LINE_MAX_LEN];
} modem_line_t;

typedef struct {
    QueueHandle_t line_queue;
    SemaphoreHandle_t uart_lock;
    TaskHandle_t rx_task;
} modem_io_t;

static modem_config_t g_cfg = {0};
static modem_io_t g_io = {0};
static volatile bool g_coap_restart = false;
static bool g_coap_reinit_allowed = true;
static bool g_publish_pdp_check_enabled = true;
static int g_coap_publish_failures = 0;
static uint64_t g_init_start_ms = 0;
static bool g_sleep_enabled = false;
static bool g_modem_is_sleeping = false;
static bool g_uart_started = false;
static bool g_pdp_known_active = false;
static int g_dtr_level = -1;

static uint64_t now_ms(void) {
    return (uint64_t)(esp_timer_get_time() / 1000);
}

static bool is_urc(const char *line) {
    return strncmp(line, "+CCOAP", 6) == 0 ||
           strncmp(line, "+APP PDP: 0,DEACTIVE", 19) == 0;
}

static bool is_error_line(const char *line) {
    if (!line) {
        return false;
    }
    return strcmp(line, "ERROR") == 0 ||
           strstr(line, "+CME ERROR") != NULL ||
           strstr(line, "+CMS ERROR") != NULL;
}

static void handle_urc(const char *line) {
    if (strncmp(line, "+CCOAP", 6) == 0) {
        ESP_LOGW(TAG, "URC: CoAP event (%s)", line);
        g_coap_restart = true;
    } else if (strncmp(line, "+APP PDP: 0,DEACTIVE", 19) == 0) {
        ESP_LOGW(TAG, "URC: PDP deactivated -> restart CoAP init");
        g_pdp_known_active = false;
        g_coap_restart = true;
    } else {
        ESP_LOGI(TAG, "URC: %s", line);
    }
}

static void modem_flush_input(void) {
    if (!g_io.line_queue) {
        uart_flush_input(g_cfg.uart_num);
        return;
    }

    modem_line_t item;
    for (int i = 0; i < CMD_QUEUE_LEN; ++i) {
        if (xQueueReceive(g_io.line_queue, &item, 0) != pdPASS) {
            break;
        }
    }
}

static void modem_push_line(const char *line) {
    if (!g_io.line_queue || !line || line[0] == '\0') {
        return;
    }

    modem_line_t item = {0};
    strlcpy(item.line, line, sizeof(item.line));
    if (xQueueSend(g_io.line_queue, &item, 0) != pdPASS) {
        ESP_LOGW(TAG, "line queue full, dropping: %s", line);
    }
}

static bool modem_uart_write_exact(const char *data, size_t len, const char *label) {
    if (!data || len == 0) {
        return true;
    }

    int written = uart_write_bytes(g_cfg.uart_num, data, len);
    if (written < 0) {
        ESP_LOGE(TAG, "UART write failed for %s", label ? label : "data");
        return false;
    }
    if ((size_t)written != len) {
        ESP_LOGE(TAG, "UART short write for %s: wrote %d of %u bytes",
                 label ? label : "data", written, (unsigned)len);
        return false;
    }

    esp_err_t wait_err = uart_wait_tx_done(g_cfg.uart_num, pdMS_TO_TICKS(1000));
    if (wait_err != ESP_OK) {
        ESP_LOGE(TAG, "UART TX wait failed for %s: %s",
                 label ? label : "data", esp_err_to_name(wait_err));
        return false;
    }

    return true;
}

static void modem_send_line(const char *line) {
    xSemaphoreTake(g_io.uart_lock, portMAX_DELAY);
    bool ok = modem_uart_write_exact(line, strlen(line), line);
    ok = modem_uart_write_exact("\r\n", 2, "CRLF") && ok;
    xSemaphoreGive(g_io.uart_lock);
    ESP_LOGI(TAG, ">> %s%s", line, ok ? "" : " [UART TX FAILED]");
}

static void modem_pwrkey_pulse(void) {
    if (g_cfg.power_gpio < 0) {
        ESP_LOGW(TAG, "PWRKEY pulse skipped: power GPIO not configured");
        return;
    }

    const int pulse_ms = g_cfg.power_pulse_ms > 0 ? g_cfg.power_pulse_ms : 1500;
    ESP_LOGI(TAG, "PWRKEY pulse on GPIO %d for %d ms", g_cfg.power_gpio, pulse_ms);
    gpio_set_level((gpio_num_t)g_cfg.power_gpio, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level((gpio_num_t)g_cfg.power_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(pulse_ms));
    gpio_set_level((gpio_num_t)g_cfg.power_gpio, 0);
}

static void modem_set_dtr_level(int level) {
#ifdef PIN_NB_DTR
    const int normalized = level ? 1 : 0;
    if (g_dtr_level == normalized) {
        return;
    }
    gpio_set_level((gpio_num_t)PIN_NB_DTR, normalized);
    g_dtr_level = normalized;
    ESP_LOGI(TAG, "DTR set %s on GPIO %d", normalized ? "HIGH" : "LOW", PIN_NB_DTR);
#else
    (void)level;
#endif
}

static void modem_hold_dtr_level(bool enabled) {
#ifdef PIN_NB_DTR
    if (enabled) {
        ESP_ERROR_CHECK(gpio_hold_en((gpio_num_t)PIN_NB_DTR));
        gpio_deep_sleep_hold_en();
        ESP_LOGI(TAG, "DTR GPIO hold enabled");
    } else {
        gpio_hold_dis((gpio_num_t)PIN_NB_DTR);
        gpio_deep_sleep_hold_dis();
        ESP_LOGI(TAG, "DTR GPIO hold disabled");
    }
#else
    (void)enabled;
#endif
}

static void modem_rx_task(void *arg) {
    (void)arg;

    char buf[LINE_MAX_LEN] = {0};
    size_t pos = 0;
    uint8_t rx[64];

    while (true) {
        int len = uart_read_bytes(g_cfg.uart_num, rx, sizeof(rx), pdMS_TO_TICKS(50));
        if (len <= 0) {
            continue;
        }

        for (int i = 0; i < len; ++i) {
            char ch = (char)rx[i];
            if (ch == '\r') {
                continue;
            }
            if (ch == '\n') {
                if (pos > 0) {
                    buf[pos] = '\0';
                    modem_push_line(buf);
                    pos = 0;
                    buf[0] = '\0';
                }
                continue;
            }
            if (ch == '>' && pos == 0) {
                modem_push_line(">");
                continue;
            }
            if (pos + 1 < sizeof(buf)) {
                buf[pos++] = ch;
            }
        }
    }
}

static bool modem_read_line(char *out, size_t out_size, int timeout_ms) {
    if (!out || out_size == 0) {
        return false;
    }
    out[0] = '\0';

    if (!g_io.line_queue) {
        return false;
    }

    modem_line_t item = {0};
    if (xQueueReceive(g_io.line_queue, &item, pdMS_TO_TICKS(timeout_ms)) != pdPASS) {
        return false;
    }

    strlcpy(out, item.line, out_size);
    return true;
}

static bool modem_expect_contains(const char *token, int timeout_ms) {
    uint64_t deadline = now_ms() + timeout_ms;
    char line[LINE_MAX_LEN];

    while (now_ms() < deadline) {
        int wait_ms = (int)(deadline - now_ms());
        if (wait_ms < 1) {
            wait_ms = 1;
        }
        if (!modem_read_line(line, sizeof(line), wait_ms)) {
            continue;
        }
        const bool line_is_urc = is_urc(line);
        if (line_is_urc) {
            handle_urc(line);
        } else {
            ESP_LOGI(TAG, "<< %s", line);
        }
        if (is_error_line(line)) {
            return false;
        }
        if (strstr(line, token) != NULL) {
            return true;
        }
        if (line_is_urc) {
            continue;
        }
    }

    return false;
}

static bool modem_expect_sequence(const char *first, const char *second, int timeout_ms) {
    uint64_t deadline = now_ms() + timeout_ms;
    bool got_first = false;
    bool got_second = false;
    char line[LINE_MAX_LEN];

    while (now_ms() < deadline) {
        int wait_ms = (int)(deadline - now_ms());
        if (wait_ms < 1) {
            wait_ms = 1;
        }
        if (!modem_read_line(line, sizeof(line), wait_ms)) {
            continue;
        }
        const bool line_is_urc = is_urc(line);
        if (line_is_urc) {
            handle_urc(line);
        } else {
            ESP_LOGI(TAG, "<< %s", line);
        }
        if (is_error_line(line)) {
            return false;
        }
        if (strstr(line, first)) {
            got_first = true;
        }
        if (strstr(line, second)) {
            got_second = true;
        }
        if (got_first && got_second) {
            return true;
        }
        if (line_is_urc) {
            continue;
        }
    }

    return false;
}

static bool modem_cmd_expect(const char *cmd, const char *expect, int timeout_ms) {
    modem_send_line(cmd);
    return modem_expect_contains(expect, timeout_ms);
}

static bool modem_cmd_expect_sequence(const char *cmd, const char *first, const char *second, int timeout_ms) {
    modem_send_line(cmd);
    return modem_expect_sequence(first, second, timeout_ms);
}

static void check_init_timeout(void) {
    if (now_ms() - g_init_start_ms <= INIT_TIMEOUT_MS) {
        return;
    }

    ESP_LOGW(TAG, "Init exceeded 3 minutes, restarting modem");
    g_init_start_ms = now_ms();
    if (g_cfg.reset_method == MODEM_RESET_GPIO_PULSE) {
        modem_pwrkey_pulse();
    } else {
        modem_cmd_expect("AT+NRB", "OK", 60000);
    }
}

static void retry_cmd_expect(const char *cmd, const char *expect, int timeout_ms, int delay_ms, const char *warn_msg) {
    while (!modem_cmd_expect(cmd, expect, timeout_ms)) {
        ESP_LOGW(TAG, "%s", warn_msg);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        check_init_timeout();
    }
}

static void retry_cmd_expect_seq(const char *cmd, const char *first, const char *second, int timeout_ms, int delay_ms, const char *warn_msg) {
    while (!modem_cmd_expect_sequence(cmd, first, second, timeout_ms)) {
        ESP_LOGW(TAG, "%s", warn_msg);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        check_init_timeout();
    }
}

static int parse_int_after(const char *line, const char *prefix) {
    const char *p = strstr(line, prefix);
    if (!p) {
        return -1;
    }
    p += strlen(prefix);
    return atoi(p);
}

static int parse_cereg_stat(const char *line) {
    int n = -1;
    int stat = -1;
    if (sscanf(line, "+CEREG: %d,%d", &n, &stat) == 2) {
        return stat;
    }
    return -1;
}

static bool parse_cgnsinf_fix(const char *line, modem_gnss_fix_t *fix) {
    if (!line || !fix) {
        return false;
    }

    const char *prefix = strstr(line, "+CGNSINF:");
    if (!prefix) {
        return false;
    }

    prefix += strlen("+CGNSINF:");
    while (*prefix == ' ') {
        prefix++;
    }

    char tmp[LINE_MAX_LEN];
    strlcpy(tmp, prefix, sizeof(tmp));

    char *fields[5] = {0};
    size_t field_count = 0;
    char *field_start = tmp;
    for (char *p = tmp; *p != '\0' && field_count < 5; ++p) {
        if (*p == ',') {
            *p = '\0';
            fields[field_count++] = field_start;
            field_start = p + 1;
        }
    }
    if (field_count < 5) {
        fields[field_count++] = field_start;
    }
    if (field_count < 5) {
        return false;
    }

    const int run_status = atoi(fields[0]);
    const int fix_status = fields[1][0] != '\0' ? atoi(fields[1]) : 0;
    const char *utc = fields[2];
    const char *lat = fields[3];
    const char *lon = fields[4];

    if (run_status != 1 || lat[0] == '\0' || lon[0] == '\0') {
        return false;
    }

    fix->valid = true;
    strlcpy(fix->utc, utc, sizeof(fix->utc));
    fix->latitude = strtof(lat, NULL);
    fix->longitude = strtof(lon, NULL);
    ESP_LOGI(TAG, "Parsed GNSS fix: run=%d fix=%d utc='%s' lat=%f lon=%f",
             run_status, fix_status, utc, (double)fix->latitude, (double)fix->longitude);
    return true;
}

static bool wait_for_signal(void) {
    char line[LINE_MAX_LEN];
    while (true) {
        check_init_timeout();
        modem_send_line("AT+CSQ");
        uint64_t deadline = now_ms() + 5000;
        int rssi = -1;
        bool ok = false;

        while (now_ms() < deadline) {
          if (!modem_read_line(line, sizeof(line), 500)) {
              continue;
          }
          if (is_urc(line)) {
              handle_urc(line);
              continue;
          }
          ESP_LOGI(TAG, "<< %s", line);
          if (strncmp(line, "+CSQ:", 5) == 0) {
              rssi = parse_int_after(line, "+CSQ: ");
          } else if (strcmp(line, "OK") == 0) {
              ok = true;
              break;
          } else if (strcmp(line, "ERROR") == 0) {
              break;
          }
        }

        if (ok && rssi >= 0 && rssi != 99) {
            ESP_LOGI(TAG, "Signal RSSI raw: %d", rssi);
            return true;
        }

        ESP_LOGW(TAG, "Waiting for usable signal (rssi=%d)", rssi);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static bool wait_for_cereg(void) {
    char line[LINE_MAX_LEN];
    while (true) {
        check_init_timeout();
        modem_send_line("AT+CEREG?");
        uint64_t deadline = now_ms() + 5000;
        int stat = -1;
        bool ok = false;

        while (now_ms() < deadline) {
            if (!modem_read_line(line, sizeof(line), 500)) {
                continue;
            }
            if (is_urc(line)) {
                handle_urc(line);
                continue;
            }
            ESP_LOGI(TAG, "<< %s", line);
            if (strncmp(line, "+CEREG:", 7) == 0) {
                stat = parse_cereg_stat(line);
            } else if (strcmp(line, "OK") == 0) {
                ok = true;
                break;
            } else if (strcmp(line, "ERROR") == 0) {
                break;
            }
        }

        if (ok && (stat == 1 || stat >= 5)) {
            ESP_LOGI(TAG, "Registered, stat=%d", stat);
            return true;
        }

        ESP_LOGW(TAG, "Waiting for registration, stat=%d", stat);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static bool wait_for_service(void) {
    char line[LINE_MAX_LEN];
    while (true) {
        check_init_timeout();
        modem_send_line("AT+CPSI?");
        uint64_t deadline = now_ms() + 9000;
        bool ok = false;
        bool no_service = false;

        while (now_ms() < deadline) {
            if (!modem_read_line(line, sizeof(line), 500)) {
                continue;
            }
            if (is_urc(line)) {
                handle_urc(line);
                continue;
            }
            ESP_LOGI(TAG, "<< %s", line);
            if (strncmp(line, "+CPSI:", 6) == 0 && strstr(line, "NO SERVICE")) {
                no_service = true;
            } else if (strcmp(line, "OK") == 0) {
                ok = true;
                break;
            } else if (strcmp(line, "ERROR") == 0) {
                break;
            }
        }

        if (ok && !no_service) {
            return true;
        }

        ESP_LOGW(TAG, "Waiting for service...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static bool is_pdp_active(void) {
    modem_send_line("AT+CNACT?");

    uint64_t deadline = now_ms() + 5000;
    char line[LINE_MAX_LEN];
    bool active = false;
    bool got_ok = false;

    while (now_ms() < deadline) {
        int wait_ms = (int)(deadline - now_ms());
        if (wait_ms < 1) {
            wait_ms = 1;
        }
        if (!modem_read_line(line, sizeof(line), wait_ms)) {
            continue;
        }
        if (is_urc(line)) {
            handle_urc(line);
            continue;
        }
        ESP_LOGI(TAG, "<< %s", line);
        if (strncmp(line, "+CNACT:", 7) == 0 && strstr(line, "+CNACT: 0,1") != NULL) {
            active = true;
        } else if (strcmp(line, "OK") == 0) {
            got_ok = true;
            break;
        } else if (is_error_line(line)) {
            return false;
        }
    }

    g_pdp_known_active = got_ok && active;
    return g_pdp_known_active;
}

static bool ensure_pdp_active(void) {
    if (g_pdp_known_active) {
        ESP_LOGI(TAG, "PDP context already active");
        return true;
    }

    if (is_pdp_active()) {
        ESP_LOGI(TAG, "PDP context already active");
        return true;
    }

    char pdp[96];
    snprintf(pdp, sizeof(pdp), "AT+CGDCONT=1,\"IP\",\"%s\",\"0.0.0.0\",0,0", g_cfg.apn);
    if (!modem_cmd_expect(pdp, "OK", CMD_DEFAULT_TIMEOUT_MS)) {
        return false;
    }

    for (int attempt = 0; attempt < 3; ++attempt) {
        modem_cmd_expect("AT+CGACT=1,1", "OK", CMD_DEFAULT_TIMEOUT_MS);
        modem_cmd_expect("AT+CNACT=0,1", "OK", CMD_DEFAULT_TIMEOUT_MS);
        vTaskDelay(pdMS_TO_TICKS(150));
        if (is_pdp_active()) {
            g_pdp_known_active = true;
            return true;
        }
        ESP_LOGW(TAG, "PDP not active yet, retry %d/3", attempt + 1);
    }

    return false;
}

static void modem_basic_init(void) {
    g_init_start_ms = now_ms();
    int retry = 0;
    modem_pwrkey_pulse();

    while (!modem_cmd_expect("AT", "OK", CMD_DEFAULT_TIMEOUT_MS)) {
        ESP_LOGW(TAG, "AT failed, retrying...");
        if (retry++ > 10) {
            modem_pwrkey_pulse();
            retry = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        check_init_timeout();
    }

    retry = 0;
    while (!modem_cmd_expect("ATE0", "OK", CMD_DEFAULT_TIMEOUT_MS)) {
        ESP_LOGW(TAG, "ATE0 failed, retrying...");
        if (retry++ > 6) {
            modem_pwrkey_pulse();
            retry = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
        check_init_timeout();
    }

    // Keep DTR-controlled sleep disabled while the modem is active.
    g_sleep_enabled = false;

    modem_cmd_expect("AT+CMEE=2", "OK", CMD_DEFAULT_TIMEOUT_MS);
    // PSM is intentionally disabled; idle power is handled by ordinary DRX and DTR sleep.
    modem_cmd_expect("AT+CPSMS=0", "OK", 5000);
    retry_cmd_expect("AT+CFUN=1", "OK", 10000, 500, "CFUN=1 failed, retrying");
    retry_cmd_expect_seq("AT+CPIN?", "+CPIN: READY", "OK", 5000, 500, "Waiting for SIM ready");

    wait_for_signal();
    wait_for_cereg();
    wait_for_service();

    if (!ensure_pdp_active()) {
        ESP_LOGW(TAG, "PDP activation failed during basic init");
    }

    modem_flush_input();
    modem_send_line("AT+CGACT?");
    uint64_t deadline = now_ms() + CMD_DEFAULT_TIMEOUT_MS;
    char line[LINE_MAX_LEN];
    bool active = false;
    while (now_ms() < deadline) {
        if (!modem_read_line(line, sizeof(line), 200)) {
            continue;
        }
        if (is_urc(line)) {
            handle_urc(line);
            continue;
        }
        ESP_LOGI(TAG, "<< %s", line);
        if (strncmp(line, "+CGACT:", 7) == 0 && strstr(line, ",1")) {
            active = true;
        } else if (strcmp(line, "OK") == 0) {
            break;
        }
    }
    if (!active) {
        ESP_LOGW(TAG, "No active PDP context reported");
    }

    modem_cmd_expect("AT+CNETLIGHT=0", "OK", CMD_DEFAULT_TIMEOUT_MS);
}

static bool coap_init_sim7080(void) {
    modem_flush_input();

    if (!ensure_pdp_active()) {
        ESP_LOGW(TAG, "CoAP init aborted: PDP is not active");
        return false;
    }

    // CoAP stack init may report ERROR if already initialized; do not fail hard on it.
    modem_cmd_expect("AT+CCOAPINIT", "OK", 5000);

    char url[192];
    snprintf(url, sizeof(url), "coap://%s:%d", g_cfg.coap_server, g_cfg.coap_port);

    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+CCOAPURL=\"%s\"", url);
    if (!modem_cmd_expect(cmd, "OK", 5000)) {
        ESP_LOGW(TAG, "CCOAPURL setup failed for %s", url);
        return false;
    }

    return true;
}

static bool escape_at_quoted_arg(const char *in, char *out, size_t out_size) {
    if (!in || !out || out_size == 0) {
        return false;
    }

    size_t j = 0;
    for (size_t i = 0; in[i] != '\0'; ++i) {
        const char ch = in[i];
        if (ch == '"' || ch == '\\') {
            if (j + 2 >= out_size) {
                return false;
            }
            out[j++] = '\\';
            out[j++] = ch;
        } else {
            if (j + 1 >= out_size) {
                return false;
            }
            out[j++] = ch;
        }
    }
    out[j] = '\0';
    return true;
}

static bool coap_init_internal(void) {
    if (!coap_init_sim7080()) {
        return false;
    }

    g_coap_publish_failures = 0;
    g_coap_restart = false;
    const char *path = g_cfg.coap_path ? g_cfg.coap_path : "";
    while (*path == '/') {
        path++;
    }
    ESP_LOGI(TAG, "CoAP endpoint ready: coap://%s:%d/%s", g_cfg.coap_server, g_cfg.coap_port, path);
    return true;
}

static bool coap_publish_impl(const char *payload, int timeout_ms) {
    char cmd[512];
    char payload_escaped[320];
    const char *path = g_cfg.coap_path ? g_cfg.coap_path : "";
    while (*path == '/') {
        path++;
    }

    if (!escape_at_quoted_arg(payload, payload_escaped, sizeof(payload_escaped))) {
        ESP_LOGW(TAG, "Payload too long for CCOAPPARA");
        return false;
    }

    // Use CCOAPPARA/CCOAPACTION flow for broader SIM7080 firmware compatibility.
    snprintf(cmd,
             sizeof(cmd),
             "AT+CCOAPPARA=\"TYPE\",\"NON\",\"CODE\",1,uri-path,0,\"%s\",payload,0,\"%s\"",
             path,
             payload_escaped);
    if (!modem_cmd_expect(cmd, "OK", 8000)) {
        ESP_LOGW(TAG, "CCOAPPARA failed");
        return false;
    }

    if (!modem_cmd_expect_sequence("AT+CCOAPACTION", "+CCOAPACTION:", "OK", timeout_ms)) {
        ESP_LOGW(TAG, "CCOAPACTION failed");
        return false;
    }

    return true;
}

static bool coap_publish(const char *payload, int timeout_ms) {
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (coap_publish_impl(payload, timeout_ms)) {
            g_coap_publish_failures = 0;
            return true;
        }
        ESP_LOGW(TAG, "Publish attempt %d/3 failed", attempt + 1);
    }

    if (g_coap_reinit_allowed) {
        ESP_LOGW(TAG, "Publish failed after 3 attempts, trying CoAP reinit");
        if (coap_init_internal() && coap_publish_impl(payload, timeout_ms)) {
            g_coap_publish_failures = 0;
            return true;
        }
    } else {
        ESP_LOGW(TAG, "Publish failed after 3 attempts, CoAP reinit skipped on wake path");
        g_coap_publish_failures++;
        return false;
    }

    ESP_LOGW(TAG, "CoAP reinit didn't help, restarting modem");
    modem_basic_init();
    if (coap_init_internal() && coap_publish_impl(payload, timeout_ms)) {
        g_coap_publish_failures = 0;
        return true;
    }

    ESP_LOGE(TAG, "Publish failed even after modem restart");
    g_coap_publish_failures++;
    return false;
}

static void modem_setup_uart(void) {
    gpio_config_t pwr_conf = {
        .pin_bit_mask = g_cfg.power_gpio >= 0 ? (1ULL << g_cfg.power_gpio) : 0,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (pwr_conf.pin_bit_mask != 0) {
        ESP_ERROR_CHECK(gpio_config(&pwr_conf));
        gpio_set_level((gpio_num_t)g_cfg.power_gpio, 0);
        ESP_LOGI(TAG, "PWRKEY configured LOW on GPIO %d", g_cfg.power_gpio);
    }

#ifdef PIN_NB_DTR
    gpio_config_t dtr_conf = {
        .pin_bit_mask = (1ULL << PIN_NB_DTR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    modem_hold_dtr_level(false);
    ESP_ERROR_CHECK(gpio_config(&dtr_conf));
    modem_set_dtr_level(0);
#endif

    uart_config_t cfg = {
        .baud_rate = g_cfg.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_LOGI(TAG, "UART config: uart=%d baud=%d tx=%d rx=%d",
             g_cfg.uart_num, g_cfg.baud_rate, g_cfg.tx_pin, g_cfg.rx_pin);

    ESP_ERROR_CHECK(uart_driver_install(g_cfg.uart_num, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(g_cfg.uart_num, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(g_cfg.uart_num, g_cfg.tx_pin, g_cfg.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    g_uart_started = true;
}

void modem_driver_start(const modem_config_t *cfg) {
    g_cfg = *cfg;
    g_sleep_enabled = false;
    g_modem_is_sleeping = false;
    g_coap_restart = false;
    g_coap_publish_failures = 0;
    g_pdp_known_active = false;

    if (!g_uart_started) {
        modem_setup_uart();
    }
    if (!g_io.line_queue) {
        g_io.line_queue = xQueueCreate(CMD_QUEUE_LEN, sizeof(modem_line_t));
        ESP_ERROR_CHECK(g_io.line_queue ? ESP_OK : ESP_ERR_NO_MEM);
    }
    if (!g_io.uart_lock) {
        g_io.uart_lock = xSemaphoreCreateMutex();
        ESP_ERROR_CHECK(g_io.uart_lock ? ESP_OK : ESP_ERR_NO_MEM);
    }
    if (!g_io.rx_task) {
        BaseType_t ok = xTaskCreate(modem_rx_task, "modem_rx", 4096, NULL, 12, &g_io.rx_task);
        ESP_ERROR_CHECK(ok == pdPASS ? ESP_OK : ESP_FAIL);
    }
    modem_flush_input();
}

void modem_driver_stop(void) {
    modem_hold_dtr_level(false);
    modem_set_dtr_level(0);
    if (g_io.rx_task) {
        TaskHandle_t task = g_io.rx_task;
        g_io.rx_task = NULL;
        vTaskDelete(task);
    }
    if (g_uart_started) {
        uart_wait_tx_done(g_cfg.uart_num, pdMS_TO_TICKS(100));
        uart_driver_delete(g_cfg.uart_num);
        g_uart_started = false;
    }
    g_sleep_enabled = false;
    g_modem_is_sleeping = false;
    g_dtr_level = -1;
}

void modem_init_modem(void) {
    ESP_LOGI(TAG, "Starting modem init");
    modem_basic_init();
}

bool modem_try_resume(void) {
    ESP_LOGI(TAG, "Trying modem resume without reinitialization");
    modem_exit_sleep();

    for (int attempt = 0; attempt < 20; ++attempt) {
        if (modem_cmd_expect("AT", "OK", 1500)) {
            g_sleep_enabled = true;
            g_coap_restart = false;
            g_coap_publish_failures = 0;
            g_modem_is_sleeping = false;
            ESP_LOGI(TAG, "Modem resume probe succeeded");
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    ESP_LOGW(TAG, "Modem resume probe failed");
    return false;
}

bool modem_resume_fast(void) {
    if (!g_uart_started) {
        return false;
    }

    modem_exit_sleep();
    g_sleep_enabled = true;
    g_modem_is_sleeping = false;
    g_coap_restart = false;
    return true;
}

bool modem_wake_with_pwrkey(void) {
    if (!g_uart_started) {
        return false;
    }

    ESP_LOGI(TAG, "Waking modem with PWRKEY pulse");
    modem_pwrkey_pulse();
    modem_exit_sleep();

    for (int attempt = 0; attempt < 8; ++attempt) {
        if (modem_cmd_expect("AT", "OK", 600)) {
            g_sleep_enabled = true;
            g_modem_is_sleeping = false;
            ESP_LOGI(TAG, "Modem wake via PWRKEY succeeded");
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    ESP_LOGW(TAG, "Modem wake via PWRKEY failed");
    return false;
}

bool modem_enter_sleep(void) {
    if (!g_uart_started) {
        return false;
    }
    if (!g_sleep_enabled) {
        if (!modem_cmd_expect("AT+CSCLK=1", "OK", CMD_DEFAULT_TIMEOUT_MS)) {
            ESP_LOGW(TAG, "Failed to enable CSCLK sleep mode");
            return false;
        }
        g_sleep_enabled = true;
    }
    modem_set_dtr_level(1);
    vTaskDelay(pdMS_TO_TICKS(DTR_SETTLE_MS));
    modem_hold_dtr_level(true);
    g_modem_is_sleeping = true;
    ESP_LOGI(TAG, "Modem entered DTR-controlled sleep mode");
    return true;
}

bool modem_exit_sleep(void) {
    if (!g_uart_started) {
        return false;
    }
    modem_hold_dtr_level(false);
    modem_set_dtr_level(0);
    vTaskDelay(pdMS_TO_TICKS(DTR_SETTLE_MS));
    if (g_modem_is_sleeping) {
        ESP_LOGI(TAG, "Modem wake requested via DTR");
    }
    g_modem_is_sleeping = false;
    return true;
}

bool modem_init_coap(void) {
    ESP_LOGI(TAG, "Starting CoAP init");
    g_coap_reinit_allowed = true;
    bool ok = coap_init_internal();
    if (!ok) {
        ESP_LOGW(TAG, "CoAP init failed");
    }
    return ok;
}

void modem_set_coap_reinit_allowed(bool allowed) {
    g_coap_reinit_allowed = allowed;
}

void modem_set_publish_pdp_check_enabled(bool enabled) {
    g_publish_pdp_check_enabled = enabled;
}

bool modem_publish_coap(const char *path, const char *payload, int timeout_sec) {
    (void)path;
    if (g_publish_pdp_check_enabled && !ensure_pdp_active()) {
        ESP_LOGW(TAG, "Publish aborted: PDP is not active");
        return false;
    }
    const int timeout_ms = timeout_sec > 0 ? timeout_sec * 1000 : 10000;
    return coap_publish(payload, timeout_ms);
}

bool modem_get_cpsi(char *buf, size_t buf_size, int timeout_ms) {
    if (!buf || buf_size == 0) {
        return false;
    }
    buf[0] = '\0';

    modem_send_line("AT+CPSI?");
    uint64_t deadline = now_ms() + timeout_ms;
    bool got_cpsi = false;
    bool got_ok = false;
    char line[LINE_MAX_LEN];

    while (now_ms() < deadline) {
        int wait_ms = (int)(deadline - now_ms());
        if (wait_ms < 1) {
            wait_ms = 1;
        }
        if (!modem_read_line(line, sizeof(line), wait_ms)) {
            continue;
        }
        if (is_urc(line)) {
            handle_urc(line);
            continue;
        }
        ESP_LOGI(TAG, "<< %s", line);
        if (strncmp(line, "+CPSI:", 6) == 0) {
            strlcpy(buf, line, buf_size);
            got_cpsi = true;
        } else if (strcmp(line, "OK") == 0) {
            got_ok = true;
            break;
        } else if (strcmp(line, "ERROR") == 0) {
            return false;
        }
    }

    return got_cpsi && got_ok;
}

bool modem_set_functionality(int fun) {
    char cmd[24];
    snprintf(cmd, sizeof(cmd), "AT+CFUN=%d", fun);
    bool ok = modem_cmd_expect(cmd, "OK", 10000);
    ESP_LOGI(TAG, "CFUN set to %d -> %s", fun, ok ? "OK" : "FAILED");
    return ok;
}

void modem_wait_for_network(void) {
    g_init_start_ms = now_ms();
    wait_for_signal();
    wait_for_cereg();
}

bool modem_has_active_pdp(void) {
    return is_pdp_active();
}

bool modem_get_gnss_fix(modem_gnss_fix_t *fix, int timeout_ms) {
    if (!fix) {
        return false;
    }

    memset(fix, 0, sizeof(*fix));
    if (!modem_cmd_expect("AT+CGNSPWR=1", "OK", 5000)) {
        ESP_LOGW(TAG, "GNSS power on failed");
        return false;
    }

    uint64_t deadline = now_ms() + timeout_ms;
    char line[LINE_MAX_LEN];

    while (now_ms() < deadline) {
        modem_send_line("AT+CGNSINF");
        bool got_ok = false;
        bool got_fix = false;
        uint64_t cmd_deadline = now_ms() + 5000;

        while (now_ms() < cmd_deadline) {
            int wait_ms = (int)(cmd_deadline - now_ms());
            if (wait_ms < 1) {
                wait_ms = 1;
            }
            if (!modem_read_line(line, sizeof(line), wait_ms)) {
                continue;
            }
            if (is_urc(line)) {
                handle_urc(line);
                continue;
            }
            ESP_LOGI(TAG, "<< %s", line);
            if (strncmp(line, "+CGNSINF:", 9) == 0) {
                got_fix = parse_cgnsinf_fix(line, fix);
            } else if (strcmp(line, "OK") == 0) {
                got_ok = true;
                break;
            } else if (strcmp(line, "ERROR") == 0) {
                modem_cmd_expect("AT+CGNSPWR=0", "OK", 5000);
                return false;
            }
        }

        if (got_ok && got_fix) {
            modem_cmd_expect("AT+CGNSPWR=0", "OK", 5000);
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    modem_cmd_expect("AT+CGNSPWR=0", "OK", 5000);
    return false;
}
