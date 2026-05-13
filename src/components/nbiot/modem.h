// Modem driver public interface (SIM7080G) with CoAP helper.
#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "driver/gpio.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MODEM_RESET_AT_NRB = 0,
    MODEM_RESET_GPIO_PULSE = 1,
} modem_reset_method_t;

typedef struct {
    // UART configuration
    uart_port_t uart_num;
    int tx_pin;
    int rx_pin;
    int baud_rate;

    // Reset strategy
    modem_reset_method_t reset_method;
    int power_gpio;      // used only for GPIO pulse reset
    int power_pulse_ms;  // used only for GPIO pulse reset

    // Network / CoAP endpoint
    const char *apn;
    const char *coap_server;
    int coap_port;
    const char *coap_path;
} modem_config_t;

typedef struct {
    bool valid;
    char utc[24];
    float latitude;
    float longitude;
} modem_gnss_fix_t;

// Initialize synchronous UART driver state. Call once at boot.
void modem_driver_start(const modem_config_t *cfg);

// Stop UART RX task and release the UART driver before deep sleep.
void modem_driver_stop(void);

// Perform full modem bring-up (AT, CFUN, PDP, etc.). Blocking.
void modem_init_modem(void);

// Probe an already powered modem after deep-sleep wake without forcing a reset.
// Returns true if the modem responds and the existing session may be reused.
bool modem_try_resume(void);

// Fast deep-sleep wake path: release DTR sleep without sending probe AT commands.
// Any stale CoAP/session state is handled lazily by the first publish attempt.
bool modem_resume_fast(void);

// Wake modem by issuing a PWRKEY pulse and then releasing DTR sleep.
bool modem_wake_with_pwrkey(void);

// Enter or exit SIM7080 slow-clock sleep controlled by DTR.
bool modem_enter_sleep(void);
bool modem_exit_sleep(void);

// Initialize CoAP stack and prepare endpoint. Returns true on success.
bool modem_init_coap(void);

// Allow or block CoAP stack reinitialization during publish recovery.
void modem_set_coap_reinit_allowed(bool allowed);

// Allow or skip the pre-publish PDP check.
void modem_set_publish_pdp_check_enabled(bool enabled);

// Publish a CoAP payload to the configured path.
bool modem_publish_coap(const char *path, const char *payload, int timeout_sec);

// Get network information via AT+CPSI?. Returns true on success.
// Response will be stored in buf (e.g., "+CPSI: LTE NB-IOT,...")
bool modem_get_cpsi(char *buf, size_t buf_size, int timeout_ms);

// Set modem functionality level (e.g. 0=minimal, 1=full RF).
bool modem_set_functionality(int fun);

// Wait for usable signal and network registration. Blocking.
void modem_wait_for_network(void);

// Return true when the modem still has an active application PDP context.
bool modem_has_active_pdp(void);

// Acquire a GNSS fix using the modem. Returns true when a valid lat/lon fix was obtained.
bool modem_get_gnss_fix(modem_gnss_fix_t *fix, int timeout_ms);

#ifdef __cplusplus
}
#endif
