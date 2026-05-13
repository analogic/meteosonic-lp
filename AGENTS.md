# Agent Instructions

This repository contains firmware for a low-power ESP32-S3 weather probe based on the LilyGO T-SIM7080-S3, AXP2101 PMU, SIM7080G NB-IoT modem, SDI-12 ultrasonic wind sensor, optional DS18B20 temperature sensor, and an ESP32-S3 ULP RISC-V wind reader.

Follow these instructions when working in this repository.

## Project Priorities

- Preserve low-power behavior. Power-rail state, GPIO state, RTC domain settings, and modem sleep behavior are part of the product, not incidental implementation details.
- Keep normal measurement sleep and low-battery guard sleep separate:
  - Normal sleep keeps ULP wind sampling alive and preserves faster modem recovery.
  - Low-battery sleep aggressively powers down ULP, modem, wind sensor, level conversion, and unused rails.
- Do not change battery thresholds, wake intervals, APN, CoAP endpoint, or pin mapping without updating `platformio.ini` and README.
- Treat AXP2101 battery voltage reads carefully. `getBattVoltage()` may return stale/delayed values if PMU battery ADC/detection was disabled.
- Prefer small, testable changes over broad refactors. Hardware behavior is difficult to infer without measurements.

## Repository Layout

- `src/main.cpp` - application state machine, PMU setup, sleep transitions, wind statistics, payload creation.
- `src/components/nbiot/` - synchronous SIM7080G modem driver and CoAP helper.
- `src/components/hal_ulp_sdi_wind/` - main-core wrapper around ULP SDI-12 wind sampling state.
- `src/components/hal_ds18b20/` - optional DS18B20 temperature sensor driver.
- `ulp/` - ULP RISC-V program for SDI-12 wind sensor polling.
- `platformio.ini` - build flags, board config, pin mapping, power thresholds, modem and CoAP configuration.
- `README.md` - user-facing hardware and firmware documentation.
- `image/` - documentation images.

## Build And Verification

Use PlatformIO:

```sh
pio run
```

When changing ULP code or the ULP wrapper, run a full build, not only a C++ compile. The ULP binary is generated as part of the PlatformIO build.

When changing modem behavior, verify at least that `pio run` passes. Hardware validation should include serial logs for:

- modem init/resume path
- CoAP publish result
- DTR sleep entry
- wake reason and path

When changing PMU or sleep behavior, hardware validation should include measured current for:

- active measurement
- normal deep sleep between measurements
- low-battery guard sleep
- wake from low-battery guard

## Coding Guidelines

- Use the existing ESP-IDF style and APIs already present in the repo.
- Keep code in C/C++ compatible with ESP-IDF and PlatformIO.
- Do not introduce Arduino-only APIs in `src/main.cpp` or ESP-IDF components.
- Keep comments short and only where they explain hardware intent or non-obvious timing.
- Avoid adding heap-heavy abstractions in wake/sleep paths.
- Prefer explicit PMU rail operations over generic helpers when the rail meaning matters.
- Keep firmware configuration in `platformio.ini` build flags where it affects deployment.

## Power Management Rules

Current rail mapping:

- `DC3` - SIM7080G modem power, configured at 3.0 V.
- `DC5` - ultrasonic wind sensor power, configured at 3.3 V.
- `BLDO1` - level conversion / helper circuitry, configured at 3.3 V.

Normal sleep should:

- enter modem DTR/CSCLK sleep
- keep `DC3`, `DC5`, and `BLDO1` enabled
- keep RTC peripheral domain on for ULP operation
- preserve ULP SDI-12 wind sampling

Low-battery sleep should:

- stop ULP SDI-12 sampling
- stop modem UART driver
- turn off `DC3`, `DC5`, and `BLDO1`
- turn off unused LDO/DCDC rails
- deinitialize PMU and I2C where appropriate
- put selected GPIOs into low-leakage states
- enter deep sleep with RTC peripherals off

Do not merge these two sleep modes unless the user explicitly asks for that tradeoff.

## Battery Guard Behavior

Battery guard thresholds live in `platformio.ini`:

- `LOW_BATTERY_ENTER_MV`
- `LOW_BATTERY_RECOVER_MV`
- `LOW_BATTERY_CHECK_INTERVAL_SEC`

The guard uses hysteresis. Enter below `LOW_BATTERY_ENTER_MV`; return to normal operation only at or above `LOW_BATTERY_RECOVER_MV`.

If changing PMU ADC behavior:

- document whether battery ADC/detection is left enabled during low-battery sleep
- expect stale PMU voltage registers after disabling ADC
- avoid assuming a single PMU voltage read is always fresh

## Modem Rules

- SIM7080G is controlled through the driver in `src/components/nbiot/`.
- DTR sleep and GPIO hold behavior can affect sleep current. Be careful when changing `modem_enter_sleep()`, `modem_exit_sleep()`, or `modem_driver_stop()`.
- Normal wake may use fast resume only when the modem rail stayed powered through sleep.
- Wake from power-cut sleep must use full modem initialization.

## ULP Rules

- ULP code is responsible for SDI-12 wind sampling during normal sleep.
- Main CPU code must not assume ULP samples are valid after low-battery power-cut sleep.
- If ULP shared-memory layout changes, update both `ulp/` code and `src/components/hal_ulp_sdi_wind/`.
- Keep ULP timing calibration logic intact unless validating with real sensor data.

## Documentation Rules

- Keep README in English.
- Update README when changing:
  - hardware assumptions
  - power rail mapping
  - published payload fields
  - battery thresholds or wake intervals
  - sleep-state behavior
- Use existing images in `image/` when documenting hardware.

## Git And Generated Files

- Do not commit `.pio/`, `.vscode/`, or `build/`.
- Do not overwrite user changes. The worktree may contain hardware-test edits.
- Avoid destructive git commands.
- Do not reformat unrelated files.

