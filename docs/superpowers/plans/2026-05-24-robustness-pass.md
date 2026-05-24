# Robustness Pass Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make new-keyboard pairing reliable, reconnect work without re-pair, already-connected targets fail cleanly, LED respond to mode change in ≤50ms, no keystroke-path malloc, no infinite scan waits — all guarded by host-side unit tests for the pure logic that was extracted.

**Architecture:** Surgical bug fixes on the existing layout (no new modules). Two pure-C helpers (`led_duty_for`, `pack_ch9329_data`) are extracted into their own files so they can be compiled against host gcc and unit-tested without ESP-IDF. A new `test/host/` tree builds these against a vendored Unity. Everything else (BLE security init, NVS namespace bump + bond wipe, scan-deadlock bounded waits, CONNECTING state machine, stack/watchdog hardening) is verified by manual on-device verification through a written checklist.

**Tech Stack:** ESP-IDF (ESP32-C3), Bluedroid BLE host, FreeRTOS, NVS, LEDC PWM. Host tests: gcc, CMake, Unity 2.5.2.

**Companion spec:** `docs/superpowers/specs/2026-05-24-robustness-pass-design.md` — design rationale, risks, and the verification plan referenced from Task 13.

---

## Task ordering rationale

1. Bootstrap host test infrastructure first (Task 1) — every later TDD task depends on it.
2. Extract pure helpers under TDD (Tasks 2, 4) — write tests before code.
3. Wire the helpers into firmware (Tasks 3, 5) — removes the bugs from the keystroke + LED paths.
4. BLE security + NVS migration (Tasks 6, 7) — together these fix pairing/reconnect.
5. Bounded waits (Task 8) — removes the `portMAX_DELAY` deadlock.
6. CONNECT state machine (Task 9) — the largest behavior change; depends on Task 8's bounded waits being in place so the scan path can recover.
7. Stack/watchdog hardening (Task 10).
8. Build script + README polish (Tasks 11, 12).
9. On-device verification at the end (Task 13).

Commit at the end of every task. If a task has TDD steps, the test/code/refactor steps may also be committed separately when convenient — but always at task end.

---

## File map

**New files:**
- `main/ch9329_pack.c` / `main/ch9329_pack.h` — pure `pack_ch9329_data(cmd, data, len, out)`, no ESP-IDF deps.
- `main/led_duty.c` / `main/led_duty.h` — pure `led_duty_for(mode, tick)`, no ESP-IDF deps.
- `test/host/CMakeLists.txt` — host gcc build for the two pure files + Unity + tests.
- `test/host/runner.c` — Unity main, registers both suites.
- `test/host/test_ch9329_pack.c` — ~6 cases.
- `test/host/test_led_duty.c` — ~5 cases.
- `test/host/vendor/unity.c`, `unity.h`, `unity_internals.h` — Unity 2.5.2 (MIT), fetched from upstream.

**Modified files:**
- `main/CMakeLists.txt` — add `ch9329_pack.c`, `led_duty.c` to srcs.
- `main/ch932x.c` / `main/ch932x.h` — remove `g_packed_data`/`g_packed_data_len`/`pack_ch9329_data`. Add static `s_packed_buf[70]` exposed via header.
- `main/led.c` / `main/led.h` — rename file-static to `g_update_led`. Fix `get_led_mode` pointer bug. Replace per-mode `while` helpers with one single-tick loop calling `led_duty_for`. Remove `set_update_led` and the second param of `get_led_mode`.
- `main/storage.c` / `main/storage.h` — bump namespace to `hidproxy_v2`, add `schema_version` key, phase-1 erase old namespace in `init_nvs_flash`, new export `storage_complete_migration()` for phase-2 bond wipe.
- `main/esp_hid_gap.c` — add `esp_ble_gap_set_security_param` block in `init_low_level`. Replace `WAIT_BLE_CB()` with 3s bounded wait, propagate timeout. Drive-by fix: replace `if (memcpy(...) == NULL)` with `name_len <= NAME_LEN_MAX` guard.
- `main/esp_hid_host_main.c` — add `BLE_STATUS_CONNECTING`, `s_connecting_since_us`, `s_failed_bda`, `s_failed_at_us`. Move `save_ble_device`/`set_led_mode`/status-flip into `OPEN_EVENT` success branch. On `OPEN_EVENT` failure: log + `esp_hidh_dev_free` + record backoff. Restructure `connect_hid_dev` as dispatch-only. `hid_connect` loop honors `CONNECTING` (10s timeout) and skips backed-off candidates (30s). Call `storage_complete_migration()` after `esp_hid_gap_init()`. Bump `hid_connect` stack 6KB→8KB. Optional heap-log under `#ifdef DEBUG_HEAP`. Pass `s_packed_buf` to new `pack_ch9329_data`.
- `sdkconfig.defaults` — add `CONFIG_FREERTOS_CHECK_STACKOVERFLOW=2`.
- `build_flash_monitor.sh` — `PORT="${PORT:-/dev/ttyUSB0}"`.
- `README.org` — flash example mentions `PORT=…`, remove fixed items from "Known issues", remove "No authentication is required", add "Tests" section.

---

## Task 1: Bootstrap host test infrastructure

Lays the groundwork for TDD in later tasks. Vendors Unity, sets up a CMake host build, registers a trivial smoke test, and verifies the build/run loop works before any real test is written.

**Files:**
- Create: `test/host/vendor/unity.c`, `test/host/vendor/unity.h`, `test/host/vendor/unity_internals.h`
- Create: `test/host/CMakeLists.txt`
- Create: `test/host/runner.c`
- Create: `test/host/test_smoke.c` (deleted at end of Task 2)

- [ ] **Step 1: Fetch Unity 2.5.2 sources**

Run from repo root:

```bash
mkdir -p test/host/vendor
curl -fsSL -o test/host/vendor/unity.c           https://raw.githubusercontent.com/ThrowTheSwitch/Unity/v2.5.2/src/unity.c
curl -fsSL -o test/host/vendor/unity.h           https://raw.githubusercontent.com/ThrowTheSwitch/Unity/v2.5.2/src/unity.h
curl -fsSL -o test/host/vendor/unity_internals.h https://raw.githubusercontent.com/ThrowTheSwitch/Unity/v2.5.2/src/unity_internals.h
```

Verify file sizes are nonzero:

```bash
wc -c test/host/vendor/unity.{c,h} test/host/vendor/unity_internals.h
```

Expected: each file > 0 bytes (`unity.c` ~50 KB, `unity.h` ~30 KB, `unity_internals.h` ~40 KB).

- [ ] **Step 2: Write `test/host/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.16)
project(hid_proxy_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)

set(MAIN_DIR   ${CMAKE_SOURCE_DIR}/../../main)
set(VENDOR_DIR ${CMAKE_SOURCE_DIR}/vendor)

add_executable(run_tests
    runner.c
    test_smoke.c
    ${VENDOR_DIR}/unity.c
)

target_include_directories(run_tests PRIVATE
    ${MAIN_DIR}
    ${VENDOR_DIR}
    ${CMAKE_SOURCE_DIR}
)
```

(`test_ch9329_pack.c`, `test_led_duty.c`, `main/ch9329_pack.c`, and `main/led_duty.c` get added to `add_executable` in Tasks 2 and 4. `test_smoke.c` is removed from the list in Task 2.)

- [ ] **Step 3: Write `test/host/runner.c`**

```c
#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

void run_smoke_tests(void);

int main(void) {
    UNITY_BEGIN();
    run_smoke_tests();
    return UNITY_END();
}
```

(Later tasks add `run_ch9329_pack_tests();` and `run_led_duty_tests();` calls and drop the smoke call.)

- [ ] **Step 4: Write `test/host/test_smoke.c`**

```c
#include "unity.h"

static void test_smoke_passes(void) {
    TEST_ASSERT_EQUAL_INT(2, 1 + 1);
}

void run_smoke_tests(void) {
    RUN_TEST(test_smoke_passes);
}
```

- [ ] **Step 5: Configure and build**

```bash
cd test/host && cmake -B build && cmake --build build
```

Expected: configuration succeeds, build produces `test/host/build/run_tests` with no warnings (Unity itself may print one or two; ignore those — only watch the project sources).

- [ ] **Step 6: Run the smoke test**

```bash
./build/run_tests
```

Expected output ends with:

```
1 Tests 0 Failures 0 Ignored
OK
```

- [ ] **Step 7: Add `test/host/build/` to `.gitignore`**

Open `.gitignore` (create if missing). Append:

```
test/host/build/
```

Verify the build dir is ignored:

```bash
git status --porcelain | grep -E '^\?\? test/host/build' && echo FAIL || echo OK
```

Expected: `OK`.

- [ ] **Step 8: Commit**

```bash
git add test/host/CMakeLists.txt test/host/runner.c test/host/test_smoke.c \
        test/host/vendor/unity.c test/host/vendor/unity.h test/host/vendor/unity_internals.h \
        .gitignore
git commit -m "Add host test scaffolding (Unity 2.5.2, CMake, smoke test)"
```

---

## Task 2: Extract `pack_ch9329_data` as a pure function (TDD)

The current `pack_ch9329_data` in `main/ch932x.c` mallocs from the BLE event-task context on every keystroke. Replace it with a pure function that writes into a caller-supplied buffer. Tests are written first.

**Files:**
- Create: `main/ch9329_pack.h`, `main/ch9329_pack.c`
- Create: `test/host/test_ch9329_pack.c`
- Modify: `test/host/CMakeLists.txt`, `test/host/runner.c`
- Delete: `test/host/test_smoke.c`

- [ ] **Step 1: Write the header `main/ch9329_pack.h`**

```c
#ifndef CH9329_PACK_H_
#define CH9329_PACK_H_

#include <stdint.h>

#define CH9329_HEADER_LEN  6
#define CH9329_MAX_PAYLOAD 64

typedef enum {
    CMD_GET_INFO              = 0x01,
    CMD_SEND_KB_GENERAL_DATA  = 0x02,
    CMD_SEND_KB_MEDIA_DATA    = 0x03,
} CH9329CMD;

// Writes header + payload + checksum to `out` (caller-owned, at least
// CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD bytes). Returns total bytes
// written on success, or 0 if length is out of range.
int pack_ch9329_data(CH9329CMD cmd, const uint8_t *data, int length, uint8_t *out);

#endif
```

- [ ] **Step 2: Write `test/host/test_ch9329_pack.c` (tests first, no implementation yet)**

```c
#include "unity.h"
#include "ch9329_pack.h"
#include <string.h>

static void test_header_addr_cmd_len(void) {
    uint8_t out[CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD] = {0};
    uint8_t payload[3] = {0x11, 0x22, 0x33};
    int n = pack_ch9329_data(CMD_SEND_KB_GENERAL_DATA, payload, 3, out);
    TEST_ASSERT_EQUAL_INT(3 + CH9329_HEADER_LEN, n);
    TEST_ASSERT_EQUAL_HEX8(0x57, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xAB, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[2]);
    TEST_ASSERT_EQUAL_HEX8(CMD_SEND_KB_GENERAL_DATA, out[3]);
    TEST_ASSERT_EQUAL_HEX8(3,    out[4]);
}

static void test_payload_copied(void) {
    uint8_t out[CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD] = {0};
    uint8_t payload[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    int n = pack_ch9329_data(CMD_SEND_KB_GENERAL_DATA, payload, 4, out);
    TEST_ASSERT_EQUAL_INT(4 + CH9329_HEADER_LEN, n);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, &out[5], 4);
}

static void test_checksum(void) {
    uint8_t out[CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD] = {0};
    uint8_t payload[3] = {0x01, 0x02, 0x03};
    int n = pack_ch9329_data(CMD_SEND_KB_GENERAL_DATA, payload, 3, out);
    uint8_t expected = 0;
    for (int i = 0; i < n - 1; ++i) expected = (uint8_t)(expected + out[i]);
    TEST_ASSERT_EQUAL_HEX8(expected, out[n - 1]);
}

static void test_zero_length(void) {
    uint8_t out[CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD] = {0xFF};
    int n = pack_ch9329_data(CMD_GET_INFO, NULL, 0, out);
    TEST_ASSERT_EQUAL_INT(CH9329_HEADER_LEN, n);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[4]);
    uint8_t expected = (uint8_t)(0x57 + 0xAB + 0x00 + CMD_GET_INFO + 0x00);
    TEST_ASSERT_EQUAL_HEX8(expected, out[5]);
}

static void test_max_length(void) {
    uint8_t out[CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD] = {0};
    uint8_t payload[CH9329_MAX_PAYLOAD];
    for (int i = 0; i < CH9329_MAX_PAYLOAD; ++i) payload[i] = (uint8_t)i;
    int n = pack_ch9329_data(CMD_SEND_KB_GENERAL_DATA, payload, CH9329_MAX_PAYLOAD, out);
    TEST_ASSERT_EQUAL_INT(CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD, n);
}

static void test_length_too_big_returns_zero(void) {
    uint8_t out[CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD] = {0};
    int n = pack_ch9329_data(CMD_SEND_KB_GENERAL_DATA, NULL, CH9329_MAX_PAYLOAD + 1, out);
    TEST_ASSERT_EQUAL_INT(0, n);
}

static void test_negative_length_returns_zero(void) {
    uint8_t out[CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD] = {0};
    int n = pack_ch9329_data(CMD_SEND_KB_GENERAL_DATA, NULL, -1, out);
    TEST_ASSERT_EQUAL_INT(0, n);
}

void run_ch9329_pack_tests(void) {
    RUN_TEST(test_header_addr_cmd_len);
    RUN_TEST(test_payload_copied);
    RUN_TEST(test_checksum);
    RUN_TEST(test_zero_length);
    RUN_TEST(test_max_length);
    RUN_TEST(test_length_too_big_returns_zero);
    RUN_TEST(test_negative_length_returns_zero);
}
```

- [ ] **Step 3: Wire tests into the runner and CMake**

Edit `test/host/runner.c` to call the new suite and drop the smoke call:

```c
#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

void run_ch9329_pack_tests(void);

int main(void) {
    UNITY_BEGIN();
    run_ch9329_pack_tests();
    return UNITY_END();
}
```

Edit `test/host/CMakeLists.txt` — replace `test_smoke.c` with `test_ch9329_pack.c` and add `${MAIN_DIR}/ch9329_pack.c`:

```cmake
add_executable(run_tests
    runner.c
    test_ch9329_pack.c
    ${MAIN_DIR}/ch9329_pack.c
    ${VENDOR_DIR}/unity.c
)
```

Delete the smoke file:

```bash
rm test/host/test_smoke.c
```

- [ ] **Step 4: Confirm the tests fail (no implementation yet)**

```bash
cd test/host && cmake --build build
```

Expected: link error — `undefined reference to 'pack_ch9329_data'`. This confirms the tests are reaching for the symbol.

- [ ] **Step 5: Write `main/ch9329_pack.c` (minimal implementation)**

```c
#include "ch9329_pack.h"

int pack_ch9329_data(CH9329CMD cmd, const uint8_t *data, int length, uint8_t *out) {
    if (length < 0 || length > CH9329_MAX_PAYLOAD) {
        return 0;
    }

    out[0] = 0x57;
    out[1] = 0xAB;
    out[2] = 0x00;
    out[3] = (uint8_t)cmd;
    out[4] = (uint8_t)length;
    for (int i = 0; i < length; ++i) {
        out[5 + i] = data[i];
    }

    int sum_pos = length + CH9329_HEADER_LEN - 1;
    uint8_t sum = 0;
    for (int i = 0; i < sum_pos; ++i) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[sum_pos] = sum;

    return length + CH9329_HEADER_LEN;
}
```

- [ ] **Step 6: Rebuild and run tests**

```bash
cd test/host && cmake --build build && ./build/run_tests
```

Expected output ends with:

```
7 Tests 0 Failures 0 Ignored
OK
```

- [ ] **Step 7: Commit**

```bash
git add main/ch9329_pack.h main/ch9329_pack.c \
        test/host/test_ch9329_pack.c test/host/runner.c test/host/CMakeLists.txt
git rm test/host/test_smoke.c
git commit -m "Extract pack_ch9329_data as pure function with host tests"
```

---

## Task 3: Wire `ch9329_pack` into the firmware

Replace the malloc-per-keystroke path with the new pure function backed by a static buffer.

**Files:**
- Modify: `main/ch932x.h`, `main/ch932x.c`
- Modify: `main/esp_hid_host_main.c`
- Modify: `main/CMakeLists.txt`

- [ ] **Step 1: Update `main/ch932x.h`**

Remove the `CH9329CMD` enum, `FIXED_CH9329_DATA_LEN`, the `g_packed_data`/`g_packed_data_len` externs, and the `pack_ch9329_data` declaration (they now live in `ch9329_pack.h`). Add an `extern` for the static-buffer pointer. Result:

```c
#ifndef CH932X_H_
#define CH932X_H_

/*
 *  Send data to CH9328/CH9329
 */

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
#include <stdint.h>

static const int TX_BUF_SIZE = 1024;
static const int uart_num = UART_NUM_1;

#define TXD_PIN (GPIO_NUM_7)
#define RXD_PIN (GPIO_NUM_8)

/*Init uart*/
void init_uart();

/*send data to port*/
int send_data_to_uart(void *data, int len);

#ifdef USE_CH9329
#include "ch9329_pack.h"

// Shared packed-frame buffer owned by ch932x.c; caller fills via pack_ch9329_data.
extern uint8_t s_packed_buf[CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD];
#endif

#endif // CH932X_H_
```

- [ ] **Step 2: Update `main/ch932x.c`**

Replace the whole `#ifdef USE_CH9329 ... #endif` block at the bottom with a single buffer definition. Final state:

```c
#include "ch932x.h"
#include <esp_log.h>
#include <stdio.h>

#ifdef USE_CH9329
static const char *CH932X_TAG = "CH9329";
uint8_t s_packed_buf[CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD];
#else
static const char *CH932X_TAG = "CH9328";
#endif

void init_uart() {
  uart_config_t uart_config = {
      .baud_rate = 9600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };

  ESP_ERROR_CHECK(
      uart_driver_install(uart_num, TX_BUF_SIZE * 2, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE));
  ESP_LOGI("UART", "SET PIN: TX = %d, RX = %d\n", TXD_PIN, RXD_PIN);
}

int send_data_to_uart(void *data, int length) {
  const int tx_bytes = uart_write_bytes(uart_num, data, length);
  ESP_LOG_BUFFER_HEX(CH932X_TAG, data, length);
  return tx_bytes;
}
```

The `#include <stdlib.h>` is no longer needed (no more `malloc`/`free`).

- [ ] **Step 3: Update the call site in `main/esp_hid_host_main.c`**

Find the existing `ESP_HIDH_INPUT_EVENT` block (around line 73). Replace the `USE_CH9329` branch:

```c
    // Send keycodes to ch9328/ch9329
#ifdef USE_CH9329
    int n = pack_ch9329_data(CMD_SEND_KB_GENERAL_DATA, param->input.data,
                             param->input.length, s_packed_buf);
    if (n > 0) {
      send_data_to_uart(s_packed_buf, n);
    } else {
      ESP_LOGW(TAG, "pack_ch9329_data rejected length %d", param->input.length);
    }
#else
    // For ch9328, pass the keycode data directly
    send_data_to_uart(param->input.data, param->input.length);
#endif
```

- [ ] **Step 4: Add `ch9329_pack.c` to `main/CMakeLists.txt`**

```cmake
set(srcs "esp_hid_host_main.c"
         "esp_hid_gap.c"
         "ch932x.c"
         "ch9329_pack.c"
         "led.c"
         "storage.c")
```

(Leave the rest of the file untouched. `led_duty.c` is added in Task 4.)

- [ ] **Step 5: Build the firmware**

```bash
idf.py build
```

Expected: clean build, no warnings about the changed files.

If `idf.py` is not on PATH for this session, source ESP-IDF's `export.sh` first, then re-run. The build verifies compile only — runtime verification happens in Task 13.

- [ ] **Step 6: Re-run host tests (sanity check — extracted file still compiles in host context)**

```bash
cd test/host && cmake --build build && ./build/run_tests
```

Expected: `7 Tests 0 Failures 0 Ignored OK`.

- [ ] **Step 7: Commit**

```bash
git add main/ch932x.h main/ch932x.c main/esp_hid_host_main.c main/CMakeLists.txt
git commit -m "Wire pack_ch9329_data through static buffer; drop heap alloc"
```

---

## Task 4: Extract `led_duty_for` as a pure function (TDD)

Same pattern as Task 2 — pure function with host tests, no firmware integration yet.

**Files:**
- Create: `main/led_duty.h`, `main/led_duty.c`
- Create: `test/host/test_led_duty.c`
- Modify: `test/host/CMakeLists.txt`, `test/host/runner.c`

- [ ] **Step 1: Write `main/led_duty.h`**

```c
#ifndef LED_DUTY_H_
#define LED_DUTY_H_

#include <stdint.h>

// Duty values mirror led.c's existing LEDC_DUTY_ON / LEDC_DUTY_DIME.
#define LED_DUTY_ON   0
#define LED_DUTY_DIME 8000

// Mode constants — kept in sync with led.h LED_MODE_* values.
#define LED_MODE_OFF        0
#define LED_MODE_FAST_BLINK 1
#define LED_MODE_ALWAYS_ON  2
#define LED_MODE_SLOW_BLINK 3

// Returns the LEDC duty value for the given mode at the given 50ms tick.
int led_duty_for(int mode, int tick);

#endif
```

(`led.h` already has identical mode constants. We re-declare them here so the host test build doesn't need to drag in `led.h` and its ESP-IDF includes. Task 5 will guard against drift by referencing these in `led.c` directly.)

- [ ] **Step 2: Write `test/host/test_led_duty.c`**

```c
#include "unity.h"
#include "led_duty.h"

static void test_off_is_always_dim(void) {
    for (int t = 0; t < 100; ++t) {
        TEST_ASSERT_EQUAL_INT(LED_DUTY_DIME, led_duty_for(LED_MODE_OFF, t));
    }
}

static void test_always_on_is_always_on(void) {
    for (int t = 0; t < 100; ++t) {
        TEST_ASSERT_EQUAL_INT(LED_DUTY_ON, led_duty_for(LED_MODE_ALWAYS_ON, t));
    }
}

static void test_fast_blink_toggles_every_two_ticks(void) {
    TEST_ASSERT_EQUAL_INT(LED_DUTY_ON,   led_duty_for(LED_MODE_FAST_BLINK, 0));
    TEST_ASSERT_EQUAL_INT(LED_DUTY_ON,   led_duty_for(LED_MODE_FAST_BLINK, 1));
    TEST_ASSERT_EQUAL_INT(LED_DUTY_DIME, led_duty_for(LED_MODE_FAST_BLINK, 2));
    TEST_ASSERT_EQUAL_INT(LED_DUTY_DIME, led_duty_for(LED_MODE_FAST_BLINK, 3));
    TEST_ASSERT_EQUAL_INT(LED_DUTY_ON,   led_duty_for(LED_MODE_FAST_BLINK, 4));
    TEST_ASSERT_EQUAL_INT(LED_DUTY_DIME, led_duty_for(LED_MODE_FAST_BLINK, 6));
}

static void test_slow_blink_toggles_every_twenty_ticks(void) {
    TEST_ASSERT_EQUAL_INT(LED_DUTY_ON,   led_duty_for(LED_MODE_SLOW_BLINK, 0));
    TEST_ASSERT_EQUAL_INT(LED_DUTY_ON,   led_duty_for(LED_MODE_SLOW_BLINK, 19));
    TEST_ASSERT_EQUAL_INT(LED_DUTY_DIME, led_duty_for(LED_MODE_SLOW_BLINK, 20));
    TEST_ASSERT_EQUAL_INT(LED_DUTY_DIME, led_duty_for(LED_MODE_SLOW_BLINK, 39));
    TEST_ASSERT_EQUAL_INT(LED_DUTY_ON,   led_duty_for(LED_MODE_SLOW_BLINK, 40));
    TEST_ASSERT_EQUAL_INT(LED_DUTY_DIME, led_duty_for(LED_MODE_SLOW_BLINK, 60));
}

static void test_unknown_mode_defaults_dim(void) {
    TEST_ASSERT_EQUAL_INT(LED_DUTY_DIME, led_duty_for(999, 0));
    TEST_ASSERT_EQUAL_INT(LED_DUTY_DIME, led_duty_for(-1, 5));
}

void run_led_duty_tests(void) {
    RUN_TEST(test_off_is_always_dim);
    RUN_TEST(test_always_on_is_always_on);
    RUN_TEST(test_fast_blink_toggles_every_two_ticks);
    RUN_TEST(test_slow_blink_toggles_every_twenty_ticks);
    RUN_TEST(test_unknown_mode_defaults_dim);
}
```

- [ ] **Step 3: Update runner and CMake**

`test/host/runner.c`:

```c
#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

void run_ch9329_pack_tests(void);
void run_led_duty_tests(void);

int main(void) {
    UNITY_BEGIN();
    run_ch9329_pack_tests();
    run_led_duty_tests();
    return UNITY_END();
}
```

`test/host/CMakeLists.txt` — add the two new files to `add_executable`:

```cmake
add_executable(run_tests
    runner.c
    test_ch9329_pack.c
    test_led_duty.c
    ${MAIN_DIR}/ch9329_pack.c
    ${MAIN_DIR}/led_duty.c
    ${VENDOR_DIR}/unity.c
)
```

- [ ] **Step 4: Confirm the new tests fail**

```bash
cd test/host && cmake --build build
```

Expected: link error — `undefined reference to 'led_duty_for'`.

- [ ] **Step 5: Write `main/led_duty.c`**

```c
#include "led_duty.h"

int led_duty_for(int mode, int tick) {
    switch (mode) {
    case LED_MODE_OFF:
        return LED_DUTY_DIME;
    case LED_MODE_ALWAYS_ON:
        return LED_DUTY_ON;
    case LED_MODE_FAST_BLINK:
        return ((tick / 2) % 2 == 0) ? LED_DUTY_ON : LED_DUTY_DIME;
    case LED_MODE_SLOW_BLINK:
        return ((tick / 20) % 2 == 0) ? LED_DUTY_ON : LED_DUTY_DIME;
    default:
        return LED_DUTY_DIME;
    }
}
```

- [ ] **Step 6: Rebuild and run**

```bash
cd test/host && cmake --build build && ./build/run_tests
```

Expected: `12 Tests 0 Failures 0 Ignored OK`.

- [ ] **Step 7: Confirm `LED_MODE_*` constants agree with `main/led.h`**

```bash
grep -E 'LED_MODE_(OFF|FAST_BLINK|ALWAYS_ON|SLOW_BLINK)' main/led.h main/led_duty.h
```

Expected: both files define them to the same integers (`OFF=0`, `FAST_BLINK=1`, `ALWAYS_ON=2`, `SLOW_BLINK=3`). If anything differs, fix `led_duty.h` to match `led.h` — `led.h` is the existing source of truth.

- [ ] **Step 8: Commit**

```bash
git add main/led_duty.h main/led_duty.c \
        test/host/test_led_duty.c test/host/runner.c test/host/CMakeLists.txt
git commit -m "Extract led_duty_for as pure function with host tests"
```

---

## Task 5: Fix LED bugs and restructure `change_led`

Three things at once because they touch the same file region: (1) fix the `update_led` self-write bug, (2) replace per-mode `while` helpers with a single 50ms-tick loop driven by `led_duty_for`, (3) drop the redundant `update_led` flag (the loop detects mode changes itself).

**Files:**
- Modify: `main/led.h`, `main/led.c`
- Modify: `main/esp_hid_host_main.c` (only if any caller passes the now-removed second arg of `get_led_mode`)
- Modify: `main/CMakeLists.txt` (add `led_duty.c`)

- [ ] **Step 1: Add `led_duty.c` to `main/CMakeLists.txt`**

```cmake
set(srcs "esp_hid_host_main.c"
         "esp_hid_gap.c"
         "ch932x.c"
         "ch9329_pack.c"
         "led.c"
         "led_duty.c"
         "storage.c")
```

- [ ] **Step 2: Update `main/led.h`**

Drop `set_update_led` and the second parameter of `get_led_mode`. Final:

```c
#ifndef LED_H_
#define LED_H_

#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdio.h>

/* try to connect to any bluetooth */
#define LED_MODE_FAST_BLINK 1
/* already connected */
#define LED_MODE_ALWAYS_ON 2
/* try to connect to a recorded (previously connected) bluetooth */
#define LED_MODE_SLOW_BLINK 3
/* light off */
#define LED_MODE_OFF 0

typedef struct _LEDMessage {
  int32_t mode;
} LEDMessage;

void ledc_init(void);

// FreeRTOS task: drive the LED according to the current mode.
void change_led(void *);

// Set the LED mode (thread-safe).
bool set_led_mode(int m);

// Read the current LED mode (thread-safe).
bool get_led_mode(int *mode);

void print_led_mode(int mode);

#endif // LED_H_
```

- [ ] **Step 3: Rewrite `main/led.c`**

Replace the entire file with:

```c
#include "led.h"
#include "led_duty.h"
#include "driver/ledc.h"
#include "freertos/portmacro.h"
#include "freertos/projdefs.h"
#include "hal/ledc_types.h"
#include <stdio.h>

#define LEDC_TIMER     LEDC_TIMER_0
#define LEDC_MODE      LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO (3)
#define LEDC_CHANNEL   LEDC_CHANNEL_0
#define LEDC_DUTY_RES  LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY (5000)
#define LED_TICK_MS    50

static LEDMessage led_message;
static SemaphoreHandle_t led_semaphor = NULL;

void change_led(void *_) {
  int last_mode = -1;
  int phase = 0;
  while (true) {
    int mode = LED_MODE_OFF;
    if (!get_led_mode(&mode)) {
      vTaskDelay(LED_TICK_MS / portTICK_PERIOD_MS);
      continue;
    }
    if (mode != last_mode) {
      phase = 0;
      last_mode = mode;
    }
    int duty = led_duty_for(mode, phase);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    phase++;
    vTaskDelay(LED_TICK_MS / portTICK_PERIOD_MS);
  }
}

bool get_led_mode(int *mode) {
  if (xSemaphoreTake(led_semaphor, (TickType_t)10) == pdTRUE) {
    if (mode != NULL) {
      *mode = led_message.mode;
    }
    xSemaphoreGive(led_semaphor);
    return true;
  }
  return false;
}

bool set_led_mode(int m) {
  if (xSemaphoreTake(led_semaphor, (TickType_t)10) == pdTRUE) {
    led_message.mode = m;
    xSemaphoreGive(led_semaphor);
    return true;
  }
  return false;
}

void ledc_init(void) {
  ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_MODE,
      .timer_num = LEDC_TIMER,
      .duty_resolution = LEDC_DUTY_RES,
      .freq_hz = LEDC_FREQUENCY,
      .clk_cfg = LEDC_AUTO_CLK};
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  ledc_channel_config_t ledc_channel = {.speed_mode = LEDC_MODE,
                                        .channel = LEDC_CHANNEL,
                                        .timer_sel = LEDC_TIMER,
                                        .intr_type = LEDC_INTR_DISABLE,
                                        .gpio_num = LEDC_OUTPUT_IO,
                                        .duty = LED_DUTY_DIME,
                                        .hpoint = 0};
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
  vSemaphoreCreateBinary(led_semaphor);
}

void print_led_mode(int mode) {
  switch (mode) {
  case LED_MODE_ALWAYS_ON:  printf("LED_MODE_ALWAYS_ON");  break;
  case LED_MODE_FAST_BLINK: printf("LED_MODE_FAST_BLINK"); break;
  case LED_MODE_SLOW_BLINK: printf("LED_MODE_SLOW_BLINK"); break;
  case LED_MODE_OFF:        printf("LED_MODE_OFF");        break;
  default:                  printf("Unknown LED_MODE");    break;
  }
}
```

- [ ] **Step 4: Update callers of `get_led_mode` / `set_update_led`**

Search for any remaining callers:

```bash
grep -rn 'get_led_mode\|set_update_led' main/
```

Expected: only `main/led.c` references remain. If `main/esp_hid_host_main.c` (or any other file) calls `get_led_mode(&x, NULL)` or `set_update_led(...)`, update those calls. Current state of `esp_hid_host_main.c` does not call either, so no change is expected — but verify.

- [ ] **Step 5: Bump LED task stack to handle the slightly larger loop**

In `main/esp_hid_host_main.c::app_main`, the LED task is created with 2048 bytes. The new loop is smaller (no nested helpers), so 2048 is still fine — no change needed. Confirm by reading the existing line:

```bash
grep -n 'change_led' main/esp_hid_host_main.c
```

Expected: line `xTaskCreate(&change_led, "change_led", 2048, NULL, 2, NULL);` unchanged.

- [ ] **Step 6: Build the firmware**

```bash
idf.py build
```

Expected: clean compile, no warnings on `led.c` / `led.h`.

- [ ] **Step 7: Commit**

```bash
git add main/led.h main/led.c main/CMakeLists.txt
git commit -m "Fix LED pointer bug; collapse change_led to one 50ms-tick loop"
```

---

## Task 6: BLE security parameters

Configure SMP bonding so modern BLE keyboards can pair and reconnect.

**Files:**
- Modify: `main/esp_hid_gap.c`

- [ ] **Step 1: Add security-param configuration in `init_low_level`**

In `main/esp_hid_gap.c`, locate `init_low_level` (around line 392). After the `esp_bluedroid_enable()` success check (around line 423–426) and **before** the `#if CONFIG_BT_BLE_ENABLED` / `init_ble_gap()` block, insert:

```c
#if CONFIG_BT_BLE_ENABLED
  if (mode & ESP_BT_MODE_BLE) {
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_BOND;
    esp_ble_io_cap_t   iocap    = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key  = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE,      &iocap,    sizeof(iocap)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE,    &key_size, sizeof(key_size)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY,    &init_key, sizeof(init_key)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY,     &rsp_key,  sizeof(rsp_key)));
  }
#endif
```

The block sits between the bluedroid enable and the existing `init_ble_gap()` call, *not* inside the existing `#if CONFIG_BT_BLE_ENABLED` that wraps `init_ble_gap`. Two adjacent `#if` blocks is acceptable. Alternatively merge them — pick whichever reads better in context.

- [ ] **Step 2: Build**

```bash
idf.py build
```

Expected: clean compile. If any of the `ESP_BLE_SM_*` or `ESP_LE_AUTH_REQ_SC_BOND` symbols are missing, `#include "esp_gap_ble_api.h"` is already present at the top of the file — re-check.

- [ ] **Step 3: Commit**

```bash
git add main/esp_hid_gap.c
git commit -m "Configure BLE SMP security params (LESC bonding, no MITM, IO=NONE)"
```

---

## Task 7: NVS schema bump + first-boot wipe

After the security change, any MAC saved by the previous firmware will not have a matching Bluedroid bond. A two-phase migration erases both stores on the first boot of the new firmware.

**Files:**
- Modify: `main/storage.h`, `main/storage.c`
- Modify: `main/esp_hid_host_main.c`

- [ ] **Step 1: Update `main/storage.h`**

Bump the namespace, add the schema-version key, declare the new export. Final:

```c
#ifndef HID_PROXY_STORAGE_H
#define HID_PROXY_STORAGE_H

#include "driver/gpio.h"
#include "esp_bt_defs.h"
#include "esp_err.h"
#include "esp_hid_gap.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <stdio.h>

#define STORAGE_NAMESPACE   "hidproxy_v2"
#define STORAGE_OLD_NAMESPACE "storage"
#define BOOT_MODE_PIN       GPIO_NUM_9
#define BLE_RESULTS_STORAGE "ble_results"
#define BLE_STATUS          "ble_status"
#define SCHEMA_VERSION_KEY  "schema_version"
#define SCHEMA_VERSION_VAL  2

esp_err_t init_nvs_flash(void);

// Finish first-boot migration (erases old Bluedroid bonds, writes schema_version).
// Must be called AFTER esp_hid_gap_init() so the bond list is available.
// Idempotent: no-op once schema_version is written.
esp_err_t storage_complete_migration(void);

bool save_ble_device(uint8_t *mac_addr);
bool read_ble_device(uint8_t *mac_addr);
bool clear_ble_devices(void);

#endif // HID_PROXY_STORAGE_H
```

- [ ] **Step 2: Update `main/storage.c`**

Add the migration flag and phase-1 logic, plus the new export. Insert near the top, just below the existing `static SemaphoreHandle_t sema_handle = NULL;`:

```c
static bool g_need_bond_wipe = false;
```

Replace `init_nvs_flash` with:

```c
esp_err_t init_nvs_flash() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  if (ret != ESP_OK) {
    return ret;
  }

  vSemaphoreCreateBinary(sema_handle);

  // Phase 1 of first-boot migration: if schema_version is missing from the new
  // namespace, erase the old "storage" namespace and remember to wipe bonds in
  // phase 2 (which requires Bluedroid to be up — see storage_complete_migration).
  nvs_handle_t h;
  esp_err_t open_err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &h);
  if (open_err == ESP_OK) {
    int32_t schema = 0;
    esp_err_t get_err = nvs_get_i32(h, SCHEMA_VERSION_KEY, &schema);
    nvs_close(h);
    if (get_err == ESP_OK && schema == SCHEMA_VERSION_VAL) {
      g_need_bond_wipe = false;
      return ESP_OK;
    }
  }

  // Either the new namespace didn't open or schema_version was missing/wrong:
  // wipe the old namespace and mark phase 2 needed.
  nvs_handle_t old_h;
  if (nvs_open(STORAGE_OLD_NAMESPACE, NVS_READWRITE, &old_h) == ESP_OK) {
    nvs_erase_all(old_h);
    nvs_commit(old_h);
    nvs_close(old_h);
  }
  g_need_bond_wipe = true;
  printf("storage: first-boot phase 1: erased old NVS namespace\n");
  return ESP_OK;
}

esp_err_t storage_complete_migration(void) {
  if (!g_need_bond_wipe) {
    return ESP_OK;
  }

  int dev_num = esp_ble_get_bond_device_num();
  if (dev_num > 0) {
    esp_ble_bond_dev_t *bond_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    if (bond_list == NULL) {
      return ESP_ERR_NO_MEM;
    }
    esp_ble_get_bond_device_list(&dev_num, bond_list);
    for (int i = 0; i < dev_num; ++i) {
      esp_ble_remove_bond_device(bond_list[i].bd_addr);
    }
    free(bond_list);
  }

  nvs_handle_t h;
  esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &h);
  if (err != ESP_OK) {
    return err;
  }
  err = nvs_set_i32(h, SCHEMA_VERSION_KEY, SCHEMA_VERSION_VAL);
  if (err == ESP_OK) {
    err = nvs_commit(h);
  }
  nvs_close(h);

  if (err == ESP_OK) {
    g_need_bond_wipe = false;
    printf("storage: first-boot phase 2: erased %d bonds, schema_version=%d\n",
           dev_num, SCHEMA_VERSION_VAL);
  }
  return err;
}
```

`save_ble_device`, `read_ble_device`, `clear_ble_devices`, and their helpers `get_ble_status` / `set_ble_status` / `save_ble_results` / `read_ble_results` are unchanged — they already use `STORAGE_NAMESPACE` and pick up the bumped value automatically.

Add a `#include <stdlib.h>` near the top (for `malloc`/`free`) if not already present. Confirm:

```bash
grep -n stdlib.h main/storage.c
```

- [ ] **Step 3: Wire `storage_complete_migration()` into `init()`**

In `main/esp_hid_host_main.c::init` (around line 181), immediately after the existing `ESP_ERROR_CHECK(esp_hid_gap_init(HID_HOST_MODE));`, add:

```c
  ESP_ERROR_CHECK(storage_complete_migration());
```

- [ ] **Step 4: Build**

```bash
idf.py build
```

Expected: clean. The `esp_ble_get_bond_device_num` / `esp_ble_get_bond_device_list` / `esp_ble_remove_bond_device` / `esp_ble_bond_dev_t` symbols come from `esp_gap_ble_api.h`, which is already transitively included via `esp_hid_gap.h`.

- [ ] **Step 5: Commit**

```bash
git add main/storage.h main/storage.c main/esp_hid_host_main.c
git commit -m "Bump NVS namespace to hidproxy_v2 with two-phase first-boot migration"
```

---

## Task 8: Bounded scan waits + drive-by `add_ble_scan_result` fix

Replace `portMAX_DELAY` semaphore takes with a 3-second timeout so the connect loop can never hang on a missed controller event.

**Files:**
- Modify: `main/esp_hid_gap.c`

- [ ] **Step 1: Bound `WAIT_BLE_CB()` and propagate timeouts**

In `main/esp_hid_gap.c`, replace the macros around line 32–33:

```c
#define BLE_CB_TIMEOUT_MS 3000
#define WAIT_BLE_CB() xSemaphoreTake(ble_hidh_cb_semaphore, pdMS_TO_TICKS(BLE_CB_TIMEOUT_MS))
#define SEND_BLE_CB() xSemaphoreGive(ble_hidh_cb_semaphore)
```

Then update both callers to check the return value.

In `start_ble_scan` (around line 373), replace:

```c
static esp_err_t start_ble_scan(uint32_t seconds) {
  esp_err_t ret = ESP_OK;
  if ((ret = esp_ble_gap_set_scan_params(&hid_scan_params)) != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_gap_set_scan_params failed: %d", ret);
    return ret;
  }
  if (WAIT_BLE_CB() != pdTRUE) {
    ESP_LOGW(TAG, "BLE scan param-set timed out, retrying");
    return ESP_ERR_TIMEOUT;
  }

  if ((ret = esp_ble_gap_start_scanning(seconds)) != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_gap_start_scanning failed: %d", ret);
    return ret;
  }
  return ret;
}
```

In `esp_hid_scan` (around line 462), replace the `WAIT_BLE_CB();` line with:

```c
  if (start_ble_scan(seconds) == ESP_OK) {
    if (WAIT_BLE_CB() != pdTRUE) {
      ESP_LOGW(TAG, "BLE scan completion timed out, retrying");
      return ESP_ERR_TIMEOUT;
    }
  } else {
    return ESP_FAIL;
  }
```

- [ ] **Step 2: Drive-by fix in `add_ble_scan_result`**

Locate lines 161–165 of `esp_hid_gap.c`:

```c
  name_len = (name_len > NAME_LEN_MAX) ? NAME_LEN_MAX : name_len;
  if (memcpy(r->name, name, name_len) == NULL) {
    ESP_LOGE(TAG, "Copy name failed");
    return;
  }
```

Replace with:

```c
  if (name_len > NAME_LEN_MAX) {
    name_len = NAME_LEN_MAX;
  }
  memcpy(r->name, name, name_len);
```

(`memcpy` is specified to always return its first argument; the previous check could never trigger and leaked `r` on a false alarm.)

- [ ] **Step 3: Confirm the `hid_connect` loop tolerates `ESP_FAIL` / `ESP_ERR_TIMEOUT`**

Read `main/esp_hid_host_main.c::hid_connect` (around lines 134–179) and verify the loop body around `esp_hid_scan(...)` does not panic on a non-OK return — it does not currently check the return at all, so a non-OK simply falls through to `results_len == 0` and the loop iterates. No change needed. (Task 9 will replace this whole loop body anyway.)

- [ ] **Step 4: Build**

```bash
idf.py build
```

Expected: clean.

- [ ] **Step 5: Commit**

```bash
git add main/esp_hid_gap.c
git commit -m "Bound WAIT_BLE_CB at 3s; fix add_ble_scan_result name guard"
```

---

## Task 9: CONNECT state machine

The biggest behavior change. Move "connection succeeded" decisions out of the synchronous `esp_hidh_dev_open` return and into the async `ESP_HIDH_OPEN_EVENT`. Add a `CONNECTING` sub-state with a 10 s timeout and a single-slot 30 s backoff for failed candidates.

**Files:**
- Modify: `main/esp_hid_host_main.c`

- [ ] **Step 1: Add the `esp_timer.h` include**

At the top of `main/esp_hid_host_main.c`, find the include block (lines 13–34). Add (in alphabetical position among the `esp_*` headers):

```c
#include "esp_timer.h"
```

`esp_timer_get_time()` lives there and is used by the new state-machine logic.

- [ ] **Step 2: Add the new state constant and statics**

Around lines 36–46, update the state defines and add new statics. Final state of that region:

```c
#define BLE_STATUS_CONNECTED  1
#define BLE_STATUS_SCAN       2
#define BLE_STATUS_CONNECTING 3

#define BLE_CONNECTED_LED_MODE   LED_MODE_OFF
#define BLE_SCAN_NEW_LED_MODE    LED_MODE_FAST_BLINK
#define BLE_SCAN_SAVED_LED_MODE  LED_MODE_SLOW_BLINK
#define BLE_UNSET_LED_MODE       LED_MODE_ALWAYS_ON

#define CONNECT_TIMEOUT_US        (10LL * 1000 * 1000)
#define FAILED_DEVICE_BACKOFF_US  (30LL * 1000 * 1000)

static const char *TAG = "ESP_HIDH_DEMO";
static esp_hidh_dev_t *connected_dev = NULL;
static EspBleStatus ble_status;

static int64_t s_connecting_since_us = 0;
static esp_bd_addr_t s_failed_bda = {0};
static int64_t s_failed_at_us = 0;
```

- [ ] **Step 3: Rewrite `connect_hid_dev` as dispatch-only**

Replace the existing function (around lines 117–132) with:

```c
static bool connect_hid_dev(esp_hid_scan_result_t *cr) {
  esp_hidh_dev_t *dev =
      esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
  if (dev == NULL) {
    ESP_LOGE(TAG, "esp_hidh_dev_open returned NULL for " ESP_BD_ADDR_STR,
             ESP_BD_ADDR_HEX(cr->bda));
    return false;
  }
  ble_status.status = BLE_STATUS_CONNECTING;
  s_connecting_since_us = esp_timer_get_time();
  ESP_LOGI(TAG, "CONNECTING to " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(cr->bda));
  return true;
}
```

- [ ] **Step 4: Move success logic into `ESP_HIDH_OPEN_EVENT` and handle failure**

Replace the `case ESP_HIDH_OPEN_EVENT:` block in `hidh_callback` (around lines 54–66):

```c
  case ESP_HIDH_OPEN_EVENT: {
    if (param->open.status == ESP_OK) {
      const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
      ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda),
               esp_hidh_dev_name_get(param->open.dev));
      esp_hidh_dev_dump(param->open.dev, stdout);

      save_ble_device((uint8_t *)bda);
      set_led_mode(BLE_CONNECTED_LED_MODE);
      ble_status.status = BLE_STATUS_CONNECTED;
      connected_dev = param->open.dev;
    } else {
      const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
      ESP_LOGE(TAG, ESP_BD_ADDR_STR " OPEN failed: status=0x%x",
               ESP_BD_ADDR_HEX(bda), param->open.status);
      memcpy(s_failed_bda, bda, sizeof(esp_bd_addr_t));
      s_failed_at_us = esp_timer_get_time();
      esp_hidh_dev_free(param->open.dev);
      ble_status.status = BLE_STATUS_SCAN;
    }
    break;
  }
```

- [ ] **Step 5: Update `hid_connect` to honor `CONNECTING` and backoff**

Replace the entire `hid_connect` task (around lines 134–179) with:

```c
void hid_connect(void *pvParameters) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(400));

    if (ble_status.status == BLE_STATUS_CONNECTED) {
      continue;
    }

    if (ble_status.status == BLE_STATUS_CONNECTING) {
      if ((esp_timer_get_time() - s_connecting_since_us) > CONNECT_TIMEOUT_US) {
        ESP_LOGW(TAG, "CONNECTING timed out after 10s — back to SCAN");
        ble_status.status = BLE_STATUS_SCAN;
      }
      continue;
    }

    size_t results_len = 0;
    esp_hid_scan_result_t *results = NULL;
    ESP_LOGI(TAG, "SCAN...");

    esp_bd_addr_t addr;
    memset(addr, 0, sizeof(esp_bd_addr_t));
    bool has_con_dev = read_ble_device(addr);
    if (has_con_dev) {
      ESP_LOGI(TAG, "FOUND SAVED BLE DEVICE: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(addr));
      set_led_mode(BLE_SCAN_SAVED_LED_MODE);
      esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results, addr);
    } else {
      ESP_LOGI(TAG, "TRY CONNECT TO NEW BLE DEVICE");
      set_led_mode(BLE_SCAN_NEW_LED_MODE);
      esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results, NULL);
    }

    ESP_LOGI(TAG, "SCAN: %u results", results_len);
    if (results_len) {
      int64_t now = esp_timer_get_time();
      esp_hid_scan_result_t *r = results;
      while (r) {
        print_esp_hid_scan_results(r);

        bool backoff_active =
            (memcmp(r->bda, s_failed_bda, sizeof(esp_bd_addr_t)) == 0) &&
            ((now - s_failed_at_us) < FAILED_DEVICE_BACKOFF_US);

        if (backoff_active) {
          ESP_LOGI(TAG, "Skipping " ESP_BD_ADDR_STR " (backoff)", ESP_BD_ADDR_HEX(r->bda));
          r = r->next;
          continue;
        }

        if (connect_hid_dev(r)) {
          // Dispatched; wait for OPEN_EVENT to decide success/failure.
          break;
        }
        r = r->next;
      }
      esp_hid_scan_results_free(results);
    }
  }
}
```

- [ ] **Step 6: Remove obsolete logic from the old `connect_hid_dev` callers**

The old loop set `ble_status.status = BLE_STATUS_CONNECTED` when `connect_hid_dev` returned true. The new loop does **not** — `connect_hid_dev` now transitions to `BLE_STATUS_CONNECTING`, and `OPEN_EVENT` advances to `BLE_STATUS_CONNECTED`. The block above already reflects this; just double-check no other site flips the status to `CONNECTED` outside `OPEN_EVENT`. Run:

```bash
grep -n 'BLE_STATUS_CONNECTED' main/esp_hid_host_main.c
```

Expected: only one assignment, inside the `ESP_HIDH_OPEN_EVENT` success branch.

- [ ] **Step 7: Build**

```bash
idf.py build
```

Expected: clean. If `esp_hidh_dev_free` is unknown, confirm `#include "esp_hidh.h"` is present (it already is, line 33).

- [ ] **Step 8: Commit**

```bash
git add main/esp_hid_host_main.c
git commit -m "Add CONNECTING state, OPEN_EVENT-driven success, 30s backoff slot"
```

---

## Task 10: Stack + watchdog hardening

Three small, independent tweaks.

**Files:**
- Modify: `main/esp_hid_host_main.c`
- Modify: `sdkconfig.defaults`

- [ ] **Step 1: Bump `hid_connect` stack 6KB → 8KB**

In `main/esp_hid_host_main.c::app_main`, change:

```c
xTaskCreate(&hid_connect, "hid_connect", 6 * 1024, NULL, 2, NULL);
```

to:

```c
xTaskCreate(&hid_connect, "hid_connect", 8 * 1024, NULL, 2, NULL);
```

- [ ] **Step 2: Add optional heap-watermark logging**

In `main/esp_hid_host_main.c::hid_connect`, just inside the top of the `while (true)` loop (immediately after `vTaskDelay(pdMS_TO_TICKS(400));`):

```c
#ifdef DEBUG_HEAP
    static int64_t s_last_heap_log_us = 0;
    int64_t heap_now = esp_timer_get_time();
    if ((heap_now - s_last_heap_log_us) > (30LL * 1000 * 1000)) {
      ESP_LOGI(TAG, "free heap: %u", (unsigned)esp_get_free_heap_size());
      s_last_heap_log_us = heap_now;
    }
#endif
```

(`esp_get_free_heap_size` is in `esp_system.h`, already included.)

- [ ] **Step 3: Enable stack-overflow checking in `sdkconfig.defaults`**

Append:

```
CONFIG_FREERTOS_CHECK_STACKOVERFLOW_PTRVAL=
CONFIG_FREERTOS_CHECK_STACKOVERFLOW_CANARY=y
```

Notes: ESP-IDF expresses stack-overflow check level 2 as `CONFIG_FREERTOS_CHECK_STACKOVERFLOW_CANARY=y` (the alternative is `_PTRVAL` for level 1). Setting `_CANARY=y` and leaving `_PTRVAL` blank is the documented way to select level 2. If `idf.py reconfigure` rejects this form, fall back to the legacy single-symbol form:

```
CONFIG_FREERTOS_CHECK_STACKOVERFLOW=2
```

(Either form ends up generating the same `configCHECK_FOR_STACK_OVERFLOW = 2` in the FreeRTOS config.)

- [ ] **Step 4: Reconfigure and build**

```bash
idf.py reconfigure && idf.py build
```

Expected: clean build. The reconfigure step regenerates `sdkconfig` from defaults — confirm with:

```bash
grep STACKOVERFLOW sdkconfig
```

Expected: `CONFIG_FREERTOS_CHECK_STACKOVERFLOW_CANARY=y` (or `CONFIG_FREERTOS_CHECK_STACKOVERFLOW=2`, depending on which form your IDF version uses).

- [ ] **Step 5: Commit**

```bash
git add main/esp_hid_host_main.c sdkconfig.defaults
git commit -m "Harden runtime: 8KB hid_connect stack, stack-overflow canary, heap log"
```

---

## Task 11: Build script + README polish

**Files:**
- Modify: `build_flash_monitor.sh`
- Modify: `README.org`

- [ ] **Step 1: Make `build_flash_monitor.sh` honor `$PORT`**

Replace the contents with:

```sh
#!/usr/bin/env sh
set -e
PORT="${PORT:-/dev/ttyUSB0}"
idf.py -p "$PORT" -b 115200 flash monitor
```

- [ ] **Step 2: Update `README.org`**

Open `README.org` and make four edits:

(a) Replace the "Which device will be selected" bullet `4. No authentication is required` — delete that line and follow it with a new bullet:

```
  4. LESC bonding (Just Works); no MITM
```

(b) Update the flash example block to:

```
#+begin_src bash
PORT=/dev/ttyACM1 ./build_flash_monitor.sh
#+end_src
```

(or keep the existing `idf.py …` line in addition, whichever you prefer — the spec mentions the env-var form).

(c) Remove the two fixed items from the `* Known issues` section:

- `Crash if the opened keyboard is already connected to another hid_proxy.`
- `LED mode change delay after pressing the boot button to erase.`

Leave the `* Known issues` heading if any other items remain, otherwise delete the section.

(d) Add a `* Tests` section after `* Build and flash`:

```
* Tests
Host-side unit tests for the pure helpers (=ch9329_pack=, =led_duty=) build with plain gcc — no ESP-IDF required.

#+begin_src bash
cd test/host && cmake -B build && cmake --build build && ./build/run_tests
#+end_src
```

- [ ] **Step 3: Commit**

```bash
git add build_flash_monitor.sh README.org
git commit -m "Honor PORT env var; refresh README (security, tests, known issues)"
```

---

## Task 12: Self-check before hardware verification

Quick sanity sweep to catch any leftover references to removed symbols before flashing.

- [ ] **Step 1: Confirm no callers reference the removed APIs**

```bash
grep -rn 'set_update_led\|g_packed_data\|FIXED_CH9329_DATA_LEN' main/ test/host/
```

Expected: no matches.

- [ ] **Step 2: Confirm there is exactly one definition of `pack_ch9329_data`**

```bash
grep -rn 'pack_ch9329_data' main/ test/host/
```

Expected: declaration in `main/ch9329_pack.h`, definition in `main/ch9329_pack.c`, call site in `main/esp_hid_host_main.c`, and references in the test file — and **no** definition in `main/ch932x.c`.

- [ ] **Step 3: Final host-test run**

```bash
cd test/host && cmake --build build && ./build/run_tests
```

Expected: `12 Tests 0 Failures 0 Ignored OK`.

- [ ] **Step 4: Final firmware build**

```bash
idf.py build
```

Expected: clean. Note the binary size for comparison after first flash.

No commit needed (no file changes).

---

## Task 13: On-device verification (manual)

The user runs each step on the actual hardware at `/dev/ttyACM1` (or wherever `$PORT` points). Tick each checkbox as the corresponding test passes. If any step fails, stop and diagnose — do not flash forward.

- [ ] **Step 1: Flash the new firmware**

```bash
PORT=/dev/ttyACM1 ./build_flash_monitor.sh
```

Expected: flash success, monitor attaches.

- [ ] **Step 2: First-boot migration log appears once**

In the monitor output of the first boot after flashing, look for the two log lines:

```
storage: first-boot phase 1: erased old NVS namespace
storage: first-boot phase 2: erased N bonds, schema_version=2
```

Then reboot the proxy (Ctrl-T, Ctrl-R in `idf.py monitor`, or pull/restore power). Confirm those lines do **not** reappear.

- [ ] **Step 3: New-device pairing (with NVS clean)**

Power-cycle the proxy with no saved keyboard, put a fresh ZMK keyboard into pairing mode. Expect an `OPEN: <name>` log, LED → off (steady), keystrokes flow through to the CH9329/CH9328.

- [ ] **Step 4: Reconnect (no re-pair)**

Power-cycle the proxy with the keyboard already paired. Within ~5–10 s, expect `OPEN: <name>` and LED → off. No keyboard-side re-pair gesture should be needed.

- [ ] **Step 5: Already-connected-elsewhere target fails cleanly**

Pair the keyboard to a phone first, then power-cycle the proxy. Expect:
- `OPEN failed: status=0x...` log.
- No reboot, no panic.
- Backoff log `Skipping <MAC> (backoff)` on subsequent scans for the next ~30 s.

Disconnect the keyboard from the phone; the proxy should pick it up on the next scan after the backoff window expires.

- [ ] **Step 6: LED responsiveness**

While in slow-blink mode (saved-device search), press the boot button (IO9) and hold for ~1 s. The LED should switch to fast-blink within roughly one 50 ms tick (visually instant — well under the previous ~1 s lag).

- [ ] **Step 7: Watchdog soak**

Type continuously for 5 minutes. Expect no reboots, no `Task watchdog got triggered` panics. Optionally, edit `main/esp_hid_host_main.c` and add `#define DEBUG_HEAP 1` at the top, rebuild, and observe the periodic `free heap: …` log staying flat (within ±a few hundred bytes) over 30 minutes. Remove the define before committing.

- [ ] **Step 8: Scan-deadlock recovery**

While the proxy is mid-scan, power off the keyboard. Expect a `BLE scan ... timed out, retrying` log and recovery within one scan cycle (no infinite hang).

- [ ] **Step 9: Note any deviations**

If a verification step diverges from the expected behavior, capture the relevant log lines and stop. The most likely culprits:
- Pairing fails after Step 4 → ZMK + RPA edge case (spec §8). Likely needs an `ESP_GAP_BLE_AUTH_CMPL_EVT` handler.
- Reboot loop after first flash → schema migration didn't complete; manually `idf.py erase-flash` once and re-flash.
- LED stuck → confirm the `change_led` task is still running (heap-log periodic print is a good liveness signal).

No commit at the end of this task — the work is complete and committed task-by-task. Optional: tag the commit at the head of `main` (`git tag robustness-pass-2026-05-24`) for traceability.

---

## Cross-task invariants (skim before executing)

These conditions must remain true at the end of every task. If a build or test step fails to honor any of these, you've drifted — stop and reconcile before continuing.

- `LED_MODE_*` integer values must match between `main/led.h` and `main/led_duty.h`. (Source of truth: `main/led.h`.)
- `CH9329_HEADER_LEN` and `CH9329_MAX_PAYLOAD` must agree between `main/ch9329_pack.h` and the size of `s_packed_buf` in `main/ch932x.c`.
- Only one symbol named `pack_ch9329_data` in the firmware build — defined in `main/ch9329_pack.c`, declared in `main/ch9329_pack.h`.
- `ble_status.status` is written to `BLE_STATUS_CONNECTED` from exactly one place: the success branch of `ESP_HIDH_OPEN_EVENT`.
- Host tests `cd test/host && cmake --build build && ./build/run_tests` pass after every task that touches `main/ch9329_pack.c` or `main/led_duty.c`.
- `idf.py build` succeeds with no new warnings on touched files after every task.
