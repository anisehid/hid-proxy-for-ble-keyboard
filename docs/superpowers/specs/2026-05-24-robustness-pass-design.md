# Robustness Pass — Design Spec

**Date:** 2026-05-24
**Scope:** First of four planned passes on `hid-proxy-for-ble-keyboard`. This pass fixes user-visible reliability symptoms and the latent bugs likely causing them. Subsequent passes (not in scope here): multi-device support, BLE mouse, web config server.

---

## 1. Background

ESP32-C3 BLE HID host that relays BLE keyboard input over UART to a CH9329/CH9328 (USB HID emulator). Tested against ZMK keyboards. Current pain points reported by the user:

- New-device pairing fails with auth errors.
- Reconnect to a previously-paired keyboard is unreliable.
- Occasional watchdog reboots under normal use.
- (From README) Crash when the target keyboard is already paired with another host.
- (From README) LED takes up to ~1s to reflect a mode change after the boot button is pressed.

Code review surfaced four latent bugs likely contributing to the above:

- `led.c::get_led_mode` writes the parameter pointer back to itself instead of the file-static `update_led` flag — callers always receive `true`.
- `ch932x.c::pack_ch9329_data` allocates on the heap on every keystroke (and on every length change), with no lock, from the HIDH event-task context — a known source of long-runtime crashes on Bluedroid.
- `esp_hid_gap.c::start_ble_scan` calls `WAIT_BLE_CB()` (`portMAX_DELAY`) — if the controller event never fires, the `hid_connect` task hangs forever.
- `esp_hid_host_main.c::connect_hid_dev` reports `CONNECT SUCCESS` and writes the device MAC to NVS *before* the async `ESP_HIDH_OPEN_EVENT` confirms the connection. A failed open leaks the half-open device handle.

A fifth root cause for the auth/reconnect symptoms: **`esp_ble_gap_set_security_param` is never called.** SMP is enabled in `sdkconfig` but the application never configures bonding parameters, IO capability, or key distribution. The official `esp_hid_host` example sets these; this fork dropped them. Without bonding parameters, a modern BLE keyboard (ZMK, most others) will either reject the connection or accept it once and fail to reconnect because no keys were exchanged or stored.

## 2. Goals

1. New BLE keyboards pair successfully on first attempt.
2. Previously-paired keyboards reconnect without re-entering pairing mode.
3. Already-paired-elsewhere target devices fail cleanly and don't crash the proxy.
4. LED reflects mode changes within ≤50ms.
5. No keystroke-path heap allocation; no infinite waits in the scan path.
6. Bug classes from the latent issues above are guarded by host-side unit tests.

## 3. Non-Goals

- Multi-device support (separate spec).
- BLE mouse support (separate spec).
- Web config UI (separate spec).
- Structural refactor into new modules (`ble_host`, `input_pipeline`, NVS owner thread) — earned incrementally in future passes.
- Mocking BLE/GAP/GATTC for tests. Not worth the scaffolding for a single-developer project.
- GitHub Actions / CI setup.

## 4. Detailed Design

### 4.1 BLE bonding & security

In `esp_hid_gap.c::init_low_level`, after `esp_bluedroid_enable()` succeeds, configure security:

| Param | Value | Reason |
|---|---|---|
| `ESP_BLE_SM_AUTHEN_REQ_MODE` | `ESP_LE_AUTH_REQ_SC_BOND` | LESC + bondable, no MITM. Matches ZMK's "Just Works" default. |
| `ESP_BLE_SM_IOCAP_MODE` | `ESP_IO_CAP_NONE` | No display, no keyboard input on the proxy. |
| `ESP_BLE_SM_MAX_KEY_SIZE` | `16` | Standard. |
| `ESP_BLE_SM_SET_INIT_KEY` | `ESP_BLE_ENC_KEY_MASK \| ESP_BLE_ID_KEY_MASK` | Persist encryption + identity keys across reboots. |
| `ESP_BLE_SM_SET_RSP_KEY` | `ESP_BLE_ENC_KEY_MASK \| ESP_BLE_ID_KEY_MASK` | Same, peer-side. |

Bond keys are stored by Bluedroid in its own NVS partition; the application does not manage them directly. The existing `ESP_GAP_BLE_SEC_REQ_EVT` handler (accepts the security request) remains correct. The passkey display / notify handlers stay as no-op log lines — they only fire if a peer advertises with display IO cap, which is unusual but harmless.

The README line "No authentication is required" is removed; pairing now uses LESC bonding.

### 4.2 NVS schema bump + first-boot wipe

Current `storage.c` uses namespace `"storage"` with keys `ble_status` (int32) and `ble_results` (blob of one MAC). After the bonding change, the proxy's stored MAC must agree with Bluedroid's bond list — old-firmware MACs have no corresponding bond and would loop forever.

**Change:**
1. Bump `STORAGE_NAMESPACE` from `"storage"` to `"hidproxy_v2"`.
2. Add a `SCHEMA_VERSION` key (`schema_version`, int32, value `2`).
3. In `init_nvs_flash()`, after `nvs_flash_init()`:
   - Open `"hidproxy_v2"`, read `schema_version`.
   - If missing: this is a first boot on new firmware. Open and `nvs_erase_all` on the old `"storage"` namespace, then iterate `esp_ble_get_bond_device_list` and call `esp_ble_remove_bond_device` for each entry, then write `schema_version = 2` to the new namespace.
   - Log: `"first-boot migration: erased old NVS, erased N bonds"`.
4. `BLE_RESULTS_STORAGE` and `BLE_STATUS` keys keep their names inside the new namespace.

Public API (`save_ble_device`, `read_ble_device`, `clear_ble_devices`) does not change.

User impact: one re-pair required after flashing the update.

### 4.3 Already-connected target crash

Restructure the connect path so success is determined by `ESP_HIDH_OPEN_EVENT`, not by the synchronous return of `esp_hidh_dev_open`.

**`esp_hid_host_main.c::connect_hid_dev`** becomes "request open and return immediately":

- Call `esp_hidh_dev_open(...)`.
- Do **not** call `save_ble_device` here.
- Do **not** set `ble_status = CONNECTED` here.
- Do **not** call `set_led_mode(BLE_CONNECTED_LED_MODE)` here.
- Return only whether the open call was dispatched.

**`hidh_callback::ESP_HIDH_OPEN_EVENT`** owns the success/failure transition:

- On `param->open.status == ESP_OK`:
  - `save_ble_device(bda)`.
  - `set_led_mode(BLE_CONNECTED_LED_MODE)`.
  - `ble_status.status = BLE_STATUS_CONNECTED`.
  - `connected_dev = param->open.dev`.
- On failure:
  - Log BDA + `param->open.status`.
  - `esp_hidh_dev_free(param->open.dev)` to release the half-open handle.
  - Record `bda` and `now` in a one-slot "backoff" static (`static esp_bd_addr_t failed_bda; static int64_t failed_at_us;`).
  - Leave `ble_status` at `SCAN`.

**`hid_connect` loop** consults the backoff before attempting `connect_hid_dev`:

- If candidate's `bda` equals `failed_bda` and `(esp_timer_get_time() - failed_at_us) < 30_000_000`, skip this candidate.
- Otherwise proceed.

**Backoff capacity**: single slot. Single-device proxy; revisited in the multi-device pass.

**Connect wait**: after dispatching open, `hid_connect` polls `ble_status` for up to 5s (10 × 500ms ticks); if still `SCAN`, give up and re-scan. The OPEN_EVENT handler is what actually changes the status.

### 4.4 LED bugs

**Fix the pointer bug.** Rename the file-static `update_led` to `g_update_led`. In `get_led_mode`, write `*update_led = g_update_led;`. Same rename in `set_update_led` and `set_led_mode`.

**Restructure `change_led` as a single-tick loop.** Today each mode runs its own `while (true) { ledc_blink(...); }` whose own `vTaskDelay` determines the response latency to a mode change. Replace with one loop:

```c
void change_led(void *_) {
  int last_mode = -1;
  int phase = 0;
  while (true) {
    int mode;
    get_led_mode(&mode, NULL);
    if (mode != last_mode) { phase = 0; last_mode = mode; }
    int duty = led_duty_for(mode, phase);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    phase++;
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}
```

`led_duty_for(int mode, int tick)` is a new pure function exposed in `led.h`:

| Mode | Behavior |
|---|---|
| `LED_MODE_OFF` | `LEDC_DUTY_DIME` always. |
| `LED_MODE_ALWAYS_ON` | `LEDC_DUTY_ON` always. |
| `LED_MODE_FAST_BLINK` | `LEDC_DUTY_ON` if `(tick / 2) % 2 == 0` else `LEDC_DUTY_DIME` (100ms toggle at 50ms tick). |
| `LED_MODE_SLOW_BLINK` | `LEDC_DUTY_ON` if `(tick / 20) % 2 == 0` else `LEDC_DUTY_DIME` (1000ms toggle). |

Mode changes take effect within one 50ms tick.

The `update_led` flag becomes redundant (the loop detects mode changes itself) and is removed: `get_led_mode`'s second parameter goes away, callers updated, `set_update_led` deleted.

### 4.5 Hot-path malloc

Replace `g_packed_data` (heap) with a file-static buffer in `ch932x.c`:

```c
static uint8_t s_packed_buf[FIXED_CH9329_DATA_LEN + 64];
```

CH9329 protocol caps payload at 64 bytes; 70 bytes total is acceptable in BSS.

`pack_ch9329_data` becomes a pure function:

```c
int pack_ch9329_data(CH9329CMD cmd, const uint8_t *data, int length, uint8_t *out);
```

- Returns total packed length (`length + 6`) on success.
- Returns `0` if `length < 0 || length > 64` (caller logs and drops the report).
- Writes header, addr, cmd, len, payload, checksum to `out`.

Default call site in `hidh_callback` passes `s_packed_buf`:

```c
int n = pack_ch9329_data(CMD_SEND_KB_GENERAL_DATA, param->input.data, param->input.length, s_packed_buf);
if (n > 0) send_data_to_uart(s_packed_buf, n);
```

The `g_packed_data` / `g_packed_data_len` externs are removed from `ch932x.h`.

### 4.6 Scan deadlock

Replace the `WAIT_BLE_CB()` macro (`xSemaphoreTake(..., portMAX_DELAY)`) with a 3-second bounded wait in `start_ble_scan` and `esp_hid_scan`. On timeout:

- Log `"BLE scan param/start timed out, retrying"`.
- Return `ESP_ERR_TIMEOUT`.
- The `hid_connect` loop iterates and tries the next scan from scratch.

The `WAIT_BLE_CB()` / `SEND_BLE_CB()` macros stay (with a 3s timeout in `WAIT_BLE_CB`) because they're used in multiple places.

### 4.7 Stack + watchdog

- Bump the `hid_connect` task stack from 6KB to 8KB in `app_main` (`xTaskCreate(..., 8 * 1024, ...)`).
- Enable `CONFIG_FREERTOS_CHECK_STACKOVERFLOW=2` in `sdkconfig.defaults` — panics with a clear message on overflow instead of corrupting nearby memory; ~zero runtime cost.
- Add an optional heap-watermark log (`ESP_LOGI(TAG, "free heap: %u", esp_get_free_heap_size())`) under `#ifdef DEBUG_HEAP` in `hid_connect`, every ~30s. Off by default.

### 4.8 Build script

`build_flash_monitor.sh`:

```sh
#!/usr/bin/env sh
set -e
PORT="${PORT:-/dev/ttyUSB0}"
idf.py -p "$PORT" -b 115200 flash monitor
```

README updated to mention `PORT=/dev/ttyACM1 ./build_flash_monitor.sh`.

### 4.9 Drive-by fix

While in `esp_hid_gap.c`, remove the misleading `if (memcpy(...) == NULL)` check in `add_ble_scan_result` (it can never be NULL) and replace with a `name_len <= NAME_LEN_MAX` guard before the `memcpy`.

## 5. Tests (host-side, pure logic)

A new top-level `test/host/` directory builds the pure-function pieces against host gcc with Unity (vendored from `$IDF_PATH/components/unity/unity/src/`; if that path is unstable, vendor `unity.c`/`unity.h` into `test/host/vendor/`).

**Suite 1 — `test_ch932x.c`** (~5 cases):
- Header bytes are `0x57 0xAB`.
- Address byte is `0x00`.
- Command byte matches the enum.
- Length byte matches the payload length.
- Checksum equals `sum(packet[0..n-1]) & 0xFF`.
- `length > 64` returns 0; output buffer untouched (or zeroed before — test doesn't assert).
- `length == 0` returns 6, no payload bytes written.

**Suite 2 — `test_led.c`** (~6 cases):
- `led_duty_for(LED_MODE_OFF, *)` always returns the dim duty.
- `led_duty_for(LED_MODE_ALWAYS_ON, *)` always returns the on duty.
- `led_duty_for(LED_MODE_FAST_BLINK, tick)` toggles every 2 ticks; assert `[0]=on, [2]=dim, [4]=on, [6]=dim`.
- `led_duty_for(LED_MODE_SLOW_BLINK, tick)` toggles every 20 ticks; assert `[0]=on, [20]=dim, [40]=on, [60]=dim`.
- Unknown mode returns the dim duty (default-safe).

**Infrastructure:**
- `test/host/CMakeLists.txt`: host toolchain, compiles `main/ch932x.c` and `main/led.c` plus a stubs include dir (`test/host/stubs/`) with empty inline shims for `esp_log.h`, `driver/uart.h`, `driver/ledc.h`, `freertos/*.h` — the pure functions never call into these at runtime, only include them at compile time.
- `test/host/runner.c`: Unity main; registers both suites.
- Build/run: `cd test/host && cmake -B build && cmake --build build && ./build/run_tests`.
- README gets a "Tests" section with the above command.

What is **not** tested here: BLE/GAP/GATTC, NVS migration (shallow; on-target verification cost ≈ host test cost), UART send, FreeRTOS task loops, the LED `change_led` task itself (only the extracted `led_duty_for` calculator).

## 6. File-by-File Delta

| File | Change |
|---|---|
| `main/esp_hid_gap.c` | Add `esp_ble_gap_set_security_param` block (§4.1). Bound `WAIT_BLE_CB()` with 3s timeout (§4.6). Fix the `memcpy() == NULL` check (§4.9). |
| `main/storage.c` / `main/storage.h` | Bump namespace to `hidproxy_v2`; add `schema_version` key + first-boot migration that erases old namespace and Bluedroid bonds (§4.2). |
| `main/esp_hid_host_main.c` | Move `save_ble_device` + `ble_status = CONNECTED` + `set_led_mode` into OPEN_EVENT success branch (§4.3). Handle OPEN_EVENT failure: log + `esp_hidh_dev_free` + 30s backoff slot (§4.3). `connect_hid_dev` becomes dispatch-only (§4.3). Bump `hid_connect` stack 6→8KB (§4.7). Optional heap-log under `#ifdef DEBUG_HEAP` (§4.7). Pass `s_packed_buf` to the new pure `pack_ch9329_data` (§4.5). |
| `main/led.c` / `main/led.h` | Rename file-static to `g_update_led` (§4.4). Extract pure `led_duty_for(mode, tick)` (§4.4, §5). Restructure `change_led` as single 50ms-tick loop (§4.4). Drop `update_led` parameter from `get_led_mode`; delete `set_update_led`. |
| `main/ch932x.c` / `main/ch932x.h` | Replace `g_packed_data`/`g_packed_data_len` globals with file-static `s_packed_buf[70]`. `pack_ch9329_data` becomes pure function returning packed length, with `length > 64` guard (§4.5). Remove externs from header. |
| `sdkconfig.defaults` | Add `CONFIG_FREERTOS_CHECK_STACKOVERFLOW=2` (§4.7). |
| `build_flash_monitor.sh` | `PORT="${PORT:-/dev/ttyUSB0}"` (§4.8). |
| `README.org` | Update flash example to mention `PORT=…`. Remove the two fixed items from "Known issues". Remove "No authentication is required". Add "Tests" section with the host-test command. |
| `test/host/CMakeLists.txt` | New: host-build target with Unity + pure-function sources + stubs include dir. |
| `test/host/stubs/esp_log.h`, `test/host/stubs/driver/uart.h`, `test/host/stubs/driver/ledc.h`, `test/host/stubs/freertos/*.h` | New: empty inline shims. |
| `test/host/test_ch932x.c` | New: ~5 Unity test cases. |
| `test/host/test_led.c` | New: ~6 Unity test cases. |
| `test/host/runner.c` | New: Unity main. |

## 7. Verification Plan

All hardware verification is manual, user-driven, on the device at `/dev/ttyACM1`.

1. **Build + flash**: `PORT=/dev/ttyACM1 ./build_flash_monitor.sh`. Expect compile clean, flash success.
2. **Host unit tests**: `cd test/host && cmake -B build && cmake --build build && ./build/run_tests`. All cases pass.
3. **First-boot migration**: serial log shows `"first-boot migration: erased old NVS, erased N bonds"`. Reboot. Confirm no migration log on second boot.
4. **New-device pairing**: with NVS clean, scan + connect a fresh keyboard. Expect log `OPEN: <name>`, LED → off, keystrokes flow.
5. **Reconnect**: reboot the proxy without re-pairing the keyboard. Expect re-connect within a scan cycle (~5–10s), no keyboard-side re-pair.
6. **Already-connected target**: pair the keyboard to a phone first, then power-cycle the proxy. Expect log of OPEN failure, no crash, 30s backoff, scan loop continues. Disconnect the phone; proxy picks up the keyboard on next scan.
7. **LED responsiveness**: while in slow-blink (saved-device search), press boot button for 1s. Expect LED → fast-blink within ~50ms.
8. **Watchdog soak**: type for 5 minutes; observe no reboot. Optional: define `DEBUG_HEAP` and watch `free heap` stay flat over 30 minutes.
9. **Scan-deadlock recovery**: yank power from the keyboard mid-scan. Expect `"BLE scan ... timed out, retrying"` log, recovery within one scan cycle.

## 8. Risks

- **ZMK + RPA**: some ZMK builds advertise with random-resolvable addresses. If pairing succeeds but reconnect fails after step 5, we may need to handle `ESP_GAP_BLE_AUTH_CMPL_EVT` and refresh the resolved peer address via `esp_ble_gap_read_rssi` / identity-address lookup. Will diagnose if it appears.
- **Bond-list capacity**: Bluedroid defaults to 8 bonds, rotates oldest-out. Fine for single-device; revisit in the multi-device pass.
- **Backoff UX**: power-cycling the proxy after using the keyboard elsewhere produces a one-time ~30s delay before scan-new succeeds. Trade-off vs. tight-looping a failing connect.
- **Stubbed headers drift**: if ESP-IDF changes the signatures of functions our pure code transitively references at compile time, the stubs need updating. The pure functions only use `uint8_t`, `int`, basic C — risk is low.
- **NVS migration race**: if power is lost mid-migration, the new namespace may have erased bonds but not yet written `schema_version`. Next boot repeats the migration — idempotent. Acceptable.

## 9. Out-of-Scope (Tracked for Future Specs)

- Multi-device support (storage of N bonded keyboards, switching UX).
- BLE mouse support (CH9329 mouse commands, dual report routing).
- Web/Wi-Fi config server (device list, pick/forget, status).
- Structural refactor into `ble_host` / `input_pipeline` / NVS owner thread.
- On-target Unity tests, pytest-embedded integration tests, GitHub Actions CI.
