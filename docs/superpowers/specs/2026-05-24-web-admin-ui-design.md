# Web Admin UI — Design Spec

**Date:** 2026-05-24
**Author:** Brainstorming session with assistant
**Status:** Draft for review

## 1. Goal

Add a browser-based admin UI to the hid_proxy so users can:
- Pair multiple BLE keyboards and switch between them.
- See current connection status and manage saved devices.
- Authenticate before performing any admin action.

The UI is served from the proxy itself via a triggered Wi-Fi soft-AP. There is no cloud, no router dependency, no Wi-Fi credentials onboarding.

## 2. Non-goals

- Wi-Fi station mode (joining a home network).
- Captive-portal DNS hijack.
- HTTPS (LAN-only over WPA2 is deemed sufficient).
- Per-user accounts. Single shared admin password.
- Web-based firmware OTA.
- Mouse support (covered by a separate todo).

## 3. User-visible behavior

### 3.1 Runtime modes

Two modes; the boot button (IO9) triggers transitions:

```
   ┌─────────────────────────────────────────────────────┐
   │  RELAY (default after boot)                         │
   │  • BLE scan loop runs continuously                  │
   │  • Auto-reconnects to any *saved* device only       │
   │  • Wi-Fi radio off                                  │
   └────────────┬────────────────────┬───────────────────┘
       IO9 1s   │                    │ IO9 triple-tap
                ▼                    ▼
   ┌──────────────────┐   ┌──────────────────────────────┐
   │  Forget device   │   │  ADMIN (web UI up)           │
   │  (action only)   │   │  • Wi-Fi soft-AP on          │
   │  • Drop active   │   │  • HTTP server up            │
   │  • Stays in RELAY│   │  • Current BLE conn kept     │
   └──────────────────┘   │  • Saved-device reconnect    │
                          │    keeps running             │
       IO9 5s             │  • Discovery of *new*        │
       │                  │    devices only when web-UI  │
       ▼                  │    toggle is ON              │
   ┌──────────────────┐   │  • 10-minute idle → back to  │
   │  Factory reset   │   │    RELAY                     │
   │  • Wipe NVS      │   └──────────────────────────────┘
   │  • Remove bonds  │
   │  • Drop pw hash  │
   │  • Reboot        │
   └──────────────────┘
```

The button is **the only** way to enter ADMIN. There is no factory default that exposes the AP.

### 3.2 Button gestures

| Gesture | Action | LED feedback during gesture |
|---|---|---|
| Hold IO9 ≥1s and release before 5s | Forget active device | Fast blink after 1s ("release now to forget") |
| Hold IO9 ≥5s | Factory reset + reboot | All-on after 5s ("release now to factory-reset") |
| Three taps within 1.5s (each <300ms) | Enter ADMIN mode | Slow blink for ~1s on entry |

Tap = press-and-release within 300ms. Press exceeding 300ms cancels any in-progress triple-tap counter. The gesture detector is a small explicit state machine (see §6.2).

### 3.3 LED scheme additions

The existing LED modes (off / always-on / fast-blink / slow-blink) gain one ADMIN-mode pattern: **double-pulse every 2 s** (e.g., on-100ms / off-100ms / on-100ms / off-1700ms). Distinct from all four current modes so users can tell at a glance that the AP is up.

### 3.4 Joining the AP

- SSID: `hidproxy-XXXX` where `XXXX` = last 4 hex digits of the chip MAC.
- WPA2 password: deterministically derived from the chip MAC (e.g., last 8 hex digits, lowercase). Printed on serial each boot.
- Once joined, the proxy is reachable at `http://192.168.4.1` (esp_netif default for soft-AP).

### 3.5 First-time setup vs. subsequent use

- First ADMIN entry after factory reset / fresh flash: `/api/auth/init` is open; UI shows "Set admin password" (≥8 chars). Until set, no other API works.
- Subsequent entries: `/login` form; valid login sets an HMAC-signed session cookie, 30-min sliding expiry.

### 3.6 In ADMIN: discovering and pairing new keyboards

Saved-device reconnect keeps running in the background. A toggle in the UI ("Discover new keyboards") controls whether unsaved scan results are surfaced to the UI:

- Toggle OFF (default): scan results are filtered to saved devices only. Picker shows nothing.
- Toggle ON: each scan cycle posts unsaved results to a small in-RAM ring; UI polls them. Clicking a result triggers `esp_hidh_dev_open` and adds the device to saved storage on `OPEN_EVENT` success.

This prevents accidental pairings with random nearby keyboards and matches the rest of the system's "explicit, button-driven" philosophy.

## 4. Architecture

### 4.1 New modules

| File | Responsibility | Depends on |
|---|---|---|
| `main/runtime_mode.{c,h}` | Owns `runtime_mode_t` (RELAY/ADMIN), thread-safe getter/setter, transition entrypoints. | semphr |
| `main/button_gesture.{c,h}` | State machine over IO9 polling; emits `GESTURE_HOLD_1S` / `GESTURE_HOLD_5S` / `GESTURE_TRIPLE_TAP`. Provides LED feedback hooks. | esp_timer, led_duty |
| `main/wifi_ap.{c,h}` | `wifi_ap_start(ssid, pass)` / `wifi_ap_stop()`. Builds SSID and WPA2 pw from MAC. | esp_wifi, esp_netif |
| `main/web_server.{c,h}` | esp_http_server lifecycle, route registration, session middleware. | esp_http_server, web_auth |
| `main/web_auth.{c,h}` | PBKDF2-SHA256 password hashing; HMAC-signed session tokens; in-RAM session table (4 slots LRU). | mbedtls |
| `main/device_store.{c,h}` | Multi-device NVS layer (5-slot ring of `device_entry_t`). Wraps storage.c. | nvs_flash |
| `main/web_assets/index.html` | Single-page dashboard markup. | — |
| `main/web_assets/login.html` | Login + set-password form. | — |
| `main/web_assets/app.css` | Layout + minimal styling (no framework). | — |
| `main/web_assets/app.js` | Fetch wrappers, status poller, scan-toggle handling. | — |

### 4.2 Modified modules

- `main/storage.{c,h}` — schema bump v2 → v3 (see §5); add `admin_pw_hash` key; new `storage_factory_reset()` entrypoint shared by 5s-hold and `/api/admin/factory_reset`.
- `main/esp_hid_host_main.c::app_main` — replaces the inline `gpio_get_level(BOOT_MODE_PIN)` poll with `button_gesture_poll()`; starts `wifi_ap` + `web_server` on `GESTURE_TRIPLE_TAP`.
- `main/esp_hid_host_main.c::hid_connect` — consults `runtime_mode_get()` and `device_store` (instead of single `read_ble_device`). Skips auto-connect for unsaved devices unless discovery flag is set.
- `main/led.{c,h}` — adds `LED_MODE_ADMIN` (double-pulse-every-2s); `led_duty_for` extended to handle it.
- `main/CMakeLists.txt` — adds new sources + `target_add_binary_data` for `web_assets/*`; adds `esp_wifi`, `esp_http_server`, `mbedtls` to REQUIRES.

### 4.3 Concurrency

Wi-Fi + BLE share one radio on ESP32-C3. To minimize collisions and crashes:

- All BLE-state mutations stay in the existing `hid_connect` task. Web handlers post commands to a FreeRTOS queue (`web_cmd_queue`, depth 4) that `hid_connect` drains between scan cycles.
- Web handler waits on a per-command response semaphore (5s timeout). On timeout: returns 504 to the client; `hid_connect` ignores any late reply (sequence number).
- `wifi_ap_start` / `wifi_ap_stop` may only be called from the runtime-mode transition function (single caller, no concurrent calls).

### 4.4 Idle shutdown

The `web_server` records the timestamp of every authenticated request. A 5-second-tick task checks `(now - last_activity_us) > 10 min` → calls `runtime_mode_set(RELAY)` which tears down web server + AP. The user can also explicitly POST `/api/admin/shutdown_ap`.

## 5. Data model

### 5.1 NVS schema v3

Namespace bump: `hidproxy_v2` → `hidproxy_v3`. Migration runs once on first boot of the new firmware.

| Key | Type | Purpose |
|---|---|---|
| `schema_version` | i32 | = 3 |
| `devices` | blob | Fixed 5 × `sizeof(device_entry_t)` bytes; zero-padded |
| `device_count` | u8 | Valid entries (0..5) |
| `admin_pw_hash` | blob | 16-byte salt ‖ 32-byte PBKDF2-SHA256 hash (48 B total). Absent until first AP session sets it. |

```c
typedef struct {
  uint8_t bda[6];
  uint8_t addr_type;   // ESP_BLE_ADDR_TYPE_*
  uint8_t _reserved;
  char    name[8];     // first 7 chars of advertised name + NUL
} device_entry_t;       // 16 B
```

5 slots × 16 B = 80 B blob. Plenty of headroom in NVS.

### 5.2 Migration v2 → v3

In `storage_complete_migration` (called once after `esp_hid_gap_init`):

1. If `schema_version == 3` in `hidproxy_v3` → no-op.
2. Else if `hidproxy_v2` exists:
   - Read single-MAC entry from v2 (`ble_status==1` and `ble_results` blob).
   - Write `devices[0]` with that MAC (name empty, `addr_type=PUBLIC`), `device_count = 1`.
   - Erase the v2 namespace (`nvs_erase_all`).
3. Else (no v2 either — fresh install): `device_count = 0`.
4. Write `schema_version = 3` to v3.

Bonds are **kept** across v2→v3 (Task 7 of robustness-pass already wiped pre-v2 bonds; v2 already used IDF SMP keys).

### 5.3 Eviction when saved-device list is full

A 6th successful pair calls `device_store_upsert(entry)`:
- If MAC already present → update name in place.
- Else if `device_count < 5` → append.
- Else shift `devices[0..3]` ← `devices[1..4]`, write new entry at `devices[4]`, `device_count` stays 5.

No UI prompt. Logged on serial as a warning.

### 5.4 Factory reset

`storage_factory_reset()`:
1. `esp_ble_remove_bond_device` for every bond.
2. `nvs_erase_all` on `hidproxy_v3` and (defensively) `hidproxy_v2`.
3. `esp_restart()`. The next boot looks identical to a freshly-flashed device.

## 6. Web API

### 6.1 Routes

All `/api/*` paths return JSON. Static paths return their MIME type. Cookie name: `hp_session`, HTTP-only, SameSite=Strict.

| Method | Path | Auth | Body | Description |
|---|---|---|---|---|
| GET | `/` | none | — | index.html (200) |
| GET | `/login` | none | — | login.html (200) |
| GET | `/app.css`, `/app.js` | none | — | static assets |
| GET | `/api/auth/state` | none | — | `{password_set: bool, logged_in: bool}` |
| POST | `/api/auth/init` | none | `{password}` | 200 + cookie if pw not yet set; 409 if already set; 400 if password <8 chars |
| POST | `/api/auth/login` | none | `{password}` | 200 + cookie on match; 401 otherwise. Rate-limit: 5/min/IP. |
| POST | `/api/auth/logout` | cookie | — | 204 |
| GET | `/api/status` | cookie | — | `{mode, connected_device?, ble_status, saved_count, uptime_s, ap_idle_remaining_s}` |
| GET | `/api/devices` | cookie | — | `[{slot, name, mac, is_connected}]` |
| DELETE | `/api/devices/:slot` | cookie | — | Removes from saved list + bond + disconnects if active. 204. |
| GET | `/api/discovery` | cookie | — | `{enabled: bool, results: [{mac, name, rssi, addr_type}]}` |
| POST | `/api/discovery` | cookie | `{enabled: bool}` | Toggles discovery flag. 200. |
| POST | `/api/devices/connect` | cookie | `{mac, addr_type}` | Triggers `esp_hidh_dev_open`. 202; final result via next `/api/status` poll. |
| POST | `/api/admin/shutdown_ap` | cookie | — | Transitions ADMIN → RELAY immediately. 204. Connection drops on reply. |
| POST | `/api/admin/factory_reset` | cookie | `{confirm: "WIPE"}` | Triggers `storage_factory_reset()` after 200 reply. |

### 6.2 Button gesture state machine

States: `IDLE → PRESSED → COUNTING_TAPS → HOLDING_1S → HOLDING_5S`. Polled every 50 ms from `app_main`. Returns one of `GESTURE_NONE` / `GESTURE_HOLD_1S` / `GESTURE_HOLD_5S` / `GESTURE_TRIPLE_TAP`.

Transitions (level 0 = pressed, level 1 = released; thresholds in ms):

| From | Event | To | Emit |
|---|---|---|---|
| IDLE | pressed | PRESSED (start = now) | — |
| PRESSED | released and now-start < 300 | COUNTING_TAPS (count=1) | — |
| PRESSED | held and now-start ≥ 1000 | HOLDING_1S | — |
| COUNTING_TAPS | pressed and now-last < 500 | COUNTING_TAPS (count++) | if count==3 → emit + reset to IDLE |
| COUNTING_TAPS | now-last ≥ 500 | IDLE | — (taps timed out) |
| HOLDING_1S | held and now-start ≥ 5000 | HOLDING_5S | — |
| HOLDING_1S | released | IDLE | emit `GESTURE_HOLD_1S` |
| HOLDING_5S | released | IDLE | emit `GESTURE_HOLD_5S` |

Pure-logic core (`button_gesture_step(state, level, now_us) → new_state, emit`) is host-testable.

### 6.3 Web↔BLE command bridge

```
web handler ──┐
              ├──> web_cmd_queue ──> hid_connect (drains between scans)
              │                            │
              │      response semaphore <──┘
              └────────────────────────────┘
```

Each `web_cmd_t` has a request id and a pointer to a small response struct. `hid_connect` consumes one command per loop iteration (just before deciding whether to scan), writes result, signals semaphore. Web handler returns 504 on 5 s timeout (rare; bounded scan time is ~5 s + 2 s buffer).

Commands: `START_DISCOVERY`, `STOP_DISCOVERY`, `CONNECT_DEVICE`, `REMOVE_DEVICE`, `SHUTDOWN_AP`. Status reads (`/api/status`, `/api/devices`) bypass the queue — they only read atomically-replicated snapshots. Factory reset also bypasses the queue: the handler writes a 200 response, calls `storage_factory_reset()`, then `esp_restart()` directly — there is no live state worth preserving.

## 7. UI sketch

Single page, no SPA framework. Vanilla JS + `fetch`. Layout (compact dashboard):

```
┌────────────────────────────────────────┐
│  HID Proxy                    [Logout] │
│                                        │
│  Status                                │
│  Connected: ZMK-Corne (a1:b2:..:e5)    │
│  Mode:      ADMIN (idle 9m 12s)        │
│                                        │
│  Saved keyboards (3/5)                 │
│  [✓] ZMK-Corne   a1:b2:..:e5   [X]     │
│  [ ] ZMK-Lily58  fc:11:..:80   [X]     │
│  [ ] Anne Pro 2  88:77:..:44   [X]     │
│                                        │
│  Add keyboard                          │
│  Discover new keyboards:  [ OFF | ON ] │
│  (when ON, results stream below)       │
│  ─ ZMK-Sweep   c3:d4:..:99   [Connect] │
│                                        │
│  Admin                                 │
│  [ Shutdown AP now ]  [ Factory reset ]│
└────────────────────────────────────────┘
```

Polling: 1 s interval on `/api/status`; when discovery is ON, 1 s interval on `/api/discovery`. Manual refresh button as fallback.

Login screen: single password field, "Set password" mode adds a confirm field. Inline error display only — no toasts.

## 8. Error handling

| Failure mode | Behavior |
|---|---|
| `esp_wifi_start` fails | LED rapid-double-blink for 5 s; revert to RELAY; log error. |
| `httpd_start` fails (e.g., out of memory) | Same as above. |
| Web client posts malformed JSON | 400 with `{"error":"..."}`. No crash. |
| Login flood | 5/min/IP token bucket (RAM only). 6th → 429 with `Retry-After: 60`. |
| Session table full (4 slots) | LRU evicts oldest token; new login still succeeds. |
| `web_cmd_queue` full | Web handler returns 503 immediately. (Unlikely with depth 4 + 5 s commands.) |
| Command timeout (5 s) | 504. Late reply discarded by sequence-id check. |
| BLE OPEN failure during web-triggered connect | Existing backoff path (Task 9) applies. `/api/status` reflects via `ble_status` field. |
| Factory reset triggered during connection | Bonds removed → connection drops cleanly → reboot. |
| NVS write fails mid-upsert | Original blob unchanged (NVS is atomic per key). Logged; UI receives 500. |

## 9. Testing

### 9.1 Host-side (extends `test/host/`)

- `test_button_gesture.c`: feed synthetic `(level, now_us)` sequences. ~12 cases — lone tap, double-tap with timeout, triple-tap, hold-1s release, hold-5s release, mid-hold release at 4s, hold-then-tap, etc.
- `test_device_store.c`: in-memory NVS stub (function-pointer indirection in test build). Cases: empty → 5 entries → 6th triggers shift; upsert-existing updates in place; migration from v2 stub; eviction order.
- `test_web_auth.c`: PBKDF2 known-answer test vector; session token sign/verify round-trip; tampered token rejected; expired token rejected.

### 9.2 On-device (manual checklist)

Each item must pass before merging:

1. Fresh flash → boot → triple-tap → AP visible → join via phone → `/api/auth/state` shows `password_set: false` → set 8-char password → cookie issued.
2. With one saved keyboard → triple-tap → AP up → dashboard shows it; existing connection (if any) keeps relaying keystrokes during admin session.
3. Toggle discovery ON → put a new keyboard in pairing → it appears in picker → click Connect → pairing succeeds → both devices show in saved list → relay works.
4. Add 4 more keyboards (total 5) → 6th eviction shifts oldest out without prompt.
5. Hold 1 s → only active device removed; other saved entries remain.
6. Hold 5 s → device reboots; next boot acts like fresh flash (AP not up, no saved devices, `password_set: false`).
7. AP idle 10 min with no clicks → automatically returns to RELAY; saved keyboard auto-reconnects.
8. Type continuously for 5 min in RELAY (no admin) → no Wi-Fi-related crashes or regressions.
9. Wrong password 6 times in quick succession → 6th attempt blocked with 429.
10. Login session → close tab → reopen within 30 min → still logged in; after 30 min → forced re-login.

### 9.3 Pre-merge invariants

- All host tests pass.
- `idf.py size` shows app partition still ≤ 95 % full. (See risk in §10.)
- `idf.py monitor` shows no panics during the on-device checklist.
- Watchdog soak (5 min typing) shows no resets.

## 10. Risks and open questions

| # | Risk | Mitigation |
|---|---|---|
| 1 | Flash size — adding Wi-Fi + http_server + mbedtls + UI may overflow the 1 MB app partition (current = 905 KB, 14 % free). | Measure after first integration. If >95 %: switch to a custom partition layout that drops OTA (or shrinks free space). Fallback crypto: SHA-256+salt instead of PBKDF2. |
| 2 | Coexistence — running Wi-Fi AP and BLE concurrently can throttle both on a single-radio C3. | ADMIN mode keeps an active BLE connection but does not scan unless explicitly toggled, limiting concurrent radio use to the user-initiated discovery window. |
| 3 | Captive portal not implemented → users must know `http://192.168.4.1`. | Documented in README; revisit if friction is reported. |
| 4 | HTTPS not implemented → admin password traverses Wi-Fi in plaintext within WPA2 frames. | WPA2 + LAN scope deemed acceptable for this device class. Revisit if hosting on shared networks. |
| 5 | No rate-limiting on `/api/auth/init`. | Acceptable because the route refuses once `admin_pw_hash` is set. |
| 6 | Triple-tap could collide with debouncing on cheap buttons. | The detector requires release between taps (level transitions, not just edges); the 300 ms tap limit + 500 ms inter-tap window are generous. Adjustable in `button_gesture.h`. |
| 7 | mbedtls PBKDF2 cost on the C3 (160 MHz) — rounds=10 000 takes ~1 s. | Single time during login, acceptable. Document expected latency. |
| 8 | NVS wear from frequent saved-device updates (e.g., name updates) — but normal operation only writes on pair/unpair, which is rare. | No mitigation needed at this scale. |
| 9 | esp_http_server worker stack defaults (4 KB) may be tight once JSON parsing + PBKDF2 buffers stack. | Measure with `uxTaskGetStackHighWaterMark` during first integration; bump `httpd_config_t.stack_size` to 8 KB if margin is <1 KB. |

## 11. Out-of-scope follow-ups

These are explicitly deferred to keep the spec focused. Not promises; just markers so they don't get re-litigated mid-implementation:

- Captive-portal DNS hijack (§10 #3).
- HTTPS / self-signed cert workflow (§10 #4).
- Web-based firmware OTA.
- Per-user accounts or 2FA.
- Mobile-optimized UI (current sketch is desktop-first; should still be usable on phone since it's a single column).
- Streaming scan results via Server-Sent Events instead of polling.
- Status webhooks for home-automation integration.
- Switching active device from the UI (current model is "all saved are candidates, first found wins" — explicit active-selection is a future change).

---

**Companion file (to be written next):** implementation plan at `docs/superpowers/plans/2026-05-24-web-admin-ui.md`.
