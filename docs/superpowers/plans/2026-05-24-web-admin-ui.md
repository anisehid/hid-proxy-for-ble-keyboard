# Web Admin UI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a triple-tap-triggered Wi-Fi soft-AP web admin UI to hid_proxy, gated by WPA2 + a per-device admin password, supporting up to 5 saved keyboards and explicit discovery-toggle pairing.

**Architecture:** Two runtime modes (`RELAY` / `ADMIN`) gated by an IO9 gesture detector. ADMIN brings up esp_wifi soft-AP + esp_http_server; all BLE-state mutations are funneled through a FreeRTOS command queue consumed by the existing `hid_connect` task. NVS schema bumped to v3 with multi-device storage. Pure C helpers (gesture state machine, device store, web auth crypto) are extracted into separate files and unit-tested host-side under gcc + Unity. Crypto vendored as public-domain SHA-256 to keep host and device builds bit-identical.

**Tech Stack:** ESP-IDF 5.3.1 (esp_wifi, esp_http_server, mbedtls), FreeRTOS, NVS, vanilla HTML/CSS/JS. Host tests: gcc + Unity 2.5.2 (already vendored in `test/host/vendor/`).

**Companion spec:** `docs/superpowers/specs/2026-05-24-web-admin-ui-design.md` — design rationale, risks, and the verification plan referenced from Task 15.

---

## Task ordering rationale

1. **Pure helpers first under TDD** (Tasks 1, 2, 3, 6) — fast host iteration, no firmware build cycle.
2. **Storage + LED + runtime mode** (Tasks 4, 5) — backbone that everything else plugs into.
3. **Button + mode wiring** (Task 7) — replaces the current inline button poll, no Wi-Fi yet.
4. **Wi-Fi + HTTP server skeleton** (Tasks 8, 9) — firmware infrastructure, smoke-tested via curl.
5. **Route implementations** (Tasks 10, 11, 12) — incremental routes; queue bridge added in Task 11.
6. **Web assets + integration** (Task 13).
7. **Idle shutdown** (Task 14) — closes the lifecycle loop.
8. **Polish + verification** (Tasks 15, 16).

Commit at the end of every task. If a task has TDD steps, the test/code steps may also be committed separately when convenient — but always at task end.

---

## File map

**New files (main/):**
- `button_gesture.h`, `button_gesture.c` — pure gesture state machine + `button_gesture_poll()` wrapper.
- `runtime_mode.h`, `runtime_mode.c` — `RUNTIME_MODE_RELAY` / `RUNTIME_MODE_ADMIN`, thread-safe getter/setter, transition entrypoints.
- `device_store.h`, `device_store.c` — multi-device NVS layer (5-slot ring). Wraps storage helpers.
- `wifi_ap.h`, `wifi_ap.c` — soft-AP lifecycle, derives SSID + WPA2 from MAC.
- `web_auth.h`, `web_auth.c` — PBKDF2-SHA256 password hash; HMAC-signed session tokens; in-RAM session table.
- `web_server.h`, `web_server.c` — esp_http_server lifecycle + route registration + JSON helpers.
- `sha256.h`, `sha256.c` — public-domain SHA-256 (RFC 6234), used by web_auth so host tests run unchanged.
- `web_assets/index.html`, `web_assets/login.html`, `web_assets/app.css`, `web_assets/app.js` — static UI.

**New files (test/host/):**
- `test_button_gesture.c`
- `test_device_store.c`
- `test_web_auth.c`
- `nvs_stub.h`, `nvs_stub.c` — function-pointer NVS facade used by `device_store` host tests.

**Modified files (main/):**
- `CMakeLists.txt` — add new sources; `target_add_binary_data` for the four web assets; REQUIRES bumped with `esp_wifi`, `esp_http_server`, `esp_netif`, `nvs_flash`, `mbedtls`.
- `led_duty.h`, `led_duty.c` — add `LED_MODE_ADMIN` + handle it in `led_duty_for`.
- `led.h` — mirror `LED_MODE_ADMIN` constant.
- `storage.h`, `storage.c` — schema bump v2 → v3; add `admin_pw_hash` key; new `storage_factory_reset()` shared by 5s-hold and `/api/admin/factory_reset`. Delegates device storage to `device_store`.
- `esp_hid_host_main.c` — replace inline button poll with `button_gesture_poll()`; integrate `runtime_mode`; rewrite `hid_connect` to iterate `device_store_list()` and consume `web_cmd_queue`.
- `sdkconfig.defaults` — enable Wi-Fi, esp_http_server, lwIP knobs.

**Modified files (test/host/):**
- `runner.c` — add three new `run_*_tests()` calls.
- `CMakeLists.txt` — add new test sources + `${MAIN_DIR}/sha256.c`, `${MAIN_DIR}/button_gesture.c`, `${MAIN_DIR}/device_store.c`, `${MAIN_DIR}/web_auth.c`, plus the new `nvs_stub.c`.

---

## Task 1: Button gesture state machine (TDD)

A pure state machine that, fed `(level, now_us)` polls every 50 ms, emits one of `NONE`/`HOLD_1S`/`HOLD_5S`/`TRIPLE_TAP`. Pure C, host-testable.

**Files:**
- Create: `main/button_gesture.h`, `main/button_gesture.c`
- Create: `test/host/test_button_gesture.c`
- Modify: `test/host/CMakeLists.txt`, `test/host/runner.c`

- [ ] **Step 1: Write `main/button_gesture.h`**

```c
#ifndef BUTTON_GESTURE_H_
#define BUTTON_GESTURE_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    GESTURE_NONE        = 0,
    GESTURE_HOLD_1S     = 1,
    GESTURE_HOLD_5S     = 2,
    GESTURE_TRIPLE_TAP  = 3,
} gesture_t;

typedef enum {
    GS_IDLE          = 0,
    GS_PRESSED       = 1,
    GS_COUNTING_TAPS = 2,
    GS_HOLDING_1S    = 3,
    GS_HOLDING_5S    = 4,
} gs_state_t;

typedef struct {
    gs_state_t state;
    int64_t    press_start_us;   // when current press started
    int64_t    last_tap_us;      // last completed-tap timestamp
    uint8_t    tap_count;        // taps so far in current burst
} gesture_ctx_t;

#define GESTURE_TAP_MAX_US      (300LL * 1000)         // <=300 ms = tap
#define GESTURE_TAP_WINDOW_US   (500LL * 1000)         // <=500 ms between taps
#define GESTURE_HOLD_1S_US      (1000LL * 1000)        // >=1 s = forget
#define GESTURE_HOLD_5S_US      (5000LL * 1000)        // >=5 s = factory reset

// Reset to IDLE.
void gesture_init(gesture_ctx_t *ctx);

// Pure step: feed current button level (0 = pressed, 1 = released)
// and current time. Updates state and returns the gesture to emit
// (GESTURE_NONE if nothing fired this tick).
gesture_t gesture_step(gesture_ctx_t *ctx, int level, int64_t now_us);

#endif
```

- [ ] **Step 2: Write `test/host/test_button_gesture.c`**

```c
#include "unity.h"
#include "button_gesture.h"

#define MS *1000LL

static gesture_ctx_t ctx;

void setUp(void); // shared with other suites — keep prototype only here

static void reset(void) { gesture_init(&ctx); }

static void test_quiet_idle_emits_nothing(void) {
    reset();
    TEST_ASSERT_EQUAL_INT(GESTURE_NONE, gesture_step(&ctx, 1, 0));
    TEST_ASSERT_EQUAL_INT(GESTURE_NONE, gesture_step(&ctx, 1, 50 MS));
}

static void test_single_tap_emits_nothing(void) {
    reset();
    gesture_step(&ctx, 0, 0);            // press
    gesture_step(&ctx, 1, 100 MS);       // release within 300ms = tap
    // Tap window not yet expired; no gesture emitted
    TEST_ASSERT_EQUAL_INT(GESTURE_NONE, gesture_step(&ctx, 1, 200 MS));
    // After window expires, taps are forgotten silently
    TEST_ASSERT_EQUAL_INT(GESTURE_NONE, gesture_step(&ctx, 1, 800 MS));
}

static void test_double_tap_emits_nothing(void) {
    reset();
    gesture_step(&ctx, 0, 0);             // press
    gesture_step(&ctx, 1, 100 MS);        // release
    gesture_step(&ctx, 0, 300 MS);        // press
    gesture_step(&ctx, 1, 400 MS);        // release
    TEST_ASSERT_EQUAL_INT(GESTURE_NONE, gesture_step(&ctx, 1, 1000 MS));
}

static void test_triple_tap_emits(void) {
    reset();
    gesture_step(&ctx, 0, 0);
    gesture_step(&ctx, 1, 100 MS);
    gesture_step(&ctx, 0, 300 MS);
    gesture_step(&ctx, 1, 400 MS);
    gesture_step(&ctx, 0, 600 MS);
    gesture_t emitted = gesture_step(&ctx, 1, 700 MS);
    TEST_ASSERT_EQUAL_INT(GESTURE_TRIPLE_TAP, emitted);
}

static void test_hold_1s_release_emits_hold_1s(void) {
    reset();
    gesture_step(&ctx, 0, 0);                          // press
    gesture_step(&ctx, 0, 999 MS);                     // still held below threshold
    gesture_step(&ctx, 0, 1100 MS);                    // now in HOLDING_1S
    gesture_t emitted = gesture_step(&ctx, 1, 2000 MS); // release at 2s
    TEST_ASSERT_EQUAL_INT(GESTURE_HOLD_1S, emitted);
}

static void test_hold_5s_release_emits_hold_5s(void) {
    reset();
    gesture_step(&ctx, 0, 0);
    gesture_step(&ctx, 0, 1100 MS);                    // -> HOLDING_1S
    gesture_step(&ctx, 0, 5100 MS);                    // -> HOLDING_5S
    gesture_t emitted = gesture_step(&ctx, 1, 5500 MS);
    TEST_ASSERT_EQUAL_INT(GESTURE_HOLD_5S, emitted);
}

static void test_release_below_1s_after_long_press_starts_resets(void) {
    // Press, hold to 800ms (still not 1s), release. Nothing emits;
    // state returns to IDLE so the next press starts fresh.
    reset();
    gesture_step(&ctx, 0, 0);
    TEST_ASSERT_EQUAL_INT(GESTURE_NONE, gesture_step(&ctx, 1, 800 MS));
}

static void test_press_during_counting_resets_tap_count_if_too_late(void) {
    // Tap 1, then wait > 500ms before pressing again -> count starts over.
    reset();
    gesture_step(&ctx, 0, 0);
    gesture_step(&ctx, 1, 100 MS);
    // tap_count is 1; window is 500ms after release
    gesture_step(&ctx, 0, 800 MS);     // window expired -> count restarts
    gesture_step(&ctx, 1, 900 MS);
    // We've effectively done one tap (the second). Expect no triple-tap yet.
    gesture_step(&ctx, 0, 1100 MS);
    gesture_step(&ctx, 1, 1200 MS);    // two taps total since restart
    TEST_ASSERT_EQUAL_INT(GESTURE_NONE, gesture_step(&ctx, 1, 1300 MS));
}

void run_button_gesture_tests(void) {
    RUN_TEST(test_quiet_idle_emits_nothing);
    RUN_TEST(test_single_tap_emits_nothing);
    RUN_TEST(test_double_tap_emits_nothing);
    RUN_TEST(test_triple_tap_emits);
    RUN_TEST(test_hold_1s_release_emits_hold_1s);
    RUN_TEST(test_hold_5s_release_emits_hold_5s);
    RUN_TEST(test_release_below_1s_after_long_press_starts_resets);
    RUN_TEST(test_press_during_counting_resets_tap_count_if_too_late);
}
```

- [ ] **Step 3: Wire into `test/host/runner.c` and `test/host/CMakeLists.txt`**

`runner.c`:

```c
#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

void run_ch9329_pack_tests(void);
void run_led_duty_tests(void);
void run_button_gesture_tests(void);

int main(void) {
    UNITY_BEGIN();
    run_ch9329_pack_tests();
    run_led_duty_tests();
    run_button_gesture_tests();
    return UNITY_END();
}
```

`test/host/CMakeLists.txt` — add `test_button_gesture.c` and `${MAIN_DIR}/button_gesture.c` to `add_executable`:

```cmake
add_executable(run_tests
    runner.c
    test_ch9329_pack.c
    test_led_duty.c
    test_button_gesture.c
    ${MAIN_DIR}/ch9329_pack.c
    ${MAIN_DIR}/led_duty.c
    ${MAIN_DIR}/button_gesture.c
    ${VENDOR_DIR}/unity.c
)
```

- [ ] **Step 4: Configure & build — confirm tests fail (link error)**

```bash
cd test/host && cmake -B build && cmake --build build
```

Expected: `undefined reference to 'gesture_init'` / `'gesture_step'`. Confirms tests reach the symbols.

- [ ] **Step 5: Write `main/button_gesture.c`**

```c
#include "button_gesture.h"

void gesture_init(gesture_ctx_t *ctx) {
    ctx->state          = GS_IDLE;
    ctx->press_start_us = 0;
    ctx->last_tap_us    = 0;
    ctx->tap_count      = 0;
}

gesture_t gesture_step(gesture_ctx_t *ctx, int level, int64_t now_us) {
    bool pressed = (level == 0);

    // Tap-window timeout while idle/counting taps.
    if (ctx->state == GS_COUNTING_TAPS &&
        (now_us - ctx->last_tap_us) >= GESTURE_TAP_WINDOW_US) {
        ctx->state     = GS_IDLE;
        ctx->tap_count = 0;
    }

    switch (ctx->state) {
    case GS_IDLE:
        if (pressed) {
            ctx->state          = GS_PRESSED;
            ctx->press_start_us = now_us;
        }
        return GESTURE_NONE;

    case GS_PRESSED: {
        int64_t held = now_us - ctx->press_start_us;
        if (!pressed) {
            // Released. Tap if short enough, else nothing.
            if (held <= GESTURE_TAP_MAX_US) {
                ctx->tap_count   = 1;
                ctx->last_tap_us = now_us;
                ctx->state       = GS_COUNTING_TAPS;
            } else {
                ctx->state = GS_IDLE;
            }
            return GESTURE_NONE;
        }
        if (held >= GESTURE_HOLD_1S_US) {
            ctx->state = GS_HOLDING_1S;
        }
        return GESTURE_NONE;
    }

    case GS_COUNTING_TAPS:
        if (pressed) {
            ctx->state          = GS_PRESSED;
            ctx->press_start_us = now_us;
        }
        return GESTURE_NONE;

    case GS_HOLDING_1S: {
        int64_t held = now_us - ctx->press_start_us;
        if (!pressed) {
            ctx->state = GS_IDLE;
            return GESTURE_HOLD_1S;
        }
        if (held >= GESTURE_HOLD_5S_US) {
            ctx->state = GS_HOLDING_5S;
        }
        return GESTURE_NONE;
    }

    case GS_HOLDING_5S:
        if (!pressed) {
            ctx->state = GS_IDLE;
            return GESTURE_HOLD_5S;
        }
        return GESTURE_NONE;
    }
    return GESTURE_NONE;
}
```

Now augment `GS_PRESSED` to handle tap-count completion (extra release-while-counting). The current implementation handles tap counting via the `GS_PRESSED → GS_COUNTING_TAPS` flow, but doesn't emit `TRIPLE_TAP` on the third tap. Add the check right after `ctx->tap_count = 1;` becomes increment instead of assignment:

Replace the relevant `GS_PRESSED` `if (!pressed)` branch:

```c
        if (!pressed) {
            if (held <= GESTURE_TAP_MAX_US) {
                // increment tap count (preserves count from previous taps)
                if (ctx->state == GS_PRESSED && ctx->tap_count == 0) {
                    ctx->tap_count = 1;
                } else {
                    ctx->tap_count += 1;
                }
                ctx->last_tap_us = now_us;
                ctx->state       = (ctx->tap_count >= 3) ? GS_IDLE : GS_COUNTING_TAPS;
                if (ctx->tap_count >= 3) {
                    ctx->tap_count = 0;
                    return GESTURE_TRIPLE_TAP;
                }
            } else {
                ctx->state     = GS_IDLE;
                ctx->tap_count = 0;
            }
            return GESTURE_NONE;
        }
```

(Replace the simpler version above with this one — the original was the "first pass" body. Keep only this final version in the file.)

- [ ] **Step 6: Rebuild and run**

```bash
cmake --build build && ./build/run_tests
```

Expected:

```
... 19+ tests, all PASS
20 Tests 0 Failures 0 Ignored
OK
```

(12 existing + 8 new = 20.)

- [ ] **Step 7: Commit**

```bash
git add main/button_gesture.h main/button_gesture.c \
        test/host/test_button_gesture.c test/host/runner.c test/host/CMakeLists.txt
git commit -m "Add button gesture state machine (tap, hold-1s, hold-5s, triple-tap)"
```

---

## Task 2: LED_MODE_ADMIN pattern (TDD extension)

Add a fifth LED mode and tick→duty mapping so ADMIN mode is visually distinct.

**Pattern:** 100 ms on, 100 ms off, 100 ms on, 1700 ms off — repeating 2 s period.

In 50 ms ticks:

| Tick % 40 | Duty |
|---|---|
| 0, 1 | ON |
| 2, 3 | DIME |
| 4, 5 | ON |
| 6..39 | DIME |

**Files:**
- Modify: `main/led_duty.h`, `main/led_duty.c`, `main/led.h`
- Modify: `test/host/test_led_duty.c`

- [ ] **Step 1: Add `LED_MODE_ADMIN` to both header constants**

`main/led.h` — add after the existing `#define LED_MODE_OFF 0` block:

```c
/* web admin AP is up (double-pulse every 2 s) */
#define LED_MODE_ADMIN 4
```

`main/led_duty.h` — same value:

```c
#define LED_MODE_ADMIN 4
```

- [ ] **Step 2: Add failing tests to `test/host/test_led_duty.c`**

Insert before `run_led_duty_tests`:

```c
static void test_admin_first_pulse(void) {
    TEST_ASSERT_EQUAL_INT(LED_DUTY_ON,   led_duty_for(LED_MODE_ADMIN, 0));
    TEST_ASSERT_EQUAL_INT(LED_DUTY_ON,   led_duty_for(LED_MODE_ADMIN, 1));
    TEST_ASSERT_EQUAL_INT(LED_DUTY_DIME, led_duty_for(LED_MODE_ADMIN, 2));
    TEST_ASSERT_EQUAL_INT(LED_DUTY_DIME, led_duty_for(LED_MODE_ADMIN, 3));
}

static void test_admin_second_pulse(void) {
    TEST_ASSERT_EQUAL_INT(LED_DUTY_ON,   led_duty_for(LED_MODE_ADMIN, 4));
    TEST_ASSERT_EQUAL_INT(LED_DUTY_ON,   led_duty_for(LED_MODE_ADMIN, 5));
    TEST_ASSERT_EQUAL_INT(LED_DUTY_DIME, led_duty_for(LED_MODE_ADMIN, 6));
}

static void test_admin_long_dark(void) {
    for (int t = 6; t < 40; ++t) {
        TEST_ASSERT_EQUAL_INT(LED_DUTY_DIME, led_duty_for(LED_MODE_ADMIN, t));
    }
}

static void test_admin_period_repeats(void) {
    TEST_ASSERT_EQUAL_INT(LED_DUTY_ON,   led_duty_for(LED_MODE_ADMIN, 40));
    TEST_ASSERT_EQUAL_INT(LED_DUTY_ON,   led_duty_for(LED_MODE_ADMIN, 41));
    TEST_ASSERT_EQUAL_INT(LED_DUTY_ON,   led_duty_for(LED_MODE_ADMIN, 44));
    TEST_ASSERT_EQUAL_INT(LED_DUTY_DIME, led_duty_for(LED_MODE_ADMIN, 46));
}
```

Add to `run_led_duty_tests`:

```c
    RUN_TEST(test_admin_first_pulse);
    RUN_TEST(test_admin_second_pulse);
    RUN_TEST(test_admin_long_dark);
    RUN_TEST(test_admin_period_repeats);
```

- [ ] **Step 3: Run — confirm 4 failures**

```bash
cmake --build build && ./build/run_tests
```

Expected: 4 new tests fail.

- [ ] **Step 4: Implement in `main/led_duty.c`**

Add a case to the existing switch:

```c
    case LED_MODE_ADMIN: {
        int phase = tick % 40;
        // ON at 0..1 and 4..5, DIME everywhere else
        if ((phase >= 0 && phase < 2) || (phase >= 4 && phase < 6)) {
            return LED_DUTY_ON;
        }
        return LED_DUTY_DIME;
    }
```

- [ ] **Step 5: Run — expect all pass**

```bash
cmake --build build && ./build/run_tests
```

Expected: `24 Tests 0 Failures 0 Ignored OK`.

- [ ] **Step 6: Sanity grep constants agree**

```bash
grep -E 'LED_MODE_(OFF|FAST_BLINK|ALWAYS_ON|SLOW_BLINK|ADMIN)' main/led.h main/led_duty.h
```

Expected: both define `ADMIN = 4`.

- [ ] **Step 7: Commit**

```bash
git add main/led.h main/led_duty.h main/led_duty.c test/host/test_led_duty.c
git commit -m "Add LED_MODE_ADMIN (double-pulse every 2 s)"
```

---

## Task 3: device_store with NVS stub (TDD)

A 5-slot ring with upsert/list/remove/migrate semantics. NVS access goes through a `nvs_ops_t` struct of function pointers so host tests can use an in-memory stub.

**Files:**
- Create: `main/device_store.h`, `main/device_store.c`
- Create: `test/host/nvs_stub.h`, `test/host/nvs_stub.c`
- Create: `test/host/test_device_store.c`
- Modify: `test/host/CMakeLists.txt`, `test/host/runner.c`

- [ ] **Step 1: Write `main/device_store.h`**

```c
#ifndef DEVICE_STORE_H_
#define DEVICE_STORE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DEVICE_STORE_MAX 5
#define DEVICE_NAME_LEN  8   // 7 chars + NUL

typedef struct {
    uint8_t bda[6];
    uint8_t addr_type;
    uint8_t _reserved;
    char    name[DEVICE_NAME_LEN];
} device_entry_t;

// Opaque NVS facade — production wires this to nvs_flash, tests wire to a stub.
typedef struct nvs_ops {
    int  (*open)(const char *ns, void **out_handle);
    int  (*get_blob)(void *handle, const char *key, void *buf, size_t *size);
    int  (*set_blob)(void *handle, const char *key, const void *buf, size_t size);
    int  (*get_u8)(void *handle, const char *key, uint8_t *out);
    int  (*set_u8)(void *handle, const char *key, uint8_t in);
    int  (*get_i32)(void *handle, const char *key, int32_t *out);
    int  (*set_i32)(void *handle, const char *key, int32_t in);
    int  (*erase_all)(void *handle);
    int  (*commit)(void *handle);
    void (*close)(void *handle);
} nvs_ops_t;

// Install ops (call once at boot). Idempotent.
void device_store_set_ops(const nvs_ops_t *ops);

// Read full list from NVS. Returns count (0..DEVICE_STORE_MAX).
// out[] must hold DEVICE_STORE_MAX entries.
int device_store_list(device_entry_t *out);

// Upsert. If MAC already present, updates name in place.
// If full, shifts slot 0 out and appends at slot 4. Returns true on success.
bool device_store_upsert(const device_entry_t *entry);

// Remove by MAC. Returns true if removed, false if not found.
bool device_store_remove(const uint8_t bda[6]);

// True if this MAC is in the saved list.
bool device_store_contains(const uint8_t bda[6]);

// Erase everything (called from storage_factory_reset).
void device_store_clear(void);

// One-shot migration from v2 single-MAC storage.
// Reads v2 keys "ble_status" and "ble_results"; if v2 has a device,
// installs it as devices[0]. Idempotent (no-op if v2 absent).
void device_store_migrate_from_v2(void);

#endif
```

- [ ] **Step 2: Write `test/host/nvs_stub.h`**

```c
#ifndef NVS_STUB_H_
#define NVS_STUB_H_

#include "device_store.h"

// Install the in-memory stub as the device_store's NVS ops.
void nvs_stub_install(void);

// Reset the in-memory store between tests.
void nvs_stub_reset(void);

// Helpers to pre-seed v2 state for migration tests.
void nvs_stub_seed_v2_device(const uint8_t bda[6]);

#endif
```

- [ ] **Step 3: Write `test/host/nvs_stub.c`**

```c
#include "nvs_stub.h"
#include <stdlib.h>
#include <string.h>

#define MAX_ENTRIES 32
#define MAX_NS      4
#define MAX_KEY     32

typedef struct {
    char    ns[MAX_KEY];
    char    key[MAX_KEY];
    uint8_t bytes[256];
    size_t  size;
    int     kind; // 0=blob, 1=u8, 2=i32
} entry_t;

static entry_t g_entries[MAX_ENTRIES];
static int     g_count = 0;

static entry_t *find(const char *ns, const char *key) {
    for (int i = 0; i < g_count; ++i) {
        if (strcmp(g_entries[i].ns, ns) == 0 &&
            strcmp(g_entries[i].key, key) == 0) {
            return &g_entries[i];
        }
    }
    return NULL;
}

static entry_t *put(const char *ns, const char *key) {
    entry_t *e = find(ns, key);
    if (e) return e;
    if (g_count >= MAX_ENTRIES) return NULL;
    e = &g_entries[g_count++];
    snprintf(e->ns,  sizeof e->ns,  "%s", ns);
    snprintf(e->key, sizeof e->key, "%s", key);
    e->size = 0;
    return e;
}

static const char *current_ns = NULL;

static int stub_open(const char *ns, void **out_handle) {
    current_ns  = ns;
    *out_handle = (void *)ns;
    return 0;
}
static int stub_get_blob(void *h, const char *key, void *buf, size_t *size) {
    entry_t *e = find((const char *)h, key);
    if (!e || e->kind != 0) return -1;
    if (buf == NULL) { *size = e->size; return 0; }
    if (*size < e->size) return -1;
    memcpy(buf, e->bytes, e->size);
    *size = e->size;
    return 0;
}
static int stub_set_blob(void *h, const char *key, const void *buf, size_t size) {
    entry_t *e = put((const char *)h, key);
    if (!e) return -1;
    if (size > sizeof e->bytes) return -1;
    memcpy(e->bytes, buf, size);
    e->size = size;
    e->kind = 0;
    return 0;
}
static int stub_get_u8(void *h, const char *key, uint8_t *out) {
    entry_t *e = find((const char *)h, key);
    if (!e || e->kind != 1) return -1;
    *out = e->bytes[0];
    return 0;
}
static int stub_set_u8(void *h, const char *key, uint8_t in) {
    entry_t *e = put((const char *)h, key);
    if (!e) return -1;
    e->bytes[0] = in;
    e->size     = 1;
    e->kind     = 1;
    return 0;
}
static int stub_get_i32(void *h, const char *key, int32_t *out) {
    entry_t *e = find((const char *)h, key);
    if (!e || e->kind != 2) return -1;
    memcpy(out, e->bytes, sizeof *out);
    return 0;
}
static int stub_set_i32(void *h, const char *key, int32_t in) {
    entry_t *e = put((const char *)h, key);
    if (!e) return -1;
    memcpy(e->bytes, &in, sizeof in);
    e->size = sizeof in;
    e->kind = 2;
    return 0;
}
static int stub_erase_all(void *h) {
    const char *ns = (const char *)h;
    int w = 0;
    for (int i = 0; i < g_count; ++i) {
        if (strcmp(g_entries[i].ns, ns) != 0) g_entries[w++] = g_entries[i];
    }
    g_count = w;
    return 0;
}
static int  stub_commit(void *h) { (void)h; return 0; }
static void stub_close (void *h) { (void)h; }

static const nvs_ops_t g_stub_ops = {
    .open = stub_open, .get_blob = stub_get_blob, .set_blob = stub_set_blob,
    .get_u8 = stub_get_u8, .set_u8 = stub_set_u8,
    .get_i32 = stub_get_i32, .set_i32 = stub_set_i32,
    .erase_all = stub_erase_all, .commit = stub_commit, .close = stub_close,
};

void nvs_stub_install(void) { device_store_set_ops(&g_stub_ops); }

void nvs_stub_reset(void) { g_count = 0; }

void nvs_stub_seed_v2_device(const uint8_t bda[6]) {
    void *h = NULL;
    stub_open("hidproxy_v2", &h);
    int32_t one = 1;
    stub_set_i32(h, "ble_status", one);
    stub_set_blob(h, "ble_results", bda, 6);
}
```

- [ ] **Step 4: Write `test/host/test_device_store.c`**

```c
#include "unity.h"
#include "device_store.h"
#include "nvs_stub.h"
#include <string.h>

static const uint8_t MAC_A[6] = {0xAA,1,2,3,4,5};
static const uint8_t MAC_B[6] = {0xBB,1,2,3,4,5};
static const uint8_t MAC_C[6] = {0xCC,1,2,3,4,5};
static const uint8_t MAC_D[6] = {0xDD,1,2,3,4,5};
static const uint8_t MAC_E[6] = {0xEE,1,2,3,4,5};
static const uint8_t MAC_F[6] = {0xFF,1,2,3,4,5};

static device_entry_t entry_for(const uint8_t bda[6], const char *name) {
    device_entry_t e = {0};
    memcpy(e.bda, bda, 6);
    e.addr_type = 1;
    snprintf(e.name, sizeof e.name, "%s", name);
    return e;
}

static void setup(void) {
    nvs_stub_reset();
    nvs_stub_install();
    device_store_clear();
}

static void test_empty_list(void) {
    setup();
    device_entry_t out[DEVICE_STORE_MAX];
    TEST_ASSERT_EQUAL_INT(0, device_store_list(out));
}

static void test_upsert_and_list(void) {
    setup();
    device_entry_t e = entry_for(MAC_A, "A");
    TEST_ASSERT_TRUE(device_store_upsert(&e));
    device_entry_t out[DEVICE_STORE_MAX];
    TEST_ASSERT_EQUAL_INT(1, device_store_list(out));
    TEST_ASSERT_EQUAL_MEMORY(MAC_A, out[0].bda, 6);
    TEST_ASSERT_EQUAL_STRING("A", out[0].name);
}

static void test_upsert_updates_existing_name(void) {
    setup();
    device_entry_t e1 = entry_for(MAC_A, "old");
    device_entry_t e2 = entry_for(MAC_A, "new");
    device_store_upsert(&e1);
    device_store_upsert(&e2);
    device_entry_t out[DEVICE_STORE_MAX];
    TEST_ASSERT_EQUAL_INT(1, device_store_list(out));
    TEST_ASSERT_EQUAL_STRING("new", out[0].name);
}

static void test_upsert_fills_to_max(void) {
    setup();
    const uint8_t *macs[5] = {MAC_A, MAC_B, MAC_C, MAC_D, MAC_E};
    for (int i = 0; i < 5; ++i) {
        device_entry_t e = entry_for(macs[i], "");
        TEST_ASSERT_TRUE(device_store_upsert(&e));
    }
    device_entry_t out[DEVICE_STORE_MAX];
    TEST_ASSERT_EQUAL_INT(5, device_store_list(out));
}

static void test_upsert_shifts_oldest_when_full(void) {
    setup();
    const uint8_t *macs[5] = {MAC_A, MAC_B, MAC_C, MAC_D, MAC_E};
    for (int i = 0; i < 5; ++i) {
        device_entry_t e = entry_for(macs[i], "");
        device_store_upsert(&e);
    }
    device_entry_t f = entry_for(MAC_F, "F");
    TEST_ASSERT_TRUE(device_store_upsert(&f));
    device_entry_t out[DEVICE_STORE_MAX];
    TEST_ASSERT_EQUAL_INT(5, device_store_list(out));
    // Slot 0 should now be MAC_B (oldest was MAC_A, shifted out)
    TEST_ASSERT_EQUAL_MEMORY(MAC_B, out[0].bda, 6);
    // Slot 4 should be the new MAC_F
    TEST_ASSERT_EQUAL_MEMORY(MAC_F, out[4].bda, 6);
}

static void test_remove(void) {
    setup();
    device_entry_t a = entry_for(MAC_A, "A");
    device_entry_t b = entry_for(MAC_B, "B");
    device_store_upsert(&a);
    device_store_upsert(&b);
    TEST_ASSERT_TRUE(device_store_remove(MAC_A));
    device_entry_t out[DEVICE_STORE_MAX];
    TEST_ASSERT_EQUAL_INT(1, device_store_list(out));
    TEST_ASSERT_EQUAL_MEMORY(MAC_B, out[0].bda, 6);
    // Removing again returns false
    TEST_ASSERT_FALSE(device_store_remove(MAC_A));
}

static void test_contains(void) {
    setup();
    device_entry_t a = entry_for(MAC_A, "A");
    device_store_upsert(&a);
    TEST_ASSERT_TRUE(device_store_contains(MAC_A));
    TEST_ASSERT_FALSE(device_store_contains(MAC_B));
}

static void test_migrate_from_v2_picks_up_single_device(void) {
    setup();
    nvs_stub_seed_v2_device(MAC_C);
    device_store_migrate_from_v2();
    device_entry_t out[DEVICE_STORE_MAX];
    TEST_ASSERT_EQUAL_INT(1, device_store_list(out));
    TEST_ASSERT_EQUAL_MEMORY(MAC_C, out[0].bda, 6);
}

static void test_migrate_from_v2_noop_when_no_v2(void) {
    setup();
    device_store_migrate_from_v2();
    device_entry_t out[DEVICE_STORE_MAX];
    TEST_ASSERT_EQUAL_INT(0, device_store_list(out));
}

void run_device_store_tests(void) {
    RUN_TEST(test_empty_list);
    RUN_TEST(test_upsert_and_list);
    RUN_TEST(test_upsert_updates_existing_name);
    RUN_TEST(test_upsert_fills_to_max);
    RUN_TEST(test_upsert_shifts_oldest_when_full);
    RUN_TEST(test_remove);
    RUN_TEST(test_contains);
    RUN_TEST(test_migrate_from_v2_picks_up_single_device);
    RUN_TEST(test_migrate_from_v2_noop_when_no_v2);
}
```

- [ ] **Step 5: Wire into runner + CMakeLists**

`runner.c` — add prototype and call:

```c
void run_device_store_tests(void);
// ...
run_device_store_tests();
```

`test/host/CMakeLists.txt` — extend `add_executable`:

```cmake
add_executable(run_tests
    runner.c
    test_ch9329_pack.c
    test_led_duty.c
    test_button_gesture.c
    test_device_store.c
    nvs_stub.c
    ${MAIN_DIR}/ch9329_pack.c
    ${MAIN_DIR}/led_duty.c
    ${MAIN_DIR}/button_gesture.c
    ${MAIN_DIR}/device_store.c
    ${VENDOR_DIR}/unity.c
)
```

- [ ] **Step 6: Confirm tests fail**

```bash
cd test/host && cmake -B build && cmake --build build
```

Expected: `undefined reference to 'device_store_set_ops'` etc.

- [ ] **Step 7: Write `main/device_store.c`**

```c
#include "device_store.h"
#include <string.h>

#define NS_V3 "hidproxy_v3"
#define NS_V2 "hidproxy_v2"
#define K_DEVICES "devices"
#define K_COUNT   "device_count"

static const nvs_ops_t *g_ops = NULL;

void device_store_set_ops(const nvs_ops_t *ops) { g_ops = ops; }

static int macs_equal(const uint8_t a[6], const uint8_t b[6]) {
    return memcmp(a, b, 6) == 0;
}

int device_store_list(device_entry_t *out) {
    if (!g_ops || !out) return 0;
    void *h = NULL;
    if (g_ops->open(NS_V3, &h) != 0) return 0;
    uint8_t count = 0;
    g_ops->get_u8(h, K_COUNT, &count);
    if (count > DEVICE_STORE_MAX) count = DEVICE_STORE_MAX;
    size_t size = sizeof(device_entry_t) * DEVICE_STORE_MAX;
    int rc = g_ops->get_blob(h, K_DEVICES, out, &size);
    g_ops->close(h);
    if (rc != 0) {
        memset(out, 0, sizeof(device_entry_t) * DEVICE_STORE_MAX);
        return 0;
    }
    return count;
}

static int load(device_entry_t *buf, uint8_t *out_count) {
    void *h = NULL;
    if (g_ops->open(NS_V3, &h) != 0) return -1;
    uint8_t c = 0;
    g_ops->get_u8(h, K_COUNT, &c);
    if (c > DEVICE_STORE_MAX) c = DEVICE_STORE_MAX;
    size_t size = sizeof(device_entry_t) * DEVICE_STORE_MAX;
    if (g_ops->get_blob(h, K_DEVICES, buf, &size) != 0) {
        memset(buf, 0, sizeof(device_entry_t) * DEVICE_STORE_MAX);
    }
    *out_count = c;
    g_ops->close(h);
    return 0;
}

static int save(const device_entry_t *buf, uint8_t count) {
    void *h = NULL;
    if (g_ops->open(NS_V3, &h) != 0) return -1;
    g_ops->set_blob(h, K_DEVICES, buf, sizeof(device_entry_t) * DEVICE_STORE_MAX);
    g_ops->set_u8(h, K_COUNT, count);
    g_ops->commit(h);
    g_ops->close(h);
    return 0;
}

bool device_store_upsert(const device_entry_t *entry) {
    if (!g_ops || !entry) return false;
    device_entry_t buf[DEVICE_STORE_MAX];
    uint8_t count = 0;
    if (load(buf, &count) != 0) return false;

    for (uint8_t i = 0; i < count; ++i) {
        if (macs_equal(buf[i].bda, entry->bda)) {
            buf[i] = *entry;
            return save(buf, count) == 0;
        }
    }
    if (count < DEVICE_STORE_MAX) {
        buf[count++] = *entry;
        return save(buf, count) == 0;
    }
    // Full — shift oldest out.
    memmove(&buf[0], &buf[1], sizeof(device_entry_t) * (DEVICE_STORE_MAX - 1));
    buf[DEVICE_STORE_MAX - 1] = *entry;
    return save(buf, DEVICE_STORE_MAX) == 0;
}

bool device_store_remove(const uint8_t bda[6]) {
    if (!g_ops || !bda) return false;
    device_entry_t buf[DEVICE_STORE_MAX];
    uint8_t count = 0;
    if (load(buf, &count) != 0) return false;
    for (uint8_t i = 0; i < count; ++i) {
        if (macs_equal(buf[i].bda, bda)) {
            for (uint8_t j = i; j + 1 < count; ++j) buf[j] = buf[j + 1];
            count--;
            memset(&buf[count], 0, sizeof buf[0]);
            return save(buf, count) == 0;
        }
    }
    return false;
}

bool device_store_contains(const uint8_t bda[6]) {
    if (!g_ops || !bda) return false;
    device_entry_t buf[DEVICE_STORE_MAX];
    uint8_t count = 0;
    if (load(buf, &count) != 0) return false;
    for (uint8_t i = 0; i < count; ++i) {
        if (macs_equal(buf[i].bda, bda)) return true;
    }
    return false;
}

void device_store_clear(void) {
    if (!g_ops) return;
    void *h = NULL;
    if (g_ops->open(NS_V3, &h) != 0) return;
    g_ops->erase_all(h);
    g_ops->commit(h);
    g_ops->close(h);
}

void device_store_migrate_from_v2(void) {
    if (!g_ops) return;
    // If v3 already has entries, skip.
    device_entry_t existing[DEVICE_STORE_MAX];
    uint8_t existing_count = 0;
    if (load(existing, &existing_count) == 0 && existing_count > 0) return;

    void *h = NULL;
    if (g_ops->open(NS_V2, &h) != 0) return;
    int32_t status = 0;
    if (g_ops->get_i32(h, "ble_status", &status) != 0 || status != 1) {
        g_ops->close(h);
        return;
    }
    uint8_t bda[6] = {0};
    size_t  sz = sizeof bda;
    if (g_ops->get_blob(h, "ble_results", bda, &sz) != 0 || sz != 6) {
        g_ops->close(h);
        return;
    }
    g_ops->close(h);

    device_entry_t e = {0};
    memcpy(e.bda, bda, 6);
    e.addr_type = 0;
    device_store_upsert(&e);
}
```

- [ ] **Step 8: Run host tests**

```bash
cmake --build build && ./build/run_tests
```

Expected: `33 Tests 0 Failures 0 Ignored OK` (24 from previous + 9 new).

- [ ] **Step 9: Commit**

```bash
git add main/device_store.h main/device_store.c \
        test/host/nvs_stub.h test/host/nvs_stub.c \
        test/host/test_device_store.c test/host/runner.c test/host/CMakeLists.txt
git commit -m "Add device_store (5-slot ring, upsert/remove/migrate) with NVS stub tests"
```

---

## Task 4: Wire device_store into firmware storage + factory reset

Replace the single-device save/read path in `storage.c` with `device_store`, bump schema to v3, and add `storage_factory_reset()`.

**Files:**
- Modify: `main/storage.h`, `main/storage.c`, `main/esp_hid_host_main.c`, `main/CMakeLists.txt`

- [ ] **Step 1: Update `main/CMakeLists.txt`**

Add `device_store.c` and `button_gesture.c` to srcs (button_gesture is firmware-side compiled now too — it was only built host-side before):

```cmake
set(srcs "esp_hid_host_main.c"
         "esp_hid_gap.c"
         "ch932x.c"
         "ch9329_pack.c"
         "led.c"
         "led_duty.c"
         "storage.c"
         "device_store.c"
         "button_gesture.c")
```

- [ ] **Step 2: Add real NVS ops to `main/storage.c`**

At the top of `storage.c` add:

```c
#include "device_store.h"
// ... existing includes ...
```

After the existing static declarations, add a real NVS ops table:

```c
static int real_open(const char *ns, void **out_handle) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(ns, NVS_READWRITE, &h);
    if (err != ESP_OK) return -1;
    *out_handle = (void *)(uintptr_t)h;
    return 0;
}
static int real_get_blob(void *h, const char *key, void *buf, size_t *size) {
    return nvs_get_blob((nvs_handle_t)(uintptr_t)h, key, buf, size) == ESP_OK ? 0 : -1;
}
static int real_set_blob(void *h, const char *key, const void *buf, size_t size) {
    return nvs_set_blob((nvs_handle_t)(uintptr_t)h, key, buf, size) == ESP_OK ? 0 : -1;
}
static int real_get_u8(void *h, const char *key, uint8_t *out) {
    return nvs_get_u8((nvs_handle_t)(uintptr_t)h, key, out) == ESP_OK ? 0 : -1;
}
static int real_set_u8(void *h, const char *key, uint8_t in) {
    return nvs_set_u8((nvs_handle_t)(uintptr_t)h, key, in) == ESP_OK ? 0 : -1;
}
static int real_get_i32(void *h, const char *key, int32_t *out) {
    return nvs_get_i32((nvs_handle_t)(uintptr_t)h, key, out) == ESP_OK ? 0 : -1;
}
static int real_set_i32(void *h, const char *key, int32_t in) {
    return nvs_set_i32((nvs_handle_t)(uintptr_t)h, key, in) == ESP_OK ? 0 : -1;
}
static int real_erase_all(void *h) {
    return nvs_erase_all((nvs_handle_t)(uintptr_t)h) == ESP_OK ? 0 : -1;
}
static int real_commit(void *h) {
    return nvs_commit((nvs_handle_t)(uintptr_t)h) == ESP_OK ? 0 : -1;
}
static void real_close(void *h) { nvs_close((nvs_handle_t)(uintptr_t)h); }

static const nvs_ops_t g_real_ops = {
    .open = real_open, .get_blob = real_get_blob, .set_blob = real_set_blob,
    .get_u8 = real_get_u8, .set_u8 = real_set_u8,
    .get_i32 = real_get_i32, .set_i32 = real_set_i32,
    .erase_all = real_erase_all, .commit = real_commit, .close = real_close,
};
```

- [ ] **Step 3: Replace `init_nvs_flash` and migration**

Update header `main/storage.h` to bump version constant and expose new entrypoints:

```c
#define STORAGE_NAMESPACE   "hidproxy_v3"
#define STORAGE_V2_NAMESPACE "hidproxy_v2"
#define BOOT_MODE_PIN       GPIO_NUM_9
#define SCHEMA_VERSION_KEY  "schema_version"
#define SCHEMA_VERSION_VAL  3
#define ADMIN_PW_HASH_KEY   "admin_pw_hash"
#define ADMIN_PW_HASH_LEN   48   // 16 salt + 32 hash

esp_err_t init_nvs_flash(void);
esp_err_t storage_complete_migration(void);
esp_err_t storage_factory_reset(void);

// Admin password hash (read returns false if not set).
bool storage_admin_pw_set(const uint8_t hash[ADMIN_PW_HASH_LEN]);
bool storage_admin_pw_get(uint8_t hash[ADMIN_PW_HASH_LEN]);
bool storage_admin_pw_is_set(void);
```

Remove the old declarations: `save_ble_device`, `read_ble_device`, `clear_ble_devices`, `BLE_RESULTS_STORAGE`, `BLE_STATUS`.

In `storage.c`, replace `init_nvs_flash` body with:

```c
esp_err_t init_nvs_flash(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) return ret;
    vSemaphoreCreateBinary(sema_handle);
    device_store_set_ops(&g_real_ops);
    return ESP_OK;
}
```

Replace `storage_complete_migration` with:

```c
esp_err_t storage_complete_migration(void) {
    // Check schema_version in v3 namespace.
    nvs_handle_t h;
    int32_t schema = 0;
    if (nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        if (nvs_get_i32(h, SCHEMA_VERSION_KEY, &schema) == ESP_OK &&
            schema == SCHEMA_VERSION_VAL) {
            nvs_close(h);
            return ESP_OK;
        }
        nvs_close(h);
    }

    // Pull single device from v2 (if present) into v3 multi-device storage.
    device_store_migrate_from_v2();

    // Wipe the v2 namespace so we never re-migrate.
    if (nvs_open(STORAGE_V2_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }

    // Stamp schema_version.
    if (nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        return ESP_FAIL;
    }
    nvs_set_i32(h, SCHEMA_VERSION_KEY, SCHEMA_VERSION_VAL);
    nvs_commit(h);
    nvs_close(h);
    printf("storage: migrated to schema v%d\n", SCHEMA_VERSION_VAL);
    return ESP_OK;
}
```

Add factory reset:

```c
esp_err_t storage_factory_reset(void) {
    int dev_num = esp_ble_get_bond_device_num();
    if (dev_num > 0) {
        esp_ble_bond_dev_t *list = malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
        if (list) {
            esp_ble_get_bond_device_list(&dev_num, list);
            for (int i = 0; i < dev_num; ++i) esp_ble_remove_bond_device(list[i].bd_addr);
            free(list);
        }
    }
    device_store_clear();
    // Wipe BOTH namespaces (defensive against pre-migration installs).
    nvs_handle_t h;
    if (nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h); nvs_commit(h); nvs_close(h);
    }
    if (nvs_open(STORAGE_V2_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h); nvs_commit(h); nvs_close(h);
    }
    return ESP_OK;
}
```

Add password helpers:

```c
bool storage_admin_pw_set(const uint8_t hash[ADMIN_PW_HASH_LEN]) {
    nvs_handle_t h;
    if (nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return false;
    bool ok = (nvs_set_blob(h, ADMIN_PW_HASH_KEY, hash, ADMIN_PW_HASH_LEN) == ESP_OK) &&
              (nvs_commit(h) == ESP_OK);
    nvs_close(h);
    return ok;
}
bool storage_admin_pw_get(uint8_t hash[ADMIN_PW_HASH_LEN]) {
    nvs_handle_t h;
    if (nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;
    size_t len = ADMIN_PW_HASH_LEN;
    bool ok = (nvs_get_blob(h, ADMIN_PW_HASH_KEY, hash, &len) == ESP_OK) &&
              (len == ADMIN_PW_HASH_LEN);
    nvs_close(h);
    return ok;
}
bool storage_admin_pw_is_set(void) {
    uint8_t buf[ADMIN_PW_HASH_LEN];
    return storage_admin_pw_get(buf);
}
```

Delete the old `save_ble_device`, `read_ble_device`, `clear_ble_devices`, and the helpers `get_ble_status` / `set_ble_status` / `save_ble_results` / `read_ble_results`. (Callers will be updated in this same task.)

- [ ] **Step 4: Update `esp_hid_host_main.c` callers**

Replace remaining `save_ble_device`/`read_ble_device`/`clear_ble_devices` calls.

- `ESP_HIDH_OPEN_EVENT` success branch: replace `save_ble_device((uint8_t *)bda);` with:

```c
        device_entry_t entry = {0};
        memcpy(entry.bda, bda, 6);
        entry.addr_type = 0;  // not stored; we always re-determine from live scan on reconnect
        const char *name = esp_hidh_dev_name_get(param->open.dev);
        if (name) snprintf(entry.name, DEVICE_NAME_LEN, "%s", name);
        device_store_upsert(&entry);
```

(Add `#include "device_store.h"` near the top.)

- `hid_connect`: replace the saved-device lookup with iteration over `device_store_list`:

Replace:

```c
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
```

With:

```c
    device_entry_t saved[DEVICE_STORE_MAX];
    int saved_count = device_store_list(saved);

    if (saved_count > 0) {
        set_led_mode(BLE_SCAN_SAVED_LED_MODE);
    } else {
        set_led_mode(BLE_SCAN_NEW_LED_MODE);
    }
    // Scan without an address filter — we filter results against saved[] below.
    esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results, NULL);
```

In the inner result loop, before `connect_hid_dev(r)`, add:

```c
        if (saved_count > 0 && !device_store_contains(r->bda)) {
            // Not a saved device. In Task 11 we'll honor a "discovery"
            // toggle; for now (RELAY only) ignore unsaved candidates.
            r = r->next;
            continue;
        }
```

- `disconnect_device()`: replace `bool ret = clear_ble_devices();` with — if `connected_dev` exists, look up its BDA and call `device_store_remove(bda)`:

```c
    bool ret = true;
    if (esp_hidh_dev_exists(connected_dev)) {
        const uint8_t *bda = esp_hidh_dev_bda_get(connected_dev);
        ret = device_store_remove(bda);
    }
    // ... rest unchanged ...
```

- [ ] **Step 5: Build firmware**

```bash
. /home/hy/System/esp/esp-idf/export.sh
idf.py build
```

Expected: clean.

- [ ] **Step 6: Re-run host tests (regression)**

```bash
cd test/host && cmake --build build && ./build/run_tests
```

Expected: `33 Tests 0 Failures 0 Ignored OK`.

- [ ] **Step 7: Commit**

```bash
git add main/storage.h main/storage.c main/esp_hid_host_main.c main/CMakeLists.txt
git commit -m "Bump NVS schema to v3 with multi-device storage and factory reset"
```

---

## Task 5: runtime_mode module

Single source of truth for the proxy's high-level mode.

**Files:**
- Create: `main/runtime_mode.h`, `main/runtime_mode.c`
- Modify: `main/CMakeLists.txt`

- [ ] **Step 1: Write `main/runtime_mode.h`**

```c
#ifndef RUNTIME_MODE_H_
#define RUNTIME_MODE_H_

typedef enum {
    RUNTIME_MODE_RELAY = 0,
    RUNTIME_MODE_ADMIN = 1,
} runtime_mode_t;

// Initialize internal semaphore. Call once before any other function.
void runtime_mode_init(void);

runtime_mode_t runtime_mode_get(void);
void           runtime_mode_set(runtime_mode_t m);

#endif
```

- [ ] **Step 2: Write `main/runtime_mode.c`**

```c
#include "runtime_mode.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static runtime_mode_t   g_mode = RUNTIME_MODE_RELAY;
static SemaphoreHandle_t g_sem = NULL;

void runtime_mode_init(void) {
    if (g_sem == NULL) g_sem = xSemaphoreCreateMutex();
}

runtime_mode_t runtime_mode_get(void) {
    runtime_mode_t m = RUNTIME_MODE_RELAY;
    if (g_sem && xSemaphoreTake(g_sem, pdMS_TO_TICKS(50)) == pdTRUE) {
        m = g_mode;
        xSemaphoreGive(g_sem);
    }
    return m;
}

void runtime_mode_set(runtime_mode_t m) {
    if (g_sem && xSemaphoreTake(g_sem, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_mode = m;
        xSemaphoreGive(g_sem);
    }
}
```

- [ ] **Step 3: Add to `main/CMakeLists.txt`**

```cmake
set(srcs "esp_hid_host_main.c"
         "esp_hid_gap.c"
         "ch932x.c"
         "ch9329_pack.c"
         "led.c"
         "led_duty.c"
         "storage.c"
         "device_store.c"
         "button_gesture.c"
         "runtime_mode.c")
```

- [ ] **Step 4: Make `hid_connect` LED-policy aware of ADMIN mode**

In `main/esp_hid_host_main.c`, add `#include "runtime_mode.h"` near other includes.

Wrap the LED writes added in Task 4 with a runtime-mode guard. Replace the block written in Task 4 step 4:

```c
    if (saved_count > 0) {
        set_led_mode(BLE_SCAN_SAVED_LED_MODE);
    } else {
        set_led_mode(BLE_SCAN_NEW_LED_MODE);
    }
```

with:

```c
    // Skip LED writes when in ADMIN — the admin transition owns the LED pattern.
    if (runtime_mode_get() != RUNTIME_MODE_ADMIN) {
        if (saved_count > 0) {
            set_led_mode(BLE_SCAN_SAVED_LED_MODE);
        } else {
            set_led_mode(BLE_SCAN_NEW_LED_MODE);
        }
    }
```

(`runtime_mode_get()` returns `RUNTIME_MODE_RELAY` until `runtime_mode_init()` is called — the underlying semaphore is NULL — so this is safe even before Task 7 wires the init call.)

- [ ] **Step 5: Build**

```bash
. /home/hy/System/esp/esp-idf/export.sh
idf.py build
```

Expected: clean.

- [ ] **Step 6: Commit**

```bash
git add main/runtime_mode.h main/runtime_mode.c main/CMakeLists.txt main/esp_hid_host_main.c
git commit -m "Add runtime_mode module; gate hid_connect LED writes by mode"
```

---

## Task 6: web_auth + vendored SHA-256 (TDD)

PBKDF2-SHA256 password hashing, HMAC-SHA256 session tokens, in-RAM session table. Crypto provided by a vendored public-domain SHA-256 so host tests run unchanged.

**Files:**
- Create: `main/sha256.h`, `main/sha256.c` (vendored)
- Create: `main/web_auth.h`, `main/web_auth.c`
- Create: `test/host/test_web_auth.c`
- Modify: `test/host/CMakeLists.txt`, `test/host/runner.c`

- [ ] **Step 1: Vendor SHA-256 (public domain by Brad Conte)**

Fetch from a trusted public-domain source. Use Brad Conte's `crypto-algorithms` repo (public domain):

```bash
curl -fsSL -o main/sha256.h https://raw.githubusercontent.com/B-Con/crypto-algorithms/cbdf76d1f2c875acc32a3e688c0cef0a5e7d4712/sha256.h
curl -fsSL -o main/sha256.c https://raw.githubusercontent.com/B-Con/crypto-algorithms/cbdf76d1f2c875acc32a3e688c0cef0a5e7d4712/sha256.c
```

Verify file sizes:

```bash
wc -c main/sha256.h main/sha256.c
```

Expected: header ~1.5 KB, source ~7 KB.

If the file uses `BYTE`/`WORD` typedefs from a missing header, edit `main/sha256.h` to add the typedefs explicitly above the function prototypes:

```c
#include <stddef.h>
#include <stdint.h>
typedef uint8_t  BYTE;
typedef uint32_t WORD;
```

- [ ] **Step 2: Write `main/web_auth.h`**

```c
#ifndef WEB_AUTH_H_
#define WEB_AUTH_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define WEB_AUTH_SALT_LEN     16
#define WEB_AUTH_HASH_LEN     32
#define WEB_AUTH_STORED_LEN   (WEB_AUTH_SALT_LEN + WEB_AUTH_HASH_LEN)
#define WEB_AUTH_PBKDF2_ROUNDS 10000

#define WEB_AUTH_TOKEN_LEN    40   // 8-byte exp + 16 random + 16 HMAC

// Compute salt||hash for a password. salt is the first WEB_AUTH_SALT_LEN
// bytes of out; hash follows. Caller provides a 16-byte random salt
// (zeros allowed in tests). out must be at least WEB_AUTH_STORED_LEN bytes.
void web_auth_hash_password(const char *password,
                            const uint8_t salt[WEB_AUTH_SALT_LEN],
                            uint8_t out[WEB_AUTH_STORED_LEN]);

// Constant-time compare of a candidate password against stored salt||hash.
bool web_auth_verify_password(const char *candidate,
                              const uint8_t stored[WEB_AUTH_STORED_LEN]);

// Mint a session token: 8-byte expiry (ms-since-epoch BE) + 16 random + 16 HMAC.
// `key` must be a 32-byte process-wide secret. Returns true on success.
bool web_auth_token_mint(uint64_t expiry_ms,
                         const uint8_t random_nonce[16],
                         const uint8_t key[32],
                         uint8_t out[WEB_AUTH_TOKEN_LEN]);

// Verify a token: HMAC check + expiry check. Returns 0 if valid,
// negative for tampered (HMAC mismatch), -2 for expired.
int web_auth_token_verify(const uint8_t token[WEB_AUTH_TOKEN_LEN],
                          uint64_t now_ms,
                          const uint8_t key[32]);

#endif
```

- [ ] **Step 3: Write `test/host/test_web_auth.c`**

```c
#include "unity.h"
#include "web_auth.h"
#include <string.h>

static void test_hash_deterministic_with_fixed_salt(void) {
    uint8_t salt[WEB_AUTH_SALT_LEN] = {0};
    uint8_t out1[WEB_AUTH_STORED_LEN];
    uint8_t out2[WEB_AUTH_STORED_LEN];
    web_auth_hash_password("hello", salt, out1);
    web_auth_hash_password("hello", salt, out2);
    TEST_ASSERT_EQUAL_MEMORY(out1, out2, WEB_AUTH_STORED_LEN);
}

static void test_hash_different_passwords_differ(void) {
    uint8_t salt[WEB_AUTH_SALT_LEN] = {0};
    uint8_t a[WEB_AUTH_STORED_LEN], b[WEB_AUTH_STORED_LEN];
    web_auth_hash_password("hello", salt, a);
    web_auth_hash_password("world", salt, b);
    TEST_ASSERT_NOT_EQUAL(0, memcmp(a + WEB_AUTH_SALT_LEN,
                                    b + WEB_AUTH_SALT_LEN,
                                    WEB_AUTH_HASH_LEN));
}

static void test_verify_password_round_trip(void) {
    uint8_t salt[WEB_AUTH_SALT_LEN];
    for (int i = 0; i < WEB_AUTH_SALT_LEN; ++i) salt[i] = (uint8_t)i;
    uint8_t stored[WEB_AUTH_STORED_LEN];
    web_auth_hash_password("correcthorse", salt, stored);
    TEST_ASSERT_TRUE (web_auth_verify_password("correcthorse", stored));
    TEST_ASSERT_FALSE(web_auth_verify_password("wronghorse",   stored));
}

static void test_token_round_trip(void) {
    uint8_t key[32];
    for (int i = 0; i < 32; ++i) key[i] = (uint8_t)(i + 1);
    uint8_t nonce[16];
    for (int i = 0; i < 16; ++i) nonce[i] = (uint8_t)(i * 3);
    uint8_t tok[WEB_AUTH_TOKEN_LEN];
    TEST_ASSERT_TRUE(web_auth_token_mint(10000, nonce, key, tok));
    TEST_ASSERT_EQUAL_INT(0,  web_auth_token_verify(tok, 5000,  key));
    TEST_ASSERT_EQUAL_INT(-2, web_auth_token_verify(tok, 11000, key));
}

static void test_token_tampered_rejects(void) {
    uint8_t key[32] = {0};
    uint8_t nonce[16] = {0};
    uint8_t tok[WEB_AUTH_TOKEN_LEN];
    web_auth_token_mint(10000, nonce, key, tok);
    tok[10] ^= 0xFF; // flip a byte in the random portion
    TEST_ASSERT_LESS_THAN_INT(0, web_auth_token_verify(tok, 5000, key));
}

void run_web_auth_tests(void) {
    RUN_TEST(test_hash_deterministic_with_fixed_salt);
    RUN_TEST(test_hash_different_passwords_differ);
    RUN_TEST(test_verify_password_round_trip);
    RUN_TEST(test_token_round_trip);
    RUN_TEST(test_token_tampered_rejects);
}
```

- [ ] **Step 4: Wire into runner + CMakeLists**

`runner.c`:

```c
void run_web_auth_tests(void);
// ...
run_web_auth_tests();
```

`test/host/CMakeLists.txt` — add `test_web_auth.c`, `${MAIN_DIR}/web_auth.c`, and `${MAIN_DIR}/sha256.c`:

```cmake
add_executable(run_tests
    runner.c
    test_ch9329_pack.c
    test_led_duty.c
    test_button_gesture.c
    test_device_store.c
    test_web_auth.c
    nvs_stub.c
    ${MAIN_DIR}/ch9329_pack.c
    ${MAIN_DIR}/led_duty.c
    ${MAIN_DIR}/button_gesture.c
    ${MAIN_DIR}/device_store.c
    ${MAIN_DIR}/web_auth.c
    ${MAIN_DIR}/sha256.c
    ${VENDOR_DIR}/unity.c
)
```

- [ ] **Step 5: Confirm link failure**

```bash
cd test/host && cmake -B build && cmake --build build
```

Expected: undefined references to web_auth_*.

- [ ] **Step 6: Write `main/web_auth.c`**

```c
#include "web_auth.h"
#include "sha256.h"
#include <string.h>

#define SHA256_BLOCK_SIZE_BYTES 64
#define HMAC_KEY_MAX SHA256_BLOCK_SIZE_BYTES

static void hmac_sha256(const uint8_t *key, size_t key_len,
                        const uint8_t *msg, size_t msg_len,
                        uint8_t out[32]) {
    uint8_t k[SHA256_BLOCK_SIZE_BYTES] = {0};
    if (key_len > SHA256_BLOCK_SIZE_BYTES) {
        SHA256_CTX c; sha256_init(&c);
        sha256_update(&c, key, key_len);
        sha256_final(&c, k);
    } else {
        memcpy(k, key, key_len);
    }
    uint8_t inner[SHA256_BLOCK_SIZE_BYTES], outer[SHA256_BLOCK_SIZE_BYTES];
    for (int i = 0; i < SHA256_BLOCK_SIZE_BYTES; ++i) {
        inner[i] = k[i] ^ 0x36;
        outer[i] = k[i] ^ 0x5c;
    }
    SHA256_CTX c; sha256_init(&c);
    sha256_update(&c, inner, SHA256_BLOCK_SIZE_BYTES);
    sha256_update(&c, msg, msg_len);
    uint8_t inner_digest[32];
    sha256_final(&c, inner_digest);

    sha256_init(&c);
    sha256_update(&c, outer, SHA256_BLOCK_SIZE_BYTES);
    sha256_update(&c, inner_digest, 32);
    sha256_final(&c, out);
}

static void pbkdf2_sha256(const char *password, const uint8_t *salt, size_t salt_len,
                          unsigned rounds, uint8_t out[32]) {
    size_t pwlen = strlen(password);
    uint8_t block[68];
    memcpy(block, salt, salt_len);
    block[salt_len+0] = 0; block[salt_len+1] = 0;
    block[salt_len+2] = 0; block[salt_len+3] = 1;  // i = 1

    uint8_t u[32], t[32];
    hmac_sha256((const uint8_t *)password, pwlen, block, salt_len + 4, u);
    memcpy(t, u, 32);
    for (unsigned r = 1; r < rounds; ++r) {
        hmac_sha256((const uint8_t *)password, pwlen, u, 32, u);
        for (int i = 0; i < 32; ++i) t[i] ^= u[i];
    }
    memcpy(out, t, 32);
}

void web_auth_hash_password(const char *password,
                            const uint8_t salt[WEB_AUTH_SALT_LEN],
                            uint8_t out[WEB_AUTH_STORED_LEN]) {
    memcpy(out, salt, WEB_AUTH_SALT_LEN);
    pbkdf2_sha256(password, salt, WEB_AUTH_SALT_LEN,
                  WEB_AUTH_PBKDF2_ROUNDS,
                  out + WEB_AUTH_SALT_LEN);
}

bool web_auth_verify_password(const char *candidate,
                              const uint8_t stored[WEB_AUTH_STORED_LEN]) {
    uint8_t computed[WEB_AUTH_STORED_LEN];
    web_auth_hash_password(candidate, stored, computed);
    uint8_t diff = 0;
    for (int i = WEB_AUTH_SALT_LEN; i < WEB_AUTH_STORED_LEN; ++i) {
        diff |= (uint8_t)(computed[i] ^ stored[i]);
    }
    return diff == 0;
}

bool web_auth_token_mint(uint64_t expiry_ms,
                         const uint8_t random_nonce[16],
                         const uint8_t key[32],
                         uint8_t out[WEB_AUTH_TOKEN_LEN]) {
    // [0..7] = expiry big-endian; [8..23] = random; [24..39] = HMAC truncated to 16
    for (int i = 0; i < 8; ++i) out[i] = (uint8_t)(expiry_ms >> (56 - 8*i));
    memcpy(out + 8, random_nonce, 16);
    uint8_t mac[32];
    hmac_sha256(key, 32, out, 24, mac);
    memcpy(out + 24, mac, 16);
    return true;
}

int web_auth_token_verify(const uint8_t token[WEB_AUTH_TOKEN_LEN],
                          uint64_t now_ms,
                          const uint8_t key[32]) {
    uint8_t mac[32];
    hmac_sha256(key, 32, token, 24, mac);
    uint8_t diff = 0;
    for (int i = 0; i < 16; ++i) diff |= (uint8_t)(mac[i] ^ token[24+i]);
    if (diff != 0) return -1;
    uint64_t exp = 0;
    for (int i = 0; i < 8; ++i) exp = (exp << 8) | token[i];
    if (now_ms >= exp) return -2;
    return 0;
}
```

- [ ] **Step 7: Run host tests**

```bash
cmake --build build && ./build/run_tests
```

Expected: `38 Tests 0 Failures 0 Ignored OK`.

(PBKDF2 with 10 000 rounds runs ~5 times in tests; should still finish under a few seconds on host.)

- [ ] **Step 8: Commit**

```bash
git add main/sha256.h main/sha256.c main/web_auth.h main/web_auth.c \
        test/host/test_web_auth.c test/host/runner.c test/host/CMakeLists.txt
git commit -m "Add web_auth (PBKDF2-SHA256 + HMAC session tokens) with vendored SHA-256"
```

---

## Task 7: Button polling + runtime mode integration

Replace the inline `gpio_get_level` polling in `esp_hid_host_main.c::app_main` with the gesture detector; map each gesture to an action.

**Files:**
- Modify: `main/esp_hid_host_main.c`

- [ ] **Step 1: Add includes**

Near other includes:

```c
#include "button_gesture.h"
#include "runtime_mode.h"
#include "esp_timer.h"   // already added in robustness-pass; keep
```

- [ ] **Step 2: Replace the `app_main` button-poll loop**

Current loop:

```c
while (true) {
    if (gpio_get_level(BOOT_MODE_PIN) == 0) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (gpio_get_level(BOOT_MODE_PIN) == 0) {
            ESP_LOGI(TAG, "BUTTON IS DOWN");
            if (disconnect_device()) {
                ESP_LOGI(TAG, "CLEAR BLE STATUS SUCCESS");
            }
        }
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
}
```

Replace with:

```c
gesture_ctx_t gctx;
gesture_init(&gctx);
runtime_mode_init();

// Configure IO9 as input with internal pull-up (boot button).
gpio_config_t io = {
    .pin_bit_mask = 1ULL << BOOT_MODE_PIN,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
};
gpio_config(&io);

while (true) {
    int level = gpio_get_level(BOOT_MODE_PIN);
    gesture_t g = gesture_step(&gctx, level, esp_timer_get_time());
    switch (g) {
    case GESTURE_HOLD_1S:
        ESP_LOGI(TAG, "Gesture: HOLD_1S — forget active device");
        disconnect_device();
        break;
    case GESTURE_HOLD_5S:
        ESP_LOGW(TAG, "Gesture: HOLD_5S — FACTORY RESET");
        storage_factory_reset();
        esp_restart();
        break;
    case GESTURE_TRIPLE_TAP:
        ESP_LOGI(TAG, "Gesture: TRIPLE_TAP — entering ADMIN mode");
        runtime_mode_set(RUNTIME_MODE_ADMIN);
        // wifi_ap + web_server startup wired in Task 9
        break;
    default: break;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
}
```

(`disconnect_device()` is the existing helper. It already drops the BLE connection and sets `LED_MODE_FAST_BLINK`.)

- [ ] **Step 3: Build**

```bash
. /home/hy/System/esp/esp-idf/export.sh
idf.py build
```

Expected: clean.

- [ ] **Step 4: Commit**

```bash
git add main/esp_hid_host_main.c
git commit -m "Drive runtime_mode and storage_factory_reset from gesture detector"
```

---

## Task 8: Wi-Fi soft-AP module

Lifecycle for the soft-AP. SSID + WPA2 derived deterministically from the chip MAC and printed to serial.

**Files:**
- Create: `main/wifi_ap.h`, `main/wifi_ap.c`
- Modify: `main/CMakeLists.txt`, `main/sdkconfig.defaults`

- [ ] **Step 1: Write `main/wifi_ap.h`**

```c
#ifndef WIFI_AP_H_
#define WIFI_AP_H_

#include "esp_err.h"

// Brings up Wi-Fi as soft-AP. SSID = "hidproxy-XXXX" (last 4 MAC hex digits).
// WPA2 password = last 8 MAC hex digits (lowercase). Logs both to serial.
esp_err_t wifi_ap_start(void);

// Tears down soft-AP and stops Wi-Fi.
esp_err_t wifi_ap_stop(void);

#endif
```

- [ ] **Step 2: Write `main/wifi_ap.c`**

```c
#include "wifi_ap.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include <string.h>

static const char *TAG = "WIFI_AP";
static bool g_started = false;
static esp_netif_t *g_netif = NULL;

esp_err_t wifi_ap_start(void) {
    if (g_started) return ESP_OK;

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    char ssid[16];
    snprintf(ssid, sizeof ssid, "hidproxy-%02x%02x", mac[4], mac[5]);
    char pass[16];
    snprintf(pass, sizeof pass, "%02x%02x%02x%02x", mac[2], mac[3], mac[4], mac[5]);

    ESP_ERROR_CHECK(esp_netif_init());
    if (g_netif == NULL) g_netif = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wcfg = {0};
    wcfg.ap.channel = 6;
    wcfg.ap.max_connection = 3;
    wcfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wcfg.ap.pmf_cfg.required = false;
    strncpy((char *)wcfg.ap.ssid, ssid, sizeof wcfg.ap.ssid);
    wcfg.ap.ssid_len = strlen(ssid);
    strncpy((char *)wcfg.ap.password, pass, sizeof wcfg.ap.password);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP up: SSID='%s' PSK='%s' (open http://192.168.4.1)",
             ssid, pass);
    g_started = true;
    return ESP_OK;
}

esp_err_t wifi_ap_stop(void) {
    if (!g_started) return ESP_OK;
    esp_wifi_stop();
    esp_wifi_deinit();
    ESP_LOGI(TAG, "AP down");
    g_started = false;
    return ESP_OK;
}
```

- [ ] **Step 3: Add to `main/CMakeLists.txt`**

Add `wifi_ap.c` to srcs and `esp_wifi`, `esp_netif`, `esp_event` to REQUIRES:

```cmake
set(srcs "esp_hid_host_main.c"
         "esp_hid_gap.c"
         "ch932x.c"
         "ch9329_pack.c"
         "led.c"
         "led_duty.c"
         "storage.c"
         "device_store.c"
         "button_gesture.c"
         "runtime_mode.c"
         "wifi_ap.c")
set(include_dirs ".")
idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid driver esp_wifi esp_netif esp_event
                       PRIV_REQUIRES nvs_flash mbedtls)
```

- [ ] **Step 4: Wi-Fi sdkconfig knobs**

Append to `sdkconfig.defaults`:

```
CONFIG_ESP32_WIFI_NVS_ENABLED=y
CONFIG_ESP_WIFI_SOFTAP_SUPPORT=y
CONFIG_LWIP_LOCAL_HOSTNAME="hidproxy"
```

- [ ] **Step 5: Build**

```bash
. /home/hy/System/esp/esp-idf/export.sh
idf.py reconfigure && idf.py build
```

Expected: clean. Binary will grow noticeably (Wi-Fi stack adds ~150 KB). Note the size.

If size exceeds 100% of the app partition: stop here and report to the user — they'll need to choose a partition layout adjustment (covered in spec §10 risk #1).

- [ ] **Step 6: Commit**

```bash
git add main/wifi_ap.h main/wifi_ap.c main/CMakeLists.txt sdkconfig.defaults
git commit -m "Add Wi-Fi soft-AP module (SSID and PSK derived from MAC)"
```

---

## Task 9: HTTP server skeleton + auth routes

Add `web_server` with `/api/auth/state`, `/api/auth/init`, `/api/auth/login`, `/api/auth/logout`. Wire it up to start when entering ADMIN.

**Files:**
- Create: `main/web_server.h`, `main/web_server.c`
- Modify: `main/CMakeLists.txt`, `main/esp_hid_host_main.c`

- [ ] **Step 1: Write `main/web_server.h`**

```c
#ifndef WEB_SERVER_H_
#define WEB_SERVER_H_

#include "esp_err.h"

esp_err_t web_server_start(void);
esp_err_t web_server_stop(void);

// Touched by every authenticated request; consumed by the idle-shutdown task (Task 14).
int64_t web_server_last_activity_us(void);

#endif
```

- [ ] **Step 2: Write `main/web_server.c` (auth routes only — rest in Tasks 10-12)**

```c
#include "web_server.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "storage.h"
#include "web_auth.h"
#include <string.h>

static const char *TAG = "WEB";
static httpd_handle_t g_server = NULL;
static int64_t        g_last_activity_us = 0;

// Process-wide secret for signing session tokens (regenerated on boot).
static uint8_t g_session_key[32];

#define COOKIE_NAME "hp_session"

static void touch_activity(void) { g_last_activity_us = esp_timer_get_time(); }

// ---- helpers ----------------------------------------------------------------

static esp_err_t send_json(httpd_req_t *req, int code, const char *json) {
    httpd_resp_set_status(req, (code == 200) ? "200 OK"
                              : (code == 201) ? "201 Created"
                              : (code == 204) ? "204 No Content"
                              : (code == 400) ? "400 Bad Request"
                              : (code == 401) ? "401 Unauthorized"
                              : (code == 409) ? "409 Conflict"
                              : (code == 429) ? "429 Too Many Requests"
                              : "500 Internal Server Error");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json ? json : "");
}

static bool body_read(httpd_req_t *req, char *buf, size_t buflen, size_t *out_len) {
    int total = req->content_len;
    if (total < 0 || (size_t)total >= buflen) return false;
    int got = httpd_req_recv(req, buf, total);
    if (got != total) return false;
    buf[got] = 0;
    *out_len = (size_t)got;
    return true;
}

// Naive substring lookup for "key":"value" in a tiny JSON body.
// Sufficient for our flat single-field payloads.
static bool json_extract_string(const char *body, const char *key,
                                char *out, size_t outlen) {
    char needle[32];
    snprintf(needle, sizeof needle, "\"%s\"", key);
    const char *p = strstr(body, needle);
    if (!p) return false;
    p = strchr(p + strlen(needle), ':');
    if (!p) return false;
    while (*p && (*p == ':' || *p == ' ' || *p == '\t')) p++;
    if (*p != '"') return false;
    p++;
    size_t n = 0;
    while (*p && *p != '"' && n + 1 < outlen) out[n++] = *p++;
    out[n] = 0;
    return *p == '"';
}

static void emit_session_cookie(httpd_req_t *req, const uint8_t token[WEB_AUTH_TOKEN_LEN]) {
    char hex[WEB_AUTH_TOKEN_LEN*2 + 1];
    for (int i = 0; i < WEB_AUTH_TOKEN_LEN; ++i)
        snprintf(hex + i*2, 3, "%02x", token[i]);
    char header[200];
    snprintf(header, sizeof header,
             COOKIE_NAME "=%s; Path=/; HttpOnly; SameSite=Strict; Max-Age=1800",
             hex);
    httpd_resp_set_hdr(req, "Set-Cookie", header);
}

static bool read_session_cookie(httpd_req_t *req, uint8_t token[WEB_AUTH_TOKEN_LEN]) {
    char cookie[256];
    if (httpd_req_get_hdr_value_str(req, "Cookie", cookie, sizeof cookie) != ESP_OK)
        return false;
    char *p = strstr(cookie, COOKIE_NAME "=");
    if (!p) return false;
    p += strlen(COOKIE_NAME "=");
    char *end = strchr(p, ';');
    size_t len = end ? (size_t)(end - p) : strlen(p);
    if (len != WEB_AUTH_TOKEN_LEN * 2) return false;
    for (int i = 0; i < WEB_AUTH_TOKEN_LEN; ++i) {
        unsigned v;
        if (sscanf(p + i*2, "%2x", &v) != 1) return false;
        token[i] = (uint8_t)v;
    }
    return true;
}

bool web_request_authenticated(httpd_req_t *req) {
    uint8_t tok[WEB_AUTH_TOKEN_LEN];
    if (!read_session_cookie(req, tok)) return false;
    uint64_t now_ms = esp_timer_get_time() / 1000;
    return web_auth_token_verify(tok, now_ms, g_session_key) == 0;
}

// ---- handlers ---------------------------------------------------------------

static esp_err_t h_auth_state(httpd_req_t *req) {
    char body[64];
    snprintf(body, sizeof body, "{\"password_set\":%s,\"logged_in\":%s}",
             storage_admin_pw_is_set() ? "true" : "false",
             web_request_authenticated(req) ? "true" : "false");
    return send_json(req, 200, body);
}

static esp_err_t h_auth_init(httpd_req_t *req) {
    if (storage_admin_pw_is_set()) return send_json(req, 409, "{\"error\":\"already_set\"}");
    char body[128]; size_t blen = 0;
    if (!body_read(req, body, sizeof body, &blen))
        return send_json(req, 400, "{\"error\":\"bad_body\"}");
    char pw[64];
    if (!json_extract_string(body, "password", pw, sizeof pw))
        return send_json(req, 400, "{\"error\":\"no_password\"}");
    if (strlen(pw) < 8) return send_json(req, 400, "{\"error\":\"too_short\"}");

    uint8_t salt[WEB_AUTH_SALT_LEN];
    esp_fill_random(salt, sizeof salt);
    uint8_t stored[WEB_AUTH_STORED_LEN];
    web_auth_hash_password(pw, salt, stored);
    if (!storage_admin_pw_set(stored)) return send_json(req, 500, "{\"error\":\"nvs\"}");

    // Issue a session immediately.
    uint8_t nonce[16];
    esp_fill_random(nonce, sizeof nonce);
    uint8_t tok[WEB_AUTH_TOKEN_LEN];
    uint64_t exp = (esp_timer_get_time() / 1000) + 1800 * 1000;
    web_auth_token_mint(exp, nonce, g_session_key, tok);
    emit_session_cookie(req, tok);
    touch_activity();
    return send_json(req, 200, "{\"ok\":true}");
}

static esp_err_t h_auth_login(httpd_req_t *req) {
    // Global sliding-window rate limit: <=5 failed attempts in the last 60 s.
    // (Implementing it per-IP requires lwip socket plumbing that esp_http_server
    // doesn't expose stably; for a soft-AP with at most a handful of clients
    // the global bucket gives the same brute-force protection.)
    static int64_t fail_times_us[5] = {0};
    int64_t now = esp_timer_get_time();
    int recent = 0;
    for (int i = 0; i < 5; ++i) {
        if (now - fail_times_us[i] < 60LL * 1000 * 1000) recent++;
    }
    if (recent >= 5) {
        httpd_resp_set_hdr(req, "Retry-After", "60");
        return send_json(req, 429, "{\"error\":\"too_many\"}");
    }

    char body[128]; size_t blen = 0;
    if (!body_read(req, body, sizeof body, &blen))
        return send_json(req, 400, "{\"error\":\"bad_body\"}");
    char pw[64];
    if (!json_extract_string(body, "password", pw, sizeof pw))
        return send_json(req, 400, "{\"error\":\"no_password\"}");
    uint8_t stored[WEB_AUTH_STORED_LEN];
    if (!storage_admin_pw_get(stored))
        return send_json(req, 401, "{\"error\":\"not_initialized\"}");
    if (!web_auth_verify_password(pw, stored)) {
        // Record failure in the oldest slot.
        int oldest = 0;
        for (int i = 1; i < 5; ++i) {
            if (fail_times_us[i] < fail_times_us[oldest]) oldest = i;
        }
        fail_times_us[oldest] = now;
        return send_json(req, 401, "{\"error\":\"bad_password\"}");
    }
    uint8_t nonce[16];
    esp_fill_random(nonce, sizeof nonce);
    uint8_t tok[WEB_AUTH_TOKEN_LEN];
    uint64_t exp = (esp_timer_get_time() / 1000) + 1800 * 1000;
    web_auth_token_mint(exp, nonce, g_session_key, tok);
    emit_session_cookie(req, tok);
    touch_activity();
    return send_json(req, 200, "{\"ok\":true}");
}

static esp_err_t h_auth_logout(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Set-Cookie",
                       COOKIE_NAME "=; Path=/; HttpOnly; Max-Age=0");
    return send_json(req, 204, "");
}

// ---- lifecycle --------------------------------------------------------------

esp_err_t web_server_start(void) {
    if (g_server) return ESP_OK;
    esp_fill_random(g_session_key, sizeof g_session_key);

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8 * 1024;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.max_uri_handlers = 16;

    if (httpd_start(&g_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        return ESP_FAIL;
    }
    static const httpd_uri_t routes[] = {
        {.uri="/api/auth/state",  .method=HTTP_GET,  .handler=h_auth_state},
        {.uri="/api/auth/init",   .method=HTTP_POST, .handler=h_auth_init},
        {.uri="/api/auth/login",  .method=HTTP_POST, .handler=h_auth_login},
        {.uri="/api/auth/logout", .method=HTTP_POST, .handler=h_auth_logout},
    };
    for (size_t i = 0; i < sizeof routes / sizeof routes[0]; ++i) {
        httpd_register_uri_handler(g_server, &routes[i]);
    }
    touch_activity();
    ESP_LOGI(TAG, "web server up");
    return ESP_OK;
}

esp_err_t web_server_stop(void) {
    if (!g_server) return ESP_OK;
    httpd_stop(g_server);
    g_server = NULL;
    ESP_LOGI(TAG, "web server down");
    return ESP_OK;
}

int64_t web_server_last_activity_us(void) { return g_last_activity_us; }
```

(`web_request_authenticated` declared inline as `bool` — only used in this file for now; later tasks that need it from other TUs will move it to a header. We'll do that when we get there.)

- [ ] **Step 3: Add `web_server.c` to `main/CMakeLists.txt`**

```cmake
set(srcs ... "web_server.c")
# REQUIRES already includes esp_event; ensure esp_http_server is too:
REQUIRES esp_hid driver esp_wifi esp_netif esp_event esp_http_server
```

- [ ] **Step 4: Start the AP + server on TRIPLE_TAP**

In `main/esp_hid_host_main.c`, replace the TRIPLE_TAP case body in the app_main loop:

```c
    case GESTURE_TRIPLE_TAP:
        ESP_LOGI(TAG, "Gesture: TRIPLE_TAP — entering ADMIN mode");
        runtime_mode_set(RUNTIME_MODE_ADMIN);
        set_led_mode(LED_MODE_ADMIN);
        if (wifi_ap_start() == ESP_OK) {
            web_server_start();
        }
        break;
```

Add includes:

```c
#include "wifi_ap.h"
#include "web_server.h"
```

- [ ] **Step 5: Build**

```bash
. /home/hy/System/esp/esp-idf/export.sh
idf.py build
```

Expected: clean. Note the binary size growth.

- [ ] **Step 6: Commit**

```bash
git add main/web_server.h main/web_server.c main/CMakeLists.txt main/esp_hid_host_main.c
git commit -m "Add HTTP server skeleton with auth routes; start on triple-tap"
```

---

## Task 10: /api/status + /api/devices routes

**Files:**
- Modify: `main/web_server.c`

- [ ] **Step 1: Helper exported so other handlers can demand auth**

Move the `web_request_authenticated` declaration to `main/web_server.h`:

```c
#include "esp_http_server.h"
bool web_request_authenticated(struct httpd_req *req);
```

(Forward-declare with `struct httpd_req` to avoid pulling the full header in dependents.)

Remove the `static` from its definition in `web_server.c`.

- [ ] **Step 2: Add status and devices handlers in `main/web_server.c`**

```c
#include "device_store.h"
#include "runtime_mode.h"

static esp_err_t h_status(httpd_req_t *req) {
    if (!web_request_authenticated(req)) return send_json(req, 401, "{\"error\":\"auth\"}");
    touch_activity();
    runtime_mode_t m = runtime_mode_get();
    int64_t uptime_s = esp_timer_get_time() / 1000000;
    int64_t idle_s = (esp_timer_get_time() - g_last_activity_us) / 1000000;
    int64_t idle_remaining_s = (600 - idle_s);
    if (idle_remaining_s < 0) idle_remaining_s = 0;
    char body[160];
    snprintf(body, sizeof body,
        "{\"mode\":\"%s\",\"uptime_s\":%lld,\"ap_idle_remaining_s\":%lld,"
        "\"saved_count\":%d}",
        m == RUNTIME_MODE_ADMIN ? "admin" : "relay",
        uptime_s, idle_remaining_s,
        device_store_list((device_entry_t[DEVICE_STORE_MAX]){0}));
    return send_json(req, 200, body);
}

static esp_err_t h_devices_list(httpd_req_t *req) {
    if (!web_request_authenticated(req)) return send_json(req, 401, "{\"error\":\"auth\"}");
    touch_activity();
    device_entry_t devs[DEVICE_STORE_MAX];
    int n = device_store_list(devs);
    char body[1024];
    int off = snprintf(body, sizeof body, "[");
    for (int i = 0; i < n; ++i) {
        const uint8_t *m = devs[i].bda;
        off += snprintf(body + off, sizeof body - off,
            "%s{\"slot\":%d,\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\","
            "\"name\":\"%.7s\"}",
            i ? "," : "", i, m[0], m[1], m[2], m[3], m[4], m[5], devs[i].name);
    }
    snprintf(body + off, sizeof body - off, "]");
    return send_json(req, 200, body);
}

static esp_err_t h_devices_delete(httpd_req_t *req) {
    if (!web_request_authenticated(req)) return send_json(req, 401, "{\"error\":\"auth\"}");
    touch_activity();
    // URL: /api/devices/<slot>
    const char *uri = req->uri;
    const char *slot_str = strrchr(uri, '/');
    if (!slot_str) return send_json(req, 400, "{\"error\":\"bad_uri\"}");
    int slot = atoi(slot_str + 1);
    if (slot < 0 || slot >= DEVICE_STORE_MAX)
        return send_json(req, 400, "{\"error\":\"bad_slot\"}");
    device_entry_t devs[DEVICE_STORE_MAX];
    int n = device_store_list(devs);
    if (slot >= n) return send_json(req, 404, "{\"error\":\"empty\"}");
    if (!device_store_remove(devs[slot].bda))
        return send_json(req, 500, "{\"error\":\"remove\"}");
    return send_json(req, 204, "");
}
```

Register them in `web_server_start`:

```c
    static const httpd_uri_t more[] = {
        {.uri="/api/status",          .method=HTTP_GET,    .handler=h_status},
        {.uri="/api/devices",         .method=HTTP_GET,    .handler=h_devices_list},
        {.uri="/api/devices/*",       .method=HTTP_DELETE, .handler=h_devices_delete},
    };
    for (size_t i = 0; i < sizeof more / sizeof more[0]; ++i) {
        httpd_register_uri_handler(g_server, &more[i]);
    }
```

- [ ] **Step 3: Build**

```bash
idf.py build
```

Expected: clean.

- [ ] **Step 4: Commit**

```bash
git add main/web_server.c main/web_server.h
git commit -m "Add /api/status, /api/devices GET and DELETE routes"
```

---

## Task 11: Discovery + connect routes (introduces web_cmd queue)

A FreeRTOS queue lets web handlers send commands to `hid_connect`. Used initially for `START_DISCOVERY`/`STOP_DISCOVERY`/`CONNECT_DEVICE`.

**Files:**
- Modify: `main/web_server.c`, `main/web_server.h`, `main/esp_hid_host_main.c`

- [ ] **Step 1: Declare command types in `main/web_server.h`**

```c
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    WEB_CMD_START_DISCOVERY,
    WEB_CMD_STOP_DISCOVERY,
    WEB_CMD_CONNECT,
    WEB_CMD_SHUTDOWN_AP,
} web_cmd_kind_t;

typedef struct {
    web_cmd_kind_t kind;
    uint8_t        bda[6];
    uint8_t        addr_type;
} web_cmd_t;

// Initialized in app_main; visible to both web handlers and hid_connect.
extern QueueHandle_t web_cmd_queue;
```

- [ ] **Step 2: Create the queue in `app_main`**

In `main/esp_hid_host_main.c`, add at top:

```c
#include "web_server.h"
QueueHandle_t web_cmd_queue = NULL;
```

In `app_main` before the gesture loop:

```c
web_cmd_queue = xQueueCreate(4, sizeof(web_cmd_t));
```

- [ ] **Step 3: Add discovery flag and consume the queue in `hid_connect`**

Add the shared struct + accessors to `main/web_server.h`:

```c
#define DISC_RING_MAX 8
typedef struct {
    uint8_t bda[6];
    uint8_t addr_type;
    int8_t  rssi;
    char    name[12];
    int64_t seen_at_us;
} disc_entry_t;
int  hid_discovery_snapshot(disc_entry_t *out, int max);
bool hid_discovery_is_enabled(void);
```

At the top of `esp_hid_host_main.c` (file scope):

```c
static volatile bool s_discovery_enabled = false;
static disc_entry_t  s_disc_ring[DISC_RING_MAX];
static int           s_disc_count = 0;

int hid_discovery_snapshot(disc_entry_t *out, int max) {
    int n = s_disc_count < max ? s_disc_count : max;
    memcpy(out, s_disc_ring, sizeof(disc_entry_t) * n);
    return n;
}
bool hid_discovery_is_enabled(void) { return s_discovery_enabled; }
```

In `hid_connect`, **at the top of the loop body** (just after `vTaskDelay`), drain the queue:

```c
        web_cmd_t cmd;
        while (web_cmd_queue && xQueueReceive(web_cmd_queue, &cmd, 0) == pdTRUE) {
            switch (cmd.kind) {
            case WEB_CMD_START_DISCOVERY:
                s_discovery_enabled = true;
                s_disc_count        = 0;
                break;
            case WEB_CMD_STOP_DISCOVERY:
                s_discovery_enabled = false;
                s_disc_count        = 0;
                break;
            case WEB_CMD_CONNECT: {
                esp_hid_scan_result_t fake = {0};
                memcpy(fake.bda, cmd.bda, 6);
                fake.transport       = ESP_HID_TRANSPORT_BLE;
                fake.ble.addr_type   = cmd.addr_type;
                connect_hid_dev(&fake);
                break;
            }
            case WEB_CMD_SHUTDOWN_AP:
                // handled in Task 12
                break;
            }
        }
```

In the scan-result loop, when a result is **not** in `device_store_contains` and `s_discovery_enabled` is true, add it to `s_disc_ring` (deduplicate by MAC) instead of skipping:

Replace the earlier "not a saved device, ignore" block:

```c
        if (saved_count > 0 && !device_store_contains(r->bda)) {
            if (s_discovery_enabled) {
                // Upsert into discovery ring.
                bool found = false;
                for (int i = 0; i < s_disc_count; ++i) {
                    if (memcmp(s_disc_ring[i].bda, r->bda, 6) == 0) {
                        s_disc_ring[i].rssi = r->rssi;
                        s_disc_ring[i].seen_at_us = esp_timer_get_time();
                        found = true; break;
                    }
                }
                if (!found && s_disc_count < DISC_RING_MAX) {
                    disc_entry_t *e = &s_disc_ring[s_disc_count++];
                    memcpy(e->bda, r->bda, 6);
                    e->addr_type = r->ble.addr_type;
                    e->rssi      = r->rssi;
                    snprintf(e->name, sizeof e->name, "%s",
                             r->name ? r->name : "");
                    e->seen_at_us = esp_timer_get_time();
                }
            }
            r = r->next;
            continue;
        }
```

- [ ] **Step 4: Add discovery + connect routes in `main/web_server.c`**

```c
#include "freertos/queue.h"

static esp_err_t h_discovery_get(httpd_req_t *req) {
    if (!web_request_authenticated(req)) return send_json(req, 401, "{\"error\":\"auth\"}");
    touch_activity();
    disc_entry_t buf[DISC_RING_MAX];
    int n = hid_discovery_snapshot(buf, DISC_RING_MAX);
    char body[1024];
    int off = snprintf(body, sizeof body,
        "{\"enabled\":%s,\"results\":[",
        hid_discovery_is_enabled() ? "true" : "false");
    for (int i = 0; i < n; ++i) {
        const uint8_t *m = buf[i].bda;
        off += snprintf(body + off, sizeof body - off,
            "%s{\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"name\":\"%.11s\","
            "\"rssi\":%d,\"addr_type\":%u}",
            i ? "," : "", m[0],m[1],m[2],m[3],m[4],m[5],
            buf[i].name, buf[i].rssi, buf[i].addr_type);
    }
    snprintf(body + off, sizeof body - off, "]}");
    return send_json(req, 200, body);
}

static esp_err_t h_discovery_post(httpd_req_t *req) {
    if (!web_request_authenticated(req)) return send_json(req, 401, "{\"error\":\"auth\"}");
    touch_activity();
    char body[64]; size_t blen=0;
    if (!body_read(req, body, sizeof body, &blen)) return send_json(req, 400, "{}");
    bool enable = strstr(body, "\"enabled\":true") != NULL;
    web_cmd_t c = { .kind = enable ? WEB_CMD_START_DISCOVERY : WEB_CMD_STOP_DISCOVERY };
    if (web_cmd_queue) xQueueSend(web_cmd_queue, &c, 0);
    return send_json(req, 200, "{\"ok\":true}");
}

// Body: {"mac":"aa:bb:cc:dd:ee:ff","addr_type":1}
static esp_err_t h_connect(httpd_req_t *req) {
    if (!web_request_authenticated(req)) return send_json(req, 401, "{\"error\":\"auth\"}");
    touch_activity();
    char body[128]; size_t blen=0;
    if (!body_read(req, body, sizeof body, &blen)) return send_json(req, 400, "{}");
    char mac_s[18];
    if (!json_extract_string(body, "mac", mac_s, sizeof mac_s))
        return send_json(req, 400, "{\"error\":\"no_mac\"}");
    unsigned m[6];
    if (sscanf(mac_s, "%2x:%2x:%2x:%2x:%2x:%2x",
               &m[0],&m[1],&m[2],&m[3],&m[4],&m[5]) != 6)
        return send_json(req, 400, "{\"error\":\"bad_mac\"}");
    // addr_type may be absent; default to public (0).
    char at[4]; int addr_type = 0;
    if (json_extract_string(body, "addr_type", at, sizeof at)) addr_type = atoi(at);
    web_cmd_t c = { .kind = WEB_CMD_CONNECT, .addr_type = (uint8_t)addr_type };
    for (int i = 0; i < 6; ++i) c.bda[i] = (uint8_t)m[i];
    if (web_cmd_queue) xQueueSend(web_cmd_queue, &c, 0);
    return send_json(req, 202, "{\"ok\":true}");
}
```

Register them in `web_server_start`:

```c
    static const httpd_uri_t more2[] = {
        {.uri="/api/discovery",         .method=HTTP_GET,  .handler=h_discovery_get},
        {.uri="/api/discovery",         .method=HTTP_POST, .handler=h_discovery_post},
        {.uri="/api/devices/connect",   .method=HTTP_POST, .handler=h_connect},
    };
    for (size_t i = 0; i < sizeof more2 / sizeof more2[0]; ++i) {
        httpd_register_uri_handler(g_server, &more2[i]);
    }
```

- [ ] **Step 5: Build**

```bash
idf.py build
```

Expected: clean.

- [ ] **Step 6: Commit**

```bash
git add main/web_server.h main/web_server.c main/esp_hid_host_main.c
git commit -m "Add /api/discovery and /api/devices/connect; web<->hid command queue"
```

---

## Task 12: Admin routes (shutdown_ap, factory_reset)

**Files:**
- Modify: `main/web_server.c`, `main/esp_hid_host_main.c`

- [ ] **Step 1: Add handlers**

```c
#include "wifi_ap.h"

static esp_err_t h_shutdown_ap(httpd_req_t *req) {
    if (!web_request_authenticated(req)) return send_json(req, 401, "{}");
    touch_activity();
    web_cmd_t c = { .kind = WEB_CMD_SHUTDOWN_AP };
    if (web_cmd_queue) xQueueSend(web_cmd_queue, &c, 0);
    return send_json(req, 204, "");
}

static esp_err_t h_factory_reset(httpd_req_t *req) {
    if (!web_request_authenticated(req)) return send_json(req, 401, "{}");
    char body[64]; size_t blen=0;
    if (!body_read(req, body, sizeof body, &blen)) return send_json(req, 400, "{}");
    if (!strstr(body, "\"confirm\":\"WIPE\""))
        return send_json(req, 400, "{\"error\":\"need_confirm\"}");
    send_json(req, 200, "{\"ok\":true}");
    // Brief delay so the response actually flushes before we wipe + restart.
    vTaskDelay(pdMS_TO_TICKS(250));
    storage_factory_reset();
    esp_restart();
    return ESP_OK; // unreachable
}
```

Register:

```c
    static const httpd_uri_t more3[] = {
        {.uri="/api/admin/shutdown_ap",    .method=HTTP_POST, .handler=h_shutdown_ap},
        {.uri="/api/admin/factory_reset",  .method=HTTP_POST, .handler=h_factory_reset},
    };
    for (size_t i = 0; i < sizeof more3 / sizeof more3[0]; ++i) {
        httpd_register_uri_handler(g_server, &more3[i]);
    }
```

- [ ] **Step 2: Handle SHUTDOWN_AP command in `hid_connect`**

Replace the placeholder in Task 11's queue switch:

```c
            case WEB_CMD_SHUTDOWN_AP:
                runtime_mode_set(RUNTIME_MODE_RELAY);
                set_led_mode(LED_MODE_OFF);
                web_server_stop();
                wifi_ap_stop();
                break;
```

- [ ] **Step 3: Build**

```bash
idf.py build
```

- [ ] **Step 4: Commit**

```bash
git add main/web_server.c main/esp_hid_host_main.c
git commit -m "Add /api/admin/shutdown_ap and /api/admin/factory_reset"
```

---

## Task 13: Web assets (HTML/CSS/JS) and static file routes

**Files:**
- Create: `main/web_assets/index.html`, `login.html`, `app.css`, `app.js`
- Modify: `main/web_server.c`, `main/CMakeLists.txt`

- [ ] **Step 1: Write `main/web_assets/app.css`**

```css
*{box-sizing:border-box;margin:0;padding:0;font-family:system-ui,sans-serif}
body{max-width:540px;margin:1rem auto;padding:0 1rem;line-height:1.5;color:#111}
h1{font-size:1.3rem;margin-bottom:1rem}
h2{font-size:1rem;margin:1.5rem 0 .5rem;border-bottom:1px solid #ddd;padding-bottom:.25rem}
button{padding:.5rem 1rem;border:1px solid #888;background:#f5f5f5;border-radius:4px;cursor:pointer}
button:hover{background:#e5e5e5}
button.danger{border-color:#a33;color:#a33}
input{padding:.5rem;border:1px solid #888;border-radius:4px;width:100%}
.row{display:flex;align-items:center;gap:.5rem;padding:.25rem 0}
.row code{flex:1;font-family:monospace;font-size:.85rem;color:#555}
.muted{color:#666;font-size:.85rem}
.err{color:#a33;margin-top:.5rem}
.toggle{display:inline-flex;gap:.5rem;align-items:center}
.toggle input{width:auto}
```

- [ ] **Step 2: Write `main/web_assets/login.html`**

```html
<!doctype html>
<html><head><meta charset=utf-8><title>HID Proxy login</title>
<link rel=stylesheet href=/app.css></head>
<body>
<h1>HID Proxy</h1>
<form id=f>
  <h2 id=t>Login</h2>
  <input id=pw type=password placeholder="Password">
  <input id=pw2 type=password placeholder="Confirm password" style=display:none>
  <div class=err id=err></div>
  <button type=submit style=margin-top:.5rem>Submit</button>
</form>
<script>
const f=document.getElementById('f'),pw=document.getElementById('pw'),
      pw2=document.getElementById('pw2'),err=document.getElementById('err'),
      t=document.getElementById('t');
let init=false;
fetch('/api/auth/state').then(r=>r.json()).then(s=>{
  if(s.logged_in){location='/'}
  init=!s.password_set;
  if(init){t.textContent='Set admin password';pw2.style.display='block'}
});
f.onsubmit=async e=>{e.preventDefault();err.textContent='';
  if(init && pw.value!==pw2.value){err.textContent='Passwords differ';return}
  const path=init?'/api/auth/init':'/api/auth/login';
  const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},
                            body:JSON.stringify({password:pw.value})});
  if(r.ok){location='/'}else{const j=await r.json();err.textContent=j.error||'failed'}};
</script>
</body></html>
```

- [ ] **Step 3: Write `main/web_assets/index.html`**

```html
<!doctype html>
<html><head><meta charset=utf-8><title>HID Proxy</title>
<link rel=stylesheet href=/app.css></head>
<body>
<h1>HID Proxy <button id=logout style=float:right>Logout</button></h1>

<h2>Status</h2>
<div id=status class=muted>Loading…</div>

<h2>Saved keyboards (<span id=savedN>0</span>/5)</h2>
<div id=saved></div>

<h2>Add keyboard</h2>
<label class=toggle><input type=checkbox id=discT> Discover new keyboards</label>
<div id=disc class=muted>Discovery off.</div>

<h2>Admin</h2>
<button id=shut>Shutdown AP now</button>
<button id=reset class=danger>Factory reset</button>

<script src=/app.js></script>
</body></html>
```

- [ ] **Step 4: Write `main/web_assets/app.js`**

```js
const $ = id => document.getElementById(id);
const j = async (m,u,b) => fetch(u,{method:m,headers:{'Content-Type':'application/json'},
                                    body:b?JSON.stringify(b):undefined}).then(r=>r.json());

async function tick() {
  const s = await fetch('/api/status').then(r=>r.ok?r.json():null);
  if (!s) { location = '/login'; return; }
  $('status').innerHTML =
    `Mode: <b>${s.mode}</b> &middot; uptime ${s.uptime_s}s &middot; ` +
    `AP idle remaining ${s.ap_idle_remaining_s}s`;
  $('savedN').textContent = s.saved_count;

  const devs = await fetch('/api/devices').then(r=>r.json());
  $('saved').innerHTML = devs.map(d=>
    `<div class=row><span>${d.name||'(no name)'}</span>
     <code>${d.mac}</code>
     <button data-slot=${d.slot} class=del>Remove</button></div>`).join('') ||
    '<div class=muted>No saved keyboards yet.</div>';
  for (const b of document.querySelectorAll('.del')) b.onclick = async () => {
    await fetch('/api/devices/'+b.dataset.slot, {method:'DELETE'});
    tick();
  };

  if ($('discT').checked) {
    const d = await fetch('/api/discovery').then(r=>r.json());
    $('disc').innerHTML = d.results.length
      ? d.results.map(r=>`<div class=row><span>${r.name||'(no name)'}</span>
            <code>${r.mac}</code> <span class=muted>${r.rssi} dBm</span>
            <button data-mac="${r.mac}" data-at="${r.addr_type}" class=conn>Connect</button></div>`).join('')
      : '<div class=muted>Scanning…</div>';
    for (const b of document.querySelectorAll('.conn')) b.onclick = async () => {
      await j('POST','/api/devices/connect',
              {mac:b.dataset.mac, addr_type: parseInt(b.dataset.at,10)});
    };
  } else {
    $('disc').textContent = 'Discovery off.';
  }
}

$('discT').onchange = async e => {
  await j('POST','/api/discovery', {enabled: e.target.checked});
};
$('logout').onclick = async () => {
  await fetch('/api/auth/logout', {method:'POST'}); location='/login';
};
$('shut').onclick = async () => {
  await fetch('/api/admin/shutdown_ap', {method:'POST'});
};
$('reset').onclick = async () => {
  if (!confirm('Factory reset will erase all saved keyboards and the admin password. Continue?')) return;
  await j('POST','/api/admin/factory_reset', {confirm:'WIPE'});
};
tick();
setInterval(tick, 1000);
```

- [ ] **Step 5: Embed assets in `main/CMakeLists.txt`**

After `idf_component_register(...)` block, append:

```cmake
target_add_binary_data(${COMPONENT_LIB} "web_assets/index.html" TEXT)
target_add_binary_data(${COMPONENT_LIB} "web_assets/login.html" TEXT)
target_add_binary_data(${COMPONENT_LIB} "web_assets/app.css"    TEXT)
target_add_binary_data(${COMPONENT_LIB} "web_assets/app.js"     TEXT)
```

- [ ] **Step 6: Serve assets in `main/web_server.c`**

Add references and handlers:

```c
extern const uint8_t  index_html_start[]  asm("_binary_index_html_start");
extern const uint8_t  index_html_end[]    asm("_binary_index_html_end");
extern const uint8_t  login_html_start[]  asm("_binary_login_html_start");
extern const uint8_t  login_html_end[]    asm("_binary_login_html_end");
extern const uint8_t  app_css_start[]     asm("_binary_app_css_start");
extern const uint8_t  app_css_end[]       asm("_binary_app_css_end");
extern const uint8_t  app_js_start[]      asm("_binary_app_js_start");
extern const uint8_t  app_js_end[]        asm("_binary_app_js_end");

static esp_err_t serve_static(httpd_req_t *req, const char *mime,
                              const uint8_t *start, const uint8_t *end) {
    httpd_resp_set_type(req, mime);
    return httpd_resp_send(req, (const char *)start, end - start);
}

static esp_err_t h_index(httpd_req_t *req) {
    return serve_static(req, "text/html", index_html_start, index_html_end);
}
static esp_err_t h_login_page(httpd_req_t *req) {
    return serve_static(req, "text/html", login_html_start, login_html_end);
}
static esp_err_t h_css(httpd_req_t *req) {
    return serve_static(req, "text/css", app_css_start, app_css_end);
}
static esp_err_t h_js(httpd_req_t *req) {
    return serve_static(req, "application/javascript", app_js_start, app_js_end);
}
```

Register them in `web_server_start`:

```c
    static const httpd_uri_t statics[] = {
        {.uri="/",          .method=HTTP_GET, .handler=h_index},
        {.uri="/login",     .method=HTTP_GET, .handler=h_login_page},
        {.uri="/app.css",   .method=HTTP_GET, .handler=h_css},
        {.uri="/app.js",    .method=HTTP_GET, .handler=h_js},
    };
    for (size_t i = 0; i < sizeof statics / sizeof statics[0]; ++i) {
        httpd_register_uri_handler(g_server, &statics[i]);
    }
```

- [ ] **Step 7: Build**

```bash
idf.py build
```

Expected: clean. Binary should still fit; if not, see spec §10 risk #1.

- [ ] **Step 8: Commit**

```bash
git add main/web_assets/ main/CMakeLists.txt main/web_server.c
git commit -m "Add embedded web assets (index, login, css, js) and static routes"
```

---

## Task 14: Idle-shutdown task

Background watcher that returns to RELAY after 10 min of no authenticated activity.

**Files:**
- Modify: `main/esp_hid_host_main.c`

- [ ] **Step 1: Add the watcher task**

In `esp_hid_host_main.c`:

```c
#define ADMIN_IDLE_TIMEOUT_US (10LL * 60 * 1000 * 1000)

static void admin_idle_task(void *_) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (runtime_mode_get() != RUNTIME_MODE_ADMIN) continue;
        int64_t idle = esp_timer_get_time() - web_server_last_activity_us();
        if (idle > ADMIN_IDLE_TIMEOUT_US) {
            ESP_LOGI(TAG, "ADMIN idle 10 min — back to RELAY");
            web_cmd_t c = { .kind = WEB_CMD_SHUTDOWN_AP };
            if (web_cmd_queue) xQueueSend(web_cmd_queue, &c, 0);
        }
    }
}
```

Create the task in `app_main` after the others:

```c
xTaskCreate(&admin_idle_task, "admin_idle", 2048, NULL, 1, NULL);
```

- [ ] **Step 2: Build**

```bash
idf.py build
```

- [ ] **Step 3: Commit**

```bash
git add main/esp_hid_host_main.c
git commit -m "Auto-shutdown ADMIN after 10 min idle"
```

---

## Task 15: README + final self-check

**Files:**
- Modify: `README.org`

- [ ] **Step 1: Update README**

Add a new top-level section after `* Tests`:

```
* Web Admin UI

The proxy has a triggered Wi-Fi soft-AP that hosts a small admin web UI.

Triple-tap the boot button (IO9) to enter ADMIN mode. The LED switches to a
double-pulse-every-2s pattern. The proxy then exposes:

- SSID =cidproxy-XXXX= where =XXXX= is the last 4 hex digits of the MAC.
- WPA2 password = last 8 hex digits of the MAC (printed on serial each boot).
- Web UI at =http://192.168.4.1=.

On first connect the UI prompts you to set an admin password (≥8 chars).
Subsequent connects show a login form.

Inside the UI you can:
- See and remove saved keyboards (up to 5).
- Enable "Discover new keyboards" to surface unsaved nearby BLE keyboards;
  click Connect to pair.
- Shut down the AP manually or trigger a factory reset.

ADMIN auto-shuts off after 10 minutes idle.

Button gestures:

| Gesture        | Action                                       |
| Hold ~1s       | Forget the currently-connected keyboard      |
| Hold ~5s       | Factory reset (wipe NVS, bonds, password)    |
| Triple-tap     | Enter ADMIN mode                             |
```

Update Known issues if any have surfaced.

- [ ] **Step 2: Self-check greps**

```bash
echo "=== removed APIs ==="
grep -rn 'save_ble_device\|read_ble_device\|clear_ble_devices' main/
echo "=== device_store usage ==="
grep -rn 'device_store_' main/ | wc -l
echo "=== unused includes ==="
# no automated tool here; manual scan of esp_hid_host_main.c for stale includes
```

Expected: first command prints nothing (all replaced); second command shows >0.

- [ ] **Step 3: Final host tests**

```bash
cd test/host && cmake --build build && ./build/run_tests
```

Expected: `38 Tests 0 Failures 0 Ignored OK`.

- [ ] **Step 4: Final firmware build with size report**

```bash
idf.py build
idf.py size
```

Expected: app partition ≤95% full. Note the size for the PR.

- [ ] **Step 5: Commit**

```bash
git add README.org
git commit -m "Document Web Admin UI in README"
```

---

## Task 16: On-device verification (manual, user-only)

Run the verification checklist from spec §9.2 on real hardware at `/dev/ttyACM1`. The maintainer (not an agent) executes each step and ticks the boxes. If any step fails, stop and diagnose.

- [ ] **Step 1: Flash and serial-watch**

```bash
. /home/hy/System/esp/esp-idf/export.sh
PORT=/dev/ttyACM1 ./build_flash_monitor.sh
```

- [ ] **Step 2: First-boot path**

- Boot logs show SSID and PSK.
- Triple-tap IO9 → "entering ADMIN mode" log.
- Phone sees `hidproxy-XXXX`, joins with the printed PSK.
- Open http://192.168.4.1 → first-time "Set admin password" screen.
- Set 8-char password → land on dashboard.

- [ ] **Step 3: Saved-device persistence**

- Pair a keyboard via the discovery toggle → it appears in saved list.
- Reboot → keyboard auto-reconnects in RELAY mode.

- [ ] **Step 4: Multi-device + eviction**

- Add 4 more (total 5).
- Add a 6th → oldest entry is gone; serial logs "shift oldest out".

- [ ] **Step 5: Gestures**

- 1s hold → only active device removed; rest persist.
- 5s hold → reboot; next boot has no saved devices and `password_set:false`.

- [ ] **Step 6: Idle shutdown**

- Enter ADMIN, idle 10 min → "ADMIN idle 10 min — back to RELAY" log; AP disappears.

- [ ] **Step 7: Watchdog soak**

- Type continuously 5 min in RELAY → no resets.

If any step fails, capture the relevant log lines and stop. Likely culprits:
- AP can't start → Wi-Fi NVS partition collision; try `idf.py erase-flash` once.
- HTTP 500 on login → check PBKDF2 stack usage with `uxTaskGetStackHighWaterMark`.
- Discovery toggle on but nothing appears → confirm `s_discovery_enabled` is true via a temp log line in `hid_connect`.

No commit at the end of this task.

---

## Cross-task invariants (skim before executing)

- `LED_MODE_*` constants stay in sync between `main/led.h` and `main/led_duty.h`.
- `STORAGE_NAMESPACE` is `"hidproxy_v3"` everywhere it appears.
- Only one place writes `ble_status.status = BLE_STATUS_CONNECTED`: the success branch of `ESP_HIDH_OPEN_EVENT`.
- All BLE-host mutations happen in `hid_connect` or `hidh_callback`; web handlers never call `esp_hidh_*` directly.
- `web_request_authenticated` is called at the top of every `/api/*` handler except `/api/auth/state`, `/api/auth/init`, and `/api/auth/login`.
- Host tests `cd test/host && cmake --build build && ./build/run_tests` pass after every TDD task.
- `idf.py build` succeeds with no new warnings on touched files after every firmware task.
