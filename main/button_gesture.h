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
#define GESTURE_TAP_WINDOW_US   (500LL * 1000)         // <500 ms between taps
#define GESTURE_HOLD_1S_US      (1000LL * 1000)        // >=1 s = forget
#define GESTURE_HOLD_5S_US      (5000LL * 1000)        // >=5 s = factory reset

// Reset to IDLE.
void gesture_init(gesture_ctx_t *ctx);

// Pure step: feed current button level (0 = pressed, 1 = released)
// and current time. Updates state and returns the gesture to emit
// (GESTURE_NONE if nothing fired this tick).
gesture_t gesture_step(gesture_ctx_t *ctx, int level, int64_t now_us);

#endif
