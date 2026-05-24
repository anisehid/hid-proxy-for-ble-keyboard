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
