#include "unity.h"
#include "button_gesture.h"

#define MS *1000LL

static gesture_ctx_t ctx;

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

static void test_release_in_dead_zone_returns_to_idle(void) {
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

static void test_tap_then_hold_then_two_taps_no_triple(void) {
    // Regression: tap_count must not survive a hold gesture.
    // 1) one tap; 2) hold 1.5s -> HOLD_1S; 3) two more taps must NOT emit TRIPLE_TAP.
    // The follow-up press must occur within the tap window (<500ms) of the
    // previous tap's release so that the GS_COUNTING_TAPS timeout doesn't
    // pre-clear tap_count for us — only the hold-commit code path is at issue.
    reset();
    // Tap 1: press at 0, release at 100ms (last_tap_us = 100ms).
    gesture_step(&ctx, 0, 0);
    gesture_step(&ctx, 1, 100 MS);
    // Press again at 300ms — within 500ms tap window, so tap_count stays at 1.
    gesture_step(&ctx, 0, 300 MS);
    gesture_step(&ctx, 0, 1300 MS);                          // crosses 1s -> HOLDING_1S
    gesture_t held = gesture_step(&ctx, 1, 1800 MS);         // release after 1.5s held
    TEST_ASSERT_EQUAL_INT(GESTURE_HOLD_1S, held);
    // Two more taps within tap window of each other.
    gesture_step(&ctx, 0, 2000 MS);
    gesture_step(&ctx, 1, 2100 MS);                          // tap A
    gesture_step(&ctx, 0, 2300 MS);
    gesture_t final = gesture_step(&ctx, 1, 2400 MS);        // tap B
    // Must be GESTURE_NONE, not GESTURE_TRIPLE_TAP.
    TEST_ASSERT_EQUAL_INT(GESTURE_NONE, final);
}

static void test_release_at_exactly_300ms_counts_as_tap(void) {
    // Boundary: held == GESTURE_TAP_MAX_US is inclusive (<=300ms = tap).
    reset();
    gesture_step(&ctx, 0, 0);
    TEST_ASSERT_EQUAL_INT(GESTURE_NONE, gesture_step(&ctx, 1, 300 MS));
    // Internal: tap_count should be 1. Two more taps within window -> TRIPLE_TAP.
    gesture_step(&ctx, 0, 400 MS);
    gesture_step(&ctx, 1, 500 MS);
    gesture_step(&ctx, 0, 600 MS);
    TEST_ASSERT_EQUAL_INT(GESTURE_TRIPLE_TAP, gesture_step(&ctx, 1, 700 MS));
}

static void test_release_at_301ms_is_dead_zone(void) {
    // Boundary: held > 300ms and < 1000ms = dead zone, no tap counted.
    reset();
    gesture_step(&ctx, 0, 0);
    TEST_ASSERT_EQUAL_INT(GESTURE_NONE, gesture_step(&ctx, 1, 301 MS));
    // Two more taps must NOT yield TRIPLE_TAP because previous release was dead.
    gesture_step(&ctx, 0, 400 MS);
    gesture_step(&ctx, 1, 500 MS);
    gesture_step(&ctx, 0, 600 MS);
    TEST_ASSERT_EQUAL_INT(GESTURE_NONE, gesture_step(&ctx, 1, 700 MS));
}

static void test_hold_release_at_exactly_1000ms_emits_hold_1s(void) {
    // Boundary: held == GESTURE_HOLD_1S_US must trigger HOLD_1S.
    reset();
    gesture_step(&ctx, 0, 0);
    gesture_step(&ctx, 0, 1000 MS);                  // crosses threshold (>= 1s)
    TEST_ASSERT_EQUAL_INT(GESTURE_HOLD_1S, gesture_step(&ctx, 1, 1000 MS));
}

static void test_hold_release_at_exactly_5000ms_emits_hold_5s(void) {
    // Boundary: held == GESTURE_HOLD_5S_US must trigger HOLD_5S.
    reset();
    gesture_step(&ctx, 0, 0);
    gesture_step(&ctx, 0, 1000 MS);                  // -> HOLDING_1S
    gesture_step(&ctx, 0, 5000 MS);                  // -> HOLDING_5S
    TEST_ASSERT_EQUAL_INT(GESTURE_HOLD_5S, gesture_step(&ctx, 1, 5000 MS));
}

void run_button_gesture_tests(void) {
    RUN_TEST(test_quiet_idle_emits_nothing);
    RUN_TEST(test_single_tap_emits_nothing);
    RUN_TEST(test_double_tap_emits_nothing);
    RUN_TEST(test_triple_tap_emits);
    RUN_TEST(test_hold_1s_release_emits_hold_1s);
    RUN_TEST(test_hold_5s_release_emits_hold_5s);
    RUN_TEST(test_release_in_dead_zone_returns_to_idle);
    RUN_TEST(test_press_during_counting_resets_tap_count_if_too_late);
    RUN_TEST(test_tap_then_hold_then_two_taps_no_triple);
    RUN_TEST(test_release_at_exactly_300ms_counts_as_tap);
    RUN_TEST(test_release_at_301ms_is_dead_zone);
    RUN_TEST(test_hold_release_at_exactly_1000ms_emits_hold_1s);
    RUN_TEST(test_hold_release_at_exactly_5000ms_emits_hold_5s);
}
