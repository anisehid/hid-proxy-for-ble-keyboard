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
