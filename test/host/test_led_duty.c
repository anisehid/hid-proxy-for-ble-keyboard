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
