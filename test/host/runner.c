#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

void run_ch9329_pack_tests(void);
void run_led_duty_tests(void);
void run_button_gesture_tests(void);
void run_device_store_tests(void);
void run_web_auth_tests(void);

int main(void) {
    UNITY_BEGIN();
    run_ch9329_pack_tests();
    run_led_duty_tests();
    run_button_gesture_tests();
    run_device_store_tests();
    run_web_auth_tests();
    return UNITY_END();
}
