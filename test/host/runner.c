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
