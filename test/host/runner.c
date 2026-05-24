#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

void run_smoke_tests(void);

int main(void) {
    UNITY_BEGIN();
    run_smoke_tests();
    return UNITY_END();
}
