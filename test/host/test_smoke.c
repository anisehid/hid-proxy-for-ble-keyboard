#include "unity.h"

static void test_smoke_passes(void) {
    TEST_ASSERT_EQUAL_INT(2, 1 + 1);
}

void run_smoke_tests(void) {
    RUN_TEST(test_smoke_passes);
}
