#include "unity.h"
#include "device_store.h"
#include "nvs_stub.h"
#include <stdio.h>
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
