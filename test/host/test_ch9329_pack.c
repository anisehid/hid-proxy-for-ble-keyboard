#include "unity.h"
#include "ch9329_pack.h"
#include <string.h>

static void test_header_addr_cmd_len(void) {
    uint8_t out[CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD] = {0};
    uint8_t payload[3] = {0x11, 0x22, 0x33};
    int n = pack_ch9329_data(CMD_SEND_KB_GENERAL_DATA, payload, 3, out);
    TEST_ASSERT_EQUAL_INT(3 + CH9329_HEADER_LEN, n);
    TEST_ASSERT_EQUAL_HEX8(0x57, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xAB, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[2]);
    TEST_ASSERT_EQUAL_HEX8(CMD_SEND_KB_GENERAL_DATA, out[3]);
    TEST_ASSERT_EQUAL_HEX8(3,    out[4]);
}

static void test_payload_copied(void) {
    uint8_t out[CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD] = {0};
    uint8_t payload[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    int n = pack_ch9329_data(CMD_SEND_KB_GENERAL_DATA, payload, 4, out);
    TEST_ASSERT_EQUAL_INT(4 + CH9329_HEADER_LEN, n);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, &out[5], 4);
}

static void test_checksum(void) {
    uint8_t out[CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD] = {0};
    uint8_t payload[3] = {0x01, 0x02, 0x03};
    int n = pack_ch9329_data(CMD_SEND_KB_GENERAL_DATA, payload, 3, out);
    uint8_t expected = 0;
    for (int i = 0; i < n - 1; ++i) expected = (uint8_t)(expected + out[i]);
    TEST_ASSERT_EQUAL_HEX8(expected, out[n - 1]);
}

static void test_zero_length(void) {
    uint8_t out[CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD] = {0xFF};
    int n = pack_ch9329_data(CMD_GET_INFO, NULL, 0, out);
    TEST_ASSERT_EQUAL_INT(CH9329_HEADER_LEN, n);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[4]);
    uint8_t expected = (uint8_t)(0x57 + 0xAB + 0x00 + CMD_GET_INFO + 0x00);
    TEST_ASSERT_EQUAL_HEX8(expected, out[5]);
}

static void test_max_length(void) {
    uint8_t out[CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD] = {0};
    uint8_t payload[CH9329_MAX_PAYLOAD];
    for (int i = 0; i < CH9329_MAX_PAYLOAD; ++i) payload[i] = (uint8_t)i;
    int n = pack_ch9329_data(CMD_SEND_KB_GENERAL_DATA, payload, CH9329_MAX_PAYLOAD, out);
    TEST_ASSERT_EQUAL_INT(CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD, n);
}

static void test_length_too_big_returns_zero(void) {
    uint8_t out[CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD] = {0};
    int n = pack_ch9329_data(CMD_SEND_KB_GENERAL_DATA, NULL, CH9329_MAX_PAYLOAD + 1, out);
    TEST_ASSERT_EQUAL_INT(0, n);
}

static void test_negative_length_returns_zero(void) {
    uint8_t out[CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD] = {0};
    int n = pack_ch9329_data(CMD_SEND_KB_GENERAL_DATA, NULL, -1, out);
    TEST_ASSERT_EQUAL_INT(0, n);
}

void run_ch9329_pack_tests(void) {
    RUN_TEST(test_header_addr_cmd_len);
    RUN_TEST(test_payload_copied);
    RUN_TEST(test_checksum);
    RUN_TEST(test_zero_length);
    RUN_TEST(test_max_length);
    RUN_TEST(test_length_too_big_returns_zero);
    RUN_TEST(test_negative_length_returns_zero);
}
