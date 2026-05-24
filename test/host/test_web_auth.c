#include "unity.h"
#include "web_auth.h"
#include <string.h>

static void test_hash_deterministic_with_fixed_salt(void) {
    uint8_t salt[WEB_AUTH_SALT_LEN] = {0};
    uint8_t out1[WEB_AUTH_STORED_LEN];
    uint8_t out2[WEB_AUTH_STORED_LEN];
    web_auth_hash_password("hello", salt, out1);
    web_auth_hash_password("hello", salt, out2);
    TEST_ASSERT_EQUAL_MEMORY(out1, out2, WEB_AUTH_STORED_LEN);
}

static void test_hash_different_passwords_differ(void) {
    uint8_t salt[WEB_AUTH_SALT_LEN] = {0};
    uint8_t a[WEB_AUTH_STORED_LEN], b[WEB_AUTH_STORED_LEN];
    web_auth_hash_password("hello", salt, a);
    web_auth_hash_password("world", salt, b);
    TEST_ASSERT_NOT_EQUAL(0, memcmp(a + WEB_AUTH_SALT_LEN,
                                    b + WEB_AUTH_SALT_LEN,
                                    WEB_AUTH_HASH_LEN));
}

static void test_verify_password_round_trip(void) {
    uint8_t salt[WEB_AUTH_SALT_LEN];
    for (int i = 0; i < WEB_AUTH_SALT_LEN; ++i) salt[i] = (uint8_t)i;
    uint8_t stored[WEB_AUTH_STORED_LEN];
    web_auth_hash_password("correcthorse", salt, stored);
    TEST_ASSERT_TRUE (web_auth_verify_password("correcthorse", stored));
    TEST_ASSERT_FALSE(web_auth_verify_password("wronghorse",   stored));
}

static void test_token_round_trip(void) {
    uint8_t key[32];
    for (int i = 0; i < 32; ++i) key[i] = (uint8_t)(i + 1);
    uint8_t nonce[16];
    for (int i = 0; i < 16; ++i) nonce[i] = (uint8_t)(i * 3);
    uint8_t tok[WEB_AUTH_TOKEN_LEN];
    TEST_ASSERT_TRUE(web_auth_token_mint(10000, nonce, key, tok));
    TEST_ASSERT_EQUAL_INT(0,  web_auth_token_verify(tok, 5000,  key));
    TEST_ASSERT_EQUAL_INT(-2, web_auth_token_verify(tok, 11000, key));
}

static void test_token_tampered_rejects(void) {
    uint8_t key[32] = {0};
    uint8_t nonce[16] = {0};
    uint8_t tok[WEB_AUTH_TOKEN_LEN];
    web_auth_token_mint(10000, nonce, key, tok);
    tok[10] ^= 0xFF; // flip a byte in the random portion
    TEST_ASSERT_LESS_THAN_INT(0, web_auth_token_verify(tok, 5000, key));
}

void run_web_auth_tests(void) {
    RUN_TEST(test_hash_deterministic_with_fixed_salt);
    RUN_TEST(test_hash_different_passwords_differ);
    RUN_TEST(test_verify_password_round_trip);
    RUN_TEST(test_token_round_trip);
    RUN_TEST(test_token_tampered_rejects);
}
