#include "web_auth.h"
#include "sha256.h"
#include <string.h>

#define SHA256_BLOCK_SIZE_BYTES 64
#define HMAC_KEY_MAX SHA256_BLOCK_SIZE_BYTES

static void hmac_sha256(const uint8_t *key, size_t key_len,
                        const uint8_t *msg, size_t msg_len,
                        uint8_t out[32]) {
    uint8_t k[SHA256_BLOCK_SIZE_BYTES] = {0};
    if (key_len > SHA256_BLOCK_SIZE_BYTES) {
        SHA256_CTX c; sha256_init(&c);
        sha256_update(&c, key, key_len);
        sha256_final(&c, k);
    } else {
        memcpy(k, key, key_len);
    }
    uint8_t inner[SHA256_BLOCK_SIZE_BYTES], outer[SHA256_BLOCK_SIZE_BYTES];
    for (int i = 0; i < SHA256_BLOCK_SIZE_BYTES; ++i) {
        inner[i] = k[i] ^ 0x36;
        outer[i] = k[i] ^ 0x5c;
    }
    SHA256_CTX c; sha256_init(&c);
    sha256_update(&c, inner, SHA256_BLOCK_SIZE_BYTES);
    sha256_update(&c, msg, msg_len);
    uint8_t inner_digest[32];
    sha256_final(&c, inner_digest);

    sha256_init(&c);
    sha256_update(&c, outer, SHA256_BLOCK_SIZE_BYTES);
    sha256_update(&c, inner_digest, 32);
    sha256_final(&c, out);
}

static void pbkdf2_sha256(const char *password, const uint8_t *salt, size_t salt_len,
                          unsigned rounds, uint8_t out[32]) {
    size_t pwlen = strlen(password);
    uint8_t block[68];
    memcpy(block, salt, salt_len);
    block[salt_len+0] = 0; block[salt_len+1] = 0;
    block[salt_len+2] = 0; block[salt_len+3] = 1;  // i = 1

    uint8_t u[32], t[32];
    hmac_sha256((const uint8_t *)password, pwlen, block, salt_len + 4, u);
    memcpy(t, u, 32);
    for (unsigned r = 1; r < rounds; ++r) {
        hmac_sha256((const uint8_t *)password, pwlen, u, 32, u);
        for (int i = 0; i < 32; ++i) t[i] ^= u[i];
    }
    memcpy(out, t, 32);
}

void web_auth_hash_password(const char *password,
                            const uint8_t salt[WEB_AUTH_SALT_LEN],
                            uint8_t out[WEB_AUTH_STORED_LEN]) {
    memcpy(out, salt, WEB_AUTH_SALT_LEN);
    pbkdf2_sha256(password, salt, WEB_AUTH_SALT_LEN,
                  WEB_AUTH_PBKDF2_ROUNDS,
                  out + WEB_AUTH_SALT_LEN);
}

bool web_auth_verify_password(const char *candidate,
                              const uint8_t stored[WEB_AUTH_STORED_LEN]) {
    uint8_t computed[WEB_AUTH_STORED_LEN];
    web_auth_hash_password(candidate, stored, computed);
    uint8_t diff = 0;
    for (int i = WEB_AUTH_SALT_LEN; i < WEB_AUTH_STORED_LEN; ++i) {
        diff |= (uint8_t)(computed[i] ^ stored[i]);
    }
    return diff == 0;
}

bool web_auth_token_mint(uint64_t expiry_ms,
                         const uint8_t random_nonce[16],
                         const uint8_t key[32],
                         uint8_t out[WEB_AUTH_TOKEN_LEN]) {
    // [0..7] = expiry big-endian; [8..23] = random; [24..39] = HMAC truncated to 16
    for (int i = 0; i < 8; ++i) out[i] = (uint8_t)(expiry_ms >> (56 - 8*i));
    memcpy(out + 8, random_nonce, 16);
    uint8_t mac[32];
    hmac_sha256(key, 32, out, 24, mac);
    memcpy(out + 24, mac, 16);
    return true;
}

int web_auth_token_verify(const uint8_t token[WEB_AUTH_TOKEN_LEN],
                          uint64_t now_ms,
                          const uint8_t key[32]) {
    uint8_t mac[32];
    hmac_sha256(key, 32, token, 24, mac);
    uint8_t diff = 0;
    for (int i = 0; i < 16; ++i) diff |= (uint8_t)(mac[i] ^ token[24+i]);
    if (diff != 0) return -1;
    uint64_t exp = 0;
    for (int i = 0; i < 8; ++i) exp = (exp << 8) | token[i];
    if (now_ms >= exp) return -2;
    return 0;
}
