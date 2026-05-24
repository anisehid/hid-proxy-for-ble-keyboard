#ifndef WEB_AUTH_H_
#define WEB_AUTH_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define WEB_AUTH_SALT_LEN     16
#define WEB_AUTH_HASH_LEN     32
#define WEB_AUTH_STORED_LEN   (WEB_AUTH_SALT_LEN + WEB_AUTH_HASH_LEN)
#define WEB_AUTH_PBKDF2_ROUNDS 10000

#define WEB_AUTH_TOKEN_LEN    40   // 8-byte exp + 16 random + 16 HMAC

// Compute salt||hash for a password. salt is the first WEB_AUTH_SALT_LEN
// bytes of out; hash follows. Caller provides a 16-byte random salt
// (zeros allowed in tests). out must be at least WEB_AUTH_STORED_LEN bytes.
void web_auth_hash_password(const char *password,
                            const uint8_t salt[WEB_AUTH_SALT_LEN],
                            uint8_t out[WEB_AUTH_STORED_LEN]);

// Constant-time compare of a candidate password against stored salt||hash.
bool web_auth_verify_password(const char *candidate,
                              const uint8_t stored[WEB_AUTH_STORED_LEN]);

// Mint a session token: 8-byte expiry (ms-since-epoch BE) + 16 random + 16 HMAC.
// `key` must be a 32-byte process-wide secret. Returns true on success.
bool web_auth_token_mint(uint64_t expiry_ms,
                         const uint8_t random_nonce[16],
                         const uint8_t key[32],
                         uint8_t out[WEB_AUTH_TOKEN_LEN]);

// Verify a token: HMAC check + expiry check. Returns 0 if valid,
// negative for tampered (HMAC mismatch), -2 for expired.
int web_auth_token_verify(const uint8_t token[WEB_AUTH_TOKEN_LEN],
                          uint64_t now_ms,
                          const uint8_t key[32]);

#endif
