#include "web_server.h"
#include "device_store.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "runtime_mode.h"
#include "storage.h"
#include "web_auth.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "WEB";
static httpd_handle_t g_server = NULL;
static int64_t        g_last_activity_us = 0;

// Process-wide secret for signing session tokens (regenerated on boot).
static uint8_t g_session_key[32];

#define COOKIE_NAME "hp_session"

static void touch_activity(void) { g_last_activity_us = esp_timer_get_time(); }

// ---- helpers ----------------------------------------------------------------

static esp_err_t send_json(httpd_req_t *req, int code, const char *json) {
    httpd_resp_set_status(req, (code == 200) ? "200 OK"
                              : (code == 201) ? "201 Created"
                              : (code == 204) ? "204 No Content"
                              : (code == 400) ? "400 Bad Request"
                              : (code == 401) ? "401 Unauthorized"
                              : (code == 409) ? "409 Conflict"
                              : (code == 429) ? "429 Too Many Requests"
                              : "500 Internal Server Error");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json ? json : "");
}

static bool body_read(httpd_req_t *req, char *buf, size_t buflen, size_t *out_len) {
    int total = req->content_len;
    if (total < 0 || (size_t)total >= buflen) return false;
    int got = httpd_req_recv(req, buf, total);
    if (got != total) return false;
    buf[got] = 0;
    *out_len = (size_t)got;
    return true;
}

// Naive substring lookup for "key":"value" in a tiny JSON body.
// Sufficient for our flat single-field payloads.
static bool json_extract_string(const char *body, const char *key,
                                char *out, size_t outlen) {
    char needle[32];
    snprintf(needle, sizeof needle, "\"%s\"", key);
    const char *p = strstr(body, needle);
    if (!p) return false;
    p = strchr(p + strlen(needle), ':');
    if (!p) return false;
    while (*p && (*p == ':' || *p == ' ' || *p == '\t')) p++;
    if (*p != '"') return false;
    p++;
    size_t n = 0;
    while (*p && *p != '"' && n + 1 < outlen) out[n++] = *p++;
    out[n] = 0;
    return *p == '"';
}

static void emit_session_cookie(httpd_req_t *req, const uint8_t token[WEB_AUTH_TOKEN_LEN]) {
    char hex[WEB_AUTH_TOKEN_LEN*2 + 1];
    for (int i = 0; i < WEB_AUTH_TOKEN_LEN; ++i)
        snprintf(hex + i*2, 3, "%02x", token[i]);
    char header[200];
    snprintf(header, sizeof header,
             COOKIE_NAME "=%s; Path=/; HttpOnly; SameSite=Strict; Max-Age=1800",
             hex);
    httpd_resp_set_hdr(req, "Set-Cookie", header);
}

static bool read_session_cookie(httpd_req_t *req, uint8_t token[WEB_AUTH_TOKEN_LEN]) {
    char cookie[256];
    if (httpd_req_get_hdr_value_str(req, "Cookie", cookie, sizeof cookie) != ESP_OK)
        return false;
    char *p = strstr(cookie, COOKIE_NAME "=");
    if (!p) return false;
    p += strlen(COOKIE_NAME "=");
    char *end = strchr(p, ';');
    size_t len = end ? (size_t)(end - p) : strlen(p);
    if (len != WEB_AUTH_TOKEN_LEN * 2) return false;
    for (int i = 0; i < WEB_AUTH_TOKEN_LEN; ++i) {
        unsigned v;
        if (sscanf(p + i*2, "%2x", &v) != 1) return false;
        token[i] = (uint8_t)v;
    }
    return true;
}

bool web_request_authenticated(httpd_req_t *req) {
    uint8_t tok[WEB_AUTH_TOKEN_LEN];
    if (!read_session_cookie(req, tok)) return false;
    uint64_t now_ms = esp_timer_get_time() / 1000;
    return web_auth_token_verify(tok, now_ms, g_session_key) == 0;
}

// ---- handlers ---------------------------------------------------------------

static esp_err_t h_auth_state(httpd_req_t *req) {
    char body[64];
    snprintf(body, sizeof body, "{\"password_set\":%s,\"logged_in\":%s}",
             storage_admin_pw_is_set() ? "true" : "false",
             web_request_authenticated(req) ? "true" : "false");
    return send_json(req, 200, body);
}

static esp_err_t h_auth_init(httpd_req_t *req) {
    if (storage_admin_pw_is_set()) return send_json(req, 409, "{\"error\":\"already_set\"}");
    char body[128]; size_t blen = 0;
    if (!body_read(req, body, sizeof body, &blen))
        return send_json(req, 400, "{\"error\":\"bad_body\"}");
    char pw[64];
    if (!json_extract_string(body, "password", pw, sizeof pw))
        return send_json(req, 400, "{\"error\":\"no_password\"}");
    if (strlen(pw) < 8) return send_json(req, 400, "{\"error\":\"too_short\"}");

    uint8_t salt[WEB_AUTH_SALT_LEN];
    esp_fill_random(salt, sizeof salt);
    uint8_t stored[WEB_AUTH_STORED_LEN];
    web_auth_hash_password(pw, salt, stored);
    if (!storage_admin_pw_set(stored)) return send_json(req, 500, "{\"error\":\"nvs\"}");

    // Issue a session immediately.
    uint8_t nonce[16];
    esp_fill_random(nonce, sizeof nonce);
    uint8_t tok[WEB_AUTH_TOKEN_LEN];
    uint64_t exp = (esp_timer_get_time() / 1000) + 1800 * 1000;
    web_auth_token_mint(exp, nonce, g_session_key, tok);
    emit_session_cookie(req, tok);
    touch_activity();
    return send_json(req, 200, "{\"ok\":true}");
}

static esp_err_t h_auth_login(httpd_req_t *req) {
    // Global sliding-window rate limit: <=5 failed attempts in the last 60 s.
    // (Implementing it per-IP requires lwip socket plumbing that esp_http_server
    // doesn't expose stably; for a soft-AP with at most a handful of clients
    // the global bucket gives the same brute-force protection.)
    static int64_t fail_times_us[5] = {0};
    int64_t now = esp_timer_get_time();
    int recent = 0;
    for (int i = 0; i < 5; ++i) {
        if (now - fail_times_us[i] < 60LL * 1000 * 1000) recent++;
    }
    if (recent >= 5) {
        httpd_resp_set_hdr(req, "Retry-After", "60");
        return send_json(req, 429, "{\"error\":\"too_many\"}");
    }

    char body[128]; size_t blen = 0;
    if (!body_read(req, body, sizeof body, &blen))
        return send_json(req, 400, "{\"error\":\"bad_body\"}");
    char pw[64];
    if (!json_extract_string(body, "password", pw, sizeof pw))
        return send_json(req, 400, "{\"error\":\"no_password\"}");
    uint8_t stored[WEB_AUTH_STORED_LEN];
    if (!storage_admin_pw_get(stored))
        return send_json(req, 401, "{\"error\":\"not_initialized\"}");
    if (!web_auth_verify_password(pw, stored)) {
        // Record failure in the oldest slot.
        int oldest = 0;
        for (int i = 1; i < 5; ++i) {
            if (fail_times_us[i] < fail_times_us[oldest]) oldest = i;
        }
        fail_times_us[oldest] = now;
        return send_json(req, 401, "{\"error\":\"bad_password\"}");
    }
    uint8_t nonce[16];
    esp_fill_random(nonce, sizeof nonce);
    uint8_t tok[WEB_AUTH_TOKEN_LEN];
    uint64_t exp = (esp_timer_get_time() / 1000) + 1800 * 1000;
    web_auth_token_mint(exp, nonce, g_session_key, tok);
    emit_session_cookie(req, tok);
    touch_activity();
    return send_json(req, 200, "{\"ok\":true}");
}

static esp_err_t h_auth_logout(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Set-Cookie",
                       COOKIE_NAME "=; Path=/; HttpOnly; Max-Age=0");
    return send_json(req, 204, "");
}

static esp_err_t h_status(httpd_req_t *req) {
    if (!web_request_authenticated(req)) return send_json(req, 401, "{\"error\":\"auth\"}");
    touch_activity();
    runtime_mode_t m = runtime_mode_get();
    int64_t uptime_s = esp_timer_get_time() / 1000000;
    int64_t idle_s = (esp_timer_get_time() - g_last_activity_us) / 1000000;
    int64_t idle_remaining_s = (600 - idle_s);
    if (idle_remaining_s < 0) idle_remaining_s = 0;
    device_entry_t tmp[DEVICE_STORE_MAX];
    int saved_count = device_store_list(tmp);
    char body[160];
    snprintf(body, sizeof body,
        "{\"mode\":\"%s\",\"uptime_s\":%lld,\"ap_idle_remaining_s\":%lld,"
        "\"saved_count\":%d}",
        m == RUNTIME_MODE_ADMIN ? "admin" : "relay",
        uptime_s, idle_remaining_s, saved_count);
    return send_json(req, 200, body);
}

static esp_err_t h_devices_list(httpd_req_t *req) {
    if (!web_request_authenticated(req)) return send_json(req, 401, "{\"error\":\"auth\"}");
    touch_activity();
    device_entry_t devs[DEVICE_STORE_MAX];
    int n = device_store_list(devs);
    char body[1024];
    int off = snprintf(body, sizeof body, "[");
    for (int i = 0; i < n; ++i) {
        if (off >= (int)sizeof body - 1) break;
        const uint8_t *m = devs[i].bda;
        off += snprintf(body + off, sizeof body - off,
            "%s{\"slot\":%d,\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\","
            "\"name\":\"%.7s\"}",
            i ? "," : "", i, m[0], m[1], m[2], m[3], m[4], m[5], devs[i].name);
    }
    if (off < (int)sizeof body - 1) {
        snprintf(body + off, sizeof body - off, "]");
    } else {
        body[sizeof body - 1] = 0;
    }
    return send_json(req, 200, body);
}

static esp_err_t h_devices_delete(httpd_req_t *req) {
    if (!web_request_authenticated(req)) return send_json(req, 401, "{\"error\":\"auth\"}");
    touch_activity();
    // URL: /api/devices/<slot>
    static const char prefix[] = "/api/devices/";
    if (strncmp(req->uri, prefix, sizeof(prefix) - 1) != 0)
        return send_json(req, 400, "{\"error\":\"bad_uri\"}");
    const char *slot_str = req->uri + sizeof(prefix) - 1;
    if (*slot_str == '\0')
        return send_json(req, 400, "{\"error\":\"bad_slot\"}");
    // Slot must be a pure non-negative integer with no trailing characters.
    int slot = 0;
    for (const char *p = slot_str; *p; ++p) {
        if (*p < '0' || *p > '9')
            return send_json(req, 400, "{\"error\":\"bad_slot\"}");
        slot = slot * 10 + (*p - '0');
        if (slot >= DEVICE_STORE_MAX)
            return send_json(req, 400, "{\"error\":\"bad_slot\"}");
    }
    device_entry_t devs[DEVICE_STORE_MAX];
    int n = device_store_list(devs);
    if (slot >= n) return send_json(req, 404, "{\"error\":\"empty\"}");
    if (!device_store_remove(devs[slot].bda))
        return send_json(req, 500, "{\"error\":\"remove\"}");
    return send_json(req, 204, "");
}

// ---- lifecycle --------------------------------------------------------------

esp_err_t web_server_start(void) {
    if (g_server) return ESP_OK;
    esp_fill_random(g_session_key, sizeof g_session_key);

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8 * 1024;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.max_uri_handlers = 16;

    if (httpd_start(&g_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        return ESP_FAIL;
    }
    static const httpd_uri_t routes[] = {
        {.uri="/api/auth/state",  .method=HTTP_GET,  .handler=h_auth_state},
        {.uri="/api/auth/init",   .method=HTTP_POST, .handler=h_auth_init},
        {.uri="/api/auth/login",  .method=HTTP_POST, .handler=h_auth_login},
        {.uri="/api/auth/logout", .method=HTTP_POST, .handler=h_auth_logout},
    };
    for (size_t i = 0; i < sizeof routes / sizeof routes[0]; ++i) {
        httpd_register_uri_handler(g_server, &routes[i]);
    }
    static const httpd_uri_t more[] = {
        {.uri="/api/status",    .method=HTTP_GET,    .handler=h_status},
        {.uri="/api/devices",   .method=HTTP_GET,    .handler=h_devices_list},
        {.uri="/api/devices/*", .method=HTTP_DELETE, .handler=h_devices_delete},
    };
    for (size_t i = 0; i < sizeof more / sizeof more[0]; ++i) {
        httpd_register_uri_handler(g_server, &more[i]);
    }
    touch_activity();
    ESP_LOGI(TAG, "web server up");
    return ESP_OK;
}

esp_err_t web_server_stop(void) {
    if (!g_server) return ESP_OK;
    httpd_stop(g_server);
    g_server = NULL;
    ESP_LOGI(TAG, "web server down");
    return ESP_OK;
}

int64_t web_server_last_activity_us(void) { return g_last_activity_us; }
