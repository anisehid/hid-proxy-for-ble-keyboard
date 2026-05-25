#include "web_server.h"
#include "device_store.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "runtime_mode.h"
#include "storage.h"
#include "web_auth.h"
#include "wifi_ap.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "WEB";
static httpd_handle_t g_server = NULL;
static int64_t        g_last_activity_us = 0;

extern const uint8_t  index_html_start[]  asm("_binary_index_html_start");
extern const uint8_t  index_html_end[]    asm("_binary_index_html_end");
extern const uint8_t  login_html_start[]  asm("_binary_login_html_start");
extern const uint8_t  login_html_end[]    asm("_binary_login_html_end");
extern const uint8_t  app_css_start[]     asm("_binary_app_css_start");
extern const uint8_t  app_css_end[]       asm("_binary_app_css_end");
extern const uint8_t  app_js_start[]      asm("_binary_app_js_start");
extern const uint8_t  app_js_end[]        asm("_binary_app_js_end");

// Process-wide secret for signing session tokens (regenerated on boot).
static uint8_t g_session_key[32];

#define COOKIE_NAME "hp_session"

static void touch_activity(void) { g_last_activity_us = esp_timer_get_time(); }

// ---- helpers ----------------------------------------------------------------

static esp_err_t send_json(httpd_req_t *req, int code, const char *json) {
    httpd_resp_set_status(req, (code == 200) ? "200 OK"
                              : (code == 201) ? "201 Created"
                              : (code == 202) ? "202 Accepted"
                              : (code == 204) ? "204 No Content"
                              : (code == 400) ? "400 Bad Request"
                              : (code == 401) ? "401 Unauthorized"
                              : (code == 404) ? "404 Not Found"
                              : (code == 409) ? "409 Conflict"
                              : (code == 429) ? "429 Too Many Requests"
                              : (code == 503) ? "503 Service Unavailable"
                              : "500 Internal Server Error");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json ? json : "");
}

static bool body_read(httpd_req_t *req, char *buf, size_t buflen, size_t *out_len) {
    int total = req->content_len;
    if (total < 0 || (size_t)total >= buflen) return false;
    int got = 0;
    while (got < total) {
        int n = httpd_req_recv(req, buf + got, total - got);
        if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (n <= 0) return false;
        got += n;
    }
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
    // Find COOKIE_NAME="..." with a proper name boundary so "Xhp_session=..." doesn't match.
    const char *needle = COOKIE_NAME "=";
    char *p = cookie;
    for (;;) {
        p = strstr(p, needle);
        if (!p) return false;
        if (p == cookie || p[-1] == ';' || p[-1] == ' ') break;
        p++;
    }
    p += strlen(needle);
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
    // Slots default to 0, which means "never used"; skip those so that fresh
    // boots aren't locked out for the first 60 s after start-up.
    static int64_t fail_times_us[5] = {0};
    int64_t now = esp_timer_get_time();
    int recent = 0;
    for (int i = 0; i < 5; ++i) {
        if (fail_times_us[i] != 0 &&
            now - fail_times_us[i] < 60LL * 1000 * 1000) recent++;
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
    int ble_st = hid_ble_status();
    uint8_t cbda[6] = {0};
    bool has_conn = hid_connected_bda(cbda);
    char conn_field[64] = "null";
    if (has_conn) {
        snprintf(conn_field, sizeof conn_field,
            "\"%02x:%02x:%02x:%02x:%02x:%02x\"",
            cbda[0], cbda[1], cbda[2], cbda[3], cbda[4], cbda[5]);
    }
    char body[256];
    snprintf(body, sizeof body,
        "{\"mode\":\"%s\",\"uptime_s\":%lld,\"ap_idle_remaining_s\":%lld,"
        "\"saved_count\":%d,\"ble_status\":%d,\"connected_device\":%s}",
        m == RUNTIME_MODE_ADMIN ? "admin" : "relay",
        uptime_s, idle_remaining_s, saved_count, ble_st, conn_field);
    ESP_LOGI(TAG, "GET /api/status -> %s", body);
    return send_json(req, 200, body);
}

static esp_err_t h_devices_list(httpd_req_t *req) {
    if (!web_request_authenticated(req)) return send_json(req, 401, "{\"error\":\"auth\"}");
    touch_activity();
    device_entry_t devs[DEVICE_STORE_MAX];
    int n = device_store_list(devs);
    uint8_t cbda[6] = {0};
    bool has_conn = hid_connected_bda(cbda);
    char body[1024];
    int off = snprintf(body, sizeof body, "[");
    for (int i = 0; i < n; ++i) {
        if (off >= (int)sizeof body - 1) break;
        const uint8_t *m = devs[i].bda;
        bool is_connected = has_conn && memcmp(m, cbda, 6) == 0;
        off += snprintf(body + off, sizeof body - off,
            "%s{\"slot\":%d,\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\","
            "\"name\":\"%.7s\",\"is_connected\":%s}",
            i ? "," : "", i, m[0], m[1], m[2], m[3], m[4], m[5], devs[i].name,
            is_connected ? "true" : "false");
    }
    if (off < (int)sizeof body - 1) {
        snprintf(body + off, sizeof body - off, "]");
    } else {
        body[sizeof body - 1] = 0;
    }
    ESP_LOGI(TAG, "GET /api/devices -> %d devices", n);
    return send_json(req, 200, body);
}

static esp_err_t h_discovery_get(httpd_req_t *req) {
    if (!web_request_authenticated(req)) return send_json(req, 401, "{\"error\":\"auth\"}");
    touch_activity();
    disc_entry_t buf[DISC_RING_MAX];
    int n = hid_discovery_snapshot(buf, DISC_RING_MAX);
    char body[1024];
    int off = snprintf(body, sizeof body,
        "{\"enabled\":%s,\"results\":[",
        hid_discovery_is_enabled() ? "true" : "false");
    for (int i = 0; i < n; ++i) {
        if (off >= (int)sizeof body - 1) break;
        const uint8_t *m = buf[i].bda;
        off += snprintf(body + off, sizeof body - off,
            "%s{\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"name\":\"%.11s\","
            "\"rssi\":%d,\"addr_type\":%u}",
            i ? "," : "", m[0], m[1], m[2], m[3], m[4], m[5],
            buf[i].name, buf[i].rssi, buf[i].addr_type);
    }
    if (off < (int)sizeof body - 1) {
        snprintf(body + off, sizeof body - off, "]}");
    } else {
        body[sizeof body - 1] = 0;
    }
    ESP_LOGI(TAG, "GET /api/discovery -> enabled=%d n=%d",
             (int)hid_discovery_is_enabled(), n);
    return send_json(req, 200, body);
}

static esp_err_t h_discovery_post(httpd_req_t *req) {
    if (!web_request_authenticated(req)) return send_json(req, 401, "{\"error\":\"auth\"}");
    touch_activity();
    char body[64]; size_t blen = 0;
    if (!body_read(req, body, sizeof body, &blen))
        return send_json(req, 400, "{\"error\":\"bad_body\"}");
    bool enable = strstr(body, "\"enabled\":true") != NULL;
    ESP_LOGI(TAG, "POST /api/discovery enable=%d", (int)enable);
    web_cmd_t c = { .kind = enable ? WEB_CMD_START_DISCOVERY : WEB_CMD_STOP_DISCOVERY };
    if (!web_cmd_queue || xQueueSend(web_cmd_queue, &c, 0) != pdTRUE) {
        return send_json(req, 503, "{\"error\":\"busy\"}");
    }
    return send_json(req, 200, "{\"ok\":true}");
}

// Body: {"mac":"aa:bb:cc:dd:ee:ff","addr_type":1}
static esp_err_t h_connect(httpd_req_t *req) {
    if (!web_request_authenticated(req)) return send_json(req, 401, "{\"error\":\"auth\"}");
    touch_activity();
    char body[128]; size_t blen = 0;
    if (!body_read(req, body, sizeof body, &blen))
        return send_json(req, 400, "{\"error\":\"bad_body\"}");
    char mac_s[18];
    if (!json_extract_string(body, "mac", mac_s, sizeof mac_s))
        return send_json(req, 400, "{\"error\":\"no_mac\"}");
    unsigned m[6];
    if (sscanf(mac_s, "%2x:%2x:%2x:%2x:%2x:%2x",
               &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) != 6)
        return send_json(req, 400, "{\"error\":\"bad_mac\"}");
    // addr_type may be absent; default to public (0).
    char at[4]; int addr_type = 0;
    if (json_extract_string(body, "addr_type", at, sizeof at)) addr_type = atoi(at);
    web_cmd_t c = { .kind = WEB_CMD_CONNECT, .addr_type = (uint8_t)addr_type };
    for (int i = 0; i < 6; ++i) c.bda[i] = (uint8_t)m[i];
    if (!web_cmd_queue || xQueueSend(web_cmd_queue, &c, 0) != pdTRUE) {
        return send_json(req, 503, "{\"error\":\"busy\"}");
    }
    return send_json(req, 202, "{\"ok\":true}");
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

static esp_err_t h_shutdown_ap(httpd_req_t *req) {
    if (!web_request_authenticated(req)) return send_json(req, 401, "{\"error\":\"auth\"}");
    touch_activity();
    web_cmd_t c = { .kind = WEB_CMD_SHUTDOWN_AP };
    if (!web_cmd_queue || xQueueSend(web_cmd_queue, &c, 0) != pdTRUE) {
        return send_json(req, 503, "{\"error\":\"busy\"}");
    }
    return send_json(req, 204, "");
}

static esp_err_t h_factory_reset(httpd_req_t *req) {
    if (!web_request_authenticated(req)) return send_json(req, 401, "{\"error\":\"auth\"}");
    char body[64]; size_t blen = 0;
    if (!body_read(req, body, sizeof body, &blen))
        return send_json(req, 400, "{\"error\":\"bad_body\"}");
    if (!strstr(body, "\"confirm\":\"WIPE\""))
        return send_json(req, 400, "{\"error\":\"need_confirm\"}");
    send_json(req, 200, "{\"ok\":true}");
    // Brief delay so the response actually flushes before we wipe + restart.
    vTaskDelay(pdMS_TO_TICKS(250));
    storage_factory_reset();
    esp_restart();
    return ESP_OK; // unreachable
}

// ---- static assets ----------------------------------------------------------

static esp_err_t serve_static(httpd_req_t *req, const char *mime,
                              const uint8_t *start, const uint8_t *end) {
    httpd_resp_set_type(req, mime);
    // Assets are embedded with target_add_binary_data(... TEXT), which appends a
    // trailing NUL; _binary_*_end points past it. Drop that byte from the wire.
    size_t len = (size_t)(end - start);
    if (len > 0 && start[len - 1] == '\0') len--;
    return httpd_resp_send(req, (const char *)start, len);
}

static esp_err_t h_index(httpd_req_t *req) {
    return serve_static(req, "text/html", index_html_start, index_html_end);
}
static esp_err_t h_login_page(httpd_req_t *req) {
    return serve_static(req, "text/html", login_html_start, login_html_end);
}
static esp_err_t h_css(httpd_req_t *req) {
    return serve_static(req, "text/css", app_css_start, app_css_end);
}
static esp_err_t h_js(httpd_req_t *req) {
    return serve_static(req, "application/javascript", app_js_start, app_js_end);
}

// ---- lifecycle --------------------------------------------------------------

esp_err_t web_server_start(void) {
    if (g_server) return ESP_OK;
    esp_fill_random(g_session_key, sizeof g_session_key);

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8 * 1024;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.max_uri_handlers = 20;

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
    static const httpd_uri_t more2[] = {
        {.uri="/api/discovery",       .method=HTTP_GET,  .handler=h_discovery_get},
        {.uri="/api/discovery",       .method=HTTP_POST, .handler=h_discovery_post},
        {.uri="/api/devices/connect", .method=HTTP_POST, .handler=h_connect},
    };
    for (size_t i = 0; i < sizeof more2 / sizeof more2[0]; ++i) {
        httpd_register_uri_handler(g_server, &more2[i]);
    }
    static const httpd_uri_t more3[] = {
        {.uri="/api/admin/shutdown_ap",   .method=HTTP_POST, .handler=h_shutdown_ap},
        {.uri="/api/admin/factory_reset", .method=HTTP_POST, .handler=h_factory_reset},
    };
    for (size_t i = 0; i < sizeof more3 / sizeof more3[0]; ++i) {
        httpd_register_uri_handler(g_server, &more3[i]);
    }
    static const httpd_uri_t statics[] = {
        {.uri="/",          .method=HTTP_GET, .handler=h_index},
        {.uri="/login",     .method=HTTP_GET, .handler=h_login_page},
        {.uri="/app.css",   .method=HTTP_GET, .handler=h_css},
        {.uri="/app.js",    .method=HTTP_GET, .handler=h_js},
    };
    for (size_t i = 0; i < sizeof statics / sizeof statics[0]; ++i) {
        httpd_register_uri_handler(g_server, &statics[i]);
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
