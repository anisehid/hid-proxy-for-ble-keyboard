#ifndef WEB_SERVER_H_
#define WEB_SERVER_H_

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdbool.h>
#include <stdint.h>

struct httpd_req;

esp_err_t web_server_start(void);
esp_err_t web_server_stop(void);

// Touched by every authenticated request; consumed by the idle-shutdown task (Task 14).
int64_t web_server_last_activity_us(void);

// Returns true if the request carries a valid (non-expired, well-signed) session cookie.
bool web_request_authenticated(struct httpd_req *req);

// ---- web <-> hid_connect command queue --------------------------------------

typedef enum {
    WEB_CMD_START_DISCOVERY,
    WEB_CMD_STOP_DISCOVERY,
    WEB_CMD_CONNECT,
    WEB_CMD_SHUTDOWN_AP,
} web_cmd_kind_t;

typedef struct {
    web_cmd_kind_t kind;
    uint8_t        bda[6];
    uint8_t        addr_type;
} web_cmd_t;

// Created in app_main; visible to both web handlers and hid_connect.
extern QueueHandle_t web_cmd_queue;

// ---- discovery ring snapshot -------------------------------------------------

#define DISC_RING_MAX 8

typedef struct {
    uint8_t bda[6];
    uint8_t addr_type;
    int8_t  rssi;
    char    name[12];
    int64_t seen_at_us;
} disc_entry_t;

// Copy up to `max` discovery ring entries into `out`. Returns number copied.
int  hid_discovery_snapshot(disc_entry_t *out, int max);

// True if discovery is currently enabled (ADMIN mode + START_DISCOVERY in effect).
bool hid_discovery_is_enabled(void);

// Fill `out[6]` with the currently-connected keyboard's BDA. Returns false if
// no keyboard is currently connected.
bool hid_connected_bda(uint8_t out[6]);

// Raw BLE_STATUS_* code: 0 = unknown, 1 = CONNECTED, 2 = SCAN, 3 = CONNECTING.
int  hid_ble_status(void);

#endif
