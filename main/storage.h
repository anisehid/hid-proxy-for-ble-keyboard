#ifndef HID_PROXY_STORAGE_H
#define HID_PROXY_STORAGE_H

#include "driver/gpio.h"
#include "esp_bt_defs.h"
#include "esp_err.h"
#include "esp_hid_gap.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define STORAGE_NAMESPACE    "hidproxy_v3"
#define STORAGE_V2_NAMESPACE "hidproxy_v2"
#define BOOT_MODE_PIN        GPIO_NUM_9
#define SCHEMA_VERSION_KEY   "schema_version"
#define SCHEMA_VERSION_VAL   3
#define ADMIN_PW_HASH_KEY    "admin_pw_hash"
#define ADMIN_PW_HASH_LEN    48   // 16 salt + 32 hash

// Initialize NVS flash + install real NVS ops on device_store. Does NOT run
// any v2->v3 migration; that happens in storage_complete_migration after
// Bluedroid is up.
esp_err_t init_nvs_flash(void);

// Finish first-boot migration. Must be called AFTER esp_hid_gap_init() so the
// bond list is available if needed. Idempotent: no-op once schema_version is
// written.
esp_err_t storage_complete_migration(void);

// Wipe all saved devices, bonds, and NVS state. Used by both the 5s-hold
// gesture and /api/admin/factory_reset.
esp_err_t storage_factory_reset(void);

// Admin password hash (salt || PBKDF2 hash).
bool storage_admin_pw_set(const uint8_t hash[ADMIN_PW_HASH_LEN]);
bool storage_admin_pw_get(uint8_t hash[ADMIN_PW_HASH_LEN]);
bool storage_admin_pw_is_set(void);

#endif // HID_PROXY_STORAGE_H
