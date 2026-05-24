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
#include <stdio.h>

#define STORAGE_NAMESPACE   "hidproxy_v2"
#define STORAGE_OLD_NAMESPACE "storage"
#define BOOT_MODE_PIN       GPIO_NUM_9
#define BLE_RESULTS_STORAGE "ble_results"
#define BLE_STATUS          "ble_status"
#define SCHEMA_VERSION_KEY  "schema_version"
#define SCHEMA_VERSION_VAL  2

esp_err_t init_nvs_flash(void);

// Finish first-boot migration (erases old Bluedroid bonds, writes schema_version).
// Must be called AFTER esp_hid_gap_init() so the bond list is available.
// Idempotent: no-op once schema_version is written.
esp_err_t storage_complete_migration(void);

bool save_ble_device(uint8_t *mac_addr);
bool read_ble_device(uint8_t *mac_addr);
bool clear_ble_devices(void);

#endif // HID_PROXY_STORAGE_H
