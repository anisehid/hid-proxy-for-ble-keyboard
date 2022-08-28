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

#define STORAGE_NAMESPACE "storage"
#define BOOT_MODE_PIN GPIO_NUM_9
#define BLE_RESULTS_STORAGE "ble_results"
#define BLE_STATUS "ble_status"

/**
 * ble_results data layouts
 * result_number (4B)
 * sizeof(scan_result)(4B) scan_result
 * sizeof(scan_result)(4B) scan_result
 **/

/* esp_err_t save_ble_results(uint8_t *results); */

/* esp_err_t read_ble_results(uint8_t *results); */

/* esp_err_t set_ble_status(int32_t status); */

/* int32_t get_ble_status(); */

esp_err_t init_nvs_flash();

// save a new ble device
// set ble status to 1, and save mac_addr
// return true if success
bool save_ble_device(uint8_t *mac_addr);

// get saved device
// return true if found saved device
// otherwise, return false
bool read_ble_device(uint8_t *mac_addr);

// set ble_status to 0
bool clear_ble_devices();

#endif // HID_PROXY_STORAGE_H
