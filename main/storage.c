#include "storage.h"
#include "esp_bt_defs.h"
#include "esp_err.h"
#include "esp_hid_gap.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include <stddef.h>
#include <stdint.h>

static SemaphoreHandle_t sema_handle = NULL;

static int32_t get_ble_status() {
  nvs_handle_t my_handle;
  esp_err_t err;
  // Open
  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
    return err;
  // Read
  int32_t status = 0;
  err = nvs_get_i32(my_handle, BLE_STATUS, &status);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    return 0;
  nvs_close(my_handle);
  return status;
}

static esp_err_t set_ble_status(int32_t status) {
  nvs_handle_t my_handle;
  esp_err_t err;

  // Open
  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
    return err;

  // Write
  err = nvs_set_i32(my_handle, BLE_STATUS, status);
  if (err != ESP_OK)
    return err;

  err = nvs_commit(my_handle);
  if (err != ESP_OK)
    return err;

  // Close
  nvs_close(my_handle);
  return ESP_OK;
}

static esp_err_t save_ble_results(uint8_t *results) {
  nvs_handle_t handle;
  esp_err_t err;
  // Open
  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK)
    return err;

  // get bt results storage
  size_t required_size = sizeof(esp_bd_addr_t);

  // copy result data
  err = nvs_set_blob(handle, BLE_RESULTS_STORAGE, results, required_size);

  if (err != ESP_OK)
    return err;

  nvs_close(handle);
  return ESP_OK;
}

static esp_err_t read_ble_results(uint8_t *results) {
  nvs_handle_t handle;
  esp_err_t err;

  // open nvs flash
  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK)
    return err;

  size_t required_size = 0; // value will default to 0, if not set yet in NVS
  // obtain required memory space to store blob being read from NVS
  err = nvs_get_blob(handle, BLE_RESULTS_STORAGE, NULL, &required_size);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    return err;

  printf("BLE results:\n");
  if (required_size == 0) {
    printf("Nothing saved yet!\n");
  } else {
    if (required_size != ESP_BD_ADDR_LEN) {
      printf("Wrong required size (%d) from ble_results, should be %d",
             required_size, ESP_BD_ADDR_LEN);
    }
    err = nvs_get_blob(handle, BLE_RESULTS_STORAGE, results, &required_size);
    if (err != ESP_OK) {
      printf("Get blob failed");
      return err;
    }
  }
  // Close
  nvs_close(handle);
  return ESP_OK;
}

bool save_ble_device(uint8_t *mac_addr) {
  if (xSemaphoreTake(sema_handle, (TickType_t)10) == pdTRUE) {
    set_ble_status(1);
    save_ble_results(mac_addr);
    xSemaphoreGive(sema_handle);
    return true;
  }
  return false;
}

bool read_ble_device(uint8_t *mac_addr) {
  if (xSemaphoreTake(sema_handle, (TickType_t)10) == pdTRUE) {
    int32_t status = get_ble_status();
    bool ret = false;
    // found saved
    if (status == 1) {
      ret = true;
      if (ESP_OK != read_ble_results(mac_addr)) {
        ret = false;
      }
    }
    xSemaphoreGive(sema_handle);
    return ret;
  }
  return false;
}

esp_err_t init_nvs_flash() {
  esp_err_t ret;
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  vSemaphoreCreateBinary(sema_handle);
  return ret;
}

bool clear_ble_devices() {
  if (xSemaphoreTake(sema_handle, (TickType_t)10) == pdTRUE) {
    set_ble_status(0);
    xSemaphoreGive(sema_handle);
    return true;
  }
  return false;
}
