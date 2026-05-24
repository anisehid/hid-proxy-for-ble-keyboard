#include "storage.h"
#include "esp_bt_defs.h"
#include "esp_err.h"
#include "esp_gap_ble_api.h"
#include "esp_hid_gap.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

static SemaphoreHandle_t sema_handle = NULL;
static bool g_need_bond_wipe = false;

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
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  if (ret != ESP_OK) {
    return ret;
  }

  vSemaphoreCreateBinary(sema_handle);

  // Phase 1 of first-boot migration: if schema_version is missing from the new
  // namespace, erase the old "storage" namespace and remember to wipe bonds in
  // phase 2 (which requires Bluedroid to be up — see storage_complete_migration).
  nvs_handle_t h;
  esp_err_t open_err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &h);
  if (open_err == ESP_OK) {
    int32_t schema = 0;
    esp_err_t get_err = nvs_get_i32(h, SCHEMA_VERSION_KEY, &schema);
    nvs_close(h);
    if (get_err == ESP_OK && schema == SCHEMA_VERSION_VAL) {
      g_need_bond_wipe = false;
      return ESP_OK;
    }
  }

  // Either the new namespace didn't open or schema_version was missing/wrong:
  // wipe the old namespace and mark phase 2 needed.
  nvs_handle_t old_h;
  if (nvs_open(STORAGE_OLD_NAMESPACE, NVS_READWRITE, &old_h) == ESP_OK) {
    nvs_erase_all(old_h);
    nvs_commit(old_h);
    nvs_close(old_h);
  }
  g_need_bond_wipe = true;
  printf("storage: first-boot phase 1: erased old NVS namespace\n");
  return ESP_OK;
}

esp_err_t storage_complete_migration(void) {
  if (!g_need_bond_wipe) {
    return ESP_OK;
  }

  int dev_num = esp_ble_get_bond_device_num();
  if (dev_num > 0) {
    esp_ble_bond_dev_t *bond_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    if (bond_list == NULL) {
      return ESP_ERR_NO_MEM;
    }
    esp_ble_get_bond_device_list(&dev_num, bond_list);
    for (int i = 0; i < dev_num; ++i) {
      esp_ble_remove_bond_device(bond_list[i].bd_addr);
    }
    free(bond_list);
  }

  nvs_handle_t h;
  esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &h);
  if (err != ESP_OK) {
    return err;
  }
  err = nvs_set_i32(h, SCHEMA_VERSION_KEY, SCHEMA_VERSION_VAL);
  if (err == ESP_OK) {
    err = nvs_commit(h);
  }
  nvs_close(h);

  if (err == ESP_OK) {
    g_need_bond_wipe = false;
    printf("storage: first-boot phase 2: erased %d bonds, schema_version=%d\n",
           dev_num, SCHEMA_VERSION_VAL);
  }
  return err;
}

bool clear_ble_devices() {
  if (xSemaphoreTake(sema_handle, (TickType_t)10) == pdTRUE) {
    set_ble_status(0);
    xSemaphoreGive(sema_handle);
    return true;
  }
  return false;
}
