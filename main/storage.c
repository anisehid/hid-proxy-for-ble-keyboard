#include "storage.h"
#include "device_store.h"
#include "esp_bt_defs.h"
#include "esp_err.h"
#include "esp_gap_ble_api.h"
#include "esp_hid_gap.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "nvs.h"
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

// ---- Real NVS ops wired into device_store ---------------------------------

static int real_open(const char *ns, void **out_handle) {
  nvs_handle_t h;
  esp_err_t err = nvs_open(ns, NVS_READWRITE, &h);
  if (err != ESP_OK) return -1;
  *out_handle = (void *)(uintptr_t)h;
  return 0;
}
static int real_get_blob(void *h, const char *key, void *buf, size_t *size) {
  return nvs_get_blob((nvs_handle_t)(uintptr_t)h, key, buf, size) == ESP_OK ? 0 : -1;
}
static int real_set_blob(void *h, const char *key, const void *buf, size_t size) {
  return nvs_set_blob((nvs_handle_t)(uintptr_t)h, key, buf, size) == ESP_OK ? 0 : -1;
}
static int real_get_u8(void *h, const char *key, uint8_t *out) {
  return nvs_get_u8((nvs_handle_t)(uintptr_t)h, key, out) == ESP_OK ? 0 : -1;
}
static int real_set_u8(void *h, const char *key, uint8_t in) {
  return nvs_set_u8((nvs_handle_t)(uintptr_t)h, key, in) == ESP_OK ? 0 : -1;
}
static int real_get_i32(void *h, const char *key, int32_t *out) {
  return nvs_get_i32((nvs_handle_t)(uintptr_t)h, key, out) == ESP_OK ? 0 : -1;
}
static int real_set_i32(void *h, const char *key, int32_t in) {
  return nvs_set_i32((nvs_handle_t)(uintptr_t)h, key, in) == ESP_OK ? 0 : -1;
}
static int real_erase_all(void *h) {
  return nvs_erase_all((nvs_handle_t)(uintptr_t)h) == ESP_OK ? 0 : -1;
}
static int real_commit(void *h) {
  return nvs_commit((nvs_handle_t)(uintptr_t)h) == ESP_OK ? 0 : -1;
}
static void real_close(void *h) { nvs_close((nvs_handle_t)(uintptr_t)h); }

static const nvs_ops_t g_real_ops = {
    .open = real_open, .get_blob = real_get_blob, .set_blob = real_set_blob,
    .get_u8 = real_get_u8, .set_u8 = real_set_u8,
    .get_i32 = real_get_i32, .set_i32 = real_set_i32,
    .erase_all = real_erase_all, .commit = real_commit, .close = real_close,
};

// ---- Public API ----------------------------------------------------------

esp_err_t init_nvs_flash(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  if (ret != ESP_OK) {
    return ret;
  }

  device_store_set_ops(&g_real_ops);
  return ESP_OK;
}

esp_err_t storage_complete_migration(void) {
  // Check schema_version in v3 namespace.
  nvs_handle_t h;
  int32_t schema = 0;
  if (nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
    if (nvs_get_i32(h, SCHEMA_VERSION_KEY, &schema) == ESP_OK &&
        schema == SCHEMA_VERSION_VAL) {
      nvs_close(h);
      return ESP_OK;
    }
    nvs_close(h);
  }

  // Pull single device from v2 (if present) into v3 multi-device storage.
  device_store_migrate_from_v2();

  // Wipe the v2 namespace so we never re-migrate.
  if (nvs_open(STORAGE_V2_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
    nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
  }

  // Stamp schema_version. If this fails we MUST report it, otherwise the next
  // boot re-runs the v2->v3 migration and produces duplicate entries.
  if (nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
    return ESP_FAIL;
  }
  esp_err_t stamp_err = nvs_set_i32(h, SCHEMA_VERSION_KEY, SCHEMA_VERSION_VAL);
  if (stamp_err == ESP_OK) stamp_err = nvs_commit(h);
  nvs_close(h);
  if (stamp_err != ESP_OK) {
    printf("storage: schema-version stamp failed: %d\n", (int)stamp_err);
    return stamp_err;
  }
  printf("storage: migrated to schema v%d\n", SCHEMA_VERSION_VAL);
  return ESP_OK;
}

esp_err_t storage_factory_reset(void) {
  int dev_num = esp_ble_get_bond_device_num();
  if (dev_num > 0) {
    esp_ble_bond_dev_t *list =
        (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    if (list) {
      esp_ble_get_bond_device_list(&dev_num, list);
      for (int i = 0; i < dev_num; ++i) {
        esp_ble_remove_bond_device(list[i].bd_addr);
      }
      free(list);
    }
  }
  device_store_clear();
  // Wipe BOTH namespaces (defensive against pre-migration installs).
  nvs_handle_t h;
  if (nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
    nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
  }
  if (nvs_open(STORAGE_V2_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
    nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
  }
  return ESP_OK;
}

bool storage_admin_pw_set(const uint8_t hash[ADMIN_PW_HASH_LEN]) {
  nvs_handle_t h;
  if (nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return false;
  bool ok = (nvs_set_blob(h, ADMIN_PW_HASH_KEY, hash, ADMIN_PW_HASH_LEN) == ESP_OK) &&
            (nvs_commit(h) == ESP_OK);
  nvs_close(h);
  return ok;
}
bool storage_admin_pw_get(uint8_t hash[ADMIN_PW_HASH_LEN]) {
  nvs_handle_t h;
  if (nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;
  size_t len = ADMIN_PW_HASH_LEN;
  bool ok = (nvs_get_blob(h, ADMIN_PW_HASH_KEY, hash, &len) == ESP_OK) &&
            (len == ADMIN_PW_HASH_LEN);
  nvs_close(h);
  return ok;
}
bool storage_admin_pw_is_set(void) {
  uint8_t buf[ADMIN_PW_HASH_LEN];
  return storage_admin_pw_get(buf);
}
