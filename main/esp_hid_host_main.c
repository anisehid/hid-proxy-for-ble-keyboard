/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch932x.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_gap_ble_api.h"
#include "esp_gap_bt_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "led.h"
#include "nvs_flash.h"

#include "button_gesture.h"
#include "device_store.h"
#include "esp_hid_gap.h"
#include "esp_hidh.h"
#include "runtime_mode.h"
#include "storage.h"
#include "web_server.h"
#include "wifi_ap.h"

#define BLE_STATUS_CONNECTED  1
#define BLE_STATUS_SCAN       2
#define BLE_STATUS_CONNECTING 3

#define BLE_CONNECTED_LED_MODE   LED_MODE_OFF
#define BLE_SCAN_NEW_LED_MODE    LED_MODE_FAST_BLINK
#define BLE_SCAN_SAVED_LED_MODE  LED_MODE_SLOW_BLINK
#define BLE_UNSET_LED_MODE       LED_MODE_ALWAYS_ON

#define CONNECT_TIMEOUT_US        (10LL * 1000 * 1000)
#define FAILED_DEVICE_BACKOFF_US  (30LL * 1000 * 1000)
#define ADMIN_IDLE_TIMEOUT_US     (10LL * 60 * 1000 * 1000)

static const char *TAG = "ESP_HIDH_DEMO";
static esp_hidh_dev_t *connected_dev = NULL;
static EspBleStatus ble_status;

static int64_t s_connecting_since_us = 0;
static esp_bd_addr_t s_failed_bda = {0};
static int64_t s_failed_at_us = 0;

// ---- web <-> hid_connect queue + discovery ring -----------------------------
QueueHandle_t web_cmd_queue = NULL;

static volatile bool s_discovery_enabled = false;
static disc_entry_t  s_disc_ring[DISC_RING_MAX];
static int           s_disc_count = 0;

int hid_discovery_snapshot(disc_entry_t *out, int max) {
  // Acquire load pairs with the release store in the writer above, so memcpy
  // can't be hoisted before the count read.
  int cur = __atomic_load_n(&s_disc_count, __ATOMIC_ACQUIRE);
  int n = cur < max ? cur : max;
  if (n > 0 && out) memcpy(out, s_disc_ring, sizeof(disc_entry_t) * n);
  return n;
}

bool hid_discovery_is_enabled(void) { return s_discovery_enabled; }

bool hid_connected_bda(uint8_t out[6]) {
  if (!connected_dev || !esp_hidh_dev_exists(connected_dev)) return false;
  const uint8_t *bda = esp_hidh_dev_bda_get(connected_dev);
  if (!bda) return false;
  memcpy(out, bda, 6);
  return true;
}

bool hid_connected_name(char *out, size_t outlen) {
  if (outlen == 0) return false;
  out[0] = 0;
  if (!connected_dev || !esp_hidh_dev_exists(connected_dev)) return false;
  const char *name = esp_hidh_dev_name_get(connected_dev);
  if (!name || !name[0]) return false;
  snprintf(out, outlen, "%s", name);
  return true;
}

int hid_ble_status(void) { return ble_status.status; }

void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id,
                   void *event_data) {
  esp_hidh_event_t event = (esp_hidh_event_t)id;
  esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

  switch (event) {
  case ESP_HIDH_OPEN_EVENT: {
    if (param->open.status == ESP_OK) {
      const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
      ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda),
               esp_hidh_dev_name_get(param->open.dev));
      esp_hidh_dev_dump(param->open.dev, stdout);

      device_entry_t entry = {0};
      memcpy(entry.bda, bda, 6);
      entry.addr_type = 0;  // re-determined from live scan on reconnect
      const char *name = esp_hidh_dev_name_get(param->open.dev);
      if (name) snprintf(entry.name, DEVICE_NAME_LEN, "%s", name);
      device_store_upsert(&entry);

      set_led_mode(BLE_CONNECTED_LED_MODE);
      ble_status.status = BLE_STATUS_CONNECTED;
      connected_dev = param->open.dev;
    } else {
      const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
      ESP_LOGE(TAG, ESP_BD_ADDR_STR " OPEN failed: status=0x%x",
               ESP_BD_ADDR_HEX(bda), param->open.status);
      memcpy(s_failed_bda, bda, sizeof(esp_bd_addr_t));
      s_failed_at_us = esp_timer_get_time();
      esp_hidh_dev_free(param->open.dev);
      ble_status.status = BLE_STATUS_SCAN;
    }
    break;
  }
  case ESP_HIDH_BATTERY_EVENT: {
    const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
    ESP_LOGI(TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda),
             param->battery.level);
    break;
  }
  case ESP_HIDH_INPUT_EVENT: {
    const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
    ESP_LOGI(
        TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:",
        ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage),
        param->input.map_index, param->input.report_id, param->input.length);
    /* ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length); */

    // Send keycodes to ch9328/ch9329
#ifdef USE_CH9329
    int n = pack_ch9329_data(CMD_SEND_KB_GENERAL_DATA, param->input.data,
                             param->input.length, s_packed_buf);
    if (n > 0) {
      send_data_to_uart(s_packed_buf, n);
    } else {
      ESP_LOGW(TAG, "pack_ch9329_data rejected length %d", param->input.length);
    }
#else
    // For ch9328, pass the keycode data directly
    send_data_to_uart(param->input.data, param->input.length);
#endif
    break;
  }
  case ESP_HIDH_FEATURE_EVENT: {
    const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
    ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d",
             ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->feature.usage),
             param->feature.map_index, param->feature.report_id,
             param->feature.length);
    ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
    break;
  }
  case ESP_HIDH_CLOSE_EVENT: {
    const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
    ESP_LOGI(TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda),
             esp_hidh_dev_name_get(param->close.dev));
    ble_status.status = BLE_STATUS_SCAN;
    if (runtime_mode_get() != RUNTIME_MODE_ADMIN) {
      ESP_LOGI(TAG, "BLE link dropped, bringing AP back up");
      runtime_mode_set(RUNTIME_MODE_ADMIN);
      set_led_mode(LED_MODE_ADMIN);
      esp_event_loop_create_default();
      if (wifi_ap_start() == ESP_OK) web_server_start();
    }

    break;
  }
  default:
    ESP_LOGI(TAG, "EVENT: %d", event);
    break;
  }
}

#define SCAN_DURATION_SECONDS 5

static bool connect_hid_dev(esp_hid_scan_result_t *cr) {
  esp_hidh_dev_t *dev =
      esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
  if (dev == NULL) {
    ESP_LOGE(TAG, "esp_hidh_dev_open returned NULL for " ESP_BD_ADDR_STR,
             ESP_BD_ADDR_HEX(cr->bda));
    return false;
  }
  ble_status.status = BLE_STATUS_CONNECTING;
  s_connecting_since_us = esp_timer_get_time();
  ESP_LOGI(TAG, "CONNECTING to " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(cr->bda));
  return true;
}

void hid_connect(void *pvParameters) {
  int hb = 0;
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(400));

    // Heartbeat every ~4 s so we can tell from the serial log that this task
    // is alive even when esp_hid_scan is timing out repeatedly.
    if ((++hb % 10) == 0) {
      ESP_LOGI(TAG, "hid_connect alive: ble=%d admin=%d disc=%d",
               (int)ble_status.status,
               (int)(runtime_mode_get() == RUNTIME_MODE_ADMIN),
               (int)s_discovery_enabled);
    }

    // Drain web command queue at top of loop.
    if (web_cmd_queue) {
      web_cmd_t cmd;
      while (xQueueReceive(web_cmd_queue, &cmd, 0) == pdTRUE) {
        switch (cmd.kind) {
        case WEB_CMD_START_DISCOVERY:
          s_discovery_enabled = true;
          s_disc_count = 0;
          ESP_LOGI(TAG, "DISCOVERY: ON");
          break;
        case WEB_CMD_STOP_DISCOVERY:
          s_discovery_enabled = false;
          s_disc_count = 0;
          ESP_LOGI(TAG, "DISCOVERY: OFF");
          break;
        case WEB_CMD_CONNECT: {
          // Same-device click? Treat as no-op so we don't churn the link.
          uint8_t cur_bda[6];
          if (hid_connected_bda(cur_bda) &&
              memcmp(cur_bda, cmd.bda, 6) == 0) {
            ESP_LOGI(TAG, "WEB_CMD_CONNECT: already connected to "
                     ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(cmd.bda));
            break;
          }
          // Different device while one is live: close the current link
          // before opening a new one. esp_hidh on ESP32-C3 only supports a
          // single BLE HID link; without this, esp_hidh_dev_open returns
          // NULL or the new link comes up wedged. Do NOT remove the old
          // device from device_store - the operator just wants to switch.
          if (esp_hidh_dev_exists(connected_dev)) {
            ESP_LOGI(TAG, "WEB_CMD_CONNECT: closing previous "
                     ESP_BD_ADDR_STR " before opening "
                     ESP_BD_ADDR_STR,
                     ESP_BD_ADDR_HEX(esp_hidh_dev_bda_get(connected_dev)),
                     ESP_BD_ADDR_HEX(cmd.bda));
            esp_hidh_dev_close(connected_dev);
            ble_status.status = BLE_STATUS_SCAN;
            // Give Bluedroid a moment to tear the link down before we try
            // to open the next one. Empirically 500 ms is enough on C3.
            vTaskDelay(pdMS_TO_TICKS(500));
          }
          esp_hid_scan_result_t fake = {0};
          memcpy(fake.bda, cmd.bda, 6);
          fake.transport = ESP_HID_TRANSPORT_BLE;
          fake.ble.addr_type = cmd.addr_type;
          connect_hid_dev(&fake);
          break;
        }
        case WEB_CMD_SHUTDOWN_AP:
          // Second-time wifi_ap_start (after a stop + later triple-tap) is
          // unreliable - the Wi-Fi/Bluedroid coex state machine doesn't
          // round-trip cleanly on the C3 in IDF 5.3.1. Avoid that path
          // entirely: just reboot. The boot-into-ADMIN handler in app_main
          // brings the AP back up cleanly on the next start.
          ESP_LOGI(TAG, "WEB_CMD_SHUTDOWN_AP: rebooting (clean re-bringup)");
          vTaskDelay(pdMS_TO_TICKS(200));  // let the HTTP 204 flush
          esp_restart();
          break;
        }
      }
    }

#ifdef DEBUG_HEAP
    static int64_t s_last_heap_log_us = 0;
    int64_t heap_now = esp_timer_get_time();
    if ((heap_now - s_last_heap_log_us) > (30LL * 1000 * 1000)) {
      ESP_LOGI(TAG, "free heap: %u", (unsigned)esp_get_free_heap_size());
      s_last_heap_log_us = heap_now;
    }
#endif

    bool in_admin = (runtime_mode_get() == RUNTIME_MODE_ADMIN);

    if (ble_status.status == BLE_STATUS_CONNECTED) {
      // While connected, keep relaying keystrokes. EXCEPT: if the user
      // has explicitly enabled discovery (in ADMIN), we need to scan to
      // collect candidates for the picker — that's the whole point of
      // discovery. Allow it through.
      if (!(in_admin && s_discovery_enabled)) {
        continue;
      }
    }

    if (ble_status.status == BLE_STATUS_CONNECTING) {
      if ((esp_timer_get_time() - s_connecting_since_us) > CONNECT_TIMEOUT_US) {
        ESP_LOGW(TAG, "CONNECTING timed out after 10s — back to SCAN");
        ble_status.status = BLE_STATUS_SCAN;
      }
      continue;
    }

    // Scan when (a) the dashboard has discovery toggled on (picker) OR
    // (b) we have a saved keyboard that isn't currently connected (so we
    // can auto-reconnect when it wakes from sleep). Otherwise stay idle.
    device_entry_t saved_pre[DEVICE_STORE_MAX];
    int saved_pre_count = device_store_list(saved_pre);
    bool want_discovery = in_admin && s_discovery_enabled;
    bool want_reconnect = (ble_status.status == BLE_STATUS_SCAN) &&
                          (saved_pre_count > 0);
    if (!want_discovery && !want_reconnect) {
      continue;
    }

    size_t results_len = 0;
    esp_hid_scan_result_t *results = NULL;
    ESP_LOGI(TAG, "SCAN (admin=%d, discovery=%d, ble=%d)",
             in_admin, (int)s_discovery_enabled, (int)ble_status.status);

    device_entry_t saved[DEVICE_STORE_MAX];
    int saved_count = device_store_list(saved);

    // Skip LED writes when in ADMIN — the admin transition owns the LED pattern.
    if (runtime_mode_get() != RUNTIME_MODE_ADMIN) {
      if (saved_count > 0) {
        set_led_mode(BLE_SCAN_SAVED_LED_MODE);
      } else {
        set_led_mode(BLE_SCAN_NEW_LED_MODE);
      }
    }
    // Scan without an address filter — we filter results against saved[] below.
    esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results, NULL);

    ESP_LOGI(TAG, "SCAN: %u results", results_len);
    if (results_len) {
      int64_t now = esp_timer_get_time();
      esp_hid_scan_result_t *r = results;
      while (r) {
        print_esp_hid_scan_results(r);

        bool is_saved = device_store_contains(r->bda);

        if (!is_saved) {
          // Surface to the discovery ring while ADMIN+discovery is on.
          if (s_discovery_enabled &&
              runtime_mode_get() == RUNTIME_MODE_ADMIN) {
            bool found = false;
            for (int i = 0; i < s_disc_count; ++i) {
              if (memcmp(s_disc_ring[i].bda, r->bda, 6) == 0) {
                s_disc_ring[i].rssi       = r->rssi;
                s_disc_ring[i].seen_at_us = esp_timer_get_time();
                found = true;
                break;
              }
            }
            if (!found && s_disc_count < DISC_RING_MAX) {
              // Fully populate the slot BEFORE publishing the new count, so a
              // concurrent hid_discovery_snapshot() reader on an httpd task
              // never observes a half-written entry (would surface a phantom
              // 00:00:00:00:00:00 row in the picker).
              int idx = s_disc_count;
              disc_entry_t *e = &s_disc_ring[idx];
              memcpy(e->bda, r->bda, 6);
              e->addr_type  = r->ble.addr_type;
              e->rssi       = r->rssi;
              size_t nl     = strnlen(r->name, sizeof(e->name) - 1);
              memcpy(e->name, r->name, nl);
              e->name[nl]   = 0;
              e->seen_at_us = esp_timer_get_time();
              __atomic_store_n(&s_disc_count, idx + 1, __ATOMIC_RELEASE);
              ESP_LOGI(TAG, "DISCOVERY+ " ESP_BD_ADDR_STR " '%s' rssi=%d",
                       ESP_BD_ADDR_HEX(r->bda), e->name, r->rssi);
            }
          }
          // Do not auto-connect to unsaved devices. ADMIN+discovery surfaces
          // them for the user to pick; RELAY mode (or discovery off) ignores them.
          r = r->next;
          continue;
        }

        bool backoff_active =
            (memcmp(r->bda, s_failed_bda, sizeof(esp_bd_addr_t)) == 0) &&
            ((now - s_failed_at_us) < FAILED_DEVICE_BACKOFF_US);

        if (backoff_active) {
          ESP_LOGI(TAG, "Skipping " ESP_BD_ADDR_STR " (backoff)", ESP_BD_ADDR_HEX(r->bda));
          r = r->next;
          continue;
        }

        // Auto-reconnect to SAVED keyboards only. Unsaved devices reach this
        // loop too (via the relaxed scan filter) but are intercepted earlier
        // - they only ever flow into the discovery ring for the picker, never
        // into connect_hid_dev. So when we get here, r->bda is in the store
        // and was added by an explicit Connect click in the past.
        if (connect_hid_dev(r)) {
          break;  // wait for OPEN_EVENT to decide success/failure
        }
        r = r->next;
      }
      esp_hid_scan_results_free(results);
    }
  }
}

static esp_err_t init() {
  runtime_mode_init();
  esp_err_t ret;
  /* Init nvs flash for BLE status storing */
  init_nvs_flash();

  /* Init Uart for storage flash ***/
  init_uart();

  /* Init LEDC */
  ledc_init();

  /* Init esp hid gap */
  ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
  ESP_ERROR_CHECK(esp_hid_gap_init(HID_HOST_MODE));
  ESP_ERROR_CHECK(storage_complete_migration());
  ble_status.status = BLE_STATUS_SCAN;

  ESP_ERROR_CHECK(
      esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler));

  esp_hidh_config_t config = {
      .callback = hidh_callback,
      .event_stack_size = 4096,
      .callback_arg = NULL,
  };

  ret = esp_hidh_init(&config);
  return ret;
}

static bool disconnect_device() {
  ESP_LOGI(TAG, "Remove device!");
  // Forget the currently-connected device (if any) from the store.
  bool ret = true;
  if (esp_hidh_dev_exists(connected_dev)) {
    const uint8_t *bda = esp_hidh_dev_bda_get(connected_dev);
    ret = device_store_remove(bda);
  }
  ESP_LOGI(TAG, "Set ble status to scan");
  ble_status.status = BLE_STATUS_SCAN;
  // set blink status
  ret &= set_led_mode(LED_MODE_FAST_BLINK);
  ESP_LOGI(TAG, "Set LED mode to fast blink");

  // close bluetooth connection
  if (esp_hidh_dev_exists(connected_dev)) {
    // remove device
    esp_err_t err = esp_hidh_dev_close(connected_dev);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Close device failed");
      return false;
    }
  }

  return ret;
}

static void admin_idle_task(void *_) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    if (runtime_mode_get() != RUNTIME_MODE_ADMIN) continue;
    int64_t idle = esp_timer_get_time() - web_server_last_activity_us();
    if (idle > ADMIN_IDLE_TIMEOUT_US) {
      ESP_LOGI(TAG, "ADMIN idle 10 min - back to RELAY");
      web_cmd_t c = { .kind = WEB_CMD_SHUTDOWN_AP };
      if (web_cmd_queue) xQueueSend(web_cmd_queue, &c, 0);
    }
  }
}

void app_main(void) {
  ESP_ERROR_CHECK(init());
  // Print the Wi-Fi admin AP credentials right after init so they're easy to
  // find in the serial log without scrolling through Bluedroid noise.
  wifi_ap_print_banner();
  web_cmd_queue = xQueueCreate(4, sizeof(web_cmd_t));
  configASSERT(web_cmd_queue);
  // If any of these task creations fail (OOM), the device is in a totally
  // unusable state — assert loudly rather than continue with a half-booted
  // BLE/HID/admin task topology.
  configASSERT(xTaskCreate(&change_led, "change_led", 2048, NULL, 2, NULL) == pdPASS);
  configASSERT(xTaskCreate(&hid_connect, "hid_connect", 8 * 1024, NULL, 2, NULL) == pdPASS);
  configASSERT(xTaskCreate(&admin_idle_task, "admin_idle", 2048, NULL, 1, NULL) == pdPASS);

  gesture_ctx_t gctx;
  gesture_init(&gctx);

  // Configure IO9 as input with internal pull-up (boot button).
  gpio_config_t io = {
      .pin_bit_mask = 1ULL << BOOT_MODE_PIN,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&io);

  // Boot directly into ADMIN: every fresh boot is "no connection" by
  // definition (auto-open is disabled), so the operator needs the AP up
  // to be able to pick a keyboard. Mirrors the GESTURE_TRIPLE_TAP handler.
  ESP_LOGI(TAG, "Boot: entering ADMIN mode (no connection yet)");
  runtime_mode_set(RUNTIME_MODE_ADMIN);
  set_led_mode(LED_MODE_ADMIN);
  esp_event_loop_create_default();
  if (wifi_ap_start() == ESP_OK) {
    web_server_start();
  }

  while (true) {
    int level = gpio_get_level(BOOT_MODE_PIN);
    gesture_t g = gesture_step(&gctx, level, esp_timer_get_time());
    switch (g) {
    case GESTURE_HOLD_1S:
      ESP_LOGI(TAG, "Gesture: HOLD_1S - forget active device");
      disconnect_device();
      break;
    case GESTURE_HOLD_5S:
      ESP_LOGW(TAG, "Gesture: HOLD_5S - FACTORY RESET");
      storage_factory_reset();
      esp_restart();
      break;
    case GESTURE_TRIPLE_TAP:
      ESP_LOGI(TAG, "Gesture: TRIPLE_TAP - entering ADMIN mode");
      runtime_mode_set(RUNTIME_MODE_ADMIN);
      set_led_mode(LED_MODE_ADMIN);
      // Wi-Fi driver requires the default event loop. Idempotent on subsequent calls.
      esp_event_loop_create_default();
      if (wifi_ap_start() == ESP_OK) {
        web_server_start();
      }
      break;
    default:
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
