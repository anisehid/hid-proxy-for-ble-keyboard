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
  int n = s_disc_count < max ? s_disc_count : max;
  if (n > 0 && out) memcpy(out, s_disc_ring, sizeof(disc_entry_t) * n);
  return n;
}

bool hid_discovery_is_enabled(void) { return s_discovery_enabled; }

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
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(400));

    // Drain web command queue at top of loop.
    if (web_cmd_queue) {
      web_cmd_t cmd;
      while (xQueueReceive(web_cmd_queue, &cmd, 0) == pdTRUE) {
        switch (cmd.kind) {
        case WEB_CMD_START_DISCOVERY:
          s_discovery_enabled = true;
          s_disc_count = 0;
          break;
        case WEB_CMD_STOP_DISCOVERY:
          s_discovery_enabled = false;
          s_disc_count = 0;
          break;
        case WEB_CMD_CONNECT: {
          esp_hid_scan_result_t fake = {0};
          memcpy(fake.bda, cmd.bda, 6);
          fake.transport = ESP_HID_TRANSPORT_BLE;
          fake.ble.addr_type = cmd.addr_type;
          connect_hid_dev(&fake);
          break;
        }
        case WEB_CMD_SHUTDOWN_AP:
          runtime_mode_set(RUNTIME_MODE_RELAY);
          set_led_mode(LED_MODE_OFF);
          web_server_stop();
          wifi_ap_stop();
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

    if (ble_status.status == BLE_STATUS_CONNECTED) {
      continue;
    }

    if (ble_status.status == BLE_STATUS_CONNECTING) {
      if ((esp_timer_get_time() - s_connecting_since_us) > CONNECT_TIMEOUT_US) {
        ESP_LOGW(TAG, "CONNECTING timed out after 10s — back to SCAN");
        ble_status.status = BLE_STATUS_SCAN;
      }
      continue;
    }

    size_t results_len = 0;
    esp_hid_scan_result_t *results = NULL;
    ESP_LOGI(TAG, "SCAN...");

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
              disc_entry_t *e = &s_disc_ring[s_disc_count++];
              memcpy(e->bda, r->bda, 6);
              e->addr_type  = r->ble.addr_type;
              e->rssi       = r->rssi;
              size_t nl     = strnlen(r->name, sizeof(e->name) - 1);
              memcpy(e->name, r->name, nl);
              e->name[nl]   = 0;
              e->seen_at_us = esp_timer_get_time();
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

        if (connect_hid_dev(r)) {
          // Dispatched; wait for OPEN_EVENT to decide success/failure.
          break;
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
  web_cmd_queue = xQueueCreate(4, sizeof(web_cmd_t));
  xTaskCreate(&change_led, "change_led", 2048, NULL, 2, NULL);
  xTaskCreate(&hid_connect, "hid_connect", 8 * 1024, NULL, 2, NULL);
  xTaskCreate(&admin_idle_task, "admin_idle", 2048, NULL, 1, NULL);

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
