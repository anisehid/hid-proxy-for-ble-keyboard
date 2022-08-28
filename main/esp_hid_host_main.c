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
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "led.h"
#include "nvs_flash.h"

#include "esp_hid_gap.h"
#include "esp_hidh.h"
#include "storage.h"

#define BLE_STATUS_CONNECTED 1
#define BLE_STATUS_SCAN 2

#define BLE_CONNECTED_LED_MODE LED_MODE_OFF
#define BLE_SCAN_NEW_LED_MODE LED_MODE_FAST_BLINK
#define BLE_SCAN_SAVED_LED_MODE LED_MODE_SLOW_BLINK
#define BLE_UNSET_LED_MODE LED_MODE_ALWAYS_ON

static const char *TAG = "ESP_HIDH_DEMO";
static esp_hidh_dev_t *connected_dev = NULL;
static EspBleStatus ble_status;

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
      ble_status.status = BLE_STATUS_CONNECTED;
      connected_dev = param->open.dev;
    } else {
      ESP_LOGE(TAG, " OPEN failed!");
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
    // For ch9329, pack the keycode data
    pack_ch9329_data(CMD_SEND_KB_GENERAL_DATA, param->input.data,
                     param->input.length);
    send_data_to_uart(g_packed_data, g_packed_data_len + FIXED_CH9329_DATA_LEN);
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
  if (!save_ble_device(cr->bda)) {
    ESP_LOGE(TAG, "SAVE BLE DRIVER FAILED");
    return false;
  }

  set_led_mode(BLE_CONNECTED_LED_MODE);
  esp_hidh_dev_t *dev =
      esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);

  if (dev != NULL) {
    ESP_LOGI(TAG, "CONNECT SUCCESS!");
    return true;
  }
  return false;
}

void hid_connect(void *pvParameters) {
  while (true) {
    vTaskDelay(400);
    if (ble_status.status == BLE_STATUS_CONNECTED) {
      continue;
    }
    size_t results_len = 0;
    esp_hid_scan_result_t *results = NULL;
    ESP_LOGI(TAG, "SCAN...");

    esp_bd_addr_t addr;
    memset(addr, 0, sizeof(esp_bd_addr_t));
    bool has_con_dev = read_ble_device(addr);
    if (has_con_dev) {
      ESP_LOGI(TAG, "FOUND SAVED BLE DEVICE: " ESP_BD_ADDR_STR ", ",
               ESP_BD_ADDR_HEX(addr));
      set_led_mode(BLE_SCAN_SAVED_LED_MODE);
      esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results, addr);
    } else {
      ESP_LOGI(TAG, "TRY CONNECT TO NEW BLE DEVICE: " ESP_BD_ADDR_STR ", ",
               ESP_BD_ADDR_HEX(addr));
      set_led_mode(BLE_SCAN_NEW_LED_MODE);
      esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results, NULL);
    }

    /// print scanned results
    ESP_LOGI(TAG, "SCAN: %u results", results_len);
    if (results_len) {
      esp_hid_scan_result_t *r = results;
      while (r) {
        print_esp_hid_scan_results(r);
        /* try open hid_dev */
        bool con_suc = connect_hid_dev(r);
        if (con_suc) {
          ble_status.status = BLE_STATUS_CONNECTED;
          ESP_LOGI(TAG, "Connect to " ESP_BD_ADDR_STR ": ",
                   ESP_BD_ADDR_HEX(r->bda));
          break;
        }
        r = r->next;
      }
      // free the results
      esp_hid_scan_results_free(results);
    }
  }
}

static esp_err_t init() {
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
  // clear stored status
  bool ret = clear_ble_devices();
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

void app_main(void) {
  ESP_ERROR_CHECK(init());
  xTaskCreate(&change_led, "change_led", 2048, NULL, 2, NULL);
  xTaskCreate(&hid_connect, "hid_connect", 6 * 1024, NULL, 2, NULL);
  while (true) {
    // check for reset button, if pressed, reset scan
    if (gpio_get_level(BOOT_MODE_PIN) == 0) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      if (gpio_get_level(BOOT_MODE_PIN) == 0) {
        ESP_LOGI(TAG, "BUTTON IS DOWN");
        if (disconnect_device()) {
          ESP_LOGI(TAG, "CLEAR BLE STATUS SUCCESS");
        }
      }
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}
