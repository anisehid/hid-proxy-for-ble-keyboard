#include "wifi_ap.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "WIFI_AP";
static bool g_started = false;
static bool g_inited  = false;
static esp_netif_t *g_netif = NULL;

void wifi_ap_get_credentials(char *ssid, size_t ssid_len,
                             char *pass, size_t pass_len) {
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(ssid, ssid_len, "hidproxy-%02x%02x", mac[4], mac[5]);
    snprintf(pass, pass_len, "%02x%02x%02x%02x", mac[2], mac[3], mac[4], mac[5]);
}

void wifi_ap_print_banner(void) {
    char ssid[16];
    char pass[16];
    wifi_ap_get_credentials(ssid, sizeof ssid, pass, sizeof pass);
    // Use printf so the banner survives any ESP_LOG level filter and stands
    // out from tagged log lines.
    printf("\n");
    printf("================ HID PROXY — Wi-Fi admin AP ================\n");
    printf("  Triple-tap the boot button (IO9) to start the admin AP.\n");
    printf("  SSID: %s\n", ssid);
    printf("  PSK : %s\n", pass);
    printf("  URL : http://192.168.4.1\n");
    printf("============================================================\n");
    printf("\n");
}

esp_err_t wifi_ap_start(void) {
    if (g_started) return ESP_OK;

    char ssid[16];
    char pass[16];
    wifi_ap_get_credentials(ssid, sizeof ssid, pass, sizeof pass);

    // esp_netif_init() is not idempotent in 5.3.1 — calling it a second time
    // returns ESP_ERR_INVALID_STATE which ESP_ERROR_CHECK turns into an abort.
    // Same story for esp_netif_create_default_wifi_ap() and esp_wifi_init().
    // Do them exactly once across the lifetime of the firmware.
    if (!g_inited) {
        ESP_ERROR_CHECK(esp_netif_init());
        g_netif = esp_netif_create_default_wifi_ap();
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        g_inited = true;
    }

    wifi_config_t wcfg = {0};
    wcfg.ap.channel = 6;
    wcfg.ap.max_connection = 3;
    wcfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wcfg.ap.pmf_cfg.required = false;
    strncpy((char *)wcfg.ap.ssid, ssid, sizeof wcfg.ap.ssid);
    wcfg.ap.ssid_len = strlen(ssid);
    strncpy((char *)wcfg.ap.password, pass, sizeof wcfg.ap.password);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Loud banner on AP start so users can find the credentials in the log.
    printf("\n");
    printf(">>>>>>>>>>>>>>>>  ADMIN AP IS UP  <<<<<<<<<<<<<<<<\n");
    printf("    SSID: %s\n", ssid);
    printf("    PSK : %s\n", pass);
    printf("    URL : http://192.168.4.1\n");
    printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
    printf("\n");
    ESP_LOGI(TAG, "AP up: SSID='%s' PSK='%s'", ssid, pass);
    g_started = true;
    return ESP_OK;
}

esp_err_t wifi_ap_stop(void) {
    if (!g_started) return ESP_OK;
    // Leave wifi/netif inited so a subsequent wifi_ap_start() can just call
    // esp_wifi_start() again without re-running the one-shot init paths.
    esp_err_t err = esp_wifi_stop();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_stop failed: %d - leaving g_started set", err);
        return err;
    }
    ESP_LOGI(TAG, "AP down");
    g_started = false;
    return ESP_OK;
}
