#include "wifi_ap.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include <string.h>

static const char *TAG = "WIFI_AP";
static bool g_started = false;
static esp_netif_t *g_netif = NULL;

esp_err_t wifi_ap_start(void) {
    if (g_started) return ESP_OK;

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    char ssid[16];
    snprintf(ssid, sizeof ssid, "hidproxy-%02x%02x", mac[4], mac[5]);
    char pass[16];
    snprintf(pass, sizeof pass, "%02x%02x%02x%02x", mac[2], mac[3], mac[4], mac[5]);

    ESP_ERROR_CHECK(esp_netif_init());
    if (g_netif == NULL) g_netif = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

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

    ESP_LOGI(TAG, "AP up: SSID='%s' PSK='%s' (open http://192.168.4.1)",
             ssid, pass);
    g_started = true;
    return ESP_OK;
}

esp_err_t wifi_ap_stop(void) {
    if (!g_started) return ESP_OK;
    esp_wifi_stop();
    esp_wifi_deinit();
    ESP_LOGI(TAG, "AP down");
    g_started = false;
    return ESP_OK;
}
