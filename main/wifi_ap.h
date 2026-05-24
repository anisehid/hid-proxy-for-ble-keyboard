#ifndef WIFI_AP_H_
#define WIFI_AP_H_

#include "esp_err.h"
#include <stddef.h>

// Brings up Wi-Fi as soft-AP. SSID = "hidproxy-XXXX" (last 4 MAC hex digits).
// WPA2 password = last 8 MAC hex digits (lowercase). Logs both to serial.
esp_err_t wifi_ap_start(void);

// Tears down soft-AP and stops Wi-Fi.
esp_err_t wifi_ap_stop(void);

// Fill caller buffers with the deterministic SSID and PSK derived from the
// SoftAP MAC. ssid >= 16 bytes, pass >= 16 bytes.
void wifi_ap_get_credentials(char *ssid, size_t ssid_len,
                             char *pass, size_t pass_len);

// Print a loud banner with the SSID + PSK + URL via printf (bypasses ESP_LOG
// level filtering and tag formatting so it survives any log config). Safe to
// call at boot, before Wi-Fi is initialized.
void wifi_ap_print_banner(void);

#endif
