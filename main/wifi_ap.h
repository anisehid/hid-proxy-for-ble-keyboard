#ifndef WIFI_AP_H_
#define WIFI_AP_H_

#include "esp_err.h"

// Brings up Wi-Fi as soft-AP. SSID = "hidproxy-XXXX" (last 4 MAC hex digits).
// WPA2 password = last 8 MAC hex digits (lowercase). Logs both to serial.
esp_err_t wifi_ap_start(void);

// Tears down soft-AP and stops Wi-Fi.
esp_err_t wifi_ap_stop(void);

#endif
