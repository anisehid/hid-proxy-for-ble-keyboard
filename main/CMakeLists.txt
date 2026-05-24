set(srcs "esp_hid_host_main.c"
         "esp_hid_gap.c"
         "ch932x.c"
         "ch9329_pack.c"
         "led.c"
         "led_duty.c"
         "storage.c"
         "device_store.c"
         "button_gesture.c"
         "runtime_mode.c"
         "wifi_ap.c"
         "sha256.c"
         "web_auth.c"
         "web_server.c")
set(include_dirs ".")
idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid driver esp_wifi esp_netif esp_event esp_http_server
                       PRIV_REQUIRES nvs_flash mbedtls)

target_add_binary_data(${COMPONENT_LIB} "web_assets/index.html" TEXT)
target_add_binary_data(${COMPONENT_LIB} "web_assets/login.html" TEXT)
target_add_binary_data(${COMPONENT_LIB} "web_assets/app.css"    TEXT)
target_add_binary_data(${COMPONENT_LIB} "web_assets/app.js"     TEXT)
