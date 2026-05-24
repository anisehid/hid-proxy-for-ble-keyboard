set(srcs "esp_hid_host_main.c"
         "esp_hid_gap.c"
         "ch932x.c"
         "ch9329_pack.c"
         "led.c"
         "led_duty.c"
         "storage.c"
         "device_store.c"
         "button_gesture.c")
set(include_dirs ".")
idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid driver
                       PRIV_REQUIRES nvs_flash)
