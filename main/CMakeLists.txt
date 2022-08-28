set(srcs "esp_hid_host_main.c"
         "esp_hid_gap.c"
         "ch932x.c"
         "led.c"
         "storage.c")
set(include_dirs ".")
idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid driver
                       PRIV_REQUIRES nvs_flash)
