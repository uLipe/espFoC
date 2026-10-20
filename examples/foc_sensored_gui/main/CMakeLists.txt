idf_component_register(SRCS "main.c" "commissioning_store.c"
                       PRIV_REQUIRES espFoC esp_driver_uart nvs_flash esp_rom)
