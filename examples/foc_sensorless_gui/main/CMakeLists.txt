# hal/uart_ll.h lives in esp_hal_uart from IDF 6, in hal before.
if(IDF_VERSION_MAJOR GREATER_EQUAL 6)
    set(uart_ll_component esp_hal_uart)
else()
    set(uart_ll_component hal)
endif()

idf_component_register(SRCS "main.c" "commissioning_store.c"
                       PRIV_REQUIRES espFoC ${uart_ll_component} nvs_flash esp_rom)
