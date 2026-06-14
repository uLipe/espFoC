/*
 * Encoder read loop — logs shaft angle over console UART.
 */

#include "esp_log.h"

#include "espFoC/esp_foc_encoder_as5600.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/osal/os_interface.h"
#if defined(CONFIG_AXIS_TUNING_ENCODER_AS5048)
#include "espFoC/esp_foc_encoder_as5048.h"
#include "driver/spi_common.h"
#endif

static const char *TAG = "test_encoder";

static esp_foc_encoder_t *encoder_new(void)
{
#if defined(CONFIG_AXIS_TUNING_ENCODER_AS5600)
    return esp_foc_encoder_as5600_new(
        CONFIG_AXIS_TUNING_ENC_SDA,
        CONFIG_AXIS_TUNING_ENC_SCL,
        0);
#elif defined(CONFIG_AXIS_TUNING_ENCODER_AS5048)
    return esp_foc_encoder_as5048_new(
        CONFIG_AXIS_TUNING_ENC_MOSI,
        CONFIG_AXIS_TUNING_ENC_MISO,
        CONFIG_AXIS_TUNING_ENC_SCLK,
        CONFIG_AXIS_TUNING_ENC_CS,
        SPI2_HOST,
        0);
#else
    return NULL;
#endif
}

void app_main(void)
{
    esp_foc_encoder_t *enc = encoder_new();
    if (enc == NULL) {
        ESP_LOGE(TAG, "encoder init failed");
        return;
    }

    ESP_LOGI(TAG, "encoder test — rotate shaft (CPR=%lu)",
             (unsigned long)enc->get_counts_per_revolution(enc));

    while (1) {
        q16_t ticks = enc->read_counts(enc);
        ESP_LOGI(TAG, "counts=%lld", (long long)q16_to_int(ticks));
        esp_foc_sleep_ms(100);
    }
}
