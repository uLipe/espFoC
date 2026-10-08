/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Hardware PID hook. No SoC in the current support set exposes a PID
 * accelerator — always report unavailable.
 */
#include <stdbool.h>

#include "espFoC/utils/esp_foc_pid.h"
#include "sdkconfig.h"

#if CONFIG_ESP_FOC_USE_HW_ACCEL_PID

bool esp_foc_pid_hw_available(void)
{
    return false;
}

q16_t esp_foc_pid_hw_update(esp_foc_pid_t *p, q16_t sp, q16_t meas)
{
    (void)p;
    (void)sp;
    (void)meas;
    return 0;
}

#endif
