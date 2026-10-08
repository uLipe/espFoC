/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include "soc/soc_caps.h"

#if !SOC_ETM_SUPPORTED || !SOC_MCPWM_SUPPORTED || !SOC_ADC_DIG_CTRL_SUPPORTED \
    || !SOC_ADC_DMA_SUPPORTED || !SOC_MCPWM_SUPPORT_ETM
#error "espFoC inverter requires MCPWM + ETM + ADC digi/DMA (SOC_CAPS)"
#endif
