/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

/*
 * HAL/LL names that differ between IDF 5.5 and IDF 6.x. Drivers use the
 * ESP_FOC_* spelling so the same source builds against both. Each macro
 * expands to symbols from the LL/periph header the caller already includes.
 */

#include "esp_idf_version.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)

#define ESP_FOC_ETM_CHANNELS                ETM_LL_CHANS_PER_INST

#define ESP_FOC_GPIO_MATRIX_OUT_DEFAULT(hw, gpio) \
    gpio_ll_set_output_signal_matrix_source((hw), (gpio), SIG_GPIO_OUT_IDX, false)
#define ESP_FOC_GPIO_ETM_HW                 GPIO_LL_ETM_GET_HW()

#define ESP_FOC_MCPWM_SIGNALS(group)        (soc_mcpwm_signals[(group)])

#define ESP_FOC_TIMG_GROUPS                 TIMG_LL_INST_NUM
#define ESP_FOC_TIMG_RCC_MODULE(group)      (soc_timg_gptimer_signals[(group)][0].parent_module)
#define ESP_FOC_TIMG_ENABLE_BUS_CLOCK(group, en) timg_ll_enable_bus_clock((group), (en))

#define ESP_FOC_GDMA_RX_CONNECT_ADC(dev, ch, periph_id) \
    gdma_ll_rx_connect_to_periph((dev), (ch), (periph_id))
#define ESP_FOC_GDMA_RX_DISCONNECT(dev, ch) gdma_ll_rx_disconnect_all((dev), (ch))

#define ESP_FOC_PCNT_GROUPS                 PCNT_LL_INST_NUM
#define ESP_FOC_PCNT_UNITS                  PCNT_LL_UNITS_PER_INST
#define ESP_FOC_PCNT_IRQ(group)             (soc_pcnt_signals[(group)].irq_id)
#define ESP_FOC_PCNT_PULSE_SIG(group, unit, ch) \
    (soc_pcnt_signals[(group)].units[(unit)].channels[(ch)].pulse_sig_id_matrix)
#define ESP_FOC_PCNT_CTRL_SIG(group, unit, ch) \
    (soc_pcnt_signals[(group)].units[(unit)].channels[(ch)].ctl_sig_id_matrix)

#else

#define ESP_FOC_ETM_CHANNELS                SOC_ETM_CHANNELS_PER_GROUP

#define ESP_FOC_GPIO_MATRIX_OUT_DEFAULT(hw, gpio) gpio_ll_matrix_out_default((hw), (gpio))
#define ESP_FOC_GPIO_ETM_HW                 (&GPIO_ETM)

#define ESP_FOC_MCPWM_SIGNALS(group)        (mcpwm_periph_signals.groups[(group)])

#define ESP_FOC_TIMG_GROUPS                 SOC_TIMER_GROUPS
#define ESP_FOC_TIMG_RCC_MODULE(group)      (timer_group_periph_signals.groups[(group)].module)
#define ESP_FOC_TIMG_ENABLE_BUS_CLOCK(group, en) timer_ll_enable_bus_clock((group), (en))

#define ESP_FOC_GDMA_RX_CONNECT_ADC(dev, ch, periph_id) \
    gdma_ll_rx_connect_to_periph((dev), (ch), GDMA_TRIG_PERIPH_ADC, (periph_id))
#define ESP_FOC_GDMA_RX_DISCONNECT(dev, ch) gdma_ll_rx_disconnect_from_periph((dev), (ch))

#define ESP_FOC_PCNT_GROUPS                 SOC_PCNT_GROUPS
#define ESP_FOC_PCNT_UNITS                  SOC_PCNT_UNITS_PER_GROUP
#define ESP_FOC_PCNT_IRQ(group)             (pcnt_periph_signals.groups[(group)].irq)
#define ESP_FOC_PCNT_PULSE_SIG(group, unit, ch) \
    (pcnt_periph_signals.groups[(group)].units[(unit)].channels[(ch)].pulse_sig)
#define ESP_FOC_PCNT_CTRL_SIG(group, unit, ch) \
    (pcnt_periph_signals.groups[(group)].units[(unit)].channels[(ch)].control_sig)

#endif
