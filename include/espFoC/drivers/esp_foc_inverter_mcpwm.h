/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#pragma once

#include "espFoC/drivers/esp_foc_inverter.h"
#include "hal/adc_types.h"

/**
 * MCPWM 6-PWM inverter with on-chip ADC current sense (U/V shunts).
 * ETM PWM-synchronized sampling is wired internally when supported.
 */
esp_foc_inverter_t *esp_foc_inverter_mcpwm_6pwm_new(int gpio_u_high, int gpio_u_low,
                                                    int gpio_v_high, int gpio_v_low,
                                                    int gpio_w_high, int gpio_w_low,
                                                    int gpio_enable,
                                                    float dc_link_voltage,
                                                    int port,
                                                    adc_channel_t adc_ch_u,
                                                    adc_channel_t adc_ch_v,
                                                    float amp_gain,
                                                    float shunt_resistance);

/**
 * MCPWM 3-PWM inverter with on-chip ADC current sense (U/V shunts).
 */
esp_foc_inverter_t *esp_foc_inverter_mcpwm_3pwm_new(int gpio_u, int gpio_v, int gpio_w,
                                                    int gpio_enable,
                                                    float dc_link_voltage,
                                                    int port,
                                                    adc_channel_t adc_ch_u,
                                                    adc_channel_t adc_ch_v,
                                                    float amp_gain,
                                                    float shunt_resistance);
