/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include "espFoC/esp_foc.h"

/**
 * @brief 6-PWM inverter driver using MCPWM (ESP-IDF MCPWM prelude API).
 *
 * Outputs complementary PWM signals (high/low) for each phase.
 *
 * IMPORTANT: This driver does NOT configure dead-time. Ensure your gate driver
 * provides dead-time or your power stage is otherwise protected against
 * shoot-through.
 *
 * @param gpio_u_high  GPIO for phase U high-side PWM
 * @param gpio_u_low   GPIO for phase U low-side PWM
 * @param gpio_v_high  GPIO for phase V high-side PWM
 * @param gpio_v_low   GPIO for phase V low-side PWM
 * @param gpio_w_high  GPIO for phase W high-side PWM
 * @param gpio_w_low   GPIO for phase W low-side PWM
 * @param gpio_enable  Optional enable GPIO (set HIGH on enable, LOW on disable). Pass -1 to ignore.
 * @param dc_link_voltage DC bus voltage used for duty scaling
 * @param port         Axis/port index (0..CONFIG_NOOF_AXIS-1). Also selects MCPWM group id.
 *
 * @return Pointer to esp_foc_inverter_t interface, or NULL on error.
 */
esp_foc_inverter_t *inverter_6pwm_mpcwm_new(int gpio_u_high, int gpio_u_low,
                                           int gpio_v_high, int gpio_v_low,
                                           int gpio_w_high, int gpio_w_low,
                                           int gpio_enable,
                                           float dc_link_voltage,
                                           int port);
