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

#include "espFoC/inverter_6pwm_mcpwm.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

/*
 * Notes:
 * - Mirrors inverter_3pwm_mcpwm.c but creates two generators per phase.
 */

typedef struct {
    int enable_gpio;

    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t operators[3];
    mcpwm_cmpr_handle_t comparators[3];

    mcpwm_gen_handle_t gen_high[3];
    mcpwm_gen_handle_t gen_low[3];

    float voltage_to_duty_ratio;
    float dc_link_voltage;

    esp_foc_inverter_callback_t notifier;
    void *arg;

    esp_foc_inverter_t interface;
} esp_foc_mcpwm6_inverter_t;

#define MCPWM_RATE_HZ         ESP_FOC_PWM_RATE_HZ
#define MCPWM_RESOLUTION_HZ   160000000
#define MCPWM_PERIOD_TOP      (MCPWM_RESOLUTION_HZ / MCPWM_RATE_HZ)
#define MCPWM_PERIOD_TOP_HALF (MCPWM_PERIOD_TOP / 2)
#define MCPWM_DEADTIME_US     1

DRAM_ATTR static esp_foc_mcpwm6_inverter_t mcpwm6s[CONFIG_NOOF_AXIS];

static mcpwm_timer_config_t timer_config = {
    .group_id = 0,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = MCPWM_RESOLUTION_HZ,
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
    .period_ticks = MCPWM_PERIOD_TOP,
};

static mcpwm_operator_config_t operator_config = {
    .group_id = 0,
};

static mcpwm_comparator_config_t compare_config = {
    .flags.update_cmp_on_tez = true,
};

static const char *TAG = "INVERTER_MCPWM6";

static inline uint32_t deadtime_us_to_ticks(uint32_t us)
{
    /* resolution_hz is in Hz -> ticks per second */
    /* ticks = us * resolution_hz / 1_000_000 */
    return (uint32_t)(((uint64_t)us * (uint64_t)MCPWM_RESOLUTION_HZ) / 1000000ULL);
}

static IRAM_ATTR bool inverter_isr(mcpwm_timer_handle_t timer,
                                  const mcpwm_timer_event_data_t *edata,
                                  void *user_data)
{
    (void)timer;
    (void)edata;

    esp_foc_mcpwm6_inverter_t *inverter = (esp_foc_mcpwm6_inverter_t *)user_data;

    if (inverter->notifier) {
        inverter->notifier(inverter->arg);
    }
    return false;
}

static inline float clamp_pu(float x)
{
    if (x > 1.0f) return 1.0f;
    if (x < 0.0f) return 0.0f;
    return x;
}

IRAM_ATTR static float get_dc_link_voltage(esp_foc_inverter_t *self)
{
    esp_foc_mcpwm6_inverter_t *obj = __containerof(self, esp_foc_mcpwm6_inverter_t, interface);
    return obj->dc_link_voltage;
}

IRAM_ATTR static void set_voltages(esp_foc_inverter_t *self, float v_u, float v_v, float v_w)
{
    esp_foc_mcpwm6_inverter_t *obj = __containerof(self, esp_foc_mcpwm6_inverter_t, interface);

    /* Keep same clamping behavior as inverter_3pwm_mcpwm */
    v_u = clamp_pu(v_u);
    v_v = clamp_pu(v_v);
    v_w = clamp_pu(v_w);

    v_u *= obj->voltage_to_duty_ratio;
    v_v *= obj->voltage_to_duty_ratio;
    v_w *= obj->voltage_to_duty_ratio;

    mcpwm_comparator_set_compare_value(obj->comparators[0], (uint16_t)v_u);
    mcpwm_comparator_set_compare_value(obj->comparators[1], (uint16_t)v_v);
    mcpwm_comparator_set_compare_value(obj->comparators[2], (uint16_t)v_w);
}

IRAM_ATTR static void set_inverter_callback(esp_foc_inverter_t *self,
                                           esp_foc_inverter_callback_t callback,
                                           void *argument)
{
    esp_foc_mcpwm6_inverter_t *obj = __containerof(self, esp_foc_mcpwm6_inverter_t, interface);
    obj->notifier = callback;
    obj->arg = argument;
}

IRAM_ATTR static void phase_remap(esp_foc_inverter_t *self)
{
    (void)self;
}

IRAM_ATTR static float get_inverter_pwm_rate(esp_foc_inverter_t *self)
{
    (void)self;
    return (float)MCPWM_RATE_HZ;
}

IRAM_ATTR static void inverter_enable(esp_foc_inverter_t *self)
{
    esp_foc_mcpwm6_inverter_t *obj = __containerof(self, esp_foc_mcpwm6_inverter_t, interface);
    if (obj->enable_gpio >= 0) {
        gpio_set_level(obj->enable_gpio, true);
    }
}

IRAM_ATTR static void inverter_disable(esp_foc_inverter_t *self)
{
    esp_foc_mcpwm6_inverter_t *obj = __containerof(self, esp_foc_mcpwm6_inverter_t, interface);
    if (obj->enable_gpio >= 0) {
        gpio_set_level(obj->enable_gpio, false);
    }
}

esp_foc_inverter_t *inverter_6pwm_mpcwm_new(int gpio_u_high, int gpio_u_low,
                                           int gpio_v_high, int gpio_v_low,
                                           int gpio_w_high, int gpio_w_low,
                                           int gpio_enable,
                                           float dc_link_voltage,
                                           int port)
{
    if (port < 0 || port >= CONFIG_NOOF_AXIS) {
        ESP_LOGE(TAG, "Invalid port %d (CONFIG_NOOF_AXIS=%d)", port, CONFIG_NOOF_AXIS);
        return NULL;
    }

    esp_foc_mcpwm6_inverter_t *obj = &mcpwm6s[port];
    memset(obj, 0, sizeof(*obj));

    obj->enable_gpio = gpio_enable;
    obj->dc_link_voltage = dc_link_voltage;

    obj->interface.get_dc_link_voltage = get_dc_link_voltage;
    obj->interface.set_voltages = set_voltages;
    obj->interface.set_inverter_callback = set_inverter_callback;
    obj->interface.phase_remap = phase_remap;
    obj->interface.get_inverter_pwm_rate = get_inverter_pwm_rate;
    obj->interface.enable = inverter_enable;
    obj->interface.disable = inverter_disable;

    obj->voltage_to_duty_ratio = MCPWM_PERIOD_TOP_HALF;
    obj->notifier = NULL;
    obj->arg = NULL;

    if (obj->enable_gpio >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << obj->enable_gpio,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = 0,
            .pull_down_en = 0,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_set_level(obj->enable_gpio, false);
    }

    /* Create MCPWM timer/operator/comparator resources in selected group (port). */
    timer_config.group_id = port;
    mcpwm_new_timer(&timer_config, &obj->timer);

    operator_config.group_id = port;
    for (int i = 0; i < 3; i++) {
        mcpwm_new_operator(&operator_config, &obj->operators[i]);
        mcpwm_operator_connect_timer(obj->operators[i], obj->timer);
    }

    for (int i = 0; i < 3; i++) {
        mcpwm_new_comparator(obj->operators[i], &compare_config, &obj->comparators[i]);
        mcpwm_comparator_set_compare_value(obj->comparators[i], 0);
    }

    /* Create generators: two per phase (high/low). */
    mcpwm_generator_config_t gen_config = {};

    gen_config.gen_gpio_num = gpio_u_high;
    mcpwm_new_generator(obj->operators[0], &gen_config, &obj->gen_high[0]);
    gen_config.gen_gpio_num = gpio_u_low;
    mcpwm_new_generator(obj->operators[0], &gen_config, &obj->gen_low[0]);

    gen_config.gen_gpio_num = gpio_v_high;
    mcpwm_new_generator(obj->operators[1], &gen_config, &obj->gen_high[1]);
    gen_config.gen_gpio_num = gpio_v_low;
    mcpwm_new_generator(obj->operators[1], &gen_config, &obj->gen_low[1]);

    gen_config.gen_gpio_num = gpio_w_high;
    mcpwm_new_generator(obj->operators[2], &gen_config, &obj->gen_high[2]);
    gen_config.gen_gpio_num = gpio_w_low;
    mcpwm_new_generator(obj->operators[2], &gen_config, &obj->gen_low[2]);

    /* Generator actions:
     * High-side (same as inverter_3pwm_mcpwm):
     *  - on UP compare:  set LOW
     *  - on DOWN compare:set HIGH
     *
     * Low-side: complementary (inverted):
     *  - on UP compare:  set HIGH
     *  - on DOWN compare:set LOW
     */
    for (int i = 0; i < 3; i++) {
        mcpwm_generator_set_actions_on_compare_event(obj->gen_high[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, obj->comparators[i], MCPWM_GEN_ACTION_LOW),
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, obj->comparators[i], MCPWM_GEN_ACTION_HIGH),
            MCPWM_GEN_COMPARE_EVENT_ACTION_END());

        mcpwm_generator_set_actions_on_compare_event(obj->gen_low[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, obj->comparators[i], MCPWM_GEN_ACTION_HIGH),
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, obj->comparators[i], MCPWM_GEN_ACTION_LOW),
            MCPWM_GEN_COMPARE_EVENT_ACTION_END());
    }

    /* Dead-time insertion (conservative: 1us).
     * IMPORTANT: This assumes gen_high/gen_low are complementary pairs.
     */
    const uint32_t dt_ticks = deadtime_us_to_ticks(MCPWM_DEADTIME_US);
    mcpwm_dead_time_config_t dt_cfg = {
        .posedge_delay_ticks = dt_ticks,
        .negedge_delay_ticks = 0,
        .flags.invert_output = false,
    };

    for (int i = 0; i < 3; i++) {
        esp_err_t err = mcpwm_generator_set_dead_time(obj->gen_high[i], obj->gen_high[i], &dt_cfg);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Dead-time config failed on phase %d (err=%d). Continuing without dead-time.", i, (int)err);
        }
    }

    mcpwm_timer_event_callbacks_t driver_cb = {
        .on_full = inverter_isr,
    };
    mcpwm_timer_register_event_callbacks(obj->timer, &driver_cb, obj);
    mcpwm_timer_enable(obj->timer);
    mcpwm_timer_start_stop(obj->timer, MCPWM_TIMER_START_NO_STOP);

    ESP_LOGI(TAG, "6PWM MCPWM inverter ready, instance %p (port=%d)", &obj->interface, port);
    return &obj->interface;
}
