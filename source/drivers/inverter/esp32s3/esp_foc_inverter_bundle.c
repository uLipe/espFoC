/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include "sdkconfig.h"
#include "espFoC/drivers/esp_foc_inverter_mcpwm.h"
#include "esp_foc_inverter_mcpwm_6pwm.h"
#include "esp_foc_inverter_mcpwm_3pwm.h"
#include "esp_foc_isensor_adc_private.h"
#include "esp_log.h"
#include "soc/soc_caps.h"

#if SOC_ETM_SUPPORTED
#include "esp_foc_inverter_mcpwm_etm.h"
#endif

static const char *TAG = "esp_foc_inv_mcpwm";

typedef struct {
    esp_foc_inverter_t pub;
    esp_foc_inverter_t *pwm;
    esp_foc_isensor_t *adc;
} esp_foc_inverter_bundle_t;

static esp_foc_inverter_bundle_t s_bundles[CONFIG_NOOF_AXIS];

static esp_foc_inverter_bundle_t *bundle_from_pub(esp_foc_inverter_t *self)
{
    if (self == NULL) {
        return NULL;
    }
    for (int i = 0; i < CONFIG_NOOF_AXIS; ++i) {
        if (&s_bundles[i].pub == self) {
            return &s_bundles[i];
        }
    }
    return NULL;
}

static void uni_set_inverter_callback(esp_foc_inverter_t *self,
                                      esp_foc_inverter_callback_t callback,
                                      void *argument)
{
    esp_foc_inverter_bundle_t *b = bundle_from_pub(self);
    if (b != NULL && b->pwm != NULL && b->pwm->set_inverter_callback != NULL) {
        b->pwm->set_inverter_callback(b->pwm, callback, argument);
    }
}

static q16_t uni_get_dc_link_voltage(esp_foc_inverter_t *self)
{
    esp_foc_inverter_bundle_t *b = bundle_from_pub(self);
    if (b != NULL && b->pwm != NULL && b->pwm->get_dc_link_voltage != NULL) {
        return b->pwm->get_dc_link_voltage(b->pwm);
    }
    return 0;
}

static void uni_set_duties(esp_foc_inverter_t *self, q16_t duty_a, q16_t duty_b, q16_t duty_c)
{
    esp_foc_inverter_bundle_t *b = bundle_from_pub(self);
    if (b != NULL && b->pwm != NULL && b->pwm->set_duties != NULL) {
        b->pwm->set_duties(b->pwm, duty_a, duty_b, duty_c);
    }
}

static uint32_t uni_get_inverter_pwm_rate(esp_foc_inverter_t *self)
{
    esp_foc_inverter_bundle_t *b = bundle_from_pub(self);
    if (b != NULL && b->pwm != NULL && b->pwm->get_inverter_pwm_rate != NULL) {
        return b->pwm->get_inverter_pwm_rate(b->pwm);
    }
    return 0;
}

static void uni_enable(esp_foc_inverter_t *self)
{
    esp_foc_inverter_bundle_t *b = bundle_from_pub(self);
    if (b != NULL && b->pwm != NULL && b->pwm->enable != NULL) {
        b->pwm->enable(b->pwm);
    }
}

static void uni_disable(esp_foc_inverter_t *self)
{
    esp_foc_inverter_bundle_t *b = bundle_from_pub(self);
    if (b != NULL && b->pwm != NULL && b->pwm->disable != NULL) {
        b->pwm->disable(b->pwm);
    }
}

static void uni_fetch_isensors(esp_foc_inverter_t *self, esp_foc_inverter_isensor_values_t *values)
{
    esp_foc_inverter_bundle_t *b = bundle_from_pub(self);
    if (b == NULL || b->adc == NULL || b->adc->fetch_isensors == NULL || values == NULL) {
        return;
    }
    isensor_values_t raw;
    b->adc->fetch_isensors(b->adc, &raw);
    values->iu_axis_0 = raw.iu_axis_0;
    values->iv_axis_0 = raw.iv_axis_0;
    values->iw_axis_0 = raw.iw_axis_0;
    values->iu_axis_1 = raw.iu_axis_1;
    values->iv_axis_1 = raw.iv_axis_1;
    values->iw_axis_1 = raw.iw_axis_1;
}

static void uni_sample_isensors(esp_foc_inverter_t *self)
{
    esp_foc_inverter_bundle_t *b = bundle_from_pub(self);
    if (b != NULL && b->adc != NULL && b->adc->sample_isensors != NULL) {
        b->adc->sample_isensors(b->adc);
    }
}

static void uni_calibrate_isensors(esp_foc_inverter_t *self, int calibration_rounds)
{
    esp_foc_inverter_bundle_t *b = bundle_from_pub(self);
    if (b != NULL && b->adc != NULL && b->adc->calibrate_isensors != NULL) {
        b->adc->calibrate_isensors(b->adc, calibration_rounds);
    }
}

static void uni_set_filter_cutoff(esp_foc_inverter_t *self, float fc_hz, float fs_hz)
{
    esp_foc_inverter_bundle_t *b = bundle_from_pub(self);
    if (b != NULL && b->adc != NULL && b->adc->set_filter_cutoff != NULL) {
        b->adc->set_filter_cutoff(b->adc, fc_hz, fs_hz);
    }
}

static void uni_set_publish_targets(esp_foc_inverter_t *self,
                                      q16_t *i_alpha,
                                      q16_t *i_beta,
                                      q16_t *iu,
                                      q16_t *iv)
{
    esp_foc_inverter_bundle_t *b = bundle_from_pub(self);
    if (b != NULL && b->adc != NULL && b->adc->set_publish_targets != NULL) {
        b->adc->set_publish_targets(b->adc, i_alpha, i_beta, iu, iv);
    }
}

static void fill_unified_vtable(esp_foc_inverter_t *pub)
{
    pub->set_inverter_callback = uni_set_inverter_callback;
    pub->get_dc_link_voltage = uni_get_dc_link_voltage;
    pub->set_duties = uni_set_duties;
    pub->get_inverter_pwm_rate = uni_get_inverter_pwm_rate;
    pub->enable = uni_enable;
    pub->disable = uni_disable;
    pub->fetch_isensors = uni_fetch_isensors;
    pub->sample_isensors = uni_sample_isensors;
    pub->calibrate_isensors = uni_calibrate_isensors;
    pub->set_filter_cutoff = uni_set_filter_cutoff;
    pub->set_publish_targets = uni_set_publish_targets;
}

static esp_foc_inverter_t *bundle_init(int port,
                                       esp_foc_inverter_t *pwm,
                                       esp_foc_isensor_adc_config_t *adc_cfg)
{
    if (port < 0 || port >= CONFIG_NOOF_AXIS || pwm == NULL || adc_cfg == NULL) {
        return NULL;
    }

    esp_foc_inverter_bundle_t *b = &s_bundles[port];
    b->pwm = pwm;
    b->adc = isensor_adc_new(adc_cfg);
    if (b->adc == NULL) {
        ESP_LOGE(TAG, "ADC init failed on port %d", port);
        return NULL;
    }

#if SOC_ETM_SUPPORTED
    esp_foc_err_t etm_err =
        inverter_mcpwm_connect_adc_etm(pwm, INVERTER_MCPWM_ADC_TRIGGER_PEAK);
    if (etm_err != ESP_FOC_OK) {
        ESP_LOGW(TAG, "ETM ADC trigger wiring failed (%d); software trigger", (int)etm_err);
        isensor_adc_set_software_trigger(b->adc);
    } else {
        isensor_adc_set_etm_trigger(b->adc);
    }
#else
    isensor_adc_set_software_trigger(b->adc);
#endif

    fill_unified_vtable(&b->pub);
    ESP_LOGI(TAG, "unified MCPWM inverter ready port=%d pub=%p", port, (void *)&b->pub);
    return &b->pub;
}

esp_foc_inverter_t *esp_foc_inverter_mcpwm_6pwm_new(int gpio_u_high, int gpio_u_low,
                                                    int gpio_v_high, int gpio_v_low,
                                                    int gpio_w_high, int gpio_w_low,
                                                    int gpio_enable,
                                                    float dc_link_voltage,
                                                    int port,
                                                    adc_channel_t adc_ch_u,
                                                    adc_channel_t adc_ch_v,
                                                    float amp_gain,
                                                    float shunt_resistance)
{
    esp_foc_inverter_t *pwm = inverter_6pwm_mpcwm_new(gpio_u_high, gpio_u_low, gpio_v_high,
                                                      gpio_v_low, gpio_w_high, gpio_w_low,
                                                      gpio_enable, dc_link_voltage, port);
    if (pwm == NULL) {
        return NULL;
    }

    esp_foc_isensor_adc_config_t adc_cfg = {
        .channels = {adc_ch_u, adc_ch_v},
        .unit = ADC_UNIT_1,
        .amp_gain = amp_gain,
        .shunt_resistance = shunt_resistance,
    };
    return bundle_init(port, pwm, &adc_cfg);
}

esp_foc_inverter_t *esp_foc_inverter_mcpwm_3pwm_new(int gpio_u, int gpio_v, int gpio_w,
                                                    int gpio_enable,
                                                    float dc_link_voltage,
                                                    int port,
                                                    adc_channel_t adc_ch_u,
                                                    adc_channel_t adc_ch_v,
                                                    float amp_gain,
                                                    float shunt_resistance)
{
    esp_foc_inverter_t *pwm =
        inverter_3pwm_mpcwm_new(gpio_u, gpio_v, gpio_w, gpio_enable, dc_link_voltage, port);
    if (pwm == NULL) {
        return NULL;
    }

    esp_foc_isensor_adc_config_t adc_cfg = {
        .channels = {adc_ch_u, adc_ch_v},
        .unit = ADC_UNIT_1,
        .amp_gain = amp_gain,
        .shunt_resistance = shunt_resistance,
    };
    return bundle_init(port, pwm, &adc_cfg);
}
