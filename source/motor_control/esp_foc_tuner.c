/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <string.h>
#include "espFoC/esp_foc_tuner.h"
#include "espFoC/esp_foc_axis_tuning.h"

/* Per-axis registry. Pointers are only mutated from esp_foc_tuner_attach_axis,
 * which is expected to be called from a non-time-critical context. */
static esp_foc_axis_t *s_axes[ESP_FOC_TUNER_MAX_AXES];

esp_foc_err_t esp_foc_tuner_attach_axis(uint8_t axis_id, esp_foc_axis_t *axis)
{
    if (axis_id >= ESP_FOC_TUNER_MAX_AXES) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    s_axes[axis_id] = axis;
    return ESP_FOC_OK;
}

static esp_foc_axis_t *get_axis(uint8_t axis_id)
{
    if (axis_id >= ESP_FOC_TUNER_MAX_AXES) {
        return NULL;
    }
    return s_axes[axis_id];
}

static void write_q16(void *dst, q16_t v)
{
    /* Little-endian byte serialization (matches typical wire formats). */
    uint8_t *b = (uint8_t *)dst;
    uint32_t u = (uint32_t)v;
    b[0] = (uint8_t)(u & 0xFF);
    b[1] = (uint8_t)((u >> 8) & 0xFF);
    b[2] = (uint8_t)((u >> 16) & 0xFF);
    b[3] = (uint8_t)((u >> 24) & 0xFF);
}

static q16_t read_q16(const void *src)
{
    const uint8_t *b = (const uint8_t *)src;
    uint32_t u = (uint32_t)b[0] |
                 ((uint32_t)b[1] << 8) |
                 ((uint32_t)b[2] << 16) |
                 ((uint32_t)b[3] << 24);
    return (q16_t)u;
}

static esp_foc_err_t handle_read(esp_foc_axis_t *axis,
                                 esp_foc_tuner_id_t id,
                                 void *response, size_t *response_len)
{
    if (response == NULL || response_len == NULL || *response_len < 4) {
        return ESP_FOC_ERR_INVALID_ARG;
    }

    q16_t kp = 0, ki = 0, lim = 0;
    esp_foc_axis_get_current_pi_gains_q16(axis, &kp, &ki, &lim);

    q16_t value = 0;
    switch (id) {
    case ESP_FOC_TUNER_PARAM_KP_Q16:      value = kp;  break;
    case ESP_FOC_TUNER_PARAM_KI_Q16:      value = ki;  break;
    case ESP_FOC_TUNER_PARAM_INT_LIM_Q16: value = lim; break;
    case ESP_FOC_TUNER_PARAM_V_MAX_Q16:   value = axis->max_voltage; break;
    default:
        return ESP_FOC_ERR_INVALID_ARG;
    }

    write_q16(response, value);
    *response_len = 4;
    return ESP_FOC_OK;
}

static esp_foc_err_t handle_write(esp_foc_axis_t *axis,
                                  esp_foc_tuner_id_t id,
                                  const void *payload, size_t payload_len)
{
    if (payload == NULL || payload_len < 4) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    q16_t v = read_q16(payload);

    /* All three values are written together: read current, swap one, push back.
     * The set call performs the atomic swap so the PID never sees a partial
     * update. */
    q16_t kp, ki, lim;
    esp_foc_axis_get_current_pi_gains_q16(axis, &kp, &ki, &lim);
    switch (id) {
    case ESP_FOC_TUNER_WRITE_KP_Q16:      kp  = v; break;
    case ESP_FOC_TUNER_WRITE_KI_Q16:      ki  = v; break;
    case ESP_FOC_TUNER_WRITE_INT_LIM_Q16: lim = v; break;
    default:
        return ESP_FOC_ERR_INVALID_ARG;
    }
    return esp_foc_axis_set_current_pi_gains_q16(axis, kp, ki, lim);
}

static esp_foc_err_t handle_exec(esp_foc_axis_t *axis,
                                 esp_foc_tuner_id_t id,
                                 const void *payload, size_t payload_len)
{
    if (id == ESP_FOC_TUNER_CMD_RECOMPUTE_GAINS) {
        if (payload == NULL || payload_len < 12) {
            return ESP_FOC_ERR_INVALID_ARG;
        }
        const uint8_t *p = (const uint8_t *)payload;
        q16_t r  = read_q16(p);
        q16_t l  = read_q16(p + 4);
        q16_t bw = read_q16(p + 8);
        return esp_foc_axis_retune_current_pi_q16(axis, r, l, bw);
    }
    return ESP_FOC_ERR_INVALID_ARG;
}

esp_foc_err_t esp_foc_tuner_handle_request(
    uint8_t axis_id,
    esp_foc_tuner_op_t op,
    esp_foc_tuner_id_t id,
    const void *payload,
    size_t payload_len,
    void *response,
    size_t *response_len)
{
    esp_foc_axis_t *axis = get_axis(axis_id);
    if (axis == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }

    /* Default: no response payload. */
    size_t initial_resp_cap = (response_len != NULL) ? *response_len : 0;
    if (response_len != NULL) {
        *response_len = 0;
    }

    switch (op) {
    case ESP_FOC_TUNER_OP_READ:
        if (response_len == NULL) {
            return ESP_FOC_ERR_INVALID_ARG;
        }
        *response_len = initial_resp_cap;
        return handle_read(axis, id, response, response_len);
    case ESP_FOC_TUNER_OP_WRITE:
        return handle_write(axis, id, payload, payload_len);
    case ESP_FOC_TUNER_OP_EXEC:
        return handle_exec(axis, id, payload, payload_len);
    default:
        return ESP_FOC_ERR_INVALID_ARG;
    }
}

/* --- Default weak transport callbacks (no-op) -------------------------- */

__attribute__((weak)) void esp_foc_tuner_init_bus_callback(void)
{
}

__attribute__((weak)) int esp_foc_tuner_recv_callback(uint8_t *buf, size_t max)
{
    (void)buf;
    (void)max;
    return 0;
}

__attribute__((weak)) void esp_foc_tuner_send_callback(const uint8_t *buf, size_t len)
{
    (void)buf;
    (void)len;
}
