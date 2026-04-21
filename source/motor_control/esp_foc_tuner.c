/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <string.h>
#include "espFoC/esp_foc_tuner.h"
#include "espFoC/esp_foc_axis_tuning.h"
#include "espFoC/esp_foc_axis.h"

/* Per-axis registry. Pointers are only mutated from esp_foc_tuner_attach_axis,
 * which is expected to be called from a non-time-critical context. */
static esp_foc_axis_t *s_axes[ESP_FOC_TUNER_MAX_AXES];

esp_foc_err_t esp_foc_tuner_attach_axis(uint8_t axis_id, esp_foc_axis_t *axis)
{
    if (axis_id >= ESP_FOC_TUNER_MAX_AXES) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    /* NULL is allowed (detach); a non-NULL pointer must be a real,
     * initialized axis so no later request operates on garbage memory. */
    if (axis != NULL && axis->magic != ESP_FOC_AXIS_MAGIC) {
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }
    s_axes[axis_id] = axis;
    return ESP_FOC_OK;
}

static esp_foc_axis_t *get_axis(uint8_t axis_id)
{
    if (axis_id >= ESP_FOC_TUNER_MAX_AXES) {
        return NULL;
    }
    esp_foc_axis_t *a = s_axes[axis_id];
    if (a == NULL || a->magic != ESP_FOC_AXIS_MAGIC) {
        return NULL;
    }
    return a;
}

static uint8_t compute_axis_state(const esp_foc_axis_t *axis)
{
    uint8_t s = 0;
    if (axis->magic == ESP_FOC_AXIS_MAGIC) {
        s |= ESP_FOC_AXIS_STATE_INITIALIZED;
    }
    if (axis->rotor_aligned == ESP_FOC_OK) {
        s |= ESP_FOC_AXIS_STATE_ALIGNED;
    }
    /* esp_foc_run() spawns the outer-loop runner which sets regulator_ev. */
    if (axis->regulator_ev != NULL) {
        s |= ESP_FOC_AXIS_STATE_RUNNING;
    }
    if (axis->tuner_override.active) {
        s |= ESP_FOC_AXIS_STATE_TUNER_OVERRIDE;
    }
    return s;
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
    if (response == NULL || response_len == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }

    /* AXIS_STATE / AXIS_LAST_ERR return a single byte; everything else 4. */
    if (id == ESP_FOC_TUNER_PARAM_AXIS_STATE ||
        id == ESP_FOC_TUNER_PARAM_AXIS_LAST_ERR) {
        if (*response_len < 1) {
            return ESP_FOC_ERR_INVALID_ARG;
        }
        uint8_t *b = (uint8_t *)response;
        b[0] = (id == ESP_FOC_TUNER_PARAM_AXIS_STATE)
                 ? compute_axis_state(axis)
                 : (uint8_t)(int8_t)axis->rotor_aligned;
        *response_len = 1;
        return ESP_FOC_OK;
    }

    if (*response_len < 4) {
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

    /* Gain writes feed the existing atomic swap helper so the PID never
     * sees a partial update. */
    if (id == ESP_FOC_TUNER_WRITE_KP_Q16 ||
        id == ESP_FOC_TUNER_WRITE_KI_Q16 ||
        id == ESP_FOC_TUNER_WRITE_INT_LIM_Q16) {
        q16_t kp, ki, lim;
        esp_foc_axis_get_current_pi_gains_q16(axis, &kp, &ki, &lim);
        switch (id) {
        case ESP_FOC_TUNER_WRITE_KP_Q16:      kp  = v; break;
        case ESP_FOC_TUNER_WRITE_KI_Q16:      ki  = v; break;
        case ESP_FOC_TUNER_WRITE_INT_LIM_Q16: lim = v; break;
        default: break;
        }
        return esp_foc_axis_set_current_pi_gains_q16(axis, kp, ki, lim);
    }

    /* Motion-target writes only land while the override is active.
     * The four shadow targets are tiny so we use the global critical
     * section for the swap. */
    if (id == ESP_FOC_TUNER_WRITE_TARGET_ID_Q16 ||
        id == ESP_FOC_TUNER_WRITE_TARGET_IQ_Q16 ||
        id == ESP_FOC_TUNER_WRITE_TARGET_UD_Q16 ||
        id == ESP_FOC_TUNER_WRITE_TARGET_UQ_Q16) {
        if (!axis->tuner_override.active) {
            return ESP_FOC_ERR_AXIS_INVALID_STATE;
        }
        esp_foc_critical_enter();
        switch (id) {
        case ESP_FOC_TUNER_WRITE_TARGET_ID_Q16: axis->tuner_override.target_id = v; break;
        case ESP_FOC_TUNER_WRITE_TARGET_IQ_Q16: axis->tuner_override.target_iq = v; break;
        case ESP_FOC_TUNER_WRITE_TARGET_UD_Q16: axis->tuner_override.target_ud = v; break;
        case ESP_FOC_TUNER_WRITE_TARGET_UQ_Q16: axis->tuner_override.target_uq = v; break;
        default: break;
        }
        esp_foc_critical_leave();
        return ESP_FOC_OK;
    }

    return ESP_FOC_ERR_INVALID_ARG;
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

    if (id == ESP_FOC_TUNER_CMD_OVERRIDE_ON) {
        /* Refuse to take over an axis that is not aligned: spinning a
         * motor with an unaligned encoder produces wild currents. */
        if (axis->rotor_aligned != ESP_FOC_OK) {
            return ESP_FOC_ERR_NOT_ALIGNED;
        }
        esp_foc_critical_enter();
        axis->tuner_override.target_id = 0;
        axis->tuner_override.target_iq = 0;
        axis->tuner_override.target_ud = 0;
        axis->tuner_override.target_uq = 0;
        axis->tuner_override.active = true;
        esp_foc_critical_leave();
        return ESP_FOC_OK;
    }

    if (id == ESP_FOC_TUNER_CMD_OVERRIDE_OFF) {
        esp_foc_critical_enter();
        axis->tuner_override.active = false;
        esp_foc_critical_leave();
        return ESP_FOC_OK;
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
