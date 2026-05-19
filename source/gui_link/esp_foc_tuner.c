/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <string.h>
#include "esp_system.h"
#include "espFoC/gui_link/esp_foc_tuner.h"
#include "espFoC/gui_link/esp_foc_link_session.h"
#include "espFoC/esp_foc.h"
#include "espFoC/gui_link/esp_foc_link.h"
#include "espFoC/calibration/esp_foc_calibration.h"
#include "espFoC/utils/esp_foc_q16.h"

static esp_foc_axis_t *s_axes[ESP_FOC_TUNER_MAX_AXES];

esp_foc_err_t esp_foc_tuner_attach_axis(uint8_t axis_id, esp_foc_axis_t *axis)
{
    if (axis_id >= ESP_FOC_TUNER_MAX_AXES) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
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
    if (axis->state == ESP_FOC_AXIS_STATE_ALIGNING) {
        s |= ESP_FOC_AXIS_STATE_ALIGNING;
    }
    if (axis->state == ESP_FOC_AXIS_STATE_ALIGNED ||
        axis->state == ESP_FOC_AXIS_STATE_RUNNING) {
        s |= ESP_FOC_AXIS_STATE_ALIGNED;
    }
    if (axis->state == ESP_FOC_AXIS_STATE_RUNNING) {
        s |= ESP_FOC_AXIS_STATE_RUNNING;
    }
    return s;
}

static void write_q16(void *dst, q16_t v)
{
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

    if (id == ESP_FOC_TUNER_PARAM_AXIS_STATE ||
        id == ESP_FOC_TUNER_PARAM_AXIS_LAST_ERR ||
        id == ESP_FOC_TUNER_PARAM_NVS_PRESENT) {
        if (*response_len < 1) {
            return ESP_FOC_ERR_INVALID_ARG;
        }
        uint8_t *b = (uint8_t *)response;
        switch (id) {
        case ESP_FOC_TUNER_PARAM_AXIS_STATE:
            b[0] = compute_axis_state(axis);
            break;
        case ESP_FOC_TUNER_PARAM_AXIS_LAST_ERR:
            b[0] = (uint8_t)(int8_t)axis->state;
            break;
        case ESP_FOC_TUNER_PARAM_NVS_PRESENT:
            b[0] = esp_foc_calibration_present(axis->cal.axis_id) ? 1 : 0;
            break;
        default: break;
        }
        *response_len = 1;
        return ESP_FOC_OK;
    }

    if (*response_len < 4) {
        return ESP_FOC_ERR_INVALID_ARG;
    }

    if (id == ESP_FOC_TUNER_PARAM_FIRMWARE_TYPE) {
        uint32_t v = esp_foc_tuner_firmware_type();
        uint8_t *b = (uint8_t *)response;
        b[0] = (uint8_t)v;
        b[1] = (uint8_t)(v >> 8);
        b[2] = (uint8_t)(v >> 16);
        b[3] = (uint8_t)(v >> 24);
        *response_len = 4;
        return ESP_FOC_OK;
    }

    if (id == ESP_FOC_TUNER_PARAM_MOTOR_POLE_PAIRS) {
        int32_t pp = (int32_t)axis->motor_pole_pairs;
        uint8_t *b = (uint8_t *)response;
        uint32_t u = (uint32_t)pp;
        b[0] = (uint8_t)(u & 0xFFu);
        b[1] = (uint8_t)((u >> 8) & 0xFFu);
        b[2] = (uint8_t)((u >> 16) & 0xFFu);
        b[3] = (uint8_t)((u >> 24) & 0xFFu);
        *response_len = 4;
        return ESP_FOC_OK;
    }

    q16_t kp = 0, ki = 0, kd = 0, kff = 0, lim = 0;
    esp_foc_axis_get_current_loop_gains_q16(axis, &kp, &ki, &kd, &kff, &lim);

    q16_t value = 0;
    switch (id) {
    case ESP_FOC_TUNER_PARAM_KP_Q16:          value = kp;  break;
    case ESP_FOC_TUNER_PARAM_KI_Q16:          value = ki;  break;
    case ESP_FOC_TUNER_PARAM_INT_LIM_Q16:     value = lim; break;
    case ESP_FOC_TUNER_PARAM_V_MAX_Q16:       value = axis->mod_index_limit_q16; break;
    case ESP_FOC_TUNER_PARAM_I_FILTER_FC_Q16: value = axis->current_filter_fc_hz_q16; break;
    case ESP_FOC_TUNER_PARAM_LOOP_FS_HZ_Q16:  value = axis->current_filter_fs_hz_q16; break;
    case ESP_FOC_TUNER_PARAM_KD_Q16:          value = kd;  break;
    case ESP_FOC_TUNER_PARAM_KFF_Q16:         value = kff; break;
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
    if (payload == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if (payload_len < 4) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if (id == ESP_FOC_TUNER_WRITE_MOTOR_POLE_PAIRS) {
        const uint8_t *p = (const uint8_t *)payload;
        uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
            ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
        int32_t pp = (int32_t)u;
        if (pp < 1 || pp > 64) {
            return ESP_FOC_ERR_INVALID_ARG;
        }
        axis->motor_pole_pairs = (int)pp;
        esp_foc_axis_refresh_encoder_q16_scales(axis);
        return ESP_FOC_OK;
    }
    q16_t v = read_q16(payload);

    if (id == ESP_FOC_TUNER_WRITE_KP_Q16 ||
        id == ESP_FOC_TUNER_WRITE_KI_Q16 ||
        id == ESP_FOC_TUNER_WRITE_INT_LIM_Q16 ||
        id == ESP_FOC_TUNER_WRITE_KD_Q16 ||
        id == ESP_FOC_TUNER_WRITE_KFF_Q16) {
        q16_t kp, ki, kd, kff, lim;
        esp_foc_axis_get_current_loop_gains_q16(axis, &kp, &ki, &kd, &kff, &lim);
        switch (id) {
        case ESP_FOC_TUNER_WRITE_KP_Q16:      kp  = v; break;
        case ESP_FOC_TUNER_WRITE_KI_Q16:      ki  = v; break;
        case ESP_FOC_TUNER_WRITE_INT_LIM_Q16: lim = v; break;
        case ESP_FOC_TUNER_WRITE_KD_Q16:      kd  = v; break;
        case ESP_FOC_TUNER_WRITE_KFF_Q16:     kff = v; break;
        default: break;
        }
        return esp_foc_axis_set_current_loop_gains_q16(axis, kp, ki, kd, kff, lim);
    }

    if (id == ESP_FOC_TUNER_WRITE_I_FILTER_FC_Q16) {
        if (axis->isensor_driver == NULL ||
            axis->isensor_driver->set_filter_cutoff == NULL) {
            return ESP_FOC_ERR_AXIS_INVALID_STATE;
        }
        float fc_hz = q16_to_float(v);
        float fs_hz = q16_to_float(axis->current_filter_fs_hz_q16);
        axis->isensor_driver->set_filter_cutoff(axis->isensor_driver,
                                                fc_hz, fs_hz);
        axis->current_filter_fc_hz_q16 = v;
        return ESP_FOC_OK;
    }

    if (id == ESP_FOC_TUNER_WRITE_TARGET_ID_Q16 ||
        id == ESP_FOC_TUNER_WRITE_TARGET_IQ_Q16) {
        if (axis->state != ESP_FOC_AXIS_STATE_RUNNING) {
            return ESP_FOC_ERR_AXIS_INVALID_STATE;
        }
        esp_foc_critical_enter();
        if (id == ESP_FOC_TUNER_WRITE_TARGET_ID_Q16) {
            axis->target_i_d.raw = v;
        } else {
            axis->target_i_q.raw = v;
        }
        esp_foc_critical_leave();
        return ESP_FOC_OK;
    }

    return ESP_FOC_ERR_INVALID_ARG;
}

static void board_reset_task(void *arg)
{
    (void)arg;
    esp_foc_sleep_ms(150);
    esp_restart();
}

static esp_foc_err_t handle_exec_session(esp_foc_tuner_id_t id,
                                         const void *payload,
                                         size_t payload_len,
                                         void *response,
                                         size_t *response_len)
{
    if (id == ESP_FOC_TUNER_CMD_CONNECT) {
        if (payload == NULL || payload_len < 3) {
            return ESP_FOC_ERR_INVALID_ARG;
        }
        const uint8_t *p = (const uint8_t *)payload;
        if (p[0] != ESP_FOC_LINK_PROTO_VER) {
            return ESP_FOC_ERR_INVALID_ARG;
        }
        (void)(p[1] | ((uint16_t)p[2] << 8));
        esp_foc_link_session_on_connect();
        if (response == NULL || response_len == NULL || *response_len < 10) {
            return ESP_FOC_ERR_INVALID_ARG;
        }
        uint8_t *b = (uint8_t *)response;
        b[0] = ESP_FOC_LINK_PROTO_VER;
        uint32_t ft = esp_foc_tuner_firmware_type();
        b[1] = (uint8_t)(ft & 0xFFu);
        b[2] = (uint8_t)((ft >> 8) & 0xFFu);
        b[3] = (uint8_t)((ft >> 16) & 0xFFu);
        b[4] = (uint8_t)((ft >> 24) & 0xFFu);
        b[5] = 1u;
#if defined(CONFIG_ESP_FOC_SCOPE)
        b[6] = (uint8_t)CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS;
#else
        b[6] = 0u;
#endif
        b[7] = (uint8_t)(ESP_FOC_LINK_HEARTBEAT_PERIOD_MS & 0xFFu);
        b[8] = (uint8_t)((ESP_FOC_LINK_HEARTBEAT_PERIOD_MS >> 8) & 0xFFu);
        *response_len = 9;
        return ESP_FOC_OK;
    }
    if (id == ESP_FOC_TUNER_CMD_DISCONNECT) {
        esp_foc_link_session_on_disconnect();
        if (response_len != NULL) {
            *response_len = 0;
        }
        return ESP_FOC_OK;
    }
    if (id == ESP_FOC_TUNER_CMD_SCOPE_START) {
#if defined(CONFIG_ESP_FOC_SCOPE)
        esp_foc_link_session_on_scope_start();
        if (response_len != NULL) {
            *response_len = 0;
        }
        return ESP_FOC_OK;
#else
        return ESP_FOC_ERR_INVALID_ARG;
#endif
    }
    if (id == ESP_FOC_TUNER_CMD_SCOPE_STOP) {
        esp_foc_link_session_on_scope_stop();
        if (response_len != NULL) {
            *response_len = 0;
        }
        return ESP_FOC_OK;
    }
    return ESP_FOC_ERR_INVALID_ARG;
}

static esp_foc_err_t handle_exec(esp_foc_axis_t *axis,
                                 esp_foc_tuner_id_t id,
                                 const void *payload, size_t payload_len,
                                 void *response, size_t *response_len)
{
    (void)payload;
    (void)payload_len;

    if (id == ESP_FOC_TUNER_CMD_CONNECT ||
        id == ESP_FOC_TUNER_CMD_DISCONNECT ||
        id == ESP_FOC_TUNER_CMD_SCOPE_START ||
        id == ESP_FOC_TUNER_CMD_SCOPE_STOP) {
        return handle_exec_session(id, payload, payload_len,
                                   response, response_len);
    }

    if (id == ESP_FOC_TUNER_CMD_STOP_AXIS) {
        return esp_foc_stop(axis);
    }

    if (id == ESP_FOC_TUNER_CMD_ALIGN_AXIS) {
        return esp_foc_align_axis(axis);
    }

    if (id == ESP_FOC_TUNER_CMD_RUN_AXIS) {
        if (axis->state != ESP_FOC_AXIS_STATE_ALIGNED) {
            return ESP_FOC_ERR_NOT_ALIGNED;
        }
        return esp_foc_run(axis);
    }

    if (id == ESP_FOC_TUNER_CMD_STORE_NVS) {
        return esp_foc_calibration_axis_tuner_store(axis);
    }

    if (id == ESP_FOC_TUNER_CMD_ERASE_NVS) {
        if (axis->state == ESP_FOC_AXIS_STATE_RUNNING ||
            axis->state == ESP_FOC_AXIS_STATE_ALIGNED) {
            (void)esp_foc_stop(axis);
        }
        esp_foc_critical_enter();
        axis->target_i_d.raw = 0;
        axis->target_i_q.raw = 0;
        esp_foc_critical_leave();
        esp_foc_axis_apply_bypass_current_loop_gains(axis);
        esp_foc_err_t err = esp_foc_calibration_erase();
        if (err == ESP_FOC_OK) {
            esp_foc_calibration_axis_tuner_clear_cache(axis);
        }
        return err;
    }

    if (id == ESP_FOC_TUNER_CMD_RESET_BOARD) {
        if (esp_foc_task_spawn(board_reset_task, NULL, 3072, 1, NULL) != 0) {
            esp_restart();
        }
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
    if (op == ESP_FOC_TUNER_OP_EXEC &&
        (id == ESP_FOC_TUNER_CMD_CONNECT ||
         id == ESP_FOC_TUNER_CMD_DISCONNECT ||
         id == ESP_FOC_TUNER_CMD_SCOPE_START ||
         id == ESP_FOC_TUNER_CMD_SCOPE_STOP)) {
        return handle_exec_session(id, payload, payload_len,
                                 response, response_len);
    }

    esp_foc_axis_t *axis = get_axis(axis_id);
    if (axis == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }

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
        return handle_exec(axis, id, payload, payload_len,
                           response, response_len);
    default:
        return ESP_FOC_ERR_INVALID_ARG;
    }
}

static void send_response_frame(uint8_t seq,
                                int8_t status,
                                const uint8_t *resp_payload,
                                size_t resp_len)
{
    uint8_t app[2 + ESP_FOC_LINK_MAX_PAYLOAD];
    uint8_t frame[ESP_FOC_LINK_MAX_FRAME];
    size_t frame_len = 0;

    if (resp_len > sizeof(app) - 2) {
        resp_len = sizeof(app) - 2;
    }
    app[0] = (uint8_t)status;
    app[1] = seq;
    if (resp_len > 0 && resp_payload != NULL) {
        memcpy(&app[2], resp_payload, resp_len);
    }
    if (esp_foc_link_encode(ESP_FOC_LINK_CH_TUNER, seq, app, 2 + resp_len,
                            frame, sizeof(frame), &frame_len) == ESP_FOC_LINK_OK) {
        esp_foc_tuner_send_callback(frame, frame_len);
    }
}

void esp_foc_tuner_dispatch_link_frame(uint8_t seq,
                                       const uint8_t *payload,
                                       size_t payload_len)
{
    bool silent = false;
    esp_foc_tuner_op_t op;
    esp_foc_tuner_id_t id;

    if (payload_len < 4) {
        send_response_frame(seq, (int8_t)ESP_FOC_ERR_INVALID_ARG, NULL, 0);
        return;
    }
    op = (esp_foc_tuner_op_t)payload[0];
    id = (esp_foc_tuner_id_t)(payload[1] | ((uint16_t)payload[2] << 8));

    if (!esp_foc_link_session_tuner_may_dispatch(op, id, &silent)) {
        if (!silent) {
            send_response_frame(seq, (int8_t)ESP_FOC_ERR_INVALID_ARG, NULL, 0);
        }
        return;
    }

    {
        uint8_t axis_id = payload[3];
        const uint8_t *cmd_payload = payload + 4;
        size_t cmd_len = payload_len - 4;
        uint8_t resp_buf[16];
        size_t resp_len = sizeof(resp_buf);
        esp_foc_err_t err = esp_foc_tuner_handle_request(
            axis_id, op, id, cmd_payload, cmd_len, resp_buf, &resp_len);
        if (err != ESP_FOC_OK) {
            resp_len = 0;
        }
        send_response_frame(seq, (int8_t)err, resp_buf, resp_len);
    }
}

void esp_foc_tuner_process_byte(uint8_t byte)
{
    esp_foc_link_process_byte(byte);
}

void esp_foc_tuner_reactor_reset(void)
{
    esp_foc_link_reactor_reset();
}

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

__attribute__((weak)) uint32_t esp_foc_tuner_firmware_type(void)
{
    return ESP_FOC_TUNER_FIRMWARE_TYPE_GENERIC;
}
