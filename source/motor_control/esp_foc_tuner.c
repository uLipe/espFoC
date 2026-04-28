/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "espFoC/esp_foc_tuner.h"
#include "espFoC/esp_foc_axis_tuning.h"
#include "espFoC/esp_foc_axis.h"
#include "espFoC/esp_foc_link.h"
#include "espFoC/esp_foc_calibration.h"
#include "espFoC/utils/esp_foc_q16.h"

static const char *tun_tag = "esp_foc_tuner";

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

/* Forward declaration: tuner_log is defined further down but
 * handle_write needs it for the LOG-channel breadcrumb on the
 * current-filter cutoff write. */
static void tuner_log(const char *msg);

static void board_reset_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(150));
    esp_restart();
}

static esp_foc_err_t handle_read(esp_foc_axis_t *axis,
                                 esp_foc_tuner_id_t id,
                                 void *response, size_t *response_len)
{
    if (response == NULL || response_len == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }

    /* Single-byte responses */
    if (id == ESP_FOC_TUNER_PARAM_AXIS_STATE ||
        id == ESP_FOC_TUNER_PARAM_AXIS_LAST_ERR ||
        id == ESP_FOC_TUNER_PARAM_NVS_PRESENT ||
        id == ESP_FOC_TUNER_PARAM_SKIP_TORQUE_U8) {
        if (*response_len < 1) {
            return ESP_FOC_ERR_INVALID_ARG;
        }
        uint8_t *b = (uint8_t *)response;
        switch (id) {
        case ESP_FOC_TUNER_PARAM_AXIS_STATE:
            b[0] = compute_axis_state(axis);
            break;
        case ESP_FOC_TUNER_PARAM_AXIS_LAST_ERR:
            b[0] = (uint8_t)(int8_t)axis->rotor_aligned;
            break;
        case ESP_FOC_TUNER_PARAM_NVS_PRESENT:
            b[0] = esp_foc_calibration_present(axis->nvs_axis_id) ? 1 : 0;
            break;
        case ESP_FOC_TUNER_PARAM_SKIP_TORQUE_U8:
            b[0] = axis->skip_torque_control ? 1u : 0u;
            break;
        default: break;
        }
        *response_len = 1;
        return ESP_FOC_OK;
    }

    if (*response_len < 4) {
        return ESP_FOC_ERR_INVALID_ARG;
    }

    /* FIRMWARE_TYPE is a u32 magic — handled separately because it does
     * not come from the axis. */
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
        if (*response_len < 4) {
            return ESP_FOC_ERR_INVALID_ARG;
        }
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

    q16_t kp = 0, ki = 0, lim = 0;
    esp_foc_axis_get_current_pi_gains_q16(axis, &kp, &ki, &lim);

    q16_t value = 0;
    switch (id) {
    case ESP_FOC_TUNER_PARAM_KP_Q16:          value = kp;  break;
    case ESP_FOC_TUNER_PARAM_KI_Q16:          value = ki;  break;
    case ESP_FOC_TUNER_PARAM_INT_LIM_Q16:     value = lim; break;
    case ESP_FOC_TUNER_PARAM_V_MAX_Q16:       value = axis->max_voltage; break;
    case ESP_FOC_TUNER_PARAM_I_FILTER_FC_Q16: value = axis->current_filter_fc_hz_q16; break;
    case ESP_FOC_TUNER_PARAM_LOOP_FS_HZ_Q16:  value = axis->current_filter_fs_hz_q16; break;
    case ESP_FOC_TUNER_PARAM_MOTOR_R_OHM_Q16: value = axis->nvs_motor_r_ohm; break;
    case ESP_FOC_TUNER_PARAM_MOTOR_L_H_Q16:   value = axis->nvs_motor_l_h; break;
    case ESP_FOC_TUNER_PARAM_MOTOR_BW_HZ_Q16: value = axis->nvs_bandwidth_hz; break;
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
    if (id == ESP_FOC_TUNER_WRITE_SKIP_TORQUE_U8) {
        if (payload_len < 1) {
            return ESP_FOC_ERR_INVALID_ARG;
        }
        const uint8_t *p = (const uint8_t *)payload;
        esp_foc_critical_enter();
        axis->skip_torque_control = (p[0] != 0) ? 1 : 0;
        esp_foc_critical_leave();
        return ESP_FOC_OK;
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
        {
            char msg[56];
            (void)snprintf(msg, sizeof(msg),
                "motor: pole pairs = %d", (int)axis->motor_pole_pairs);
            tuner_log(msg);
        }
        return ESP_FOC_OK;
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

    /* Current-sense LPF cutoff. Re-runs the Butterworth designer on
     * the per-channel biquads inside the isensor driver. fs comes
     * from the value the axis captured at init (loop rate). */
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
        char msg[48];
        snprintf(msg, sizeof(msg), "current filter: fc = %.1f Hz", (double)fc_hz);
        tuner_log(msg);
        return ESP_FOC_OK;
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

/* Send a one-line text message on the LOG link channel. Used by the
 * blocking commands (alignment, persist) to surface progress without
 * blocking the user behind a giant tuner-channel response. */
static void tuner_log(const char *msg)
{
    if (msg == NULL) {
        return;
    }
    size_t len = strlen(msg);
    if (len == 0) {
        return;
    }
    if (len > ESP_FOC_LINK_MAX_PAYLOAD) {
        len = ESP_FOC_LINK_MAX_PAYLOAD;
    }
    uint8_t frame[ESP_FOC_LINK_MAX_FRAME];
    size_t flen = 0;
    if (esp_foc_link_encode(ESP_FOC_LINK_CH_LOG, 0,
                            (const uint8_t *)msg, len,
                            frame, sizeof(frame), &flen) == ESP_FOC_LINK_OK) {
        esp_foc_tuner_send_callback(frame, flen);
    }
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

    if (id == ESP_FOC_TUNER_CMD_ALIGN_AXIS) {
        /* Blocking — parking + Vq-sweep direction + re-park takes ~4-5 s
         * with default sweep steps. The host should use a long timeout
         * for this command; LOG frames bracket the call. */
        tuner_log("alignment: started");
        esp_foc_err_t err = esp_foc_align_axis(axis);
        if (err == ESP_FOC_OK) {
            tuner_log(axis->natural_direction == Q16_ONE
                      ? "alignment: complete (direction = CW)"
                      : "alignment: complete (direction = CCW)");
        } else {
            tuner_log("alignment: failed");
        }
        return err;
    }

    if (id == ESP_FOC_TUNER_CMD_PERSIST_NVS) {
        /* Payload [R_q16, L_q16, bw_q16] — host knows the motor params
         * the user just typed in; firmware combines them with its own
         * current Kp/Ki/integrator_limit + the live current-filter
         * cutoff before persisting. */
        esp_foc_calibration_data_t data;
        if (esp_foc_calibration_load(axis->nvs_axis_id, &data) != ESP_FOC_OK) {
            memset(&data, 0, sizeof(data));
        }
        esp_foc_axis_get_current_pi_gains_q16(axis, &data.kp, &data.ki,
                                              &data.integrator_limit);
        data.current_filter_fc_hz = axis->current_filter_fc_hz_q16;
        if (payload != NULL && payload_len >= 12) {
            const uint8_t *p = (const uint8_t *)payload;
            data.motor_r_ohm = read_q16(p);
            data.motor_l_h = read_q16(p + 4);
            data.bandwidth_hz = read_q16(p + 8);
        }
        esp_foc_calibration_pack_pole_pairs(
            &data, (int32_t)axis->motor_pole_pairs);
        esp_foc_err_t err = esp_foc_calibration_save(axis->nvs_axis_id, &data);
        if (err == ESP_FOC_OK) {
            axis->nvs_motor_r_ohm = data.motor_r_ohm;
            axis->nvs_motor_l_h = data.motor_l_h;
            axis->nvs_bandwidth_hz = data.bandwidth_hz;
        }
        if (err == ESP_FOC_OK) {
            char msg[72];
            (void)snprintf(
                msg, sizeof(msg),
                "calibration: saved to NVS (pole pairs=%d)",
                axis->motor_pole_pairs);
            tuner_log(msg);
        } else {
            tuner_log("calibration: save failed");
        }
        return err;
    }

    if (id == ESP_FOC_TUNER_CMD_LOAD_NVS) {
        esp_foc_calibration_data_t data;
        esp_foc_err_t err = esp_foc_calibration_load(axis->nvs_axis_id, &data);
        if (err != ESP_FOC_OK) {
            tuner_log("calibration: nothing to load");
            return err;
        }
        err = esp_foc_axis_set_current_pi_gains_q16(axis, data.kp, data.ki,
                                                    data.integrator_limit);
        if (err == ESP_FOC_OK && data.current_filter_fc_hz > 0 &&
            axis->isensor_driver != NULL &&
            axis->isensor_driver->set_filter_cutoff != NULL) {
            float fc = q16_to_float(data.current_filter_fc_hz);
            float fs = q16_to_float(axis->current_filter_fs_hz_q16);
            axis->isensor_driver->set_filter_cutoff(axis->isensor_driver,
                                                    fc, fs);
            axis->current_filter_fc_hz_q16 = data.current_filter_fc_hz;
        }
        if (err == ESP_FOC_OK) {
            axis->nvs_motor_r_ohm = data.motor_r_ohm;
            axis->nvs_motor_l_h = data.motor_l_h;
            axis->nvs_bandwidth_hz = data.bandwidth_hz;
            {
                int32_t ppn = esp_foc_calibration_get_pole_pairs(&data);
                if (ppn >= 1 && ppn <= 64) {
                    axis->motor_pole_pairs = (int)ppn;
                }
            }
            {
                uint8_t af;
                uint16_t eraw;
                q16_t nstore;
                esp_foc_calibration_get_align(&data, &af, &eraw, &nstore);
                const char *cw = (nstore == Q16_ONE)   ? "CW" :
                    (nstore == Q16_MINUS_ONE)         ? "CCW" : "?";
                ESP_LOGI(
                    tun_tag,
                    "LOAD_NVS axis %u: pole pairs=%d  R=%.4f ohm L=%.4e H "
                    "bw=%.1f  align: off=%d enc=%u dir=%d %s",
                    (unsigned)axis->nvs_axis_id,
                    axis->motor_pole_pairs,
                    (double)q16_to_float(data.motor_r_ohm),
                    (double)q16_to_float(data.motor_l_h),
                    (double)q16_to_float(data.bandwidth_hz),
                    (int)((af & ESP_FOC_CAL_ALIGN_FLAG_OFFSET) ? 1 : 0),
                    (unsigned)eraw,
                    (int)((af & ESP_FOC_CAL_ALIGN_FLAG_DIR) ? 1 : 0),
                    (af & ESP_FOC_CAL_ALIGN_FLAG_DIR) ? cw : "—");
            }
        }
        if (err == ESP_FOC_OK) {
            char tmsg[80];
            (void)snprintf(
                tmsg, sizeof(tmsg),
                "calibration: NVS applied (pole pairs=%d)",
                axis->motor_pole_pairs);
            tuner_log(tmsg);
        } else {
            tuner_log("calibration: apply failed");
        }
        return err;
    }

    if (id == ESP_FOC_TUNER_CMD_ERASE_NVS) {
        esp_foc_err_t err = esp_foc_calibration_erase();
        if (err == ESP_FOC_OK) {
            axis->nvs_motor_r_ohm = 0;
            axis->nvs_motor_l_h = 0;
            axis->nvs_bandwidth_hz = 0;
        }
        tuner_log(err == ESP_FOC_OK
                  ? "calibration: NVS namespace erased"
                  : "calibration: erase failed");
        return err;
    }

    if (id == ESP_FOC_TUNER_CMD_RESET_BOARD) {
        (void)axis;
        (void)payload;
        (void)payload_len;
        tuner_log("board: host requested reset (rebooting)");
        if (xTaskCreate(board_reset_task, "foc_host_rst", 3072, NULL,
                        tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
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

/* --- Reactor: link decoder + dispatcher ------------------------------- */

static esp_foc_link_decoder_t s_dec;
static bool s_dec_initialized = false;

void esp_foc_tuner_reactor_reset(void)
{
    esp_foc_link_decoder_reset(&s_dec);
    s_dec_initialized = true;
}

static void send_response_frame(uint8_t seq,
                                int8_t status,
                                const uint8_t *resp_payload,
                                size_t resp_len)
{
    /* Application-level response: [status:i8][seq:u8][payload]. The seq
     * is echoed so the host can correlate responses to requests. */
    uint8_t app[2 + ESP_FOC_LINK_MAX_PAYLOAD];
    if (resp_len > sizeof(app) - 2) {
        resp_len = sizeof(app) - 2;
    }
    app[0] = (uint8_t)status;
    app[1] = seq;
    if (resp_len > 0 && resp_payload != NULL) {
        memcpy(&app[2], resp_payload, resp_len);
    }

    uint8_t frame[ESP_FOC_LINK_MAX_FRAME];
    size_t frame_len = 0;
    if (esp_foc_link_encode(ESP_FOC_LINK_CH_TUNER, seq, app, 2 + resp_len,
                            frame, sizeof(frame), &frame_len) != ESP_FOC_LINK_OK) {
        return;
    }
    esp_foc_tuner_send_callback(frame, frame_len);
}

static void dispatch_frame(uint8_t seq,
                           const uint8_t *payload,
                           size_t payload_len)
{
    /* Application-level request: [op:u8][id:u16 LE][axis:u8][cmd_payload]. */
    if (payload_len < 4) {
        send_response_frame(seq, (int8_t)ESP_FOC_ERR_INVALID_ARG, NULL, 0);
        return;
    }
    esp_foc_tuner_op_t op = (esp_foc_tuner_op_t)payload[0];
    esp_foc_tuner_id_t id = (esp_foc_tuner_id_t)(payload[1] | ((uint16_t)payload[2] << 8));
    uint8_t axis_id = payload[3];
    const uint8_t *cmd_payload = payload + 4;
    size_t cmd_len = payload_len - 4;

    uint8_t resp_buf[8];
    size_t  resp_len = sizeof(resp_buf);
    esp_foc_err_t err = esp_foc_tuner_handle_request(axis_id, op, id,
                                                     cmd_payload, cmd_len,
                                                     resp_buf, &resp_len);
    if (err != ESP_FOC_OK) {
        resp_len = 0;
    }
    send_response_frame(seq, (int8_t)err, resp_buf, resp_len);
}

void esp_foc_tuner_process_byte(uint8_t byte)
{
    if (!s_dec_initialized) {
        esp_foc_tuner_reactor_reset();
    }
    esp_foc_link_status_t st = esp_foc_link_decoder_push(&s_dec, byte);
    if (st != ESP_FOC_LINK_OK) {
        return;
    }
    if (esp_foc_link_decoder_channel(&s_dec) == ESP_FOC_LINK_CH_TUNER) {
        dispatch_frame(esp_foc_link_decoder_seq(&s_dec),
                       esp_foc_link_decoder_payload(&s_dec),
                       esp_foc_link_decoder_payload_len(&s_dec));
    }
    /* Reset for the next frame. */
    esp_foc_link_decoder_reset(&s_dec);
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

__attribute__((weak)) uint32_t esp_foc_tuner_firmware_type(void)
{
    return ESP_FOC_TUNER_FIRMWARE_TYPE_GENERIC;
}
