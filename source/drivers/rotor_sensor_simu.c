/*
 * MIT License
 */
#include <math.h>
#include <string.h>
#include <stddef.h>
#include "sdkconfig.h"
#include "espFoC/rotor_sensor_simu.h"
#include "espFoC/osal/os_interface.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "esp_log.h"

#define CPR_SIMU 4096u
#define CPR_MASK 0x0FFFu
#define TWO_PI_F (2.0f * 3.14159265f)
#define INV_SQRT3_F (0.57735027f)

static const char *TAG = "ROTOR_SIMU";

typedef struct {
    esp_foc_rotor_sensor_t interface;
    q16_t *u_d;
    q16_t *u_q;
    q16_t vdc_q16;
    int pole_pairs;
    float r_ohm;
    float l_henry;
    float dt_outer_s;
    float v_line_volts;
    float j_kgm2;
    float b_nms;
    float kt_nm_per_a;
    float id_a;
    float iq_a;
    float theta_m_rad;
    float omega_m_rad_s;
    int64_t accum_ticks_i64;
} esp_foc_rotor_simu_t;

static esp_foc_rotor_simu_t s_sensors[CONFIG_NOOF_AXIS];

static esp_foc_rotor_simu_t *sim_from_iface(esp_foc_rotor_sensor_t *self)
{
    return __containerof(self, esp_foc_rotor_simu_t, interface);
}

static void set_to_zero(esp_foc_rotor_sensor_t *self)
{
    esp_foc_rotor_simu_t *o = sim_from_iface(self);
    o->id_a = 0.0f;
    o->iq_a = 0.0f;
    o->theta_m_rad = 0.0f;
    o->omega_m_rad_s = 0.0f;
    o->accum_ticks_i64 = 0;
    ESP_LOGI(TAG, "sim rotor: mechanical + electrical state cleared");
}

static uint32_t get_counts_per_revolution(esp_foc_rotor_sensor_t *self)
{
    (void)self;
    return CPR_SIMU;
}

static void sim_step(esp_foc_rotor_simu_t *o, q16_t ud_raw, q16_t uq_raw)
{
    float ud = q16_to_float(ud_raw);
    float uq = q16_to_float(uq_raw);
    float vd = ud * o->v_line_volts;
    float vq = uq * o->v_line_volts;
    float we = (float)o->pole_pairs * o->omega_m_rad_s;
    float L = o->l_henry;
    float R = o->r_ohm;
    float dt = o->dt_outer_s;
    if (L < 1e-9f || R < 1e-9f || dt < 1e-12f) {
        return;
    }
    float did = (dt / L) * (vd - R * o->id_a + we * L * o->iq_a);
    float diq = (dt / L) * (vq - R * o->iq_a - we * L * o->id_a);
    o->id_a += did;
    o->iq_a += diq;
    float te = o->kt_nm_per_a * o->iq_a;
    float wdot = (te - o->b_nms * o->omega_m_rad_s) / o->j_kgm2;
    o->omega_m_rad_s += wdot * dt;
    o->theta_m_rad += o->omega_m_rad_s * dt;
    while (o->theta_m_rad >= TWO_PI_F) {
        o->theta_m_rad -= TWO_PI_F;
        o->accum_ticks_i64 += (int64_t)CPR_SIMU;
    }
    while (o->theta_m_rad < 0.0f) {
        o->theta_m_rad += TWO_PI_F;
        o->accum_ticks_i64 -= (int64_t)CPR_SIMU;
    }
}

static q16_t read_counts(esp_foc_rotor_sensor_t *self)
{
    esp_foc_rotor_simu_t *o = sim_from_iface(self);
    q16_t ud = 0;
    q16_t uq = 0;
    if (o->u_d != NULL && o->u_q != NULL) {
        esp_foc_critical_enter();
        ud = *o->u_d;
        uq = *o->u_q;
        esp_foc_critical_leave();
        sim_step(o, ud, uq);
    }
    int32_t tick = (int32_t)((o->theta_m_rad / TWO_PI_F) * (float)CPR_SIMU);
    tick &= (int32_t)CPR_MASK;
    return q16_from_int(tick);
}

static int64_t read_accumulated_counts_i64(esp_foc_rotor_sensor_t *self)
{
    esp_foc_rotor_simu_t *o = sim_from_iface(self);
    int32_t tick = (int32_t)((o->theta_m_rad / TWO_PI_F) * (float)CPR_SIMU);
    tick &= (int32_t)CPR_MASK;
    return o->accum_ticks_i64 + (int64_t)tick;
}

static void set_simulation_count(esp_foc_rotor_sensor_t *self, q16_t increment_normalized)
{
    esp_foc_rotor_simu_t *o = sim_from_iface(self);
    int64_t dt = ((int64_t)increment_normalized * (int64_t)CPR_SIMU) >> 31;
    o->accum_ticks_i64 += dt;
    float dth = ((float)dt / (float)CPR_SIMU) * TWO_PI_F;
    o->theta_m_rad += dth;
    while (o->theta_m_rad >= TWO_PI_F) {
        o->theta_m_rad -= TWO_PI_F;
    }
    while (o->theta_m_rad < 0.0f) {
        o->theta_m_rad += TWO_PI_F;
    }
}

void rotor_sensor_simu_wire_ud_uq(esp_foc_rotor_sensor_t *self,
                                  q16_t *ud_raw,
                                  q16_t *uq_raw)
{
    if (self == NULL) {
        return;
    }
    esp_foc_rotor_simu_t *o = sim_from_iface(self);
    o->u_d = ud_raw;
    o->u_q = uq_raw;
}

esp_foc_rotor_sensor_t *rotor_sensor_simu_new(int port,
                                              int motor_pole_pairs,
                                              float r_ohm,
                                              float l_henry,
                                              q16_t vdc_q16)
{
    if (port < 0 || port >= CONFIG_NOOF_AXIS) {
        ESP_LOGE(TAG, "invalid port %d", port);
        return NULL;
    }
    if (motor_pole_pairs < 1 || motor_pole_pairs > 64) {
        ESP_LOGE(TAG, "invalid pole_pairs %d", motor_pole_pairs);
        return NULL;
    }
    if (r_ohm < 1e-6f || l_henry < 1e-9f) {
        ESP_LOGE(TAG, "invalid R or L");
        return NULL;
    }

    esp_foc_rotor_simu_t *o = &s_sensors[port];
    memset(o, 0, sizeof(*o));
    o->interface.set_to_zero = set_to_zero;
    o->interface.get_counts_per_revolution = get_counts_per_revolution;
    o->interface.read_counts = read_counts;
    o->interface.read_accumulated_counts_i64 = read_accumulated_counts_i64;
    o->interface.set_simulation_count = set_simulation_count;
    o->interface.set_zero_offset_raw_12b = NULL;
    o->interface.get_zero_offset_12b = NULL;

    o->pole_pairs = motor_pole_pairs;
    o->r_ohm = r_ohm;
    o->l_henry = l_henry;
    o->vdc_q16 = vdc_q16;
    o->v_line_volts = q16_to_float(vdc_q16) * INV_SQRT3_F;

    float pwm_hz = (float)CONFIG_ESP_FOC_PWM_RATE_HZ;
    o->dt_outer_s = (1.0f / pwm_hz) * (float)CONFIG_ESP_FOC_LOW_SPEED_DOWNSAMPLING;

    o->j_kgm2 = (float)CONFIG_ESP_FOC_ROTOR_SIMU_DEFAULT_J_X1E7 * 1e-7f;
    if (o->j_kgm2 < 1e-9f) {
        o->j_kgm2 = 1e-7f;
    }
    o->b_nms = (float)CONFIG_ESP_FOC_ROTOR_SIMU_DEFAULT_B_X1E7 * 1e-7f;
    o->kt_nm_per_a = (float)CONFIG_ESP_FOC_ROTOR_SIMU_DEFAULT_KT_X1E7 * 1e-7f;
    if (o->kt_nm_per_a < 1e-9f) {
        o->kt_nm_per_a = 1e-7f;
    }

    ESP_LOGI(TAG,
             "port=%d pp=%d R=%.4f L=%.6f Vdc_q16=%ld dt_outer=%.6f s",
             port, motor_pole_pairs, r_ohm, l_henry, (long)vdc_q16,
             (double)o->dt_outer_s);

    return &o->interface;
}
