/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#include "esp_foc_itl_plant.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FOC_ITL_TWO_PI  (2.0f * (float)M_PI)
#define FOC_ITL_INV_SQRT3  0.57735026919f

typedef struct {
    float id;
    float iq;
    float omega_m;
    float theta_m;
} esp_foc_itl_state_t;

typedef struct {
    float did;
    float diq;
    float domega_m;
    float dtheta_m;
} esp_foc_itl_deriv_t;

static float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static void abc_to_dq(float iu, float iv, float iw, float theta_e,
                      float *id, float *iq)
{
    const float c = cosf(theta_e);
    const float s = sinf(theta_e);
    const float ialpha = iu;
    const float ibeta = (iu + 2.0f * iv) * (1.0f / sqrtf(3.0f));
    *id = ialpha * c + ibeta * s;
    *iq = ibeta * c - ialpha * s;
    (void)iw;
}

static void dq_to_abc(float id, float iq, float theta_e,
                      float *iu, float *iv, float *iw)
{
    const float c = cosf(theta_e);
    const float s = sinf(theta_e);
    const float ialpha = id * c - iq * s;
    const float ibeta = id * s + iq * c;
    *iu = ialpha;
    *iv = -0.5f * ialpha + 0.86602540378f * ibeta;
    *iw = -(*iu + *iv);
}

static void plant_deriv(const esp_foc_itl_plant_t *plant, const esp_foc_itl_state_t *st,
                        float vu, float vv, float vw, esp_foc_itl_deriv_t *out)
{
    const esp_foc_itl_plant_params_t *p = &plant->p;
    const float pp = (float)p->pole_pairs;
    const float theta_e = pp * st->theta_m;
    const float omega_e = pp * st->omega_m;

    float vd;
    float vq;
    abc_to_dq(vu, vv, vw, theta_e, &vd, &vq);

    const float psi_f = p->kt_nm_per_a / (1.5f * pp);
    const float inv_l = 1.0f / p->l_henry;

    out->did = inv_l * (vd - p->r_ohm * st->id + omega_e * p->l_henry * st->iq);
    out->diq = inv_l * (vq - p->r_ohm * st->iq - omega_e * p->l_henry * st->id
                        - omega_e * psi_f);

    if (p->locked_rotor) {
        out->domega_m = 0.0f;
        out->dtheta_m = 0.0f;
    } else {
        const float te = p->kt_nm_per_a * st->iq;
        out->domega_m = (te - p->b_nms * st->omega_m) / p->j_kgm2;
        out->dtheta_m = st->omega_m;
    }
}

static void plant_apply_saturation(esp_foc_itl_plant_t *plant)
{
    dq_to_abc(plant->id_a, plant->iq_a,
              (float)plant->p.pole_pairs * plant->theta_m_rad,
              &plant->iu_a, &plant->iv_a, &plant->iw_a);

    const float lim = plant->p.i_max_a;
    plant->iu_a = clampf(plant->iu_a, -lim, lim);
    plant->iv_a = clampf(plant->iv_a, -lim, lim);
    plant->iw_a = clampf(plant->iw_a, -lim, lim);

    abc_to_dq(plant->iu_a, plant->iv_a, plant->iw_a,
              (float)plant->p.pole_pairs * plant->theta_m_rad,
              &plant->id_a, &plant->iq_a);
}

static void rk_add_state(const esp_foc_itl_state_t *a, const esp_foc_itl_deriv_t *b,
                         float scale, esp_foc_itl_state_t *out)
{
    out->id = a->id + scale * b->did;
    out->iq = a->iq + scale * b->diq;
    out->omega_m = a->omega_m + scale * b->domega_m;
    out->theta_m = a->theta_m + scale * b->dtheta_m;
}

#if defined(CONFIG_FOC_ITL_USE_RK2)
static void plant_integrate(esp_foc_itl_plant_t *plant, float vu, float vv, float vw,
                            float dt_s)
{
    esp_foc_itl_state_t st = {
        .id = plant->id_a,
        .iq = plant->iq_a,
        .omega_m = plant->omega_m_rad_s,
        .theta_m = plant->theta_m_rad,
    };
    esp_foc_itl_deriv_t k1;
    esp_foc_itl_deriv_t k2;
    esp_foc_itl_state_t mid;

    plant_deriv(plant, &st, vu, vv, vw, &k1);
    rk_add_state(&st, &k1, 0.5f * dt_s, &mid);
    plant_deriv(plant, &mid, vu, vv, vw, &k2);

    plant->id_a = st.id + dt_s * k2.did;
    plant->iq_a = st.iq + dt_s * k2.diq;
    if (plant->p.locked_rotor) {
        plant->omega_m_rad_s = 0.0f;
        plant->theta_m_rad = st.theta_m;
    } else {
        plant->omega_m_rad_s = st.omega_m + dt_s * k2.domega_m;
        plant->theta_m_rad = st.theta_m + dt_s * k2.dtheta_m;
    }
}
#else
static void plant_integrate(esp_foc_itl_plant_t *plant, float vu, float vv, float vw,
                            float dt_s)
{
    esp_foc_itl_state_t st = {
        .id = plant->id_a,
        .iq = plant->iq_a,
        .omega_m = plant->omega_m_rad_s,
        .theta_m = plant->theta_m_rad,
    };
    esp_foc_itl_deriv_t k1;
    esp_foc_itl_deriv_t k2;
    esp_foc_itl_deriv_t k3;
    esp_foc_itl_deriv_t k4;
    esp_foc_itl_state_t s2;
    esp_foc_itl_state_t s3;
    esp_foc_itl_state_t s4;

    plant_deriv(plant, &st, vu, vv, vw, &k1);
    rk_add_state(&st, &k1, 0.5f * dt_s, &s2);
    plant_deriv(plant, &s2, vu, vv, vw, &k2);
    rk_add_state(&st, &k2, 0.5f * dt_s, &s3);
    plant_deriv(plant, &s3, vu, vv, vw, &k3);
    rk_add_state(&st, &k3, dt_s, &s4);
    plant_deriv(plant, &s4, vu, vv, vw, &k4);

    plant->id_a = st.id + (dt_s / 6.0f) * (k1.did + 2.0f * k2.did + 2.0f * k3.did + k4.did);
    plant->iq_a = st.iq + (dt_s / 6.0f) * (k1.diq + 2.0f * k2.diq + 2.0f * k3.diq + k4.diq);
    if (plant->p.locked_rotor) {
        plant->omega_m_rad_s = 0.0f;
        plant->theta_m_rad = st.theta_m;
    } else {
        plant->omega_m_rad_s = st.omega_m
            + (dt_s / 6.0f) * (k1.domega_m + 2.0f * k2.domega_m + 2.0f * k3.domega_m + k4.domega_m);
        plant->theta_m_rad = st.theta_m
            + (dt_s / 6.0f) * (k1.dtheta_m + 2.0f * k2.dtheta_m + 2.0f * k3.dtheta_m + k4.dtheta_m);
    }
}
#endif

void esp_foc_itl_plant_init(esp_foc_itl_plant_t *plant, const esp_foc_itl_plant_params_t *params)
{
    memset(plant, 0, sizeof(*plant));
    if (params != NULL) {
        plant->p = *params;
    }
    esp_foc_itl_plant_reset_parked(plant);
}

void esp_foc_itl_plant_reset_parked(esp_foc_itl_plant_t *plant)
{
    plant->id_a = 0.0f;
    plant->iq_a = 0.0f;
    plant->omega_m_rad_s = 0.0f;
    plant->theta_m_rad = 0.0f;
    plant->iu_a = 0.0f;
    plant->iv_a = 0.0f;
    plant->iw_a = 0.0f;
}

void esp_foc_itl_plant_duties_to_phase_volts(float duty_u, float duty_v, float duty_w,
                                          const esp_foc_itl_plant_t *plant,
                                          float *vu, float *vv, float *vw)
{
    const float vdc = plant->p.vdc_volts;
    const float half = 0.5f * vdc;

    *vu = (duty_u - 0.5f) * vdc;
    *vv = (duty_v - 0.5f) * vdc;
    *vw = (duty_w - 0.5f) * vdc;

    if (plant->p.delta_connection) {
        *vu *= FOC_ITL_INV_SQRT3;
        *vv *= FOC_ITL_INV_SQRT3;
        *vw *= FOC_ITL_INV_SQRT3;
    }

    *vu = clampf(*vu, -half, half);
    *vv = clampf(*vv, -half, half);
    *vw = clampf(*vw, -half, half);
}

void esp_foc_itl_plant_step(esp_foc_itl_plant_t *plant, float vu, float vv, float vw, float dt_s)
{
    if (plant == NULL || dt_s <= 0.0f) {
        return;
    }

    plant_integrate(plant, vu, vv, vw, dt_s);
    plant_apply_saturation(plant);

    if (!plant->p.locked_rotor) {
        if (plant->theta_m_rad >= FOC_ITL_TWO_PI) {
            plant->theta_m_rad = fmodf(plant->theta_m_rad, FOC_ITL_TWO_PI);
        } else if (plant->theta_m_rad < 0.0f) {
            plant->theta_m_rad = fmodf(plant->theta_m_rad, FOC_ITL_TWO_PI) + FOC_ITL_TWO_PI;
        }
    }
}

int32_t esp_foc_itl_plant_encoder_ticks(const esp_foc_itl_plant_t *plant)
{
    if (plant == NULL) {
        return 0;
    }
    const float turns = plant->theta_m_rad / FOC_ITL_TWO_PI;
    int32_t ticks = (int32_t)lroundf(turns * (float)ESP_FOC_ITL_CPR);
    while (ticks < 0) {
        ticks += (int32_t)ESP_FOC_ITL_CPR;
    }
    while (ticks >= (int32_t)ESP_FOC_ITL_CPR) {
        ticks -= (int32_t)ESP_FOC_ITL_CPR;
    }
    return ticks;
}
