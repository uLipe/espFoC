/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Discretized PMSM plant for FOC-in-the-loop simulation.
 *
 * Continuous model (dq frame, SI parameters mapped to Q16 at init):
 *   L·di_d/dt = v_d − R·i_d + ω_e·L·i_q
 *   L·di_q/dt = v_q − R·i_q − ω_e·L·i_d − ω_e·ψ_f
 *   J·dω_m/dt = k_t·i_q − b·ω_m        (free rotor; locked ⇒ ω_m = θ_m = 0)
 *   dθ_m/dt   = ω_m                     (θ_m in turns, ω_m in rev/s)
 *
 * Terminal abc voltages enter via Clarke + Park; phase currents leave via
 * inverse transforms. Time stepping: RK4 (default) or RK2 (Kconfig).
 */
#include "esp_foc_itl_plant.h"

#include <string.h>

#include "espFoC/utils/foc_math_q16.h"

#define ESP_FOC_ITL_INV_SQRT3_Q16  ((q16_t)37837)

typedef struct {
    q16_t id;
    q16_t iq;
    q16_t omega_m;
    q16_t theta_m;
} esp_foc_itl_state_q16_t;

typedef struct {
    q16_t did;
    q16_t diq;
    q16_t domega_m;
    q16_t dtheta_m;
} esp_foc_itl_deriv_q16_t;

static void elec_sin_cos_q16(const esp_foc_itl_plant_t *plant, q16_t theta_m,
                               q16_t *sin_e, q16_t *cos_e)
{
    /* θ_e = p·θ_m (mod 1 turn) for Park alignment with electrical angle. */
    q16_t theta_e = q16_mul(theta_m, q16_from_int(plant->c.pole_pairs));
    theta_e = q16_normalize_angle(theta_e);
    *sin_e = q16_sin(theta_e);
    *cos_e = q16_cos(theta_e);
}

static void abc_to_dq_q16(q16_t iu, q16_t iv, q16_t iw, q16_t sin_e, q16_t cos_e,
                          q16_t *id, q16_t *iq)
{
    q16_t alpha;
    q16_t beta;
    q16_clarke(iu, iv, iw, &alpha, &beta);
    q16_park(sin_e, cos_e, alpha, beta, id, iq);
}

static void dq_to_abc_q16(q16_t id, q16_t iq, q16_t sin_e, q16_t cos_e,
                          q16_t *iu, q16_t *iv, q16_t *iw)
{
    q16_t alpha;
    q16_t beta;
    q16_inverse_park(sin_e, cos_e, id, iq, &alpha, &beta);
    q16_inverse_clarke(alpha, beta, iu, iv, iw);
}

static void plant_deriv_q16(const esp_foc_itl_plant_t *plant, const esp_foc_itl_state_q16_t *st,
                            q16_t vu, q16_t vv, q16_t vw, esp_foc_itl_deriv_q16_t *out)
{
    const esp_foc_itl_plant_coeff_t *c = &plant->c;
    q16_t sin_e;
    q16_t cos_e;
    q16_t vd;
    q16_t vq;
    q16_t omega_e;

    /* v_dq from terminal abc (same basis the real inverter drives). */
    elec_sin_cos_q16(plant, st->theta_m, &sin_e, &cos_e);
    abc_to_dq_q16(vu, vv, vw, sin_e, cos_e, &vd, &vq);

    omega_e = q16_mul(st->omega_m, q16_from_int(c->pole_pairs));

    /* d-axis: L·di_d/dt = v_d − R·i_d + ω_e·L·i_q (cross-coupling from rotation). */
    {
        q16_t back_emf = q16_mul(omega_e, c->l_q16);
        back_emf = q16_mul(back_emf, st->iq);
        q16_t num = q16_add(q16_sub(vd, q16_mul(c->r_q16, st->id)), back_emf);
        out->did = q16_mul(c->inv_l_q16, num);
    }
    /* q-axis: L·di_q/dt = v_q − R·i_q − ω_e·L·i_d − ω_e·ψ_f (PM back-EMF term). */
    {
        q16_t cross = q16_mul(omega_e, c->l_q16);
        cross = q16_mul(cross, st->id);
        q16_t emf = q16_mul(omega_e, c->psi_f_q16);
        q16_t num = q16_sub(vq, q16_mul(c->r_q16, st->iq));
        num = q16_sub(num, cross);
        num = q16_sub(num, emf);
        out->diq = q16_mul(c->inv_l_q16, num);
    }

    if (c->locked_rotor) {
        out->domega_m = 0;
        out->dtheta_m = 0;
    } else {
        /* J·dω_m/dt = T_e − b·ω_m with T_e ≈ k_t·i_q; dθ_m/dt = ω_m (turns/s). */
        q16_t te = q16_mul(c->kt_q16, st->iq);
        q16_t drag = q16_mul(c->b_q16, st->omega_m);
        out->domega_m = q16_mul(c->inv_j_q16, q16_sub(te, drag));
        out->dtheta_m = q16_mul(st->omega_m, Q16_INV_TWO_PI);
    }
}

static void rk_add_state_q16(const esp_foc_itl_state_q16_t *a, const esp_foc_itl_deriv_q16_t *b,
                             q16_t scale, esp_foc_itl_state_q16_t *out)
{
    out->id = q16_add(a->id, q16_mul(scale, b->did));
    out->iq = q16_add(a->iq, q16_mul(scale, b->diq));
    out->omega_m = q16_add(a->omega_m, q16_mul(scale, b->domega_m));
    out->theta_m = q16_add(a->theta_m, q16_mul(scale, b->dtheta_m));
}

static void plant_apply_saturation_q16(esp_foc_itl_plant_t *plant)
{
    q16_t sin_e;
    q16_t cos_e;
    q16_t iw;

    /* |i_u|, |i_v| clamp then re-Park; enforces i_max without breaking KCL on i_w. */
    elec_sin_cos_q16(plant, plant->theta_m_q16, &sin_e, &cos_e);
    dq_to_abc_q16(plant->id_q16, plant->iq_q16, sin_e, cos_e,
                  &plant->iu_q16, &plant->iv_q16, &iw);

    plant->iu_q16 = q16_clamp(plant->iu_q16, q16_sub(0, plant->c.i_max_q16), plant->c.i_max_q16);
    plant->iv_q16 = q16_clamp(plant->iv_q16, q16_sub(0, plant->c.i_max_q16), plant->c.i_max_q16);

    abc_to_dq_q16(plant->iu_q16, plant->iv_q16, q16_sub(0, q16_add(plant->iu_q16, plant->iv_q16)),
                  sin_e, cos_e, &plant->id_q16, &plant->iq_q16);
}

#if defined(CONFIG_FOC_ITL_USE_RK2)
static void plant_integrate_q16(esp_foc_itl_plant_t *plant, q16_t vu, q16_t vv, q16_t vw)
{
    const esp_foc_itl_plant_coeff_t *c = &plant->c;
    esp_foc_itl_state_q16_t st = {
        .id = plant->id_q16,
        .iq = plant->iq_q16,
        .omega_m = plant->omega_m_q16,
        .theta_m = plant->theta_m_q16,
    };
    esp_foc_itl_deriv_q16_t k1;
    esp_foc_itl_deriv_q16_t k2;
    esp_foc_itl_state_q16_t mid;

    /* RK2 (midpoint): x_{n+1} = x_n + dt·f(x_n + 0.5·dt·f(x_n)). */
    plant_deriv_q16(plant, &st, vu, vv, vw, &k1);
    rk_add_state_q16(&st, &k1, c->dt_half_q16, &mid);
    plant_deriv_q16(plant, &mid, vu, vv, vw, &k2);

    plant->id_q16 = q16_add(st.id, q16_mul(c->dt_q16, k2.did));
    plant->iq_q16 = q16_add(st.iq, q16_mul(c->dt_q16, k2.diq));
    if (c->locked_rotor) {
        plant->omega_m_q16 = 0;
        plant->theta_m_q16 = st.theta_m;
    } else {
        plant->omega_m_q16 = q16_add(st.omega_m, q16_mul(c->dt_q16, k2.domega_m));
        plant->theta_m_q16 = q16_add(st.theta_m, q16_mul(c->dt_q16, k2.dtheta_m));
    }
}
#else
static void plant_integrate_q16(esp_foc_itl_plant_t *plant, q16_t vu, q16_t vv, q16_t vw)
{
    const esp_foc_itl_plant_coeff_t *c = &plant->c;
    esp_foc_itl_state_q16_t st = {
        .id = plant->id_q16,
        .iq = plant->iq_q16,
        .omega_m = plant->omega_m_q16,
        .theta_m = plant->theta_m_q16,
    };
    esp_foc_itl_deriv_q16_t k1;
    esp_foc_itl_deriv_q16_t k2;
    esp_foc_itl_deriv_q16_t k3;
    esp_foc_itl_deriv_q16_t k4;
    esp_foc_itl_state_q16_t s2;
    esp_foc_itl_state_q16_t s3;
    esp_foc_itl_state_q16_t s4;
    q16_t two;

    /* RK4: x_{n+1} = x_n + (dt/6)·(k1 + 2k2 + 2k3 + k4). */
    plant_deriv_q16(plant, &st, vu, vv, vw, &k1);
    rk_add_state_q16(&st, &k1, c->dt_half_q16, &s2);
    plant_deriv_q16(plant, &s2, vu, vv, vw, &k2);
    rk_add_state_q16(&st, &k2, c->dt_half_q16, &s3);
    plant_deriv_q16(plant, &s3, vu, vv, vw, &k3);
    rk_add_state_q16(&st, &k3, c->dt_q16, &s4);
    plant_deriv_q16(plant, &s4, vu, vv, vw, &k4);

    two = q16_from_int(2);
    plant->id_q16 = q16_add(st.id,
                            q16_mul(c->dt_over_6_q16,
                                    q16_add(k1.did,
                                            q16_add(q16_mul(two, k2.did),
                                                    q16_add(q16_mul(two, k3.did), k4.did)))));
    plant->iq_q16 = q16_add(st.iq,
                            q16_mul(c->dt_over_6_q16,
                                    q16_add(k1.diq,
                                            q16_add(q16_mul(two, k2.diq),
                                                    q16_add(q16_mul(two, k3.diq), k4.diq)))));
    if (c->locked_rotor) {
        plant->omega_m_q16 = 0;
        plant->theta_m_q16 = st.theta_m;
    } else {
        q16_t dom = q16_add(k1.domega_m,
                            q16_add(q16_mul(two, k2.domega_m),
                                    q16_add(q16_mul(two, k3.domega_m), k4.domega_m)));
        q16_t dth = q16_add(k1.dtheta_m,
                            q16_add(q16_mul(two, k2.dtheta_m),
                                    q16_add(q16_mul(two, k3.dtheta_m), k4.dtheta_m)));
        plant->omega_m_q16 = q16_add(st.omega_m, q16_mul(c->dt_over_6_q16, dom));
        plant->theta_m_q16 = q16_add(st.theta_m, q16_mul(c->dt_over_6_q16, dth));
    }
}
#endif

static void plant_build_coeff(esp_foc_itl_plant_t *plant, const esp_foc_itl_plant_params_t *params)
{
    const float pp = (float)params->pole_pairs;

    plant->c.locked_rotor = params->locked_rotor;
    plant->c.delta_connection = params->delta_connection;
    plant->c.pole_pairs = params->pole_pairs;
    plant->c.r_q16 = q16_from_float(params->r_ohm);
    plant->c.l_q16 = q16_from_float(params->l_henry);
    plant->c.inv_l_q16 = q16_reciprocal_positive(plant->c.l_q16);
    /* ψ_f = k_t / (1.5·p): surface PM torque T_e = 1.5·p·ψ_f·i_q. */
    plant->c.psi_f_q16 = q16_from_float(params->kt_nm_per_a / (1.5f * pp));
    plant->c.kt_q16 = q16_from_float(params->kt_nm_per_a);
    plant->c.b_q16 = q16_from_float(params->b_nms);
    plant->c.j_q16 = q16_from_float(params->j_kgm2);
    plant->c.inv_j_q16 = q16_reciprocal_positive(plant->c.j_q16);
    plant->c.i_max_q16 = q16_from_float(params->i_max_a);
    plant->c.vdc_q16 = q16_from_float(params->vdc_volts);
    plant->c.half_vdc_q16 = q16_mul(plant->c.vdc_q16, Q16_HALF);
    /* Delta: line-neutral drive scaled by 1/√3 to match wye-equivalent phase voltage. */
    plant->c.delta_scale_q16 = params->delta_connection ? ESP_FOC_ITL_INV_SQRT3_Q16 : Q16_ONE;
}

void esp_foc_itl_plant_init(esp_foc_itl_plant_t *plant, const esp_foc_itl_plant_params_t *params)
{
    memset(plant, 0, sizeof(*plant));
    if (params != NULL) {
        plant_build_coeff(plant, params);
    }
    esp_foc_itl_plant_reset_parked(plant);
}

void esp_foc_itl_plant_set_dt(esp_foc_itl_plant_t *plant, uint32_t pwm_hz)
{
    if (plant == NULL || pwm_hz == 0u) {
        return;
    }
    /* dt matches plant task rate (ISR rate / downsample when FITL decimates). */
    plant->c.dt_q16 = q16_from_float(1.0f / (float)pwm_hz);
    plant->c.dt_half_q16 = q16_mul(plant->c.dt_q16, Q16_HALF);
    plant->c.dt_over_6_q16 = q16_mul(plant->c.dt_q16, q16_from_float(1.0f / 6.0f));
}

void esp_foc_itl_plant_reset_parked(esp_foc_itl_plant_t *plant)
{
    plant->id_q16 = 0;
    plant->iq_q16 = 0;
    plant->omega_m_q16 = 0;
    plant->theta_m_q16 = 0;
    plant->iu_q16 = 0;
    plant->iv_q16 = 0;
}

void esp_foc_itl_plant_duties_to_phase_volts(q16_t duty_u, q16_t duty_v, q16_t duty_w,
                                              const esp_foc_itl_plant_t *plant,
                                              q16_t *vu, q16_t *vv, q16_t *vw)
{
    const esp_foc_itl_plant_coeff_t *c = &plant->c;

    /* SVPWM midpoint: v_phase ≈ (duty − 0.5)·V_dc, clamped to ±V_dc/2. */
    *vu = q16_mul(q16_sub(duty_u, Q16_HALF), c->vdc_q16);
    *vv = q16_mul(q16_sub(duty_v, Q16_HALF), c->vdc_q16);
    *vw = q16_mul(q16_sub(duty_w, Q16_HALF), c->vdc_q16);

    if (c->delta_connection) {
        *vu = q16_mul(*vu, c->delta_scale_q16);
        *vv = q16_mul(*vv, c->delta_scale_q16);
        *vw = q16_mul(*vw, c->delta_scale_q16);
    }

    *vu = q16_clamp(*vu, q16_sub(0, c->half_vdc_q16), c->half_vdc_q16);
    *vv = q16_clamp(*vv, q16_sub(0, c->half_vdc_q16), c->half_vdc_q16);
    *vw = q16_clamp(*vw, q16_sub(0, c->half_vdc_q16), c->half_vdc_q16);
}

void esp_foc_itl_plant_step(esp_foc_itl_plant_t *plant, q16_t vu, q16_t vv, q16_t vw)
{
    if (plant == NULL || plant->c.dt_q16 <= 0) {
        return;
    }

    plant_integrate_q16(plant, vu, vv, vw);
    plant_apply_saturation_q16(plant);

    if (!plant->c.locked_rotor) {
        plant->theta_m_q16 = q16_normalize_angle(plant->theta_m_q16);
    }
}

int32_t esp_foc_itl_plant_encoder_ticks(const esp_foc_itl_plant_t *plant)
{
    if (plant == NULL) {
        return 0;
    }
    int64_t ticks = ((int64_t)plant->theta_m_q16 * (int64_t)ESP_FOC_ITL_CPR) >> Q16_FRAC_BITS;
    int32_t t = (int32_t)ticks;
    while (t < 0) {
        t += (int32_t)ESP_FOC_ITL_CPR;
    }
    while (t >= (int32_t)ESP_FOC_ITL_CPR) {
        t -= (int32_t)ESP_FOC_ITL_CPR;
    }
    return t;
}
