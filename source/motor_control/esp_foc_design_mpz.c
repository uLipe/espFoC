/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include "espFoC/utils/esp_foc_design_mpz.h"

/* --- Internal helpers --------------------------------------------------- */

/* Q16 division a/b with int64 intermediate; saturates on overflow.
 * Caller must ensure b > 0. */
static inline q16_t q16_div_internal(q16_t a, q16_t b)
{
    int64_t num = ((int64_t)a) << Q16_FRAC_BITS;
    if (num >= 0) {
        num += b / 2;
    } else {
        num -= b / 2;
    }
    int64_t r = num / (int64_t)b;
    if (r > (int64_t)INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    if (r < (int64_t)INT32_MIN) {
        return (q16_t)INT32_MIN;
    }
    return (q16_t)r;
}

/* Pade [3/3] approximation of exp(-x) for x in [0, 1]:
 *   exp(-x) ~= (1 - x/2 + x^2/10 - x^3/120) /
 *             (1 + x/2 + x^2/10 + x^3/120)
 * Relative error stays below ~5e-5 in the reduction interval; outside it
 * we fall back on range reduction in esp_foc_q16_exp_neg.
 */
static q16_t pade33_exp_neg(q16_t x)
{
    /* Constants in Q16 */
    static const q16_t INV_10  = (q16_t)6554;   /* round(0.1   * 65536) */
    static const q16_t INV_120 = (q16_t)546;    /* round(1/120 * 65536) */

    q16_t x_half = x >> 1;
    q16_t x2 = q16_mul(x, x);
    q16_t x3 = q16_mul(x2, x);

    q16_t x2_term  = q16_mul(x2, INV_10);
    q16_t x3_term  = q16_mul(x3, INV_120);

    q16_t num = q16_add(Q16_ONE, q16_sub(x2_term, q16_add(x_half, x3_term)));
    q16_t den = q16_add(Q16_ONE, q16_add(x_half, q16_add(x2_term, x3_term)));

    if (den <= 0) {
        return 0;
    }
    return q16_div_internal(num, den);
}

/* --- Public API --------------------------------------------------------- */

q16_t esp_foc_q16_exp_neg(q16_t x)
{
    if (x <= 0) {
        return Q16_ONE;
    }

    /* Beyond ~22.18 (=ln(2^32)/ln(e) ~ 22.18) exp(-x) underflows below 1 LSB. */
    if (x >= q16_from_int(23)) {
        return 0;
    }

    /* Range reduction by halving until argument is well inside [0, 1].
     * Each halving requires a final squaring (exp(-x) = exp(-x/2)^2).
     * Threshold 0.5 keeps Pade error below 1e-6. */
    int reductions = 0;
    q16_t y = x;
    while (y > Q16_HALF) {
        y >>= 1;
        reductions++;
        if (reductions > 10) {
            /* Defensive: 10 halvings cover up to x = 1024 in Q16 (>> our 23 cap). */
            break;
        }
    }

    q16_t result = pade33_exp_neg(y);
    while (reductions-- > 0) {
        result = q16_mul(result, result);
    }
    return result;
}

/* Compute exp(-R*Ts/L) where Ts is given in microseconds.
 * Avoids quantizing Ts into Q16 seconds (~15us LSB) which destroys precision
 * for fast loops. Stays in int64 throughout.
 *   arg_q16 = (R[Q16] / L[Q16]) * (ts_us / 1e6)  in Q16
 *           = ((R<<16)/L) * ts_us / 1e6
 */
static q16_t compute_exp_arg_alpha(q16_t r_q16, q16_t l_q16, uint32_t ts_us)
{
    int64_t r_over_l_q16 = ((int64_t)r_q16 << Q16_FRAC_BITS) / (int64_t)l_q16;
    int64_t raw = r_over_l_q16 * (int64_t)ts_us + 500000;
    raw /= 1000000;
    if (raw < 0) {
        raw = 0;
    }
    if (raw > (int64_t)INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    return (q16_t)raw;
}

/* Compute exp argument for beta = exp(-w_bw*Ts) with Ts in microseconds.
 *   arg_q16 = w_bw[Q16] * ts_us / 1e6
 */
static q16_t compute_exp_arg_beta(q16_t omega_bw_q16, uint32_t ts_us)
{
    int64_t raw = (int64_t)omega_bw_q16 * (int64_t)ts_us + 500000;
    raw /= 1000000;
    if (raw < 0) {
        raw = 0;
    }
    if (raw > (int64_t)INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    return (q16_t)raw;
}

/* Ki = R*(1-beta)/Ts with Ts in microseconds.
 *   ki_q16 = num_q16[V/A] * 1e6 / ts_us  → V/(A*s) in Q16
 */
static q16_t compute_ki_from_ts_us(q16_t numerator_q16, uint32_t ts_us)
{
    int64_t raw = (int64_t)numerator_q16 * 1000000LL + (int64_t)ts_us / 2;
    raw /= (int64_t)ts_us;
    if (raw > (int64_t)INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    if (raw < (int64_t)INT32_MIN) {
        return (q16_t)INT32_MIN;
    }
    return (q16_t)raw;
}

esp_foc_design_status_t esp_foc_design_pi_current_mpz_q16(
    const esp_foc_pi_design_input_t *in,
    esp_foc_pi_design_output_t *out)
{
    if (in == NULL || out == NULL) {
        return ESP_FOC_DESIGN_ERR_INVALID_ARG;
    }

    out->valid = false;
    out->kp = 0;
    out->ki = 0;
    out->integrator_limit = 0;
    out->alpha = 0;
    out->beta = 0;

    if (in->motor_r_ohm <= 0 ||
        in->motor_l_h <= 0 ||
        in->loop_ts_us == 0 ||
        in->bandwidth_hz <= 0 ||
        in->v_max <= 0) {
        return ESP_FOC_DESIGN_ERR_INVALID_ARG;
    }

    /* omega_bw = 2*pi*bw_hz */
    q16_t omega_bw = q16_mul(Q16_TWO_PI, in->bandwidth_hz);

    /* Nyquist guard: bw[Hz]*Ts[s] >= 0.5 ⇔ bw_q16*ts_us >= 0.5e6*Q16_ONE */
    int64_t bw_ts_term = (int64_t)in->bandwidth_hz * (int64_t)in->loop_ts_us;
    if (bw_ts_term >= 500000LL * (int64_t)Q16_ONE) {
        return ESP_FOC_DESIGN_ERR_OUT_OF_RANGE;
    }

    q16_t exp_arg_alpha = compute_exp_arg_alpha(in->motor_r_ohm,
                                                in->motor_l_h,
                                                in->loop_ts_us);
    q16_t alpha = esp_foc_q16_exp_neg(exp_arg_alpha);

    q16_t exp_arg_beta = compute_exp_arg_beta(omega_bw, in->loop_ts_us);
    q16_t beta = esp_foc_q16_exp_neg(exp_arg_beta);

    if (alpha <= 0 || alpha >= Q16_ONE ||
        beta  <= 0 || beta  >= Q16_ONE) {
        return ESP_FOC_DESIGN_ERR_OUT_OF_RANGE;
    }

    q16_t one_minus_alpha = q16_sub(Q16_ONE, alpha);
    q16_t one_minus_beta  = q16_sub(Q16_ONE, beta);
    if (one_minus_alpha <= 0 || one_minus_beta <= 0) {
        return ESP_FOC_DESIGN_ERR_OUT_OF_RANGE;
    }

    /* Kp = R * (1 - beta) / (1 - alpha) */
    q16_t r_one_minus_beta = q16_mul(in->motor_r_ohm, one_minus_beta);
    q16_t kp = q16_div_internal(r_one_minus_beta, one_minus_alpha);

    /* Ki = R * (1 - beta) / Ts */
    q16_t ki = compute_ki_from_ts_us(r_one_minus_beta, in->loop_ts_us);

    out->kp = kp;
    out->ki = ki;
    out->alpha = alpha;
    out->beta = beta;
    /* Integrator state is in voltage units (see pid_controller.h derivation),
     * so the natural anti-windup ceiling is the output saturation itself. */
    out->integrator_limit = in->v_max;
    out->valid = true;

    return ESP_FOC_DESIGN_OK;
}
