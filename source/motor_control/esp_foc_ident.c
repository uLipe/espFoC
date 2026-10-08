/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include <string.h>

#include "espFoC/motor_control/esp_foc_ident.h"
#include "espFoC/utils/esp_foc_trig.h"

/* 2*pi in q16 radians. */
#define IDENT_TWO_PI_Q16 411775LL
/* Centi-degrees per radian, scaled by 100 to keep the divide exact. */
#define IDENT_CDEG_PER_RAD 572958LL
/* Below this the correlated response is indistinguishable from sense noise. */
#define IDENT_MIN_RESPONSE_MA 15
/* How far above the baseline's own scatter a step has to rise to be an answer. */
#define IDENT_STEP_MARGIN 3

static uint32_t isqrt64(uint64_t v)
{
    uint64_t rem = 0;
    uint64_t root = 0;
    for (int i = 0; i < 32; i++) {
        root <<= 1;
        rem = (rem << 2) | (v >> 62);
        v <<= 2;
        if (root < rem) {
            rem -= root | 1u;
            root += 2;
        }
    }
    return (uint32_t)(root >> 1);
}

static int32_t q16_to_milli(q16_t v)
{
    return (int32_t)(((int64_t)v * 1000) >> 16);
}

bool esp_foc_ident_zprobe_init(esp_foc_ident_zprobe_t *p,
                               const esp_foc_ident_excite_t *cfg)
{
    if (p == NULL || cfg == NULL) {
        return false;
    }
    if (cfg->fs_hz == 0u || cfg->f_hz == 0u || cfg->periods == 0u) {
        return false;
    }
    /* Fewer than 8 samples per period leaves nothing to demodulate. */
    if (cfg->f_hz * 8u > cfg->fs_hz) {
        return false;
    }
    if (cfg->lag_samples >= ESP_FOC_IDENT_LAG_MAX || cfg->v_amp <= 0) {
        return false;
    }

    memset(p, 0, sizeof(*p));
    p->cfg = *cfg;

    uint64_t n = ((uint64_t)cfg->periods * cfg->fs_hz + cfg->f_hz / 2u) / cfg->f_hz;
    if (n < (uint64_t)cfg->periods * 8u) {
        return false;
    }
    p->n_target = (uint32_t)n;
    p->n_skip = (uint32_t)((uint64_t)cfg->settle_periods * n / cfg->periods);
    /* Commensurate frequency: the window is exactly `periods` cycles. */
    p->f_actual_hz = (uint32_t)(((uint64_t)cfg->fs_hz * cfg->periods + n / 2u) / n);
    p->dtheta_hi = ((IDENT_TWO_PI_Q16 * (int64_t)cfg->periods) << 16) / (int64_t)n;
    return true;
}

q16_t esp_foc_ident_zprobe_step(esp_foc_ident_zprobe_t *p, q16_t i_meas)
{
    if (p == NULL || p->done) {
        return 0;
    }

    q16_t s;
    q16_t c;
    esp_foc_sincos((q16_t)(p->theta_hi >> 16), &s, &c);

    uint32_t lag = p->cfg.lag_samples;
    q16_t s_ref = (lag == 0u) ? s : p->s_hist[lag - 1u];
    q16_t c_ref = (lag == 0u) ? c : p->c_hist[lag - 1u];

    p->n++;
    if (p->n > p->n_skip) {
        p->acc_cos += (int64_t)i_meas * (int64_t)c_ref;
        p->acc_sin += (int64_t)i_meas * (int64_t)s_ref;
        if ((p->n - p->n_skip) >= p->n_target) {
            p->done = true;
        }
    }

    for (int k = ESP_FOC_IDENT_LAG_MAX - 1; k > 0; k--) {
        p->s_hist[k] = p->s_hist[k - 1];
        p->c_hist[k] = p->c_hist[k - 1];
    }
    p->s_hist[0] = s;
    p->c_hist[0] = c;

    p->theta_hi += p->dtheta_hi;
    if (p->theta_hi >= (IDENT_TWO_PI_Q16 << 16)) {
        p->theta_hi -= (IDENT_TWO_PI_Q16 << 16);
    }

    return q16_add(p->cfg.v_bias, q16_mul(p->cfg.v_amp, c));
}

bool esp_foc_ident_zprobe_solve(const esp_foc_ident_zprobe_t *p,
                                esp_foc_ident_z_t *out)
{
    if (p == NULL || out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    if (!p->done || p->n_target == 0u) {
        return false;
    }

    out->f_hz = (int32_t)p->f_actual_hz;

    /*
     * i = A*cos(wt - phi) correlated against the command basis gives
     * 2*mean(i*cos) = A*cos(phi) and 2*mean(i*sin) = A*sin(phi), so the
     * in-phase part carries R and the quadrature part carries omega*L.
     */
    int64_t ic_q32 = (p->acc_cos * 2) / (int64_t)p->n_target;
    int64_t is_q32 = (p->acc_sin * 2) / (int64_t)p->n_target;

    /*
     * The bridge holds v across the whole PWM period, so the voltage the
     * winding integrates is centred half a sample after the command phase the
     * correlation referenced. Rotating that back is exact, not a fudge: without
     * it the 605 Hz probe on a 20 kHz carrier reads R +9% and L -9% while |Z|
     * stays within 0.3%, because the error is purely angular.
     */
    int32_t rot_q16 = (int32_t)((IDENT_TWO_PI_Q16 * out->f_hz) /
                                (2 * (int64_t)p->cfg.fs_hz));
    rot_q16 += (int32_t)(((int64_t)p->cfg.phase_trim_cdeg << 16) * 100 /
                         IDENT_CDEG_PER_RAD);
    q16_t s_rot;
    q16_t c_rot;
    esp_foc_sincos((q16_t)rot_q16, &s_rot, &c_rot);
    int64_t ic_rot = ((ic_q32 * c_rot) - (is_q32 * s_rot)) >> 16;
    int64_t is_rot = ((is_q32 * c_rot) + (ic_q32 * s_rot)) >> 16;
    ic_q32 = ic_rot;
    is_q32 = is_rot;

    int32_t ic_ma = (int32_t)((ic_q32 * 1000) >> 32);
    int32_t is_ma = (int32_t)((is_q32 * 1000) >> 32);

    uint64_t mag2 = (uint64_t)((int64_t)ic_ma * ic_ma + (int64_t)is_ma * is_ma);
    int32_t i_ma = (int32_t)isqrt64(mag2);
    out->i_amp_ma = i_ma;
    if (i_ma < IDENT_MIN_RESPONSE_MA) {
        return false;
    }

    int32_t v_mv = q16_to_milli(p->cfg.v_amp);
    out->z_mohm = (int32_t)(((int64_t)v_mv * 1000) / i_ma);
    out->r_mohm = (int32_t)(((int64_t)out->z_mohm * ic_ma) / i_ma);

    int32_t wl_mohm = (int32_t)(((int64_t)out->z_mohm * is_ma) / i_ma);
    int32_t w_rad_s = (int32_t)((IDENT_TWO_PI_Q16 * out->f_hz) >> 16);
    if (w_rad_s > 0) {
        out->l_uh = (int32_t)(((int64_t)wl_mohm * 1000) / w_rad_s);
    }

    q16_t ph = esp_foc_atan2((q16_t)(is_q32 >> 16), (q16_t)(ic_q32 >> 16));
    out->phase_cdeg = (int32_t)(((int64_t)ph * IDENT_CDEG_PER_RAD) >> 16) / 100;

    out->valid = (out->r_mohm > 0);
    return out->valid;
}

int32_t esp_foc_ident_best_probe_hz(int32_t r_mohm, int32_t l_uh)
{
    if (r_mohm <= 0 || l_uh <= 0) {
        return 0;
    }
    return (int32_t)(((int64_t)r_mohm * 1000 << 16) /
                     ((int64_t)l_uh * IDENT_TWO_PI_Q16));
}

int32_t esp_foc_ident_l_from_mag_uh(int32_t z_mohm, int32_t r_mohm, int32_t f_hz)
{
    if (z_mohm <= 0 || r_mohm < 0 || f_hz <= 0 || z_mohm <= r_mohm) {
        return 0;
    }
    const int64_t z2 = (int64_t)z_mohm * z_mohm;
    const int64_t r2 = (int64_t)r_mohm * r_mohm;
    const int32_t xl_mohm = (int32_t)isqrt64((uint64_t)(z2 - r2));
    /* L = XL / (2*pi*f), carried in microhenries. */
    return (int32_t)(((int64_t)xl_mohm * 1000 << 16) /
                     ((int64_t)f_hz * IDENT_TWO_PI_Q16));
}

void esp_foc_ident_dcprobe_init(esp_foc_ident_dcprobe_t *p,
                                uint32_t skip_samples,
                                uint32_t avg_samples)
{
    if (p == NULL) {
        return;
    }
    memset(p, 0, sizeof(*p));
    p->n_skip = skip_samples;
    p->n_target = (avg_samples == 0u) ? 1u : avg_samples;
}

void esp_foc_ident_dcprobe_add(esp_foc_ident_dcprobe_t *p, q16_t i_meas)
{
    if (p == NULL || p->done) {
        return;
    }
    p->n++;
    if (p->n <= p->n_skip) {
        return;
    }
    p->acc += i_meas;
    if ((p->n - p->n_skip) >= p->n_target) {
        p->done = true;
    }
}

int32_t esp_foc_ident_dcprobe_ma(const esp_foc_ident_dcprobe_t *p)
{
    if (p == NULL || p->n_target == 0u) {
        return 0;
    }
    uint32_t got = (p->n > p->n_skip) ? (p->n - p->n_skip) : 0u;
    if (got == 0u) {
        return 0;
    }
    if (got > p->n_target) {
        got = p->n_target;
    }
    return q16_to_milli((q16_t)(p->acc / (int64_t)got));
}

int32_t esp_foc_ident_r_slope_mohm(int32_t v1_mv, int32_t i1_ma,
                                   int32_t v2_mv, int32_t i2_ma)
{
    int32_t di = i2_ma - i1_ma;
    if (di == 0) {
        return 0;
    }
    return (int32_t)(((int64_t)(v2_mv - v1_mv) * 1000) / di);
}

int32_t esp_foc_ident_step_lag(const q16_t *i, uint32_t n, uint32_t n_pre,
                               q16_t thresh)
{
    if (i == NULL || n_pre == 0u || n <= n_pre || thresh <= 0) {
        return -1;
    }

    int64_t acc = 0;
    for (uint32_t k = 0; k < n_pre; k++) {
        acc += i[k];
    }
    const q16_t base = (q16_t)(acc / (int64_t)n_pre);

    /*
     * The bar is what the quiet samples already do, times a margin. On this bench
     * the sense floor peaks at 700 mA against a first-sample rise of 170 mA, so a
     * single step cannot be timed at all — the caller averages many of them, and
     * this is what turns that averaging into a lower bar rather than a fixed
     * guess.
     */
    q16_t spread = 0;
    for (uint32_t k = 0; k < n_pre; k++) {
        const q16_t d = i[k] - base;
        const q16_t mag = (d < 0) ? -d : d;
        if (mag > spread) {
            spread = mag;
        }
    }
    const q16_t bar = spread * IDENT_STEP_MARGIN;
    if (bar > thresh) {
        thresh = bar;
    }

    /*
     * The delay is counted from the sample that carries the step, not from the one
     * after it: the ISR reads the current before it writes the new duty, so the
     * sample taken in the same interrupt as the edge still belongs to the old
     * command and a winding answering at the very next one is one sample late.
     */
    for (uint32_t k = n_pre; k < n; k++) {
        const q16_t d = i[k] - base;
        const q16_t mag = (d < 0) ? -d : d;
        if (mag >= thresh) {
            return (int32_t)(k - n_pre);
        }
    }
    return -1;
}



void esp_foc_ident_symmetry(const int32_t i_ma[3],
                            int32_t limit_permil,
                            esp_foc_ident_sym_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (i_ma == NULL) {
        return;
    }

    int32_t mag[3];
    for (int k = 0; k < 3; k++) {
        mag[k] = (i_ma[k] < 0) ? -i_ma[k] : i_ma[k];
    }

    int hi = 0;
    int lo = 0;
    for (int k = 1; k < 3; k++) {
        if (mag[k] > mag[hi]) {
            hi = k;
        }
        if (mag[k] < mag[lo]) {
            lo = k;
        }
    }
    out->strong_idx = hi;
    out->weak_idx = lo;
    if (mag[hi] <= 0) {
        return;
    }
    out->spread_permil =
        (int32_t)(((int64_t)(mag[hi] - mag[lo]) * 1000) / mag[hi]);
    out->ok = (out->spread_permil <= limit_permil);
}

int32_t esp_foc_ident_psi_uwb(int32_t vq_mv, int32_t iq_ma, int32_t id_ma,
                              int32_t r_mohm, int32_t l_uh, int32_t w_rad_s)
{
    if (w_rad_s <= 0) {
        return 0;
    }
    int64_t drop_r_mv = ((int64_t)r_mohm * iq_ma) / 1000;
    int64_t drop_l_mv = ((int64_t)w_rad_s * l_uh * id_ma) / 1000000;
    int64_t emf_mv = (int64_t)vq_mv - drop_r_mv - drop_l_mv;
    return (int32_t)((emf_mv * 1000) / w_rad_s);
}
