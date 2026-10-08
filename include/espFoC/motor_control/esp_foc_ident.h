/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Machine identification kernels: pure math, no driver and no OS.
 *
 * Split from the sequencer so the numerics are testable without an inverter.
 * The accumulate calls are O(1) and run in the PWM ISR; the solve calls run
 * once when a probe window closes and are not hot-path.
 *
 * Units: per-sample state stays q16 (currents in amps, angles in radians),
 * but solved machine parameters are plain integers in scaled engineering units
 * (milliohm, microhenry, microweber). 500 uH is 33 counts as q16 henry — a 3%
 * quantization on the headline result — and worse once multiplied by omega.
 */

/**
 * Largest command-to-measurement delay the probe can reference, in samples.
 *
 * The ADC chain on this bench measures about 3.5 PWM periods behind the command:
 * the ETM trigger, the DMA block, and the interrupt that latches it each cost
 * time. A range of 4 could not reach it, and the residual showed up as a 42 per
 * cent error on L at the coarse point.
 */
#define ESP_FOC_IDENT_LAG_MAX 8

/**
 * Sine excitation for a synchronous-demodulation impedance probe.
 *
 * @p f_hz is a request: the probe rounds the window to a whole number of
 * periods and reports the commensurate frequency it actually used. Whole
 * periods are what make the ADC offset and any residual BEMF fundamental
 * cancel in the correlation.
 *
 * @p lag_samples must match the sense pipeline delay, otherwise the demodulated
 * vector is rotated and the R/L split is wrong: one PWM period at 605 Hz on a
 * 20 kHz carrier is already 10.9 degrees. Feed pre-LPF currents; a sense LPF in
 * the path adds phase of its own that this does not model.
 */
typedef struct {
    q16_t v_amp;            /**< excitation amplitude, volts */
    /**
     * DC term the sine rides on, volts. A whole-period correlation is exactly
     * orthogonal to a constant, so this does not enter R or L — its job is to
     * keep the current from changing sign.
     *
     * A real bridge has no gain until the applied voltage clears its deadtime
     * dead zone, because while the current is near zero the deadtime interval
     * leaves the phase floating instead of driving it. On this bench that zone
     * is 0.7 V wide: an unbiased 0.84 V probe spent most of its cycle inside it
     * and the demodulator saw the fundamental of a clipped pulse train, reading
     * R at twice its value and L as negative. Bias the current past zero and the
     * deadtime error becomes a constant offset, which the correlation rejects.
     *
     * The current must never reverse, so this has to cover the dead zone plus
     * the drop the AC amplitude itself produces.
     */
    q16_t v_bias;
    uint32_t f_hz;          /**< requested frequency */
    uint32_t fs_hz;         /**< sample (PWM) rate */
    uint32_t periods;       /**< whole periods to correlate over */
    uint32_t settle_periods;/**< periods discarded before correlating */
    /** Command-to-measurement delay, 0..ESP_FOC_IDENT_LAG_MAX-1 */
    uint32_t lag_samples;
    /**
     * Extra phase added to the demodulated vector, on top of the half-sample
     * ZOH the solve always removes. Leave 0 for pre-LPF currents; a sense LPF
     * in the path needs its phase at the probe frequency put here.
     */
    int32_t phase_trim_cdeg;
} esp_foc_ident_excite_t;

typedef struct {
    int32_t r_mohm;         /**< series R at the probe frequency */
    int32_t l_uh;           /**< series L at the probe frequency */
    int32_t z_mohm;         /**< |Z| */
    int32_t phase_cdeg;     /**< arg(Z), centi-degrees; 4500 is best-conditioned */
    int32_t i_amp_ma;       /**< response amplitude */
    int32_t f_hz;           /**< frequency actually used */
    bool valid;
} esp_foc_ident_z_t;

typedef struct {
    esp_foc_ident_excite_t cfg;
    /*
     * Phase advances in q32 radians. A q16 step accumulates enough rounding to
     * drift ~2 degrees across a 32-period window, which breaks the whole-period
     * cancellation the probe relies on.
     */
    int64_t dtheta_hi;
    int64_t theta_hi;
    /* Newest first, so index lag-1 is the command that produced this sample. */
    q16_t s_hist[ESP_FOC_IDENT_LAG_MAX];
    q16_t c_hist[ESP_FOC_IDENT_LAG_MAX];
    int64_t acc_cos;
    int64_t acc_sin;
    uint32_t n;
    uint32_t n_skip;
    uint32_t n_target;
    uint32_t f_actual_hz;
    bool done;
} esp_foc_ident_zprobe_t;

/**
 * Arm an impedance probe. Rejects f_hz above fs/8 (too few samples per period
 * to demodulate) and periods == 0.
 */
bool esp_foc_ident_zprobe_init(esp_foc_ident_zprobe_t *p,
                               const esp_foc_ident_excite_t *cfg);

/**
 * One ISR step: returns the excitation voltage to apply now and correlates
 * @p i_meas against the command phase from @p lag_samples ago.
 */
q16_t esp_foc_ident_zprobe_step(esp_foc_ident_zprobe_t *p, q16_t i_meas);

static inline bool esp_foc_ident_zprobe_done(const esp_foc_ident_zprobe_t *p)
{
    return p->done;
}

/** Solve R and L from the correlated vector. False if the response was noise. */
bool esp_foc_ident_zprobe_solve(const esp_foc_ident_zprobe_t *p,
                                esp_foc_ident_z_t *out);

/**
 * Frequency where omega*L equals R, i.e. arg(Z) = 45 degrees. Probing here
 * splits R and L best; a probe far below it sees almost pure R (100 Hz on a
 * 1.9 ohm / 500 uH machine moves |Z| by 1.4%, which noise swamps).
 */
int32_t esp_foc_ident_best_probe_hz(int32_t r_mohm, int32_t l_uh);

/**
 * Inductance from the magnitude of Z and a known R, in microhenries.
 *
 * |Z| is the one part of the measurement that no phase error can touch, while the
 * R and L split relies on the angle and therefore on knowing the delay from
 * command to measurement exactly. On a bench where that delay is around five
 * samples, the angle came back negative on a plain RL winding, and matching R
 * across frequencies cannot even see the difference because cos is even. So L is
 * taken from sqrt(|Z|^2 - R^2) / omega instead, and only needs R, which the
 * low-frequency probe measures well.
 *
 * Returns 0 when |Z| does not exceed R, meaning the probe frequency is too low
 * for the reactance to show above the resistance.
 */
int32_t esp_foc_ident_l_from_mag_uh(int32_t z_mohm, int32_t r_mohm,
                                    int32_t f_hz);

typedef struct {
    int64_t acc;
    uint32_t n;
    uint32_t n_skip;
    uint32_t n_target;
    bool done;
} esp_foc_ident_dcprobe_t;

void esp_foc_ident_dcprobe_init(esp_foc_ident_dcprobe_t *p,
                                uint32_t skip_samples,
                                uint32_t avg_samples);
void esp_foc_ident_dcprobe_add(esp_foc_ident_dcprobe_t *p, q16_t i_meas);

static inline bool esp_foc_ident_dcprobe_done(const esp_foc_ident_dcprobe_t *p)
{
    return p->done;
}

/** Mean current over the averaging window, milliamps. */
int32_t esp_foc_ident_dcprobe_ma(const esp_foc_ident_dcprobe_t *p);

/**
 * R from the slope through two DC operating points.
 *
 * A single V/I point folds the bridge's fixed deadtime and Vds drop into Rs: on
 * this bench 0.75 V drew 0.609 A, reading 1.23 ohm across a 0.75 ohm winding.
 * The slope cancels that offset because it is a voltage, not a resistance. Any
 * genuinely series resistance such as Rds_on stays in the result.
 */
int32_t esp_foc_ident_r_slope_mohm(int32_t v1_mv, int32_t i1_ma,
                                   int32_t v2_mv, int32_t i2_ma);

/** Samples a step window records after the edge; a few electrical time constants
 *  at 20 kHz on any winding this bridge can drive. */
#define ESP_FOC_IDENT_STEP_MAX 24

/**
 * Command-to-measurement delay, timed off a voltage step.
 *
 * Matching R across two frequencies infers this delay instead of measuring it,
 * and it can only do so if R is the same at both — which iron loss denies. This
 * bench swept eight candidates and got 740, 502, 313, 158, 88 per-mil with no
 * minimum in range: every extra sample of compensation kept improving the match
 * because part of the mismatch was never a delay at all, and the sweep would have
 * rotated the vector until L came out negative.
 *
 * A step needs no such premise. The winding cannot answer before it is driven, so
 * the first sample that moves is the delay, whatever R does with frequency.
 *
 * What counts as moving is decided from the quiet samples, not from a fraction of
 * the settled current: that fraction would time when the winding reached a given
 * per cent, which is L over R all over again. The baseline's own scatter sets the
 * bar, so a window averaged over many steps earns a lower one.
 *
 * @param i         window samples, @p n_pre of them taken before the step
 * @param n         total samples in @p i
 * @param n_pre     samples that precede the edge, used as the quiet baseline
 * @param thresh    smallest rise to accept regardless of how quiet the baseline
 *                  looks, q16 amps; guards against a window that is quiet only
 *                  because nothing is connected
 * @return delay in samples, or -1 if nothing crossed the bar
 */
int32_t esp_foc_ident_step_lag(const q16_t *i, uint32_t n, uint32_t n_pre,
                               q16_t thresh);

typedef struct {
    int32_t spread_permil;  /**< (max-min)/max of the three terminals */
    int weak_idx;
    int strong_idx;
    bool ok;
} esp_foc_ident_sym_t;

/**
 * Winding symmetry from per-terminal DC injection.
 *
 * Refuses asymmetric machines before any gain is designed. An open delta
 * winding shows up as a 1:2:1 current ratio across the three terminals, which
 * is a 500 per-mil spread; a healthy machine stays inside a few tens.
 */
void esp_foc_ident_symmetry(const int32_t i_ma[3],
                            int32_t limit_permil,
                            esp_foc_ident_sym_t *out);

/**
 * Permanent-magnet flux from the steady-state q-axis balance
 * vq = R*iq + omega*L*id + omega*psi, in microweber. Needs omega != 0.
 */
int32_t esp_foc_ident_psi_uwb(int32_t vq_mv, int32_t iq_ma, int32_t id_ma,
                              int32_t r_mohm, int32_t l_uh, int32_t w_rad_s);

#ifdef __cplusplus
}
#endif
