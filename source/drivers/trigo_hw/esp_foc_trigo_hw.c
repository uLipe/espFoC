/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include "espFoC/drivers/esp_foc_trigo_hw.h"

#include "soc/soc_caps.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_trig.h"

#if SOC_CORDIC_SUPPORTED && __has_include("hal/cordic_ll.h")
#define TRIGO_HW_CORDIC 1
#include "hal/cordic_ll.h"
#include "soc/cordic_struct.h"
#else
#define TRIGO_HW_CORDIC 0
#endif

static q16_t soft_magnitude(q16_t x, q16_t y)
{
    return esp_foc_sqrt(q16_add(q16_mul(x, x), q16_mul(y, y)));
}

#if TRIGO_HW_CORDIC

/*
 * Every register access costs ~31 CPU cycles at 320 MHz and the computation
 * hides behind them, so each operation is one composed csr write
 * (configuration + start) rather than the LL read-modify-write setters, and
 * the configuration is rewritten per call because sincos and phase alternate
 * in the same ISR. The ready poll is required: a read without it can return
 * the previous result. Press fields 6 and up return wrong sin/cos/modulus
 * on silicon; 4 gives 0.6 LSB (q1.31) and 1.4 LSB phase.
 */
#define CSR_FUNC_COS    0u
#define CSR_FUNC_PHASE  2u
#define CSR_PRESS       (4u << 4)
#define CSR_RES_TWO     (1u << 19)
#define CSR_ARG_TWO     (1u << 20)
#define CSR_RES_Q31     (1u << 21)
#define CSR_ARG_Q31     (1u << 22)
#define CSR_UPDATE      (1u << 29)
#define CSR_RDY         (1u << 31)
#define CSR_SINCOS      (CSR_FUNC_COS | CSR_PRESS | CSR_RES_TWO | CSR_ARG_Q31 | CSR_UPDATE)
#define CSR_PHASE       (CSR_FUNC_PHASE | CSR_PRESS | CSR_RES_TWO | CSR_ARG_TWO | CSR_RES_Q31 | \
                         CSR_ARG_Q31 | CSR_UPDATE)
#define READY_POLLS     32u
/* 2^31 / pi in Q16: Q16.16 rad -> q1.31 of x / pi. */
#define RAD_TO_Q31      683565276LL
/* pi * 2^29: q1.31 of x / pi -> Q16.16 rad after >> 44. */
#define Q31_TO_RAD      1686629713LL

static bool s_ready;
static uint32_t s_timeouts;

static inline bool wait_ready(void)
{
    for (uint32_t n = 0; n < READY_POLLS; n++) {
        if (CORDIC.csr_cfg.val & CSR_RDY) {
            return true;
        }
    }
    s_timeouts++;
    return false;
}

/* Common shift so |x|, |y| < 0.5 in q1.31: the modulus stays below 1. */
static inline int norm_shift(q16_t x, q16_t y)
{
    const uint32_t ax = (x < 0) ? (0u - (uint32_t)x) : (uint32_t)x;
    const uint32_t ay = (y < 0) ? (0u - (uint32_t)y) : (uint32_t)y;
    const uint32_t m = (ax > ay) ? ax : ay;
    return __builtin_clz(m) - 2;
}

static inline int32_t shift_arg(q16_t v, int sh)
{
    return (sh >= 0) ? (int32_t)((uint32_t)v << sh) : (v >> -sh);
}

static inline bool cordic_phase(q16_t x, q16_t y, int *sh, int32_t *p31, uint32_t *r31)
{
    const int s = norm_shift(x, y);
    CORDIC.arg1.val = (uint32_t)shift_arg(x, s);
    CORDIC.arg2.val = (uint32_t)shift_arg(y, s);
    CORDIC.csr_cfg.val = CSR_PHASE;
    if (!wait_ready()) {
        return false;
    }
    *p31 = (int32_t)CORDIC.res1.val;
    *r31 = CORDIC.res2.val;
    *sh = s;
    return true;
}

esp_err_t esp_foc_trigo_hw_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }
    cordic_ll_enable_bus_clock(true);
    cordic_ll_reset_module_register();
    cordic_ll_enable_clock(true);
    cordic_ll_set_clock_source(CORDIC_CLK_SRC_PLL_F160M);
    s_timeouts = 0;
    s_ready = true;
    return ESP_OK;
}

bool esp_foc_trigo_hw_accelerated(void)
{
    return s_ready;
}

void esp_foc_trigo_hw_sincos(q16_t angle, q16_t *s_out, q16_t *c_out)
{
    if (s_ready) {
        CORDIC.arg1.val = (uint32_t)(int32_t)(((int64_t)angle * RAD_TO_Q31) >> 16);
        CORDIC.csr_cfg.val = CSR_SINCOS;
        if (wait_ready()) {
            const uint32_t r = CORDIC.res1.val;
            *c_out = (q16_t)(int16_t)(r & 0xFFFFu) * 2;
            *s_out = (q16_t)(int16_t)(r >> 16) * 2;
            return;
        }
    }
    esp_foc_sincos(angle, s_out, c_out);
}

q16_t esp_foc_trigo_hw_atan2(q16_t y, q16_t x)
{
    if (s_ready) {
        if ((x | y) == 0) {
            return 0;
        }
        int sh;
        int32_t p31;
        uint32_t r31;
        if (cordic_phase(x, y, &sh, &p31, &r31)) {
            if (p31 == INT32_MIN) {
                return Q16_PI;
            }
            return (q16_t)(((int64_t)p31 * Q31_TO_RAD + (1LL << 43)) >> 44);
        }
    }
    return esp_foc_atan2(y, x);
}

q16_t esp_foc_trigo_hw_magnitude(q16_t x, q16_t y)
{
    if (s_ready) {
        if ((x | y) == 0) {
            return 0;
        }
        int sh;
        int32_t p31;
        uint32_t r31;
        if (cordic_phase(x, y, &sh, &p31, &r31)) {
            const uint64_t m = (sh >= 0) ? ((uint64_t)r31 >> sh) : ((uint64_t)r31 << -sh);
            return (m > (uint64_t)INT32_MAX) ? (q16_t)INT32_MAX : (q16_t)m;
        }
    }
    return soft_magnitude(x, y);
}

uint32_t esp_foc_trigo_hw_get_timeouts(void)
{
    return s_timeouts;
}

#else /* !TRIGO_HW_CORDIC */

esp_err_t esp_foc_trigo_hw_init(void)
{
    return ESP_OK;
}

bool esp_foc_trigo_hw_accelerated(void)
{
    return false;
}

void esp_foc_trigo_hw_sincos(q16_t angle, q16_t *s_out, q16_t *c_out)
{
    esp_foc_sincos(angle, s_out, c_out);
}

q16_t esp_foc_trigo_hw_atan2(q16_t y, q16_t x)
{
    return esp_foc_atan2(y, x);
}

q16_t esp_foc_trigo_hw_magnitude(q16_t x, q16_t y)
{
    return soft_magnitude(x, y);
}

uint32_t esp_foc_trigo_hw_get_timeouts(void)
{
    return 0;
}

#endif /* TRIGO_HW_CORDIC */
