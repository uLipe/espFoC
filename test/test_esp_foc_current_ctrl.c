/*
 * Unit tests for esp_foc_current_ctrl (dq current loop iface, PI and ADRC).
 *
 * Plant: decoupled RL axes in pu of Vdc, exact ZOH, input computed at k
 * reaching the plant at k+1 (delay_ts = 1). The FF path mirrors the stacks: controller
 * share + FF, circle limit, then set_applied() with the total minus FF.
 */
#include <math.h>
#include <string.h>

#include "unity.h"
#include "esp_err.h"
#include "espFoC/motor_control/esp_foc_current_ctrl.h"
#include "espFoC/utils/esp_foc_pid.h"
#include "espFoC/utils/esp_foc_q16.h"

#define RS      1.0f
#define LS      0.5e-3f
#define VDC     12.0f
#define FS      20000.0f
#define TS      (1.0f / FS)
#define VMAX    0.57735f

typedef struct {
    float a, b_u, b_v;
    float id, iq;
    float ud, uq;       /* applied over the current period */
    float vd_dist, vq_dist;
} dq_plant_t;

static void plant_init(dq_plant_t *p)
{
    memset(p, 0, sizeof(*p));
    p->a = expf(-RS / LS * TS);
    p->b_v = (1.0f - p->a) / RS;
    p->b_u = p->b_v * VDC;
}

static esp_foc_current_ctrl_design_t design_default(void)
{
    return (esp_foc_current_ctrl_design_t){
        .rs_ohm = RS, .ls_h = LS, .vdc_v = VDC, .fs_hz = FS,
        .bw_hz = 200.0f, .gain_scale = 0.5f, .wo_ratio = 5.0f, .delay_ts = 1u,
    };
}

static void clamp_circle(q16_t *vd, q16_t *vq)
{
    const float d = q16_to_float(*vd), q = q16_to_float(*vq);
    const float m = sqrtf(d * d + q * q);
    if (m > VMAX) {
        *vd = q16_from_float(d * VMAX / m);
        *vq = q16_from_float(q * VMAX / m);
    }
}

/* Returns the controller share of vq (before FF) of the last period. */
static q16_t run(esp_foc_current_ctrl_t *c, dq_plant_t *p, float idr, float iqr, float vd_ff,
                 float vq_ff, int n)
{
    const q16_t ffd = q16_from_float(vd_ff), ffq = q16_from_float(vq_ff);
    q16_t vq_ctrl = 0;
    for (int k = 0; k < n; k++) {
        q16_t vd, vq;
        esp_foc_current_ctrl_update(c, q16_from_float(idr), q16_from_float(iqr),
                                    q16_from_float(p->id), q16_from_float(p->iq), &vd, &vq);
        vq_ctrl = vq;
        vd = q16_add(vd, ffd);
        vq = q16_add(vq, ffq);
        clamp_circle(&vd, &vq);
        esp_foc_current_ctrl_set_applied(c, q16_sub(vd, ffd), q16_sub(vq, ffq));
        p->id = p->a * p->id + p->b_u * p->ud - p->b_v * p->vd_dist;
        p->iq = p->a * p->iq + p->b_u * p->uq - p->b_v * p->vq_dist;
        p->ud = q16_to_float(vd);
        p->uq = q16_to_float(vq);
    }
    return vq_ctrl;
}

TEST_CASE("current_ctrl bind wires every op", "[espFoC][current_ctrl]")
{
    esp_foc_current_pi_t pi;
    esp_foc_current_adrc_t ad;
    esp_foc_current_ctrl_t *c = esp_foc_current_pi_bind(&pi);
    TEST_ASSERT_EQUAL_PTR(&pi.iface, c);
    TEST_ASSERT_NOT_NULL(c->design);
    TEST_ASSERT_NOT_NULL(c->reset);
    TEST_ASSERT_NOT_NULL(c->update);
    TEST_ASSERT_NOT_NULL(c->set_applied);
    c = esp_foc_current_adrc_bind(&ad);
    TEST_ASSERT_EQUAL_PTR(&ad.iface, c);
    TEST_ASSERT_NOT_NULL(c->design);
    TEST_ASSERT_NULL(esp_foc_current_pi_bind(NULL));
    TEST_ASSERT_NULL(esp_foc_current_adrc_bind(NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_current_ctrl_design(NULL, NULL));
}

TEST_CASE("current_ctrl design rejects bad plants", "[espFoC][current_ctrl]")
{
    esp_foc_current_pi_t pi;
    esp_foc_current_adrc_t ad;
    esp_foc_current_ctrl_t *cp = esp_foc_current_pi_bind(&pi);
    esp_foc_current_ctrl_t *ca = esp_foc_current_adrc_bind(&ad);
    esp_foc_current_ctrl_design_t d = design_default();
    d.ls_h = 0.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_current_ctrl_design(cp, &d));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_current_ctrl_design(ca, &d));
    d = design_default();
    d.gain_scale = 1.5f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_current_ctrl_design(cp, &d));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_current_ctrl_design(ca, &d));
    d = design_default();
    d.wo_ratio = 1.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_current_ctrl_design(ca, &d));
    d = design_default();
    d.delay_ts = ESP_FOC_ADRC_DELAY_MAX + 1u;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_current_ctrl_design(ca, &d));
    d = design_default();
    d.kp = -1.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_current_ctrl_design(cp, &d));
}

TEST_CASE("current_ctrl PI matches the bare IMC-zoh PI", "[espFoC][current_ctrl]")
{
    esp_foc_current_pi_t pi;
    esp_foc_current_ctrl_t *c = esp_foc_current_pi_bind(&pi);
    const esp_foc_current_ctrl_design_t d = design_default();
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_current_ctrl_design(c, &d));

    float kp, ki;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_design_imc_zoh(VDC / RS, LS / RS, FS, d.bw_hz, &kp, &ki));
    kp *= d.gain_scale;
    ki *= d.gain_scale;
    TEST_ASSERT_EQUAL_FLOAT(kp, c->tune.kp);
    TEST_ASSERT_EQUAL_FLOAT(ki, c->tune.ki);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c->tune.wo_hz);

    esp_foc_pid_t rd, rq;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_init(&rd, kp, ki, 0.0f, 0.0f, TS));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_init(&rq, kp, ki, 0.0f, 0.0f, TS));
    const q16_t ffd = q16_from_float(-0.05f), ffq = q16_from_float(0.1f);
    for (int k = 0; k < 500; k++) {
        const q16_t idr = 0, iqr = q16_from_float(k < 250 ? 1.0f : -2.0f);
        const q16_t id = q16_from_float(0.01f * (k % 7)), iq = q16_from_float(0.003f * k);
        q16_t vd, vq;
        esp_foc_current_ctrl_update(c, idr, iqr, id, iq, &vd, &vq);
        const q16_t rvd = esp_foc_pid_update(&rd, idr, id);
        const q16_t rvq = esp_foc_pid_update(&rq, iqr, iq);
        TEST_ASSERT_EQUAL_INT32(rvd, vd);
        TEST_ASSERT_EQUAL_INT32(rvq, vq);
        vd = q16_add(vd, ffd);
        vq = q16_add(vq, ffq);
        clamp_circle(&vd, &vq);
        esp_foc_current_ctrl_set_applied(c, q16_sub(vd, ffd), q16_sub(vq, ffq));
        esp_foc_pid_set_applied(&rd, q16_sub(vd, ffd));
        esp_foc_pid_set_applied(&rq, q16_sub(vq, ffq));
    }
}

TEST_CASE("current_ctrl PI explicit gains bypass the design", "[espFoC][current_ctrl]")
{
    esp_foc_current_pi_t pi;
    esp_foc_current_ctrl_t *c = esp_foc_current_pi_bind(&pi);
    esp_foc_current_ctrl_design_t d = design_default();
    d.kp = 0.02f;
    d.ki = 40.0f;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_current_ctrl_design(c, &d));
    TEST_ASSERT_EQUAL_FLOAT(0.02f, c->tune.kp);
    TEST_ASSERT_EQUAL_FLOAT(40.0f, c->tune.ki);
}

TEST_CASE("current_ctrl ADRC design reports wc, wo and b0", "[espFoC][current_ctrl]")
{
    esp_foc_current_adrc_t ad;
    esp_foc_current_ctrl_t *c = esp_foc_current_adrc_bind(&ad);
    const esp_foc_current_ctrl_design_t d = design_default();
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_current_ctrl_design(c, &d));
    TEST_ASSERT_EQUAL_FLOAT(100.0f, c->tune.wc_hz);
    TEST_ASSERT_EQUAL_FLOAT(500.0f, c->tune.wo_hz);
    TEST_ASSERT_EQUAL_FLOAT(VDC / LS, c->tune.b0);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c->tune.kp);
}

TEST_CASE("current_ctrl ADRC keeps the observer near a fast plant pole", "[espFoC][current_ctrl]")
{
    esp_foc_current_adrc_t ad;
    esp_foc_current_ctrl_t *c = esp_foc_current_adrc_bind(&ad);
    esp_foc_current_ctrl_design_t d = design_default();
    /* Rs/Ls = 7143 rad/s: 0.8 of it is 909 Hz, above 5 x 100 Hz */
    d.rs_ohm = 3.0f;
    d.ls_h = 420e-6f;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_current_ctrl_design(c, &d));
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.8f * 3.0f / 420e-6f / 6.2831853f, c->tune.wo_hz);
    /* the ESO correction gain stays positive */
    TEST_ASSERT_TRUE(ad.d.l1 > 0);
    TEST_ASSERT_TRUE(ad.q.l1 > 0);

    /* Rs/Ls = 25000 rad/s: capped at fs/16 */
    d.rs_ohm = 10.0f;
    d.ls_h = 400e-6f;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_current_ctrl_design(c, &d));
    TEST_ASSERT_EQUAL_FLOAT(FS / 16.0f, c->tune.wo_hz);

    /* the ratio floor wins when it is above the capped pole term */
    d.wo_ratio = 15.0f;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_current_ctrl_design(c, &d));
    TEST_ASSERT_EQUAL_FLOAT(1500.0f, c->tune.wo_hz);
}

TEST_CASE("current_ctrl both controllers track iq and hold id at zero", "[espFoC][current_ctrl]")
{
    esp_foc_current_pi_t pi;
    esp_foc_current_adrc_t ad;
    esp_foc_current_ctrl_t *ctl[2] = { esp_foc_current_pi_bind(&pi), esp_foc_current_adrc_bind(&ad) };
    const esp_foc_current_ctrl_design_t d = design_default();
    for (int j = 0; j < 2; j++) {
        dq_plant_t p;
        plant_init(&p);
        TEST_ASSERT_EQUAL(ESP_OK, esp_foc_current_ctrl_design(ctl[j], &d));
        p.vd_dist = 0.4f;
        p.vq_dist = 2.0f;
        (void)run(ctl[j], &p, 0.0f, 1.5f, 0.0f, 0.0f, 6000);
        TEST_ASSERT_FLOAT_WITHIN(0.015f, 1.5f, p.iq);
        TEST_ASSERT_FLOAT_WITHIN(0.015f, 0.0f, p.id);
        const float iq0 = p.iq;
        (void)run(ctl[j], &p, 0.0f, 1.5f, 0.0f, 0.0f, 2000);
        TEST_ASSERT_FLOAT_WITHIN(0.002f, iq0, p.iq);
        (void)run(ctl[j], &p, 0.0f, -1.0f, 0.0f, 0.0f, 6000);
        TEST_ASSERT_FLOAT_WITHIN(0.015f, -1.0f, p.iq);
    }
}

TEST_CASE("current_ctrl ADRC with an exact feedforward adds nothing", "[espFoC][current_ctrl]")
{
    esp_foc_current_adrc_t ad;
    esp_foc_current_ctrl_t *c = esp_foc_current_adrc_bind(&ad);
    const esp_foc_current_ctrl_design_t d = design_default();
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_current_ctrl_design(c, &d));
    dq_plant_t p;
    plant_init(&p);
    /* FF = back-EMF plus the resistive drop at the target, exactly. */
    p.vq_dist = 2.0f;
    const float iqr = 1.0f;
    const float ffq = (p.vq_dist + RS * iqr) / VDC;
    const q16_t vq_ctrl = run(c, &p, 0.0f, iqr, 0.0f, ffq, 8000);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, iqr, p.iq);
    /* With a perfect FF the controller has nothing left to add. */
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.0f, q16_to_float(vq_ctrl));
}

TEST_CASE("current_ctrl reset returns both controllers to rest", "[espFoC][current_ctrl]")
{
    esp_foc_current_pi_t pi;
    esp_foc_current_adrc_t ad;
    esp_foc_current_ctrl_t *ctl[2] = { esp_foc_current_pi_bind(&pi), esp_foc_current_adrc_bind(&ad) };
    const esp_foc_current_ctrl_design_t d = design_default();
    for (int j = 0; j < 2; j++) {
        dq_plant_t p;
        plant_init(&p);
        TEST_ASSERT_EQUAL(ESP_OK, esp_foc_current_ctrl_design(ctl[j], &d));
        (void)run(ctl[j], &p, 0.3f, 1.0f, 0.0f, 0.0f, 500);
        esp_foc_current_ctrl_reset(ctl[j]);
        q16_t vd, vq;
        esp_foc_current_ctrl_update(ctl[j], 0, 0, 0, 0, &vd, &vq);
        TEST_ASSERT_INT32_WITHIN(8, 0, vd);
        TEST_ASSERT_INT32_WITHIN(8, 0, vq);
    }
}
