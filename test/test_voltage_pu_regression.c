/*
 * 48 V bus, Uq = 0.5 pu → SVPWM duties near 0.5 on the driven axis.
 */
#include <math.h>
#include <unity.h>
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/modulator.h"

#define FLOAT_TOL 0.03f

TEST_CASE("voltage_pu: 48V nominal Uq=0.5pu yields ~0.5 duty", "[espFoC][voltage_pu]")
{
    q16_t sin_t = 0;
    q16_t cos_t = Q16_ONE;
    q16_t vd = 0;
    q16_t vq = q16_mul(ESP_FOC_VPU_ONE_Q16, q16_from_float(0.5f));
    q16_t alpha, beta;
    q16_t da, db, dc;

    esp_foc_modulate_dq_to_duties(sin_t, cos_t, vd, vq,
                                  &alpha, &beta, &da, &db, &dc,
                                  ESP_FOC_MOD_INDEX_LIMIT_Q16);

    float duty_a = q16_to_float(da);
    float duty_b = q16_to_float(db);
    float duty_c = q16_to_float(dc);

    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.5f, duty_a);
    TEST_ASSERT_TRUE(duty_b >= 0.0f && duty_b <= 1.0f);
    TEST_ASSERT_TRUE(duty_c >= 0.0f && duty_c <= 1.0f);
}
