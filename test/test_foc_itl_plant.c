/*
 * Unit tests for esp_foc_itl_plant (pure physics, no FITL driver).
 */
#include "sdkconfig.h"
#if CONFIG_ESP_FOC_FITL

#include <math.h>
#include <unity.h>

#include "esp_foc_itl_plant.h"
#include "espFoC/utils/esp_foc_q16.h"

static esp_foc_itl_plant_t s_plant;

static void init_locked_plant(float r, float l, float vdc)
{
    esp_foc_itl_plant_params_t p = {
        .r_ohm = r,
        .l_henry = l,
        .j_kgm2 = 1e-4f,
        .b_nms = 1e-5f,
        .kt_nm_per_a = 1e-4f,
        .pole_pairs = 7,
        .vdc_volts = vdc,
        .i_max_a = 20.0f,
        .locked_rotor = true,
        .delta_connection = false,
    };
    esp_foc_itl_plant_init(&s_plant, &p);
    esp_foc_itl_plant_set_dt(&s_plant, 20000);
}

TEST_CASE("plant: locked rotor reaches steady-state current", "[espFoC][foc_itl_plant]")
{
    init_locked_plant(1.0f, 0.002f, 12.0f);
    const q16_t vu = q16_mul(q16_sub(q16_from_float(0.55f), Q16_HALF), q16_from_float(12.0f));
    const q16_t target = q16_mul(vu, q16_reciprocal_positive(q16_from_float(1.0f)));

    for (int i = 0; i < 8000; i++) {
        esp_foc_itl_plant_step(&s_plant, vu, 0, q16_sub(0, vu));
    }

    TEST_ASSERT_INT32_WITHIN(q16_from_float(0.5f), target, s_plant.iu_q16);
    TEST_ASSERT_INT32_WITHIN(q16_from_float(0.05f), 0, s_plant.omega_m_q16);
}

TEST_CASE("plant: current saturation clamps phase current", "[espFoC][foc_itl_plant]")
{
    init_locked_plant(0.01f, 0.0001f, 48.0f);
    s_plant.c.i_max_q16 = q16_from_float(2.0f);

    for (int i = 0; i < 4000; i++) {
        esp_foc_itl_plant_step(&s_plant, q16_from_float(20.0f), q16_from_float(-10.0f),
                               q16_from_float(-10.0f));
    }

    TEST_ASSERT_LESS_OR_EQUAL(q16_from_float(2.01f), q16_abs(s_plant.iu_q16));
    TEST_ASSERT_LESS_OR_EQUAL(q16_from_float(2.01f), q16_abs(s_plant.iv_q16));
}

TEST_CASE("plant: free rotor step response does not diverge", "[espFoC][foc_itl_plant]")
{
    esp_foc_itl_plant_params_t p = {
        .r_ohm = 1.0f,
        .l_henry = 0.002f,
        .j_kgm2 = 5e-5f,
        .b_nms = 1e-5f,
        .kt_nm_per_a = 1e-4f,
        .pole_pairs = 7,
        .vdc_volts = 12.0f,
        .i_max_a = 10.0f,
        .locked_rotor = false,
        .delta_connection = false,
    };
    esp_foc_itl_plant_init(&s_plant, &p);
    esp_foc_itl_plant_set_dt(&s_plant, 20000);

    for (int i = 0; i < 20000; i++) {
        esp_foc_itl_plant_step(&s_plant, q16_from_float(1.0f), q16_from_float(-0.5f),
                               q16_from_float(-0.5f));
    }

    TEST_ASSERT_LESS_OR_EQUAL(q16_from_float(10.01f), q16_abs(s_plant.iu_q16));
    TEST_ASSERT(s_plant.omega_m_q16 != (q16_t)INT32_MIN);
    TEST_ASSERT(s_plant.theta_m_q16 >= 0);
}

TEST_CASE("plant: encoder ticks wrap at CPR", "[espFoC][foc_itl_plant]")
{
    init_locked_plant(1.0f, 0.002f, 12.0f);
    s_plant.theta_m_q16 = q16_from_float(0.99f);
    const int32_t ticks = esp_foc_itl_plant_encoder_ticks(&s_plant);
    TEST_ASSERT_GREATER_OR_EQUAL(0, ticks);
    TEST_ASSERT_LESS_THAN((int)ESP_FOC_ITL_CPR, ticks);
}

#endif /* CONFIG_ESP_FOC_FITL */
