/*
 * Unit tests for esp_foc_itl_plant (pure physics, no FITL driver).
 */
#include "sdkconfig.h"
#if CONFIG_ESP_FOC_FITL

#include <math.h>
#include <unity.h>

#include "esp_foc_itl_plant.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
}

TEST_CASE("plant: locked rotor reaches steady-state current", "[espFoC][foc_itl_plant]")
{
    init_locked_plant(1.0f, 0.002f, 12.0f);
    const float dt = 1.0f / 20000.0f;
    const float duty = 0.55f;
    const float vu = (duty - 0.5f) * 12.0f;
    const float target = vu / 1.0f;

    for (int i = 0; i < 8000; i++) {
        esp_foc_itl_plant_step(&s_plant, vu, 0.0f, -vu, dt);
    }

    TEST_ASSERT_FLOAT_WITHIN(0.5f, target, s_plant.iu_a);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, s_plant.omega_m_rad_s);
}

TEST_CASE("plant: current saturation clamps phase current", "[espFoC][foc_itl_plant]")
{
    init_locked_plant(0.01f, 0.0001f, 48.0f);
    s_plant.p.i_max_a = 2.0f;
    const float dt = 1.0f / 20000.0f;

    for (int i = 0; i < 4000; i++) {
        esp_foc_itl_plant_step(&s_plant, 20.0f, -10.0f, -10.0f, dt);
    }

    TEST_ASSERT_LESS_OR_EQUAL(2.01f, fabsf(s_plant.iu_a));
    TEST_ASSERT_LESS_OR_EQUAL(2.01f, fabsf(s_plant.iv_a));
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

    const float dt = 1.0f / 20000.0f;
    for (int i = 0; i < 20000; i++) {
        esp_foc_itl_plant_step(&s_plant, 1.0f, -0.5f, -0.5f, dt);
    }

    TEST_ASSERT_LESS_OR_EQUAL(10.01f, fabsf(s_plant.iu_a));
    TEST_ASSERT(isfinite(s_plant.omega_m_rad_s));
    TEST_ASSERT(isfinite(s_plant.theta_m_rad));
}

TEST_CASE("plant: encoder ticks wrap at CPR", "[espFoC][foc_itl_plant]")
{
    init_locked_plant(1.0f, 0.002f, 12.0f);
    s_plant.theta_m_rad = (float)(2.0 * M_PI * 0.99);
    const int32_t ticks = esp_foc_itl_plant_encoder_ticks(&s_plant);
    TEST_ASSERT_GREATER_OR_EQUAL(0, ticks);
    TEST_ASSERT_LESS_THAN((int)ESP_FOC_ITL_CPR, ticks);
}

#endif /* CONFIG_ESP_FOC_FITL */
