/*
 * Unit tests for rotor_sensor_simu (open-loop motor + encoder mimic).
 */
#include "sdkconfig.h"
#if !CONFIG_ESP_FOC_FITL

#include <unity.h>
#include <math.h>
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/rotor_sensor_simu.h"

#define MANY_STEPS 5000

TEST_CASE("rotor_sensor_simu: CPR is 4096", "[espFoC][rotor_simu]")
{
    q16_t vdc = q16_from_float(48.0f);
    esp_foc_rotor_sensor_t *s =
        rotor_sensor_simu_new(0, 4, 1.0f, 0.002f, vdc);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_EQUAL(4096u, s->get_counts_per_revolution(s));
}

TEST_CASE("rotor_sensor_simu: unwired u stays at zero angle", "[espFoC][rotor_simu]")
{
    q16_t vdc = q16_from_float(48.0f);
    esp_foc_rotor_sensor_t *s =
        rotor_sensor_simu_new(0, 4, 1.0f, 0.002f, vdc);
    TEST_ASSERT_NOT_NULL(s);
    rotor_sensor_simu_wire_ud_uq(s, NULL, NULL);
    for (int i = 0; i < 50; ++i) {
        q16_t c = s->read_counts(s);
        TEST_ASSERT_INT_WITHIN(1, 0, (int)q16_to_float(c));
    }
}

TEST_CASE("rotor_sensor_simu: wired uq produces motion", "[espFoC][rotor_simu]")
{
    q16_t vdc = q16_from_float(48.0f);
    esp_foc_rotor_sensor_t *s =
        rotor_sensor_simu_new(0, 4, 0.5f, 0.001f, vdc);
    TEST_ASSERT_NOT_NULL(s);
    q16_t ud = q16_from_float(0.0f);
    q16_t uq = q16_from_float(0.15f);
    rotor_sensor_simu_wire_ud_uq(s, &ud, &uq);
    q16_t c0 = s->read_counts(s);
    for (int i = 0; i < MANY_STEPS; ++i) {
        (void)s->read_counts(s);
    }
    q16_t c1 = s->read_counts(s);
    int t0 = (int)(q16_to_float(c0) + 0.5f);
    int t1 = (int)(q16_to_float(c1) + 0.5f);
    TEST_ASSERT_NOT_EQUAL(t0, t1);
}

TEST_CASE("rotor_sensor_simu: set_to_zero clears state", "[espFoC][rotor_simu]")
{
    q16_t vdc = q16_from_float(48.0f);
    esp_foc_rotor_sensor_t *s =
        rotor_sensor_simu_new(0, 4, 0.5f, 0.001f, vdc);
    TEST_ASSERT_NOT_NULL(s);
    q16_t ud = q16_from_float(0.0f);
    q16_t uq = q16_from_float(0.2f);
    rotor_sensor_simu_wire_ud_uq(s, &ud, &uq);
    for (int i = 0; i < 2000; ++i) {
        (void)s->read_counts(s);
    }
    s->set_to_zero(s);
    q16_t c = s->read_counts(s);
    TEST_ASSERT_INT_WITHIN(2, 0, (int)(q16_to_float(c) + 0.5f));
}

TEST_CASE("rotor_sensor_simu: accumulated tracks unwrap", "[espFoC][rotor_simu]")
{
    q16_t vdc = q16_from_float(48.0f);
    esp_foc_rotor_sensor_t *s =
        rotor_sensor_simu_new(0, 4, 0.5f, 0.001f, vdc);
    TEST_ASSERT_NOT_NULL(s);
    q16_t ud = q16_from_float(0.0f);
    q16_t uq = q16_from_float(0.25f);
    rotor_sensor_simu_wire_ud_uq(s, &ud, &uq);
    int64_t a0 = s->read_accumulated_counts_i64(s);
    for (int i = 0; i < 20000; ++i) {
        (void)s->read_counts(s);
    }
    int64_t a1 = s->read_accumulated_counts_i64(s);
    TEST_ASSERT_TRUE(a1 > a0);
}

#endif /* !CONFIG_ESP_FOC_FITL */
