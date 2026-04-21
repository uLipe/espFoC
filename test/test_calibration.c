/*
 * MIT License
 *
 * Tests for esp_foc_calibration (NVS overlay).
 *
 * Each test starts by erasing the calibration namespace so the cases
 * stay independent of test order. The unit_test_runner ships with the
 * default partition table that already includes "nvs", so no extra
 * setup is needed in QEMU.
 */

#include <string.h>
#include <unity.h>
#include "espFoC/esp_foc_calibration.h"
#include "espFoC/utils/esp_foc_q16.h"

#if defined(CONFIG_ESP_FOC_CALIBRATION_NVS)

static esp_foc_calibration_data_t make_payload(float kp, float ki, float lim,
                                               float r, float l, float bw)
{
    esp_foc_calibration_data_t d = {0};
    d.kp = q16_from_float(kp);
    d.ki = q16_from_float(ki);
    d.integrator_limit = q16_from_float(lim);
    d.motor_r_ohm = q16_from_float(r);
    d.motor_l_h = q16_from_float(l);
    d.bandwidth_hz = q16_from_float(bw);
    return d;
}

TEST_CASE("calibration: empty NVS reports absent",
          "[espFoC][calibration]")
{
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_calibration_erase());
    TEST_ASSERT_FALSE(esp_foc_calibration_present(0));
}

TEST_CASE("calibration: save then load round-trip",
          "[espFoC][calibration]")
{
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_calibration_erase());

    esp_foc_calibration_data_t in = make_payload(
        1.234f, 567.89f, 12.0f, 1.08f, 0.0018f, 150.0f);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_calibration_save(0, &in));
    TEST_ASSERT_TRUE(esp_foc_calibration_present(0));

    esp_foc_calibration_data_t out = {0};
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_calibration_load(0, &out));
    TEST_ASSERT_EQUAL_INT32(in.kp, out.kp);
    TEST_ASSERT_EQUAL_INT32(in.ki, out.ki);
    TEST_ASSERT_EQUAL_INT32(in.integrator_limit, out.integrator_limit);
    TEST_ASSERT_EQUAL_INT32(in.motor_r_ohm, out.motor_r_ohm);
    TEST_ASSERT_EQUAL_INT32(in.motor_l_h, out.motor_l_h);
    TEST_ASSERT_EQUAL_INT32(in.bandwidth_hz, out.bandwidth_hz);
}

TEST_CASE("calibration: per-axis isolation",
          "[espFoC][calibration]")
{
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_calibration_erase());

    esp_foc_calibration_data_t a = make_payload(1.0f, 100.0f, 1.0f,
                                                1.0f, 0.001f, 100.0f);
    esp_foc_calibration_data_t b = make_payload(2.0f, 200.0f, 2.0f,
                                                2.0f, 0.002f, 200.0f);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_calibration_save(0, &a));
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_calibration_save(1, &b));

    esp_foc_calibration_data_t got;
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_calibration_load(0, &got));
    TEST_ASSERT_EQUAL_INT32(a.kp, got.kp);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_calibration_load(1, &got));
    TEST_ASSERT_EQUAL_INT32(b.kp, got.kp);
}

TEST_CASE("calibration: erase wipes every axis",
          "[espFoC][calibration]")
{
    esp_foc_calibration_data_t d = make_payload(1.0f, 1.0f, 1.0f,
                                                1.0f, 1.0f, 1.0f);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_calibration_save(0, &d));
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_calibration_save(1, &d));
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_calibration_erase());
    TEST_ASSERT_FALSE(esp_foc_calibration_present(0));
    TEST_ASSERT_FALSE(esp_foc_calibration_present(1));
}

TEST_CASE("calibration: rejects out-of-range axis id",
          "[espFoC][calibration]")
{
    esp_foc_calibration_data_t d = make_payload(1.0f, 1.0f, 1.0f,
                                                1.0f, 1.0f, 1.0f);
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG,
                      esp_foc_calibration_save(99, &d));
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG,
                      esp_foc_calibration_load(99, &d));
}

TEST_CASE("calibration: profile_hash is stable per build",
          "[espFoC][calibration]")
{
    uint32_t a = esp_foc_calibration_profile_hash();
    uint32_t b = esp_foc_calibration_profile_hash();
    TEST_ASSERT_EQUAL_HEX32(a, b);
    TEST_ASSERT_NOT_EQUAL(0u, a);
}

#endif /* CONFIG_ESP_FOC_CALIBRATION_NVS */
