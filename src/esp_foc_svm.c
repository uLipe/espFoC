/*
 * Copyright (c) 2021 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <espFoC/esp_foc.h>

extern const float ESP_FOC_SQRT3;
extern const float ESP_FOC_1_SQRT3;
extern const float ESP_FOC_SQRT3_2;
extern const float ESP_FOC_2_SQRT3;

static uint8_t get_sector(float a, float b, float c)
{
	uint8_t sector = 0U;

	if (c < 0.0f) {
		if (a < 0.0f) {
			sector = 2U;
		} else {
			if (b < 0.0f) {
				sector = 6U;
			} else {
				sector = 1U;
			}
		}
	} else {
		if (a < 0.0f) {
			if (b <= 0.0f) {
				sector = 4U;
			} else {
				sector = 3U;
			}
		} else {
			sector = 5U;
		}
	}

	return sector;
}

int esp_foc_do_space_vector_pwm(const float v_alpha,
                        const float v_beta,
                        float *u,
                        float *v,
                        float *w)
{
	float a, b, c, mod;
	float x, y, z;

    if(!u || !v || !w)
        return -EINVAL;

	/* normalize and limit alpha-beta vector */
	mod = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
	if (mod > ESP_FOC_SQRT3_2) {
		v_alpha = v_alpha / mod * ESP_FOC_SQRT3_2;
		v_beta = v_beta / mod * ESP_FOC_SQRT3_2;
	}

	/* do a modified inverse clarke transform to get an auxiliary frame
	 * to compute the sector we are
	 */


	a = v_alpha - ESP_FOC_1_SQRT3 * v_beta;
	b = ESP_FOC_2_SQRT3 * v_beta;
	c = -(a + b);

	switch (get_sector(a, b, c)) {
	case 1U:
		x = a;
		y = b;
		z = 1.0f - (x + y);

		*u = x + y + z * 0.5f;
		*v = y + z * 0.5f;
		*w = z * 0.5f;

		break;
	case 2U:
		x = -c;
		y = -a;
		z = 1.0f - (x + y);

		*u = x + z * 0.5f;
		*v = x + y + z * 0.5f;
		*w = z * 0.5f;

		break;
	case 3U:
		x = b;
		y = c;
		z = 1.0f - (x + y);

		*u = z * 0.5f;
		*v = x + y + z * 0.5f;
		*w = y + z * 0.5f;

		break;
	case 4U:
		x = -a;
		y = -b;
		z = 1.0f - (x + y);

		*u = z * 0.5f;
		*v = x + z * 0.5f;
		*w = x + y + z * 0.5f;

		break;
	case 5U:
		x = c;
		y = a;
		z = 1.0f - (x + y);

		*u = y + z * 0.5f;
		*v = z * 0.5f;
		*w = x + y + z * 0.5f;

		break;
	case 6U:
		x = -b;
		y = -c;
		z = 1.0f - (x + y);

		*u = x + y + z * 0.5f;
		*v = z * 0.5f;
		*w = x + z * 0.5f;

		break;
	default:
		break;
	}

    return 0;
}