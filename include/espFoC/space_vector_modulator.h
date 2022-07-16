#pragma once

#include <math.h>

#define SQRT_3 1.7320508075688773f
static const float sqrt3_by_two = SQRT_3 / 2.0f;

static inline uint8_t esp_foc_svm_get_sector(float a, float b, float c)
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

static inline void esp_foc_svm_set(float v_alpha, float v_beta, float *dca, float *dcb, float *dcc)
{
	float a, b, c, mod;
	float x, y, z;

	/* normalize and limit alpha-beta vector */
	mod = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
	if (mod > sqrt3_by_two) {
		v_alpha = v_alpha / mod * sqrt3_by_two;
		v_beta = v_beta / mod * sqrt3_by_two;
	}

	a = v_alpha;
	b = 0.5f * (-v_alpha - SQRT_3 * v_beta);
	c = 0.5f * (-v_alpha + SQRT_3 * v_beta); 

	switch (esp_foc_svm_get_sector(a, b, c)) {
	case 1U:
		x = a;
		y = b;
		z = 1.0f - (x + y);

		*dca = x + y + z * 0.5f;
		*dcb = y + z * 0.5f;
		*dcc = z * 0.5f;

		break;
	case 2U:
		x = -c;
		y = -a;
		z = 1.0f - (x + y);

		*dca = x + z * 0.5f;
		*dcb = x + y + z * 0.5f;
		*dcc = z * 0.5f;

		break;
	case 3U:
		x = b;
		y = c;
		z = 1.0f - (x + y);

		*dca = z * 0.5f;
		*dcb = x + y + z * 0.5f;
		*dcc = y + z * 0.5f;

		break;
	case 4U:
		x = -a;
		y = -b;
		z = 1.0f - (x + y);

		*dca = z * 0.5f;
		*dcb = x + z * 0.5f;
		*dcc = x + y + z * 0.5f;

		break;
	case 5U:
		x = c;
		y = a;
		z = 1.0f - (x + y);

		*dca = y + z * 0.5f;
		*dcb = z * 0.5f;
		*dcc = x + y + z * 0.5f;

		break;
	case 6U:
		x = -b;
		y = -c;
		z = 1.0f - (x + y);

		*dca = x + y + z * 0.5f;
		*dcb = z * 0.5f;
		*dcc = x + z * 0.5f;

		break;
	default:
		break;
	}
}