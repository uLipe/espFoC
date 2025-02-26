/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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
	float a, b, c;
	float x, y, z;

	/* normalize and limit alpha-beta vector */
	// mod = esp_foc_sqrtf(v_alpha * v_alpha + v_beta * v_beta);
	// if (mod > sqrt3_by_two) {
	// 	v_alpha = v_alpha / mod * sqrt3_by_two;
	// 	v_beta = v_beta / mod * sqrt3_by_two;
	// }

    // a = v_beta;
    // b = (SQRT_3 * v_alpha - v_beta) * 0.5f;
    // c = (-SQRT_3 * v_alpha - v_beta) * 0.5f;

	a = v_alpha;
	b = 0.5f * ((SQRT_3 * v_beta) - v_alpha);
	c =  -a - b;

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