/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include "espFoC/utils/esp_foc_ripple.h"

#include <math.h>

float esp_foc_ripple_sigma(const esp_foc_ripple_t *r, uint32_t min_n)
{
    const uint32_t n = r->n;
    if ((n < min_n) || (n < 3u)) {
        return 0.0f;
    }
    const double dn = (double)n;
    const double sx = (double)r->sum;
    const double sxx_c = (double)r->sq - sx * sx / dn;
    const double sii_c = dn * (dn * dn - 1.0) / 12.0;
    const double sxi_c = (double)r->ix - 0.5 * (dn - 1.0) * sx;
    double rss = sxx_c - ((sii_c > 0.0) ? (sxi_c * sxi_c / sii_c) : 0.0);
    if (rss < 0.0) {
        rss = 0.0;
    }
    return (float)(sqrt(rss / (dn - 2.0)) * (double)(1u << ESP_FOC_RIPPLE_SHIFT) / 65536.0);
}
