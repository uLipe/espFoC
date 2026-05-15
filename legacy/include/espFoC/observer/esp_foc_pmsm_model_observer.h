// esp_foc_pmsm_model_observer.c
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

#include <espFoC/esp_foc.h>

typedef struct {
    // Electrical parameters (per-phase dq model)
    float phase_resistance;   // R [ohm]
    float Ld;                 // [H]
    float Lq;                 // [H]
    float lambda;             // flux linkage [Wb] (psi_f)

    // Mechanical parameters
    float inertia;            // J [kg*m^2]
    float friction;           // B [N*m*s/rad] (>=0)
    float load_torque;        // Tl [N*m] constant load (can be 0)

    // Motor geometry
    float pole_pairs;         // p

    // Execution
    float dt;                 // observer dt [s]

    // Optional clamps (0 disables)
    float current_limit;      // [A]
    float omega_limit;        // mechanical rad/s
} esp_foc_pmsm_model_observer_settings_t;

esp_foc_observer_t *pmsm_model_observer_new(int unit, esp_foc_pmsm_model_observer_settings_t settings);