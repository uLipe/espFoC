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

static inline void esp_foc_modulate_dq_voltage (float theta,
                                            float v_d,
                                            float v_q,
                                            float *v_u,
                                            float *v_v,
                                            float *v_w)
{
    float dq_frame[2] = {v_d, v_q};
    float ab_frame[2];

    esp_foc_inverse_park_transform(theta, dq_frame, &ab_frame[0], &ab_frame[1]);
    esp_foc_svm_set(ab_frame[0], ab_frame[1], v_u, v_v, v_w);
}

static inline void esp_foc_get_dq_currents(float theta,
                                        float i_u, 
                                        float i_v, 
                                        float i_w, 
                                        float *i_q, 
                                        float *i_d) {
    
    float phase_current_frame[3] = {i_u, i_v, i_w};
    float ab_frame[2];
    esp_foc_clarke_transform(phase_current_frame,&ab_frame[0], &ab_frame[1]);
    esp_foc_park_transform(theta, ab_frame, i_d, i_q);
}
