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

#include <errno.h>
#include <espFoC/motor_control/esp_foc_pid.h>

int esp_foc_pid_init(struct esp_foc_pid *p, float sample_time_seconds, float integral_limit)
{
    if(!p)
        return -EINVAL;

    if(sample_time_seconds == 0.0f)
        return -EINVAL;

    if(integral_limit == 0.0f)
        return -EINVAL;

    p->limit_high = integral_limit;
    p->limit_low = -integral_limit;
    p->sample_time = sample_time_seconds;
    p->x[0] = 0.0f;
    p->x[1] = 0.0f;
    p->x[2] = 0.0f;
    p->k[0] = 0.0f;
    p->k[1] = 0.0f;
    p->k[2] = 0.0f;

    /* Make a PID controller with KP = 1 providing an
     * unit gain controller
     */
    return esp_foc_pid_set_kp(p, 1.0f);
}

int esp_foc_pid_set_kp(struct esp_foc_pid *p, float kp)
{
    if(!p)
        return -EINVAL;

    p->k[0] = kp;
    p->b[0] = p->k[0] + p->k[1] + p->k[2];
    p->b[1] = -p->k[0] + p->k[1] + (2 * p->k[2]);
    p->b[2] = p->k[2];

    return 0;
}

int esp_foc_pid_set_ki(struct esp_foc_pid *p, float ki)
{
    if(!p)
        return -EINVAL;

    p->k[1] = p->sample_time * 0.5f * ki;
    p->b[0] = p->k[0] + p->k[1] + p->k[2];
    p->b[1] = -p->k[0] + p->k[1] + (2 * p->k[2]);
    p->b[2] = p->k[2];

    return 0;

}

int esp_foc_pid_set_kd(struct esp_foc_pid *p, float kd)
{
    if(!p)
        return -EINVAL;

    p->k[2] = (1.0f / p->sample_time) * kd;
    p->b[0] = p->k[0] + p->k[1] + p->k[2];
    p->b[1] = -p->k[0] + p->k[1] + (2 * p->k[2]);
    p->b[2] = p->k[2];

    return 0;

}

int esp_foc_pid_update(struct esp_foc_pid *p, float ref, float mes, float *command)
{
    if(!p)
        return -EINVAL;

    if(!command)
        return -EINVAL;

    float x1 = p->x[0];
    float x2 = p->x[1];
    float x3 = p->x[2];
    float b1 = p->b[0];
    float b2 = p->b[1];
    float b3 = p->b[2];
    float y = p->y;
    float limit_low = p->limit_low;
    float limit_high = p->limit_high;
    float u = y + x1 * b1 + x2 * b2 + x3 * b3;

    if(u > limit_high) {
        u = limit_high;
    } else if(u < limit_low) {
        u = limit_low;
    }

    p->y = -u;
    p->x[2] = p->x[1];
    p->x[1] = p->x[0];
    p->x[0] = x1;

    *command = u;

    return 0;
}