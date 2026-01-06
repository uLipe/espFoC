// /*
//  * MIT License
//  *
//  * Copyright (c) 2021 Felipe Neves
//  *
//  * Permission is hereby granted, free of charge, to any person obtaining a copy
//  * of this software and associated documentation files (the "Software"), to deal
//  * in the Software without restriction, including without limitation the rights
//  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  * copies of the Software, and to permit persons to whom the Software is
//  * furnished to do so, subject to the following conditions:
//  *
//  * The above copyright notice and this permission notice shall be included in all
//  * copies or substantial portions of the Software.
//  *
//  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  * SOFTWARE.
//  */

use std::thread;
use std::time::Duration;

use esp_foc::{Axis, Inverter, default_settings_open_loop};

fn main() {
    // NOTE: adjust pins for your hardware.
    const GPIO_U: i32 = 18;
    const GPIO_V: i32 = 17;
    const GPIO_W: i32 = 16;
    const GPIO_EN: i32 = 15;

    const DC_LINK_VOLTAGE: f32 = 24.0;
    const MCPWM_PORT: i32 = 0;

    // Open-loop: no rotor sensor, no current sensor (you can add later).
    let inverter = Inverter::mcpwm_3pwm(GPIO_U, GPIO_V, GPIO_W, GPIO_EN, DC_LINK_VOLTAGE, MCPWM_PORT);

    let mut axis = Axis::new();
    let settings = default_settings_open_loop(7);

    axis.initialize(inverter, None, None, &settings).expect("axis init failed");

    // For true open-loop bring-up you typically skip align, but it is safe to call if implemented.
    // If your open-loop strategy doesn't need alignment, you can comment this out.
    let _ = axis.align();

    axis.run().expect("axis run failed");

    // Voltage ramp on Vq (Ud = 0): go up and down forever.
    let mut vq: f32 = 0.0;
    let mut step: f32 = 0.05; // volts per step
    let vq_max: f32 = 2.0;

    loop {
        axis.set_voltage(vq, 0.0).ok();

        vq += step;
        if vq >= vq_max {
            vq = vq_max;
            step = -step;
        } else if vq <= -vq_max {
            vq = -vq_max;
            step = -step;
        }

        thread::sleep(Duration::from_millis(10));
    }
}
