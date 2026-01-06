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

use esp_foc_sys as sys;

#[derive(Debug, Copy, Clone)]
pub struct Error(pub sys::esp_foc_err_t);

pub type Result<T> = core::result::Result<T, Error>;

fn ok(err: sys::esp_foc_err_t) -> Result<()> {
    match err {
        sys::esp_foc_err_t::ESP_FOC_OK => Ok(()),
        other => Err(Error(other)),
    }
}

pub struct Axis {
    h: sys::espfoc_axis_handle_t,
}

impl Axis {
    pub fn new() -> Self {
        let h = unsafe { sys::espfoc_axis_alloc() };
        Self { h }
    }

    pub fn initialize(
        &mut self,
        inverter: sys::espfoc_inverter_handle_t,
        rotor: Option<sys::espfoc_rotor_handle_t>,
        isensor: Option<sys::espfoc_isensor_handle_t>,
        settings: &sys::esp_foc_motor_control_settings_t,
    ) -> Result<()> {
        let err = unsafe {
            sys::espfoc_initialize_axis(
                self.h,
                inverter,
                rotor.unwrap_or(core::ptr::null_mut()),
                isensor.unwrap_or(core::ptr::null_mut()),
                settings as *const _,
            )
        };
        ok(err)
    }

    pub fn align(&mut self) -> Result<()> {
        ok(unsafe { sys::espfoc_align_axis(self.h) })
    }

    pub fn run(&mut self) -> Result<()> {
        ok(unsafe { sys::espfoc_run_axis(self.h) })
    }

    pub fn set_voltage(&mut self, uq: f32, ud: f32) -> Result<()> {
        ok(unsafe { sys::espfoc_set_target_voltage(self.h, uq, ud) })
    }
}

impl Drop for Axis {
    fn drop(&mut self) {
        unsafe { sys::espfoc_axis_free(self.h) };
    }
}

pub struct Inverter;
impl Inverter {
    pub fn mcpwm_3pwm(
        gpio_u: i32,
        gpio_v: i32,
        gpio_w: i32,
        gpio_enable: i32,
        dc_link_voltage: f32,
        port: i32,
    ) -> sys::espfoc_inverter_handle_t {
        unsafe { sys::espfoc_inverter_3pwm_mcpwm_new(gpio_u, gpio_v, gpio_w, gpio_enable, dc_link_voltage, port) }
    }

    pub fn mcpwm_6pwm(
        gpio_u_high: i32, gpio_u_low: i32,
        gpio_v_high: i32, gpio_v_low: i32,
        gpio_w_high: i32, gpio_w_low: i32,
        gpio_enable: i32,
        dc_link_voltage: f32,
        port: i32,
    ) -> sys::espfoc_inverter_handle_t {
        unsafe {
            sys::espfoc_inverter_6pwm_mcpwm_new(
                gpio_u_high, gpio_u_low,
                gpio_v_high, gpio_v_low,
                gpio_w_high, gpio_w_low,
                gpio_enable,
                dc_link_voltage,
                port,
            )
        }
    }
}

pub fn default_settings_open_loop(pole_pairs: i32) -> sys::esp_foc_motor_control_settings_t {
    // Conservative defaults; tune as needed.
    sys::esp_foc_motor_control_settings_t {
        enable_position_control: false,
        enable_velocity_control: false,
        enable_torque_control: false,
        motor_pole_pairs: pole_pairs,
        ..Default::default()
    }
}
