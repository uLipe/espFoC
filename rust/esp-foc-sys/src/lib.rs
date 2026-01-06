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

#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]

use core::ffi::c_void;

#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct esp_foc_control_settings_t {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub integrator_limit: f32,
    pub max_output_value: f32,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct esp_foc_q_voltage { pub raw: f32 }
#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct esp_foc_d_voltage { pub raw: f32 }
#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct esp_foc_alpha_voltage { pub raw: f32 }
#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct esp_foc_beta_voltage { pub raw: f32 }
#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct esp_foc_u_voltage { pub raw: f32 }
#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct esp_foc_v_voltage { pub raw: f32 }
#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct esp_foc_w_voltage { pub raw: f32 }

#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct esp_foc_q_current { pub raw: f32 }
#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct esp_foc_d_current { pub raw: f32 }
#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct esp_foc_alpha_current { pub raw: f32 }
#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct esp_foc_beta_current { pub raw: f32 }
#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct esp_foc_u_current { pub raw: f32 }
#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct esp_foc_v_current { pub raw: f32 }
#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct esp_foc_w_current { pub raw: f32 }

#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct esp_foc_radians_per_second { pub raw: f32 }
#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct esp_foc_radians { pub raw: f32 }
#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct esp_foc_seconds { pub raw: f32 }

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub enum esp_foc_motor_direction_t {
    ESP_FOC_MOTOR_NATURAL_DIRECTION_CW = 0,
    ESP_FOC_MOTOR_NATURAL_DIRECTION_CCW = 1,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct esp_foc_motor_control_settings_t {
    pub torque_control_settings: [esp_foc_control_settings_t; 2],
    pub velocity_control_settings: esp_foc_control_settings_t,
    pub position_control_settings: esp_foc_control_settings_t,
    pub natural_direction: esp_foc_motor_direction_t,
    pub enable_position_control: bool,
    pub enable_velocity_control: bool,
    pub enable_torque_control: bool,
    pub motor_pole_pairs: i32,
    pub motor_resistance: f32,
    pub motor_inductance: f32,
    pub motor_inertia: f32,
    pub flux_linkage: f32,
    pub inertia: f32,
    pub friction: f32,
    pub motor_unit: i32,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct esp_foc_control_data_t {
    pub timestamp: esp_foc_seconds,
    pub dt: esp_foc_seconds,

    pub u: esp_foc_u_voltage,
    pub v: esp_foc_v_voltage,
    pub w: esp_foc_w_voltage,

    pub out_q: esp_foc_q_voltage,
    pub out_d: esp_foc_d_voltage,

    pub i_u: esp_foc_u_current,
    pub i_v: esp_foc_v_current,
    pub i_w: esp_foc_w_current,

    pub i_q: esp_foc_q_current,
    pub i_d: esp_foc_d_current,

    pub u_alpha: esp_foc_alpha_voltage,
    pub u_beta: esp_foc_beta_voltage,

    pub i_alpha: esp_foc_alpha_current,
    pub i_beta: esp_foc_beta_current,

    pub rotor_position: esp_foc_radians,
    pub extrapolated_rotor_position: esp_foc_radians,
    pub position: esp_foc_radians,
    pub speed: esp_foc_radians_per_second,

    pub target_position: esp_foc_radians,
    pub target_speed: esp_foc_radians_per_second,
    pub observer_angle: esp_foc_radians_per_second,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub enum esp_foc_err_t {
    ESP_FOC_OK = 0,
    ESP_FOC_ERR_NOT_ALIGNED = -1,
    ESP_FOC_ERR_INVALID_ARG = -2,
    ESP_FOC_ERR_AXIS_INVALID_STATE = -3,
    ESP_FOC_ERR_ALIGNMENT_IN_PROGRESS = -4,
    ESP_FOC_ERR_TIMESTEP_TOO_SMALL = -5,
    ESP_FOC_ERR_UNKNOWN = -128,
}

pub type espfoc_axis_handle_t = *mut c_void;
pub type espfoc_inverter_handle_t = *mut c_void;
pub type espfoc_rotor_handle_t = *mut c_void;
pub type espfoc_isensor_handle_t = *mut c_void;

#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct espfoc_isensor_adc_config_t {
    pub axis_channels: [i32; 4],
    pub units: [i32; 4],
    pub amp_gain: f32,
    pub shunt_resistance: f32,
    pub number_of_channels: i32,
}

extern "C" {
    pub fn espfoc_axis_alloc() -> espfoc_axis_handle_t;
    pub fn espfoc_axis_free(axis: espfoc_axis_handle_t);

    pub fn espfoc_inverter_3pwm_mcpwm_new(
        gpio_u: i32,
        gpio_v: i32,
        gpio_w: i32,
        gpio_enable: i32,
        dc_link_voltage: f32,
        port: i32,
    ) -> espfoc_inverter_handle_t;

    pub fn espfoc_inverter_6pwm_mcpwm_new(
        gpio_u_high: i32, gpio_u_low: i32,
        gpio_v_high: i32, gpio_v_low: i32,
        gpio_w_high: i32, gpio_w_low: i32,
        gpio_enable: i32,
        dc_link_voltage: f32,
        port: i32,
    ) -> espfoc_inverter_handle_t;

    pub fn espfoc_rotor_sensor_as5600_new(i2c_port: i32, i2c_address: u8) -> espfoc_rotor_handle_t;

    pub fn espfoc_isensor_adc_new(config: *const espfoc_isensor_adc_config_t) -> espfoc_isensor_handle_t;

    pub fn espfoc_initialize_axis(
        axis: espfoc_axis_handle_t,
        inverter: espfoc_inverter_handle_t,
        rotor_sensor: espfoc_rotor_handle_t,
        isensor: espfoc_isensor_handle_t,
        settings: *const esp_foc_motor_control_settings_t,
    ) -> esp_foc_err_t;

    pub fn espfoc_align_axis(axis: espfoc_axis_handle_t) -> esp_foc_err_t;
    pub fn espfoc_run_axis(axis: espfoc_axis_handle_t) -> esp_foc_err_t;

    pub fn espfoc_set_target_voltage(axis: espfoc_axis_handle_t, uq: f32, ud: f32) -> esp_foc_err_t;
    pub fn espfoc_set_target_speed(axis: espfoc_axis_handle_t, speed: f32) -> esp_foc_err_t;
    pub fn espfoc_set_target_position(axis: espfoc_axis_handle_t, position: f32) -> esp_foc_err_t;
    pub fn espfoc_set_target_current(axis: espfoc_axis_handle_t, iq: f32, id: f32) -> esp_foc_err_t;

    pub fn espfoc_get_control_data(axis: espfoc_axis_handle_t, out: *mut esp_foc_control_data_t) -> esp_foc_err_t;
}
