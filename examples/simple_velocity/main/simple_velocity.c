#include "esp_log.h"
#include "esp_err.h"

#include "espFoC/rotor_sensor_as5600.h"
#include "espFoC/inverter_3pwm_ledc.h"
#include "espFoC/esp_foc.h"

static const char *TAG = "esp-foc-example";

static esp_foc_inverter_t *inverter;
static esp_foc_rotor_sensor_t *sensor;
static esp_foc_axis_t axis;
static esp_foc_motor_control_settings_t settings = {
    .downsampling_position_rate = 0, // No position control,
    .downsampling_speed_rate = 0, //No speed control
    .motor_pole_pairs = 7, //Assuming HT2250 motor
    .velocity_control_settings.kp = 1.0f,
    .velocity_control_settings.ki = 0.0f,
    .velocity_control_settings.kd = 0.0f,
    .velocity_control_settings.integrator_limit = 100.0f,
    .velocity_control_settings.max_output_value = 4.0f, //conservative setpoint to the current controller
    .torque_control_settings[0].max_output_value = 6.0f, //Uses the max driver voltage allowed as limit
    .natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
};

static void initialize_foc_drivers(void) 
{

    inverter = inverter_3pwm_ledc_new(
        LEDC_CHANNEL_0,
        LEDC_CHANNEL_1,
        LEDC_CHANNEL_2,
        CONFIG_FOC_PWM_U_PIN,
        CONFIG_FOC_PWM_V_PIN,
        CONFIG_FOC_PWM_W_PIN,
        12.0f,
        0
    );

    if(inverter == NULL) {
        ESP_LOGE(TAG, "failed to create the inverter driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }

    sensor = rotor_sensor_as5600_new(
        CONFIG_FOC_ENCODER_SDA_PIN,
        CONFIG_FOC_ENCODER_SCL_PIN,
        0
    );

    if(sensor == NULL) {
        ESP_LOGE(TAG, "failed to create the inverter driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }

}

void app_main(void)
{
    esp_foc_control_data_t control_data;

    ESP_LOGI(TAG, "Initializing the esp foc motor controller!");

    initialize_foc_drivers();

    esp_foc_initialize_axis(
        &axis,
        inverter,
        sensor,
        settings
    );

    esp_foc_align_axis(&axis);
    esp_foc_run(&axis);

    /* Set velocity by using state vector given by vq+vd */
    esp_foc_set_target_voltage(&axis, (esp_foc_q_voltage){.raw = 4.0}, (esp_foc_d_voltage){.raw = 0.0});

    while(1) {
        esp_foc_sleep_ms(100);
        esp_foc_get_control_data(&axis, &control_data);        
        ESP_LOGI(TAG, "Current quadrature voltage: %f [V]", control_data.out_q.raw);
        ESP_LOGI(TAG, "Current direct voltage: %f [V]", control_data.out_d.raw);
        ESP_LOGI(TAG, "Current mechanical position: %f [rad]", control_data.position.raw);
    }
}