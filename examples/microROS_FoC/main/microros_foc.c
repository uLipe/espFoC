#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_err.h"

#include "espFoC/rotor_sensor_as5600.h"
#include "espFoC/inverter_3pwm_ledc.h"
#include "espFoC/esp_foc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG,"Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG,"Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

static const char *TAG = "ESP_FOC_MICROROS";

rcl_subscription_t subscriber;
std_msgs__msg__Float32 recv_msg;

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

void subscription_callback(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    ESP_LOGI(TAG, "Received new VQ command: %f [V]", msg->data);
    esp_foc_set_target_voltage(&axis, (esp_foc_q_voltage){.raw = msg->data}, (esp_foc_d_voltage){.raw = -3.0});
}

static void micro_ros_task(void * arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    esp_foc_align_axis(&axis);
    esp_foc_run(&axis);

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

    #ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // Static Agent IP and port can be used instead of autodisvery.
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    //RCCHECK(rmw_uros_discover_agent(rmw_options));
    #endif
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "esp32_bldc_controller", "", &support));

    RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "esp32/bldc/vq"));

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(1000)));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));

    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000);
    }
}

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

static void micro_ros_initialize(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreatePinnedToCore(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL,
            APP_CPU_NUM);
}


void app_main(void)
{
    ESP_LOGI(TAG, "Initializing the esp foc motor controller!");

    initialize_foc_drivers();

    esp_foc_initialize_axis(
        &axis,
        inverter,
        sensor,
        NULL,
        settings
    );

    micro_ros_initialize();
}
