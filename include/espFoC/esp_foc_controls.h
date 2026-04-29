/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#pragma once

#define ESP_FOC_LOW_SPEED_DOWNSAMPLING      10
#define ESP_FOC_ISENSOR_CALIBRATION_ROUNDS  100
#define ESP_FOC_PLL_BANDWIDTH_HZ            150.0f

void do_current_mode_sensored_high_speed_loop(void *arg);
void do_current_mode_sensored_low_speed_loop(void *arg);
