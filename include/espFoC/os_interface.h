#pragma once

typedef void (*foc_loop_runner) (void *arg);

int esp_foc_create_runner(foc_loop_runner runner, void *argument);
void esp_foc_sleep_ms(int sleep_ms);
void esp_foc_runner_yield(void);
float esp_foc_now_seconds(void);
void esp_foc_fpu_isr_enter(void);
void esp_foc_fpu_isr_leave(void);