/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <stddef.h>
#include <stdint.h>

void esp_foc_tuner_init_bus_callback(void)
{
}

int esp_foc_tuner_recv_callback(uint8_t *buf, size_t max)
{
    (void)buf;
    (void)max;
    return 0;
}

void esp_foc_tuner_send_callback(const uint8_t *buf, size_t len)
{
    (void)buf;
    (void)len;
}

void esp_foc_init_bus_callback(void)
{
}

void esp_foc_send_buffer_callback(const uint8_t *buffer, int size)
{
    (void)buffer;
    (void)size;
}
