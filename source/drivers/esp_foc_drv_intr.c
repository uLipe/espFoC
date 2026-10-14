/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include "esp_foc_drv_intr.h"

#include "sdkconfig.h"

#if CONFIG_ESP_FOC_CORE_ID >= 0
#include "esp_cpu.h"
#include "esp_ipc.h"

typedef struct {
    int source;
    int flags;
    uint32_t status_reg;
    uint32_t status_mask;
    intr_handler_t handler;
    void *arg;
    intr_handle_t *out;
    esp_err_t err;
} intr_req_t;

/* Runs in the IPC task of the target CPU, whose stack is small: no logging. */
static void intr_alloc_here(void *p)
{
    intr_req_t *r = (intr_req_t *)p;
    r->err = esp_intr_alloc_intrstatus(r->source, r->flags, r->status_reg,
                                       r->status_mask, r->handler, r->arg, r->out);
}
#endif

esp_err_t esp_foc_drv_intr_alloc(int source, int flags, uint32_t status_reg,
                                 uint32_t status_mask, intr_handler_t handler,
                                 void *arg, intr_handle_t *out)
{
#if CONFIG_ESP_FOC_CORE_ID >= 0
    if (esp_cpu_get_core_id() != CONFIG_ESP_FOC_CORE_ID) {
        intr_req_t r = {
            .source = source,
            .flags = flags,
            .status_reg = status_reg,
            .status_mask = status_mask,
            .handler = handler,
            .arg = arg,
            .out = out,
            .err = ESP_FAIL,
        };
        esp_err_t err = esp_ipc_call_blocking(CONFIG_ESP_FOC_CORE_ID, intr_alloc_here, &r);
        return (err != ESP_OK) ? err : r.err;
    }
#endif
    return esp_intr_alloc_intrstatus(source, flags, status_reg, status_mask,
                                     handler, arg, out);
}
