/*
 * MIT License
 *
 * Copyright (c) 2024 Felipe Neves
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

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/ipm.h>
#include <espFoC/ipc/esp_foc_ipc.h>
#include <soc.h>

#define LOCK_FREE_VAL           0xA5A5A5A5
#define ESP_FOC_IPC_CMD_MAGIC   0x434d4d44
#define ESP_FOC_IPC_STATE_MAGIC 0x45466f43
#define ESP_FOC_IPC_EV_CMD      0x01
#define ESP_FOC_IPC_EV_STATE    0x02

struct esp_foc_ipc_areas {
    uint32_t lock_command;
    uint32_t lock_state;
    struct motor_command cmd_area;
    struct motor_state state_area;
    struct motor_driver_info mc_driver_info;
};

static const struct device *ipc_dev = DEVICE_DT_GET(DT_NODELABEL(ipm0));
static struct esp_foc_ipc_areas *areas = (struct esp_foc_ipc_areas *)DT_REG_ADDR(DT_CHOSEN(zephyr_ipc_shm));

static sys_dlist_t ipc_callbacks;
static uint32_t event_type;
static struct k_work_q ipc_work_q;
static struct k_work ipc_work;
K_THREAD_STACK_DEFINE(ipc_work_q_stack, 4096);

static void ipc_work_handler(struct k_work *work)
{
    sys_dnode_t *link;
    uint32_t ev = event_type;
    event_type = 0;

    int key = irq_lock();

    if(ev == ESP_FOC_IPC_EV_CMD) {

        while (!atomic_cas((atomic_t *)&areas->lock_command, LOCK_FREE_VAL,
            ESP_FOC_IPC_CMD_MAGIC)) {
            ;
        }

        SYS_DLIST_FOR_EACH_NODE(&ipc_callbacks, link) {
            struct motor_report_callback * report = CONTAINER_OF(link, struct motor_report_callback, node);
            if(report) {
                if(report->cmd_cb) {
                    report->cmd_cb((const struct motor_command *)&areas->cmd_area);
                }
            }
        }

        atomic_set((atomic_t *)&areas->lock_command, LOCK_FREE_VAL);
    }

    if(ev == ESP_FOC_IPC_EV_STATE) {

        while (!atomic_cas((atomic_t *)&areas->lock_state, LOCK_FREE_VAL,
            ESP_FOC_IPC_STATE_MAGIC)) {
            ;
        }

        SYS_DLIST_FOR_EACH_NODE(&ipc_callbacks, link) {
            struct motor_report_callback * report = CONTAINER_OF(link, struct motor_report_callback, node);
            if(report) {
                if(report->state_cb) {
                    report->state_cb((const struct motor_state *)&areas->state_area);
                }
            }
        }

        atomic_set((atomic_t *)&areas->lock_state, LOCK_FREE_VAL);
    }

    irq_unlock(key);
}

static void ipc_dev_callback(const struct device *dev, void *context,
				  uint32_t id, volatile void *data)
{
    event_type = id;
    k_work_submit_to_queue(&ipc_work_q, &ipc_work);
}

int esp_foc_ipc_init(struct motor_driver_info *mc_info)
{
    if(esp_core_id() == 0) {

        if(!mc_info)
            return -EINVAL;

        memset(areas, 0, sizeof(*areas));
        memcpy(&areas->mc_driver_info, mc_info, sizeof(*mc_info));
        atomic_set((atomic_t *)&areas->lock_command, LOCK_FREE_VAL);
        atomic_set((atomic_t *)&areas->lock_state, LOCK_FREE_VAL);
    }

    k_work_queue_start(&ipc_work_q, ipc_work_q_stack,
                       K_THREAD_STACK_SIZEOF(ipc_work_q_stack), 0, NULL);
    k_work_init(&ipc_work, ipc_work_handler);
    sys_dlist_init(&ipc_callbacks);
    ipm_register_callback(ipc_dev, ipc_dev_callback, NULL);

    return 0;
}
int esp_foc_ipc_send_command(const struct motor_command *cmd)
{
    uint32_t ev;

    if(!cmd)
        return -EINVAL;

    int key = irq_lock();
    while (!atomic_cas((atomic_t *)&areas->lock_command, LOCK_FREE_VAL,
            ESP_FOC_IPC_CMD_MAGIC)) {
        ;
    }

    memcpy(&areas->cmd_area, cmd, sizeof(struct motor_command));
    ev = ESP_FOC_IPC_EV_CMD;
    atomic_set((atomic_t *)&areas->lock_command, LOCK_FREE_VAL);
    irq_unlock(key);

    return ipm_send(ipc_dev, -1, ev, &ev, sizeof(ev));
}

int esp_foc_ipc_send_state(const struct motor_state *state)
{
    uint32_t ev;

    if(!state)
        return -EINVAL;

    int key = irq_lock();
    while (!atomic_cas((atomic_t *)&areas->lock_state, LOCK_FREE_VAL,
            ESP_FOC_IPC_STATE_MAGIC)) {
        ;
    }

    memcpy(&areas->state_area, state, sizeof(struct motor_state));
    ev = ESP_FOC_IPC_EV_STATE;
    atomic_set((atomic_t *)&areas->lock_state, LOCK_FREE_VAL);
    irq_unlock(key);

    return ipm_send(ipc_dev, -1, ev, &ev, sizeof(ev));
}

int esp_foc_ipc_register_callback(const struct motor_report_callback *cb)
{
    if(!cb)
        return -EINVAL;

    uint32_t key = irq_lock();
    sys_dlist_append(&ipc_callbacks, (sys_dnode_t *)&cb->node);
    irq_unlock(key);

    return 0;
}

int esp_foc_ipc_remove_callback(const struct motor_report_callback *cb)
{
    if(!cb)
        return -EINVAL;

    uint32_t key = irq_lock();
    sys_dlist_remove((sys_dnode_t *)&cb->node);
    irq_unlock(key);

    return 0;
}

int esp_foc_ipc_encoder_put(float encoder_0, float encoder_1)
{
    atomic_set((atomic_t *)&areas->mc_driver_info.encoder_reading_0, encoder_0);
    atomic_set((atomic_t *)&areas->mc_driver_info.encoder_reading_1, encoder_1);
    return 0;
}

struct motor_driver_info* esp_foc_ipc_get_info(void)
{
    return &areas->mc_driver_info;
}