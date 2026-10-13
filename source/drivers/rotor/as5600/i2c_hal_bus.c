/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Private I2C master bus helper — HAL/LL only (no driver/i2c.h).
 * Sync write/write_read: two phases from task, poll `done` (init/calib).
 * Runtime read: current-address; task event-waits; ISR copies RX and posts.
 * Async: kick once; ISR starts B after A's END (RX FIFO is reset first).
 * NACK/timeout sets `need_clear`; the next task kick runs the HW 9-clock clear.
 */
#include "i2c_hal_bus.h"

#include <stddef.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "hal/gpio_ll.h"
#include "hal/i2c_ll.h"
#include "soc/gpio_struct.h"
#include "soc/i2c_periph.h"
#include "soc/io_mux_reg.h"
#include "soc/clk_tree_defs.h"
#include "espFoC/osal/esp_foc_osal.h"

static const char *TAG = "foc_i2c";

#define I2C_SRC_HZ 40000000u
#define I2C_WAIT_MS 100u
#define I2C_ASYNC_STUCK_US 2000u
#define I2C_GLITCH_FILTER 7u
#define I2C_ERR_INTR \
    (I2C_LL_INTR_NACK | I2C_LL_INTR_TIMEOUT | I2C_LL_INTR_ARBITRATION)
#define I2C_MASTER_INTR I2C_LL_MASTER_EVENT_INTR

static void gpio_od_matrix(int gpio, uint32_t out_sig, uint32_t in_sig)
{
    esp_rom_gpio_pad_select_gpio((uint32_t)gpio);
    gpio_ll_func_sel(&GPIO, (uint8_t)gpio, PIN_FUNC_GPIO);
    gpio_ll_od_enable(&GPIO, (uint32_t)gpio);
    /*
     * This bench has no external I2C pull-ups. The C6 pad ~45 kΩ is the idle
     * level; short traces at 400 kHz are inside the rise-time budget. A 5 V
     * bus with its own resistors should disable these again.
     */
    gpio_ll_pullup_en(&GPIO, (uint32_t)gpio);
    gpio_ll_pulldown_dis(&GPIO, (uint32_t)gpio);
    gpio_ll_input_enable(&GPIO, gpio);
    gpio_ll_output_enable(&GPIO, gpio);
    gpio_ll_set_level(&GPIO, gpio, 1);
    esp_rom_gpio_connect_out_signal((uint32_t)gpio, out_sig, false, false);
    esp_rom_gpio_connect_in_signal((uint32_t)gpio, in_sig, false);
}

static void bus_clear(esp_foc_i2c_hal_bus_t *bus)
{
    i2c_dev_t *hw = bus->hal.dev;
    i2c_ll_master_fsm_rst(hw);
    i2c_ll_master_clr_bus(hw, I2C_LL_RESET_SLV_SCL_PULSE_NUM_DEFAULT, true);
    uint64_t t0 = esp_foc_now_us();
    while (i2c_ll_master_is_bus_clear_done(hw)) {
        if ((esp_foc_now_us() - t0) > 2000u) {
            i2c_ll_master_clr_bus(hw, 0, false);
            break;
        }
    }
    i2c_ll_update(hw);
    i2c_ll_clear_intr_mask(hw, UINT32_MAX);
}

static void clear_cmd_regs(i2c_dev_t *hw)
{
    i2c_ll_hw_cmd_t z = { .val = 0 };
    for (int i = 0; i < 8; i++) {
        i2c_ll_master_write_cmd_reg(hw, z, i);
    }
}

static void finish_xfer(esp_foc_i2c_hal_bus_t *bus, esp_err_t err)
{
    bus->last_err = err;
    bus->phase = ESP_FOC_I2C_PHASE_IDLE;
    bus->busy = false;
    bus->done = true;
    bus->async_chain = false;
    if (bus->cb != NULL) {
        esp_foc_i2c_hal_done_cb_t cb = bus->cb;
        void *arg = bus->cb_arg;
        bus->cb = NULL;
        cb(arg, err);
    }
    if (bus->waiter != NULL) {
        esp_foc_event_post_auto(bus->waiter);
    }
}

static void start_phase_b(esp_foc_i2c_hal_bus_t *bus)
{
    i2c_dev_t *hw = bus->hal.dev;
    uint8_t addr_r = (uint8_t)((bus->addr7 << 1) | 1);
    i2c_ll_hw_cmd_t c;
    int cmd = 0;
    size_t rd_len = bus->rd_len;

    i2c_ll_txfifo_rst(hw);
    i2c_ll_rxfifo_rst(hw);
    i2c_ll_clear_intr_mask(hw, UINT32_MAX);
    clear_cmd_regs(hw);

    i2c_ll_write_txfifo(hw, &addr_r, 1);

    c.val = 0;
    c.op_code = I2C_LL_CMD_RESTART;
    i2c_ll_master_write_cmd_reg(hw, c, cmd++);

    c.val = 0;
    c.op_code = I2C_LL_CMD_WRITE;
    c.byte_num = 1;
    c.ack_en = 1;
    c.ack_exp = 0;
    i2c_ll_master_write_cmd_reg(hw, c, cmd++);

    if (rd_len > 1u) {
        c.val = 0;
        c.op_code = I2C_LL_CMD_READ;
        c.byte_num = (uint32_t)(rd_len - 1u);
        c.ack_val = 0;
        i2c_ll_master_write_cmd_reg(hw, c, cmd++);
    }

    c.val = 0;
    c.op_code = I2C_LL_CMD_READ;
    c.byte_num = 1;
    /* SDA high = NACK. AS5600 slave-transmitter will not leave the bus without it. */
    c.ack_val = 1;
    i2c_ll_master_write_cmd_reg(hw, c, cmd++);

    c.val = 0;
    c.op_code = I2C_LL_CMD_STOP;
    i2c_ll_master_write_cmd_reg(hw, c, cmd++);

    bus->phase = ESP_FOC_I2C_PHASE_B;
    bus->done = false;
    bus->last_err = ESP_ERR_TIMEOUT;
    i2c_ll_update(hw);
    i2c_ll_start_trans(hw);
}

static void start_phase_a(esp_foc_i2c_hal_bus_t *bus)
{
    i2c_dev_t *hw = bus->hal.dev;
    uint8_t addr_w = (uint8_t)((bus->addr7 << 1) | 0);
    i2c_ll_hw_cmd_t c;
    int cmd = 0;
    uint8_t tx[8];

    /* C6 needs END between write and read phases. */
    i2c_ll_txfifo_rst(hw);
    i2c_ll_rxfifo_rst(hw);
    i2c_ll_clear_intr_mask(hw, UINT32_MAX);
    clear_cmd_regs(hw);

    tx[0] = addr_w;
    memcpy(&tx[1], bus->wr, bus->wr_len);
    i2c_ll_write_txfifo(hw, tx, (uint8_t)(bus->wr_len + 1u));

    c.val = 0;
    c.op_code = I2C_LL_CMD_RESTART;
    i2c_ll_master_write_cmd_reg(hw, c, cmd++);

    c.val = 0;
    c.op_code = I2C_LL_CMD_WRITE;
    c.byte_num = (uint32_t)(bus->wr_len + 1u);
    c.ack_en = 1;
    c.ack_exp = 0;
    i2c_ll_master_write_cmd_reg(hw, c, cmd++);

    c.val = 0;
    c.op_code = I2C_LL_CMD_END;
    i2c_ll_master_write_cmd_reg(hw, c, cmd++);

    bus->phase = ESP_FOC_I2C_PHASE_A;
    bus->done = false;
    bus->last_err = ESP_ERR_TIMEOUT;
    i2c_ll_update(hw);
    i2c_ll_start_trans(hw);
}

static void i2c_isr(void *arg)
{
    esp_foc_i2c_hal_bus_t *bus = (esp_foc_i2c_hal_bus_t *)arg;
    i2c_dev_t *hw = bus->hal.dev;
    uint32_t st = 0;

    i2c_ll_get_intr_mask(hw, &st);
    if (st == 0) {
        return;
    }
    i2c_ll_clear_intr_mask(hw, st);
    bus->last_st = st;

    if (st & I2C_ERR_INTR) {
        i2c_ll_master_fsm_rst(hw);
        bus->need_clear = true;
        finish_xfer(bus, (st & I2C_LL_INTR_NACK) ? ESP_ERR_NOT_FOUND : ESP_FAIL);
        return;
    }

    if (bus->phase == ESP_FOC_I2C_PHASE_A && (st & I2C_LL_INTR_END_DETECT)) {
        if (bus->async_chain) {
            start_phase_b(bus);
        } else {
            bus->last_err = ESP_OK;
            bus->done = true;
        }
        return;
    }

    if (bus->phase == ESP_FOC_I2C_PHASE_B && (st & I2C_LL_INTR_MST_COMPLETE)) {
        uint32_t avail = 0;
        if (bus->rd_len == 0u) {
            finish_xfer(bus, ESP_OK);
            return;
        }
        i2c_ll_get_rxfifo_cnt(hw, &avail);
        if (avail < bus->rd_len || bus->rd == NULL) {
            bus->need_clear = true;
            finish_xfer(bus, ESP_ERR_INVALID_RESPONSE);
            return;
        }
        i2c_ll_read_rxfifo(hw, bus->rd, (uint8_t)bus->rd_len);
        finish_xfer(bus, ESP_OK);
        return;
    }

    i2c_ll_master_fsm_rst(hw);
    bus->need_clear = true;
    finish_xfer(bus, ESP_FAIL);
}

static esp_err_t poll_done(esp_foc_i2c_hal_bus_t *bus)
{
    uint32_t waited = 0;
    while (!bus->done && waited < I2C_WAIT_MS) {
        esp_foc_sleep_ms(1);
        waited++;
    }
    if (!bus->done) {
        i2c_ll_master_fsm_rst(bus->hal.dev);
        bus->phase = ESP_FOC_I2C_PHASE_IDLE;
        bus->busy = false;
        bus->done = true;
        bus->async_chain = false;
        bus->last_err = ESP_ERR_TIMEOUT;
        return ESP_ERR_TIMEOUT;
    }
    return bus->last_err;
}

esp_err_t esp_foc_i2c_hal_bus_init(esp_foc_i2c_hal_bus_t *bus,
                                   int port,
                                   int sda,
                                   int scl,
                                   uint32_t hz)
{
    ESP_RETURN_ON_FALSE(bus != NULL, ESP_ERR_INVALID_ARG, TAG, "null");
    ESP_RETURN_ON_FALSE(port == 0, ESP_ERR_INVALID_ARG, TAG, "port");
    ESP_RETURN_ON_FALSE(sda >= 0 && scl >= 0, ESP_ERR_INVALID_ARG, TAG, "pins");
    ESP_RETURN_ON_FALSE(hz >= 10000u && hz <= 1000000u, ESP_ERR_INVALID_ARG, TAG, "hz");

    memset(bus, 0, sizeof(*bus));
    bus->port = port;
    bus->sda = sda;
    bus->scl = scl;
    bus->hz = hz;

    i2c_ll_enable_bus_clock(port, true);
    i2c_ll_reset_register(port);

    i2c_hal_init(&bus->hal, port);
    i2c_hal_master_init(&bus->hal);
    i2c_ll_set_source_clk(bus->hal.dev, I2C_CLK_SRC_XTAL);
    i2c_hal_set_bus_timing(&bus->hal, (int)hz, I2C_CLK_SRC_XTAL, (int)I2C_SRC_HZ);
    i2c_hal_master_set_scl_timeout_val(&bus->hal, 20000, I2C_SRC_HZ);
    i2c_ll_master_set_filter(bus->hal.dev, (uint8_t)I2C_GLITCH_FILTER);
    i2c_ll_enable_fifo_mode(bus->hal.dev, true);
    i2c_ll_clear_intr_mask(bus->hal.dev, UINT32_MAX);
    i2c_ll_enable_intr_mask(bus->hal.dev, I2C_MASTER_INTR);
    i2c_ll_update(bus->hal.dev);

    gpio_od_matrix(sda,
                   i2c_periph_signal[port].sda_out_sig,
                   i2c_periph_signal[port].sda_in_sig);
    gpio_od_matrix(scl,
                   i2c_periph_signal[port].scl_out_sig,
                   i2c_periph_signal[port].scl_in_sig);

    esp_err_t ierr = esp_intr_alloc_intrstatus(
        i2c_periph_signal[port].irq,
        ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LOWMED,
        (uint32_t)i2c_ll_get_interrupt_status_reg(bus->hal.dev),
        I2C_MASTER_INTR,
        i2c_isr,
        bus,
        &bus->intr);
    ESP_RETURN_ON_ERROR(ierr, TAG, "intr alloc");

    bus->inited = true;
    return ESP_OK;
}

void esp_foc_i2c_hal_bus_deinit(esp_foc_i2c_hal_bus_t *bus)
{
    if (bus == NULL || !bus->inited) {
        return;
    }
    if (bus->intr != NULL) {
        (void)esp_intr_free(bus->intr);
        bus->intr = NULL;
    }
    i2c_hal_deinit(&bus->hal);
    i2c_ll_enable_bus_clock(bus->port, false);
    bus->inited = false;
}

bool esp_foc_i2c_hal_busy(const esp_foc_i2c_hal_bus_t *bus)
{
    return bus != NULL && bus->busy;
}

void esp_foc_i2c_hal_recover(esp_foc_i2c_hal_bus_t *bus)
{
    if (bus == NULL || !bus->inited) {
        return;
    }
    bus_clear(bus);
    bus->need_clear = false;
    if (bus->busy || bus->cb != NULL) {
        finish_xfer(bus, ESP_ERR_TIMEOUT);
    } else {
        bus->phase = ESP_FOC_I2C_PHASE_IDLE;
        bus->busy = false;
        bus->done = true;
        bus->async_chain = false;
    }
}

esp_err_t esp_foc_i2c_hal_write_read_async(esp_foc_i2c_hal_bus_t *bus,
                                           uint8_t addr7,
                                           const uint8_t *wr,
                                           size_t wr_len,
                                           uint8_t *rd,
                                           size_t rd_len,
                                           esp_foc_i2c_hal_done_cb_t cb,
                                           void *cb_arg)
{
    ESP_RETURN_ON_FALSE(bus != NULL && bus->inited, ESP_ERR_INVALID_STATE, TAG, "state");
    ESP_RETURN_ON_FALSE(wr_len <= 7u, ESP_ERR_INVALID_ARG, TAG, "wr");
    ESP_RETURN_ON_FALSE(wr_len == 0u || wr != NULL, ESP_ERR_INVALID_ARG, TAG, "wr");
    ESP_RETURN_ON_FALSE(rd != NULL && rd_len > 0 && rd_len <= 32, ESP_ERR_INVALID_ARG, TAG, "rd");
    ESP_RETURN_ON_FALSE(cb != NULL, ESP_ERR_INVALID_ARG, TAG, "cb");

    if (bus->need_clear) {
        bus_clear(bus);
        bus->need_clear = false;
        if (bus->busy || bus->cb != NULL) {
            finish_xfer(bus, ESP_ERR_TIMEOUT);
        }
    }

    /* Continue deferred phase B from task context. */
    if (bus->phase == ESP_FOC_I2C_PHASE_A_DONE && bus->busy) {
        start_phase_b(bus);
        return ESP_OK;
    }

    if (bus->busy) {
        uint64_t now = esp_foc_now_us();
        if ((now - bus->busy_since_us) < I2C_ASYNC_STUCK_US) {
            return ESP_ERR_INVALID_STATE;
        }
        esp_foc_i2c_hal_recover(bus);
    }

    bus->addr7 = addr7;
    bus->wr_len = wr_len;
    if (wr_len > 0u) {
        memcpy(bus->wr, wr, wr_len);
    }
    bus->rd = rd;
    bus->rd_len = rd_len;
    bus->cb = cb;
    bus->cb_arg = cb_arg;
    bus->waiter = NULL;
    bus->async_chain = (wr_len > 0u);
    bus->last_st = 0;
    bus->busy = true;
    bus->busy_since_us = esp_foc_now_us();

    if (wr_len == 0u) {
        /* Current-address read: START+R … NACK+STOP, no setup write. */
        start_phase_b(bus);
    } else {
        start_phase_a(bus);
    }
    return ESP_OK;
}

static void prep_sync(esp_foc_i2c_hal_bus_t *bus)
{
    if (bus->need_clear) {
        bus_clear(bus);
        bus->need_clear = false;
        if (bus->busy || bus->cb != NULL) {
            finish_xfer(bus, ESP_ERR_TIMEOUT);
        }
    }

    if (bus->busy) {
        esp_foc_i2c_hal_recover(bus);
    }
}

/* Block the calling task on the ISR's completion post, never on a sleep poll. */
static esp_err_t wait_xfer(esp_foc_i2c_hal_bus_t *bus, uint8_t addr7)
{
    uint64_t t0 = esp_foc_now_us();
    while (!bus->done) {
        uint64_t elapsed = esp_foc_now_us() - t0;
        if (elapsed >= ((uint64_t)I2C_WAIT_MS * 1000ull)) {
            break;
        }
        uint32_t remain_ms = I2C_WAIT_MS - (uint32_t)(elapsed / 1000ull);
        if (remain_ms == 0u) {
            break;
        }
        (void)esp_foc_event_wait_ms(remain_ms);
    }

    bus->waiter = NULL;
    if (!bus->done) {
        i2c_ll_master_fsm_rst(bus->hal.dev);
        bus->phase = ESP_FOC_I2C_PHASE_IDLE;
        bus->busy = false;
        bus->done = true;
        bus->async_chain = false;
        bus->need_clear = true;
        bus->last_err = ESP_ERR_TIMEOUT;
        ESP_LOGW(TAG, "rd fail addr=0x%02x err=ESP_ERR_TIMEOUT st=0x%lx",
                 (unsigned)addr7, (unsigned long)bus->last_st);
        return ESP_ERR_TIMEOUT;
    }
    if (bus->last_err != ESP_OK) {
        ESP_LOGW(TAG, "rd fail addr=0x%02x err=%s st=0x%lx",
                 (unsigned)addr7, esp_err_to_name(bus->last_err),
                 (unsigned long)bus->last_st);
    }
    return bus->last_err;
}

esp_err_t esp_foc_i2c_hal_read(esp_foc_i2c_hal_bus_t *bus,
                               uint8_t addr7,
                               uint8_t *rd,
                               size_t rd_len)
{
    ESP_RETURN_ON_FALSE(bus != NULL && bus->inited, ESP_ERR_INVALID_STATE, TAG, "state");
    ESP_RETURN_ON_FALSE(rd != NULL && rd_len > 0 && rd_len <= 32, ESP_ERR_INVALID_ARG, TAG, "rd");
    ESP_RETURN_ON_FALSE(esp_foc_in_task_context(), ESP_ERR_INVALID_STATE, TAG, "need task");

    prep_sync(bus);

    bus->addr7 = addr7;
    bus->wr_len = 0;
    bus->rd = rd;
    bus->rd_len = rd_len;
    bus->cb = NULL;
    bus->cb_arg = NULL;
    bus->async_chain = false;
    bus->waiter = esp_foc_event_handle_self();
    bus->last_st = 0;
    bus->busy = true;
    bus->busy_since_us = esp_foc_now_us();

    esp_foc_event_clear();
    start_phase_b(bus);
    return wait_xfer(bus, addr7);
}

esp_err_t esp_foc_i2c_hal_write_read(esp_foc_i2c_hal_bus_t *bus,
                                     uint8_t addr7,
                                     const uint8_t *wr,
                                     size_t wr_len,
                                     uint8_t *rd,
                                     size_t rd_len)
{
    ESP_RETURN_ON_FALSE(bus != NULL && bus->inited, ESP_ERR_INVALID_STATE, TAG, "state");
    ESP_RETURN_ON_FALSE(wr != NULL && wr_len > 0 && wr_len <= 7, ESP_ERR_INVALID_ARG, TAG, "wr");
    ESP_RETURN_ON_FALSE(rd != NULL && rd_len > 0 && rd_len <= 32, ESP_ERR_INVALID_ARG, TAG, "rd");
    ESP_RETURN_ON_FALSE(esp_foc_in_task_context(), ESP_ERR_INVALID_STATE, TAG, "need task");

    prep_sync(bus);

    bus->addr7 = addr7;
    memcpy(bus->wr, wr, wr_len);
    bus->wr_len = wr_len;
    bus->rd = rd;
    bus->rd_len = rd_len;
    bus->cb = NULL;
    bus->cb_arg = NULL;
    /* The ISR chains phase B on END, so both phases finish without a task hop. */
    bus->async_chain = true;
    bus->waiter = esp_foc_event_handle_self();
    bus->last_st = 0;
    bus->busy = true;
    bus->busy_since_us = esp_foc_now_us();

    esp_foc_event_clear();
    start_phase_a(bus);
    return wait_xfer(bus, addr7);
}

esp_err_t esp_foc_i2c_hal_write(esp_foc_i2c_hal_bus_t *bus,
                                uint8_t addr7,
                                const uint8_t *wr,
                                size_t wr_len)
{
    ESP_RETURN_ON_FALSE(bus != NULL && bus->inited, ESP_ERR_INVALID_STATE, TAG, "state");
    ESP_RETURN_ON_FALSE(wr != NULL && wr_len > 0 && wr_len <= 7, ESP_ERR_INVALID_ARG, TAG, "wr");
    ESP_RETURN_ON_FALSE(esp_foc_in_task_context(), ESP_ERR_INVALID_STATE, TAG, "need task");
    ESP_RETURN_ON_FALSE(!bus->busy, ESP_ERR_INVALID_STATE, TAG, "busy");

    i2c_dev_t *hw = bus->hal.dev;
    uint8_t tx[8];
    i2c_ll_hw_cmd_t c;
    int cmd = 0;

    bus->addr7 = addr7;
    memcpy(bus->wr, wr, wr_len);
    bus->wr_len = wr_len;
    bus->rd = NULL;
    bus->rd_len = 0;
    bus->cb = NULL;
    bus->cb_arg = NULL;
    bus->waiter = NULL;
    bus->async_chain = false;
    bus->last_st = 0;
    bus->busy = true;

    i2c_ll_txfifo_rst(hw);
    i2c_ll_rxfifo_rst(hw);
    i2c_ll_clear_intr_mask(hw, UINT32_MAX);
    clear_cmd_regs(hw);

    tx[0] = (uint8_t)((addr7 << 1) | 0);
    memcpy(&tx[1], wr, wr_len);
    i2c_ll_write_txfifo(hw, tx, (uint8_t)(wr_len + 1u));

    c.val = 0;
    c.op_code = I2C_LL_CMD_RESTART;
    i2c_ll_master_write_cmd_reg(hw, c, cmd++);

    c.val = 0;
    c.op_code = I2C_LL_CMD_WRITE;
    c.byte_num = (uint32_t)(wr_len + 1u);
    c.ack_en = 1;
    c.ack_exp = 0;
    i2c_ll_master_write_cmd_reg(hw, c, cmd++);

    c.val = 0;
    c.op_code = I2C_LL_CMD_STOP;
    i2c_ll_master_write_cmd_reg(hw, c, cmd++);

    /* Phase B so the ISR completes on MST_COMPLETE; rd_len 0 skips the read. */
    bus->phase = ESP_FOC_I2C_PHASE_B;
    bus->done = false;
    bus->last_err = ESP_ERR_TIMEOUT;
    i2c_ll_update(hw);
    i2c_ll_start_trans(hw);

    esp_err_t err = poll_done(bus);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "write fail addr=0x%02x err=%s st=0x%lx",
                 (unsigned)addr7, esp_err_to_name(err), (unsigned long)bus->last_st);
    }
    bus->busy = false;
    bus->phase = ESP_FOC_I2C_PHASE_IDLE;
    return err;
}
