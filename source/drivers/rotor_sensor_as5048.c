/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
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

#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "espFoC/esp_foc.h"
#include "espFoC/drivers/rotor_sensor_interface.h"
#include "espFoC/rotor_sensor_as5048.h"

#define AS5048_ADDR_ANGLE          0x3FFFu
#define AS5048_ADDR_CLEAR_ERR      0x0001u

#define AS5048_DATA_MASK           0x3FFFu
#define AS5048_FLAG_EF_MASK        0x4000u  // bit 14 in response frame (PAR | EF | DATA[13:0]) :contentReference[oaicite:5]{index=5}
#define AS5048_PULSES_PER_REV      16384.0f // 14-bit
#define AS5048_WRAP_VALUE          (AS5048_PULSES_PER_REV * 0.95f)

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

static const char *tag = "rotor_as5048";

typedef struct {
    float accumulated;
    float previous;
    uint16_t zero_offset;

    spi_device_handle_t dev;
    spi_host_device_t host;

    esp_foc_rotor_sensor_t interface;
} esp_foc_as5048_t;

static esp_foc_as5048_t rotor_sensors[CONFIG_NOOF_AXIS];

/* Track SPI bus init per host. ESP-IDF has SPI2_HOST/SPI3_HOST typically. */
static bool spi_bus_configured[SOC_SPI_PERIPH_NUM];

static inline uint8_t popcount16(uint16_t x)
{
    /* Tiny popcount (good enough here). */
    x = x - ((x >> 1) & 0x5555u);
    x = (x & 0x3333u) + ((x >> 2) & 0x3333u);
    x = (x + (x >> 4)) & 0x0F0Fu;
    x = x + (x >> 8);
    return (uint8_t)(x & 0x1Fu);
}

/* Even parity: total number of 1s in the 16-bit word must be even. :contentReference[oaicite:6]{index=6} */
static inline uint16_t add_even_parity(uint16_t word_15bits)
{
    /* word_15bits uses bits [14:0], parity goes to bit[15]. */
    uint8_t ones = popcount16(word_15bits);
    uint16_t par = (ones & 1u) ? 0x8000u : 0x0000u;
    return (uint16_t)(word_15bits | par);
}

/* Command package: PAR | RWn | Address[13:0] :contentReference[oaicite:7]{index=7} */
static inline uint16_t make_read_cmd(uint16_t addr14)
{
    uint16_t w = (uint16_t)((1u << 14) | (addr14 & AS5048_DATA_MASK));
    return add_even_parity(w);
}

static inline bool check_even_parity16(uint16_t word16)
{
    /* Total ones must be even. */
    return (popcount16(word16) & 1u) == 0u;
}

static esp_err_t spi_txrx16(spi_device_handle_t dev, uint16_t tx, uint16_t *rx)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    uint16_t tx_be = (uint16_t)((tx >> 8) | (tx << 8)); /* send MSB first */
    uint16_t rx_be = 0;

    t.length = 16;
    t.tx_buffer = &tx_be;
    t.rx_buffer = &rx_be;

    esp_err_t err = spi_device_transmit(dev, &t);
    if (err != ESP_OK) {
        return err;
    }

    if (rx) {
        uint16_t tmp = rx_be;
        *rx = (uint16_t)((tmp >> 8) | (tmp << 8));
    }
    return ESP_OK;
}

/* READ needs 2 transmissions: READ cmd, then "next command" (NOP is fine). Data returns at end of 2nd transmission. :contentReference[oaicite:8]{index=8} */
static esp_err_t as5048_read_register(spi_device_handle_t dev, uint16_t addr14, uint16_t *out_data14, bool *out_ef)
{
    uint16_t rx1 = 0, rx2 = 0;

    esp_err_t err = spi_txrx16(dev, make_read_cmd(addr14), &rx1);
    if (err != ESP_OK) return err;

    /* NOP command frame is all zeros and response is 0x0000 for NOP itself. :contentReference[oaicite:9]{index=9} */
    err = spi_txrx16(dev, 0x0000u, &rx2);
    if (err != ESP_OK) return err;

    if (!check_even_parity16(rx2)) {
        /* Parity error would also set EF on next reads; but we can flag it now. :contentReference[oaicite:10]{index=10} */
        if (out_ef) *out_ef = true;
        if (out_data14) *out_data14 = (uint16_t)(rx2 & AS5048_DATA_MASK);
        return ESP_OK;
    }

    if (out_ef) *out_ef = (rx2 & AS5048_FLAG_EF_MASK) != 0u;
    if (out_data14) *out_data14 = (uint16_t)(rx2 & AS5048_DATA_MASK);
    return ESP_OK;
}

/* Clear Error Flag is implemented as a READ of address 0x0001. :contentReference[oaicite:11]{index=11} */
static void as5048_clear_error(spi_device_handle_t dev)
{
    uint16_t dummy = 0;
    bool ef = false;

    /* This read returns error register content + EF in the following frame. We don't use it here. :contentReference[oaicite:12]{index=12} */
    (void)as5048_read_register(dev, AS5048_ADDR_CLEAR_ERR, &dummy, &ef);
}

static uint16_t read_angle_raw(esp_foc_as5048_t *obj)
{
    uint16_t data = 0;
    bool ef = false;

    esp_err_t err = as5048_read_register(obj->dev, AS5048_ADDR_ANGLE, &data, &ef);
    if (err != ESP_OK) {
        ESP_LOGW(tag, "SPI read failed: %s", esp_err_to_name(err));
        return 0;
    }

    if (ef) {
        /* EF indicates previous transmission error (wrong parity/invalid command/etc.). :contentReference[oaicite:13]{index=13} */
        as5048_clear_error(obj->dev);

        /* Try once more. */
        err = as5048_read_register(obj->dev, AS5048_ADDR_ANGLE, &data, &ef);
        if (err != ESP_OK) {
            ESP_LOGW(tag, "SPI re-read failed: %s", esp_err_to_name(err));
            return 0;
        }
    }

    /* Clamp just in case. */
    data &= AS5048_DATA_MASK;
    data = (uint16_t)MIN(data, (uint16_t)(AS5048_PULSES_PER_REV - 1.0f));
    return data;
}

static float get_counts_per_revolution(esp_foc_rotor_sensor_t *self)
{
    (void)self;
    return AS5048_PULSES_PER_REV;
}

static float read_accumulated_counts(esp_foc_rotor_sensor_t *self)
{
    esp_foc_as5048_t *obj = __containerof(self, esp_foc_as5048_t, interface);
    return obj->accumulated + obj->previous;
}

static void set_to_zero(esp_foc_rotor_sensor_t *self)
{
    esp_foc_as5048_t *obj = __containerof(self, esp_foc_as5048_t, interface);
    obj->zero_offset = (uint16_t)obj->previous;
    ESP_LOGI(tag, "Setting %u [ticks] as offset.", (unsigned)obj->zero_offset);
}

static float read_counts(esp_foc_rotor_sensor_t *self)
{
    esp_foc_as5048_t *obj = __containerof(self, esp_foc_as5048_t, interface);

    uint16_t raw = read_angle_raw(obj);
    float delta = (float)raw - obj->previous;

    if (fabsf(delta) >= AS5048_WRAP_VALUE) {
        esp_foc_critical_enter();
        obj->accumulated = (delta < 0.0f)
            ? (obj->accumulated + AS5048_PULSES_PER_REV)
            : (obj->accumulated - AS5048_PULSES_PER_REV);
        esp_foc_critical_leave();
    }

    esp_foc_critical_enter();
    obj->previous = (float)raw;
    esp_foc_critical_leave();

    return (float)((raw - obj->zero_offset) & AS5048_DATA_MASK);
}

esp_foc_rotor_sensor_t *rotor_sensor_as5048_new(int pin_mosi,
                                                int pin_miso,
                                                int pin_sclk,
                                                int pin_cs,
                                                int host,
                                                int port)
{
    if (port > CONFIG_NOOF_AXIS - 1) {
        return NULL;
    }

    spi_host_device_t spi_host = (spi_host_device_t)host;

    /* Init vtable */
    rotor_sensors[port].interface.get_counts_per_revolution = get_counts_per_revolution;
    rotor_sensors[port].interface.read_counts = read_counts;
    rotor_sensors[port].interface.set_to_zero = set_to_zero;
    rotor_sensors[port].interface.read_accumulated_counts = read_accumulated_counts;

    rotor_sensors[port].zero_offset = 0;
    rotor_sensors[port].previous = 0;
    rotor_sensors[port].accumulated = 0;
    rotor_sensors[port].dev = NULL;
    rotor_sensors[port].host = spi_host;

    /* SPI timing: tL (CSn falling to first CLK rising) min 350ns :contentReference[oaicite:14]{index=14}
     * Also max clock 10MHz (TCLK min 100ns) :contentReference[oaicite:15]{index=15}
     */
    const int clock_hz = 10 * 1000 * 1000; /* 10MHz */
    const int pre_cs_cycles = 4; /* at 10MHz => 400ns, satisfies tL>=350ns :contentReference[oaicite:16]{index=16} */
    const int post_cs_cycles = 1;

    if (!spi_bus_configured[spi_host]) {
        spi_bus_config_t buscfg = {
            .mosi_io_num = pin_mosi,
            .miso_io_num = pin_miso,
            .sclk_io_num = pin_sclk,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
#if SOC_SPI_SUPPORT_CLK_APB
            .max_transfer_sz = 4,
#endif
        };

        esp_err_t err = spi_bus_initialize(spi_host, &buscfg, SPI_DMA_CH_AUTO);
        if (err != ESP_OK) {
            ESP_LOGE(tag, "spi_bus_initialize failed: %s", esp_err_to_name(err));
            return NULL;
        }

        spi_bus_configured[spi_host] = true;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = clock_hz,
        .mode = 1, /* AS5048A: data latched on falling edge, changes on rising edge :contentReference[oaicite:17]{index=17} */
        .spics_io_num = pin_cs,
        .queue_size = 1,
        .flags = 0,
        .cs_ena_pretrans = pre_cs_cycles,
        .cs_ena_posttrans = post_cs_cycles,
    };

    esp_err_t err = spi_bus_add_device(spi_host, &devcfg, &rotor_sensors[port].dev);
    if (err != ESP_OK) {
        ESP_LOGE(tag, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        return NULL;
    }

    /* Prime pipeline (optional): do one NOP so first read has valid timing/CS state. */
    (void)spi_txrx16(rotor_sensors[port].dev, 0x0000u, NULL);

    return &rotor_sensors[port].interface;
}
