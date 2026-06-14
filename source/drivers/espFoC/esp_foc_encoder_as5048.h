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

#pragma once

#include "espFoC/esp_foc.h"

/**
 * AS5048A (SPI) rotor sensor driver.
 *
 * Pins MOSI/MISO/SCLK configure the SPI bus. CS is per-device.
 *
 * @param pin_mosi  SPI MOSI GPIO
 * @param pin_miso  SPI MISO GPIO
 * @param pin_sclk  SPI SCLK GPIO
 * @param pin_cs    SPI CSn GPIO (active low)
 * @param host      SPI host (e.g. SPI2_HOST / SPI3_HOST)
 * @param port      Axis index (0..CONFIG_NOOF_AXIS-1)
 */
esp_foc_rotor_sensor_t *rotor_sensor_as5048_new(int pin_mosi,
                                                int pin_miso,
                                                int pin_sclk,
                                                int pin_cs,
                                                int host,
                                                int port);
