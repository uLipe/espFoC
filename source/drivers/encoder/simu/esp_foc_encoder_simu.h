/*
 * MIT License
 */
#pragma once

#include "espFoC/drivers/esp_foc_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Open-loop rotor sensor: integrates a simplified RL dq electrical
 *        model from wired per-unit u_d/u_q plus a 1-D mechanical inertia +
 *        viscous load. Exposes the same counts contract as a 4096 CPR
 *        absolute encoder (AS5600-like).
 *
 * Call esp_foc_encoder_simu_wire_ud_uq() with pointers to the axis u_d/u_q
 * (e.g. &axis->u_d.raw) so read_counts() snapshots them under a short
 * critical section before each slow-loop step.
 */
esp_foc_encoder_t *esp_foc_encoder_simu_new(int port,
                                              int motor_pole_pairs,
                                              float r_ohm,
                                              float l_henry,
                                              q16_t vdc_q16);

void esp_foc_encoder_simu_wire_ud_uq(esp_foc_encoder_t *self,
                                  q16_t *ud_raw,
                                  q16_t *uq_raw);

#ifdef __cplusplus
}
#endif
