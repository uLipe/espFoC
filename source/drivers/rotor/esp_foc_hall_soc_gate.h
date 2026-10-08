/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include "soc/soc_caps.h"

/*
 * The hall rotor sensor needs a hardware-latched edge timestamp, which on this
 * family means GPIO ETM events driving a timer-group capture task. Deliberately
 * a different gate from the inverter's: the two drivers must not include each
 * other, and the hall works on any SoC that passes this, inverter or not.
 */
#if !SOC_ETM_SUPPORTED || !SOC_TIMER_SUPPORT_ETM
#error "espFoC hall rotor sensor requires ETM + timer-group capture (SOC_CAPS)"
#endif
