#!/usr/bin/env bash
# One-shot driver tree reorg: encoders per type, inverter per SoC target.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

ENCODERS=(as5600 as5048 pcnt simu)
for enc in "${ENCODERS[@]}"; do
    mkdir -p "source/drivers/encoder/${enc}"
done

git mv source/drivers/esp_foc_encoder_as5600.c source/drivers/encoder/as5600/
git mv source/drivers/espFoC/esp_foc_encoder_as5600.h source/drivers/encoder/as5600/
git mv source/drivers/esp_foc_encoder_as5048.c source/drivers/encoder/as5048/
git mv source/drivers/espFoC/esp_foc_encoder_as5048.h source/drivers/encoder/as5048/
git mv source/drivers/esp_foc_encoder_pcnt.c source/drivers/encoder/pcnt/
git mv source/drivers/espFoC/esp_foc_encoder_pcnt.h source/drivers/encoder/pcnt/
git mv source/drivers/esp_foc_encoder_simu.c source/drivers/encoder/simu/
git mv source/drivers/espFoC/esp_foc_encoder_simu.h source/drivers/encoder/simu/

mkdir -p include/espFoC/drivers
git mv source/drivers/espFoC/esp_foc_inverter_mcpwm.h include/espFoC/drivers/

git mv source/drivers/espFoC/driver_q16_local.h source/drivers/

TARGETS=(esp32 esp32c6 esp32s3 esp32p4)
for t in "${TARGETS[@]}"; do
    d="source/drivers/inverter/${t}"
    mkdir -p "$d"
done

setup_target() {
    local t="$1"
    local d="source/drivers/inverter/${t}"
    cp source/drivers/inverter_3pwm_mcpwm.c "$d/esp_foc_inverter_mcpwm_3pwm.c"
    cp source/drivers/inverter_6pwm_mcpwm.c "$d/esp_foc_inverter_mcpwm_6pwm.c"
    cp source/drivers/current_sensor_adc.c "$d/esp_foc_inverter_adc.c"
    cp source/drivers/esp_foc_inverter_mcpwm.c "$d/esp_foc_inverter_bundle.c"
    cp source/drivers/espFoC/inverter_3pwm_mcpwm.h "$d/esp_foc_inverter_mcpwm_3pwm.h"
    cp source/drivers/espFoC/inverter_6pwm_mcpwm.h "$d/esp_foc_inverter_mcpwm_6pwm.h"
    cp source/drivers/isensor_adc_internal.h "$d/esp_foc_inverter_internal.h"
    cp source/drivers/espFoC/isensor_adc_private.h "$d/esp_foc_isensor_adc_private.h"
    cp source/drivers/inverter_mcpwm_etm.h "$d/esp_foc_inverter_mcpwm_etm.h"
    cp source/drivers/inverter_mcpwm_etm.c "$d/esp_foc_inverter_mcpwm_etm.c"
    cp source/drivers/isensor_adc_etm.c "$d/esp_foc_inverter_adc_etm.c"
    if [[ "$t" == "esp32" ]]; then
        cp source/drivers/isensor_adc_dma_esp32.c "$d/esp_foc_inverter_dma.c"
    else
        cp source/drivers/isensor_adc_dma_gdma.c "$d/esp_foc_inverter_dma.c"
    fi
}

for t in "${TARGETS[@]}"; do
    setup_target "$t"
done

fix_includes() {
    local dir="$1"
    find "$dir" -type f \( -name '*.c' -o -name '*.h' \) -print0 | while IFS= read -r -d '' f; do
        sed -i \
            -e 's|"espFoC/inverter_3pwm_mcpwm.h"|"esp_foc_inverter_mcpwm_3pwm.h"|g' \
            -e 's|"espFoC/inverter_6pwm_mcpwm.h"|"esp_foc_inverter_mcpwm_6pwm.h"|g' \
            -e 's|"espFoC/isensor_adc_private.h"|"esp_foc_isensor_adc_private.h"|g' \
            -e 's|"espFoC/driver_q16_local.h"|"driver_q16_local.h"|g' \
            -e 's|"isensor_adc_internal.h"|"esp_foc_inverter_internal.h"|g' \
            -e 's|"inverter_mcpwm_etm.h"|"esp_foc_inverter_mcpwm_etm.h"|g' \
            -e 's|"espFoC/esp_foc_inverter_mcpwm.h"|"espFoC/drivers/esp_foc_inverter_mcpwm.h"|g' \
            -e 's|"espFoC/esp_foc_encoder_as5600.h"|"esp_foc_encoder_as5600.h"|g' \
            -e 's|"espFoC/esp_foc_encoder_as5048.h"|"esp_foc_encoder_as5048.h"|g' \
            -e 's|"espFoC/esp_foc_encoder_pcnt.h"|"esp_foc_encoder_pcnt.h"|g' \
            -e 's|"espFoC/esp_foc_encoder_simu.h"|"esp_foc_encoder_simu.h"|g' \
            "$f"
    done
}

for t in "${TARGETS[@]}"; do
    fix_includes "source/drivers/inverter/${t}"
done
for enc in "${ENCODERS[@]}"; do
    fix_includes "source/drivers/encoder/${enc}"
done

# bundle: local pwm headers
for t in "${TARGETS[@]}"; do
    f="source/drivers/inverter/${t}/esp_foc_inverter_bundle.c"
    sed -i \
        -e 's|espFoC/inverter_6pwm_mcpwm.h|esp_foc_inverter_mcpwm_6pwm.h|g' \
        -e 's|espFoC/inverter_3pwm_mcpwm.h|esp_foc_inverter_mcpwm_3pwm.h|g' \
        "$f"
done

# internal header include path
for t in "${TARGETS[@]}"; do
    sed -i 's|"espFoC/isensor_adc_private.h"|"esp_foc_isensor_adc_private.h"|g' \
        "source/drivers/inverter/${t}/esp_foc_inverter_internal.h"
done

git rm -f \
    source/drivers/inverter_3pwm_mcpwm.c \
    source/drivers/inverter_6pwm_mcpwm.c \
    source/drivers/current_sensor_adc.c \
    source/drivers/esp_foc_inverter_mcpwm.c \
    source/drivers/isensor_adc_dma_esp32.c \
    source/drivers/isensor_adc_dma_gdma.c \
    source/drivers/isensor_adc_etm.c \
    source/drivers/inverter_mcpwm_etm.c \
    source/drivers/inverter_mcpwm_etm.h \
    source/drivers/isensor_adc_internal.h \
    source/drivers/espFoC/inverter_3pwm_mcpwm.h \
    source/drivers/espFoC/inverter_6pwm_mcpwm.h \
    source/drivers/espFoC/isensor_adc_private.h \
    2>/dev/null || rm -f \
    source/drivers/inverter_3pwm_mcpwm.c \
    source/drivers/inverter_6pwm_mcpwm.c \
    source/drivers/current_sensor_adc.c \
    source/drivers/esp_foc_inverter_mcpwm.c \
    source/drivers/isensor_adc_dma_esp32.c \
    source/drivers/isensor_adc_dma_gdma.c \
    source/drivers/isensor_adc_etm.c \
    source/drivers/inverter_mcpwm_etm.c \
    source/drivers/inverter_mcpwm_etm.h \
    source/drivers/isensor_adc_internal.h \
    source/drivers/espFoC/inverter_3pwm_mcpwm.h \
    source/drivers/espFoC/inverter_6pwm_mcpwm.h \
    source/drivers/espFoC/isensor_adc_private.h

rmdir source/drivers/espFoC 2>/dev/null || true

echo "Driver reorg complete."
