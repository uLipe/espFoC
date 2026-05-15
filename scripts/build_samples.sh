#!/usr/bin/env bash
# Build espFoC example projects (mirrors CI build job).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

if ! command -v idf.py >/dev/null 2>&1; then
    if [[ -z "${IDF_PATH:-}" ]]; then
        echo "IDF_PATH not set; source esp-idf/export.sh first" >&2
        exit 1
    fi
    # shellcheck disable=SC1090
    . "${IDF_PATH}/export.sh"
fi

build_one() {
    local dir="$1"
    shift
    echo "=== $dir $* ==="
    pushd "$dir" >/dev/null
    # shellcheck disable=SC2068
    idf.py "$@"
    popd >/dev/null
}

build_one examples/axis_simple set-target esp32s3 build
build_one examples/axis_simple set-target esp32p4 build

build_one examples/tuner_demo set-target esp32 build

build_one examples/tuner_studio_target set-target esp32s3 build
rm -rf examples/tuner_studio_target/build examples/tuner_studio_target/sdkconfig
build_one examples/tuner_studio_target set-target esp32p4 build
rm -rf examples/tuner_studio_target/build examples/tuner_studio_target/sdkconfig
build_one examples/tuner_studio_target set-target esp32 build

build_one examples/test_drivers/test_current_sense set-target esp32s3 build
build_one examples/test_drivers/test_encoder set-target esp32s3 build
build_one examples/test_drivers/test_inverter set-target esp32s3 build
build_one examples/test_drivers/test_voltage_path set-target esp32s3 build

echo "All sample builds OK."
