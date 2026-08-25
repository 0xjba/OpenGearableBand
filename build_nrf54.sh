#!/usr/bin/env bash
# Build wrapper for the XIAO nRF54LM20A OLED variant.
#   * pins the NCS toolchain on PATH
#   * points at the out-of-tree board definition (boards/seeed/xiao_nrf54lm20a)
#   * builds the M1 bring-up app by default (apps/nrf54_bringup)
#
# Unlike the nRF52840 (build.sh), this board has NO UF2 mass-storage bootloader.
# Flash over the built-in USB (SAMD11 CMSIS-DAP) with probe-rs -- the chip ships
# APPROTECT-locked and only probe-rs reliably recovers+flashes it here:
#     ./flash_nrf54.sh          # wraps: probe-rs download --chip nRF54LM20A --allow-erase-all --reset
# (west flash/openocd/pyocd do NOT complete the nRF54L recover on this probe.)
#
# Usage:
#   ./build_nrf54.sh                # incremental build of the M1 bring-up app
#   ./build_nrf54.sh -p             # pristine rebuild
#   ./build_nrf54.sh <app_dir>      # build a different app dir
set -euo pipefail

NCS_ROOT="/opt/nordic/ncs"
TOOLCHAIN="${NCS_ROOT}/toolchains/185bb0e3b6"
BOARD="xiao_nrf54lm20a/nrf54lm20a/cpuapp"
BUILD_DIR="build_nrf54"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export PATH="${TOOLCHAIN}/bin:${PATH}"
export ZEPHYR_BASE="${NCS_ROOT}/zephyr"
export ZEPHYR_TOOLCHAIN_VARIANT="zephyr"
export ZEPHYR_SDK_INSTALL_DIR="${TOOLCHAIN}/opt/zephyr-sdk"

PRISTINE=""
if [[ "${1:-}" == "-p" || "${1:-}" == "--pristine" ]]; then
    PRISTINE="-p always"
    shift
fi

APP_DIR="${1:-${REPO_ROOT}/apps/nrf54_bringup}"

west build ${PRISTINE} -d "${BUILD_DIR}" --no-sysbuild -b "${BOARD}" \
    "${APP_DIR}" -- -DBOARD_ROOT="${REPO_ROOT}"

ELF="${BUILD_DIR}/zephyr/zephyr.elf"
if [[ -f "${ELF}" ]]; then
    echo
    echo "==> Built: ${ELF}"
    echo "    Flash over built-in USB (CMSIS-DAP):"
    echo "      ./flash_nrf54.sh"
fi
