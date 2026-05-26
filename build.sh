#!/usr/bin/env bash
# Wrapper around `west build` that:
#   * pins the NCS toolchain on PATH (so `west` and the Zephyr SDK resolve)
#   * disables sysbuild, so artifacts land at build/zephyr/* instead of the
#     nested build/<app>/zephyr/* sysbuild layout.
#
# Usage:
#   ./build.sh           # incremental build
#   ./build.sh -p        # pristine rebuild
#   ./build.sh flash     # rebuild + drag-drop hint
set -euo pipefail

NCS_ROOT="/opt/nordic/ncs"
TOOLCHAIN="${NCS_ROOT}/toolchains/185bb0e3b6"
BOARD="xiao_ble/nrf52840/sense"

export PATH="${TOOLCHAIN}/bin:${PATH}"
export ZEPHYR_BASE="${NCS_ROOT}/zephyr"
export ZEPHYR_TOOLCHAIN_VARIANT="zephyr"
export ZEPHYR_SDK_INSTALL_DIR="${TOOLCHAIN}/opt/zephyr-sdk"

PRISTINE=""
if [[ "${1:-}" == "-p" || "${1:-}" == "--pristine" ]]; then
    PRISTINE="-p always"
    shift
fi

west build ${PRISTINE} --no-sysbuild -b "${BOARD}" "$@"

UF2="build/zephyr/zephyr.uf2"
if [[ -f "${UF2}" ]]; then
    SIZE=$(stat -f%z "${UF2}")
    echo
    echo "==> Flashable artifact: ${UF2} (${SIZE} bytes)"
    echo "    Double-tap RESET on the Xiao, then copy to the XIAO-SENSE volume:"
    echo "      cp ${UF2} /Volumes/XIAO-SENSE/"
fi
