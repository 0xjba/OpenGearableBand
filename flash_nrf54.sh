#!/usr/bin/env bash
# Flash the XIAO nRF54LM20A over its built-in USB (SAMD11 CMSIS-DAP).
#
# WHY probe-rs (not west flash / openocd / pyocd):
#   The nRF54LM20A ships APPROTECT-LOCKED from the factory. The first flash must
#   recover (mass-erase) it. Mainline OpenOCD 0.12 and the bundled pyOCD do NOT
#   complete the nRF54L recover over this CMSIS-DAP probe (ERASEALL times out).
#   probe-rs (>= 0.32) has correct nRF54LM20A support and unlocks with
#   --allow-erase-all. No external debugger required.
#
# One-time tool install (already done on this machine, kept for reproducibility):
#   curl -fsSL https://github.com/probe-rs/probe-rs/releases/latest/download/\
#     probe-rs-tools-aarch64-apple-darwin.tar.xz | tar xJ -C /tmp
#   cp /tmp/probe-rs-tools-*/probe-rs ~/.cargo/bin/    # (on PATH)
#
# Usage:
#   ./flash_nrf54.sh                       # flash build_nrf54/zephyr/zephyr.elf
#   ./flash_nrf54.sh path/to/other.elf
set -euo pipefail

CHIP="nRF54LM20A"
ELF="${1:-build_nrf54/zephyr/zephyr.elf}"

if ! command -v probe-rs >/dev/null 2>&1; then
    echo "error: probe-rs not found on PATH (see install note in this script)" >&2
    exit 1
fi
if [[ ! -f "${ELF}" ]]; then
    echo "error: ${ELF} not found -- run ./build_nrf54.sh first" >&2
    exit 1
fi

# --allow-erase-all: permit the one-time APPROTECT recover (mass erase).
# --reset: run the firmware after programming.
exec probe-rs download --chip "${CHIP}" --allow-erase-all --reset "${ELF}"
