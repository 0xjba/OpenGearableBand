# Copyright (c) 2026 gestureband
# SPDX-License-Identifier: Apache-2.0

# The XIAO nRF54LM20A flashes via the on-board ATSAMD11 SWD bridge (CMSIS-DAP).
# Default runner is openocd (over CMSIS-DAP), matching the upstream board and the
# sibling xiao_nrf54l15. The openocd.cfg selects the cmsis-dap interface and a
# generic cortex_m target; `nrf54l-load` writes the RRAM enable reg then loads
# the image directly (no Nordic-specific flash driver needed). J-Link is kept as
# a fallback for anyone using an external SEGGER probe.
if(CONFIG_SOC_NRF54LM20A_ENGA_CPUAPP)
  board_runner_args(openocd "--cmd-load=nrf54l-load" -c "targets nrf54l.cpu")
  board_runner_args(jlink "--device=nRF54LM20A_M33" "--speed=4000")
endif()

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/nrfutil.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
