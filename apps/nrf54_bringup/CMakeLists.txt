cmake_minimum_required(VERSION 3.20.0)

# M1 bring-up app for the XIAO nRF54LM20A OLED variant.
# Kept separate from the main nRF52840 firmware (src/main.cpp) until M4, when
# the two boards converge behind Kconfig guards.

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf54_bringup)

target_sources(app PRIVATE
    src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/display_oled.cpp
)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
