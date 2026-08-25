cmake_minimum_required(VERSION 3.20.0)

# M2 app for the XIAO nRF54LM20A OLED variant: BLE -> display-text.
# Reuses src/display_oled + src/ble_display from the shared source tree.

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf54_ble)

target_sources(app PRIVATE
    src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/display_oled.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/ble_display.cpp
)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
