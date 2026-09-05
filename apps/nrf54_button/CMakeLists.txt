cmake_minimum_required(VERSION 3.20.0)

# Minimal BLE + button test app for the XIAO nRF54LM20A Sense.
# Deliberately self-contained: it pulls in NO shared sources from ../../src, so
# it boots with the OLED, IMU, mic and PMIC all physically absent.

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf54_button)

target_sources(app PRIVATE
    src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/resilience.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/battery.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/led_status.cpp
)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
