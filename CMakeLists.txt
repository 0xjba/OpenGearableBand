cmake_minimum_required(VERSION 3.20.0)

# Find Zephyr. This also loads the Zephyr build system.
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(wearable_hr_monitor)

target_sources(app PRIVATE
    src/main.cpp
    src/WearableDSP.cpp
    src/power_ctrl.cpp
    src/gesture_mode.cpp
)
