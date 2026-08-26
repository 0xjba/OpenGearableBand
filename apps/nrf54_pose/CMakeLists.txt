cmake_minimum_required(VERSION 3.20.0)

# M3b bring-up: orientation filter + pose detection on the nRF54 IMU.
# Reuses the pure-algorithm modules from the shared src/ tree.

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf54_pose)

target_sources(app PRIVATE
    src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/orientation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/gesture_poses.cpp
)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
