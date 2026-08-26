cmake_minimum_required(VERSION 3.20.0)

# M3a bring-up app: nPM1300 LDO + LSM6DS3TR-C IMU (power + gravity).

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf54_imu)

target_sources(app PRIVATE src/main.cpp)
