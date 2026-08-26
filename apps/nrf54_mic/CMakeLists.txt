cmake_minimum_required(VERSION 3.20.0)

# M3c.1 bring-up: PDM mic + mic_vad (VAD), audio_stream stubbed.

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf54_mic)

target_sources(app PRIVATE
    src/main.cpp
    src/audio_stream_stub.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/mic_vad.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/mic_vad_rms.cpp
)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
