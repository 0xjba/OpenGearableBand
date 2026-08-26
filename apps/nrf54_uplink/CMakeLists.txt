cmake_minimum_required(VERSION 3.20.0)

# M3c.2: mic -> LC3 -> BLE uplink. Real audio_stream/lc3_codec/ble_audio/mic_vad;
# gesture_mode + audio_downlink stubbed (uplink-only, force dictation via `d`).

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf54_uplink)

set(SHARED ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_sources(app PRIVATE
    src/main.cpp
    src/gesture_mode_stub.cpp
    src/audio_downlink_stub.cpp
    ${SHARED}/mic_vad.cpp
    ${SHARED}/mic_vad_rms.cpp
    ${SHARED}/audio_stream.cpp
    ${SHARED}/lc3_codec.c
    ${SHARED}/ble_audio.cpp
)
target_include_directories(app PRIVATE ${SHARED})
