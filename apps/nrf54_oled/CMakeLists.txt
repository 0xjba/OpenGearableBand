cmake_minimum_required(VERSION 3.20.0)

# M4: converged OLED-variant firmware. Assembles the input side (gesture + pose
# + mic + LC3 uplink) with OLED text output. NO audio-downlink / HR (the
# speaker_stubs replace the downlink+playback seam; HR modules aren't compiled).

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf54_oled)

set(SHARED ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_sources(app PRIVATE
    src/main.cpp
    src/speaker_stubs.cpp
    # output
    ${SHARED}/display_oled.cpp
    ${SHARED}/ble_display.cpp
    # gesture + orientation
    ${SHARED}/orientation.cpp
    ${SHARED}/gesture_poses.cpp
    ${SHARED}/gesture_mode.cpp
    # mic + LC3 uplink
    ${SHARED}/mic_vad.cpp
    ${SHARED}/mic_vad_rms.cpp
    ${SHARED}/audio_stream.cpp
    ${SHARED}/lc3_codec.c
    ${SHARED}/ble_audio.cpp
)
target_include_directories(app PRIVATE ${SHARED})
