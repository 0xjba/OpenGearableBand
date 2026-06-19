cmake_minimum_required(VERSION 3.20.0)

# Find Zephyr. This also loads the Zephyr build system.
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(wearable_hr_monitor)

target_sources(app PRIVATE
    src/main.cpp
    src/WearableDSP.cpp
    src/power_ctrl.cpp
    src/gesture_mode.cpp
    src/gesture_poses.cpp
    src/orientation.cpp
    src/mic_vad.cpp
    src/mic_vad_rms.cpp
    src/lc3_codec.c
    src/ble_audio.cpp
    src/audio_stream.cpp
    src/audio_out.cpp
    src/sd_card.cpp
)
